/*
 * Copyright 2013-2019 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#define Error(fmt, ...)   log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   log_h->debug(fmt, ##__VA_ARGS__)

#include <inttypes.h> // for printing uint64_t
#include <signal.h>
#include <stdint.h>
#include <stdlib.h>

#include "srsue/hdr/mac/proc_ra.h"
#include "srsue/hdr/mac/mac.h"
#include "srsue/hdr/mac/mux.h"

/* Random access procedure as specified in Section 5.1 of 36.321 */


namespace srsue {

// Table 7.2-1. Backoff Parameter values
uint32_t backoff_table[16] = {0, 10, 20, 30, 40, 60, 80, 120, 160, 240, 320, 480, 960, 960, 960, 960};

// Table 7.6-1: DELTA_PREAMBLE values.
int delta_preamble_db_table[5] = {0, 0, -3, -3, 8};

void ra_proc::init(phy_interface_mac*            phy_h_,
                   rrc_interface_mac*            rrc_,
                   srslte::log*                  log_h_,
                   mac_interface_rrc::ue_rnti_t* rntis_,
                   srslte::timers::timer*        time_alignment_timer_,
                   srslte::timers::timer*        contention_resolution_timer_,
                   mux*                          mux_unit_)
{
  phy_h     = phy_h_; 
  log_h     = log_h_; 
  rntis     = rntis_;
  mux_unit  = mux_unit_;
  rrc       = rrc_;

  time_alignment_timer        = time_alignment_timer_;
  contention_resolution_timer = contention_resolution_timer_;
  
  srslte_softbuffer_rx_init(&softbuffer_rar, 10);

  pthread_mutex_init(&mutex, NULL);

  reset();
}

ra_proc::~ra_proc() {
  pthread_mutex_destroy(&mutex);
  srslte_softbuffer_rx_free(&softbuffer_rar);
}

void ra_proc::reset() {
  state = IDLE;
  msg3_transmitted = false;
  started_by_pdcch = false;
  contention_resolution_timer->stop();
  contention_resolution_timer->reset();
}

void ra_proc::start_pcap(srslte::mac_pcap* pcap_)
{
  pcap = pcap_; 
}

void ra_proc::set_config(srsue::mac_interface_rrc::rach_cfg_t& rach_cfg)
{
  pthread_mutex_lock(&mutex);
  new_cfg = rach_cfg;
  pthread_mutex_unlock(&mutex);
}

void ra_proc::read_params()
{
  pthread_mutex_lock(&mutex);
  rach_cfg = new_cfg;
  pthread_mutex_unlock(&mutex);

  // Read initialization parameters   
  if (noncontention_enabled) {
    preambleIndex             = next_preamble_idx;
    maskIndex                 = next_prach_mask;
    noncontention_enabled     = false;
  } else {
    preambleIndex             = 0; // pass when called from higher layers for non-contention based RA
    maskIndex                 = 0; // same
  }

  if (rach_cfg.nof_groupA_preambles == 0) {
    rach_cfg.nof_groupA_preambles = rach_cfg.nof_preambles;
  }

  phy_interface_mac::prach_info_t prach_info = phy_h->prach_get_info();
  delta_preamble_db                          = delta_preamble_db_table[prach_info.preamble_format % 5];

  if (rach_cfg.contentionResolutionTimer > 0) {
    contention_resolution_timer->set(this, rach_cfg.contentionResolutionTimer);
  }
}

bool ra_proc::is_contention_resolution() {
  return state == CONTENTION_RESOLUTION;
}

const char* state_str[12] = {"Idle",
                            "RA:    INIT:   ",
                            "RA:    Select: ",
                            "RA:    TX:     ",
                            "RA:    PDCCH:  ",
                            "RA:    Rx:     ",
                            "RA:    RxErr:  ",
                            "RA:    Backof: ",
                            "RA:    ConRes: ",
                            "RA:    Done:   ",
                            "RA:    Done:   ",
                            "RA:    Error:  "};

                           
#define rError(fmt, ...) Error("%s" fmt, state_str[state], ##__VA_ARGS__)                         
#define rInfo(fmt, ...)  Info("%s" fmt, state_str[state], ##__VA_ARGS__)                         
#define rDebug(fmt, ...) Debug("%s" fmt, state_str[state], ##__VA_ARGS__)

                            
// Process Timing Advance Command as defined in Section 5.2
void ra_proc::process_timeadv_cmd(uint32_t ta) {
  if (preambleIndex == 0) {
    // Preamble not selected by UE MAC 
    phy_h->set_timeadv_rar(ta);
    // Only if timer is running reset the timer
    if (time_alignment_timer->is_running()) {
      time_alignment_timer->reset();
      time_alignment_timer->run();
    }
    Debug("Applying RAR TA CMD %d\n", ta);
  } else {
    // Preamble selected by UE MAC 
    if (!time_alignment_timer->is_running()) {
      phy_h->set_timeadv_rar(ta);
      time_alignment_timer->run();
      Debug("Applying RAR TA CMD %d\n", ta);
    } else {
      // Ignore TA CMD
      Warning("Ignoring RAR TA CMD because timeAlignmentTimer still running\n");
    }
  }
}

void ra_proc::step_initialization() {
  read_params();
  pdcch_to_crnti_received     = PDCCH_CRNTI_NOT_RECEIVED;
  transmitted_contention_id = 0; 
  preambleTransmissionCounter = 1; 
  first_rar_received = true; 
  mux_unit->msg3_flush();
  msg3_flushed = false; 
  backoff_param_ms = 0; 

  // FIXME: This is because RA in Connected state not working in amarisoft
  transmitted_crnti = rntis->crnti;
  if(transmitted_crnti) {
    state = RESPONSE_ERROR;
  }
  
  // Instruct phy to configure PRACH
  phy_h->configure_prach_params();
  state = RESOURCE_SELECTION; 
}

void ra_proc::step_resource_selection()
{
  ra_group_t sel_group;

  uint32_t nof_groupB_preambles = 0;
  if (rach_cfg.nof_groupA_preambles > 0) {
    nof_groupB_preambles = rach_cfg.nof_preambles - rach_cfg.nof_groupA_preambles;
  }

  if (preambleIndex > 0) {
    // Preamble is chosen by Higher layers (ie Network)
    sel_maskIndex = maskIndex;
    sel_preamble = (uint32_t) preambleIndex;
  } else {
    // Preamble is chosen by MAC UE
    if (!msg3_transmitted) {
      if (nof_groupB_preambles &&
          new_ra_msg_len > rach_cfg.messageSizeGroupA) { // Check also pathloss (Pcmax,deltaPreamble and powerOffset)
        sel_group = RA_GROUP_B; 
      } else {
        sel_group = RA_GROUP_A; 
      }
      last_msg3_group = sel_group;
    } else {
      sel_group = last_msg3_group;
    }
    if (sel_group == RA_GROUP_A) {
      if (rach_cfg.nof_groupA_preambles) {
        // randomly choose preamble from [0 nof_groupA_preambles)
        sel_preamble = rand() % rach_cfg.nof_groupA_preambles;
      } else {
        rError("Selected group preamble A but nof_groupA_preambles=0\n");
        state = RA_PROBLEM;
        return; 
      }
    } else {
      if (nof_groupB_preambles) {
        // randomly choose preamble from [nof_groupA_preambles nof_groupB_preambles)
        sel_preamble = rach_cfg.nof_groupA_preambles + rand() % nof_groupB_preambles;
      } else {
        rError("Selected group preamble B but nof_groupA_preambles=0\n");
        state = RA_PROBLEM;
        return; 
      }
    }
    sel_maskIndex = 0;           
  }

  rDebug("Selected preambleIndex=%d maskIndex=%d GroupA=%d, GroupB=%d\n",
         sel_preamble,
         sel_maskIndex,
         rach_cfg.nof_groupA_preambles,
         nof_groupB_preambles);
  state = PREAMBLE_TRANSMISSION;
}

void ra_proc::step_preamble_transmission() {

  received_target_power_dbm = rach_cfg.iniReceivedTargetPower + delta_preamble_db +
                              (preambleTransmissionCounter - 1) * rach_cfg.powerRampingStep;

  rar_received = false;
  phy_h->prach_send(sel_preamble, sel_maskIndex - 1, received_target_power_dbm);
  state = PDCCH_SETUP;
}

bool ra_proc::is_rar_window(int* rar_window_start, int* rar_window_length)
{
  if (state == RESPONSE_RECEPTION) {
    if (rar_window_length) {
      *rar_window_length = rach_cfg.responseWindowSize;
    }
    if (rar_window_start) {
      *rar_window_start = rar_window_st;
    }
    return true;
  } else {
    if (rar_window_length) {
      *rar_window_length = -1;
    }
    return false;
  }
}

void ra_proc::step_pdcch_setup()
{

  phy_interface_mac::prach_info_t info = phy_h->prach_get_info();
  if (info.is_transmitted) {
    ra_rnti = 1 + info.tti_ra % 10 + info.f_id;
    rInfo("seq=%d, ra-rnti=0x%x, ra-tti=%d, f_id=%d\n", sel_preamble, ra_rnti, info.tti_ra, info.f_id);
    log_h->console("Random Access Transmission: seq=%d, ra-rnti=0x%x\n", sel_preamble, ra_rnti);
    rar_window_st   = info.tti_ra + 3;
    rntis->rar_rnti = ra_rnti;
    state           = RESPONSE_RECEPTION;
  }
}

void ra_proc::new_grant_dl(mac_interface_phy::mac_grant_dl_t grant, mac_interface_phy::tb_action_dl_t* action)
{
  bzero(action, sizeof(mac_interface_phy::tb_action_dl_t));

  if (grant.tb[0].tbs < MAX_RAR_PDU_LEN) {
    rDebug("DL dci found RA-RNTI=%d\n", ra_rnti);
    action->tb[0].enabled       = true;
    action->tb[0].payload       = rar_pdu_buffer;
    action->tb[0].rv            = grant.tb[0].rv;
    action->tb[0].softbuffer.rx = &softbuffer_rar;
    rar_grant_nbytes            = grant.tb[0].tbs;
    if (action->tb[0].rv == 0) {
      srslte_softbuffer_rx_reset(&softbuffer_rar);
    }
  } else {
    rError("Received RAR dci exceeds buffer length (%d>%d)\n", grant.tb[0].tbs, MAX_RAR_PDU_LEN);
    state = RESPONSE_ERROR;
  }
}

void ra_proc::tb_decoded_ok() {
  if (pcap) {
    pcap->write_dl_ranti(rar_pdu_buffer, rar_grant_nbytes, ra_rnti, true, 0);
  }
  
  rDebug("RAR decoded successfully TBS=%d\n", rar_grant_nbytes);
  
  rar_pdu_msg.init_rx(rar_grant_nbytes);
  rar_pdu_msg.parse_packet(rar_pdu_buffer);
  // Set Backoff parameter
  if (rar_pdu_msg.has_backoff()) {
    backoff_param_ms = backoff_table[rar_pdu_msg.get_backoff()%16];
  } else {
    backoff_param_ms = 0; 
  }
  
  current_ta = 0; 
  
  while(rar_pdu_msg.next()) {
    if (rar_pdu_msg.get()->get_rapid() == sel_preamble) {

      rar_received = true; 
      process_timeadv_cmd(rar_pdu_msg.get()->get_ta_cmd());
      
      // FIXME: Indicate received target power
      //phy_h->set_target_power_rar(iniReceivedTargetPower, (preambleTransmissionCounter-1)*powerRampingStep);

      uint8_t grant[srslte::rar_subh::RAR_GRANT_LEN];
      rar_pdu_msg.get()->get_sched_grant(grant);

      rntis->rar_rnti = 0;
      phy_h->set_rar_grant(grant, rar_pdu_msg.get()->get_temp_crnti());

      current_ta = rar_pdu_msg.get()->get_ta_cmd();

      rInfo("RAPID=%d, TA=%d, T-CRNTI=0x%x\n",
            sel_preamble,
            rar_pdu_msg.get()->get_ta_cmd(),
            rar_pdu_msg.get()->get_temp_crnti());

      if (preambleIndex > 0) {
        // Preamble selected by Network
        state = COMPLETION; 
      } else {
        // Preamble selected by UE MAC
        mux_unit->msg3_prepare();
        rntis->temp_rnti = rar_pdu_msg.get()->get_temp_crnti();

        if (first_rar_received) {
          first_rar_received = false;

          // Save transmitted C-RNTI (if any)
          transmitted_crnti = rntis->crnti;

          // If we have a C-RNTI, tell Mux unit to append C-RNTI CE if no CCCH SDU transmission
          if (transmitted_crnti) {
            rInfo("Appending C-RNTI MAC CE 0x%x in next transmission\n", transmitted_crnti);
            mux_unit->append_crnti_ce_next_tx(transmitted_crnti);
            rntis->crnti = transmitted_crnti;
          }
        }
        rDebug("Going to Contention Resolution state\n");
        state = CONTENTION_RESOLUTION;

        // Start contention resolution timer
        rInfo("Starting ContentionResolutionTimer=%d ms\n", contention_resolution_timer->get_timeout());
        contention_resolution_timer->reset();
        contention_resolution_timer->run();
      }
    } else {
      rInfo("Found RAR for preamble %d\n", rar_pdu_msg.get()->get_rapid());
    }
  }
}

void ra_proc::step_response_reception(uint32_t tti)
{
  // do nothing. Processing done in tb_decoded_ok()
  phy_interface_mac::prach_info_t prach_info = phy_h->prach_get_info();
  if (prach_info.is_transmitted && !rar_received) {
    uint32_t interval = srslte_tti_interval(tti, prach_info.tti_ra + 3 + rach_cfg.responseWindowSize);
    if (interval > 1 && interval < 100) {
      Error("RA response not received within the response window\n");
      state = RESPONSE_ERROR;
    }
  }
}

void ra_proc::step_response_error(uint32_t tti)
{
  preambleTransmissionCounter++;
  if (preambleTransmissionCounter >= rach_cfg.preambleTransMax + 1) {
    rError("Maximum number of transmissions reached (%d)\n", rach_cfg.preambleTransMax);
    rrc->ra_problem();
    state = RA_PROBLEM;
    if (ra_is_ho) {
      rrc->ho_ra_completed(false);
    }
  } else {
    backoff_interval_start = tti;
    if (backoff_param_ms) {
      backoff_inteval = rand() % backoff_param_ms;
    } else {
      backoff_inteval = 0; 
    }
    if (backoff_inteval) {
      rDebug("Backoff wait interval %d\n", backoff_inteval);
      state = BACKOFF_WAIT;
    } else {
      rDebug("Transmitting inmediatly (%d/%d)\n", preambleTransmissionCounter, rach_cfg.preambleTransMax);
      state = RESOURCE_SELECTION;
    }
  }
}

void ra_proc::step_backoff_wait(uint32_t tti)
{
  if (srslte_tti_interval(tti, backoff_interval_start) >= backoff_inteval) {
    state = RESOURCE_SELECTION; 
  }
}

// Random Access initiated by RRC by the transmission of CCCH SDU      
bool ra_proc::contention_resolution_id_received(uint64_t rx_contention_id) {
  bool uecri_successful = false; 
  
  rDebug("MAC PDU Contains Contention Resolution ID CE\n");
  
  // MAC PDU successfully decoded and contains MAC CE contention Id
  contention_resolution_timer->stop();
  
  if (transmitted_contention_id == rx_contention_id) 
  {    
    // UE Contention Resolution ID included in MAC CE matches the CCCH SDU transmitted in Msg3
    uecri_successful = true;
    state = COMPLETION;                           
  } else {
    rInfo("Transmitted UE Contention Id differs from received Contention ID (0x%" PRIu64 " != 0x%" PRIu64 ")\n",
          transmitted_contention_id, rx_contention_id);
    // Discard MAC PDU
    uecri_successful = false;

    // Contention Resolution not successfully is like RAR not successful 
    // FIXME: Need to flush Msg3 HARQ buffer. Why? 
    state = RESPONSE_ERROR;
  }

  return uecri_successful;
}

void ra_proc::step_contention_resolution() {
  // If Msg3 has been sent
  if (mux_unit->msg3_is_transmitted()) 
  {
    msg3_transmitted = true;
    if (transmitted_crnti) {
      // Random Access with transmission of MAC C-RNTI CE
      if ((!started_by_pdcch && pdcch_to_crnti_received == PDCCH_CRNTI_UL_GRANT) || 
            (started_by_pdcch && pdcch_to_crnti_received != PDCCH_CRNTI_NOT_RECEIVED))
      {
        rDebug("PDCCH for C-RNTI received\n");
        contention_resolution_timer->stop();
        state = COMPLETION;
      }            
      pdcch_to_crnti_received = PDCCH_CRNTI_NOT_RECEIVED;      
    } else {
      // RA with transmission of CCCH SDU is resolved in contention_resolution_id_received() callback function
      if (!transmitted_contention_id) {
        // Save transmitted UE contention id, as defined by higher layers 
        transmitted_contention_id = rntis->contention_id;
        rntis->contention_id      = 0; 
      }
    }
  } else {
    rDebug("Msg3 not yet transmitted\n");
  }
}

void ra_proc::step_completition()
{

  // Start looking for PDCCH CRNTI
  if (!transmitted_crnti) {
    rntis->crnti = rntis->temp_rnti;
  }
  rntis->temp_rnti = 0;

  log_h->console("Random Access Complete.     c-rnti=0x%x, ta=%d\n", rntis->crnti, current_ta);
  rInfo("Random Access Complete.     c-rnti=0x%x, ta=%d\n",          rntis->crnti, current_ta);

  if (!msg3_flushed) {
    mux_unit->msg3_flush();
    msg3_flushed = true; 
  }

  phy_h->set_crnti(rntis->crnti);

  msg3_transmitted = false;
  state = COMPLETION_DONE;
  if (ra_is_ho) {
    rrc->ho_ra_completed(true);
  }
}

void ra_proc::step(uint32_t tti_)
{
  switch(state) {
    case IDLE: 
      break;
    case INITIALIZATION:
      step_initialization();
      break;
    case RESOURCE_SELECTION:
      step_resource_selection();
      break;
    case PREAMBLE_TRANSMISSION:
      step_preamble_transmission();
      break;
    case PDCCH_SETUP:
      step_pdcch_setup();
      break;
    case RESPONSE_RECEPTION:
      step_response_reception(tti_);
      break;
    case RESPONSE_ERROR:
      step_response_error(tti_);
      break;
    case BACKOFF_WAIT:
      step_backoff_wait(tti_);
      break;
    case CONTENTION_RESOLUTION:
      step_contention_resolution();
    break;
    case COMPLETION:
      step_completition();
    case COMPLETION_DONE:
    case RA_PROBLEM:
    break;
  }
}

void ra_proc::start_noncont(uint32_t preamble_index, uint32_t prach_mask) {
  next_preamble_idx = preamble_index;
  next_prach_mask   = prach_mask;
  noncontention_enabled = true;
  start_mac_order(56, true);
}

void ra_proc::start_mac_order(uint32_t msg_len_bits, bool is_ho)
{
  if (state == IDLE || state == COMPLETION_DONE || state == RA_PROBLEM) {
    ra_is_ho = is_ho;
    started_by_pdcch = false;
    new_ra_msg_len = msg_len_bits; 
    state = INITIALIZATION;    
    rInfo("Starting PRACH by MAC order\n");
  }
}

void ra_proc::start_pdcch_order()
{
  if (state == IDLE || state == COMPLETION_DONE || state == RA_PROBLEM) {
    started_by_pdcch = true;
    state = INITIALIZATION;    
    rInfo("Starting PRACH by PDCCH order\n");
  }
}

// Contention Resolution Timer is expired (Section 5.1.5)
void ra_proc::timer_expired(uint32_t timer_id)
{
  rInfo("Contention Resolution Timer expired. Stopping PDCCH Search and going to Response Error\n");
  bzero(rntis, sizeof(mac_interface_rrc::ue_rnti_t));
  state = RESPONSE_ERROR; 
}

void ra_proc::pdcch_to_crnti(bool contains_uplink_grant) {
  rDebug("PDCCH to C-RNTI received %s UL dci\n", contains_uplink_grant ? "with" : "without");
  if (contains_uplink_grant) {
    pdcch_to_crnti_received = PDCCH_CRNTI_UL_GRANT;     
  } else if (pdcch_to_crnti_received == PDCCH_CRNTI_NOT_RECEIVED) {
    pdcch_to_crnti_received = PDCCH_CRNTI_DL_GRANT;         
  }
}

void ra_proc::harq_retx()
{
  contention_resolution_timer->reset();
}

void ra_proc::harq_max_retx()
{
  Warning("Contention Resolution is considered not successful. Stopping PDCCH Search and going to Response Error\n");
  bzero(rntis, sizeof(mac_interface_rrc::ue_rnti_t));
  state = RESPONSE_ERROR;
}
}

