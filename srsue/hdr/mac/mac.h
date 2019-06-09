/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsUE library.
 *
 * srsUE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsUE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#ifndef SRSUE_MAC_H
#define SRSUE_MAC_H

#include "demux.h"
#include "dl_harq.h"
#include "mac_metrics.h"
#include "mux.h"
#include "proc_bsr.h"
#include "proc_phr.h"
#include "proc_ra.h"
#include "proc_sr.h"
#include "srslte/asn1/rrc_asn1.h"
#include "srslte/common/log.h"
#include "srslte/common/mac_pcap.h"
#include "srslte/common/threads.h"
#include "srslte/common/timers.h"
#include "srslte/common/tti_sync_cv.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "ul_harq.h"

namespace srsue {
  
class mac
    :public mac_interface_phy
    ,public mac_interface_rrc
    ,public srslte::timer_callback
    ,public srslte::mac_interface_timers
    ,public periodic_thread
{
public:
  mac();
  bool init(phy_interface_mac *phy, rlc_interface_mac *rlc, rrc_interface_mac* rrc, srslte::log *log_h);
  void stop();

  void get_metrics(mac_metrics_t &m);

  /******** Interface from PHY (PHY -> MAC) ****************/ 
  /* see mac_interface.h for comments */
  void new_grant_ul(mac_grant_t grant, tb_action_ul_t *action);
  void new_grant_ul_ack(mac_grant_t grant, bool ack, tb_action_ul_t *action);
  void harq_recv(uint32_t tti, bool ack, tb_action_ul_t *action);
  void new_grant_dl(mac_grant_t grant, tb_action_dl_t *action);
  void new_mch_dl(srslte_ra_dl_grant_t phy_grant, tb_action_dl_t *action);
  void tb_decoded(bool ack, uint32_t tb_idx, srslte_rnti_type_t rnti_type, uint32_t harq_pid);
  void bch_decoded_ok(uint8_t *payload, uint32_t len);

  void pch_decoded_ok(uint32_t len);
  void mch_decoded_ok(uint32_t len);
  void process_mch_pdu(uint32_t len);

  void set_mbsfn_config(uint32_t nof_mbsfn_services);
  
  /******** Interface from RRC (RRC -> MAC) ****************/
  void bcch_start_rx(); 
  void bcch_start_rx(int si_window_start, int si_window_length);
  void pcch_start_rx(); 
  void clear_rntis();
  void setup_lcid(uint32_t lcid, uint32_t lcg, uint32_t priority, int PBR_x_tti, uint32_t BSD);
  void mch_start_rx(uint32_t lcid);
  void reconfiguration(); 
  void reset();
  void wait_uplink();

  /******** set/get MAC configuration  ****************/ 
  void set_config(mac_cfg_t *mac_cfg);
  void get_config(mac_cfg_t *mac_cfg);
  void set_config_main(asn1::rrc::mac_main_cfg_s* main_cfg);
  void set_config_rach(asn1::rrc::rach_cfg_common_s* rach_cfg, uint32_t prach_config_index);
  void set_config_sr(asn1::rrc::sched_request_cfg_c* sr_cfg);
  void set_contention_id(uint64_t uecri);

  void start_noncont_ho(uint32_t preamble_index, uint32_t prach_mask);
  void start_cont_ho();

  void get_rntis(ue_rnti_t *rntis);
  void set_ho_rnti(uint16_t crnti, uint16_t target_pci);

  void start_pcap(srslte::mac_pcap* pcap);

  // Timer callback interface
  void timer_expired(uint32_t timer_id); 

  uint32_t get_current_tti();

  // Interface for upper-layer timers
  srslte::timers::timer* timer_get(uint32_t timer_id);
  void                   timer_release_id(uint32_t timer_id);
  uint32_t               timer_get_unique_id();


private:  
  void run_period();
  
  static const int MAC_MAIN_THREAD_PRIO = -1; // Use default high-priority below UHD
  static const int MAC_PDU_THREAD_PRIO  = DEFAULT_PRIORITY-5;
  static const int MAC_NOF_HARQ_PROC    = 2*HARQ_DELAY_MS;

  // Interaction with PHY 
  phy_interface_mac    *phy_h;
  rlc_interface_mac    *rlc_h; 
  rrc_interface_mac    *rrc_h; 
  srslte::log          *log_h;
  mac_interface_phy::mac_phy_cfg_mbsfn_t phy_mbsfn_cfg;
  
  // MAC configuration 
  mac_cfg_t     config; 

  // UE-specific RNTIs 
  ue_rnti_t     uernti; 
  
  uint32_t      tti; 

  /* Multiplexing/Demultiplexing Units */
  mux           mux_unit; 
  demux         demux_unit; 
  
  /* DL/UL HARQ */
  dl_harq_entity<MAC_NOF_HARQ_PROC, mac_grant_t, tb_action_dl_t, srslte_phy_grant_t> dl_harq;
  ul_harq_entity<MAC_NOF_HARQ_PROC, mac_grant_t, tb_action_ul_t, srslte_phy_grant_t> ul_harq;
  
  /* MAC Uplink-related Procedures */
  ra_proc       ra_procedure;
  sr_proc       sr_procedure; 
  bsr_proc      bsr_procedure; 
  phr_proc      phr_procedure; 
  
  /* Buffers for PCH reception (not included in DL HARQ) */
  const static uint32_t  pch_payload_buffer_sz = 8*1024;
  srslte_softbuffer_rx_t pch_softbuffer;
  uint8_t                pch_payload_buffer[pch_payload_buffer_sz];

  /* Buffers for MCH reception (not included in DL HARQ) */
  const static uint32_t  mch_payload_buffer_sz = SRSLTE_MAX_BUFFER_SIZE_BYTES;
  srslte_softbuffer_rx_t mch_softbuffer;
  uint8_t                mch_payload_buffer[mch_payload_buffer_sz];
  srslte::mch_pdu        mch_msg;
 


  /* Functions for MAC Timers */
  uint32_t        timer_alignment;
  uint32_t        contention_resolution_timer;
  void            setup_timers();
  void            timer_alignment_expire();
  srslte::timers  timers;

  // pointer to MAC PCAP object
  srslte::mac_pcap* pcap;
  bool is_first_ul_grant;

  mac_metrics_t metrics;

  /* Class to process MAC PDUs from DEMUX unit */
  class pdu_process : public thread {
  public: 
    pdu_process(demux *demux_unit);
    void notify();
    void stop();
  private:
    void run_thread();
    bool running; 
    bool have_data; 
    pthread_mutex_t mutex;
    pthread_cond_t  cvar;
    demux* demux_unit;
  };
  pdu_process pdu_process_thread;
};

} // namespace srsue

#endif // SRSUE_MAC_H
