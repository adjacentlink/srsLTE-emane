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

#include "srsue/hdr/stack/rrc/rrc.h"
#include "srslte/asn1/rrc_asn1.h"
#include "srslte/common/bcd_helpers.h"
#include "srslte/common/security.h"
#include "srsue/hdr/stack/rrc/rrc_procedures.h"
#include <cstdlib>
#include <ctime>
#include <inttypes.h> // for printing uint64_t
#include <iostream>
#include <math.h>
#include <sstream>
#include <string.h>
#include <unistd.h>

bool simulate_rlf = false;

using namespace srslte;
using namespace asn1::rrc;

namespace srsue {

const static uint32_t NOF_REQUIRED_SIBS = 4;
const static uint32_t required_sibs[NOF_REQUIRED_SIBS] = {0,1,2,12}; // SIB1, SIB2, SIB3 and SIB13 (eMBMS)

/*******************************************************************************
  Base functions 
*******************************************************************************/

rrc::rrc(srslte::log* rrc_log_) :
  state(RRC_STATE_IDLE),
  last_state(RRC_STATE_CONNECTED),
  drb_up(false),
  rrc_log(rrc_log_),
  serving_cell(unique_cell_t(new cell_t({}, 0.0)))
{
}

rrc::~rrc()
{
}

static void srslte_rrc_handler(asn1::srsasn_logger_level_t level, void* ctx, const char* str)
{
  rrc *r = (rrc *) ctx;
  r->srslte_rrc_log(str); // FIXME use log level
}

void rrc::srslte_rrc_log(const char* str)
{
  if (rrc_log) {
    rrc_log->warning("[ASN]: %s\n", str);
  } else {
    rrc_log->console("[ASN]: %s\n", str);
  }
}

template <class T>
void rrc::log_rrc_message(const std::string source, const direction_t dir, const byte_buffer_t* pdu, const T& msg)
{
  if (rrc_log->get_level() == srslte::LOG_LEVEL_INFO) {
    rrc_log->info("%s - %s %s (%d B)\n", source.c_str(), (dir == Rx) ? "Rx" : "Tx",
                  msg.msg.c1().type().to_string().c_str(), pdu->N_bytes);
  } else if (rrc_log->get_level() >= srslte::LOG_LEVEL_DEBUG) {
    asn1::json_writer json_writer;
    msg.to_json(json_writer);
    rrc_log->debug_hex(pdu->msg, pdu->N_bytes, "%s - %s %s (%d B)\n", source.c_str(), (dir == Rx) ? "Rx" : "Tx",
                       msg.msg.c1().type().to_string().c_str(), pdu->N_bytes);
    rrc_log->debug_long("Content:\n%s\n", json_writer.to_string().c_str());
  }
}

void rrc::init(phy_interface_rrc_lte* phy_,
               mac_interface_rrc*     mac_,
               rlc_interface_rrc*     rlc_,
               pdcp_interface_rrc*    pdcp_,
               nas_interface_rrc*     nas_,
               usim_interface_rrc*    usim_,
               gw_interface_rrc*      gw_,
               srslte::timers*        timers_,
               stack_interface_rrc*   stack_,
               const rrc_args_t&      args_)
{
  pool  = byte_buffer_pool::get_instance();
  phy = phy_;
  mac = mac_;
  rlc = rlc_;
  pdcp = pdcp_;
  nas   = nas_;
  usim  = usim_;
  gw    = gw_;
  stack = stack_;

  args = args_;

  // Use MAC timers
  timers           = timers_;
  state            = RRC_STATE_IDLE;
  plmn_is_selected = false;

  security_is_activated = false;

  t300 = timers->get_unique_id();
  t301 = timers->get_unique_id();
  t302 = timers->get_unique_id();
  t310 = timers->get_unique_id();
  t311 = timers->get_unique_id();
  t304 = timers->get_unique_id();

  ue_identity_configured = false;

  transaction_id = 0;

  // Register logging handler with asn1 rrc
  asn1::rrc::rrc_log_register_handler(this, srslte_rrc_handler);

  cell_clean_cnt = 0;

  pending_mob_reconf = false;

  // Set default values for RRC. MAC and PHY are set to default themselves
  set_rrc_default();

  measurements.init(this);

  struct timeval tv;
  gettimeofday(&tv, NULL);
  rrc_log->info("using srand seed of %ld\n", tv.tv_usec);

  // set seed (used in CHAP auth and attach)
  srand(tv.tv_usec);

  // initiate unique procedures
  ue_required_sibs.assign(&required_sibs[0], &required_sibs[NOF_REQUIRED_SIBS]);

  running = true;
  initiated = true;
}

void rrc::stop() {
  running = false;
  stop_timers();
  cmd_msg_t msg;
  msg.command = cmd_msg_t::STOP;
  cmd_q.push(std::move(msg));
}

void rrc::get_metrics(rrc_metrics_t& m)
{
  m.state = state;
}

bool rrc::is_connected() {
  return (RRC_STATE_CONNECTED == state);
}

bool rrc::have_drb()
{
  return drb_up;
}

/*
 *
 * RRC State Machine
 *
 */
void rrc::run_tti(uint32_t tti)
{
  if (!initiated) {
    return;
  }

  if (simulate_rlf) {
    radio_link_failure();
    simulate_rlf = false;
  }

  // Process pending PHY measurements in IDLE/CONNECTED
  process_phy_meas();

  // Process on-going callbacks, and clear finished callbacks
  callback_list.run();

  // Log state changes
  if (state != last_state) {
    rrc_log->debug("State %s\n", rrc_state_text[state]);
    last_state = state;
  }

  // Run state machine
  switch (state) {
    case RRC_STATE_IDLE:

      /* CAUTION: The execution of cell_search() and cell_selection() take more than 1 ms
       * and will slow down MAC TTI ticks. This has no major effect at the moment because
       * the UE is in IDLE but we could consider splitting MAC and RRC threads to avoid this
       */

      // If attached but not camping on the cell, perform cell reselection
      if (nas->is_attached()) {
        start_cell_reselection();
      }
      break;
    case RRC_STATE_CONNECTED:
      // Performing reestablishment cell selection
      if (m_reest_cause != asn1::rrc::reest_cause_e::nulltype) {
        proc_con_restablish_request();
      }
      measurements.run_tti(tti);
      break;
    default:
      break;
  }

  // Handle Received Messages
  if (running) {
    cmd_msg_t msg;
    if (cmd_q.try_pop(&msg)) {
      switch (msg.command) {
        case cmd_msg_t::PDU:
          process_pdu(msg.lcid, std::move(msg.pdu));
          break;
        case cmd_msg_t::PCCH:
          process_pcch(std::move(msg.pdu));
          break;
        case cmd_msg_t::PDU_MCH:
          parse_pdu_mch(msg.lcid, std::move(msg.pdu));
          break;
        case cmd_msg_t::RLF:
          radio_link_failure();
          break;
        case cmd_msg_t::PDU_BCCH_DLSCH:
          parse_pdu_bcch_dlsch(std::move(msg.pdu));
          break;
        case cmd_msg_t::STOP:
          return;
      }
    }
  }

  // Clean old neighbours
  cell_clean_cnt++;
  if (cell_clean_cnt == 1000) {
    clean_neighbours();
    cell_clean_cnt = 0;
  }
}









/*******************************************************************************
*
*
*
* NAS interface: PLMN search and RRC connection establishment
*
*
*
*******************************************************************************/

uint16_t rrc::get_mcc() {
  return serving_cell->get_mcc();
}

uint16_t rrc::get_mnc() {
  return serving_cell->get_mnc();
}

/* NAS interface to search for available PLMNs.
 * It goes through all known frequencies, synchronizes and receives SIB1 for each to extract PLMN.
 * The function is blocking and waits until all frequencies have been
 * searched and PLMNs are obtained.
 *
 * This function is thread-safe with connection_request()
 */
bool rrc::plmn_search()
{
  if (not plmn_searcher.launch(this)) {
    rrc_log->error("Unable to initiate PLMN search\n");
    return false;
  }
  callback_list.defer_proc(plmn_searcher);
  return true;
}

/* This is the NAS interface. When NAS requests to select a PLMN we have to
 * connect to either register or because there is pending higher layer traffic.
 */
void rrc::plmn_select(srslte::plmn_id_t plmn_id)
{
  plmn_is_selected = true;
  selected_plmn_id = plmn_id;

  rrc_log->info("PLMN Selected %s\n", plmn_id.to_string().c_str());
}

/* 5.3.3.2 Initiation of RRC Connection Establishment procedure
 *
 * Higher layers request establishment of RRC connection while UE is in RRC_IDLE
 *
 * This procedure selects a suitable cell for transmission of RRCConnectionRequest and configures
 * it. Sends connectionRequest message and returns if message transmitted successfully.
 * It does not wait until completition of Connection Establishment procedure
 */
bool rrc::connection_request(srslte::establishment_cause_t cause, srslte::unique_byte_buffer_t dedicated_info_nas_)
{
  if (not conn_req_proc.launch(this, cause, std::move(dedicated_info_nas_))) {
    rrc_log->error("Failed to initiate connection request procedure\n");
    return false;
  }
  callback_list.defer_proc(conn_req_proc);
  return true;
}

void rrc::set_ue_identity(srslte::s_tmsi_t s_tmsi)
{
  ue_identity_configured = true;
  ue_identity            = s_tmsi;
  rrc_log->info(
      "Set ue-Identity to 0x%" PRIu64 ":0x%" PRIu64 "\n", (uint64_t)ue_identity.mmec, (uint64_t)ue_identity.m_tmsi);
}

/*******************************************************************************
*
*
*
* PHY interface: neighbour and serving cell measurements and out-of-sync/in-sync
*
*
*
*******************************************************************************/

/* This function is called from a PHY worker thus must return very quickly.
 * Queue the values of the measurements and process them from the RRC thread
 */
void rrc::new_phy_meas(float rsrp, float rsrq, uint32_t tti, int earfcn_i, int pci_i) {
  uint32_t pci    = 0;
  uint32_t earfcn = 0;
  if (earfcn_i < 0) {
    earfcn = (uint32_t) serving_cell->get_earfcn();
  } else {
    earfcn = (uint32_t) earfcn_i;
  }
  if (pci_i < 0) {
    pci    = (uint32_t) serving_cell->get_pci();
  } else {
    pci    = (uint32_t) pci_i;
  }
  phy_meas_t new_meas = {rsrp, rsrq, tti, earfcn, pci};
  phy_meas_q.push(new_meas);
  rrc_log->info("MEAS:  New measurement earfcn=%d, pci=%d (%s), rsrp=%.1f dBm.\n",
                earfcn_i,
                pci,
                pci_i < 0 ? "serving" : "neighbour",
                rsrp);
}

/* Processes all pending PHY measurements in queue. Must be called from a mutexed function
 */
void rrc::process_phy_meas() {
  phy_meas_t m;
  while(phy_meas_q.try_pop(&m)) {
    rrc_log->debug("MEAS:  Processing measurement. %zd measurements in queue\n", phy_meas_q.size());
    process_new_phy_meas(m);
  }
}

void rrc::process_new_phy_meas(phy_meas_t meas)
{
  float rsrp   = meas.rsrp;
  float rsrq   = meas.rsrq;
  uint32_t tti = meas.tti;
  uint32_t earfcn = meas.earfcn;
  uint32_t pci    = meas.pci;

  if (state == RRC_STATE_IDLE) {
    // Update serving cell
    if (serving_cell->equals(earfcn, pci)) {
      serving_cell->set_rsrp(rsrp);
    } else {
      // Or update/add neighbour cell
      add_neighbour_cell(earfcn, pci, rsrp);
    }
  } else {
    // Measurements in RRC_CONNECTED go through measurement class to log reports etc.
    measurements.new_phy_meas(earfcn, pci, rsrp, rsrq, tti);
  }
}

// Detection of physical layer problems in RRC_CONNECTED (5.3.11.1)
void rrc::out_of_sync()
{

  // CAUTION: We do not lock in this function since they are called from real-time threads

  serving_cell->in_sync = false;

  // upon receiving N310 consecutive "out-of-sync" indications for the PCell from lower layers while neither T300,
  //   T301, T304 nor T311 is running:
  if (state == RRC_STATE_CONNECTED) {
    if (!timers->get(t300)->is_running() && !timers->get(t301)->is_running() && !timers->get(t304)->is_running() &&
        !timers->get(t310)->is_running() && !timers->get(t311)->is_running()) {
      rrc_log->info("Received out-of-sync while in state %s. n310=%d, t311=%s, t310=%s\n",
                    rrc_state_text[state],
                    n310_cnt,
                    timers->get(t311)->is_running() ? "running" : "stop",
                    timers->get(t310)->is_running() ? "running" : "stop");
      n310_cnt++;
      if (n310_cnt == N310) {
        rrc_log->info("Detected %d out-of-sync from PHY. Trying to resync. Starting T310 timer %d ms\n",
                      N310,
                      timers->get(t310)->get_timeout());
        timers->get(t310)->reset();
        timers->get(t310)->run();
        n310_cnt = 0;
      }
    }
  }
}

// Recovery of physical layer problems (5.3.11.2)
void rrc::in_sync()
{
  // CAUTION: We do not lock in this function since they are called from real-time threads
  serving_cell->in_sync = true;
  if (timers->get(t310)->is_running()) {
    n311_cnt++;
    if (n311_cnt == N311) {
      timers->get(t310)->stop();
      n311_cnt = 0;
      rrc_log->info("Detected %d in-sync from PHY. Stopping T310 timer\n", N311);
    }
  }
}















/*******************************************************************************
*
*
*
* Cell selection, reselection and neighbour cell database management
*
*
*
*******************************************************************************/

// Cell selection criteria Section 5.2.3.2 of 36.304
bool rrc::cell_selection_criteria(float rsrp, float rsrq)
{
  return (get_srxlev(rsrp) > 0 || !serving_cell->has_sib3());
}

float rrc::get_srxlev(float Qrxlevmeas) {
  // TODO: Do max power limitation
  float Pcompensation = 0;
  return Qrxlevmeas - (cell_resel_cfg.Qrxlevmin + cell_resel_cfg.Qrxlevminoffset) - Pcompensation;
}

float rrc::get_squal(float Qqualmeas) {
  return Qqualmeas - (cell_resel_cfg.Qqualmin + cell_resel_cfg.Qqualminoffset);
}

// Cell reselection in IDLE Section 5.2.4 of 36.304
void rrc::cell_reselection(float rsrp, float rsrq)
{
  // Intra-frequency cell-reselection criteria

  if (get_srxlev(rsrp) > cell_resel_cfg.s_intrasearchP && rsrp > -95.0) {
    // UE may not perform intra-frequency measurements.
    phy->meas_reset();
    // keep measuring serving cell
    phy->meas_start(phy->get_current_earfcn(), phy->get_current_pci());
  } else {
    // UE must start intra-frequency measurements
    phy->meas_start(phy->get_current_earfcn(), -1);
  }

  // TODO: Inter-frequency cell reselection
}

// Set new serving cell
void rrc::set_serving_cell(phy_interface_rrc_lte::phy_cell_t phy_cell)
{
  if (has_neighbour_cell(phy_cell.earfcn, phy_cell.cell.id)) {
    // Remove future serving cell from neighbours to make space for current serving cell
    unique_cell_t new_serving_cell = remove_neighbour_cell(phy_cell.earfcn, phy_cell.cell.id);

    // Move serving cell to neighbours list
    if (serving_cell->is_valid()) {
      if (has_neighbour_cell(serving_cell->get_earfcn(), serving_cell->get_pci())) {
        rrc_log->error("Error serving cell is already in the neighbour list.\n");
        // note, removing it here is not an option
      } else {
        if (add_neighbour_cell(std::move(serving_cell)) == false) {
          rrc_log->info("Serving cell not added to list of neighbours. Worse than current neighbours\n");
        }
      }
    }

    // Set new serving cell
    serving_cell = std::move(new_serving_cell);
    rrc_log->info("Setting serving cell earfcn=%d, PCI=%d, rsrp=%.2f, nof_neighbours=%zd\n",
                  serving_cell->get_earfcn(),
                  serving_cell->get_pci(),
                  serving_cell->get_rsrp(),
                  neighbour_cells.size());
  } else {
    rrc_log->error("Setting serving cell: Unknown cell with earfcn=%d, PCI=%d\n", phy_cell.earfcn, phy_cell.cell.id);
  }
}

void rrc::delete_last_neighbour()
{
  if (not neighbour_cells.empty()) {
    auto& it = neighbour_cells.back();
    rrc_log->debug("Delete cell earfcn=%d, pci=%d from neighbor list.\n", (*it).get_earfcn(), (*it).get_pci());
    measurements.delete_report((*it).get_earfcn(), (*it).get_pci());
    neighbour_cells.pop_back();
  }
}

/* Called by main RRC thread to remove neighbours from which measurements have not been received in a while
 */
void rrc::clean_neighbours()
{
  struct timeval now;
  gettimeofday(&now, NULL);

  for (auto it = neighbour_cells.begin(); it != neighbour_cells.end();) {
    if ((*it)->timeout_secs(now) > NEIGHBOUR_TIMEOUT) {
      rrc_log->info("Neighbour PCI=%d timed out. Deleting\n", (*it)->get_pci());
      it = neighbour_cells.erase(it);
    } else {
      ++it;
    }
  }
}

// Sort neighbour cells by decreasing order of RSRP and remove old cells from neighbor list
void rrc::sort_neighbour_cells()
{
  // Remove out-of-sync cells
  for (auto it = neighbour_cells.begin(); it != neighbour_cells.end();) {
    if ((*it)->in_sync == false) {
      rrc_log->info("Neighbour PCI=%d is out-of-sync. Deleting\n", (*it)->get_pci());
      it = neighbour_cells.erase(it);
    } else {
      ++it;
    }
  }

  std::sort(std::begin(neighbour_cells), std::end(neighbour_cells), [](const unique_cell_t& a, const unique_cell_t& b) {
    return a->greater(b.get());
  });

  if (neighbour_cells.size() > 0) {
    char ordered[512] = {};
    int  n            = 0;
    n += snprintf(ordered,
                  512,
                  "[earfcn=%d, pci=%d, rsrp=%.2f",
                  neighbour_cells[0]->get_earfcn(),
                  neighbour_cells[0]->phy_cell.cell.id,
                  neighbour_cells[0]->get_rsrp());
    for (uint32_t i=1;i<neighbour_cells.size();i++) {
      n += snprintf(&ordered[n],
                    512 - n,
                    " | earfcn=%d, pci=%d, rsrp=%.2f",
                    neighbour_cells[i]->get_earfcn(),
                    neighbour_cells[i]->get_pci(),
                    neighbour_cells[i]->get_rsrp());
    }
    rrc_log->info("Neighbours: %s]\n", ordered);
  } else {
    rrc_log->info("Neighbours: Empty\n");
  }
}

bool rrc::add_neighbour_cell(unique_cell_t new_cell)
{
  bool ret = false;
  if (neighbour_cells.size() < NOF_NEIGHBOUR_CELLS) {
    ret = true;
  } else if (new_cell->greater(neighbour_cells.back().get())) {
    // delete last neighbour cell
    delete_last_neighbour();
    ret = true;
  }
  if (ret) {
    rrc_log->info("Adding neighbour cell EARFCN=%d, PCI=%d, nof_neighbours=%zd\n",
                  new_cell->get_earfcn(),
                  new_cell->get_pci(),
                  neighbour_cells.size() + 1);
    neighbour_cells.push_back(std::move(new_cell));
  }

  sort_neighbour_cells();
  return ret;
}

// If only neighbour PCI is provided, copy full cell from serving cell
bool rrc::add_neighbour_cell(uint32_t earfcn, uint32_t pci, float rsrp)
{
  if (serving_cell->is_valid()) {
    phy_interface_rrc_lte::phy_cell_t phy_cell;
    phy_cell         = serving_cell->phy_cell;
    phy_cell.earfcn  = earfcn;
    phy_cell.cell.id = pci;
    return add_neighbour_cell(phy_cell, rsrp);
  }
  return false;
}

bool rrc::add_neighbour_cell(phy_interface_rrc_lte::phy_cell_t phy_cell, float rsrp)
{
  // if cell exists, update RSRP, if provided, sort again and return
  cell_t* cell = get_neighbour_cell_handle(phy_cell.earfcn, phy_cell.cell.id);
  if (cell && std::isnormal(rsrp)) {
    cell->set_rsrp(rsrp);
    sort_neighbour_cells();
    return true;
  }

  // if cell doesn't exist, add it
  return add_neighbour_cell(unique_cell_t(new cell_t(phy_cell, rsrp)));
}

// This will remove the cell from the current neighbour list
rrc::unique_cell_t rrc::remove_neighbour_cell(const uint32_t earfcn, const uint32_t pci)
{
  auto it = find_if(neighbour_cells.begin(), neighbour_cells.end(), [&](const unique_cell_t& cell) {
    return cell->equals(earfcn, pci);
  });
  if (it != neighbour_cells.end()) {
    auto retval = std::move(*it);
    it          = neighbour_cells.erase(it);
    return retval;
  }
  return nullptr;
}

cell_t* rrc::get_neighbour_cell_handle(const uint32_t earfcn, const uint32_t pci)
{
  for (auto& cell : neighbour_cells) {
    if (cell->equals(earfcn, pci)) {
      return cell.get();
    }
  }
  return nullptr;
}

bool rrc::has_neighbour_cell(const uint32_t earfcn, const uint32_t pci)
{
  return get_neighbour_cell_handle(earfcn, pci) != nullptr;
}

/*******************************************************************************
 *
 *
 *
 * eMBMS Related Functions
 *
 *
 *
 *******************************************************************************/
std::string rrc::print_mbms()
{
  asn1::rrc::mcch_msg_type_c msg = serving_cell->mcch.msg;
  std::stringstream          ss;
  for (uint32_t i = 0; i < msg.c1().mbsfn_area_cfg_r9().pmch_info_list_r9.size(); i++) {
    ss << "PMCH: " << i << std::endl;
    pmch_info_r9_s* pmch = &msg.c1().mbsfn_area_cfg_r9().pmch_info_list_r9[i];
    for (uint32_t j = 0; j < pmch->mbms_session_info_list_r9.size(); j++) {
      mbms_session_info_r9_s* sess = &pmch->mbms_session_info_list_r9[j];
      ss << "  Service ID: " << sess->tmgi_r9.service_id_r9.to_string();
      if (sess->session_id_r9_present) {
        ss << ", Session ID: " << sess->session_id_r9.to_string();
      }
      if (sess->tmgi_r9.plmn_id_r9.type() == tmgi_r9_s::plmn_id_r9_c_::types::explicit_value_r9) {
        ss << ", MCC: " << mcc_bytes_to_string(&sess->tmgi_r9.plmn_id_r9.explicit_value_r9().mcc[0]);
        ss << ", MNC: " << mnc_bytes_to_string(sess->tmgi_r9.plmn_id_r9.explicit_value_r9().mnc);
      } else {
        ss << ", PLMN index: " << sess->tmgi_r9.plmn_id_r9.plmn_idx_r9();
      }
      ss << ", LCID: " << sess->lc_ch_id_r9 << std::endl;
    }
  }
  return ss.str();
}

bool rrc::mbms_service_start(uint32_t serv, uint32_t port)
{
  bool ret = false;
  if (!serving_cell->has_mcch) {
    rrc_log->error("MCCH not available at MBMS Service Start\n");
    return ret;
  }

  rrc_log->info("%s\n", print_mbms().c_str());

  asn1::rrc::mcch_msg_type_c msg = serving_cell->mcch.msg;
  for (uint32_t i = 0; i < msg.c1().mbsfn_area_cfg_r9().pmch_info_list_r9.size(); i++) {
    pmch_info_r9_s* pmch = &msg.c1().mbsfn_area_cfg_r9().pmch_info_list_r9[i];
    for (uint32_t j = 0; j < pmch->mbms_session_info_list_r9.size(); j++) {
      mbms_session_info_r9_s* sess = &pmch->mbms_session_info_list_r9[j];
      if (serv == sess->tmgi_r9.service_id_r9.to_number()) {
        rrc_log->console("MBMS service started. Service id=%d, port=%d, lcid=%d\n", serv, port, sess->lc_ch_id_r9);
        ret = true;
        add_mrb(sess->lc_ch_id_r9, port);
      }
    }
  }
  return ret;
}

/*******************************************************************************
*
*
*
* Other functions
*
*
*
*******************************************************************************/

/* Detection of radio link failure (5.3.11.3)
 * Upon T310 expiry, RA problem or RLC max retx
 */
void rrc::radio_link_failure() {
  // TODO: Generate and store failure report
  rrc_log->warning("Detected Radio-Link Failure\n");
  rrc_log->console("Warning: Detected Radio-Link Failure\n");
  if (state == RRC_STATE_CONNECTED) {
    init_con_restablish_request(asn1::rrc::reest_cause_e::other_fail);
  }
}

/* Reception of PUCCH/SRS release procedure (Section 5.3.13) */
void rrc::release_pucch_srs() {
  // Apply default configuration for PUCCH (CQI and SR) and SRS (release)
  if (initiated) {
    set_phy_default_pucch_srs();
  }
}

void rrc::ra_problem() {
  radio_link_failure();
}

void rrc::max_retx_attempted() {
  //TODO: Handle the radio link failure
  rrc_log->warning("Max RLC reTx attempted\n");
  cmd_msg_t msg;
  msg.command = cmd_msg_t::RLF;
  cmd_q.push(std::move(msg));
}

void rrc::timer_expired(uint32_t timeout_id) {
  if (timeout_id == t310) {
    rrc_log->info("Timer T310 expired: Radio Link Failure\n");
    radio_link_failure();
  } else if (timeout_id == t311) {
    rrc_log->info("Timer T311 expired: Going to RRC IDLE\n");
    start_go_idle();
  } else if (timeout_id == t301) {
    if (state == RRC_STATE_IDLE) {
      rrc_log->info("Timer T301 expired: Already in IDLE.\n");
    } else {
      rrc_log->info("Timer T301 expired: Going to RRC IDLE\n");
      start_go_idle();
    }
  } else if (timeout_id == t302) {
    rrc_log->info("Timer T302 expired. Informing NAS about barrier alleviation\n");
    nas->set_barring(nas_interface_rrc::BARRING_NONE);
  } else if (timeout_id == t300) {
    // Do nothing, handled in connection_request()
  } else if (timeout_id == t304) {
    rrc_log->console("Timer T304 expired: Handover failed\n");
    ho_failed();
  // fw to measurement
  } else if (!measurements.timer_expired(timeout_id)) {
    rrc_log->error("Timeout from unknown timer id %d\n", timeout_id);
  }
}








/*******************************************************************************
*
*
*
* Connection Control: Establishment, Reconfiguration, Reestablishment and Release
*
*
*
*******************************************************************************/

void rrc::send_con_request(srslte::establishment_cause_t cause)
{
  rrc_log->debug("Preparing RRC Connection Request\n");

  // Prepare ConnectionRequest packet
  ul_ccch_msg_s ul_ccch_msg;
  rrc_conn_request_r8_ies_s* rrc_conn_req =
      &ul_ccch_msg.msg.set_c1().set_rrc_conn_request().crit_exts.set_rrc_conn_request_r8();

  if (ue_identity_configured) {
    rrc_conn_req->ue_id.set_s_tmsi();
    srslte::to_asn1(&rrc_conn_req->ue_id.s_tmsi(), ue_identity);
  } else {
    rrc_conn_req->ue_id.set_random_value();
    // TODO use proper RNG
    uint64_t random_id = 0;
    for (uint i = 0; i < 5; i++) { // fill random ID bytewise, 40 bits = 5 bytes
      random_id |= ( (uint64_t)rand() & 0xFF ) << i*8;
    }
    rrc_conn_req->ue_id.random_value().from_number(random_id);
  }
  rrc_conn_req->establishment_cause = (asn1::rrc::establishment_cause_opts::options)cause;

  send_ul_ccch_msg(ul_ccch_msg);
}

/* RRC connection re-establishment procedure (5.3.7.4) */
void rrc::send_con_restablish_request()
{
  uint16_t crnti;
  uint16_t pci;
  uint32_t cellid;

  // Clean reestablishment type
  asn1::rrc::reest_cause_e cause = m_reest_cause;
  m_reest_cause                  = asn1::rrc::reest_cause_e::nulltype;
  reestablishment_successful     = false;

  if (cause == asn1::rrc::reest_cause_e::ho_fail) {
    crnti  = ho_src_rnti;
    pci    = ho_src_cell.get_pci();
    cellid = ho_src_cell.get_cell_id();
  } else {
    crnti  = m_reest_rnti;
    pci    = serving_cell->get_pci();
    cellid = serving_cell->get_cell_id();
  }

  // Compute shortMAC-I
  uint8_t varShortMAC_packed[16];
  bzero(varShortMAC_packed, 16);
  asn1::bit_ref bref(varShortMAC_packed, sizeof(varShortMAC_packed));

  // ASN.1 encode VarShortMAC-Input
  var_short_mac_input_s varmac;
  varmac.cell_id.from_number(cellid);
  varmac.pci = pci;
  varmac.c_rnti.from_number(crnti);
  varmac.pack(bref);
  uint32_t N_bits  = (uint32_t)bref.distance(varShortMAC_packed);
  uint32_t N_bytes = ((N_bits-1)/8+1);

  rrc_log->info("Encoded varShortMAC: cellId=0x%x, PCI=%d, rnti=0x%x (%d bytes, %d bits)\n",
                cellid, pci, crnti, N_bytes, N_bits);

  // Compute MAC-I
  uint8_t mac_key[4] = {};
  switch(integ_algo) {
    case INTEGRITY_ALGORITHM_ID_128_EIA1:
      security_128_eia1(&k_rrc_int[16],
                        0xffffffff,    // 32-bit all to ones
                        0x1f,          // 5-bit all to ones
                        1,             // 1-bit to one
                        varShortMAC_packed,
                        N_bytes,
                        mac_key);
      break;
    case INTEGRITY_ALGORITHM_ID_128_EIA2:
      security_128_eia2(&k_rrc_int[16],
                        0xffffffff, // 32-bit all to ones
                        0x1f,       // 5-bit all to ones
                        1,          // 1-bit to one
                        varShortMAC_packed,
                        N_bytes,
                        mac_key);
      break;
    case INTEGRITY_ALGORITHM_ID_128_EIA3:
      security_128_eia3(&k_rrc_int[16],
                        0xffffffff, // 32-bit all to ones
                        0x1f,       // 5-bit all to ones
                        1,          // 1-bit to one
                        varShortMAC_packed,
                        N_bytes,
                        mac_key);
      break;
    default:
      rrc_log->info("Unsupported integrity algorithm during reestablishment\n");
  }

  // Prepare ConnectionRestalishmentRequest packet
  asn1::rrc::ul_ccch_msg_s ul_ccch_msg;
  rrc_conn_reest_request_r8_ies_s* rrc_conn_reest_req =
      &ul_ccch_msg.msg.set_c1().set_rrc_conn_reest_request().crit_exts.set_rrc_conn_reest_request_r8();

  rrc_conn_reest_req->ue_id.c_rnti.from_number(crnti);
  rrc_conn_reest_req->ue_id.pci = pci;
  rrc_conn_reest_req->ue_id.short_mac_i.from_number(mac_key[2] << 8 | mac_key[3]);
  rrc_conn_reest_req->reest_cause = cause;

  rrc_log->console("RRC Connection Reestablishment\n");
  send_ul_ccch_msg(ul_ccch_msg);
}

void rrc::send_con_restablish_complete() {

  rrc_log->debug("Preparing RRC Connection Reestablishment Complete\n");
  rrc_log->console("RRC Connected\n");

  // Prepare ConnectionSetupComplete packet
  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ul_dcch_msg.msg.set_c1().set_rrc_conn_reest_complete().crit_exts.set_rrc_conn_reest_complete_r8();
  ul_dcch_msg.msg.c1().rrc_conn_reest_complete().rrc_transaction_id = transaction_id;

  send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

void rrc::send_con_setup_complete(srslte::unique_byte_buffer_t nas_msg)
{
  rrc_log->debug("Preparing RRC Connection Setup Complete\n");

  // Prepare ConnectionSetupComplete packet
  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  rrc_conn_setup_complete_r8_ies_s* rrc_conn_setup_complete =
      &ul_dcch_msg.msg.set_c1().set_rrc_conn_setup_complete().crit_exts.set_c1().set_rrc_conn_setup_complete_r8();

  ul_dcch_msg.msg.c1().rrc_conn_setup_complete().rrc_transaction_id = transaction_id;

  rrc_conn_setup_complete->sel_plmn_id = 1;
  rrc_conn_setup_complete->ded_info_nas.resize(nas_msg->N_bytes);
  memcpy(rrc_conn_setup_complete->ded_info_nas.data(), nas_msg->msg, nas_msg->N_bytes); // TODO Check!

  send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

void rrc::send_ul_info_transfer(unique_byte_buffer_t nas_msg)
{
  uint32_t lcid = rlc->has_bearer(RB_ID_SRB2) ? RB_ID_SRB2 : RB_ID_SRB1;

  // Prepare UL INFO packet
  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ul_info_transfer_r8_ies_s* rrc_ul_info_transfer =
      &ul_dcch_msg.msg.set_c1().set_ul_info_transfer().crit_exts.set_c1().set_ul_info_transfer_r8();

  rrc_ul_info_transfer->ded_info_type.set_ded_info_nas();
  rrc_ul_info_transfer->ded_info_type.ded_info_nas().resize(nas_msg->N_bytes);
  memcpy(rrc_ul_info_transfer->ded_info_type.ded_info_nas().data(), nas_msg->msg, nas_msg->N_bytes); // TODO Check!

  send_ul_dcch_msg(lcid, ul_dcch_msg);
}

void rrc::send_security_mode_complete() {
  rrc_log->debug("Preparing Security Mode Complete\n");

  // Prepare Security Mode Command Complete
  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ul_dcch_msg.msg.set_c1().set_security_mode_complete().crit_exts.set_security_mode_complete_r8();
  ul_dcch_msg.msg.c1().security_mode_complete().rrc_transaction_id = transaction_id;

  send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

void rrc::send_rrc_con_reconfig_complete() {
  rrc_log->debug("Preparing RRC Connection Reconfig Complete\n");

  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ul_dcch_msg.msg.set_c1().set_rrc_conn_recfg_complete().crit_exts.set_rrc_conn_recfg_complete_r8();
  ul_dcch_msg.msg.c1().rrc_conn_recfg_complete().rrc_transaction_id = transaction_id;

  send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

bool rrc::ho_prepare()
{
  if (pending_mob_reconf) {
    asn1::rrc::rrc_conn_recfg_r8_ies_s* mob_reconf_r8 = &mob_reconf.crit_exts.c1().rrc_conn_recfg_r8();
    asn1::rrc::mob_ctrl_info_s*         mob_ctrl_info = &mob_reconf_r8->mob_ctrl_info;
    rrc_log->info("Processing HO command to target PCell=%d\n", mob_ctrl_info->target_pci);

    uint32_t target_earfcn = (mob_ctrl_info->carrier_freq_present) ? mob_ctrl_info->carrier_freq.dl_carrier_freq
                                                                   : serving_cell->get_earfcn();

    if (not has_neighbour_cell(target_earfcn, mob_ctrl_info->target_pci)) {
      rrc_log->console("Received HO command to unknown PCI=%d\n", mob_ctrl_info->target_pci);
      rrc_log->error("Could not find target cell earfcn=%d, pci=%d\n", serving_cell->get_earfcn(),
                     mob_ctrl_info->target_pci);
      return false;
    }

    // Section 5.3.5.4
    timers->get(t310)->stop();
    timers->get(t304)->set(this, mob_ctrl_info->t304.to_number());

    // Save serving cell and current configuration
    ho_src_cell = *serving_cell;
    mac_interface_rrc::ue_rnti_t uernti;
    mac->get_rntis(&uernti);
    ho_src_rnti = uernti.crnti;

    // Reset/Reestablish stack
    pdcp->reestablish();
    rlc->reestablish();
    mac->wait_uplink();
    mac->clear_rntis();
    phy->meas_reset();
    mac->reset();
    phy->reset();

    mac->set_ho_rnti(mob_ctrl_info->new_ue_id.to_number(), mob_ctrl_info->target_pci);

    // Apply common config, but do not send to lower layers if Dedicated is present (to avoid sending twice)
    apply_rr_config_common(&mob_ctrl_info->rr_cfg_common, !mob_reconf_r8->rr_cfg_ded_present);

    if (mob_reconf_r8->rr_cfg_ded_present) {
      apply_rr_config_dedicated(&mob_reconf_r8->rr_cfg_ded);
    }

    cell_t* target_cell = get_neighbour_cell_handle(target_earfcn, mob_ctrl_info->target_pci);
    if (!phy->cell_select(&target_cell->phy_cell)) {
      rrc_log->error("Could not synchronize with target cell pci=%d. Trying to return to source PCI\n",
                     target_cell->get_pci());
      return false;
    }

    set_serving_cell(target_cell->phy_cell);

    // Extract and apply scell config if any
    apply_scell_config(mob_reconf_r8);

    if (mob_ctrl_info->rach_cfg_ded_present) {
      rrc_log->info("Starting non-contention based RA with preamble_idx=%d, mask_idx=%d\n",
                    mob_ctrl_info->rach_cfg_ded.ra_preamb_idx, mob_ctrl_info->rach_cfg_ded.ra_prach_mask_idx);
      mac->start_noncont_ho(mob_ctrl_info->rach_cfg_ded.ra_preamb_idx, mob_ctrl_info->rach_cfg_ded.ra_prach_mask_idx);
    } else {
      rrc_log->info("Starting contention-based RA\n");
      mac->start_cont_ho();
    }

    int ncc = -1;
    if (mob_reconf_r8->security_cfg_ho_present) {
      ncc = mob_reconf_r8->security_cfg_ho.ho_type.intra_lte().next_hop_chaining_count;
      if (mob_reconf_r8->security_cfg_ho.ho_type.intra_lte().key_change_ind) {
        rrc_log->console("keyChangeIndicator in securityConfigHO not supported\n");
        return false;
      }
      if (mob_reconf_r8->security_cfg_ho.ho_type.intra_lte().security_algorithm_cfg_present) {
        cipher_algo = (CIPHERING_ALGORITHM_ID_ENUM)mob_reconf_r8->security_cfg_ho.ho_type.intra_lte()
                          .security_algorithm_cfg.ciphering_algorithm.to_number();
        integ_algo = (INTEGRITY_ALGORITHM_ID_ENUM)mob_reconf_r8->security_cfg_ho.ho_type.intra_lte()
                         .security_algorithm_cfg.integrity_prot_algorithm.to_number();
        rrc_log->info("Changed Ciphering to %s and Integrity to %s\n",
                      ciphering_algorithm_id_text[cipher_algo],
                      integrity_algorithm_id_text[integ_algo]);
      }
    }

    usim->generate_as_keys_ho(mob_ctrl_info->target_pci, phy->get_current_earfcn(), ncc, k_rrc_enc, k_rrc_int, k_up_enc,
                              k_up_int, cipher_algo, integ_algo);

    pdcp->config_security_all(k_rrc_enc, k_rrc_int, k_up_enc, cipher_algo, integ_algo);
    send_rrc_con_reconfig_complete();
  }
  return true;
}

void rrc::ho_ra_completed(bool ra_successful)
{
  if (pending_mob_reconf) {
    asn1::rrc::rrc_conn_recfg_r8_ies_s* mob_reconf_r8 = &mob_reconf.crit_exts.c1().rrc_conn_recfg_r8();
    if (ra_successful) {
      measurements.ho_finish();

      if (mob_reconf_r8->meas_cfg_present) {
        measurements.parse_meas_config(&mob_reconf_r8->meas_cfg);
      }

      timers->get(t304)->stop();
    }
    // T304 will expiry and send ho_failure

    rrc_log->info("HO %ssuccessful\n", ra_successful?"":"un");
    rrc_log->console("HO %ssuccessful\n", ra_successful?"":"un");

    pending_mob_reconf = false;
  } else {
    rrc_log->error("Received HO random access completed but no pending mobility reconfiguration info\n");
  }
}

bool rrc::con_reconfig_ho(asn1::rrc::rrc_conn_recfg_s* reconfig)
{
  asn1::rrc::rrc_conn_recfg_r8_ies_s* mob_reconf_r8 = &reconfig->crit_exts.c1().rrc_conn_recfg_r8();
  if (mob_reconf_r8->mob_ctrl_info.target_pci == phy->get_current_pci()) {
    rrc_log->console("Warning: Received HO command to own cell\n");
    rrc_log->warning("Received HO command to own cell\n");
    return false;
  }

  rrc_log->info("Received HO command to target PCell=%d\n", mob_reconf_r8->mob_ctrl_info.target_pci);
  rrc_log->console("Received HO command to target PCell=%d, NCC=%d\n", mob_reconf_r8->mob_ctrl_info.target_pci,
                   mob_reconf_r8->security_cfg_ho.ho_type.intra_lte().next_hop_chaining_count);

  // store mobilityControlInfo
  mob_reconf         = *reconfig;
  pending_mob_reconf = true;

  start_ho();
  return true;
}

void rrc::start_ho()
{
  callback_list.defer_task([this]() {
    if (state != RRC_STATE_CONNECTED) {
      rrc_log->info("HO interrupted, since RRC is no longer in connected state\n");
      return srslte::proc_outcome_t::success;
    }
    if (not ho_prepare()) {
      con_reconfig_failed();
      return srslte::proc_outcome_t::error;
    }
    return srslte::proc_outcome_t::success;
  });
}

void rrc::start_go_idle()
{
  if (not idle_setter.launch(this)) {
    rrc_log->info("Failed to set RRC to IDLE\n");
    return;
  }
  callback_list.defer_proc(idle_setter);
}

// Handle RRC Reconfiguration without MobilityInformation Section 5.3.5.3
bool rrc::con_reconfig(asn1::rrc::rrc_conn_recfg_s* reconfig)
{
  asn1::rrc::rrc_conn_recfg_r8_ies_s* reconfig_r8 = &reconfig->crit_exts.c1().rrc_conn_recfg_r8();

  // If this is the first con_reconfig after a reestablishment
  if (reestablishment_successful) {
    // Reestablish PDCP and RLC for SRB2 and all DRB
    // FIXME: Which is the maximum LCID?
    for (int i = 2; i < SRSLTE_N_RADIO_BEARERS; i++) {
      if (rlc->has_bearer(i)) {
        pdcp->reestablish(i);
        rlc->reestablish(i);
      }
    }
  }

  // Apply RR config as in 5.3.10
  if (reconfig_r8->rr_cfg_ded_present) {
    if (!apply_rr_config_dedicated(&reconfig_r8->rr_cfg_ded)) {
      return false;
    }
  }

  apply_scell_config(reconfig_r8);

  // If first message after reestablishment, resume SRB2 and all DRB
  if (reestablishment_successful) {
    reestablishment_successful = false;
    for (int i = 2; i < SRSLTE_N_RADIO_BEARERS; i++) {
      if (rlc->has_bearer(i)) {
        rlc->resume_bearer(i);
      }
    }
  }

  if (reconfig_r8->meas_cfg_present) {
    if (!measurements.parse_meas_config(&reconfig_r8->meas_cfg)) {
      return false;
    }
  }

  send_rrc_con_reconfig_complete();

  unique_byte_buffer_t nas_sdu;
  for (uint32_t i = 0; i < reconfig_r8->ded_info_nas_list.size(); i++) {
    nas_sdu = srslte::allocate_unique_buffer(*pool);
    if (nas_sdu.get()) {
      memcpy(nas_sdu->msg, reconfig_r8->ded_info_nas_list[i].data(), reconfig_r8->ded_info_nas_list[i].size());
      nas_sdu->N_bytes = reconfig_r8->ded_info_nas_list[i].size();
      nas->write_pdu(RB_ID_SRB1, std::move(nas_sdu));
    } else {
      rrc_log->error("Fatal Error: Couldn't allocate PDU in handle_rrc_con_reconfig().\n");
      return false;
    }
  }
  return true;
}

// HO failure from T304 expiry 5.3.5.6
void rrc::ho_failed()
{
  init_con_restablish_request(asn1::rrc::reest_cause_e::ho_fail);
}

// Reconfiguration failure or Section 5.3.5.5
void rrc::con_reconfig_failed()
{
  // Set previous PHY/MAC configuration
  phy->set_config(previous_phy_cfg);
  mac->set_config(previous_mac_cfg);

  // And restore current configs
  current_mac_cfg = previous_mac_cfg;
  current_phy_cfg = previous_phy_cfg;

  if (security_is_activated) {
    // Start the Reestablishment Procedure
    init_con_restablish_request(asn1::rrc::reest_cause_e::recfg_fail);
  } else {
    start_go_idle();
  }
}

void rrc::handle_rrc_con_reconfig(uint32_t lcid, asn1::rrc::rrc_conn_recfg_s* reconfig)
{
  previous_phy_cfg = current_phy_cfg;
  previous_mac_cfg = current_mac_cfg;

  asn1::rrc::rrc_conn_recfg_r8_ies_s* reconfig_r8 = &reconfig->crit_exts.c1().rrc_conn_recfg_r8();
  if (reconfig_r8->mob_ctrl_info_present) {
    if (!con_reconfig_ho(reconfig)) {
      con_reconfig_failed();
    }
  } else {
    if (!con_reconfig(reconfig)) {
      con_reconfig_failed();
    }
  }
}

/* Actions upon reception of RRCConnectionRelease 5.3.8.3 */
void rrc::rrc_connection_release() {
  // Save idleModeMobilityControlInfo, etc.
  rrc_log->console("Received RRC Connection Release\n");
  start_go_idle();
}

/* Actions upon leaving RRC_CONNECTED 5.3.12 */
void rrc::leave_connected()
{
  rrc_log->console("RRC IDLE\n");
  rrc_log->info("Leaving RRC_CONNECTED state\n");
  m_reest_cause         = asn1::rrc::reest_cause_e::nulltype;
  state = RRC_STATE_IDLE;
  drb_up = false;
  security_is_activated = false;
  measurements.reset();
  nas->left_rrc_connected();
  pdcp->reset();
  rlc->reset();
  mac->reset();
  phy->reset();
  set_phy_default();
  set_mac_default();
  stop_timers();
  rrc_log->info("Going RRC_IDLE\n");
  if (phy->cell_is_camping()) {
    // Receive paging
    mac->pcch_start_rx();
    // Instruct PHY to measure serving cell for cell reselection
    phy->meas_start(phy->get_current_earfcn(), phy->get_current_pci());
  }
}

void rrc::stop_timers()
{
  timers->get(t300)->stop();
  timers->get(t301)->stop();
  timers->get(t310)->stop();
  timers->get(t311)->stop();
  timers->get(t304)->stop();
}

/* Implementation of procedure in 3GPP 36.331 Section 5.3.7.2: Initiation
 *
 * This procedure shall be only initiated when:
 *   - upon detecting radio link failure, in accordance with 5.3.11; or
 *   - upon handover failure, in accordance with 5.3.5.6; or
 *   - upon mobility from E-UTRA failure, in accordance with 5.4.3.5; or
 *   - upon integrity check failure indication from lower layers; or
 *   - upon an RRC connection reconfiguration failure, in accordance with 5.3.5.5;
 *
 *   The parameter cause shall indicate the cause of the reestablishment according to the sections mentioned adobe.
 */
void rrc::init_con_restablish_request(asn1::rrc::reest_cause_e cause)
{
  // Save Current RNTI before MAC Reset
  mac_interface_rrc::ue_rnti_t uernti;
  mac->get_rntis(&uernti);

  // If security is activated, RRC connected and C-RNTI available
  if (security_is_activated && state == RRC_STATE_CONNECTED && uernti.crnti != 0) {
    // Save reestablishment cause and current C-RNTI
    m_reest_rnti  = uernti.crnti;
    m_reest_cause = cause;
    reestablishment_started = false;

    // initiation of reestablishment procedure as indicates in 3GPP 36.331 Section 5.3.7.2
    rrc_log->info("Initiating RRC Connection Reestablishment Procedure\n");
  } else {
    // 3GPP 36.331 Section 5.3.7.1
    // If AS security has not been activated, the UE does not initiate the procedure but instead
    // moves to RRC_IDLE directly
    start_go_idle();
  }
}

/* Implementation of procedure in 3GPP 36.331 Section 5.3.7.3: Actions following cell selection while T311 is running
 */
void rrc::proc_con_restablish_request()
{
  if (!reestablishment_started) {

    reestablishment_started = true;

    rrc_log->info("Resetting timers and MAC in RRC Connection Reestablishment Procedure\n");

    // stop timer T310, if running;
    timers->get(t310)->stop();

    // start timer T311;
    timers->get(t311)->reset();
    timers->get(t311)->run();

    // Suspend all RB except SRB0
    for (int i = 1; i < SRSLTE_N_RADIO_BEARERS; i++) {
      if (rlc->has_bearer(i)) {
        rlc->suspend_bearer(i);
      }
    }

    // reset MAC;
    mac->reset();

    // apply the default physical channel configuration as specified in 9.2.4;
    set_phy_default_pucch_srs();

    // apply the default semi-persistent scheduling configuration as specified in 9.2.3;
    // N.A.

    // apply the default MAC main configuration as specified in 9.2.2;
    apply_mac_config_dedicated_default();

    // perform cell selection in accordance with the cell selection process as specified in TS 36.304 [4];
    // ... this happens in rrc::run_tti()
  }

  // Check timer...
  if (timers->get(t311)->is_running()) {
    // Check for synchronism
    if (serving_cell->in_sync) {
      // Perform cell selection in accordance to 36.304
      if (cell_selection_criteria(serving_cell->get_rsrp())) {
        // Actions following cell reselection while T311 is running 5.3.7.3
        // Upon selecting a suitable E-UTRA cell, the UE shall:
        rrc_log->info("Cell Selection criteria passed after %dms. Sending RRC Connection Reestablishment Request\n",
                      timers->get(t311)->value());

        // stop timer T311;
        timers->get(t301)->reset();

        // start timer T301;
        timers->get(t301)->run();

        // apply the timeAlignmentTimerCommon included in SystemInformationBlockType2;
        timers->get(t311)->stop();

        // initiate transmission of the RRCConnectionReestablishmentRequest message in accordance with 5.3.7.4;
        send_con_restablish_request();
      } else {
        // Upon selecting an inter-RAT cell
        rrc_log->warning("Reestablishment Cell Selection criteria failed.\n");
        rrc_log->console("Reestablishment Cell Selection criteria failed. in_sync=%d\n", serving_cell->in_sync);
        leave_connected();
      }
    } else {
      // No synchronized, do nothing
    }
  } else {
    // t311 expired or stopped
    rrc_log->info("T311 expired while selecting cell. Going to IDLE\n");
    rrc_log->console("T311 expired while selecting cell. Going to IDLE\n");
    leave_connected();
  }
}

void rrc::start_cell_reselection()
{
  if (neighbour_cells.empty() and serving_cell->in_sync and phy->cell_is_camping()) {
    // don't bother with cell selection if there are no neighbours and we are already camping
    return;
  }

  if (not cell_selector.launch(this)) {
    rrc_log->error("Failed to initiate a Cell Selection procedure...\n");
    return;
  }

  rrc_log->info("Cell Reselection - Starting...\n");
  callback_list.defer_task([this]() {
    if (cell_selector.run()) {
      return srslte::proc_outcome_t::yield;
    }
    cell_selection_proc ret = cell_selector.pop();
    if (ret.is_error()) {
      rrc_log->error("Cell Reselection - Error while selecting a cell\n");
      return srslte::proc_outcome_t::error;
    } else {
      switch (ret.get_cs_result()) {
        case cs_result_t::changed_cell:
          // New cell has been selected, start receiving PCCH
          mac->pcch_start_rx();
          break;
        case cs_result_t::no_cell:
          rrc_log->warning("Could not find any cell to camp on\n");
          break;
        case cs_result_t::same_cell:
          if (!phy->cell_is_camping()) {
            rrc_log->warning("Did not reselect cell but serving cell is out-of-sync.\n");
            serving_cell->in_sync = false;
          }
          break;
      }
    }
    rrc_log->info("Cell Reselection - Finished successfully\n");
    return srslte::proc_outcome_t::success;
  });
}

void rrc::cell_search_completed(const phy_interface_rrc_lte::cell_search_ret_t& cs_ret,
                                const phy_interface_rrc_lte::phy_cell_t&        found_cell)
{
  cell_searcher.trigger_event(cell_search_proc::cell_search_event_t{cs_ret, found_cell});
}

/*******************************************************************************
 *
 *
 *
 * Reception of Broadcast messages (MIB and SIBs)
 *
 *
 *
 *******************************************************************************/
void rrc::write_pdu_bcch_bch(unique_byte_buffer_t pdu)
{
  asn1::rrc::bcch_bch_msg_s bch_msg;
  asn1::bit_ref             bch_bref(pdu->msg, pdu->N_bytes);
  asn1::SRSASN_CODE         err = bch_msg.unpack(bch_bref);
  if (err != asn1::SRSASN_SUCCESS) {
    rrc_log->error("Could not unpack BCCH-BCH message.\n");
    return;
  }

  if (rrc_log->get_level() == srslte::LOG_LEVEL_INFO) {
    rrc_log->info("BCCH-BCH - Rx (%d B)\n", pdu->N_bytes);
  } else if (rrc_log->get_level() >= srslte::LOG_LEVEL_DEBUG) {
    asn1::json_writer json_writer;
    bch_msg.to_json(json_writer);
    rrc_log->debug_hex(pdu->msg, pdu->N_bytes, "BCCH-BCH - Rx (%d B)\n", pdu->N_bytes);
    rrc_log->debug("Content:\n%s\n", json_writer.to_string().c_str());
  }

  // Do we need to do something with BCH?
}

void rrc::write_pdu_bcch_dlsch(unique_byte_buffer_t pdu)
{
  cmd_msg_t msg;
  msg.command = cmd_msg_t::PDU_BCCH_DLSCH;
  msg.pdu     = std::move(pdu);
  cmd_q.push(std::move(msg));
}

void rrc::parse_pdu_bcch_dlsch(unique_byte_buffer_t pdu)
{
  // Stop BCCH search after successful reception of 1 BCCH block
  mac->bcch_stop_rx();

  asn1::rrc::bcch_dl_sch_msg_s dlsch_msg;
  asn1::bit_ref                dlsch_bref(pdu->msg, pdu->N_bytes);
  asn1::SRSASN_CODE            err = dlsch_msg.unpack(dlsch_bref);
  if (err != asn1::SRSASN_SUCCESS or dlsch_msg.msg.type().value != bcch_dl_sch_msg_type_c::types_opts::c1) {
    rrc_log->error("Could not unpack BCCH DL-SCH message.\n");
    return;
  }

  log_rrc_message("BCCH-DLSCH", Rx, pdu.get(), dlsch_msg);

  if (dlsch_msg.msg.c1().type() == bcch_dl_sch_msg_type_c::c1_c_::types::sib_type1) {
    rrc_log->info("Processing SIB1 (1/1)\n");
    serving_cell->set_sib1(&dlsch_msg.msg.c1().sib_type1());
    handle_sib1();
  } else {
    sys_info_r8_ies_s::sib_type_and_info_l_& sib_list =
        dlsch_msg.msg.c1().sys_info().crit_exts.sys_info_r8().sib_type_and_info;
    for (uint32_t i = 0; i < sib_list.size(); ++i) {
      rrc_log->info("Processing SIB%d (%d/%d)\n", sib_list[i].type().to_number(), i, sib_list.size());
      switch (sib_list[i].type().value) {
        case sib_info_item_c::types::sib2:
          if (not serving_cell->has_sib2()) {
            serving_cell->set_sib2(&sib_list[i].sib2());
          }
          handle_sib2();
          break;
        case sib_info_item_c::types::sib3:
          if (not serving_cell->has_sib3()) {
            serving_cell->set_sib3(&sib_list[i].sib3());
          }
          handle_sib3();
          break;
        case sib_info_item_c::types::sib13_v920:
          if (not serving_cell->has_sib13()) {
            serving_cell->set_sib13(&sib_list[i].sib13_v920());
          }
          handle_sib13();
          break;
        default:
          rrc_log->warning("SIB%d is not supported\n", sib_list[i].type().to_number());
      }
    }
  }
}

void rrc::handle_sib1()
{
  sib_type1_s* sib1 = serving_cell->sib1ptr();
  rrc_log->info("SIB1 received, CellID=%d, si_window=%d, sib2_period=%d\n", serving_cell->get_cell_id() & 0xfff,
                sib1->si_win_len.to_number(), sib1->sched_info_list[0].si_periodicity.to_number());

  // Print SIB scheduling info
  for (uint32_t i = 0; i < sib1->sched_info_list.size(); ++i) {
    sched_info_s::si_periodicity_e_ p = sib1->sched_info_list[i].si_periodicity;
    for (uint32_t j = 0; j < sib1->sched_info_list[i].sib_map_info.size(); ++j) {
      sib_type_e t = sib1->sched_info_list[i].sib_map_info[j];
      rrc_log->debug("SIB scheduling info, sib_type=%d, si_periodicity=%d\n", t.to_number(), p.to_number());
    }
  }

  // Set TDD Config
  if (sib1->tdd_cfg_present) {
    srslte_tdd_config_t tdd_config;
    tdd_config.sf_config = sib1->tdd_cfg.sf_assign.to_number();
    tdd_config.ss_config = sib1->tdd_cfg.special_sf_patterns.to_number();
    phy->set_config_tdd(tdd_config);
  }
}

void rrc::handle_sib2()
{
  rrc_log->info("SIB2 received\n");

  sib_type2_s* sib2 = serving_cell->sib2ptr();

  // Apply RACH and timeAlginmentTimer configuration
  set_mac_cfg_t_rach_cfg_common(&current_mac_cfg, sib2->rr_cfg_common.rach_cfg_common);
  set_mac_cfg_t_time_alignment(&current_mac_cfg, sib2->time_align_timer_common);
  mac->set_config(current_mac_cfg);

  // Set MBSFN configs
  if (sib2->mbsfn_sf_cfg_list_present) {
    srslte::mbsfn_sf_cfg_t list[ASN1_RRC_MAX_MBSFN_ALLOCS];
    for (uint32_t i = 0; i < sib2->mbsfn_sf_cfg_list.size(); ++i) {
      list[i] = srslte::make_mbsfn_sf_cfg(sib2->mbsfn_sf_cfg_list[i]);
    }
    phy->set_config_mbsfn_sib2(&list[0], sib2->mbsfn_sf_cfg_list.size());
  }

  // Apply PHY RR Config Common
  set_phy_cfg_t_common_pdsch(&current_phy_cfg, sib2->rr_cfg_common.pdsch_cfg_common);
  set_phy_cfg_t_common_pusch(&current_phy_cfg, sib2->rr_cfg_common.pusch_cfg_common);
  set_phy_cfg_t_common_pucch(&current_phy_cfg, sib2->rr_cfg_common.pucch_cfg_common);
  set_phy_cfg_t_common_pwr_ctrl(&current_phy_cfg, sib2->rr_cfg_common.ul_pwr_ctrl_common);
  set_phy_cfg_t_common_prach(
      &current_phy_cfg, &sib2->rr_cfg_common.prach_cfg.prach_cfg_info, sib2->rr_cfg_common.prach_cfg.root_seq_idx);
  set_phy_cfg_t_common_srs(&current_phy_cfg, sib2->rr_cfg_common.srs_ul_cfg_common);

  // According to 3GPP 36.331 v12 UE-EUTRA-Capability field descriptions
  // Allow 64QAM for:
  //   ue-Category 5 and 8 when enable64QAM (without suffix)
  //   ue-CategoryUL 5 and 13 when enable64QAM (with suffix)
  // enable64QAM-v1270 shall be ignored if enable64QAM (without suffix) is false
  if (args.ue_category == 5 || (args.release >= 10 && args.ue_category == 8)) {
    set_phy_cfg_t_enable_64qam(&current_phy_cfg, sib2->rr_cfg_common.pusch_cfg_common.pusch_cfg_basic.enable64_qam);
  } else if (args.release >= 12 && sib2->rr_cfg_common.pusch_cfg_common.pusch_cfg_basic.enable64_qam) {
    if (args.ue_category_ul == 5 || args.ue_category_ul == 13) {
      // ASN1 Generator simplifies enable64QAM-v1270 because it is an enumeration that is always true
      set_phy_cfg_t_enable_64qam(&current_phy_cfg, sib2->rr_cfg_common.pusch_cfg_common_v1270.is_present());
    } else {
      set_phy_cfg_t_enable_64qam(&current_phy_cfg, false);
    }
  } else {
    set_phy_cfg_t_enable_64qam(&current_phy_cfg, false);
  }

  phy->set_config(current_phy_cfg);

  log_rr_config_common();

  timers->get(t300)->set(this, sib2->ue_timers_and_consts.t300.to_number());
  timers->get(t301)->set(this, sib2->ue_timers_and_consts.t301.to_number());
  timers->get(t310)->set(this, sib2->ue_timers_and_consts.t310.to_number());
  timers->get(t311)->set(this, sib2->ue_timers_and_consts.t311.to_number());
  N310 = sib2->ue_timers_and_consts.n310.to_number();
  N311 = sib2->ue_timers_and_consts.n311.to_number();

  rrc_log->info("Set Constants and Timers: N310=%d, N311=%d, t300=%d, t301=%d, t310=%d, t311=%d\n",
                N310,
                N311,
                timers->get(t300)->get_timeout(),
                timers->get(t301)->get_timeout(),
                timers->get(t310)->get_timeout(),
                timers->get(t311)->get_timeout());
}

void rrc::handle_sib3()
{
  rrc_log->info("SIB3 received\n");

  sib_type3_s* sib3 = serving_cell->sib3ptr();

  // cellReselectionInfoCommon
  cell_resel_cfg.q_hyst = sib3->cell_resel_info_common.q_hyst.to_number();

  // cellReselectionServingFreqInfo
  cell_resel_cfg.threshservinglow = sib3->thresh_serving_low_q_r9; // TODO: Check first if present

  // intraFreqCellReselectionInfo
  cell_resel_cfg.Qrxlevmin = sib3->intra_freq_cell_resel_info.q_rx_lev_min * 2; // multiply by two
  if (sib3->intra_freq_cell_resel_info.s_intra_search_present) {
    cell_resel_cfg.s_intrasearchP = sib3->intra_freq_cell_resel_info.s_intra_search;
  } else {
    cell_resel_cfg.s_intrasearchP = INFINITY;
  }
}

void rrc::handle_sib13()
{
  rrc_log->info("SIB13 received\n");

  sib_type13_r9_s* sib13 = serving_cell->sib13ptr();

  phy->set_config_mbsfn_sib13(srslte::make_sib13(*sib13));
  add_mrb(0, 0); // Add MRB0
}

/*******************************************************************************
*
*
*
* Reception of Paging messages
*
*
*
*******************************************************************************/
void rrc::write_pdu_pcch(unique_byte_buffer_t pdu)
{
  cmd_msg_t msg;
  msg.pdu     = std::move(pdu);
  msg.command = cmd_msg_t::PCCH;
  cmd_q.push(std::move(msg));
}

void rrc::paging_completed(bool outcome)
{
  pcch_processor.trigger_event(process_pcch_proc::paging_complete{outcome});
}

void rrc::process_pcch(unique_byte_buffer_t pdu)
{
  if (pdu->N_bytes <= 0 or pdu->N_bytes >= SRSLTE_MAX_BUFFER_SIZE_BITS) {
    rrc_log->error_hex(pdu->buffer, pdu->N_bytes, "Dropping PCCH message with %d B\n", pdu->N_bytes);
    return;
  }

  pcch_msg_s    pcch_msg;
  asn1::bit_ref bref(pdu->msg, pdu->N_bytes);
  if (pcch_msg.unpack(bref) != asn1::SRSASN_SUCCESS or pcch_msg.msg.type().value != pcch_msg_type_c::types_opts::c1) {
    rrc_log->error_hex(pdu->buffer, pdu->N_bytes, "Failed to unpack PCCH message (%d B)\n", pdu->N_bytes);
    return;
  }

  log_rrc_message("PCCH", Rx, pdu.get(), pcch_msg);

  if (not ue_identity_configured) {
    rrc_log->warning("Received paging message but no ue-Identity is configured\n");
    return;
  }

  paging_s* paging = &pcch_msg.msg.c1().paging();
  if (paging->paging_record_list.size() > ASN1_RRC_MAX_PAGE_REC) {
    paging->paging_record_list.resize(ASN1_RRC_MAX_PAGE_REC);
  }

  if (not pcch_processor.launch(this, *paging)) {
    rrc_log->error("Failed to launch process PCCH procedure\n");
    return;
  }

  // we do not care about the outcome
  callback_list.defer_proc(pcch_processor);
}

void rrc::write_pdu_mch(uint32_t lcid, srslte::unique_byte_buffer_t pdu)
{
  if (pdu->N_bytes <= 0 or pdu->N_bytes >= SRSLTE_MAX_BUFFER_SIZE_BITS) {
    return;
  }
  // TODO: handle MCCH notifications and update MCCH
  if (0 != lcid or serving_cell->has_mcch) {
    return;
  }
  cmd_msg_t msg;
  msg.command = cmd_msg_t::PDU_MCH;
  msg.pdu     = std::move(pdu);
  msg.lcid    = lcid;
  cmd_q.push(std::move(msg));
}

void rrc::parse_pdu_mch(uint32_t lcid, srslte::unique_byte_buffer_t pdu)
{
  asn1::bit_ref bref(pdu->msg, pdu->N_bytes);
  if (serving_cell->mcch.unpack(bref) != asn1::SRSASN_SUCCESS or
      serving_cell->mcch.msg.type().value != mcch_msg_type_c::types_opts::c1) {
    rrc_log->error("Failed to unpack MCCH message\n");
    return;
  }
  serving_cell->has_mcch = true;
  phy->set_config_mbsfn_mcch(srslte::make_mcch_msg(serving_cell->mcch));
  log_rrc_message("MCH", Rx, pdu.get(), serving_cell->mcch);
  if (args.mbms_service_id >= 0) {
    rrc_log->info("Attempting to auto-start MBMS service %d\n", args.mbms_service_id);
    mbms_service_start(args.mbms_service_id, args.mbms_service_port);
  }
}

/*******************************************************************************
*
*
* Packet processing
*
*
*******************************************************************************/

void rrc::send_ul_ccch_msg(const asn1::rrc::ul_ccch_msg_s& msg)
{
  // Reset and reuse sdu buffer if provided
  unique_byte_buffer_t pdcp_buf = srslte::allocate_unique_buffer(*pool, true);
  if (not pdcp_buf.get()) {
    rrc_log->error("Fatal Error: Couldn't allocate PDU in byte_align_and_pack().\n");
    return;
  }

  asn1::bit_ref bref(pdcp_buf->msg, pdcp_buf->get_tailroom());
  msg.pack(bref);
  bref.align_bytes_zero();
  pdcp_buf->N_bytes = (uint32_t)bref.distance_bytes(pdcp_buf->msg);
  pdcp_buf->set_timestamp();

  // Set UE contention resolution ID in MAC
  uint64_t uecri      = 0;
  uint8_t* ue_cri_ptr = (uint8_t*)&uecri;
  uint32_t nbytes     = 6;
  for (uint32_t i = 0; i < nbytes; i++) {
    ue_cri_ptr[nbytes - i - 1] = pdcp_buf->msg[i];
  }

  rrc_log->debug("Setting UE contention resolution ID: %" PRIu64 "\n", uecri);
  mac->set_contention_id(uecri);

  uint32_t lcid = RB_ID_SRB0;
  log_rrc_message(get_rb_name(lcid).c_str(), Tx, pdcp_buf.get(), msg);

  rlc->write_sdu(lcid, std::move(pdcp_buf));
}

void rrc::send_ul_dcch_msg(uint32_t lcid, const asn1::rrc::ul_dcch_msg_s& msg)
{
  // Reset and reuse sdu buffer if provided
  unique_byte_buffer_t pdcp_buf = srslte::allocate_unique_buffer(*pool, true);
  if (not pdcp_buf.get()) {
    rrc_log->error("Fatal Error: Couldn't allocate PDU in byte_align_and_pack().\n");
    return;
  }

  asn1::bit_ref bref(pdcp_buf->msg, pdcp_buf->get_tailroom());
  msg.pack(bref);
  bref.align_bytes_zero();
  pdcp_buf->N_bytes = (uint32_t)bref.distance_bytes(pdcp_buf->msg);
  pdcp_buf->set_timestamp();

  log_rrc_message(get_rb_name(lcid).c_str(), Tx, pdcp_buf.get(), msg);

  pdcp->write_sdu(lcid, std::move(pdcp_buf));
}

void rrc::write_sdu(srslte::unique_byte_buffer_t sdu)
{
  if (state == RRC_STATE_IDLE) {
    rrc_log->warning("Received ULInformationTransfer SDU when in IDLE\n");
    return;
  }
  send_ul_info_transfer(std::move(sdu));
}

void rrc::write_pdu(uint32_t lcid, unique_byte_buffer_t pdu)
{
  // add PDU to command queue
  cmd_msg_t msg;
  msg.pdu     = std::move(pdu);
  msg.command = cmd_msg_t::PDU;
  msg.lcid    = (uint16_t)lcid;
  cmd_q.push(std::move(msg));
}

void rrc::process_pdu(uint32_t lcid, srslte::unique_byte_buffer_t pdu)
{
  switch (lcid) {
    case RB_ID_SRB0:
      parse_dl_ccch(std::move(pdu));
      break;
    case RB_ID_SRB1:
    case RB_ID_SRB2:
      parse_dl_dcch(lcid, std::move(pdu));
      break;
    default:
      rrc_log->error("RX PDU with invalid bearer id: %d", lcid);
      break;
  }
}

void rrc::parse_dl_ccch(unique_byte_buffer_t pdu)
{
  asn1::bit_ref bref(pdu->msg, pdu->N_bytes);
  asn1::rrc::dl_ccch_msg_s dl_ccch_msg;
  if (dl_ccch_msg.unpack(bref) != asn1::SRSASN_SUCCESS or
      dl_ccch_msg.msg.type().value != dl_ccch_msg_type_c::types_opts::c1) {
    rrc_log->error("Failed to unpack DL-CCCH message\n");
    return;
  }
  log_rrc_message(get_rb_name(RB_ID_SRB0).c_str(), Rx, pdu.get(), dl_ccch_msg);

  dl_ccch_msg_type_c::c1_c_* c1 = &dl_ccch_msg.msg.c1();
  switch (dl_ccch_msg.msg.c1().type().value) {
    case dl_ccch_msg_type_c::c1_c_::types::rrc_conn_reject: {
      // 5.3.3.8
      rrc_conn_reject_r8_ies_s* reject_r8 = &c1->rrc_conn_reject().crit_exts.c1().rrc_conn_reject_r8();
      rrc_log->info("Received ConnectionReject. Wait time: %d\n", reject_r8->wait_time);
      rrc_log->console("Received ConnectionReject. Wait time: %d\n", reject_r8->wait_time);

      timers->get(t300)->stop();

      if (reject_r8->wait_time) {
        nas->set_barring(nas_interface_rrc::BARRING_ALL);
        timers->get(t302)->set(this, reject_r8->wait_time * 1000u);
        timers->get(t302)->run();
      } else {
        // Perform the actions upon expiry of T302 if wait time is zero
        nas->set_barring(nas_interface_rrc::BARRING_NONE);
        start_go_idle();
      }
    } break;
    case dl_ccch_msg_type_c::c1_c_::types::rrc_conn_setup:
      transaction_id = c1->rrc_conn_setup().rrc_transaction_id;
      handle_con_setup(&c1->rrc_conn_setup());
      break;
    case dl_ccch_msg_type_c::c1_c_::types::rrc_conn_reest:
      rrc_log->console("Reestablishment OK\n");
      transaction_id = c1->rrc_conn_reest().rrc_transaction_id;
      handle_con_reest(&c1->rrc_conn_reest());
      break;
      /* Reception of RRCConnectionReestablishmentReject 5.3.7.8 */
    case dl_ccch_msg_type_c::c1_c_::types::rrc_conn_reest_reject:
      rrc_log->console("Reestablishment Reject\n");
      start_go_idle();
      break;
    default:
      rrc_log->error("The provided DL-CCCH message type is not recognized\n");
      break;
  }
}

void rrc::parse_dl_dcch(uint32_t lcid, unique_byte_buffer_t pdu)
{
  asn1::bit_ref bref(pdu->msg, pdu->N_bytes);
  asn1::rrc::dl_dcch_msg_s dl_dcch_msg;
  if (dl_dcch_msg.unpack(bref) != asn1::SRSASN_SUCCESS or
      dl_dcch_msg.msg.type().value != dl_dcch_msg_type_c::types_opts::c1) {
    rrc_log->error("Failed to unpack DL-DCCH message\n");
    return;
  }
  log_rrc_message(get_rb_name(lcid).c_str(), Rx, pdu.get(), dl_dcch_msg);

  dl_dcch_msg_type_c::c1_c_* c1 = &dl_dcch_msg.msg.c1();
  switch (dl_dcch_msg.msg.c1().type().value) {
    case dl_dcch_msg_type_c::c1_c_::types::dl_info_transfer:
      pdu = srslte::allocate_unique_buffer(*pool, true);
      if (!pdu.get()) {
        rrc_log->error("Fatal error: out of buffers in pool\n");
        return;
      }
      pdu->N_bytes = c1->dl_info_transfer().crit_exts.c1().dl_info_transfer_r8().ded_info_type.ded_info_nas().size();
      memcpy(pdu->msg, c1->dl_info_transfer().crit_exts.c1().dl_info_transfer_r8().ded_info_type.ded_info_nas().data(),
             pdu->N_bytes);
      nas->write_pdu(lcid, std::move(pdu));
      break;
    case dl_dcch_msg_type_c::c1_c_::types::security_mode_cmd:
      transaction_id = c1->security_mode_cmd().rrc_transaction_id;

      cipher_algo = (CIPHERING_ALGORITHM_ID_ENUM)c1->security_mode_cmd()
                        .crit_exts.c1()
                        .security_mode_cmd_r8()
                        .security_cfg_smc.security_algorithm_cfg.ciphering_algorithm.value;
      integ_algo = (INTEGRITY_ALGORITHM_ID_ENUM)c1->security_mode_cmd()
                       .crit_exts.c1()
                       .security_mode_cmd_r8()
                       .security_cfg_smc.security_algorithm_cfg.integrity_prot_algorithm.value;

      rrc_log->info("Received Security Mode Command eea: %s, eia: %s\n", ciphering_algorithm_id_text[cipher_algo],
                    integrity_algorithm_id_text[integ_algo]);

      // Generate AS security keys
      uint8_t k_asme[32];
      nas->get_k_asme(k_asme, 32);
      rrc_log->debug_hex(k_asme, 32, "UE K_asme");
      rrc_log->debug("Generating K_enb with UL NAS COUNT: %d\n", nas->get_k_enb_count());
      usim->generate_as_keys(k_asme, nas->get_k_enb_count(), k_rrc_enc, k_rrc_int, k_up_enc, k_up_int, cipher_algo,
                             integ_algo);
      rrc_log->info_hex(k_rrc_enc, 32, "RRC encryption key - k_rrc_enc");
      rrc_log->info_hex(k_rrc_int, 32, "RRC integrity key  - k_rrc_int");
      rrc_log->info_hex(k_up_enc, 32, "UP encryption key  - k_up_enc");

      security_is_activated = true;

      // Configure PDCP for security
      pdcp->config_security(lcid, k_rrc_enc, k_rrc_int, k_up_enc, cipher_algo, integ_algo);
      pdcp->enable_integrity(lcid);
      send_security_mode_complete();
      pdcp->enable_encryption(lcid);
      break;
    case dl_dcch_msg_type_c::c1_c_::types::rrc_conn_recfg:
      transaction_id = c1->rrc_conn_recfg().rrc_transaction_id;
      handle_rrc_con_reconfig(lcid, &c1->rrc_conn_recfg());
      break;
    case dl_dcch_msg_type_c::c1_c_::types::ue_cap_enquiry:
      transaction_id = c1->ue_cap_enquiry().rrc_transaction_id;
      handle_ue_capability_enquiry(c1->ue_cap_enquiry());
      break;
    case dl_dcch_msg_type_c::c1_c_::types::rrc_conn_release:
      rrc_connection_release();
      break;
    default:
      rrc_log->error("The provided DL-CCCH message type is not recognized or supported\n");
      break;
  }
}

/*******************************************************************************
*
*
*
* Capabilities Message
*
*
*
*******************************************************************************/
void rrc::enable_capabilities()
{
  bool enable_ul_64 =
      args.ue_category >= 5 && serving_cell->sib2ptr()->rr_cfg_common.pusch_cfg_common.pusch_cfg_basic.enable64_qam;
  rrc_log->info("%s 64QAM PUSCH\n", enable_ul_64 ? "Enabling" : "Disabling");
}

void rrc::handle_ue_capability_enquiry(const asn1::rrc::ue_cap_enquiry_s& enquiry)
{
  rrc_log->debug("Preparing UE Capability Info\n");

  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ue_cap_info_r8_ies_s*    info = &ul_dcch_msg.msg.set_c1().set_ue_cap_info().crit_exts.set_c1().set_ue_cap_info_r8();
  ul_dcch_msg.msg.c1().ue_cap_info().rrc_transaction_id = transaction_id;

  // resize container to fit all requested RATs
  info->ue_cap_rat_container_list.resize(enquiry.crit_exts.c1().ue_cap_enquiry_r8().ue_cap_request.size());
  uint32_t rat_idx = 0;

  for (uint32_t i = 0; i < enquiry.crit_exts.c1().ue_cap_enquiry_r8().ue_cap_request.size(); i++) {
    if (enquiry.crit_exts.c1().ue_cap_enquiry_r8().ue_cap_request[i] == rat_type_e::eutra) {
      // adding EUTRA caps
      info->ue_cap_rat_container_list[0].rat_type = rat_type_e::eutra;

      // Check UE config arguments bounds
      if (args.release < SRSLTE_RELEASE_MIN || args.release > SRSLTE_RELEASE_MAX) {
        uint32_t new_release = SRSLTE_MIN(SRSLTE_RELEASE_MAX, SRSLTE_MAX(SRSLTE_RELEASE_MIN, args.release));
        rrc_log->error("Release is %d. It is out of bounds (%d ... %d), setting it to %d\n",
                       args.release,
                       SRSLTE_RELEASE_MIN,
                       SRSLTE_RELEASE_MAX,
                       new_release);
        args.release = new_release;
      }

      args.ue_category = (uint32_t)strtol(args.ue_category_str.c_str(), nullptr, 10);
      if (args.ue_category < SRSLTE_UE_CATEGORY_MIN || args.ue_category > SRSLTE_UE_CATEGORY_MAX) {
        uint32_t new_category =
            SRSLTE_MIN(SRSLTE_UE_CATEGORY_MAX, SRSLTE_MAX(SRSLTE_UE_CATEGORY_MIN, args.ue_category));
        rrc_log->error("UE Category is %d. It is out of bounds (%d ... %d), setting it to %d\n",
                       args.ue_category,
                       SRSLTE_UE_CATEGORY_MIN,
                       SRSLTE_UE_CATEGORY_MAX,
                       new_category);
        args.ue_category = new_category;
      }

      ue_eutra_cap_s cap;
      cap.access_stratum_release = (access_stratum_release_e::options)(args.release - SRSLTE_RELEASE_MIN);
      cap.ue_category            = (uint8_t)((args.ue_category < 1 || args.ue_category > 5) ? 4 : args.ue_category);
      cap.pdcp_params.max_num_rohc_context_sessions_present     = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0001_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0002_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0003_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0004_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0006_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0101_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0102_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0103_r15 = false;
      cap.pdcp_params.supported_rohc_profiles.profile0x0104_r15 = false;

      cap.phy_layer_params.ue_specific_ref_sigs_supported = false;
      cap.phy_layer_params.ue_tx_ant_sel_supported        = false;

      cap.rf_params.supported_band_list_eutra.resize(args.nof_supported_bands);
      cap.meas_params.band_list_eutra.resize(args.nof_supported_bands);
      for (uint32_t k = 0; k < args.nof_supported_bands; k++) {
        cap.rf_params.supported_band_list_eutra[k].band_eutra  = args.supported_bands[k];
        cap.rf_params.supported_band_list_eutra[k].half_duplex = false;
        cap.meas_params.band_list_eutra[k].inter_freq_band_list.resize(1);
        cap.meas_params.band_list_eutra[k].inter_freq_band_list[0].inter_freq_need_for_gaps = true;
      }

      cap.feature_group_inds_present = true;
      cap.feature_group_inds.from_number(args.feature_group);

      if (args.release > 8) {
        ue_eutra_cap_v920_ies_s cap_v920;

        cap_v920.phy_layer_params_v920.enhanced_dual_layer_fdd_r9_present                        = false;
        cap_v920.phy_layer_params_v920.enhanced_dual_layer_tdd_r9_present                        = false;
        cap_v920.inter_rat_params_geran_v920.dtm_r9_present                                      = false;
        cap_v920.inter_rat_params_geran_v920.e_redirection_geran_r9_present                      = false;
        cap_v920.csg_proximity_ind_params_r9.inter_freq_proximity_ind_r9_present                 = false;
        cap_v920.csg_proximity_ind_params_r9.intra_freq_proximity_ind_r9_present                 = false;
        cap_v920.csg_proximity_ind_params_r9.utran_proximity_ind_r9_present                      = false;
        cap_v920.neigh_cell_si_acquisition_params_r9.inter_freq_si_acquisition_for_ho_r9_present = false;
        cap_v920.neigh_cell_si_acquisition_params_r9.intra_freq_si_acquisition_for_ho_r9_present = false;
        cap_v920.neigh_cell_si_acquisition_params_r9.utran_si_acquisition_for_ho_r9_present      = false;
        cap_v920.son_params_r9.rach_report_r9_present                                            = false;

        cap.non_crit_ext_present = true;
        cap.non_crit_ext         = cap_v920;
      }

      if (args.release > 9) {

        phy_layer_params_v1020_s phy_layer_params_v1020;
        phy_layer_params_v1020.two_ant_ports_for_pucch_r10_present             = false;
        phy_layer_params_v1020.tm9_with_minus8_tx_fdd_r10_present              = false;
        phy_layer_params_v1020.pmi_disabling_r10_present                       = false;
        phy_layer_params_v1020.cross_carrier_sched_r10_present                 = args.support_ca;
        phy_layer_params_v1020.simul_pucch_pusch_r10_present                   = false;
        phy_layer_params_v1020.multi_cluster_pusch_within_cc_r10_present       = false;
        phy_layer_params_v1020.non_contiguous_ul_ra_within_cc_list_r10_present = false;

        band_combination_params_r10_l combination_params;
        if (args.support_ca) {
          for (uint32_t k = 0; k < args.nof_supported_bands; k++) {
            ca_mimo_params_dl_r10_s ca_mimo_params_dl;
            ca_mimo_params_dl.ca_bw_class_dl_r10                = ca_bw_class_r10_e::f;
            ca_mimo_params_dl.supported_mimo_cap_dl_r10_present = false;

            ca_mimo_params_ul_r10_s ca_mimo_params_ul;
            ca_mimo_params_ul.ca_bw_class_ul_r10                = ca_bw_class_r10_e::f;
            ca_mimo_params_ul.supported_mimo_cap_ul_r10_present = false;

            band_params_r10_s band_params;
            band_params.band_eutra_r10             = args.supported_bands[i];
            band_params.band_params_dl_r10_present = true;
            band_params.band_params_dl_r10.push_back(ca_mimo_params_dl);
            band_params.band_params_ul_r10_present = true;
            band_params.band_params_ul_r10.push_back(ca_mimo_params_ul);

            combination_params.push_back(band_params);
          }
        }

        rf_params_v1020_s rf_params;
        rf_params.supported_band_combination_r10.push_back(combination_params);

        ue_eutra_cap_v1020_ies_s cap_v1020;
        if (args.ue_category >= 6 && args.ue_category <= 8) {
          cap_v1020.ue_category_v1020_present = true;
          cap_v1020.ue_category_v1020         = (uint8_t)args.ue_category;
        } else {
          // Do not populate UE category for this release if the category is out of range
        }
        cap_v1020.phy_layer_params_v1020_present = true;
        cap_v1020.phy_layer_params_v1020         = phy_layer_params_v1020;
        cap_v1020.rf_params_v1020_present        = args.support_ca;
        cap_v1020.rf_params_v1020                = rf_params;

        ue_eutra_cap_v940_ies_s cap_v940;
        cap_v940.non_crit_ext_present = true;
        cap_v940.non_crit_ext         = cap_v1020;

        cap.non_crit_ext.non_crit_ext_present = true;
        cap.non_crit_ext.non_crit_ext         = cap_v940;
      }

      if (args.release > 10) {
        ue_eutra_cap_v11a0_ies_s cap_v11a0;
        if (args.ue_category >= 11 && args.ue_category <= 12) {
          cap_v11a0.ue_category_v11a0         = (uint8_t)args.ue_category;
          cap_v11a0.ue_category_v11a0_present = true;
        } else {
          // Do not populate UE category for this release if the category is out of range
        }

        ue_eutra_cap_v1180_ies_s cap_v1180;
        cap_v1180.non_crit_ext_present = true;
        cap_v1180.non_crit_ext         = cap_v11a0;

        ue_eutra_cap_v1170_ies_s cap_v1170;
        cap_v1170.non_crit_ext_present = true;
        cap_v1170.non_crit_ext         = cap_v1180;
        if (args.ue_category >= 9 && args.ue_category <= 10) {
          cap_v1170.ue_category_v1170         = (uint8_t)args.ue_category;
          cap_v1170.ue_category_v1170_present = true;
        } else {
          // Do not populate UE category for this release if the category is out of range
        }

        ue_eutra_cap_v1130_ies_s cap_v1130;
        cap_v1130.non_crit_ext_present = true;
        cap_v1130.non_crit_ext         = cap_v1170;

        ue_eutra_cap_v1090_ies_s cap_v1090;
        cap_v1090.non_crit_ext_present = true;
        cap_v1090.non_crit_ext         = cap_v1130;

        ue_eutra_cap_v1060_ies_s cap_v1060;
        cap_v1060.non_crit_ext_present = true;
        cap_v1060.non_crit_ext         = cap_v1090;

        cap.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext_present = true;
        cap.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext         = cap_v1060;
      }

      if (args.release > 11) {
        supported_band_list_eutra_v1250_l supported_band_list_eutra_v1250;
        for (uint32_t k = 0; k < args.nof_supported_bands; k++) {
          supported_band_eutra_v1250_s supported_band_eutra_v1250;
          // According to 3GPP 36.306 v12 Table 4.1A-1, 256QAM is supported for ue_category_dl 11-16
          supported_band_eutra_v1250.dl_minus256_qam_r12_present = (args.ue_category_dl >= 11);

          // According to 3GPP 36.331 v12 UE-EUTRA-Capability field descriptions
          // This field is only present when the field ue-CategoryUL is considered to 5 or 13.
          supported_band_eutra_v1250.ul_minus64_qam_r12_present = true;

          supported_band_list_eutra_v1250.push_back(supported_band_eutra_v1250);
        }

        rf_params_v1250_s rf_params_v1250;
        rf_params_v1250.supported_band_list_eutra_v1250_present = true;
        rf_params_v1250.supported_band_list_eutra_v1250         = supported_band_list_eutra_v1250;

        ue_eutra_cap_v1250_ies_s cap_v1250;

        // Optional UE Category UL/DL
        // Warning: Make sure the UE Category UL/DL matches with 3GPP 36.306 Table 4.1A-6
        if (args.ue_category_dl >= 0) {
          cap_v1250.ue_category_dl_r12_present = true;
          cap_v1250.ue_category_dl_r12         = (uint8_t)args.ue_category_dl;
        } else {
          // Do not populate UE category for this release if the category is not available
        }
        if (args.ue_category_ul >= 0) {
          cap_v1250.ue_category_ul_r12_present = true;
          cap_v1250.ue_category_ul_r12         = (uint8_t)args.ue_category_ul;
        } else {
          // Do not populate UE category for this release if the category is not available
        }
        cap_v1250.rf_params_v1250_present = true;
        cap_v1250.rf_params_v1250         = rf_params_v1250;

        cap.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext
            .non_crit_ext.non_crit_ext_present = true;
        cap.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext.non_crit_ext
            .non_crit_ext.non_crit_ext = cap_v1250;
      }

      // Pack caps and copy to cap info
      uint8_t       buf[64] = {};
      asn1::bit_ref bref(buf, sizeof(buf));
      cap.pack(bref);
      bref.align_bytes_zero();
      auto cap_len = (uint32_t)bref.distance_bytes(buf);
      info->ue_cap_rat_container_list[rat_idx].ue_cap_rat_container.resize(cap_len);
      memcpy(info->ue_cap_rat_container_list[rat_idx].ue_cap_rat_container.data(), buf, cap_len);
      rat_idx++;
    }
  }

  // resize container back to the actually filled items
  info->ue_cap_rat_container_list.resize(rat_idx);

  send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

/*******************************************************************************
 *
 *
 *
 * PHY and MAC Radio Resource configuration
 *
 *
 *
 *******************************************************************************/

void rrc::log_rr_config_common()
{
  rrc_log->info("Set RACH ConfigCommon: NofPreambles=%d, ResponseWindow=%d, ContentionResolutionTimer=%d ms\n",
                current_mac_cfg.rach_cfg.nof_preambles,
                current_mac_cfg.rach_cfg.responseWindowSize,
                current_mac_cfg.rach_cfg.contentionResolutionTimer);

  rrc_log->info("Set PUSCH ConfigCommon: P0_pusch=%f, DMRS cs=%d, delta_ss=%d, N_sb=%d\n",
                current_phy_cfg.ul_cfg.power_ctrl.p0_ue_pusch,
                current_phy_cfg.ul_cfg.dmrs.cyclic_shift,
                current_phy_cfg.ul_cfg.dmrs.delta_ss,
                current_phy_cfg.ul_cfg.hopping.n_sb);

  rrc_log->info("Set PUCCH ConfigCommon: DeltaShift=%d, CyclicShift=%d, N1=%d, NRB=%d\n",
                current_phy_cfg.ul_cfg.pucch.delta_pucch_shift,
                current_phy_cfg.ul_cfg.pucch.N_cs,
                current_phy_cfg.ul_cfg.pucch.n1_pucch_an_cs[0][0],
                current_phy_cfg.ul_cfg.pucch.n_rb_2);

  rrc_log->info("Set PRACH ConfigCommon: SeqIdx=%d, HS=%s, FreqOffset=%d, ZC=%d, ConfigIndex=%d\n",
                current_phy_cfg.prach_cfg.root_seq_idx,
                current_phy_cfg.prach_cfg.hs_flag ? "yes" : "no",
                current_phy_cfg.prach_cfg.freq_offset,
                current_phy_cfg.prach_cfg.zero_corr_zone,
                current_phy_cfg.prach_cfg.config_idx);

  if (current_phy_cfg.ul_cfg.srs.configured) {
    rrc_log->info("Set SRS ConfigCommon: BW-Configuration=%d, SF-Configuration=%d, Simult-ACKNACK=%s\n",
                  current_phy_cfg.ul_cfg.srs.bw_cfg,
                  current_phy_cfg.ul_cfg.srs.subframe_config,
                  current_phy_cfg.ul_cfg.srs.simul_ack ? "yes" : "no");
  }
}

void rrc::apply_rr_config_common(rr_cfg_common_s* config, bool send_lower_layers)
{
  if (config->rach_cfg_common_present) {
    set_mac_cfg_t_rach_cfg_common(&current_mac_cfg, config->rach_cfg_common);
  }

  if (config->prach_cfg.prach_cfg_info_present) {
    set_phy_cfg_t_common_prach(&current_phy_cfg, &config->prach_cfg.prach_cfg_info, config->prach_cfg.root_seq_idx);
  } else {
    set_phy_cfg_t_common_prach(&current_phy_cfg, NULL, config->prach_cfg.root_seq_idx);
  }

  if (config->pdsch_cfg_common_present) {
    set_phy_cfg_t_common_pdsch(&current_phy_cfg, config->pdsch_cfg_common);
  }

  set_phy_cfg_t_common_pusch(&current_phy_cfg, config->pusch_cfg_common);

  if (config->phich_cfg_present) {
    // TODO
  }

  if (config->pucch_cfg_common_present) {
    set_phy_cfg_t_common_pucch(&current_phy_cfg, config->pucch_cfg_common);
  }

  if (config->srs_ul_cfg_common_present) {
    set_phy_cfg_t_common_srs(&current_phy_cfg, config->srs_ul_cfg_common);
  }

  if (config->ul_pwr_ctrl_common_present) {
    set_phy_cfg_t_common_pwr_ctrl(&current_phy_cfg, config->ul_pwr_ctrl_common);
  }

  log_rr_config_common();

  if (send_lower_layers) {
    mac->set_config(current_mac_cfg);
    phy->set_config(current_phy_cfg);
  }
}

void rrc::log_phy_config_dedicated()
{
  if (!rrc_log) {
    return;
  }

  if (current_phy_cfg.dl_cfg.cqi_report.periodic_configured) {
    rrc_log->info("Set cqi-PUCCH-ResourceIndex=%d, cqi-pmi-ConfigIndex=%d, cqi-FormatIndicatorPeriodic=%d\n",
                  current_phy_cfg.ul_cfg.pucch.n_pucch_2,
                  current_phy_cfg.dl_cfg.cqi_report.pmi_idx,
                  current_phy_cfg.dl_cfg.cqi_report.periodic_mode);
  }
  if (current_phy_cfg.dl_cfg.cqi_report.aperiodic_configured) {
    rrc_log->info("Set cqi-ReportModeAperiodic=%d\n", current_phy_cfg.dl_cfg.cqi_report.aperiodic_mode);
  }

  if (current_phy_cfg.ul_cfg.pucch.sr_configured) {
    rrc_log->info("Set PHY config ded: SR-n_pucch=%d, SR-ConfigIndex=%d, SR-TransMax=%d\n",
                  current_phy_cfg.ul_cfg.pucch.n_pucch_sr,
                  current_phy_cfg.ul_cfg.pucch.I_sr,
                  current_mac_cfg.sr_cfg.dsr_transmax);
  }

  if (current_phy_cfg.ul_cfg.srs.configured) {
    rrc_log->info("Set PHY config ded: SRS-ConfigIndex=%d, SRS-bw=%d, SRS-Nrcc=%d, SRS-hop=%d, SRS-Ncs=%d\n",
                  current_phy_cfg.ul_cfg.srs.I_srs,
                  current_phy_cfg.ul_cfg.srs.B,
                  current_phy_cfg.ul_cfg.srs.n_rrc,
                  current_phy_cfg.ul_cfg.srs.b_hop,
                  current_phy_cfg.ul_cfg.srs.n_srs);
  }
}

// Apply default physical common and dedicated configuration
void rrc::set_phy_default()
{
  current_phy_cfg.set_defaults();

  if (phy != nullptr) {
    phy->set_config(current_phy_cfg);
  } else {
    rrc_log->info("RRC not initialized. Skipping default PHY config.\n");
  }
}

// Apply default physical channel configs (9.2.4)
void rrc::set_phy_config_dedicated_default()
{
  current_phy_cfg.set_defaults_dedicated();

  if (phy != nullptr) {
    phy->set_config(current_phy_cfg);
  } else {
    rrc_log->info("RRC not initialized. Skipping default PHY config.\n");
  }
}

// Apply provided PHY config
void rrc::apply_phy_config_dedicated(const phys_cfg_ded_s& phy_cnfg)
{
  set_phy_cfg_t_dedicated_cfg(&current_phy_cfg, phy_cnfg);

  log_phy_config_dedicated();

  if (phy != nullptr) {
    phy->set_config(current_phy_cfg);
  } else {
    rrc_log->info("RRC not initialized. Skipping PHY config.\n");
  }
}

void rrc::apply_phy_scell_config(const asn1::rrc::scell_to_add_mod_r10_s& scell_config)
{
  srslte_cell_t scell  = {};
  uint32_t      earfcn = 0;

  if (phy == nullptr) {
    rrc_log->info("RRC not initialized. Skipping PHY config.\n");
    return;
  }

  // Initialise default parameters from primary cell
  phy->get_current_cell(&scell, &earfcn);

  // Parse identification
  if (scell_config.cell_identif_r10_present) {
    scell.id = scell_config.cell_identif_r10.pci_r10;
    earfcn   = scell_config.cell_identif_r10.dl_carrier_freq_r10;
  }

  // Parse radio resource
  if (scell_config.rr_cfg_common_scell_r10_present) {
    const rr_cfg_common_scell_r10_s* rr_cfg     = &scell_config.rr_cfg_common_scell_r10;
    auto                             non_ul_cfg = &rr_cfg->non_ul_cfg_r10;
    scell.frame_type                            = (rr_cfg->tdd_cfg_v1130.is_present()) ? SRSLTE_TDD : SRSLTE_FDD;
    scell.nof_prb                               = non_ul_cfg->dl_bw_r10.to_number();
    scell.nof_ports                             = non_ul_cfg->ant_info_common_r10.ant_ports_count.to_number();
    scell.phich_length = (non_ul_cfg->phich_cfg_r10.phich_dur.value == phich_cfg_s::phich_dur_opts::normal)
                             ? SRSLTE_PHICH_NORM
                             : SRSLTE_PHICH_EXT;

    // Avoid direct conversion between different phich resource enum
    switch (non_ul_cfg->phich_cfg_r10.phich_res.value) {
      case phich_cfg_s::phich_res_opts::one_sixth:
        scell.phich_resources = SRSLTE_PHICH_R_1_6;
        break;
      case phich_cfg_s::phich_res_opts::half:
        scell.phich_resources = SRSLTE_PHICH_R_1_2;
        break;
      case phich_cfg_s::phich_res_opts::one:
        scell.phich_resources = SRSLTE_PHICH_R_1;
        break;
      case phich_cfg_s::phich_res_opts::two:
      case phich_cfg_s::phich_res_opts::nulltype:
        scell.phich_resources = SRSLTE_PHICH_R_2;
        break;
    }
  }

  // Initialize scell config with pcell cfg
  srslte::phy_cfg_t scell_phy_cfg = current_phy_cfg;
  set_phy_cfg_t_scell_config(&scell_phy_cfg, scell_config);

  phy->set_config(scell_phy_cfg, scell_config.s_cell_idx_r10, earfcn, &scell);
}

void rrc::log_mac_config_dedicated()
{
  rrc_log->info("Set MAC main config: harq-MaxReTX=%d, bsr-TimerReTX=%d, bsr-TimerPeriodic=%d\n",
                current_mac_cfg.harq_cfg.max_harq_msg3_tx,
                current_mac_cfg.bsr_cfg.retx_timer,
                current_mac_cfg.bsr_cfg.periodic_timer);
  if (current_mac_cfg.phr_cfg.enabled) {
    rrc_log->info("Set MAC PHR config: periodicPHR-Timer=%d, prohibitPHR-Timer=%d, dl-PathlossChange=%d\n",
                  current_mac_cfg.phr_cfg.periodic_timer,
                  current_mac_cfg.phr_cfg.prohibit_timer,
                  current_mac_cfg.phr_cfg.db_pathloss_change);
  }
}

// 3GPP 36.331 v10 9.2.2 Default MAC main configuration
void rrc::apply_mac_config_dedicated_default()
{
  rrc_log->info("Set MAC default configuration\n");
  current_mac_cfg.set_mac_main_cfg_default();
  mac->set_config(current_mac_cfg);
  log_mac_config_dedicated();
}

bool rrc::apply_rr_config_dedicated(rr_cfg_ded_s* cnfg)
{
  if (cnfg->phys_cfg_ded_present) {
    apply_phy_config_dedicated(cnfg->phys_cfg_ded);
    // Apply SR configuration to MAC
    if (cnfg->phys_cfg_ded.sched_request_cfg_present) {
      set_mac_cfg_t_sched_request_cfg(&current_mac_cfg, cnfg->phys_cfg_ded.sched_request_cfg);
    }
  }

  if (cnfg->mac_main_cfg_present) {
    if (cnfg->mac_main_cfg.type() == rr_cfg_ded_s::mac_main_cfg_c_::types::default_value) {
      current_mac_cfg.set_mac_main_cfg_default();
    } else {
      set_mac_cfg_t_main_cfg(&current_mac_cfg, cnfg->mac_main_cfg.explicit_value());
    }
    mac->set_config(current_mac_cfg);
  } else if (cnfg->phys_cfg_ded.sched_request_cfg_present) {
    // If MAC-main not set but SR config is set, use directly mac->set_config to update config
    mac->set_config(current_mac_cfg);
    log_mac_config_dedicated();
  }

  if (cnfg->sps_cfg_present) {
    // TODO
  }
  if (cnfg->rlf_timers_and_consts_r9.is_present() and cnfg->rlf_timers_and_consts_r9->type() == setup_e::setup) {
    timers->get(t301)->set(this, cnfg->rlf_timers_and_consts_r9->setup().t301_r9.to_number());
    timers->get(t310)->set(this, cnfg->rlf_timers_and_consts_r9->setup().t310_r9.to_number());
    timers->get(t311)->set(this, cnfg->rlf_timers_and_consts_r9->setup().t311_r9.to_number());
    N310 = cnfg->rlf_timers_and_consts_r9->setup().n310_r9.to_number();
    N311 = cnfg->rlf_timers_and_consts_r9->setup().n311_r9.to_number();

    rrc_log->info("Updated Constants and Timers: N310=%d, N311=%d, t300=%u, t301=%u, t310=%u, t311=%u\n",
                  N310,
                  N311,
                  timers->get(t300)->get_timeout(),
                  timers->get(t301)->get_timeout(),
                  timers->get(t310)->get_timeout(),
                  timers->get(t311)->get_timeout());
  }
  for (uint32_t i = 0; i < cnfg->srb_to_add_mod_list.size(); i++) {
    // TODO: handle SRB modification
    add_srb(&cnfg->srb_to_add_mod_list[i]);
  }
  for (uint32_t i = 0; i < cnfg->drb_to_release_list.size(); i++) {
    release_drb(cnfg->drb_to_release_list[i]);
  }
  for (uint32_t i = 0; i < cnfg->drb_to_add_mod_list.size(); i++) {
    // TODO: handle DRB modification
    add_drb(&cnfg->drb_to_add_mod_list[i]);
  }
  return true;
}

/*
 * Extracts and applies SCell configuration from an ASN.1 reconfiguration struct
 */
void rrc::apply_scell_config(asn1::rrc::rrc_conn_recfg_r8_ies_s* reconfig_r8)
{
  if (reconfig_r8->non_crit_ext_present) {
    auto reconfig_r890 = &reconfig_r8->non_crit_ext;
    if (reconfig_r890->non_crit_ext_present) {
      rrc_conn_recfg_v920_ies_s* reconfig_r920 = &reconfig_r890->non_crit_ext;
      if (reconfig_r920->non_crit_ext_present) {
        rrc_conn_recfg_v1020_ies_s* reconfig_r1020 = &reconfig_r920->non_crit_ext;

        // Handle Add/Modify SCell list
        if (reconfig_r1020->s_cell_to_add_mod_list_r10_present) {
          for (uint32_t i = 0; i < reconfig_r1020->s_cell_to_add_mod_list_r10.size(); i++) {
            auto scell_config = &reconfig_r1020->s_cell_to_add_mod_list_r10[i];

            // Limit enable64_qam, if the ue does not
            // since the phy does not have information about the RRC category and release, the RRC shall limit the
            if (scell_config->rr_cfg_common_scell_r10_present) {
              // enable64_qam
              auto rr_cfg_common_scell = &scell_config->rr_cfg_common_scell_r10;
              if (rr_cfg_common_scell->ul_cfg_r10_present) {
                auto ul_cfg           = &rr_cfg_common_scell->ul_cfg_r10;
                auto pusch_cfg_common = &ul_cfg->pusch_cfg_common_r10;

                // According to 3GPP 36.331 v12 UE-EUTRA-Capability field descriptions
                // Allow 64QAM for:
                //   ue-Category 5 and 8 when enable64QAM (without suffix)
                if (pusch_cfg_common->pusch_cfg_basic.enable64_qam) {
                  if (args.ue_category != 5 && args.ue_category != 8 && args.ue_category != 13) {
                    pusch_cfg_common->pusch_cfg_basic.enable64_qam = false;
                  }
                }
              }
            }

            // Call mac reconfiguration
            mac->reconfiguration(scell_config->s_cell_idx_r10, true);

            // Call phy reconfiguration
            apply_phy_scell_config(*scell_config);
          }
        }

        // Handle Remove SCell list
        if (reconfig_r1020->s_cell_to_release_list_r10_present) {
          for (uint32_t i = 0; i < reconfig_r1020->s_cell_to_release_list_r10.size(); i++) {
            // Call mac reconfiguration
            mac->reconfiguration(reconfig_r1020->s_cell_to_release_list_r10[i], false);

            // Call phy reconfiguration
            // TODO: Implement phy layer cell removal
          }
        }
      }
    }
  }
}

void rrc::handle_con_setup(rrc_conn_setup_s* setup)
{
  // Must enter CONNECT before stopping T300
  state = RRC_STATE_CONNECTED;
  timers->get(t300)->stop();
  timers->get(t302)->stop();
  rrc_log->console("RRC Connected\n");

  // Apply the Radio Resource configuration
  apply_rr_config_dedicated(&setup->crit_exts.c1().rrc_conn_setup_r8().rr_cfg_ded);

  nas->set_barring(nas_interface_rrc::BARRING_NONE);

  if (dedicated_info_nas.get()) {
    send_con_setup_complete(std::move(dedicated_info_nas));
  } else {
    rrc_log->error("Pending to transmit a ConnectionSetupComplete but no dedicatedInfoNAS was in queue\n");
  }
}

/* Reception of RRCConnectionReestablishment by the UE 5.3.7.5 */
void rrc::handle_con_reest(rrc_conn_reest_s* setup)
{

  timers->get(t301)->stop();

  // Reestablish PDCP and RLC for SRB1
  pdcp->reestablish(1);
  rlc->reestablish(1);

  // Update RRC Integrity keys
  int ncc = setup->crit_exts.c1().rrc_conn_reest_r8().next_hop_chaining_count;
  usim->generate_as_keys_ho(serving_cell->get_pci(),
                            phy->get_current_earfcn(),
                            ncc,
                            k_rrc_enc,
                            k_rrc_int,
                            k_up_enc,
                            k_up_int,
                            cipher_algo,
                            integ_algo);
  pdcp->config_security_all(k_rrc_enc, k_rrc_int, k_up_enc, cipher_algo, integ_algo);

  // Apply the Radio Resource configuration
  apply_rr_config_dedicated(&setup->crit_exts.c1().rrc_conn_reest_r8().rr_cfg_ded);

  // Resume SRB1
  rlc->resume_bearer(1);

  // Send ConnectionSetupComplete message
  send_con_restablish_complete();

  reestablishment_successful = true;
}

void rrc::add_srb(srb_to_add_mod_s* srb_cnfg)
{
  // Setup PDCP
  pdcp_config_t pdcp_cfg = {.bearer_id    = srb_cnfg->srb_id,
                            .rb_type      = PDCP_RB_IS_SRB,
                            .tx_direction = SECURITY_DIRECTION_UPLINK,
                            .rx_direction = SECURITY_DIRECTION_DOWNLINK,
                            .sn_len       = PDCP_SN_LEN_5};
  pdcp->add_bearer(srb_cnfg->srb_id, pdcp_cfg);
  if (RB_ID_SRB2 == srb_cnfg->srb_id) {
    pdcp->config_security(srb_cnfg->srb_id, k_rrc_enc, k_rrc_int, k_up_enc, cipher_algo, integ_algo);
    pdcp->enable_integrity(srb_cnfg->srb_id);
    pdcp->enable_encryption(srb_cnfg->srb_id);
  }

  // Setup RLC
  if (srb_cnfg->rlc_cfg_present) {
    rlc->add_bearer(srb_cnfg->srb_id, make_rlc_config_t(*srb_cnfg));
  }

  // Setup MAC
  uint8_t log_chan_group       = 0;
  uint8_t priority             = 0;
  int     prioritized_bit_rate = 0;
  int     bucket_size_duration = 0;

  // TODO: Move this configuration to mac_interface_rrc
  if (srb_cnfg->lc_ch_cfg_present) {
    if (srb_cnfg->lc_ch_cfg.type() == srb_to_add_mod_s::lc_ch_cfg_c_::types::default_value) {
      // Set default SRB values as defined in Table 9.2.1
      switch (srb_cnfg->srb_id) {
        case RB_ID_SRB0:
          rrc_log->error("Setting SRB0: Should not be set by RRC\n");
          break;
        case RB_ID_SRB1:
          priority             = 1;
          prioritized_bit_rate = -1;
          bucket_size_duration = 0;
          break;
        case RB_ID_SRB2:
          priority             = 3;
          prioritized_bit_rate = -1;
          bucket_size_duration = 0;
          break;
      }
    } else {
      if (srb_cnfg->lc_ch_cfg.explicit_value().lc_ch_sr_mask_r9_present) {
        //TODO
      }
      if (srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params_present) {
        if (srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params.lc_ch_group_present)
          log_chan_group = srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params.lc_ch_group;

        priority             = srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params.prio;
        prioritized_bit_rate = srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params.prioritised_bit_rate.to_number();
        bucket_size_duration = srb_cnfg->lc_ch_cfg.explicit_value().ul_specific_params.bucket_size_dur.to_number();
      }
    }
    mac->setup_lcid(srb_cnfg->srb_id, log_chan_group, priority, prioritized_bit_rate, bucket_size_duration);
  }

  srbs[srb_cnfg->srb_id] = *srb_cnfg;
  rrc_log->info("Added radio bearer %s\n", get_rb_name(srb_cnfg->srb_id).c_str());
}

void rrc::add_drb(drb_to_add_mod_s* drb_cnfg)
{
  if (!drb_cnfg->pdcp_cfg_present || !drb_cnfg->rlc_cfg_present || !drb_cnfg->lc_ch_cfg_present) {
    rrc_log->error("Cannot add DRB - incomplete configuration\n");
    return;
  }
  uint32_t lcid = 0;
  if (drb_cnfg->lc_ch_id_present) {
    lcid = drb_cnfg->lc_ch_id;
  } else {
    lcid = RB_ID_SRB2 + drb_cnfg->drb_id;
    rrc_log->warning("LCID not present, using %d\n", lcid);
  }

  // Setup PDCP
  pdcp_config_t pdcp_cfg = {.bearer_id    = drb_cnfg->drb_id,
                            .rb_type      = PDCP_RB_IS_DRB,
                            .tx_direction = SECURITY_DIRECTION_UPLINK,
                            .rx_direction = SECURITY_DIRECTION_DOWNLINK,
                            .sn_len       = PDCP_SN_LEN_12};
  if (drb_cnfg->pdcp_cfg.rlc_um_present) {
    if (drb_cnfg->pdcp_cfg.rlc_um.pdcp_sn_size == pdcp_cfg_s::rlc_um_s_::pdcp_sn_size_e_::len7bits) {
      pdcp_cfg.sn_len = 7;
    }
  }
  pdcp->add_bearer(lcid, pdcp_cfg);
  pdcp->config_security(lcid, k_rrc_enc, k_rrc_int, k_up_enc, cipher_algo, integ_algo);
  pdcp->enable_encryption(lcid);

  // Setup RLC
  rlc->add_bearer(lcid, make_rlc_config_t(drb_cnfg->rlc_cfg));

  // Setup MAC
  uint8_t log_chan_group = 0;
  uint8_t priority = 1;
  int prioritized_bit_rate = -1;
  int bucket_size_duration = -1;
  if (drb_cnfg->lc_ch_cfg.ul_specific_params_present) {
    if (drb_cnfg->lc_ch_cfg.ul_specific_params.lc_ch_group_present) {
      log_chan_group = drb_cnfg->lc_ch_cfg.ul_specific_params.lc_ch_group;
    } else {
      rrc_log->warning("LCG not present, setting to 0\n");
    }
    priority             = drb_cnfg->lc_ch_cfg.ul_specific_params.prio;
    prioritized_bit_rate = drb_cnfg->lc_ch_cfg.ul_specific_params.prioritised_bit_rate.to_number();
    bucket_size_duration = drb_cnfg->lc_ch_cfg.ul_specific_params.bucket_size_dur.to_number();
  }
  mac->setup_lcid(lcid, log_chan_group, priority, prioritized_bit_rate, bucket_size_duration);

  drbs[lcid] = *drb_cnfg;
  drb_up     = true;
  rrc_log->info("Added radio bearer %s (LCID=%d)\n", get_rb_name(lcid).c_str(), lcid);
}

void rrc::release_drb(uint32_t drb_id)
{
  uint32_t lcid = RB_ID_SRB2 + drb_id;

  if (drbs.find(drb_id) != drbs.end()) {
    rrc_log->info("Releasing radio bearer %s\n", get_rb_name(lcid).c_str());
    drbs.erase(lcid);
  } else {
    rrc_log->error("Couldn't release radio bearer %s. Doesn't exist.\n", get_rb_name(lcid).c_str());
  }
}

uint32_t rrc::get_lcid_for_eps_bearer(const uint32_t& eps_bearer_id)
{
  // check if this bearer id exists and return it's LCID
  for (auto& drb : drbs) {
    if (drb.second.eps_bearer_id == eps_bearer_id) {
      return drb.first;
    }
  }
  return 0;
}

void rrc::add_mrb(uint32_t lcid, uint32_t port)
{
  gw->add_mch_port(lcid, port);
  rlc->add_bearer_mrb(lcid);
  mac->mch_start_rx(lcid);
  rrc_log->info("Added MRB bearer for lcid:%d\n", lcid);
}

// PHY CONFIG DEDICATED Defaults (3GPP 36.331 v10 9.2.4)
void rrc::set_phy_default_pucch_srs()
{
  rrc_log->info("Setting default PHY config dedicated\n");
  set_phy_config_dedicated_default();
}

void rrc::set_mac_default() {
  apply_mac_config_dedicated_default();
}

void rrc::set_rrc_default()
{
  N310 = 1;
  N311 = 1;
  timers->get(t310)->set(this, 1000);
  timers->get(t311)->set(this, 1000);
}

/************************************************************************
 *
 *
 * RRC Measurements
 *
 *
 ************************************************************************/

void rrc::rrc_meas::init(rrc* parent_)
{
  this->parent      = parent_;
  this->log_h       = parent_->rrc_log;
  this->phy         = parent_->phy;
  this->timers      = parent_->timers;
  s_measure_enabled = false;
  reset();
}

void rrc::rrc_meas::reset()
{
  filter_k_rsrp = filt_coef_e(filt_coef_e::fc4).to_number();
  filter_k_rsrq = filt_coef_e(filt_coef_e::fc4).to_number();

  // FIXME: Turn struct into a class and use destructor
  std::map<uint32_t, meas_t>::iterator iter = active.begin();
  while (iter != active.end()) {
    remove_meas_id(iter++);
  }

  // These objects do not need destructor
  objects.clear();
  reports_cfg.clear();
  phy->meas_reset();
  bzero(&pcell_measurement, sizeof(meas_value_t));
}

/* L3 filtering 5.5.3.2 */
void rrc::rrc_meas::L3_filter(meas_value_t *value, float values[NOF_MEASUREMENTS])
{
  for (int i=0;i<NOF_MEASUREMENTS;i++) {
    if (value->ms[i]) {
      value->ms[i] = SRSLTE_VEC_EMA(values[i], value->ms[i], filter_a[i]);
    } else {
      value->ms[i] = values[i];
    }
  }
}

void rrc::rrc_meas::new_phy_meas(uint32_t earfcn, uint32_t pci, float rsrp, float rsrq, uint32_t tti)
{
  float values[NOF_MEASUREMENTS] = {rsrp, rsrq};

  // This indicates serving cell
  if (parent->serving_cell->equals(earfcn, pci)) {

    log_h->debug("MEAS:  New measurement serving cell, rsrp=%f, rsrq=%f, tti=%d\n", rsrp, rsrq, tti);

    L3_filter(&pcell_measurement, values);

    // Update serving cell measurement
    parent->serving_cell->set_rsrp(rsrp);

  } else {

    // Add to list of neighbour cells
    bool added = parent->add_neighbour_cell(earfcn, pci, rsrp);

    log_h->debug("MEAS:  New measurement %s earfcn=%d, pci=%d, rsrp=%f, rsrq=%f, tti=%d\n",
                added?"added":"not added", earfcn, pci, rsrp, rsrq, tti);

    // Only report measurements of 8th strongest cells
    if (added) {
      // Save PHY measurement for all active measurements whose earfcn/pci matches
      for(std::map<uint32_t, meas_t>::iterator iter=active.begin(); iter!=active.end(); ++iter) {
        meas_t *m = &iter->second;
        if (objects[m->object_id].earfcn == earfcn) {
          // If it's a newly discovered cell, add it to objects
          if (!m->cell_values.count(pci)) {
            uint32_t cell_idx = objects[m->object_id].found_cells.size();
            objects[m->object_id].found_cells[cell_idx].pci      = pci;
            objects[m->object_id].found_cells[cell_idx].q_offset = 0;
          }
          // Update or add cell
          L3_filter(&m->cell_values[pci], values);
        }
      }
    }
  }
}

// Remove all stored measurements for a given cell
void rrc::rrc_meas::delete_report(uint32_t earfcn, uint32_t pci) {
  for(std::map<uint32_t, meas_t>::iterator iter=active.begin(); iter!=active.end(); ++iter) {
    meas_t *m = &iter->second;
    if (objects[m->object_id].earfcn == earfcn) {
      if (m->cell_values.count(pci)) {
        m->cell_values.erase(pci);
        log_h->info("Deleting report PCI=%d from cell_values\n", pci);
      }
    }
  }
}

void rrc::rrc_meas::run_tti(uint32_t tti) {
  // Measurement Report Triggering Section 5.5.4
  calculate_triggers(tti);
}

bool rrc::rrc_meas::find_earfcn_cell(uint32_t earfcn, uint32_t pci, meas_obj_t **object, int *cell_idx) {
  if (object) {
    *object = NULL;
  }
  for (std::map<uint32_t, meas_obj_t>::iterator obj = objects.begin(); obj != objects.end(); ++obj) {
    if (obj->second.earfcn == earfcn) {
      if (object) {
        *object = &obj->second;
      }
      for (std::map<uint32_t, meas_cell_t>::iterator c = obj->second.found_cells.begin(); c != obj->second.found_cells.end(); ++c) {
        if (c->second.pci == pci) {
          if (cell_idx) {
            *cell_idx = c->first;
            return true;
          }
        }
      }
      // return true if cell idx not found but frequency is found
      if (cell_idx) {
        *cell_idx = -1;
      }
      return true;
    }
  }
  return false;
}

/* Generate report procedure 5.5.5 */
void rrc::rrc_meas::generate_report(uint32_t meas_id)
{
  asn1::rrc::ul_dcch_msg_s ul_dcch_msg;
  ul_dcch_msg.msg.set_c1().set_meas_report().crit_exts.set_c1().set_meas_report_r8();
  meas_results_s* report = &ul_dcch_msg.msg.c1().meas_report().crit_exts.c1().meas_report_r8().meas_results;

  meas_t       *m   = &active[meas_id];
  report_cfg_t *cfg = &reports_cfg[m->report_id];

  report->meas_id                       = (uint8_t)meas_id;
  report->meas_result_pcell.rsrp_result = value_to_range(RSRP, pcell_measurement.ms[RSRP]);
  report->meas_result_pcell.rsrq_result = value_to_range(RSRQ, pcell_measurement.ms[RSRQ]);

  log_h->info("MEAS:  Generate report MeasId=%d, nof_reports_send=%d, Pcell rsrp=%f rsrq=%f\n",
              report->meas_id, m->nof_reports_sent, pcell_measurement.ms[RSRP], pcell_measurement.ms[RSRQ]);

  meas_result_list_eutra_l& neigh_list = report->meas_result_neigh_cells.set_meas_result_list_eutra();
  // TODO: report up to 8 best cells
  for (const auto& cell : m->cell_values) {
    if (cell.second.triggered and neigh_list.size() <= 8) {
      meas_result_eutra_s rc;

      rc.pci                             = (uint16_t)cell.first;
      rc.meas_result.rsrp_result_present = cfg->report_quantity == RSRP || cfg->report_quantity == BOTH;
      rc.meas_result.rsrq_result_present = cfg->report_quantity == RSRQ || cfg->report_quantity == BOTH;
      rc.meas_result.rsrp_result         = value_to_range(RSRP, cell.second.ms[RSRP]);
      rc.meas_result.rsrq_result         = value_to_range(RSRQ, cell.second.ms[RSRQ]);

      log_h->info("MEAS:  Adding to report neighbour=%d, pci=%d, rsrp=%f, rsrq=%f\n",
                  neigh_list.size(),
                  rc.pci,
                  cell.second.ms[RSRP],
                  cell.second.ms[RSRQ]);

      neigh_list.push_back(rc);
    }
  }
  report->meas_result_neigh_cells_present = neigh_list.size() > 0;

  m->nof_reports_sent++;
  timers->get(m->periodic_timer)->stop();

  if (m->nof_reports_sent < cfg->amount) {
    timers->get(m->periodic_timer)->reset();
    timers->get(m->periodic_timer)->run();
  } else {
    if (cfg->trigger_type == report_cfg_t::PERIODIC) {
      m->triggered = false;
    }
  }

  // Send to lower layers
  parent->send_ul_dcch_msg(RB_ID_SRB1, ul_dcch_msg);
}

/* Handle entering/leaving event conditions 5.5.4.1 */
bool rrc::rrc_meas::process_event(eutra_event_s* event, uint32_t tti, bool enter_condition, bool exit_condition,
                                  meas_t* m, meas_value_t* cell)
{
  bool generate_report = false;
  if (enter_condition && (!m->triggered || !cell->triggered)) {
    if (!cell->timer_enter_triggered) {
      cell->timer_enter_triggered = true;
      cell->enter_tti     = tti;
    } else if (srslte_tti_interval(tti, cell->enter_tti) >= event->time_to_trigger) {
      m->triggered        = true;
      cell->triggered     = true;
      m->nof_reports_sent = 0;
      generate_report     = true;
    }
  } else if (exit_condition) {
    if (!cell->timer_exit_triggered) {
      cell->timer_exit_triggered = true;
      cell->exit_tti      = tti;
    } else if (srslte_tti_interval(tti, cell->exit_tti) >= event->time_to_trigger) {
      m->triggered    = false;
      cell->triggered = false;
      timers->get(m->periodic_timer)->stop();
      if (event) {
        if (event->event_id.type() == eutra_event_s::event_id_c_::types::event_a3 &&
            event->event_id.event_a3().report_on_leave) {
          generate_report = true;
        }
      }
    }
  }
  if (!enter_condition) {
    cell->timer_enter_triggered = false;
  }
  if (!enter_condition) {
    cell->timer_exit_triggered = false;
  }
  return generate_report;
}

/* Calculate trigger conditions for each cell 5.5.4 */
void rrc::rrc_meas::calculate_triggers(uint32_t tti)
{
  float Ofp = 0, Ocp = 0;
  meas_obj_t *serving_object   = NULL;
  int         serving_cell_idx = 0;

  // Get serving cell
  if (active.size()) {
    if (find_earfcn_cell(phy->get_current_earfcn(), phy->get_current_pci(), &serving_object, &serving_cell_idx)) {
      Ofp = serving_object->q_offset;
      if (serving_cell_idx >= 0) {
        Ocp = serving_object->found_cells[serving_cell_idx].q_offset;
      }
    } else {
      log_h->warning("Can't find current eafcn=%d, pci=%d in objects list. Using Ofp=0, Ocp=0\n",
                     phy->get_current_earfcn(), phy->get_current_pci());
    }
  }



  for (std::map<uint32_t, meas_t>::iterator m = active.begin(); m != active.end(); ++m) {
    report_cfg_t *cfg = &reports_cfg[m->second.report_id];
    double        hyst = 0.5 * cfg->event.hysteresis;
    float Mp   = pcell_measurement.ms[cfg->trigger_quantity];

    eutra_event_s::event_id_c_ event_id  = cfg->event.event_id;
    std::string                event_str = event_id.type().to_string();

    bool gen_report = false;

    if (cfg->trigger_type == report_cfg_t::EVENT) {

      // A1 & A2 are for serving cell only
      if (event_id.type().value < eutra_event_s::event_id_c_::types::event_a3) {

        bool enter_condition;
        bool exit_condition;
        if (event_id.type() == eutra_event_s::event_id_c_::types::event_a1) {
          uint8_t range;
          if (cfg->event.event_id.event_a1().a1_thres.type().value == thres_eutra_c::types::thres_rsrp) {
            range = cfg->event.event_id.event_a1().a1_thres.thres_rsrp();
          } else {
            range = cfg->event.event_id.event_a1().a1_thres.thres_rsrq();
          }
          enter_condition = Mp - hyst > range_to_value(cfg->trigger_quantity, range);
          exit_condition  = Mp + hyst < range_to_value(cfg->trigger_quantity, range);
        } else {
          uint8_t range;
          if (cfg->event.event_id.event_a2().a2_thres.type() == thres_eutra_c::types::thres_rsrp) {
            range = cfg->event.event_id.event_a2().a2_thres.thres_rsrp();
          } else {
            range = cfg->event.event_id.event_a2().a2_thres.thres_rsrq();
          }
          enter_condition = Mp + hyst < range_to_value(cfg->trigger_quantity, range);
          exit_condition  = Mp - hyst > range_to_value(cfg->trigger_quantity, range);
        }

        // check only if
        gen_report |= process_event(&cfg->event, tti, enter_condition, exit_condition, &m->second, &pcell_measurement);

        if (gen_report) {
          log_h->info("Triggered by A1/A2 event\n");
        }
        // Rest are evaluated for every cell in frequency
      } else {
        meas_obj_t *obj = &objects[m->second.object_id];
        for (std::map<uint32_t, meas_cell_t>::iterator cell = obj->found_cells.begin(); cell != obj->found_cells.end(); ++cell) {
          if (m->second.cell_values.count(cell->second.pci)) {
            float Ofn = obj->q_offset;
            float Ocn = cell->second.q_offset;
            float Mn = m->second.cell_values[cell->second.pci].ms[cfg->trigger_quantity];
            double  Off = 0;
            float   th = 0, th1 = 0, th2 = 0;
            bool enter_condition = false;
            bool exit_condition  = false;
            uint8_t range, range2;
            switch (event_id.type().value) {
              case eutra_event_s::event_id_c_::types::event_a3:
                Off             = 0.5 * cfg->event.event_id.event_a3().a3_offset;
                enter_condition = Mn + Ofn + Ocn - hyst > Mp + Ofp + Ocp + Off;
                exit_condition  = Mn + Ofn + Ocn + hyst < Mp + Ofp + Ocp + Off;
                break;
              case eutra_event_s::event_id_c_::types::event_a4:
                if (cfg->event.event_id.event_a4().a4_thres.type() == thres_eutra_c::types::thres_rsrp) {
                  range = cfg->event.event_id.event_a4().a4_thres.thres_rsrp();
                } else {
                  range = cfg->event.event_id.event_a4().a4_thres.thres_rsrq();
                }
                th              = range_to_value(cfg->trigger_quantity, range);
                enter_condition = Mn + Ofn + Ocn - hyst > th;
                exit_condition  = Mn + Ofn + Ocn + hyst < th;
                break;
              case eutra_event_s::event_id_c_::types::event_a5:
                if (cfg->event.event_id.event_a5().a5_thres1.type() == thres_eutra_c::types::thres_rsrp) {
                  range = cfg->event.event_id.event_a5().a5_thres1.thres_rsrp();
                } else {
                  range = cfg->event.event_id.event_a5().a5_thres1.thres_rsrq();
                }
                if (cfg->event.event_id.event_a5().a5_thres2.type() == thres_eutra_c::types::thres_rsrp) {
                  range2 = cfg->event.event_id.event_a5().a5_thres2.thres_rsrp();
                } else {
                  range2 = cfg->event.event_id.event_a5().a5_thres2.thres_rsrq();
                }
                th1             = range_to_value(cfg->trigger_quantity, range);
                th2             = range_to_value(cfg->trigger_quantity, range2);
                enter_condition = (Mp + hyst < th1) && (Mn + Ofn + Ocn - hyst > th2);
                exit_condition  = (Mp - hyst > th1) && (Mn + Ofn + Ocn + hyst < th2);
                break;
              default:
                log_h->error("Error event %s not implemented\n", event_str.c_str());
            }
            gen_report |= process_event(&cfg->event, tti, enter_condition, exit_condition,
                                        &m->second, &m->second.cell_values[cell->second.pci]);
          }
        }
      }
    }
    if (gen_report) {
      log_h->info("Generate report MeasId=%d, from event\n", m->first);
      generate_report(m->first);
    }
  }
}

// Procedure upon handover or reestablishment 5.5.6.1
void rrc::rrc_meas::ho_finish() {
  // Remove all measId with trigger periodic
  std::map<uint32_t, meas_t>::iterator iter = active.begin();
  while (iter != active.end()) {
    if (reports_cfg[iter->second.report_id].trigger_type == report_cfg_t::PERIODIC) {
      remove_meas_id(iter++);
    } else {
      ++iter;
    }
  }

  //TODO: Inter-frequency handover

  // Stop all reports
  for (std::map<uint32_t, meas_t>::iterator iter = active.begin(); iter != active.end(); ++iter) {
    stop_reports(&iter->second);
  }
}

// 5.5.4.1 expiry of periodical reporting timer
bool rrc::rrc_meas::timer_expired(uint32_t timer_id) {
  for (std::map<uint32_t, meas_t>::iterator iter = active.begin(); iter != active.end(); ++iter) {
    if (iter->second.periodic_timer == timer_id) {
      log_h->info("Generate report MeasId=%d, from timerId=%d\n", iter->first, timer_id);
      generate_report(iter->first);
      return true;
    }
  }
  return false;
}

void rrc::rrc_meas::stop_reports(meas_t* m)
{
  timers->get(m->periodic_timer)->stop();
  m->triggered = false;
}

void rrc::rrc_meas::stop_reports_object(uint32_t object_id) {
  for (std::map<uint32_t, meas_t>::iterator iter = active.begin(); iter != active.end(); ++iter) {
    if (iter->second.object_id == object_id) {
      stop_reports(&iter->second);
    }
  }
}

void rrc::rrc_meas::remove_meas_object(uint32_t object_id) {
  std::map<uint32_t, meas_t>::iterator iter = active.begin();
  while (iter != active.end()) {
    if (iter->second.object_id == object_id) {
      remove_meas_id(iter++);
    } else {
      ++iter;
    }
  }
}

void rrc::rrc_meas::remove_meas_report(uint32_t report_id) {
  std::map<uint32_t, meas_t>::iterator iter = active.begin();
  while (iter != active.end()) {
    if (iter->second.report_id == report_id) {
      remove_meas_id(iter++);
    } else {
      ++iter;
    }
  }
}

void rrc::rrc_meas::remove_meas_id(uint32_t measId)
{
  if (active.count(measId)) {
    timers->get(active[measId].periodic_timer)->stop();
    timers->release_id(active[measId].periodic_timer);
    log_h->info("MEAS: Removed measId=%d\n", measId);
    active.erase(measId);
  } else {
    log_h->warning("MEAS: Removing unexistent measId=%d\n", measId);
  }
}

void rrc::rrc_meas::remove_meas_id(std::map<uint32_t, meas_t>::iterator it)
{
  timers->get(it->second.periodic_timer)->stop();
  timers->release_id(it->second.periodic_timer);
  log_h->info("MEAS: Removed measId=%d\n", it->first);
  active.erase(it);
}

/* Parses MeasConfig object from RRCConnectionReconfiguration message and applies configuration
 * as per section 5.5.2
 */
bool rrc::rrc_meas::parse_meas_config(meas_cfg_s* cfg)
{
  // Measurement object removal 5.5.2.4
  for (uint32_t i = 0; i < cfg->meas_obj_to_rem_list.size(); i++) {
    objects.erase(cfg->meas_obj_to_rem_list[i]);
    remove_meas_object(cfg->meas_obj_to_rem_list[i]);
    log_h->info("MEAS: Removed measObjectId=%d\n", cfg->meas_obj_to_rem_list[i]);
  }

  // Measurement object addition/modification Section 5.5.2.5
  if (cfg->meas_obj_to_add_mod_list_present) {
    for (uint32_t i = 0; i < cfg->meas_obj_to_add_mod_list.size(); i++) {
      if (cfg->meas_obj_to_add_mod_list[i].meas_obj.type() ==
          meas_obj_to_add_mod_s::meas_obj_c_::types::meas_obj_eutra) {
        meas_obj_eutra_s* src_obj = &cfg->meas_obj_to_add_mod_list[i].meas_obj.meas_obj_eutra();

        // Access the object if exists or create it
        meas_obj_t* dst_obj = &objects[cfg->meas_obj_to_add_mod_list[i].meas_obj_id];

        dst_obj->earfcn   = src_obj->carrier_freq;;
        if (src_obj->offset_freq_present) {
          dst_obj->q_offset = src_obj->offset_freq.to_number();
        } else {
          dst_obj->q_offset = 0;
        }

        if (src_obj->black_cells_to_rem_list_present) {
          for (uint32_t j = 0; j < src_obj->black_cells_to_rem_list.size(); j++) {
            dst_obj->meas_cells.erase(src_obj->black_cells_to_rem_list[i]);
          }
        }

        for (uint32_t j = 0; j < src_obj->cells_to_add_mod_list.size(); j++) {
          dst_obj->meas_cells[src_obj->cells_to_add_mod_list[j].cell_idx].q_offset =
              src_obj->cells_to_add_mod_list[j].cell_individual_offset.to_number();
          dst_obj->meas_cells[src_obj->cells_to_add_mod_list[j].cell_idx].pci = src_obj->cells_to_add_mod_list[j].pci;

          log_h->info("MEAS: Added measObjectId=%d, earfcn=%d, q_offset=%f, pci=%d, offset_cell=%f\n",
                      cfg->meas_obj_to_add_mod_list[i].meas_obj_id, dst_obj->earfcn, dst_obj->q_offset,
                      dst_obj->meas_cells[src_obj->cells_to_add_mod_list[j].cell_idx].pci,
                      dst_obj->meas_cells[src_obj->cells_to_add_mod_list[j].cell_idx].q_offset);
        }

        // Untrigger reports and stop timers
        stop_reports_object(cfg->meas_obj_to_add_mod_list[i].meas_obj_id);

        // TODO: Blackcells
        // TODO: meassubframepattern

      } else {
        log_h->warning("MEAS: Unsupported MeasObject type %s\n",
                       cfg->meas_obj_to_add_mod_list[i].meas_obj.type().to_string().c_str());
      }
    }
  }

  // Reporting configuration removal 5.5.2.6
  for (uint32_t i = 0; i < cfg->report_cfg_to_rem_list.size(); i++) {
    reports_cfg.erase(cfg->report_cfg_to_rem_list[i]);
    remove_meas_report(cfg->report_cfg_to_rem_list[i]);
    log_h->info("MEAS: Removed reportConfigId=%d\n", cfg->report_cfg_to_rem_list[i]);
  }

  // Reporting configuration addition/modification 5.5.2.7
  if (cfg->report_cfg_to_add_mod_list_present) {
    for (uint32_t i = 0; i < cfg->report_cfg_to_add_mod_list.size(); i++) {
      if (cfg->report_cfg_to_add_mod_list[i].report_cfg.type() ==
          report_cfg_to_add_mod_s::report_cfg_c_::types::report_cfg_eutra) {
        report_cfg_eutra_s* src_rep = &cfg->report_cfg_to_add_mod_list[i].report_cfg.report_cfg_eutra();
        // Access the object if exists or create it
        report_cfg_t* dst_rep = &reports_cfg[cfg->report_cfg_to_add_mod_list[i].report_cfg_id];

        dst_rep->trigger_type = src_rep->trigger_type.type() == report_cfg_eutra_s::trigger_type_c_::types::event
                                    ? report_cfg_t::EVENT
                                    : report_cfg_t::PERIODIC;
        if (dst_rep->trigger_type == report_cfg_t::EVENT) {
          dst_rep->event = src_rep->trigger_type.event();
        }
        dst_rep->amount           = (uint8_t)src_rep->report_amount.to_number();
        dst_rep->interval         = src_rep->report_interv.to_number();
        dst_rep->max_cell         = src_rep->max_report_cells;
        dst_rep->trigger_quantity = (quantity_t)src_rep->trigger_quant.value;
        dst_rep->report_quantity  = src_rep->report_quant == report_cfg_eutra_s::report_quant_e_::same_as_trigger_quant
                                       ? dst_rep->trigger_quantity
                                       : BOTH;

        if (dst_rep->trigger_type == report_cfg_t::EVENT) {
          log_h->info("MEAS: Added reportConfigId=%d, event=%s, amount=%d, interval=%d\n",
                      cfg->report_cfg_to_add_mod_list[i].report_cfg_id,
                      dst_rep->event.event_id.type().to_string().c_str(), dst_rep->amount, dst_rep->interval);
        } else {
          log_h->info("MEAS: Added reportConfigId=%d, type=periodical, amount=%d, interval=%d\n",
                      cfg->report_cfg_to_add_mod_list[i].report_cfg_id, dst_rep->amount, dst_rep->interval);
        }

        // Reset reports counter
        for (std::map<uint32_t, meas_t>::iterator iter = active.begin(); iter != active.end(); ++iter) {
          if (iter->second.report_id == cfg->report_cfg_to_add_mod_list[i].report_cfg_id) {
            iter->second.nof_reports_sent = 0;
            stop_reports(&iter->second);
          }
        }
      } else {
        log_h->warning("MEAS: Unsupported reportConfigType %s\n",
                       cfg->report_cfg_to_add_mod_list[i].report_cfg.type().to_string().c_str());
      }
    }
  }

  // Quantity configuration 5.5.2.8
  if (cfg->quant_cfg_present && cfg->quant_cfg.quant_cfg_eutra_present) {
    if (cfg->quant_cfg.quant_cfg_eutra.filt_coef_rsrp_present) {
      filter_k_rsrp = cfg->quant_cfg.quant_cfg_eutra.filt_coef_rsrp.to_number();
    } else {
      filter_k_rsrp = filt_coef_e(filt_coef_e::fc4).to_number();
    }
    if (cfg->quant_cfg.quant_cfg_eutra.filt_coef_rsrq_present) {
      filter_k_rsrq = cfg->quant_cfg.quant_cfg_eutra.filt_coef_rsrq.to_number();
    } else {
      filter_k_rsrq = filt_coef_e(filt_coef_e::fc4).to_number();
    }
    filter_a[RSRP] = powf(0.5, (float)filter_k_rsrp / 4);
    filter_a[RSRQ] = powf(0.5, (float)filter_k_rsrq / 4);

    log_h->info("MEAS: Quantity configuration k_rsrp=%d, k_rsrq=%d\n", filter_k_rsrp, filter_k_rsrq);
  }

  // Measurement identity removal 5.5.2.2
  for (uint32_t i = 0; i < cfg->meas_id_to_rem_list.size(); i++) {
    remove_meas_id(cfg->meas_id_to_rem_list[i]);
  }

  log_h->info("nof active measId=%zd\n", active.size());

  // Measurement identity addition/modification 5.5.2.3
  if (cfg->meas_id_to_add_mod_list_present) {
    for (uint32_t i = 0; i < cfg->meas_id_to_add_mod_list.size(); i++) {
      meas_id_to_add_mod_s* meas_id = &cfg->meas_id_to_add_mod_list[i];
      // Stop the timer if the entry exists or create the timer if not
      bool is_new = false;
      if (active.count(meas_id->meas_id)) {
        timers->get(active[meas_id->meas_id].periodic_timer)->stop();
      } else {
        is_new                                  = true;
        active[meas_id->meas_id].periodic_timer = timers->get_unique_id();
      }
      active[meas_id->meas_id].object_id = meas_id->meas_obj_id;
      active[meas_id->meas_id].report_id = meas_id->report_cfg_id;
      log_h->info("MEAS: %s measId=%d, measObjectId=%d, reportConfigId=%d, timer_id=%u, nof_values=%zd\n",
                  is_new ? "Added" : "Updated", meas_id->meas_id, meas_id->meas_obj_id, meas_id->report_cfg_id,
                  active[meas_id->meas_id].periodic_timer, active[meas_id->meas_id].cell_values.size());
    }
  }

  // S-Measure
  if (cfg->s_measure_present) {
    if (cfg->s_measure) {
      s_measure_enabled = true;
      s_measure_value   = range_to_value(RSRP, cfg->s_measure);
    } else {
      s_measure_enabled = false;
    }
  }

  update_phy();

  return true;
}

/* Instruct PHY to start measurement */
void rrc::rrc_meas::update_phy()
{
  phy->meas_reset();
  for (const auto& obj : objects) {
    meas_obj_t o = obj.second;
    // Instruct PHY to look for neighbour cells on this frequency
    phy->meas_start(o.earfcn);
    for (const auto& cell : o.meas_cells) {
      // Instruct PHY to look for cells IDs on this frequency
      phy->meas_start(o.earfcn, cell.second.pci);
    }
  }
}

uint8_t rrc::rrc_meas::value_to_range(quantity_t quant, float value) {
  uint8_t range = 0;
  switch(quant) {
    case RSRP:
      if (value < -140) {
        range = 0;
      } else if (-140 <= value && value < -44) {
        range = 1u + (uint8_t)(value + 140);
      } else {
        range = 97;
      }
      break;
    case RSRQ:
      if (value < -19.5) {
        range = 0;
      } else if (-19.5 <= value && value < -3) {
        range = 1u + (uint8_t)(2 * (value + 19.5));
      } else {
        range = 34;
      }
      break;
    case BOTH:
      log_h->error("Error quantity both not supported in value_to_range\n");
      break;
  }
  return range;
}

float rrc::rrc_meas::range_to_value(quantity_t quant, uint8_t range)
{
  float val = 0;
  switch (quant) {
    case RSRP:
      val = -140 + (float)range;
      break;
    case RSRQ:
      val = -19.5f + (float)range / 2;
      break;
    case BOTH:
      log_h->error("Error quantity both not supported in range_to_value\n");
      break;
  }
  return val;
}

const std::string rrc::rb_id_str[] = {"SRB0", "SRB1", "SRB2",
                                      "DRB1", "DRB2", "DRB3",
                                      "DRB4", "DRB5", "DRB6",
                                      "DRB7", "DRB8"};

} // namespace srsue
