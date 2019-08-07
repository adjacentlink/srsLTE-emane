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

#ifndef SRSLTE_RLC_UM_H
#define SRSLTE_RLC_UM_H

#include "srslte/common/buffer_pool.h"
#include "srslte/common/log.h"
#include "srslte/common/common.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/upper/rlc_tx_queue.h"
#include "srslte/upper/rlc_common.h"
#include <pthread.h>
#include <map>
#include <queue>

namespace srslte {

struct rlc_umd_pdu_t{
  rlc_umd_pdu_header_t  header;
  byte_buffer_t        *buf;
};

class rlc_um
    :public rlc_common
{
public:
  rlc_um();
  ~rlc_um();
  void init(log                       *rlc_entity_log_,
            uint32_t                   lcid_,
            srsue::pdcp_interface_rlc *pdcp_,
            srsue::rrc_interface_rlc  *rrc_,
            mac_interface_timers      *mac_timers_);
  bool configure(srslte_rlc_config_t cnfg);
  void reestablish();
  void stop();
  void empty_queue();
  bool is_mrb();

  rlc_mode_t    get_mode();
  uint32_t      get_bearer();

  // PDCP interface
  void write_sdu(byte_buffer_t *sdu, bool blocking = true);

  // MAC interface
  bool     has_data();
  uint32_t get_buffer_state();
  int      read_pdu(uint8_t *payload, uint32_t nof_bytes);
  void     write_pdu(uint8_t *payload, uint32_t nof_bytes);
  int get_increment_sequence_num();

  uint32_t get_num_tx_bytes();
  uint32_t get_num_rx_bytes();
  void reset_metrics();

  queue_metrics_t get_qmetrics(bool bReset = false);
private:

  // Transmitter sub-class
  class rlc_um_tx
  {
  public:
    rlc_um_tx();
    ~rlc_um_tx();
    void init(srslte::log *log_);
    bool configure(srslte_rlc_config_t cfg, std::string rb_name);
    int  build_data_pdu(uint8_t *payload, uint32_t nof_bytes);
    void stop();
    void reestablish();
    void empty_queue();
    void write_sdu(byte_buffer_t *sdu);
    void try_write_sdu(byte_buffer_t *sdu);
    uint32_t get_num_tx_bytes();
    void reset_metrics();
    bool has_data();
    uint32_t get_buffer_state();

    queue_metrics_t get_qmetrics(bool bReset = false);

  private:
    byte_buffer_pool        *pool;
    srslte::log             *log;
    std::string             rb_name;

    /****************************************************************************
     * Configurable parameters
     * Ref: 3GPP TS 36.322 v10.0.0 Section 7
     ***************************************************************************/
    srslte_rlc_um_config_t  cfg;

    // TX SDU buffers
    rlc_tx_queue            tx_sdu_queue;
    byte_buffer_t           *tx_sdu;

    /****************************************************************************
     * State variables and counters
     * Ref: 3GPP TS 36.322 v10.0.0 Section 7
     ***************************************************************************/
    uint32_t                vt_us;    // Send state. SN to be assigned for next PDU.

    // Mutexes
    pthread_mutex_t         mutex;

    bool                    tx_enabled;

    uint32_t                num_tx_bytes;

    // helper functions
    void debug_state();
    const char* get_rb_name();
  };

  // Receiver sub-class
  class rlc_um_rx : public timer_callback {
  public:
    rlc_um_rx();
    ~rlc_um_rx();
    void init(srslte::log *log_,
              uint32_t lcid_,
              srsue::pdcp_interface_rlc *pdcp_,
              srsue::rrc_interface_rlc *rrc_,
              srslte::mac_interface_timers *mac_timers_);
    void stop();
    void reestablish();
    bool configure(srslte_rlc_config_t cfg, std::string rb_name);
    void handle_data_pdu(uint8_t *payload, uint32_t nof_bytes);
    void reassemble_rx_sdus();
    bool pdu_belongs_to_rx_sdu();
    bool inside_reordering_window(uint16_t sn);
    uint32_t get_num_rx_bytes();
    void reset_metrics();

    // Timeout callback interface
    void timer_expired(uint32_t timeout_id);

  private:
    void reset();

    byte_buffer_pool                    *pool;
    srslte::log                         *log;
    mac_interface_timers                *mac_timers;
    std::string                         rb_name;

    /****************************************************************************
     * Configurable parameters
     * Ref: 3GPP TS 36.322 v10.0.0 Section 7
     ***************************************************************************/
    srslte_rlc_um_config_t              cfg;

    // Rx window
    std::map<uint32_t, rlc_umd_pdu_t>   rx_window;

    // RX SDU buffers
    byte_buffer_t                       *rx_sdu;
    uint32_t                            vr_ur_in_rx_sdu;

    // Rx state variables and counter
    uint32_t                            vr_ur;  // Receive state. SN of earliest PDU still considered for reordering.
    uint32_t                            vr_ux;  // t_reordering state. SN following PDU which triggered t_reordering.
    uint32_t                            vr_uh;  // Highest rx state. SN following PDU with highest SN among rxed PDUs.
    bool                                pdu_lost;

    uint32_t                            num_rx_bytes;

    // Upper layer handles and variables
    srsue::pdcp_interface_rlc           *pdcp;
    srsue::rrc_interface_rlc            *rrc;
    uint32_t                            lcid;

    // Mutexes
    pthread_mutex_t                     mutex;

    bool                                rx_enabled;

    /****************************************************************************
     * Timers
     * Ref: 3GPP TS 36.322 v10.0.0 Section 7
     ***************************************************************************/
    srslte::timers::timer *reordering_timer;
    uint32_t               reordering_timer_id;

    // helper functions
    void debug_state();
    const char* get_rb_name();
  };

  // Rx and Tx objects
  rlc_um_tx           tx;
  rlc_um_rx           rx;

  // Common variables needed by parent class
  srsue::rrc_interface_rlc  *rrc;
  srslte::log               *log;
  uint32_t                  lcid;
  srslte_rlc_um_config_t    cfg;
  std::string               rb_name;
  byte_buffer_pool          *pool;

  std::string               get_rb_name(srsue::rrc_interface_rlc *rrc, uint32_t lcid, bool is_mrb);
};

/****************************************************************************
 * Header pack/unpack helper functions
 * Ref: 3GPP TS 36.322 v10.0.0 Section 6.2.1
 ***************************************************************************/
void        rlc_um_read_data_pdu_header(byte_buffer_t *pdu, rlc_umd_sn_size_t sn_size, rlc_umd_pdu_header_t *header);
void        rlc_um_read_data_pdu_header(uint8_t *payload, uint32_t nof_bytes, rlc_umd_sn_size_t sn_size, rlc_umd_pdu_header_t *header);
void        rlc_um_write_data_pdu_header(rlc_umd_pdu_header_t *header, byte_buffer_t *pdu);

uint32_t    rlc_um_packed_length(rlc_umd_pdu_header_t *header);
bool        rlc_um_start_aligned(uint8_t fi);
bool        rlc_um_end_aligned(uint8_t fi);

} // namespace srsue


#endif // SRSLTE_RLC_UM_H
