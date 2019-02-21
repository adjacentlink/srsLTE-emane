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

#ifndef SRSLTE_RLC_TM_H
#define SRSLTE_RLC_TM_H

#include "srslte/common/buffer_pool.h"
#include "srslte/common/log.h"
#include "srslte/common/common.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/upper/rlc_tx_queue.h"
#include "srslte/upper/rlc_common.h"

namespace srslte {

class rlc_tm : public rlc_common
{
public:
  rlc_tm(uint32_t queue_len = 16);
  ~rlc_tm();
  void init(log                       *rlc_entity_log_,
            uint32_t                   lcid_,
            srsue::pdcp_interface_rlc *pdcp_,
            srsue::rrc_interface_rlc  *rrc_,
            mac_interface_timers      *mac_timers);
  bool configure(srslte_rlc_config_t cnfg);
  void stop();
  void reestablish();
  void empty_queue(); 

  rlc_mode_t    get_mode();
  uint32_t      get_bearer();

  uint32_t get_num_tx_bytes();
  uint32_t get_num_rx_bytes();
  void reset_metrics();

  // PDCP interface
  void write_sdu(byte_buffer_t *sdu, bool blocking);

  // MAC interface
  bool     has_data();
  uint32_t get_buffer_state();
  int      read_pdu(uint8_t *payload, uint32_t nof_bytes);
  void     write_pdu(uint8_t *payload, uint32_t nof_bytes);

private:

  byte_buffer_pool          *pool;
  srslte::log               *log;
  uint32_t                   lcid;
  srsue::pdcp_interface_rlc *pdcp;
  srsue::rrc_interface_rlc  *rrc;

  bool tx_enabled;

  uint32_t num_tx_bytes;
  uint32_t num_rx_bytes;

  // Thread-safe queues for MAC messages
  rlc_tx_queue    ul_queue;
};

} // namespace srsue


#endif // SRSLTE_RLC_TM_H
