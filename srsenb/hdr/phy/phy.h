/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2017 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of srsLTE.
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

#ifndef SRSENB_PHY_H
#define SRSENB_PHY_H

#include "phch_common.h"
#include "phch_worker.h"
#include "srslte/common/log.h"
#include "srslte/common/log_filter.h"
#include "srslte/common/task_dispatcher.h"
#include "srslte/common/trace.h"
#include "srslte/interfaces/enb_interfaces.h"
#include "srslte/interfaces/enb_metrics_interface.h"
#include "srslte/radio/radio.h"
#include "txrx.h"
#include <srslte/asn1/rrc_asn1.h>

namespace srsenb {
 
typedef struct {
  srslte_cell_t                  cell;
  asn1::rrc::prach_cfg_sib_s     prach_cnfg;
  asn1::rrc::pdsch_cfg_common_s  pdsch_cnfg;
  asn1::rrc::pusch_cfg_common_s  pusch_cnfg;
  asn1::rrc::pucch_cfg_common_s  pucch_cnfg;
  asn1::rrc::srs_ul_cfg_common_c srs_ul_cnfg;
} phy_cfg_t;

class phy : public phy_interface_mac,
            public phy_interface_rrc
{
public:

  phy();
  bool init(phy_args_t *args, phy_cfg_t *common_cfg, srslte::radio *radio_handler, mac_interface_phy *mac, srslte::log_filter* log_h);
  bool init(phy_args_t *args, phy_cfg_t *common_cfg, srslte::radio *radio_handler, mac_interface_phy *mac, std::vector<srslte::log_filter *> log_vec);
  void stop();
  
  /* MAC->PHY interface */
  int  add_rnti(uint16_t rnti);
  void rem_rnti(uint16_t rnti);
  
  /*RRC-PHY interface*/
  void configure_mbsfn(asn1::rrc::sib_type2_s* sib2, asn1::rrc::sib_type13_r9_s* sib13, asn1::rrc::mcch_msg_s mcch);

  static uint32_t tti_to_SFN(uint32_t tti);
  static uint32_t tti_to_subf(uint32_t tti);
  
  void start_plot();
  void set_conf_dedicated_ack(uint16_t rnti, bool dedicated_ack);
  void set_config_dedicated(uint16_t rnti, asn1::rrc::phys_cfg_ded_s* dedicated);

  void get_metrics(phy_metrics_t metrics[ENB_METRICS_MAX_USERS]);
  
private:
  phy_rrc_cfg_t phy_rrc_config;
  uint32_t nof_workers; 
  
  const static int MAX_WORKERS         = 4;
  const static int DEFAULT_WORKERS     = 2;
  
  const static int PRACH_WORKER_THREAD_PRIO = 3;
  const static int SF_RECV_THREAD_PRIO = 1;
  const static int WORKERS_THREAD_PRIO = 2;
  
  srslte::radio         *radio_handler;
  srslte::log              *log_h;
  srslte::thread_pool      workers_pool;
  std::vector<phch_worker> workers;
  phch_common              workers_common; 
  prach_worker             prach; 
  txrx                     tx_rx; 
  
  srslte_prach_cfg_t prach_cfg; 
  
  void parse_config(phy_cfg_t* cfg);
  
};

} // namespace srsenb

#endif // SRSENB_PHY_H
