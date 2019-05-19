/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
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

#ifndef EMU_SRSENB_PHY_ADAPTER_H
#define EMU_SRSENB_PHY_ADAPTER_H

#include "srslte/config.h"
#include "srslte/common/log.h"
#include "srslte/phy/enb/enb_dl.h"
#include "srsenb/hdr/upper/rrc.h"
#include "srslte/interfaces/enb_interfaces.h"
#include "libemanelte/mhalconfig.h"

#include <vector>

namespace srsenb {

namespace phy_adapter {

void enb_initialize(srslte::log*                   log_h, 
                    uint32_t                       sf_interval,
                    uint32_t                       physical_cell_id,
                    srslte_cp_t                    cp,
                    float                          ul_freq,
                    float                          dl_freq,
                    int                            nprb, 
                    EMANELTE::MHAL::mhal_config_t& mhal_config,
                    rrc_cfg_t*                     rrc_cfg);

 // state start
 void enb_start();

 // state stop
 void enb_stop();

 // set base info
 void enb_dl_tx_init(const srslte_enb_dl_t* q,
                     uint32_t               tti,
                     uint32_t               cfi);

 // send to mhal with sot time
 bool enb_dl_send_signal(time_t sot_sec,
                         float  frac_sec);

 // get from mhal with sot time
 bool enb_ul_get_signal(uint32_t            tti,
                        srslte_timestamp_t* ts);

 // set the power scaling on a per rnti basis
 void enb_dl_set_power_allocation(uint32_t tti, 
                                  uint16_t rnti,
                                  float    rho_a_db,
                                  float    rho_b_db);

 // set dl pdcch
 int enb_dl_put_pdcch_dl(srslte_enb_dl_t*  q, 
                         srslte_dci_cfg_t* dci_cfg, 
                         srslte_dci_dl_t*  dci_dl,
                         uint32_t          num);

 // set dl pdsch
 int enb_dl_put_pdsch(srslte_enb_dl_t*    q, 
                      srslte_pdsch_cfg_t* pdsch, 
                      uint8_t*            data[SRSLTE_MAX_CODEWORDS],
                      uint32_t            num);

 // set dl mch
 int enb_dl_put_pmch(srslte_enb_dl_t*                     q,
                     srslte_pmch_cfg_t*                   pmch_cfg,
                     mac_interface_phy::dl_sched_grant_t* dl_sched_grant);

 //set ul pdcch
 int enb_dl_put_pdcch_ul(srslte_enb_dl_t*  q, 
                         srslte_dci_cfg_t* dci_cfg, 
                         srslte_dci_ul_t*  dci_ul,
                         uint32_t          num);

 // set phich
 int enb_dl_put_phich(srslte_enb_dl_t*                   q, 
                      srslte_phich_grant_t*              grant,
                      mac_interface_phy::ul_sched_ack_t* ack);

 // get prach
 int enb_ul_get_prach(uint32_t*  indicies, 
                      float*     offsets,
                      float*     avgs,
                      uint32_t   max_entries,
                      uint32_t & num_detected);

 // get pucch
 int enb_ul_get_pucch(srslte_enb_ul_t*    q,
                      srslte_ul_sf_cfg_t* ul_sf,
                      srslte_pucch_cfg_t* cfg,
                      srslte_pucch_res_t* res);

 // get pusch
 int enb_ul_get_pusch(srslte_enb_ul_t*    q,
                      srslte_ul_sf_cfg_t* ul_sf,
                      srslte_pusch_cfg_t* cfg,
                      srslte_pusch_res_t* res,
                      uint16_t            rnti);
 } 
}
#endif //EMU_SRSENB_PHY_ADAPTER_H
