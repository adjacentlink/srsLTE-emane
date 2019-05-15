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

 // DCI to pdsch data ref
 const uint32_t DL_DCIREF_ID_BEGIN = 0;

void enb_initialize(srslte::log * log_h, 
                    uint32_t sf_interval,
                    uint32_t physical_cell_id,
                    srslte_cp_t cp,
                    float ul_freq,
                    float dl_freq,
                    int nprb, 
                    EMANELTE::MHAL::mhal_config_t & mhal_config,
                    rrc_cfg_t * rrc_cfg);

 // state start
 void enb_start();

 // state stop
 void enb_stop();

 // set base info
 void enb_dl_tx_init(const srslte_enb_dl_t *q,
                     uint32_t tti,
                     uint32_t cfi);

 // mark end of transmission
 void enb_dl_tx_end();

 // set the power scaling on a per rnti basis
 void enb_dl_set_power_allocation(uint32_t tti, uint16_t rnti, float rho_a_db,  float rho_b_db);

#if 0
 // set dl pdcch
 int enb_dl_put_pdcch_dl(srslte_enb_dl_pdsch_t * grant,
                         const srslte_enb_dl_t * enb_dl,
                         uint32_t & refid);
#endif

#if 0
 // set dl mch
 void enb_dl_put_pmch(const srslte_enb_dl_pdsch_t *grant, const srslte_ra_dl_grant_t *phy_grant);
#endif

#if 0
 //set ul pdcch
 int enb_dl_put_pdcch_ul(srslte_enb_ul_pusch_t * ul_pusch,
                         const srslte_enb_dl_t * enb_dl);
#endif

#if 0
 // set phich
 void enb_dl_put_phich(const srslte_enb_dl_t *enb_dl,
                      const srslte_enb_dl_phich_t * ack,
                      uint32_t n_prb_L,
                      uint32_t n_dmrs);
#endif

 // send to mhal with sot time
 bool enb_dl_send_signal(time_t sot_sec, float frac_sec);

 // get from mhal with sot time
 bool enb_ul_get_signal(uint32_t tti, srslte_timestamp_t * ts);

 // get prach
 int enb_ul_get_prach(uint32_t * indicies, float * offsets, float * avgs, uint32_t max_entries, uint32_t & num_detected);

#if 0
 // get pucch
 int enb_ul_get_pucch(srslte_enb_ul_t * q,
                      uint16_t rnti, 
                      srslte_uci_data_t *uci_data);
#endif

#if 0
 // get pusch
 int enb_ul_get_pusch(srslte_enb_ul_t * q,
                      srslte_ra_ul_grant_t *grant,
                      uint16_t rnti, 
                      uint32_t rv_idx, 
                      uint32_t current_tx_nb,
                      uint8_t *data, 
                      srslte_cqi_value_t *cqi_value, 
                      srslte_uci_data_t *uci_data, 
                      uint32_t tti);
#endif
 } 
}
#endif //EMU_SRSENB_PHY_ADAPTER_H
