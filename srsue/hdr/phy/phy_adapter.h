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

#ifndef EMU_SRSUE_PHY_ADAPTER_H
#define EMU_SRSUE_PHY_ADAPTER_H

#include "srslte/config.h"
#include "srslte/common/log.h"
#include "srslte/srslte.h"
#include "srslte/phy/ue/ue_dl.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "libemanelte/mhalconfig.h"

#include <string>

namespace srsue {
class sync;
namespace phy_adapter {

void ue_initialize(srslte::log * log_h, 
                   uint32_t sf_interval,
                   EMANELTE::MHAL::mhal_config_t & mhal_config);

void ue_start();

void ue_stop();

void ue_set_frequency(uint32_t cc_idx,
                      double rx_freq_hz,
                      double tx_freq_hz);

void ue_set_cell(const phy_interface_rrc_lte::phy_cell_t* cell);

void ue_set_earfcn(double rx_freq_hz,
                   double tx_freq_hz,
                   uint32_t earfcn);

void ue_set_bandwidth(int n_prb);

void ue_set_prach_freq_offset(uint32_t freq_offset);

void ue_set_crnti(uint16_t rnti);

void ue_set_sync(sync * sync);

// rx frame for this tti, common to all (4) states below
int ue_dl_read_frame(srslte_timestamp_t* rx_time);


// 1 cell cearch
int ue_dl_cellsearch_scan(srslte_ue_cellsearch_t * cs,
                          srslte_ue_cellsearch_result_t * fc,
                          int force_nid_2,
                          uint32_t *max_peak);

// 2 mib search 
int ue_dl_mib_search(const srslte_ue_cellsearch_t * cs,
                     srslte_ue_mib_sync_t * ue_mib_sync,
                     srslte_cell_t * cell);

// 3 sfn search 
int ue_dl_system_frame_search(srslte_ue_sync_t * ue_sync, 
                              uint32_t * tti);

// 4 syncd search
int ue_dl_sync_search(srslte_ue_sync_t * ue_sync,
                      uint32_t tti);

// get rssi
float ue_dl_get_rssi(uint32_t cell_id, uint32_t cc_idx);

// get dl dci
int ue_dl_cc_find_dl_dci(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_ue_dl_cfg_t* cfg,
                         uint16_t            rnti,
                         srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG],
                         uint32_t            cc_idx);

// get ul dci
int ue_dl_cc_find_ul_dci(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_ue_dl_cfg_t* cfg,
                         uint16_t            rnti,
                         srslte_dci_ul_t     dci_ul[SRSLTE_MAX_DCI_MSG],
                         uint32_t cc_idx);

// decode pdsch
int ue_dl_cc_decode_pdsch(srslte_ue_dl_t*     q,
                          srslte_dl_sf_cfg_t* sf,
                          srslte_pdsch_cfg_t* pdsch_cfg,
                          srslte_pdsch_res_t  data[SRSLTE_MAX_CODEWORDS],
                          uint32_t cc_idx);

// get phich
int ue_dl_cc_decode_phich(srslte_ue_dl_t*       q,
                          srslte_dl_sf_cfg_t*   sf,
                          srslte_ue_dl_cfg_t*   cfg,
                          srslte_phich_grant_t* grant,
                          srslte_phich_res_t*   result,
                          uint16_t rnti,
                          uint32_t cc_idx);


// get pmch
int ue_dl_cc_decode_pmch(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_pmch_cfg_t*  pmch_cfg,
                         srslte_pdsch_res_t  data[SRSLTE_MAX_CODEWORDS],
                         uint32_t cc_idx);


// tx init
void ue_ul_tx_init();

// send to mhal with sot
void ue_ul_send_signal(time_t sot_secs, float frac_sec, const srslte_cell_t & cell);

// set prach
void ue_ul_put_prach(int index);

// set pucch, pusch
int ue_ul_encode(srslte_ue_ul_t* q, srslte_ul_sf_cfg_t* sf, srslte_ue_ul_cfg_t* cfg, srslte_pusch_data_t* data, uint32_t cc_idx);

} // end namespace phy_adapter
} // end namespace srsue

#endif //EMU_SRSUE_PHY_ADAPTER_H
