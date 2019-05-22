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
namespace phy_adapter {

void ue_initialize(srslte::log * log_h, 
                   uint32_t sf_interval,
                   EMANELTE::MHAL::mhal_config_t & mhal_config);

void ue_start();

void ue_stop();

void ue_set_frequencies(float ul_freq, float dl_freq, uint32_t earfcn);

void ue_set_bandwidth(int n_prb);

void ue_set_prach_freq_offset(uint32_t freq_offset);

void ue_set_crnti(uint16_t rnti);

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
int ue_dl_system_frame_search(srslte_ue_sync_t * ue_sync, uint32_t * tti);

// 4 syncd search
int ue_dl_sync_search(srslte_ue_sync_t * ue_sync, uint32_t tti);

// decode signal
float ue_dl_decode_signal(uint32_t cell_id);

// get dl dci
int ue_dl_find_dl_dci(srslte_ue_dl_t*            q,
                             srslte_dl_sf_cfg_t* sf,
                             srslte_ue_dl_cfg_t* cfg,
                             uint16_t            rnti,
                             srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG]);

#if 0
// get ul dci
int ue_dl_find_ul_dci(srslte_ue_dl_t *q, 
                      uint16_t rnti, 
                      srslte_dci_msg_t *dci_msg);

#endif


// convert ota grant to mac action
void ue_dl_decode_pdsch(srsue::mac_interface_phy::tb_action_dl_t * dl_action,
                        bool acks[SRSLTE_MAX_CODEWORDS]);

#if 0
// get phich
bool ue_dl_decode_phich(srslte_ue_dl_t * q,
                        uint32_t sfn,
                        uint16_t rnti,
                        uint32_t n_prb_L,
                        uint32_t n_dmrs);
#endif

#if 0
// get pmch
bool ue_dl_decode_pmch(srslte_ue_dl_t * q, 
                       uint16_t areaid,
                       uint8_t * payload);
#endif


// tx init
void ue_ul_tx_init();

// send to mhal with sot
void ue_ul_send_signal(time_t sot_secs, float frac_sec, const srslte_cell_t & cell);

// set prach
void ue_ul_put_prach(int index);


#if 0
// set pucch, pusch
// see lib/src/phy/ue/ue_ul.c
int ue_ul_encode(srslte_ue_ul_t* q, srslte_ul_sf_cfg_t* sf, srslte_ue_ul_cfg_t* cfg, srslte_pusch_data_t* data);
#endif

/* OLD API
bool ue_ul_put_pucch(srslte_ue_ul_t * q,
                     srslte_uci_data_t * uci,
                     uint32_t ncce);

bool ue_ul_put_pusch(uint16_t rnti,
                     srslte_ra_ul_grant_t *grant,
                     srslte_uci_data_t * uci,
                     uint8_t *payload);
*/

} // end namespace phy_adapter
} // end namespace srsue

#endif //EMU_SRSUE_PHY_ADAPTER_H
