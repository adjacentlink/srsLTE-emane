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

#ifndef SRSLTE_RLC_INTERFACE_H
#define SRSLTE_RLC_INTERFACE_H

// for custom constructors
#include "srslte/asn1/rrc_asn1.h"

namespace srslte {

typedef enum{
  RLC_MODE_TM = 0,
  RLC_MODE_UM,
  RLC_MODE_AM,
  RLC_MODE_N_ITEMS,
}rlc_mode_t;
static const char rlc_mode_text[RLC_MODE_N_ITEMS][20] = {"Transparent Mode",
                                                         "Unacknowledged Mode",
                                                         "Acknowledged Mode"};

typedef enum {
  RLC_UMD_SN_SIZE_5_BITS = 0,
  RLC_UMD_SN_SIZE_10_BITS,
  RLC_UMD_SN_SIZE_N_ITEMS,
} rlc_umd_sn_size_t;
static const char     rlc_umd_sn_size_text[RLC_UMD_SN_SIZE_N_ITEMS][20] = {"5 bits", "10 bits"};
static const uint16_t rlc_umd_sn_size_num[RLC_UMD_SN_SIZE_N_ITEMS]      = {5, 10};

typedef struct {
  /****************************************************************************
   * Configurable parameters
   * Ref: 3GPP TS 36.322 v10.0.0 Section 7
   ***************************************************************************/

  // TX configs
  int32_t  t_poll_retx;     // Poll retx timeout (ms)
  int32_t  poll_pdu;        // Insert poll bit after this many PDUs
  int32_t  poll_byte;       // Insert poll bit after this much data (KB)
  uint32_t max_retx_thresh; // Max number of retx

  // RX configs
  int32_t t_reordering;      // Timer used by rx to detect PDU loss  (ms)
  int32_t t_status_prohibit; // Timer used by rx to prohibit tx of status PDU (ms)
} srslte_rlc_am_config_t;

typedef struct {
  /****************************************************************************
   * Configurable parameters
   * Ref: 3GPP TS 36.322 v10.0.0 Section 7
   ***************************************************************************/

  int32_t           t_reordering;       // Timer used by rx to detect PDU loss  (ms)
  rlc_umd_sn_size_t tx_sn_field_length; // Number of bits used for tx (UL) sequence number
  rlc_umd_sn_size_t rx_sn_field_length; // Number of bits used for rx (DL) sequence number

  uint32_t rx_window_size;
  uint32_t rx_mod; // Rx counter modulus
  uint32_t tx_mod; // Tx counter modulus
  bool     is_mrb; // Whether this is a multicast bearer
} srslte_rlc_um_config_t;

class srslte_rlc_config_t
{
public:
  rlc_mode_t               rlc_mode;
  srslte_rlc_am_config_t    am;
  srslte_rlc_um_config_t    um;

  // Default ctor
  srslte_rlc_config_t(): rlc_mode(RLC_MODE_TM), am(), um() {};

  // Constructor based on rrc_asn1's RLC config
  srslte_rlc_config_t(asn1::rrc::rlc_cfg_c* cnfg) : rlc_mode(RLC_MODE_AM), am(), um()
  {
    // update RLC mode to internal mode struct
    rlc_mode = (cnfg->type() == asn1::rrc::rlc_cfg_c::types::am) ? RLC_MODE_AM : RLC_MODE_UM;

    switch(rlc_mode)
    {
      case RLC_MODE_AM:
        am.t_poll_retx       = cnfg->am().ul_am_rlc.t_poll_retx.to_number();
        am.poll_pdu          = cnfg->am().ul_am_rlc.poll_pdu.to_number();
        am.poll_byte         = cnfg->am().ul_am_rlc.poll_byte.to_number() * 1000; // KB
        am.max_retx_thresh   = cnfg->am().ul_am_rlc.max_retx_thres.to_number();
        am.t_reordering      = cnfg->am().dl_am_rlc.t_reordering.to_number();
        am.t_status_prohibit = cnfg->am().dl_am_rlc.t_status_prohibit.to_number();
        break;
      case RLC_MODE_UM:
        um.t_reordering        = cnfg->um_bi_dir().dl_um_rlc.t_reordering.to_number();
        um.rx_sn_field_length  = (rlc_umd_sn_size_t)cnfg->um_bi_dir().dl_um_rlc.sn_field_len.value;
        um.rx_window_size      = (RLC_UMD_SN_SIZE_5_BITS == um.rx_sn_field_length) ? 16 : 512;
        um.rx_mod              = (RLC_UMD_SN_SIZE_5_BITS == um.rx_sn_field_length) ? 32 : 1024;
        um.tx_sn_field_length  = (rlc_umd_sn_size_t)cnfg->um_bi_dir().ul_um_rlc.sn_field_len.value;
        um.tx_mod              = (RLC_UMD_SN_SIZE_5_BITS == um.tx_sn_field_length) ? 32 : 1024;
        break;
        //    case asn1::rrc::rlc_cfg_c::types::um_uni_dir_ul:
        //      um.tx_sn_field_length = (rlc_umd_sn_size_t)cnfg->um_uni_dir_ul().ul_um_rlc.sn_field_len.value;
        //      um.tx_mod             = (RLC_UMD_SN_SIZE_5_BITS == um.tx_sn_field_length) ? 32 : 1024;
        //      break;
        //    case asn1::rrc::rlc_cfg_c::types::um_uni_dir_dl:
        //      um.t_reordering       = cnfg->um_uni_dir_dl().dl_um_rlc.t_reordering.to_number();
        //      um.rx_sn_field_length = (rlc_umd_sn_size_t)cnfg->um_uni_dir_dl().dl_um_rlc.sn_field_len.value;
        //      um.rx_window_size     = (RLC_UMD_SN_SIZE_5_BITS == um.rx_sn_field_length) ? 16 : 512;
        //      um.rx_mod             = (RLC_UMD_SN_SIZE_5_BITS == um.rx_sn_field_length) ? 32 : 1024;
        //      break;
      default:
        // Handle default case
        break;
    }
  }

  // Factory for MCH
  static srslte_rlc_config_t mch_config()
  {
    srslte_rlc_config_t cfg;
    cfg.rlc_mode               = RLC_MODE_UM;
    cfg.um.t_reordering        = 0;
    cfg.um.rx_sn_field_length  = RLC_UMD_SN_SIZE_5_BITS;
    cfg.um.rx_window_size      = 0;
    cfg.um.rx_mod              = 32;
    cfg.um.tx_sn_field_length  = RLC_UMD_SN_SIZE_5_BITS;
    cfg.um.tx_mod              = 32;
    cfg.um.is_mrb              = true;
    return cfg;
  }
};

} // namespace srslte

#endif // SRSLTE_RLC_INTERFACE_H
