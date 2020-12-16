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

#include "srslte/common/log.h"
#include "srslte/config.h"

#ifdef PHY_ADAPTER_ENABLE

extern "C" {
#include "srslte/phy/phch/ra.h"
#include "srslte/phy/phch/dci.h"
#include "srslte/phy/phch/phich.h"
#include "srslte/phy/phch/pucch.h"
}

#include "lib/include/srslte/phy/phch/pdsch_cfg.h"
#include "srsenb/hdr/phy/phy_adapter.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"

#include "libemanelte/mhalenb.h"
#include "libemanelte/enbstatisticmanager.h"

#include <vector>
#include <set>


// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
  EMANELTE::MHAL::ENB_DL_Message   enb_dl_msg_;
  EMANELTE::MHAL::TxControlMessage tx_control_;
  EMANELTE::MHAL::DownlinkMessage * downlink_control_message_;

  typedef std::pair<EMANELTE::MHAL::UE_UL_Message, EMANELTE::MHAL::RxControl> UE_UL_Message;
  typedef std::vector<UE_UL_Message> UE_UL_Messages;

  UE_UL_Messages ue_ul_msgs_;

  uint64_t tx_seqnum_ = 0;
  uint8_t  my_pci_    = 0;
  uint32_t curr_tti_  = 0;
  uint32_t tti_tx_    = 0;

  uint32_t pdcch_ref_ = 0;
  uint32_t pdsch_ref_ = 0;

  // referenceSignalPower as set by sib.conf sib2.rr_config_common_sib.pdsch_cnfg.rs_power
  float pdsch_rs_power_milliwatt_ = 0.0;

  // scaling between pdsch res in symbols with reference signals to symbols without reference signals
  float pdsch_rho_b_over_rho_a_ = 1.0;

  // scaling between reference signal res and pdsch res in symbols without reference signals, by tti and rnti
  typedef std::map<uint16_t, float> rho_a_db_map_t; // map of rnti to rho_a
  rho_a_db_map_t rho_a_db_map_[10];                 // vector of rho_a maps by subframe number

  // cyclic prefix normal or extended for this cell
  srslte_cp_t cell_cp_ = SRSLTE_CP_NORM;

  inline bool rnti_is_user_i(uint32_t rnti)
   {
     return(rnti == SRSLTE_SIRNTI || 
            rnti == SRSLTE_PRNTI  || 
           (rnti >= SRSLTE_RARNTI_START && rnti <= SRSLTE_RARNTI_END));
   }

  pthread_mutex_t dl_mutex_;
  pthread_mutex_t ul_mutex_;

  srslte::log *log_h_ = NULL;

  const uint8_t zeros_[0xffff] = {0};

  inline std::string GetDebugString(const std::string & str)
   {
#ifdef ENABLE_DEBUG_STRING
       return str;
#else
       return "";
#endif
   }


  void
  initDownlinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channel_message,
                             EMANELTE::MHAL::CHANNEL_TYPE ctype,
                             EMANELTE::MHAL::MOD_TYPE modType,
                             uint16_t rnti,
                             uint32_t infoBits,
                             float txPowerScaledB = 0.0)
  {
    channel_message->set_channel_type(ctype);

    channel_message->set_modulation_type(modType);

    channel_message->set_number_of_bits(infoBits);

    channel_message->set_tx_power_scale_db(txPowerScaledB);

    if(rnti)
      channel_message->set_rnti(rnti);
  }

  inline int bits_to_bytes(int bits) { return bits/8; }
}

#define Error(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h_->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (SRSLTE_DEBUG_ENABLED) log_h_->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (SRSLTE_DEBUG_ENABLED) log_h_->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h_->debug(fmt, ##__VA_ARGS__)

#undef DEBUG_HEX

#ifdef DEBUG_HEX
#define InfoHex(p,l,fmt, ...)    if(log_h_) log_h_->info_hex((const uint8_t*)p, l, fmt, ##__VA_ARGS__)
#endif


namespace srsenb {
namespace phy_adapter {

static EMANELTE::MHAL::DCI_FORMAT convert(srslte_dci_format_t format)
 {
   switch(format)
    {
      case SRSLTE_DCI_FORMAT0:
        return(EMANELTE::MHAL::DCI_FORMAT_0);

      case SRSLTE_DCI_FORMAT1:
        return(EMANELTE::MHAL::DCI_FORMAT_1);

      case SRSLTE_DCI_FORMAT1A:
        return(EMANELTE::MHAL::DCI_FORMAT_1A);

      case SRSLTE_DCI_FORMAT1C:
        return(EMANELTE::MHAL::DCI_FORMAT_1C);

      case SRSLTE_DCI_FORMAT1B:
        return(EMANELTE::MHAL::DCI_FORMAT_1B);

      case SRSLTE_DCI_FORMAT1D:
        return(EMANELTE::MHAL::DCI_FORMAT_1D);

      case SRSLTE_DCI_FORMAT2:
        return(EMANELTE::MHAL::DCI_FORMAT_2);

      case SRSLTE_DCI_FORMAT2A:
        return(EMANELTE::MHAL::DCI_FORMAT_2A);

      case SRSLTE_DCI_FORMAT2B:
        return(EMANELTE::MHAL::DCI_FORMAT_2B);

      default:
       throw("MHAL:convert: invalid dci format");

      return (EMANELTE::MHAL::DCI_FORMAT_ERR);
   }
}

static EMANELTE::MHAL::MOD_TYPE convert(srslte_mod_t type)
{
   switch(type)
    {
       case SRSLTE_MOD_BPSK: 
         return (EMANELTE::MHAL::MOD_BPSK);

       case SRSLTE_MOD_QPSK:
         return (EMANELTE::MHAL::MOD_QPSK);

       case SRSLTE_MOD_16QAM:
         return (EMANELTE::MHAL::MOD_16QAM);

       case SRSLTE_MOD_64QAM:
         return (EMANELTE::MHAL::MOD_64QAM);

       default:
         throw("MHAL:convert: invalid mod type");

       return (EMANELTE::MHAL::MOD_ERR);
    }
}

/*
typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t;

typedef struct SRSLTE_API {
  uint32_t k[4];
  uint32_t k0;
  uint32_t l;
  bool     assigned;
}srslte_regs_reg_t;

typedef struct SRSLTE_API {
  uint32_t          nof_regs;
  srslte_regs_reg_t **regs;
}srslte_regs_ch_t;

typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t      max_ctrl_symbols;
  uint32_t      ngroups_phich;
  uint32_t      ngroups_phich_m1;

  srslte_phich_r_t      phich_res;
  srslte_phich_length_t phich_len;
  
  srslte_regs_ch_t  pcfich;
  srslte_regs_ch_t* phich; 
  srslte_regs_ch_t  pdcch[3];

  uint32_t           phich_mi;
  uint32_t           nof_regs;
  srslte_regs_reg_t* regs;
}srslte_regs_t;

typedef struct SRSLTE_API {
  srslte_cell_t  cell;
  uint32_t       nof_regs[3];
  uint32_t       nof_cce[3];
  uint32_t       max_bits;
  uint32_t       nof_rx_antennas;
  bool           is_ue;
  srslte_regs_t* regs;
  float          rm_f[3*(SRSLTE_DCI_MAX_BITS + 16)];
  float*         llr;
  srslte_modem_table_t mod;
  srslte_sequence_t    seq[SRSLTE_NOF_SF_X_FRAME];
  srslte_viterbi_t     decoder;
  srslte_crc_t         crc;
} srslte_pdcch_t;

typedef struct SRSLTE_API {
  uint8_t               payload[SRSLTE_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srslte_dci_location_t location;
  srslte_dci_format_t   format;
  uint16_t              rnti;
} srslte_dci_msg_t;

typedef struct SRSLTE_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srslte_dci_location_t;

typedef struct SRSLTE_API {
  uint32_t mcs_idx;
  int      rv;
  bool     ndi;
  uint32_t cw_idx;
} srslte_dci_tb_t; */

// see lib/src/phy/phch/pdcch.c
#define PDCCH_NOF_FORMATS               4
#define PDCCH_FORMAT_NOF_CCE(i)          (1<<i)
#define PDCCH_FORMAT_NOF_REGS(i)        ((1<<i)*9)
#define PDCCH_FORMAT_NOF_BITS(i)        ((1<<i)*72)

#define NOF_CCE(cfi)  ((cfi>0&&cfi<4)?q->pdcch.nof_cce [cfi-1]:0)
#define NOF_REGS(cfi) ((cfi>0&&cfi<4)?q->pdcch.nof_regs[cfi-1]:0)

// srslte_pdcch_encode(&q->pdcch, &q->dl_sf, &dci_msg, q->sf_symbols)
static int enb_dl_put_dl_pdcch_i(const srslte_enb_dl_t * q,
                                 const srslte_dci_msg_t * dci_msg,
                                 uint32_t ref,
                                 int type) // 0 for DL, 1 for UL
 {
   const auto rnti = dci_msg->rnti;

   // see lib/src/phy/phch/regs.c int srslte_regs_pdcch_put_offset(srslte_regs_t *h, 
   //                                                              uint32_t cfi, 
   //                                                              uint32_t start_reg,
   //                                                              uint32_t nof_regs)
   const uint32_t nof_regs = PDCCH_FORMAT_NOF_REGS(dci_msg->location.L);
   uint32_t start_reg      = dci_msg->location.ncce * 9;

   // see lib/src/phy/phch/pdcch.c srslte_pdcch_encode(srslte_pdcch_t*     q,
   //                                                  srslte_dl_sf_cfg_t* sf,
   //                                                  srslte_dci_msg_t*   msg,
   if(!((dci_msg->location.ncce + PDCCH_FORMAT_NOF_CCE(dci_msg->location.L) <= NOF_CCE(q->dl_sf.cfi)) &&
        (dci_msg->nof_bits < (SRSLTE_DCI_MAX_BITS - 16)))) 
    {
      Info("PDCCH:%s type %s, rnti 0x%hx, cfi %d, illegal dci msg, ncce %d, format_ncce %d, cfi_ncce %d, nof_bits %d, max_bits %d\n", 
             __func__,
             type ? "UL" : "DL",
             rnti,
             q->dl_sf.cfi,
             dci_msg->location.ncce, 
             PDCCH_FORMAT_NOF_CCE(dci_msg->location.L),
             NOF_CCE(q->dl_sf.cfi),
             dci_msg->nof_bits,
             (SRSLTE_DCI_MAX_BITS - 16));

      // XXX TODO
      // srslte p/r #299 amd issue #347 temp fix set start_reg to 0
      start_reg = 0;
    }

   const uint32_t regs_len = start_reg + nof_regs;

   if(regs_len > NOF_REGS(q->dl_sf.cfi))
    {
      Warning("PDCCH:%s type %s, rnti 0x%hx, cfi %d, pdccd->nof_regs %d, regs_len %u, ncce %d -> start_reg %d, L %d -> nof_regs %d\n", 
            __func__,
            type ? "UL" : "DL",
            rnti,
            q->dl_sf.cfi,
            NOF_REGS(q->dl_sf.cfi),
            regs_len,
            dci_msg->location.ncce, 
            start_reg,
            dci_msg->location.L,
            nof_regs);

      return SRSLTE_ERROR;
   }

  auto pdcch_message   = enb_dl_msg_.add_pdcch();
  auto channel_message = downlink_control_message_->add_pdcch();

  initDownlinkChannelMessage(channel_message,
                             EMANELTE::MHAL::CHAN_PDCCH,
                             EMANELTE::MHAL::MOD_QPSK,
                             rnti,
                                dci_msg->nof_bits);


  for(uint32_t i = start_reg; i < regs_len; ++i)
   {
    const auto reg = q->pdcch.regs->pdcch[q->dl_sf.cfi-1].regs[i];

    if(reg)
     {
       const uint32_t  k0 = reg->k0;
       const uint32_t  l  = reg->l;
       const uint32_t* k  = &reg->k[0];

       const uint32_t rb = k0 / 12;

       Debug("PDCCH DCI group sf_idx=%d, reg=%d, rnti=%d placement: "
             "(l=%u, "
             "k0=%u, "
             "k[0]=%u "
             "k[1]=%u "
             "k[2]=%u "
             "k[3]=%u) in rb=%u\n", tti_tx_ % 10, i, rnti, l, k0, k[0], k[1], k[2], k[3], rb);

       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
     }
   }

  if(type == 0)
   {
     // dl dci
     auto dl_dci_message = pdcch_message->mutable_dl_dci();

     dl_dci_message->set_rnti(dci_msg->rnti);
     dl_dci_message->set_refid(pdcch_ref_++);

     // dci msg
     auto dl_dci_msg = dl_dci_message->mutable_dci_msg();

     dl_dci_msg->set_num_bits(dci_msg->nof_bits);
     dl_dci_msg->set_l_level(dci_msg->location.L);
     dl_dci_msg->set_l_ncce(dci_msg->location.ncce);
     dl_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     dl_dci_msg->set_format(convert(dci_msg->format));

#ifdef DEBUG_HEX
     InfoHex(dci_msg->payload, dci_msg->nof_bits,
             "PDCCH_DL:%s rnti=0x%hx, refid %d, nof_bits %d\n",
             __func__, rnti, pdcch_ref_ - 1, dci_msg->nof_bits);
#endif
   }
  else
   {
     // ul dci
     auto dl_dci_message = pdcch_message->mutable_ul_dci();

     dl_dci_message->set_rnti(dci_msg->rnti);

     // dci msg
     auto ul_dci_msg = dl_dci_message->mutable_dci_msg();

     ul_dci_msg->set_num_bits(dci_msg->nof_bits);
     ul_dci_msg->set_l_level(dci_msg->location.L);
     ul_dci_msg->set_l_ncce(dci_msg->location.ncce);
     ul_dci_msg->set_data(dci_msg->payload, dci_msg->nof_bits);
     ul_dci_msg->set_format(convert(dci_msg->format));

#ifdef DEBUG_HEX
     InfoHex(dci_msg->payload, dci_msg->nof_bits,
             "PDCCH_UL:%s rnti=0x%hx, nof_bits %d\n",
             __func__, rnti, dci_msg->nof_bits);
#endif
   }

  return SRSLTE_SUCCESS;
}

// lib/src/phy/phch/pdsch.c
// srslte_pdsch_encode(srslte_pdsch_t* q, 
//                     srslte_dl_sf_cfg_t* sf, 
//                     srslte_pdsch_cfg_t* cfg, 
//                     uint8_t*data[SRSLTE_MAX_CODEWORDS] ...);

/*
typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srslte_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srslte_dl_sf_cfg_t;

typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srslte_softbuffer_tx_t* tx[SRSLTE_MAX_CODEWORDS];
    srslte_softbuffer_rx_t* rx[SRSLTE_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pdsch_cfg_t;

typedef struct SRSLTE_API {
  srslte_tx_scheme_t tx_scheme;
  uint32_t           pmi;
  bool               prb_idx[2][SRSLTE_MAX_PRB];
  uint32_t           nof_prb;
  uint32_t           nof_re;
  uint32_t           nof_symb_slot[2];
  srslte_ra_tb_t     tb[SRSLTE_MAX_CODEWORDS];
  int                last_tbs[SRSLTE_MAX_CODEWORDS];
  uint32_t           nof_tb;
  uint32_t           nof_layers;
} srslte_pdsch_grant_t;

typedef struct SRSLTE_API {
  srslte_mod_t mod;
  int          tbs;
  int          rv;
  uint32_t     nof_bits;
  uint32_t     cw_idx;
  bool         enabled;
  // this is for debugging and metrics purposes
  uint32_t mcs_idx;
} srslte_ra_tb_t; */

// set pdsch dl
static int enb_dl_put_dl_pdsch_i(const srslte_enb_dl_t * q,
                                 srslte_pdsch_cfg_t* pdsch, 
                                 uint8_t* data,
                                 uint32_t ref,
                                 uint32_t tb)
 {
   const auto grant = pdsch->grant;
   const auto rnti  = pdsch->rnti;

   const uint32_t sf_idx = (tti_tx_ % 10);

   float rho_a_db = 0.0;

   const auto riter = rho_a_db_map_[sf_idx].find(rnti);

   if(riter != rho_a_db_map_[sf_idx].end())
    {
     rho_a_db = riter->second;
    }

   auto channel_message = downlink_control_message_->add_pdsch();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PDSCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs,
                              rho_a_db);

   // Add resource block assignment from the phy_grant
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
    {
      if(grant.prb_idx[0][rb])
       {
         channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
       }

      if(grant.prb_idx[1][rb])
       {
         channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
       }
    }

   auto pdsch_message = enb_dl_msg_.mutable_pdsch();

   // pdsch data
   auto data_message = pdsch_message->add_data();

   data_message->set_refid(pdsch_ref_++);
   data_message->set_tb(tb);
   data_message->set_tbs(grant.tb[tb].tbs);
   data_message->set_data(data, bits_to_bytes(grant.tb[tb].tbs));
   
   ENBSTATS::putDLGrant(rnti);

#ifdef DEBUG_HEX
   InfoHex(data, bits_to_bytes(grant.tb[tb].tbs),
           "PDSCH:%s rnti=0x%hx, refid %d, tbs %d\n",
           __func__, rnti, pdsch_ref_ - 1, grant.tb[tb].tbs);
#endif

   return SRSLTE_SUCCESS;
}


/*
typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t;

typedef struct SRSLTE_API {
  srslte_pdsch_cfg_t pdsch_cfg;
  uint16_t           area_id;
} srslte_pmch_cfg_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srslte_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srslte_dl_sf_cfg_t;

typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srslte_softbuffer_tx_t* tx[SRSLTE_MAX_CODEWORDS];
    srslte_softbuffer_rx_t* rx[SRSLTE_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pdsch_cfg_t; */

static int enb_dl_put_pmch_i(const srslte_enb_dl_t * q,
                            srslte_pmch_cfg_t* pmch_cfg,
                            uint8_t* data,
                            uint16_t rnti)
 {
   const auto pdsch_cfg = pmch_cfg->pdsch_cfg;

   const auto grant = pdsch_cfg.grant;

   if(grant.nof_tb != 1)
    {
      Error("PMCH:%s rnti 0x%hx, nof_tb %u, expected 1\n", __func__, rnti, grant.nof_tb);

      return SRSLTE_ERROR;
    }

   const uint32_t tb = 0;

   // pmch
   auto pmch_message = enb_dl_msg_.mutable_pmch();

   pmch_message->set_area_id(pmch_cfg->area_id);
   pmch_message->set_tbs(grant.tb[tb].tbs);
   pmch_message->set_rnti(rnti);
   pmch_message->set_data(data ? data : zeros_, grant.tb[tb].tbs);

   auto channel_message = downlink_control_message_->mutable_pmch();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PMCH,
                              convert(grant.tb[tb].mod),
                              rnti,
                              grant.tb[tb].tbs);

   // channel_message.add_resource_blocks();
   for(uint32_t rb = 0; rb < q->cell.nof_prb; ++rb)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
     }

#ifdef DEBUG_HEX
   InfoHex(data, grant.tb[tb].tbs,
           "PMCH:%s rnti=0x%hx, area_id %d, tbs %d\n",
           __func__, rnti, pmch_cfg->area_id, grant.tb[tb].tbs);
#endif

   return SRSLTE_SUCCESS;
}


// BEGIN phy_adapter enb api

void enb_initialize(srslte::log * log_h, 
                    uint32_t sf_interval_msec, 
                    uint32_t physical_cell_id, 
                    srslte_cp_t cp,
                    float ul_freq,
                    float dl_freq, 
                    int n_prb, 
                    EMANELTE::MHAL::mhal_config_t & mhal_config,
                    rrc_cfg_t * rrc_cfg)
{
  log_h_ = log_h;

  pdsch_rs_power_milliwatt_ = pow(10.0, static_cast<float>(rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr) / 10.0);

  uint8_t p_b = rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b;

#if 0 // TODO
  if(p_b < 4)
    {
      // this table no longer resides in lib/include/srslte/phy/phch/pdsch_cfg.h
      // new location unknown
      pdsch_rho_b_over_rho_a_ = pdsch_cfg_cell_specific_ratio_table[0][p_b];
    }
#endif

  cell_cp_ = cp;

  Info("INIT:%s sf_interval "
       "%u msec, "
       "ul_freq %6.4f MHz, "
       "fl_freq %6.4f MHz, "
       "n_prb %d, "
       "rs_power=%d "
       "pdsch_rs_power_milliwatt=%0.2f "
       "p_b=%d "
       "pdsch_rho_b_over_rho_a=%.02f\n",
       __func__,
       sf_interval_msec,
       ul_freq/1e6,
       dl_freq/1e6,
       n_prb,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr,
       pdsch_rs_power_milliwatt_,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b,
       pdsch_rho_b_over_rho_a_);

  EMANELTE::MHAL::ENB::initialize(
     mhal_config,
     EMANELTE::MHAL::ENB::mhal_enb_config_t(physical_cell_id,
                                            sf_interval_msec,
                                            cp == SRSLTE_CP_NORM ? SRSLTE_CP_NORM_NSYMB : SRSLTE_CP_EXT_NSYMB,
                                            ul_freq,
                                            dl_freq,
                                            n_prb,
                                            pdsch_rs_power_milliwatt_,
                                            pdsch_rho_b_over_rho_a_));
}


void enb_start()
{
  Info("INIT:%s\n", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("INIT:%s pthread_mutexattr_init error %s, exit\n", __func__, strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("INIT:%s pthread_mutexattr_setprotocol error %s, exit\n", __func__, strerror(errno));
         exit(1);
       }

     if(pthread_mutex_init(&dl_mutex_, &mattr) < 0)
       {
         Error("INIT:%s pthread_mutex_init error %s, exit\n", __func__, strerror(errno));
         exit(1);
       }

     if(pthread_mutex_init(&ul_mutex_, &mattr) < 0)
       {
         Error("INIT:%s pthread_mutex_init error %s, exit\n", __func__, strerror(errno));
         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  enb_dl_msg_.Clear();

  tx_control_.Clear();

  downlink_control_message_ = tx_control_.mutable_downlink();

  downlink_control_message_->Clear();

  EMANELTE::MHAL::ENB::start();
}


void enb_stop()
{
  Info("STOP:%s\n", __func__);

  EMANELTE::MHAL::ENB::stop();

  pthread_mutex_destroy(&dl_mutex_);
  pthread_mutex_destroy(&ul_mutex_);
}


void enb_dl_tx_init(const srslte_enb_dl_t *q,
                    uint32_t tti_tx,
                    uint32_t cfi)
{
  // lock here, unlocked after tx_end to prevent any worker thread(s)
  // from attempting to start a new tx sequence before the current tx sequence
  // is finished
  pthread_mutex_lock(&dl_mutex_);

  enb_dl_msg_.Clear();

  tx_control_.Clear();

  downlink_control_message_ = tx_control_.mutable_downlink();

  downlink_control_message_->Clear();
  downlink_control_message_->set_num_resource_blocks(q->cell.nof_prb);

  // subframe index
  const uint32_t sf_idx = (tti_tx % 10);

  rho_a_db_map_[sf_idx].clear();

  // always set tti for timing tracking
  enb_dl_msg_.set_tti(tti_tx);

  // always set cfi
  // note - cfi should be nof_ctrl_symbols on regular frames and
  //        non_mbsfn_region_length (from sib13) on mbsfn frames
  enb_dl_msg_.set_cfi(cfi);
  downlink_control_message_->set_cfi(enb_dl_msg_.cfi());

  // always set phy_cell_id to distinguish multiple cells
  enb_dl_msg_.set_phy_cell_id(q->cell.id);
  tx_control_.set_phy_cell_id(q->cell.id);

  // save our pci
  my_pci_ = q->cell.id;

  // save the tti_tx
  tti_tx_ = tti_tx;

  // PCFICH encoding
  EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->mutable_pcfich();

  initDownlinkChannelMessage(channel_message,
                             EMANELTE::MHAL::CHAN_PCFICH,
                             EMANELTE::MHAL::MOD_QPSK,
                             0,
                             2);  // 2 bit to encode dfi

  for(int i=0; i<3; ++i)
    {
      const srslte_pcfich_t*   p1  = &q->pcfich;
      const srslte_regs_t*     p2  = p1->regs;
      const srslte_regs_ch_t*  rch = &p2->pcfich;
      const srslte_regs_reg_t* reg = rch->regs[i];

      uint32_t k0 = reg->k0;
      uint32_t l  = reg->l;
      const uint32_t * k = &reg->k[0];

      //srslte_regs_ch_t * pcfich = &((q->pcfich.regs)->pcfich);
      uint32_t rb = k0 / 12;
      Debug("TX:%s PCFICH group i=%d on this subframe placed at resource starting at "
            "(l=%u, "
            "k0=%u, "
            "k[0]=%u "
            "k[1]=%u "
            "k[2]=%u "
            "k[3]=%u) in resource block=%u\n", __func__, i, l, k0, k[0], k[1], k[2], k[3], rb);

      channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
    }

  // Set side chain PSS, SSS and MIB information on appropriate subframes
  if(sf_idx == 0 || sf_idx == 5) 
    {
      // physical cell group and id derived from pci

      // set cyclical prefix mode
      enb_dl_msg_.mutable_pss_sss()->set_cp_mode(q->cell.cp == SRSLTE_CP_NORM ? 
                                                 EMANELTE::MHAL::CP_NORM : 
                                                 EMANELTE::MHAL::CP_EXTD);

      // MIB on first subframe
      if(sf_idx == 0)
       {
         EMANELTE::MHAL::ENB_DL_Message_PBCH * pbch = enb_dl_msg_.mutable_pbch();

         EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->mutable_pbch();

         initDownlinkChannelMessage(channel_message,
                                    EMANELTE::MHAL::CHAN_PBCH,
                                    EMANELTE::MHAL::MOD_QPSK,
                                    0,
                                    40);  // MIB + 16 bit CRC

         // MIB occupies the middle 72 resource elements of the second slot of subframe 0, which
         // is the middle 6 or 7 resource blocks depending on nof_prb being even or odd.
         // Approximate this by sending a segment for each fullly occupied resource block,
         // So 5 blocks when num_prb is odd.
         int first_prb = q->cell.nof_prb / 2 - 3 + (q->cell.nof_prb % 2);

         int num_prb = q->cell.nof_prb % 2 ? 5 : 6;

         for(int i=0; i<num_prb; ++i)
           {
             channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(first_prb + i));
           }

         switch(q->cell.phich_resources) 
          {
            case SRSLTE_PHICH_R_1_6:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_SIXTH);
            break;

            case SRSLTE_PHICH_R_1_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_HALF);
            break;

            case SRSLTE_PHICH_R_1:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE);
            break;

            case SRSLTE_PHICH_R_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_TWO);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_resources type");
          }

         switch(q->cell.phich_length) 
          {
            case SRSLTE_PHICH_NORM:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM);
            break;

            case SRSLTE_PHICH_EXT:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD);
            break;

            default:
             throw("MHAL:enb_dl_put_base: unhandled cell phich_length type");
          }

         pbch->set_num_prb(q->cell.nof_prb);

         pbch->set_num_antennas(q->cell.nof_ports);
      }
   }
}


// send msg to mhal
bool enb_dl_send_signal(time_t sot_sec, float frac_sec)
{
  bool result = false;

  EMANELTE::MHAL::Data data;
  
  if(enb_dl_msg_.SerializeToString(&data))
    {
      tx_control_.set_reference_signal_power_milliwatt(pdsch_rs_power_milliwatt_);

      // align sot to sf time
      const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
      EMANELTE::MHAL::Timestamp * const ts = tx_control_.mutable_sf_time();
      ts->set_ts_sec(tv_sf_time.tv_sec);
      ts->set_ts_usec(tv_sf_time.tv_usec);

      tx_control_.set_message_type(EMANELTE::MHAL::DOWNLINK);
      tx_control_.set_tx_seqnum(tx_seqnum_++);
      tx_control_.set_tti_tx(tti_tx_);

#if 0
      Debug("TX:%s tx_ctrl:%s\n \t\tmsg:%s\n",
           __func__,
           GetDebugString(tx_control_.DebugString()).c_str(),
           GetDebugString(enb_dl_msg_.DebugString()).c_str());
#endif

      EMANELTE::MHAL::ENB::send_msg(data, tx_control_);
    }
  else
    {
      Error("TX:%s SerializeToString ERROR len %zu\n", __func__, data.length());
    }

  pthread_mutex_unlock(&dl_mutex_);

  return result;
}


// XXX TODO this needs review
// set the power scaling on a per rnti basis
void enb_dl_set_power_allocation(uint32_t tti, uint16_t rnti, float rho_a_db, float rho_b_db)
{
  const uint32_t sf_idx = (tti % 10);

  rho_a_db_map_[sf_idx].emplace(rnti, rho_a_db);

  Debug("MHAL:%s "
        "sf_idx %d, "
        "rnti 0x%hx, "
        "rho_a_db %0.2f\n",
        __func__,
        sf_idx,
        rnti,
        rho_a_db);
}

   

/* typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t; 

typedef struct SRSLTE_API {
  uint16_t              rnti;
  srslte_dci_format_t   format;
  srslte_dci_location_t location;

  // Resource Allocation
  srslte_ra_type_t alloc_type;
  union {
    srslte_ra_type0_t type0_alloc;
    srslte_ra_type1_t type1_alloc;
    srslte_ra_type2_t type2_alloc;
  };

  // Codeword information
  srslte_dci_tb_t tb[SRSLTE_MAX_CODEWORDS];
  bool            tb_cw_swap;
  uint32_t        pinfo;

  // Power control
  bool    pconf;
  bool    power_offset;
  uint8_t tpc_pucch;

  // RA order
  bool     is_ra_order;
  uint32_t ra_preamble;
  uint32_t ra_mask_idx;

  // Release 10
  uint32_t cif;
  bool     cif_present;
  bool     srs_request;
  bool     srs_request_present;

  // Other parameters
  uint32_t pid;
  uint32_t dai;
  bool     is_tdd;
  bool     is_dwpts;
  bool     sram_id;
} srslte_dci_dl_t; */

// see lib/src/phy/enb/enb_dl.c 
// int srslte_enb_dl_put_pdcch_dl(srslte_enb_dl_t* q, srslte_dci_cfg_t* dci_cfg, srslte_dci_dl_t* dci_dl)
int enb_dl_put_pdcch_dl_i(srslte_enb_dl_t* q, 
                          srslte_dci_cfg_t* dci_cfg,
                          srslte_dci_dl_t* dci_dl, 
                          uint32_t ref)
{
  srslte_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srslte_dci_msg_pack_pdsch(&q->cell, &q->dl_sf, dci_cfg, dci_dl, &dci_msg) == SRSLTE_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 0); // DL
    }
  else
    {
      Error("PDCCH:%s error calling srslte_dci_msg_pack_pdsch(), ref %u\n", __func__, ref);

      return SRSLTE_ERROR;
    }
}

/*typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t; 

 typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srslte_softbuffer_tx_t* tx[SRSLTE_MAX_CODEWORDS];
    srslte_softbuffer_rx_t* rx[SRSLTE_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pdsch_cfg_t;

 typedef struct {
   srslte_dci_dl_t         dci;
   srslte_dci_cfg_t        dci_cfg;
   uint8_t*                data[SRSLTE_MAX_TB];
   srslte_softbuffer_tx_t* softbuffer_tx[SRSLTE_MAX_TB];
 } dl_sched_grant_t; */
int enb_dl_put_pdcch_dl(srslte_enb_dl_t* q, 
                        srslte_pdsch_cfg_t* pdsch, 
                        mac_interface_phy_lte::dl_sched_grant_t* grant,
                        uint32_t ref)
{
  for(uint32_t tb = 0; tb < SRSLTE_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         Info("PDCCH:%s put tb %d, rnti 0x%hx\n", __func__, tb, grant->dci.rnti);

         if(enb_dl_put_pdcch_dl_i(q, &grant->dci_cfg, &grant->dci, ref))
          {
             Error("PDCCH:%s Error ref %u, tb %u, rnti 0x%hx\n", 
                   __func__, ref, tb, grant->dci.rnti);
          }
       }
   }

   return SRSLTE_SUCCESS;
}


/*typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t; 

 typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
  union {
    srslte_softbuffer_tx_t* tx[SRSLTE_MAX_CODEWORDS];
    srslte_softbuffer_rx_t* rx[SRSLTE_MAX_CODEWORDS];
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pdsch_cfg_t;

 typedef struct {
   srslte_dci_dl_t         dci;
   srslte_dci_cfg_t        dci_cfg;
   uint8_t*                data[SRSLTE_MAX_TB];
   srslte_softbuffer_tx_t* softbuffer_tx[SRSLTE_MAX_TB];
 } dl_sched_grant_t; */

int enb_dl_put_pdsch_dl(srslte_enb_dl_t* q, 
                        srslte_pdsch_cfg_t* pdsch, 
                        mac_interface_phy_lte::dl_sched_grant_t* grant,
                        uint32_t ref)
{
  for(uint32_t tb = 0; tb < SRSLTE_MAX_TB; ++tb)
    {
      // check if data is ready
      if(grant->data[tb])
       {
         Info("PDSCH:%s put tb %d, rnti 0x%hx\n", __func__, tb, grant->dci.rnti);

         if(enb_dl_put_dl_pdsch_i(q, pdsch, grant->data[tb], ref, tb) != SRSLTE_SUCCESS)
           {
             Error("PDSCH:%s Error ref %u, tb %u, rnti 0x%hx\n", 
                   __func__, ref, tb, grant->dci.rnti);
          }
      }
   }

   return SRSLTE_SUCCESS;
}



// see lib/src/phy/enb/enb_dl.c
// int srslte_enb_dl_put_pmch(srslte_enb_dl_t* q, srslte_pmch_cfg_t* pmch_cfg, uint8_t* data)
int enb_dl_put_pmch(srslte_enb_dl_t* q, 
                    srslte_pmch_cfg_t* pmch_cfg, 
                    mac_interface_phy_lte::dl_sched_grant_t* dl_sched_grant)
{
  if(dl_sched_grant->dci.rnti != 0)
   {
     return enb_dl_put_pmch_i(q, pmch_cfg, dl_sched_grant->data[0], dl_sched_grant->dci.rnti);
   }
  else
   {
     Error("PMCH:%s rnti is 0\n", __func__);

     return SRSLTE_ERROR;
   }
}

// see lib/src/phy/enb/enb_dl.c
// int srslte_enb_dl_put_pdcch_ul(srslte_enb_dl_t* q, srslte_dci_cfg_t* dci_cfg, srslte_dci_ul_t* dci_ul)
int enb_dl_put_pdcch_ul(srslte_enb_dl_t* q, 
                        srslte_dci_cfg_t* dci_cfg,
                        srslte_dci_ul_t* dci_ul,
                        uint32_t ref)
{
  srslte_dci_msg_t dci_msg;
  bzero(&dci_msg, sizeof(dci_msg));

  if(srslte_dci_msg_pack_pusch(&q->cell, &q->dl_sf, dci_cfg, dci_ul, &dci_msg) == SRSLTE_SUCCESS)
    {
      return enb_dl_put_dl_pdcch_i(q, &dci_msg, ref, 1); // UL
    }
  else
    {
      Error("PDCCH:%s error calling srslte_dci_msg_pack_pdcch(), ref %u\n", __func__, ref);

      return SRSLTE_ERROR;
    }
}


/* typedef struct SRSLTE_API {
  srslte_cell_t      cell;
  srslte_dl_sf_cfg_t dl_sf;
  srslte_pbch_t      pbch;
  srslte_pcfich_t    pcfich;
  srslte_regs_t      regs;
  srslte_pdcch_t     pdcch;
  srslte_pdsch_t     pdsch;
  srslte_pmch_t      pmch;
  srslte_phich_t     phich;
} srslte_enb_dl_t; 

typedef struct SRSLTE_API {
  uint32_t k[4];
  uint32_t k0;
  uint32_t l;
  bool     assigned;
}srslte_regs_reg_t;

typedef struct SRSLTE_API {
  uint32_t          nof_regs;
  srslte_regs_reg_t **regs;
}srslte_regs_ch_t;

typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t      max_ctrl_symbols;
  uint32_t      ngroups_phich;
  uint32_t      ngroups_phich_m1;

  srslte_phich_r_t      phich_res;
  srslte_phich_length_t phich_len;
  
  srslte_regs_ch_t pcfich;
  srslte_regs_ch_t *phich;   // there are several phich
  srslte_regs_ch_t pdcch[3]; // PDCCH indexing, permutation and interleaving is computed for
                             // the three possible CFI value

  uint32_t           phich_mi;
  uint32_t           nof_regs;
  srslte_regs_reg_t *regs;
}srslte_regs_t;

typedef struct SRSLTE_API {
  srslte_cell_t  cell;
  uint32_t       nof_rx_antennas;
  srslte_regs_t* regs;

  // bit message 
  uint8_t data[SRSLTE_PHICH_NBITS];
  float data_rx[SRSLTE_PHICH_NBITS];

  // tx & rx objects
  srslte_modem_table_t mod;
  srslte_sequence_t    seq[SRSLTE_NOF_SF_X_FRAME];
} srslte_phich_t;

typedef struct SRSLTE_API {
  uint32_t ngroup;
  uint32_t nseq;
} srslte_phich_resource_t;

typedef struct SRSLTE_API {
  uint32_t n_prb_lowest;
  uint32_t n_dmrs;
  uint32_t I_phich;
} srslte_phich_grant_t;

 typedef struct {
    uint16_t rnti;
    bool     ack;
  } ul_sched_ack_t; */

// see lib/src/phy/enb/enb_dl.c
int enb_dl_put_phich(srslte_enb_dl_t* q,
                     srslte_phich_grant_t* grant,
                     mac_interface_phy_lte::ul_sched_ack_t * ack)
{
  srslte_phich_resource_t resource;
  bzero(&resource, sizeof(resource));

  srslte_phich_calc(&q->phich, grant, &resource);

  auto phich = enb_dl_msg_.mutable_phich();

  phich->set_rnti(ack->rnti);
  phich->set_ack(ack->ack);
  phich->set_num_prb_low(grant->n_prb_lowest);
  phich->set_num_dmrs(grant->n_dmrs);

  auto channel_message = downlink_control_message_->add_phich();

  initDownlinkChannelMessage(channel_message,
                             EMANELTE::MHAL::CHAN_PHICH,
                             EMANELTE::MHAL::MOD_BPSK,
                             ack->rnti,
                             3);  // phich is 000 for nak, 
                                  // 111 for ack. each bit is BPSK modulated 
                                  // to a symbol, and each symbol spread to 4 REs (12 REs total)

   const auto regs = q->phich.regs;

   if (SRSLTE_CP_ISEXT(regs->cell.cp)) {
     resource.ngroup /= 2;
   }

   const auto & rch = regs->phich[resource.ngroup];

   // nof_regs is 3 for phich groups (12 REs total per group).
   // l should always be 0 for Normal PHICH duration and [0,2] for Extended
   for (uint32_t i = 0; i < rch.nof_regs; i++) {
     uint32_t rb = rch.regs[i]->k0 / 12;

     channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
   }

   Info("PHICH:%s rnti 0x%hx, ack %d, n_prb_L %d, n_dmrs %d\n", 
        __func__,
        ack->rnti,
        ack->ack,
        grant->n_prb_lowest,
        grant->n_dmrs);

   return SRSLTE_SUCCESS;
}


bool enb_ul_get_signal(uint32_t tti, srslte_timestamp_t * ts)
{
  pthread_mutex_lock(&ul_mutex_);

  curr_tti_ = tti;

  EMANELTE::MHAL::ENB::set_tti(tti);

  for(auto ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end(); ++ul_msg)
    {
      ul_msg->second.SINRTester_.release();
    }

  ue_ul_msgs_.clear();

  EMANELTE::MHAL::RxMessages messages;

  struct timeval tv_sor;

  EMANELTE::MHAL::ENB::get_messages(messages, tv_sor);

  // set rx time 
  ts->full_secs = tv_sor.tv_sec;
  ts->frac_secs = tv_sor.tv_usec/1e6;

  // for each msg rx ota
  for(auto iter = messages.begin(); iter != messages.end(); ++iter)
   {
     EMANELTE::MHAL::UE_UL_Message ue_ul_msg;

     if(ue_ul_msg.ParseFromString(iter->first))
      {
        const auto & rxControl = iter->second;

        Info("RX:%s sf_time %ld:%06ld, seqnum %lu, rnti %u, tti %u, %s %s %s\n",
                __func__,
                rxControl.rxData_.sf_time_.tv_sec,
                rxControl.rxData_.sf_time_.tv_usec,
                rxControl.rxData_.rx_seqnum_,
                ue_ul_msg.transmitter().crnti(),
                ue_ul_msg.transmitter().tti(),
                ue_ul_msg.has_prach() ? "prach" : "",
                ue_ul_msg.has_pucch() ? "pucch" : "",
                ue_ul_msg.has_pusch() ? "pusch" : "");

        const uint32_t & pci = ue_ul_msg.transmitter().phy_cell_id();

        // check ul msg pci vs our pci
        if(pci != my_pci_)
         {
           Debug("RX:%s: pci 0x%x != my_pci 0x%x, ignore\n", __func__, pci, my_pci_);
         }
        else
         {
           ue_ul_msgs_.emplace_back(UE_UL_Message(ue_ul_msg, rxControl));
         }
      }
    else
      {
        Error("RX:%s ParseFromString ERROR\n", __func__);
      }
   }

  pthread_mutex_unlock(&ul_mutex_);

  return (! ue_ul_msgs_.empty());
}


int enb_ul_get_prach(uint32_t * indices, float * offsets, float * p2avg, uint32_t max_entries, uint32_t & num_entries)
{
  int result = SRSLTE_SUCCESS;

  num_entries = 0;

  pthread_mutex_lock(&ul_mutex_);

  std::set<uint32_t> unique;

  for(auto ul_msg = ue_ul_msgs_.begin(); 
       (ul_msg != ue_ul_msgs_.end()) && (num_entries < max_entries); 
         ++ul_msg)
    {
      if(ul_msg->first.has_prach())
       {
         auto & rxControl = ul_msg->second;

         if(!rxControl.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PRACH))
           {
             continue;
           }
         else
           {
             Info("PRACH:%s: fail snr\n", __func__);
           }

         const auto & prach    = ul_msg->first.prach();
         const auto & preamble = prach.preamble();

         if(unique.count(preamble.index()) == 0) // unique
           {
             unique.insert(preamble.index());

             indices[num_entries] = preamble.index();

             // timing offset estimation not currently implemented
             offsets[num_entries] = 0.0;
             p2avg[num_entries]   = 0.0;

             ++num_entries;

             Info("PRACH:%s entry[%u], accept index %d\n",
                  __func__,
                  num_entries, 
                  preamble.index());
           }
         else
          {
             Info("PRACH:%s entry[%u], ignore duplicate index %d\n",
                  __func__,
                  num_entries, 
                  preamble.index());
          }
       }
     else
       {
         Debug("PRACH:%s no preambles\n", __func__);
       }
    }

  pthread_mutex_unlock(&ul_mutex_);

  return result;
}


/*
typedef struct SRSLTE_API {
  srslte_cell_t         cell;
  cf_t*                 sf_symbols;
  srslte_chest_ul_res_t chest_res;
  srslte_ofdm_t         fft;
  srslte_chest_ul_t     chest;
  srslte_pusch_t        pusch;
  srslte_pucch_t        pucch;
} srslte_enb_ul_t;

typedef struct SRSLTE_API {
  srslte_cell_t          cell;
  srslte_modem_table_t   mod;
  srslte_uci_cqi_pucch_t cqi;
  srslte_pucch_user_t**  users;
  srslte_sequence_t      tmp_seq;
  uint16_t               ue_rnti;
  bool                   is_ue;

  uint8_t  bits_scram[SRSLTE_PUCCH_MAX_BITS];
  cf_t     d[SRSLTE_PUCCH_MAX_BITS / 2];
  uint32_t n_cs_cell[SRSLTE_NSLOTS_X_FRAME][SRSLTE_CP_NORM_NSYMB];
  uint32_t f_gh[SRSLTE_NSLOTS_X_FRAME];
  float    tmp_arg[SRSLTE_PUCCH_N_SEQ];
} srslte_pucch_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srslte_ul_sf_cfg_t;

typedef struct SRSLTE_API {
  // Input configuration for this subframe
  uint16_t rnti;

  // UCI configuration
  srslte_uci_cfg_t uci_cfg;

  // Common configuration
  uint32_t delta_pucch_shift;
  uint32_t n_rb_2;
  uint32_t N_cs;
  uint32_t N_pucch_1;
  bool     group_hopping_en; // common pusch config

  // Dedicated PUCCH configuration
  uint32_t I_sr;
  bool     sr_configured;
  uint32_t n_pucch_1[4]; // 4 n_pucch resources specified by RRC
  uint32_t n_pucch_2;
  uint32_t n_pucch_sr;
  bool     simul_cqi_ack;
  bool     tdd_ack_bundle; // if false, multiplex
  bool     sps_enabled;
  uint32_t tpc_for_pucch;

  // Release 10 CA specific
  srslte_ack_nack_feedback_mode_t ack_nack_feedback_mode;
  uint32_t                        n1_pucch_an_cs[SRSLTE_PUCCH_SIZE_AN_CS][SRSLTE_PUCCH_NOF_AN_CS];
  uint32_t                        n3_pucch_an_list[SRSLTE_PUCCH_SIZE_AN_CS];

  // Other configuration
  float threshold_format1;
  float threshold_data_valid_format1a;
  float threshold_data_valid_format2;

  // PUCCH configuration generated during a call to encode/decode
  srslte_pucch_format_t format;
  uint32_t              n_pucch;
  uint8_t               pucch2_drs_bits[SRSLTE_PUCCH2_MAX_DMRS_BITS];
} srslte_pucch_cfg_t;

typedef struct SRSLTE_API {
  uint8_t ack_value[SRSLTE_UCI_MAX_ACK_BITS];
  bool    valid;
} srslte_uci_value_ack_t;

typedef struct SRSLTE_API {
  bool     pending_tb[SRSLTE_MAX_CODEWORDS];
  uint32_t nof_acks;
  uint32_t ncce[SRSLTE_UCI_MAX_M];
  uint32_t N_bundle;
  uint32_t tdd_ack_M;
  uint32_t tdd_ack_m;
  bool     tdd_is_bundling;
  bool     has_scell_ack;
} srslte_uci_cfg_ack_t;
    
typedef struct SRSLTE_API {
  srslte_uci_cfg_ack_t ack;
  srslte_cqi_cfg_t     cqi;
  bool                 is_scheduling_request_tti;
} srslte_uci_cfg_t;

typedef struct SRSLTE_API {
  bool                   scheduling_request;
  srslte_cqi_value_t     cqi;
  srslte_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srslte_uci_value_t;
    
typedef struct SRSLTE_API {
  srslte_uci_value_t uci_data;
  float              correlation;
  bool               detected;
} srslte_pucch_res_t; */

// see lib/src/phy/enb/enb_ul.c
/* int srslte_enb_ul_get_pucch(srslte_enb_ul_t*    q,
                               srslte_ul_sf_cfg_t* ul_sf,
                               srslte_pucch_cfg_t* cfg,
                               srslte_pucch_res_t* res)
*/

int enb_ul_get_pucch(srslte_enb_ul_t*    q,
                     srslte_ul_sf_cfg_t* ul_sf,
                     srslte_pucch_cfg_t* cfg,
                     srslte_pucch_res_t* res)
{
  pthread_mutex_lock(&ul_mutex_);

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  if (!srslte_pucch_cfg_isvalid(cfg, q->cell.nof_prb)) {
    Error("PUCCH %s, Invalid PUCCH configuration\n", __func__);
    return -1;
  }

  // see lib/src/phy/enb/enb_ul.c get_pucch()
  // and lib/src/phy/ue/test/pucch_resource_test.c
  // this is needed to set cfg->format
  srslte_uci_value_t uci_value;
  ZERO_OBJECT(uci_value);

  srslte_ue_ul_pucch_resource_selection(&q->cell, cfg, &cfg->uci_cfg, &uci_value);

  const auto rnti = cfg->rnti;

  res->correlation = 1.0;
  res->detected    = false;

  // for each uplink message
  for(auto ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end() && !res->detected; ++ul_msg)
   {
     if(ul_msg->first.has_pucch())
      {
        const auto & pucch_message = ul_msg->first.pucch();

        // for each grant
        for(int n = 0; n < pucch_message.grant_size(); ++n)
         {
           const auto & grant_message = pucch_message.grant(n);

           Info("PUCCH:%s sr %d, acks %d, ul_rnti 0x%hx vs rnti 0x%hx, %d of %d grants\n", 
                __func__, 
                cfg->uci_cfg.is_scheduling_request_tti,
                srslte_uci_cfg_total_ack(&cfg->uci_cfg),
                grant_message.rnti(), rnti, n+1, pucch_message.grant_size());

           std::string format;

           if(grant_message.rnti() == rnti)
            {
              auto & rxControl = ul_msg->second;

              const auto sinrResult = rxControl.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PUCCH, rnti);

              if(sinrResult.bPassed_)
                {
                  const auto & uci_message = grant_message.uci();

                  memcpy(&res->uci_data, uci_message.data(), uci_message.length());

                  res->detected = true;

                  q->chest_res.snr_db             = sinrResult.sinr_dB_;
                  q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                  // from lib/src/phy/phch/pucch.c srslte_pucch_decode()
                  switch (cfg->format) {
                   case SRSLTE_PUCCH_FORMAT_1A:
                   case SRSLTE_PUCCH_FORMAT_1B:
                     res->uci_data.ack.valid = true;
                     format = "1A/1B";
                   break;

                   case SRSLTE_PUCCH_FORMAT_2:
                   case SRSLTE_PUCCH_FORMAT_2A:
                   case SRSLTE_PUCCH_FORMAT_2B:
                     res->uci_data.ack.valid    = true;
                     res->uci_data.cqi.data_crc = true;
                     format = "2";
                   break;

                   case SRSLTE_PUCCH_FORMAT_1:
                   case SRSLTE_PUCCH_FORMAT_3:
                     format = "1/3";
                   break;

                   default:
                     format = "default";
                  }

#ifdef DEBUG_HEX
                  InfoHex(uci_message.data(), uci_message.length(),
                          "PUCCH:%s found pucch format %s, rnti %hx, corr %f\n",
                          __func__, format.c_str(), rnti, res->correlation);
#endif

                  // pass
                  ENBSTATS::getPUCCH(rnti, true);
                }
              else
                {
                  q->chest_res.snr_db             = sinrResult.sinr_dB_;
                  q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                  // PUCCH failed snr, ignore
                  ENBSTATS::getPUCCH(rnti, false);
                }

               // done with this rnti
               break;
             }
          }
       }
    }

  if(res->detected == false)
   {
     Info("PUCCH:%s tti %u, did NOT find rnti 0x%hx, in %zu uplink messages\n", 
           __func__, curr_tti_, rnti, ue_ul_msgs_.size());
   }
  else
   {
     Info("PUCCH:%s tti %u, DID find rnti 0x%hx, in %zu uplink messages\n", 
           __func__, curr_tti_, rnti, ue_ul_msgs_.size());
   }

  pthread_mutex_unlock(&ul_mutex_);

  return SRSLTE_SUCCESS;
}

/*

typedef struct SRSLTE_API {
  srslte_cell_t         cell;
  cf_t*                 sf_symbols;
  srslte_chest_ul_res_t chest_res;
  srslte_ofdm_t         fft;
  srslte_chest_ul_t     chest;
  srslte_pusch_t        pusch;
  srslte_pucch_t        pucch;
} srslte_enb_ul_t;

typedef struct SRSLTE_API {
  cf_t*    ce;
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbm;
  float    snr;
  float    snr_db;
  float    cfo;
} srslte_chest_ul_res_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srslte_ul_sf_cfg_t

typedef struct SRSLTE_API {
  uint16_t                rnti;
  srslte_uci_cfg_t        uci_cfg;
  srslte_uci_offset_cfg_t uci_offset;
  srslte_pusch_grant_t    grant;
  uint32_t                max_nof_iterations;
  uint32_t                last_O_cqi;
  uint32_t                K_segm;
  uint32_t                current_tx_nb;
  bool                    csi_enable;
  bool                    enable_64qam;
  union {
    srslte_softbuffer_tx_t* tx;
    srslte_softbuffer_rx_t* rx;
  } softbuffers;
  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pusch_cfg_t;

typedef struct SRSLTE_API {
  bool                   scheduling_request;
  srslte_cqi_value_t     cqi;
  srslte_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srslte_uci_value_t;
 
typedef struct SRSLTE_API {
  uint8_t*           data;
  srslte_uci_value_t uci;
  bool               crc;
  float              avg_iterations_block;
} srslte_pusch_res_t;
*/

int enb_ul_get_pusch(srslte_enb_ul_t*    q,
                     srslte_ul_sf_cfg_t* ul_sf,
                     srslte_pusch_cfg_t* cfg,
                     srslte_pusch_res_t* res,
                     uint16_t rnti)
{
  int result = SRSLTE_SUCCESS;

  pthread_mutex_lock(&ul_mutex_);

  res->crc            = false;
  res->uci.ack.valid  = false;

  // for each uplink message
  for(auto ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end() && !res->crc; ++ul_msg)
   {
     if(ul_msg->first.has_pusch())
      {
        const auto & pusch_message = ul_msg->first.pusch();

        // for each grant
        for(int n = 0; n < pusch_message.grant_size(); ++n)
         {
           const auto & grant_message = pusch_message.grant(n);

           Info("PUSCH:%s check ul_rnti 0x%hx vs rnti 0x%hx, %d of %d grants\n",
                   __func__, grant_message.rnti(), rnti, n+1, pusch_message.grant_size());

           if(grant_message.rnti() == rnti)
            {
              auto & rxControl = ul_msg->second;

              const auto sinrResult = rxControl.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PUSCH, rnti);

              if(sinrResult.bPassed_)
                {
                  const auto & ul_grant_message = grant_message.ul_grant();
                  const auto & uci_message      = grant_message.uci();
                  const auto & payload          = grant_message.payload();

                  // srslte_pusch_grant_t  
                  memcpy(&cfg->grant, ul_grant_message.data(), ul_grant_message.length());

                  // srslte_uci_value_t
                  memcpy(&res->uci, uci_message.data(), uci_message.length());

                  // payload
                  memcpy(res->data, payload.data(), payload.length());

                  // see lib/src/phy/phch/pusch.c srslte_pusch_decode()
                  res->avg_iterations_block = 1;
                  res->crc                  = true;
                  res->uci.ack.valid        = true;

                  q->chest_res.snr_db             = sinrResult.sinr_dB_;
                  q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;
 
#ifdef DEBUG_HEX
                  InfoHex(payload.data(), payload.length(),
                          "PUSCH:%s rnti %hx, snr_db %f\n",
                          __func__, rnti, q->chest_res.snr_db);
#endif

                  // pass
                  ENBSTATS::getPUSCH(rnti, true);
                }
              else
                {
                  q->chest_res.snr_db             = sinrResult.sinr_dB_;
                  q->chest_res.noise_estimate_dbm = sinrResult.noiseFloor_dBm_;

                  // PUSCH failed snr, ignore
                  ENBSTATS::getPUSCH(rnti, false);
                }

              // done with this rnti
              break;
            }
         }
      }
   }

  if(res->crc == false)
   {
     Info("PUSCH:%s tti %u, did NOT find rnti 0x%hx, in %zu uplink messages\n", 
           __func__, curr_tti_, rnti, ue_ul_msgs_.size());
   }
  else
   {
     Info("PUSCH:%s tti %u, DID find rnti 0x%hx, in %zu uplink messages\n", 
           __func__, curr_tti_, rnti, ue_ul_msgs_.size());
   }

  pthread_mutex_unlock(&ul_mutex_);

  return result;
}


} // end namespace phy_adapter
} // end namespace srsenb

#endif
