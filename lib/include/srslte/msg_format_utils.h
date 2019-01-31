/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
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


#ifndef EMU_SRSLTE_MSG_FORMAT_UTILS_H
#define EMU_SRSLTE_MSG_FORMAT_UTILS_H

#include <stdio.h>

#include "srslte/common/pdu.h"
#include "srslte/phy/phch/dci.h"
#include "srslte/interfaces/enb_interfaces.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/phy/enb/enb_dl.h"
#include "srslte/phy/enb/enb_ul.h"
#include "srslte/phy/phch/pusch.h"
#include "srslte/phy/phch/pucch.h"
#include "srslte/phy/phch/prach.h"
#include "srslte/phy/phch/sch.h"
#include "srslte/phy/sync/sync.h"
#include "srslte/phy/ue/ue_mib.h" 
#include "srslte/phy/ue/ue_mib.h"
#include "srslte/phy/ue/ue_dl.h"

#include <string>
#include <sstream>
#include <iomanip>

#define YES_NO(x)          ((x) ? "Yes" : "No")

namespace {

  std::string bits_to_bytes_hex_string(const uint8_t *b, int num_bits)
   {
     const int num_bytes = num_bits / 8;
     const int xtra_bits = num_bits % 8;

     char buf[1024] = {0};

     int len = snprintf(buf, sizeof(buf), "[");

     // handle as 8 byte blocks
     for(int i = 0; i < num_bytes; ++i)
       {
         // next byte
         const int n = i * 8;

         uint8_t val = (b[n+0] * 128) +
                       (b[n+1] *  64) +
                       (b[n+2] *  32) +
                       (b[n+3] *  16) +
                       (b[n+4] *   8) +
                       (b[n+5] *   4) +
                       (b[n+6] *   2) +
                       (b[n+7] *   1);

          len += snprintf(buf + len, sizeof(buf) - len, "%02hhx", val);

          if(i < (num_bytes - 1))
           {
             len += snprintf(buf + len, sizeof(buf) - len, " ");
           }
        }

      // any xtra bits
      if(xtra_bits > 0)
        {
          uint8_t val = 0, mult = 128;

          // start at next byte
          const int n = num_bytes * 8;

          for(int i = 0; i < xtra_bits; ++i, mult /= 2)
           {
             val += (b[n+i] * mult);
           }

          len += snprintf(buf + len, sizeof(buf) - len, " %02hhx", val);
        }

     len += snprintf(buf + len, sizeof(buf) - len, "]");

     return buf;
   }


  std::string byte_hex_string(const char * prefix, const uint8_t *hex, size_t size)
   {
    std::stringstream ss;
    size_t c = 0;

    ss << std::hex << std::setfill('0');

    while(hex && (c < size)) {
      ss << prefix << std::setw(4) << static_cast<unsigned>(c) << ": ";
      size_t tmp = (size-c < 16) ? size-c : 16;

      for(size_t i=0; i<tmp; ++i) {
        ss << std::setw(2) << static_cast<unsigned>(hex[c++]) << " ";
      }
      ss << "\n";
    }
    return ss.str();
  }

  std::string enumRangeError(const char * msg, int id)
    {
      char buf[256] = {0};

      snprintf(buf, sizeof(buf), "%s %d", msg, id);
     
      return buf;
    }
}


/* see lib/include/srslte/phy/common/phy_common.h
typedef enum SRSLTE_API {
  SRSLTE_MOD_BPSK = 0, 
  SRSLTE_MOD_QPSK, 
  SRSLTE_MOD_16QAM, 
  SRSLTE_MOD_64QAM,
  SRSLTE_MOD_LAST
} srslte_mod_t; */
inline std::string mod_t_to_name(srslte_mod_t arg)
 {
   switch(arg)
     {
       case SRSLTE_MOD_BPSK: 
        return "BPSK";
       case SRSLTE_MOD_QPSK:
        return "QPSK";
       case SRSLTE_MOD_16QAM:
        return "16QAM";
       case SRSLTE_MOD_64QAM:
        return "64QAM";
       default:
        return enumRangeError("InvalidModType", arg).c_str();
    }
}


/* see lib/include/srslte/phy/phch/dci.h */
inline std::string dci_format_t_to_name(srslte_dci_format_t arg)
 {
   switch(arg)
     {
       case SRSLTE_DCI_FORMAT0:
        return "Format0";
       case SRSLTE_DCI_FORMAT1:
        return "Format1";
       case SRSLTE_DCI_FORMAT1A:
        return "Format1A";
       case SRSLTE_DCI_FORMAT1C:
        return "Format1C";
       case SRSLTE_DCI_FORMAT1B:
        return "Format1B";
       case SRSLTE_DCI_FORMAT1D:
        return "Format1D";
       case SRSLTE_DCI_FORMAT2:
        return "Format2";
       case SRSLTE_DCI_FORMAT2A:
        return "Format2A";
       case SRSLTE_DCI_FORMAT2B:
        return "Format2B";
       default:
        return enumRangeError("InvalidDCIType", arg).c_str();
    }
}

/* see lib/include/srslte/phy/phch/pucch.h */
inline std::string pucch_format_t_to_name(srslte_pucch_format_t arg)
 {
   switch(arg)
     {
       case SRSLTE_PUCCH_FORMAT_1:
        return "Format1";
       case SRSLTE_PUCCH_FORMAT_1A:
        return "Format1A";
       case SRSLTE_PUCCH_FORMAT_1B:
        return "Format1B";
       case SRSLTE_PUCCH_FORMAT_2:
        return "Format2";
       case SRSLTE_PUCCH_FORMAT_2A:
        return "Format2A";
       case SRSLTE_PUCCH_FORMAT_2B:
        return "Format2B";
       case SRSLTE_PUCCH_FORMAT_ERROR:
        return "Error";
       default:
        return enumRangeError("InvalidPUCCHType", arg).c_str();
    }
}

 
/* see lib/include/srslte/phy/phch/ra.h */
inline std::string ra_type_t_to_name(srslte_ra_type_t arg)
{
  switch(arg)
   {
     case SRSLTE_RA_ALLOC_TYPE0:
      return  "RA_Type0";
     case   SRSLTE_RA_ALLOC_TYPE1:
      return  "RA_Type1";
     case   SRSLTE_RA_ALLOC_TYPE2:
      return  "RA_Type2";
     default:
      return enumRangeError("InvalidRAType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h */
inline std::string cp_t_to_name(srslte_cp_t arg)
{
  switch(arg)
   {
     case SRSLTE_CP_NORM:
      return  "NORM";
     case SRSLTE_CP_EXT:
      return  "EXT";
     default:
      return enumRangeError("InvalidCPType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h */
inline std::string sf_t_to_name(srslte_sf_t arg)
{
  switch(arg)
   {
     case SRSLTE_SF_NORM:
      return  "NORM";
     case SRSLTE_SF_MBSFN:
      return  "MBSFN";
     default:
      return enumRangeError("InvalidSFType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h */
inline std::string phich_length_t_to_name(srslte_phich_length_t arg)
{
  switch(arg)
   {
     case SRSLTE_PHICH_NORM:
      return  "NORM";
     case SRSLTE_PHICH_EXT:
      return  "EXT";
     default:
      return enumRangeError("InvalidLENType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h */
inline std::string phich_resources_t_to_name(srslte_phich_resources_t arg)
{
  switch(arg)
   {
     case SRSLTE_PHICH_R_1_6:
      return  "R_1_6";
     case SRSLTE_PHICH_R_1_2:
      return  "R_1_2";
     case SRSLTE_PHICH_R_1:
      return  "R_1";
     case SRSLTE_PHICH_R_2:
      return  "R_2";
     default:
      return enumRangeError("InvalidRESType", arg).c_str();
   }
}


/* lib/include/srslte/phy/common/phy_common.h */
inline std::string mimo_type_t_to_name(srslte_mimo_type_t arg)
{
  switch(arg)
   {
     case SRSLTE_MIMO_TYPE_SINGLE_ANTENNA:
      return  "Single";
     case SRSLTE_MIMO_TYPE_TX_DIVERSITY:
      return  "TxDiversity";
     case SRSLTE_MIMO_TYPE_SPATIAL_MULTIPLEX:
      return  "SpatialMultiplex";
     case SRSLTE_MIMO_TYPE_CDD:
      return  "CDD";
     default:
      return enumRangeError("InvalidMIMOType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h */
inline std::string rnti_type_t_to_name(srslte_rnti_type_t arg)
{
  switch(arg)
   {
     case SRSLTE_RNTI_USER:
      return  "Cell";
     case SRSLTE_RNTI_SI:
      return  "SysInfo";
     case SRSLTE_RNTI_RAR:
      return  "RAR";
     case SRSLTE_RNTI_TEMP:
      return  "Temp";
     case SRSLTE_RNTI_SPS:
      return  "SPS";
     case SRSLTE_RNTI_PCH:
      return  "Paging";
     case SRSLTE_RNTI_MBSFN:
      return  "MBSFN";
     default:
      return enumRangeError("InvalidRNTIType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/common/phy_common.h 
#define SRSLTE_CRNTI_START  0x000B
#define SRSLTE_CRNTI_END    0xFFF3
#define SRSLTE_RARNTI_START 0x0001
#define SRSLTE_RARNTI_END   0x000A
#define SRSLTE_SIRNTI       0xFFFF
#define SRSLTE_PRNTI        0xFFFE
#define SRSLTE_MRNTI        0xFFFD */
inline std::string rnti_to_rnti_type(uint32_t rnti)
{
  srslte_rnti_type_t type;

  if(rnti == SRSLTE_SIRNTI) {
      type = SRSLTE_RNTI_SI;
    } 
  else if(rnti == SRSLTE_PRNTI) {
      type = SRSLTE_RNTI_PCH;    
    } 
  else if(rnti == SRSLTE_MRNTI) {
      type = SRSLTE_RNTI_MBSFN;    
    } 
  else if(rnti >= SRSLTE_RARNTI_START && rnti <= SRSLTE_RARNTI_END) {
      type = SRSLTE_RNTI_RAR;    
    } 
  else if(rnti >= SRSLTE_CRNTI_START && rnti <= SRSLTE_CRNTI_END) {
      type = SRSLTE_RNTI_USER;
    }
  else {
      type = SRSLTE_RNTI_NOF_TYPES;
    }
  
  return rnti_type_t_to_name(type);
}

/* see lib/include/srslte/phy/ue/ue_sync.h
typedef enum SRSLTE_API { SF_FIND, SF_TRACK} srslte_ue_sync_state_t; */
inline std::string ue_sync_state_t_to_name(srslte_ue_sync_state_t arg)
{
  switch(arg)
   {
     case SF_FIND:
      return  "FIND";
     case SF_TRACK:
      return  "TRACK";
     default:
      return enumRangeError("InvalidSYNCSTATEType", arg).c_str();
   }
}


/* see lib/include/srslte/phy/phch/dci.h 
typedef struct SRSLTE_API {
  uint8_t data[SRSLTE_DCI_MAX_BITS];
  uint32_t nof_bits;
  srslte_dci_format_t format; 
} srslte_dci_msg_t; */
inline std::string dci_msg_t_to_string(const srslte_dci_msg_t * arg)
{
  static char buf[1024] = {0};
  
  snprintf(buf, sizeof(buf), "nof_bits %u, dci_format:%s" 
                             "\n\t\t\t data %s\n%s",
           arg->nof_bits,
           dci_format_t_to_name(arg->format).c_str(),
           bits_to_bytes_hex_string(arg->data, arg->nof_bits).c_str(),
           byte_hex_string("\t\t\t\t", arg->data, arg->nof_bits).c_str());

  return buf;
}


/* see lib/include/srslte/phy/common/sequence.h
typedef struct SRSLTE_API {
  uint8_t *c;
  uint8_t *c_bytes;
  float *c_float;
  short *c_short;
  uint32_t cur_len;
  uint32_t max_len;
} srslte_sequence_t; */
inline std::string sequence_t_to_string(const srslte_sequence_t * arg)
{
  char buf[32] = {0};
  
  int len = snprintf(buf, sizeof(buf), "cur/max len %u,%u", 
                     arg->cur_len,
                     arg->max_len);

  return buf;
}


/* see lib/include/srslte/phy/fec/cbsegm.h
 typedef struct SRSLTE_API {
  uint32_t F;
  uint32_t C;
  uint32_t K1;
  uint32_t K2;
  uint32_t K1_idx;
  uint32_t K2_idx;
  uint32_t C1;
  uint32_t C2;
  uint32_t tbs; 
} srslte_cbsegm_t; */
inline std::string cbsegm_t_to_string(const srslte_cbsegm_t * arg)
{
  char buf[256] = {0};

  if(0)  
   {
    snprintf(buf, sizeof(buf), "F %2u, C %2u, K1 %2u, K2 %2u, K1_idx %2u, K2_idx %2u, C1 %2u, C2 %2u, tbs %2u", 
             arg->F,
             arg->C,
             arg->K1,
             arg->K2,
             arg->K1_idx,
             arg->K2_idx,
             arg->C1,
             arg->C2,
             arg->tbs);
   }
  else
   {
     snprintf(buf, sizeof(buf), "tbs %2u", arg->tbs);
   }

  return buf;
 }


/* see lib/include/srslte/phy/fec/crc.h
typedef struct SRSLTE_API {
  uint64_t table[256];
  int polynom;
  int order;
  uint64_t crcinit; 
  uint64_t crcmask;
  uint64_t crchighbit;
  uint32_t srslte_crc_out;
} srslte_crc_t; */
inline std::string crc_t_to_string(const srslte_crc_t * arg)
{
  char buf[64] = {0};
  
  int len = snprintf(buf, sizeof(buf), "poly %d, order %d, crc %u", 
                     arg->polynom,
                     arg->order,
                     arg->srslte_crc_out);

  return buf;
 }


/* see lib/include/srslte/phy/phch/ra.h
 typedef struct SRSLTE_API {
  uint32_t rbg_bitmask;
} srslte_ra_type0_t; */
inline std::string ra_type0_t_to_string(const srslte_ra_type0_t * arg)
{
  char buf[256] = {0};

  snprintf(buf, sizeof(buf), "rbg_bitmask 0x%x", arg->rbg_bitmask);

  return buf; 
}

/* see lib/include/srslte/phy/phch/ra.h
typedef struct SRSLTE_API {
  uint32_t vrb_bitmask;
  uint32_t rbg_subset;
  bool shift;
} srslte_ra_type1_t; */
inline std::string ra_type1_t_to_string(const srslte_ra_type1_t * arg)
{
  char buf[256] = {0};

  snprintf(buf, sizeof(buf), "vrb_bitmask 0x%x, rbg_subset 0x%x", 
           arg->vrb_bitmask,
           arg->rbg_subset);

  return buf; 
}


/* see lib/include/srslte/phy/phch/ra.h 
typedef struct SRSLTE_API {
  uint32_t riv; // if L_crb==0, DCI message packer will take this value directly
  uint32_t L_crb;
  uint32_t RB_start;
  enum {
    SRSLTE_RA_TYPE2_NPRB1A_2 = 0, SRSLTE_RA_TYPE2_NPRB1A_3 = 1
  } n_prb1a;
  enum {
    SRSLTE_RA_TYPE2_NG1 = 0, SRSLTE_RA_TYPE2_NG2 = 1
  } n_gap;
  enum {
    SRSLTE_RA_TYPE2_LOC = 0, SRSLTE_RA_TYPE2_DIST = 1
  } mode;
} srslte_ra_type2_t; */
inline std::string ra_type2_t_to_string(const srslte_ra_type2_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "riv %hu, L_crb %u, RB_start %u, n_prb1a_%s, n_gap_%s, mode %s",
                     arg->riv,
                     arg->L_crb,
                     arg->RB_start,
                     arg->n_prb1a ? "3" : "2",
                     arg->n_gap   ? "1" : "2",
                     arg->mode    ? "DIST" : "LOC");

  return buf;
}



/* see lib/include/srslte/phy/phch/ra.h
typedef struct SRSLTE_API {
  srslte_ra_type_t alloc_type;
  union {
    srslte_ra_type0_t type0_alloc;
    srslte_ra_type1_t type1_alloc;
    srslte_ra_type2_t type2_alloc;
  };
  uint32_t harq_process;
  uint32_t mcs_idx;
  int      rv_idx;
  bool     ndi;
  uint32_t mcs_idx_1;
  int      rv_idx_1;
  bool     ndi_1;
  bool     tb_cw_swap; 
  bool     sram_id; 
  uint8_t  pinfo; 
  bool     pconf;
  bool     power_offset; 
  uint8_t tpc_pucch;
  bool     tb_en[2]; 
  bool     is_ra_order;
  uint32_t ra_preamble;
  uint32_t ra_mask_idx;
  bool     dci_is_1a;
  bool     dci_is_1c; 
} srslte_ra_dl_dci_t; */
inline std::string ra_dl_dci_t_to_string(const srslte_ra_dl_dci_t * arg)
{
  char buf[256] = {0};
  
  int len = snprintf(buf, sizeof(buf), "%s, harqp %u, mcs_idx[%u,%u] rv_idx[%d,%d] ndi[%d,%d], tb_swap %s, tpc_pucch %hhu, nof_tb %d, ra_pre %u, ra_m_idx %u, 1a:%s, 1c:%s", 
                     ra_type_t_to_name(arg->alloc_type).c_str(),
                     arg->harq_process,
                     arg->mcs_idx,
                     arg->mcs_idx_1,
                     arg->rv_idx,
                     arg->rv_idx_1,
                     arg->ndi,
                     arg->ndi_1,
                     YES_NO(arg->tb_cw_swap),
                     arg->tpc_pucch,
                     SRSLTE_RA_DL_GRANT_NOF_TB(arg),
                     arg->ra_preamble,
                     arg->ra_mask_idx,
                     YES_NO(arg->dci_is_1a),
                     YES_NO(arg->dci_is_1c));

  switch(arg->alloc_type)
   {
     case SRSLTE_RA_ALLOC_TYPE0:
       len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t   RA_TYPE0 %s", 
              ra_type0_t_to_string(&arg->type0_alloc).c_str());
     break;
 
     case SRSLTE_RA_ALLOC_TYPE1:
       len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t   RA_TYPE1 %s", 
              ra_type1_t_to_string(&arg->type1_alloc).c_str());
     break;
 
     case SRSLTE_RA_ALLOC_TYPE2:
       len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t   RA_TYPE2 %s", 
              ra_type2_t_to_string(&arg->type2_alloc).c_str());
     break;
 
     default:
      return enumRangeError("InvalidRAType", arg->alloc_type).c_str();
   }

   return buf;
 }


/* see lib/include/srslte/phy/phch/dci.h
typedef struct SRSLTE_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srslte_dci_location_t;  */
inline std::string dci_location_t_to_string(const srslte_dci_location_t * arg)
{
  char buf[64] = {0};
  
  snprintf(buf, sizeof(buf), "Level %u, nCCE %u", arg->L, arg->ncce);

  return buf;
}


/* see lib/include/srslte/phy/phch/dci.h
typedef struct SRSLTE_API {
  uint32_t rba;
  uint32_t trunc_mcs;
  uint32_t tpc_pusch;
  bool ul_delay;
  bool cqi_request; 
  bool hopping_flag; 
} srslte_dci_rar_grant_t; */
inline std::string dci_rar_grant_t_to_string(const srslte_dci_rar_grant_t * arg)
{
  char buf[256] = {0};
  
  int len = snprintf(buf, sizeof(buf), "rba %u, trunc_mcs %u, tpc_pusch %u, ul_delay %d, cqi_req %d, hop_flag %d",
                     arg->rba,
                     arg->trunc_mcs,
                     arg->tpc_pusch,
                     arg->ul_delay,
                     arg->cqi_request,
                     arg->hopping_flag);

  return buf;
}


/* see lib/include/srslte/phy/enb/enb_dl.h
typedef struct {
  uint16_t                rnti; 
  srslte_dci_format_t     dci_format;
  srslte_ra_dl_dci_t      grant;
  srslte_dci_location_t   location; 
  srslte_softbuffer_tx_t *softbuffers[SRSLTE_MAX_TB];
  uint8_t                *data[SRSLTE_MAX_TB];
} srslte_enb_dl_pdsch_t; */
inline std::string enb_dl_pdsch_t_to_string(const srslte_enb_dl_pdsch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, dp[0] %p, dp[1] %p, dci_format:%s, dci_location:%s\n\t\t\t grant:%s",
                     arg->rnti,
                     arg->data[0], arg->data[1],
                     dci_format_t_to_name(arg->dci_format).c_str(),
                     dci_location_t_to_string(&arg->location).c_str(),
                     ra_dl_dci_t_to_string(&arg->grant).c_str());

  for(int tb = 0; tb < SRSLTE_RA_DL_GRANT_NOF_TB(&arg->grant); ++tb)
   { 
     snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t    tb[%d]: has-data %s",
              tb,
              YES_NO(arg->data[tb]));
   }
 
  return buf;
}


/* see lib/include/srslte/interfaces/sched_interface.h
 typedef struct {
    uint32_t              rnti;
    srslte_dci_format_t   dci_format;
    srslte_ra_dl_dci_t    dci;
    srslte_dci_location_t dci_location;
    uint32_t              tbs[SRSLTE_MAX_TB];
    bool mac_ce_ta;
    bool mac_ce_rnti;
    uint32_t nof_pdu_elems[SRSLTE_MAX_TB];
    dl_sched_pdu_t pdu[SRSLTE_MAX_TB][MAX_RLC_PDU_LIST];
  } dl_sched_data_t; 

 typedef struct {    
    uint32_t lcid;    
    uint32_t nbytes;
  } dl_sched_pdu_t; */
inline std::string dl_sched_data_t_to_string(const srsenb::sched_interface::dl_sched_data_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, dci_fmat:%s\n\t\t\t  dci:%s\n\t\t\t  dci_loc:%s",
                      arg->rnti,
                      dci_format_t_to_name(arg->dci_format).c_str(),
                      ra_dl_dci_t_to_string(&arg->dci).c_str(),
                      dci_location_t_to_string(&arg->dci_location).c_str());

  for(uint32_t tb = 0; tb < SRSLTE_MAX_TB; ++tb)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t    tb[%d], tbs %2u, nof_pdu %2u",
                     tb,
                     arg->tbs[tb],
                     arg->nof_pdu_elems[tb]);
   
     for(uint32_t pdu = 0; pdu < arg->nof_pdu_elems[tb]; ++pdu)
       {
         len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t\tpdu[%d], lcid %2u, nbytes %2u %s",
                         pdu,
                         arg->pdu[tb][pdu].lcid, 
                         arg->pdu[tb][pdu].nbytes,
                         arg->pdu[tb][pdu].lcid == srslte::sch_subh::CON_RES_ID ? "ConRes" : "");
       }
   }

  return buf;
} 


/* see lib/include/srslte/interfaces/sched_interface.h
 typedef struct {
    uint32_t               rarnti; 
    uint32_t               tbs;
    srslte_ra_dl_dci_t     dci;  
    srslte_dci_location_t  dci_location;
    uint32_t               nof_grants;
    dl_sched_rar_grant_t   grants[MAX_RAR_LIST];    
  } dl_sched_rar_t; */
inline std::string dl_sched_rar_t_to_string(const srsenb::sched_interface::dl_sched_rar_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "rarnti 0x%hx, tbs %u, n_dlgrants %u\n\t\t\t  dci:%s\n\t\t\t  dci_loc:%s",
                      arg->rarnti,
                      arg->tbs,
                      arg->nof_grants,
                      ra_dl_dci_t_to_string(&arg->dci).c_str(),
                      dci_location_t_to_string(&arg->dci_location).c_str());

   for(uint32_t gr = 0; gr < arg->nof_grants; ++gr)
    {
      const srsenb::sched_interface::dl_sched_rar_grant_t & grant = arg->grants[gr];
 
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t gr[%d] ra_id %2u, grant:%s",
                      gr,
                      grant.ra_id,
                      dci_rar_grant_t_to_string(&grant.grant).c_str());
    }

  return buf;
} 


/* see lib/include/srslte/interfaces/sched_interface.h
  typedef struct {
    srslte_ra_dl_dci_t dci; 
    srslte_dci_location_t  dci_location;
    enum bc_type {
      BCCH, PCCH
    } type;
    uint32_t index;
    uint32_t tbs;
  } dl_sched_bc_t; */
inline std::string dl_sched_bc_t_to_string(const srsenb::sched_interface::dl_sched_bc_t * arg)
{
  char buf[256] = {0};

  snprintf(buf, sizeof(buf), "type %s, index %u, tbs %u, dci:%s\n\t\t\t  dci_loc:%s",
                      arg->type == srsenb::sched_interface::dl_sched_bc_t::BCCH ? "BCCH" : "PCCH",
                      arg->index,
                      arg->tbs,
                      ra_dl_dci_t_to_string(&arg->dci).c_str(),
                      dci_location_t_to_string(&arg->dci_location).c_str());

  return buf;
} 


/* see lib/include/srslte/phy/enb/enb_dl.h
  typedef struct {
    srslte_enb_dl_pdsch_t sched_grants[MAX_GRANTS];
    uint32_t nof_grants; 
    uint32_t cfi; 
  } dl_sched_t; */

/* see lib/include/srslte/interfaces/sched_interface.h 
  typedef struct {
    uint32_t cfi; 
    uint32_t nof_data_elems;
    uint32_t nof_rar_elems;
    uint32_t nof_bc_elems; 
    dl_sched_data_t data[MAX_DATA_LIST];
    dl_sched_rar_t  rar[MAX_RAR_LIST];
    dl_sched_bc_t   bc[MAX_BC_LIST];
  } dl_sched_res_t; */
inline std::string dl_sched_t_to_string(const srsenb::mac_interface_phy::dl_sched_t   * dl_sched, 
                                        const srsenb::sched_interface::dl_sched_res_t * dl_sched_res)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "cfi %u, n_dlgrants %u, [data %u, rar %u, bc %u]",
                     dl_sched->cfi,
                     dl_sched->nof_grants,
                     dl_sched_res->nof_data_elems, 
                     dl_sched_res->nof_rar_elems,
                     dl_sched_res->nof_bc_elems);

#if 1
  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\tdl_sched:");
 
  for(uint32_t n = 0; n < dl_sched->nof_grants; ++n)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t grant[%u]:\n\t\t\t  %s",
                     n, 
                     enb_dl_pdsch_t_to_string(&dl_sched->sched_grants[n]).c_str());
   }
#endif

  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\tdl_sched_res:");

  int n = 0;

  for(uint32_t n = 0; n < dl_sched_res->nof_data_elems; ++n, ++n)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  data:%s",
                     dl_sched_data_t_to_string(&dl_sched_res->data[n]).c_str());
   }

  for(uint32_t n = 0; n < dl_sched_res->nof_rar_elems; ++n, ++n)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  rar:%s",
                     dl_sched_rar_t_to_string(&dl_sched_res->rar[n]).c_str());
   }

  for(uint32_t n = 0; n < dl_sched_res->nof_bc_elems; ++n, ++n)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  bc:%s",
                     dl_sched_bc_t_to_string(&dl_sched_res->bc[n]).c_str());
   }

  return buf;
}


/* see lib/include/srslte/phy/common/phy_common.h
 typedef struct SRSLTE_API {
  uint32_t nof_prb;
  uint32_t nof_ports; 
  uint32_t id;
  srslte_cp_t cp;
  srslte_phich_length_t phich_length;
  srslte_phich_resources_t phich_resources;
}srslte_cell_t; */
inline std::string cell_t_to_string(const srslte_cell_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "cellid %u, n_prb %u, n_ports %u, cp_type %s, phich_len %s, phich_res %s",
                     arg->id,
                     arg->nof_prb,
                     arg->nof_ports,
                     cp_t_to_name(arg->cp).c_str(),
                     phich_length_t_to_name(arg->phich_length).c_str(),
                     phich_resources_t_to_name(arg->phich_resources).c_str());

  return buf;
}


/* see lib/include/srslte/phy/phch/regs.h
  typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t max_ctrl_symbols;
  uint32_t ngroups_phich;
  srslte_phich_resources_t phich_res;
  srslte_phich_length_t phich_len;
  srslte_regs_ch_t pcfich;
  srslte_regs_ch_t *phich; // there are several phich
  srslte_regs_ch_t pdcch[3];
  uint32_t nof_regs;
  srslte_regs_reg_t *regs;
}srslte_regs_t; */
inline std::string regs_t_to_string(const srslte_regs_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "max_ctrl_symb %u, nof_regs %u, ngroups %u, phich_len %s, phich_res %s",
                     arg->max_ctrl_symbols,
                     arg->nof_regs,
                     arg->ngroups_phich,
                     phich_length_t_to_name(arg->phich_len).c_str(),
                     phich_resources_t_to_name(arg->phich_res).c_str());

  return buf;
}
 

/* see lib/include/srslte/phy/phch/pbch.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t nof_symbols;
  cf_t *ce[SRSLTE_MAX_PORTS];
  cf_t *symbols[SRSLTE_MAX_PORTS];
  cf_t *x[SRSLTE_MAX_PORTS];
  cf_t *d;
  float *llr;
  float *temp;
  float rm_f[SRSLTE_BCH_ENCODED_LEN];
  uint8_t *rm_b;
  uint8_t data[SRSLTE_BCH_PAYLOADCRC_LEN];
  uint8_t data_enc[SRSLTE_BCH_ENCODED_LEN];
  uint32_t frame_idx;
  srslte_modem_table_t mod;
  srslte_sequence_t seq;
  srslte_viterbi_t decoder;
  srslte_crc_t crc;
  srslte_convcoder_t encoder;
  bool search_all_ports;
 } srslte_pbch_t; */
inline std::string pbch_t_to_string(const srslte_pbch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "pbch_t:n_symbols %u, frame_idx %u, seq: %s, crc: %s, sch_all %s"
                                       "\n\t\t\t data %s\n%s",
                     arg->nof_symbols,
                     arg->frame_idx,
                     sequence_t_to_string(&arg->seq).c_str(),
                     crc_t_to_string(&arg->crc).c_str(),
                     YES_NO(arg->search_all_ports),
                     bits_to_bytes_hex_string(arg->data, SRSLTE_BCH_PAYLOADCRC_LEN).c_str(),
                     byte_hex_string("\t\t\t\t", arg->data, SRSLTE_BCH_PAYLOADCRC_LEN).c_str());

  return buf;
}
 

/* see lib/include/srslte/phy/phch/pcfich.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  int nof_symbols;
  uint32_t nof_rx_antennas;
  srslte_regs_t *regs;
  cf_t ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS][PCFICH_RE];
  cf_t symbols[SRSLTE_MAX_PORTS][PCFICH_RE];
  cf_t x[SRSLTE_MAX_PORTS][PCFICH_RE];
  cf_t d[PCFICH_RE];
  float cfi_table_float[3][PCFICH_CFI_LEN];
  uint8_t data[PCFICH_CFI_LEN];
  float data_f[PCFICH_CFI_LEN]; 
  srslte_modem_table_t mod;  
  srslte_sequence_t seq[SRSLTE_NSUBFRAMES_X_FRAME];
} srslte_pcfich_t; */
inline std::string pcfich_t_to_string(const srslte_pcfich_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "pcfich_t:n_symbols %u, n_rxports %u"
                                       "\n\t\t\t  data %s\n%s",
                     arg->nof_symbols,
                     arg->nof_rx_antennas,
                     bits_to_bytes_hex_string(arg->data, PCFICH_CFI_LEN).c_str(),
                     byte_hex_string("\t\t\t\t", arg->data, PCFICH_CFI_LEN).c_str());

  if(0)  // too noisy
   {
     for(int i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; ++i)
       {
         len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t seq[%2d] %s:",
                         i,
                         sequence_t_to_string(&arg->seq[i]).c_str());
       }
   }

  return buf;
}
 
/* see lib/include/srslte/phy/phch/pdcch.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t nof_regs[3];
  uint32_t nof_cce[3];
  uint32_t max_bits;
  uint32_t nof_rx_antennas;
  bool     is_ue;
  srslte_regs_t *regs;
  cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  cf_t *symbols[SRSLTE_MAX_PORTS];
  cf_t *x[SRSLTE_MAX_PORTS];
  cf_t *d;
  uint8_t *e;
  float rm_f[3 * (SRSLTE_DCI_MAX_BITS + 16)];
  float *llr;
  srslte_modem_table_t mod;
  srslte_sequence_t seq[SRSLTE_NSUBFRAMES_X_FRAME];
  srslte_viterbi_t decoder;
  srslte_crc_t crc;
} srslte_pdcch_t; */
inline std::string pdcch_t_to_string(const srslte_pdcch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "n_regs %u,%u,%u, n_cce %u,%u,%u, max_bits %u, n_rxports %u, crc %s, is_ue %s",
                     arg->nof_regs[0],
                     arg->nof_regs[1],
                     arg->nof_regs[2],
                     arg->nof_cce[0],
                     arg->nof_cce[1],
                     arg->nof_cce[2],
                     arg->max_bits,
                     arg->nof_rx_antennas,
                     crc_t_to_string(&arg->crc).c_str(),
                     YES_NO(arg->is_ue));

  if(0)
    for(int i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; ++i)
      {
        len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t seq[%2d] %s:",
                        i,
                        sequence_t_to_string(&arg->seq[i]).c_str());
      }


  return buf;
}

/* see lib/include/srslte/phy/phch/cqi.h
 typedef struct SRSLTE_API {
  uint8_t  wideband_cqi_cw0;      // 4-bit width
  uint32_t subband_diff_cqi_cw0;  // 2N-bit width
  uint8_t  wideband_cqi_cw1;      // if RI > 1 then 4-bit width otherwise 0-bit width
  uint32_t subband_diff_cqi_cw1;  // if RI > 1 then 2N-bit width otherwise 0-bit width
  uint32_t pmi;                   // if RI > 1 then 2-bit width otherwise 1-bit width
  uint32_t N;
  bool ri_present;
  bool pmi_present;
  bool four_antenna_ports;        // If cell has 4 antenna ports then true otherwise false
  bool rank_is_not_one;           // If rank > 1 then true otherwise false
} srslte_cqi_hl_subband_t;
typedef struct SRSLTE_API {
  uint8_t  wideband_cqi; // 4-bit width
  uint8_t  subband_diff_cqi; // 2-bit width
  uint32_t position_subband; // L-bit width
  uint32_t L;
} srslte_cqi_ue_subband_t;
 typedef struct SRSLTE_API {
  uint8_t  wideband_cqi; // 4-bit width
  uint8_t  spatial_diff_cqi; // If Rank==1 then it is 0-bit width otherwise it is 3-bit width
  uint8_t  pmi;
  bool pmi_present;
  bool four_antenna_ports;        // If cell has 4 antenna ports then true otherwise false
  bool rank_is_not_one;           // If rank > 1 then true otherwise false
} srslte_cqi_format2_wideband_t;
typedef struct SRSLTE_API {
  uint8_t  subband_cqi; // 4-bit width
  uint8_t  subband_label; // 1- or 2-bit width
  bool     subband_label_2_bits; // false, label=1-bit, true label=2-bits
} srslte_cqi_format2_subband_t;
typedef enum {
  SRSLTE_CQI_TYPE_WIDEBAND = 0,
  SRSLTE_CQI_TYPE_SUBBAND,
  SRSLTE_CQI_TYPE_SUBBAND_UE,
  SRSLTE_CQI_TYPE_SUBBAND_HL
} srslte_cqi_type_t; 
typedef struct {
  union {
    srslte_cqi_format2_wideband_t wideband;
    srslte_cqi_format2_subband_t  subband; 
    srslte_cqi_ue_subband_t       subband_ue;
    srslte_cqi_hl_subband_t       subband_hl;
  };
  srslte_cqi_type_t type; 
} srslte_cqi_value_t; */
inline std::string cqi_type_t_to_string(const srslte_cqi_type_t  arg)
{
  switch(arg)
    {
      case SRSLTE_CQI_TYPE_WIDEBAND:
        return "CQI_WIDEBAND";
      case SRSLTE_CQI_TYPE_SUBBAND:
        return "CQI_SUBBAND";
      case SRSLTE_CQI_TYPE_SUBBAND_UE:
        return "CQI_SUBBAND_UE";
      case SRSLTE_CQI_TYPE_SUBBAND_HL:
        return "CQI_SUBBAND_HL";
      default:
        return enumRangeError("InvalidCQIType", arg).c_str();
    }
}

inline std::string cqi_value_t_to_string(const srslte_cqi_value_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "type %s, ", cqi_type_t_to_string(arg->type).c_str());

   switch(arg->type)
    {
      case SRSLTE_CQI_TYPE_WIDEBAND:
        len += snprintf(buf + len, sizeof(buf) - len, "wb_cqi 0x%hhx, sptl_dif 0x%04hhx, pmi 0x%hhx, has_pmi %s, 4_ports %s, ri_n1 %s\n", 
                        arg->wideband.wideband_cqi, 
                        arg->wideband.spatial_diff_cqi,
                        arg->wideband.pmi,
                        YES_NO(arg->wideband.pmi_present), 
                        YES_NO(arg->wideband.four_antenna_ports),
                        YES_NO(arg->wideband.rank_is_not_one));
      break;

      case SRSLTE_CQI_TYPE_SUBBAND:
      case SRSLTE_CQI_TYPE_SUBBAND_UE:
      case SRSLTE_CQI_TYPE_SUBBAND_HL:
      default:
        len += snprintf(buf + len, sizeof(buf) - len, "XXX TODO CQI_TYPE %d\n", arg->type);
    }

  return buf;
}
 



/* see lib/include/srslte/phy/phch/uci.h
typedef struct SRSLTE_API {
  uint8_t  uci_cqi[SRSLTE_CQI_MAX_BITS];
  uint32_t uci_cqi_len;
  uint8_t  uci_ri;  // Only 1-bit supported for RI
  uint32_t uci_ri_len;
  uint8_t  uci_ack;   // 1st codeword bit for HARQ-ACK
  uint8_t  uci_ack_2; // 2st codeword bit for HARQ-ACK
  uint32_t uci_ack_len;
  bool ri_periodic_report;
  bool scheduling_request; 
  bool channel_selection; 
  bool cqi_ack; 
} srslte_uci_data_t; */
inline std::string uci_data_t_to_string(const srslte_uci_data_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "ri_len %u, ri %hhu, ack_len %u, uci_ack %hhu, uci_ack2 %hhu, ri_per %s, schd_req %s, chan_sel %s, cqi_ack %s, cqi_len %u, cqi:",
                     arg->uci_ri_len,
                     arg->uci_ri,
                     arg->uci_ack_len,
                     arg->uci_ack,
                     arg->uci_ack_2,
                     YES_NO(arg->ri_periodic_report),
                     YES_NO(arg->scheduling_request),
                     YES_NO(arg->channel_selection),
                     YES_NO(arg->cqi_ack),
                     arg->uci_cqi_len);


  for(uint32_t n = 0; n < arg->uci_cqi_len; ++n)
   {
     len += snprintf(buf + len, sizeof(buf) -len, "%hhu", arg->uci_cqi[n]);
   }

  len += snprintf(buf + len, sizeof(buf) -len, "\n");

  return buf;
}

/* see lib/include/srslte/phy/phch/sch.h
typedef struct SRSLTE_API {
  uint32_t max_iterations;
  uint32_t nof_iterations;
  uint8_t *cb_in; 
  uint8_t *parity_bits;  
  void *e;
  uint8_t *temp_g_bits;
  uint16_t *ul_interleaver;
  srslte_uci_bit_t ack_ri_bits[12*288];
  uint32_t nof_ri_ack_bits; 
  srslte_tcod_t encoder;
  srslte_tdec_t decoder;  
  srslte_crc_t crc_tb;
  srslte_crc_t crc_cb;
  srslte_uci_cqi_pusch_t uci_cqi;
} srslte_sch_t; */
inline std::string sch_t_to_string(const srslte_sch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "max_it %u, n_it %u, n_ri_ack_bits %u",
                     arg->max_iterations,
                     arg->nof_iterations,
                     arg->nof_ri_ack_bits);

  return buf;
}

 
/* see lib/include/srslte/phy/phch/pdsch.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t nof_rx_antennas;
  uint32_t last_nof_iterations[SRSLTE_MAX_CODEWORDS];
  uint32_t max_re;
  uint16_t ue_rnti;
  bool is_ue;
  // Power allocation parameter 3GPP 36.213 Clause 5.2 Rho_b
  float rho_a;
  cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS]; // Channel estimation (Rx only)
  cf_t *symbols[SRSLTE_MAX_PORTS];              // PDSCH Encoded/Decoded Symbols
  cf_t *x[SRSLTE_MAX_LAYERS];                   // Layer mapped
  cf_t *d[SRSLTE_MAX_CODEWORDS];                // Modulated/Demodulated codewords
  void *e[SRSLTE_MAX_CODEWORDS];
  bool csi_enabled;
  float *csi[SRSLTE_MAX_CODEWORDS];             // Channel Strengh Indicator
  srslte_modem_table_t mod[4];
  srslte_pdsch_user_t **users;
  srslte_sequence_t tmp_seq;
  srslte_sch_t dl_sch;
} srslte_pdsch_t; */
inline std::string pdsch_t_to_string(const srslte_pdsch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "n_rxports %u, max_re %u, ue_rnti 0x%hx, is_ue %s, csi_en %s, rho_a %f"
                                       "\n\t\t\t dl_sch %s",
                     arg->nof_rx_antennas,
                     arg->max_re,
                     arg->ue_rnti,
                     YES_NO(arg->is_ue),
                     YES_NO(arg->csi_enabled),
                     arg->rho_a,
                     sch_t_to_string(&arg->dl_sch).c_str());
                   
  return buf;
}
 

/* see lib/include/srslte/phy/phch/phich.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t nof_rx_antennas; 
  srslte_regs_t *regs;
  cf_t ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS][SRSLTE_PHICH_MAX_NSYMB];
  cf_t sf_symbols[SRSLTE_MAX_PORTS][SRSLTE_PHICH_MAX_NSYMB];
  cf_t x[SRSLTE_MAX_PORTS][SRSLTE_PHICH_MAX_NSYMB];
  cf_t d[SRSLTE_PHICH_MAX_NSYMB];
  cf_t d0[SRSLTE_PHICH_MAX_NSYMB];
  cf_t z[SRSLTE_PHICH_NBITS];
  uint8_t data[SRSLTE_PHICH_NBITS];
  float data_rx[SRSLTE_PHICH_NBITS];
  srslte_modem_table_t mod;
  srslte_sequence_t seq[SRSLTE_NSUBFRAMES_X_FRAME];
} srslte_phich_t; */
inline std::string phich_t_to_string(const srslte_phich_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "n_rxports %u"
                                       "\n\t\t\t  data\n%s",
                     arg->nof_rx_antennas,
                     byte_hex_string("\t\t\t\t", arg->data,SRSLTE_PHICH_NBITS).c_str());

  if(0) // too noisy
    for(int i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; ++i)
      {
        len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t seq[%2d] %s:",
                        i,
                        sequence_t_to_string(&arg->seq[i]).c_str());
      }

  return buf;
}
 
/* see lib/include/srslte/phy/ch_estimation/refsignal_dl.h
 struct SRSLTE_API {
  srslte_cell_t cell; 
  cf_t         *pilots[2][SRSLTE_NSUBFRAMES_X_FRAME]; // Saves the reference signal per subframe for ports 0,1 and ports 2,3
  srslte_sf_t   type;
  uint16_t      mbsfn_area_id;
} srslte_refsignal_t; */
inline std::string refsignal_t_to_string(const srslte_refsignal_t * arg)
{
  char buf[128] = {0};

  int len = snprintf(buf, sizeof(buf), "sf_type %s, mbsfn_area_id %hu",
                     sf_t_to_name(arg->type).c_str(),
                     arg->mbsfn_area_id);

  return buf;
}


/* lib/include/srslte/phy/phch/ra.h
typedef struct {
  uint32_t lstart; 
  uint32_t nof_symb; 
  uint32_t nof_bits;
  uint32_t nof_re; 
} srslte_ra_nbits_t; */
inline std::string ra_nbits_t_to_string(const srslte_ra_nbits_t * arg)
{
  char buf[128] = {0};

  int len = snprintf(buf, sizeof(buf), "lstart %2u, n_symb %2u, n_bits %2u, n_re %2u",
                     arg->lstart,
                     arg->nof_symb,
                     arg->nof_bits,
                     arg->nof_re); 

  return buf;
}


/* see lib/include/srslte/phy/phch/ra.h
typedef struct SRSLTE_API {
  srslte_mod_t mod;
  int tbs;
  uint32_t idx; 
} srslte_ra_mcs_t; */
inline std::string ra_mcs_t_to_string(const srslte_ra_mcs_t * arg)
{
  char buf[64] = {0};

  int len = snprintf(buf, sizeof(buf), "mod_type (%d) %s, mcs_tbs %2d, mcs_idx %2u",
                     arg->mod,
                     mod_t_to_name(arg->mod).c_str(),
                     arg->tbs,
                     arg->idx); 

  return buf;
}


/* lib/include/srslte/phy/phch/ra.h
typedef struct SRSLTE_API {
  uint32_t n_prb[2];
  uint32_t n_prb_tilde[2];
  uint32_t L_prb;
  uint32_t freq_hopping; 
  uint32_t M_sc; 
  uint32_t M_sc_init; 
  uint32_t Qm; 
  srslte_ra_mcs_t mcs;
  uint32_t ncs_dmrs;
} srslte_ra_ul_grant_t; */
inline std::string ra_ul_grant_t_to_string(const srslte_ra_ul_grant_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "n_prb[%u, %u], n_prb~[%u, %u], L_prb %u, freq_hop %u, M_sc %u, M_sc_init %u, Qm %u, ncs_dmrs %u"
                                       "\n\t\t\t   ra_mcs:%s",
                     arg->n_prb[0],
                     arg->n_prb[1],
                     arg->n_prb_tilde[0],
                     arg->n_prb_tilde[1],
                     arg->L_prb,
                     arg->freq_hopping,
                     arg->M_sc,
                     arg->M_sc_init,
                     arg->Qm,
                     arg->ncs_dmrs,
                     ra_mcs_t_to_string(&arg->mcs).c_str());

  return buf;
}


/* lib/include/srslte/phy/phch/ra.h
typedef struct SRSLTE_API {
  bool prb_idx[2][SRSLTE_MAX_PRB];
  uint32_t nof_prb;  
  uint32_t Qm[SRSLTE_MAX_CODEWORDS];
  srslte_ra_mcs_t mcs[SRSLTE_MAX_CODEWORDS];
  srslte_sf_t sf_type;
  bool tb_en[SRSLTE_MAX_CODEWORDS];
  uint32_t pinfo;
  bool tb_cw_swap;
} srslte_ra_dl_grant_t;
#define SRSLTE_RA_DL_GRANT_NOF_TB(G) ((((G)->tb_en[0])?1:0)+(((G)->tb_en[1])?1:0)) */
inline std::string ra_dl_grant_t_to_string(const srslte_ra_dl_grant_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "n_prb %u, sf_type %s, pinfo %u, n_tb %u, tb_swap %s",
                     arg->nof_prb,
                     sf_t_to_name(arg->sf_type).c_str(),
                     arg->pinfo,
                     SRSLTE_RA_DL_GRANT_NOF_TB(arg),
                     YES_NO(arg->tb_cw_swap));

  for(int tb = 0; tb < SRSLTE_RA_DL_GRANT_NOF_TB(arg); ++tb)
   {
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t   tb[%d]: Qm %2u, mcs %s, tb_en %s",
                                                    tb,
                                                    arg->Qm[tb],
                                                    ra_mcs_t_to_string(&arg->mcs[tb]).c_str(),
                                                    YES_NO(arg->tb_en[tb]));
   }

  return buf;
}


/* lib/include/srslte/phy/phch/pdsch_cfg.h 
typedef struct SRSLTE_API {
  srslte_cbsegm_t cb_segm[SRSLTE_MAX_CODEWORDS];
  srslte_ra_dl_grant_t grant;
  srslte_ra_nbits_t nbits[SRSLTE_MAX_CODEWORDS];
  uint32_t rv[SRSLTE_MAX_CODEWORDS];
  uint32_t sf_idx;
  uint32_t nof_layers;
  uint32_t codebook_idx;
  srslte_mimo_type_t mimo_type;
  bool tb_cw_swap;
} srslte_pdsch_cfg_t; */
inline std::string pdsch_cfg_t_to_string(const srslte_pdsch_cfg_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "sf_idx %u, n_layers %u, cb_idx %u, tb_swap %s, mimo type:%s\n\t\t\t  ra_dl_grant:%s",
                     arg->sf_idx,
                     arg->nof_layers,
                     arg->codebook_idx,
                     YES_NO(arg->tb_cw_swap),
                     mimo_type_t_to_name(arg->mimo_type).c_str(),
                     ra_dl_grant_t_to_string(&arg->grant).c_str());

  for(int tb = 0; tb < SRSLTE_RA_DL_GRANT_NOF_TB(&arg->grant); ++tb)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t tb[%d]: rv %2u, cb_segm:%s, ra_nbits:%s",
                                                    tb, 
                                                    arg->rv[tb],
                                                    cbsegm_t_to_string(&arg->cb_segm[tb]).c_str(),
                                                    ra_nbits_t_to_string(&arg->nbits[tb]).c_str());
   }

  return buf;
}

/* see lib/include/srslte/phy/phch/pmch.h
 typedef struct {
  srslte_sequence_t seq[SRSLTE_NSUBFRAMES_X_FRAME];  
} srslte_pmch_seq_t;

typedef struct SRSLTE_API {
  srslte_cbsegm_t cb_segm;
  srslte_ra_dl_grant_t grant;
  srslte_ra_nbits_t nbits[SRSLTE_MAX_CODEWORDS];
  uint32_t sf_idx;
} srslte_pmch_cfg_t;

typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t nof_rx_antennas;
  uint32_t max_re;
  // void buffers are shared for tx and rx
  cf_t *ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  cf_t *symbols[SRSLTE_MAX_PORTS];
  cf_t *x[SRSLTE_MAX_PORTS];
  cf_t *d;
  void *e;
  // tx & rx objects
  srslte_modem_table_t mod[4];
  // This is to generate the scrambling seq for multiple MBSFN Area IDs
  srslte_pmch_seq_t **seqs;
  srslte_sch_t dl_sch;
} srslte_pmch_t; */
inline std::string pmch_t_to_string(const srslte_pmch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "n_rxports %u, max_re %u"
                                       "\n\t\t\t  cell:  %s"
                                       "\n\t\t\t  dl_sch:%s",
                     arg->nof_rx_antennas,
                     arg->max_re,
                     cell_t_to_string(&arg->cell).c_str(),
                     sch_t_to_string(&arg->dl_sch).c_str());

  return buf;
}



/* see lib/include/srslte/phy/enb/enb_dl.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  cf_t *sf_symbols[SRSLTE_MAX_PORTS]; 
  cf_t *slot1_symbols[SRSLTE_MAX_PORTS];
  srslte_ofdm_t   ifft[SRSLTE_MAX_PORTS];
  srslte_ofdm_t ifft_mbsfn;
  srslte_pbch_t   pbch;
  srslte_pcfich_t pcfich;
  srslte_regs_t   regs;
  srslte_pdcch_t  pdcch;
  srslte_pdsch_t  pdsch;
  srslte_pmch_t   pmch;
  srslte_phich_t  phich; 
  srslte_refsignal_t csr_signal;
  srslte_refsignal_t mbsfnr_signal;
  srslte_pdsch_cfg_t pdsch_cfg;
  srslte_pdsch_cfg_t pmch_cfg;
  srslte_ra_dl_dci_t dl_dci;
  srslte_dci_format_t dci_format;
  uint32_t cfi;
  cf_t pss_signal[SRSLTE_PSS_LEN];
  float sss_signal0[SRSLTE_SSS_LEN]; 
  float sss_signal5[SRSLTE_SSS_LEN]; 
  float tx_amp;
  float rho_b;
  uint8_t tmp[1024*128];
} srslte_enb_dl_t; */
inline std::string enb_dl_t_to_string(const srslte_enb_dl_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "cfi %u, tx_amp %f, rho_b %f"
                                       "\n\t\tcell:  %s"
                                       "\n\t\tpbch:  %s"
                                       "\n\t\tpcfich:%s"
                                       "\n\t\tregs:  %s"
                                       "\n\t\tpdcch: %s"
                                       "\n\t\tpdsch: %s"
                                       "\n\t\tpmsch: %s"
                                       "\n\t\tphich: %s"
                                       "\n\t\tref_signal:%s"
                                       "\n\t\tpdsch_cfg: %s"
                                       "\n\t\tpmsch_cfg: %s"
                                       "\n\t\tdci_format:%s"
                                       "\n\t\tdl_dci:    %s",
                     arg->cfi,
                     arg->tx_amp,
                     arg->rho_b,
                     cell_t_to_string(&arg->cell).c_str(),
                     pbch_t_to_string(&arg->pbch).c_str(),
                     pcfich_t_to_string(&arg->pcfich).c_str(),
                     regs_t_to_string(&arg->regs).c_str(),
                     pdcch_t_to_string(&arg->pdcch).c_str(),
                     pdsch_t_to_string(&arg->pdsch).c_str(),
                     pmch_t_to_string(&arg->pmch).c_str(),
                     phich_t_to_string(&arg->phich).c_str(),
                     refsignal_t_to_string(&arg->csr_signal).c_str(),
                     pdsch_cfg_t_to_string(&arg->pdsch_cfg).c_str(),
                     pdsch_cfg_t_to_string(&arg->pmch_cfg).c_str(),
                     dci_format_t_to_name(arg->dci_format).c_str(),
                     ra_dl_dci_t_to_string(&arg->dl_dci).c_str());

  return buf;
}


/* see lib/include/srslte/interfaces/ue_interfaces.h
typedef struct {
  bool                    decode_enabled[SRSLTE_MAX_TB];
  int                     rv[SRSLTE_MAX_TB];
  uint16_t                rnti; 
  bool                    generate_ack; 
  bool                    default_ack[SRSLTE_MAX_TB];
  // If non-null, called after tb_decoded_ok to determine if ack needs to be sent
  bool                  (*generate_ack_callback)(void*); 
  void                   *generate_ack_callback_arg;
  uint8_t                *payload_ptr[SRSLTE_MAX_TB];
  srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_TB];
  srslte_phy_grant_t      phy_grant;
} tb_action_dl_t; */
inline std::string ue_tb_action_dl_t_to_string(const srsue::mac_interface_phy::tb_action_dl_t * arg)
{
  char buf[8912] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%x, gen_ack %s",
                     arg->rnti,
                     YES_NO(arg->generate_ack));

  for(int tb = 0; tb < SRSLTE_RA_DL_GRANT_NOF_TB(&arg->phy_grant.dl); ++tb)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t tb[%d]: dec_en %s, rv %d, def_ack %s, payload_ptr %p",
                                                    tb,
                                                    YES_NO(arg->decode_enabled[tb]),
                                                    arg->rv[tb],
                                                    YES_NO(arg->default_ack[tb]),
                                                    arg->payload_ptr);
   }

  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  ra_dl_grant:%s",
                  ra_dl_grant_t_to_string(&arg->phy_grant.dl).c_str());

  return buf;
}

/* see lib/include/srslte/interfaces/ue_interfaces.h
typedef struct {
    bool                    tx_enabled;
    bool                    expect_ack;
    uint32_t                rv[SRSLTE_MAX_TB];
    uint16_t                rnti; 
    uint32_t                current_tx_nb;
    int32_t                 tti_offset;     // relative offset between grant and UL tx/HARQ rx
    srslte_softbuffer_tx_t *softbuffers;
    srslte_phy_grant_t      phy_grant;
    uint8_t                *payload_ptr[SRSLTE_MAX_TB];
  } tb_action_ul_t; */
inline std::string ue_tb_action_ul_t_to_string(const srsue::mac_interface_phy::tb_action_ul_t * arg)
{
  char buf[4096] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%x, tx_en %s, expct_ack %s, rv[%u, %u], cur_tx_nb %u"
                     "\n\t\t\t phy_grant_ul:%s",
                     arg->rnti,
                     YES_NO(arg->tx_enabled),
                     YES_NO(arg->expect_ack),
                     arg->rv[0], arg->rv[1],
                     arg->current_tx_nb,
                     ra_ul_grant_t_to_string(&arg->phy_grant.ul).c_str());

  return buf;
}


/* see lib/include/srslte/phy/phch/ra.h
typedef union {
  srslte_ra_ul_grant_t ul;
  srslte_ra_dl_grant_t dl;
} srslte_phy_grant_t; */

/* see lib/include/srslte/interfaces/ue_interfaces.h
  typedef struct {
    uint32_t    pid;    
    uint32_t    tti;
    uint32_t    last_tti;
    bool        ndi[SRSLTE_MAX_CODEWORDS];
    bool        last_ndi[SRSLTE_MAX_CODEWORDS];
    uint32_t    n_bytes[SRSLTE_MAX_CODEWORDS];
    int         rv[SRSLTE_MAX_CODEWORDS];
    bool        tb_en[SRSLTE_MAX_CODEWORDS];
    bool        tb_cw_swap;
    uint16_t    rnti; 
    bool        is_from_rar;
    bool        is_sps_release;
    bool        has_cqi_request;
    srslte_rnti_type_t rnti_type; 
    srslte_phy_grant_t phy_grant; 
  } mac_grant_t; */
inline std::string ue_mac_grant_t_to_string(const srsue::mac_interface_phy::mac_grant_t * arg)
{
  char buf[8912] = {0};

  int len = snprintf(buf, sizeof(buf), "pid %u, tti %u, last_tti %u, rnti 0x%x, tb_swap %s, rnti_type %s, n_tb %u, cqi_req %s, from_rar %s",
                     arg->pid,
                     arg->tti,
                     arg->last_tti,
                     arg->rnti,
                     YES_NO(arg->tb_cw_swap),
                     rnti_type_t_to_name(arg->rnti_type).c_str(),
                     SRSLTE_RA_DL_GRANT_NOF_TB(arg),
                     YES_NO(arg->has_cqi_request),
                     YES_NO(arg->is_from_rar));

  for(int tb = 0; tb < SRSLTE_MAX_CODEWORDS; ++tb)
   {
     len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t   tb[%d]: ndi %2u, last_ndi %2u, n_bytes %2u, rv %2u, tb_en %2u",
                     tb,
                     arg->ndi[tb],
                     arg->last_ndi[tb],
                     arg->n_bytes[tb],
                     arg->rv[tb],
                     arg->tb_en[tb]);
   }

  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t ra_dl_grant:%s",
                  ra_dl_grant_t_to_string(&arg->phy_grant.dl).c_str());

  return buf;
}


/* see lib/include/srslte/phy/phch/ra.h */
inline const char * freq_hop_fl_to_name(int arg)
{
   switch(arg)
   {
     case -1:
       return "Disabled";
     case 0:
       return "Quart";
     case 1:
       return "QuartNeg";
     case 2:
       return "Half";
     case 3:
       return "Type2";
     default:
       return "InvalidiHopFlType";
    } 
}



/* see lib/include/srslte/phy/phch/ra.h 
typedef struct SRSLTE_API {
  // 36.213 Table 8.4-2: SRSLTE_RA_PUSCH_HOP_HALF is 0 for < 10 Mhz and 10 for > 10 Mhz.
  // SRSLTE_RA_PUSCH_HOP_QUART is 00 for > 10 Mhz and SRSLTE_RA_PUSCH_HOP_QUART_NEG is 01 for > 10 Mhz.
  enum {
    SRSLTE_RA_PUSCH_HOP_DISABLED  =-1,
    SRSLTE_RA_PUSCH_HOP_QUART     = 0,
    SRSLTE_RA_PUSCH_HOP_QUART_NEG = 1,
    SRSLTE_RA_PUSCH_HOP_HALF      = 2,
    SRSLTE_RA_PUSCH_HOP_TYPE2     = 3
  } freq_hop_fl;
  srslte_ra_ul_grant_t prb_alloc;  // NOT USED
  srslte_ra_type2_t type2_alloc;
  uint32_t mcs_idx;
  uint32_t rv_idx;   
  uint32_t n_dmrs; 
  bool ndi;
  bool cqi_request;
  uint8_t tpc_pusch;
} srslte_ra_ul_dci_t; */
inline std::string ra_ul_dci_t_to_string(const srslte_ra_ul_dci_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "mcs_idx %u, rv_idx %u, n_dmrs %u, ndi %s, cqi_req %s, tpc_pusch %u, hop_type %s"
                                       "\n\t\t\t grant:      %s"
                                       "\n\t\t\t type2_alloc:%s",
                     arg->mcs_idx,
                     arg->rv_idx,
                     arg->n_dmrs,
                     YES_NO(arg->ndi),
                     YES_NO(arg->cqi_request),
                     arg->tpc_pusch,
                     freq_hop_fl_to_name(arg->freq_hop_fl),
                     ra_ul_grant_t_to_string(&arg->prb_alloc).c_str(),
                     ra_type2_t_to_string(&arg->type2_alloc).c_str());

  return buf;
}



/* lib/include/srslte/phy/enb/enb_ul.h 
typedef struct {
  uint16_t                rnti; 
  srslte_ra_ul_dci_t      grant;
  srslte_dci_location_t   location; 
  uint32_t                rv_idx; 
  uint32_t                current_tx_nb; 
  uint8_t                *data; 
  srslte_softbuffer_rx_t *softbuffer;
  bool                    needs_pdcch; 
} srslte_enb_ul_pusch_t; */
inline std::string enb_ul_pusch_t_to_string(const srslte_enb_ul_pusch_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, rv_idx %u, cur_tx_nb %u, needs_pdcch %s, has_data %s"
                                       "\n\t\t\t   location:%s"
                                       "\n\t\t\t   ul_dci:  %s",
                     arg->rnti,
                     arg->rv_idx,
                     arg->current_tx_nb,
                     YES_NO(arg->needs_pdcch),
                     YES_NO(arg->data),
                     dci_location_t_to_string(&arg->location).c_str(),
                     ra_ul_dci_t_to_string(&arg->grant).c_str());

  return buf;
}


/* see lib/include/srslte/phy/enb/enb_dl.h
typedef struct {
  uint16_t rnti; 
  uint8_t  ack;
  uint32_t n_prb_lowest;
  uint32_t n_dmrs;  
} srslte_enb_dl_phich_t; */
inline std::string enb_dl_phich_t_to_string(const srslte_enb_dl_phich_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, ack %hhu, n_prb_low %u, n_dmrs %u",
                     arg->rnti,
                     arg->ack,
                     arg->n_prb_lowest,
                     arg->n_dmrs);

  return buf;
}


/* see lib/include/srslte/interfaces/sched_interface.h
typedef struct {
    uint32_t              rnti;
    bool                  needs_pdcch; 
    uint32_t              current_tx_nb; 
    uint32_t              tbs; 
    srslte_ra_ul_dci_t    dci;     
    srslte_dci_location_t dci_location;
  } ul_sched_data_t; */
inline std::string enb_ul_sched_data_t_to_string(const srsenb::sched_interface::ul_sched_data_t * arg)
{
  char buf[1024] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, needs_pdcch %s, cur_tx_nb %u, tbs %u, dci:%s, dci_loc: %s",
                     arg->rnti,
                     YES_NO(arg->needs_pdcch),
                     arg->current_tx_nb,
                     arg->tbs,
                     ra_ul_dci_t_to_string(&arg->dci).c_str(),
                     dci_location_t_to_string(&arg->dci_location).c_str());

  return buf;
}


/* see lib/include/srslte/interfaces/sched_interface.h
typedef struct {
    uint16_t rnti; 
    enum phich_elem {
      ACK, NACK
    } phich;
  } ul_sched_phich_t; */
inline std::string ul_sched_phich_t_to_string(const srsenb::sched_interface::ul_sched_phich_t * arg)
{
  char buf[128] = {0};

  int len = snprintf(buf, sizeof(buf), "rnti 0x%hx, phich %s",
                     arg->rnti,
                     arg->phich ? "NACK" : "ACK");

  return buf;
}



/* see lib/include/srslte/interfaces/enb_interfaces.h
  typedef struct {
    srslte_enb_ul_pusch_t sched_grants[MAX_GRANTS];
    srslte_enb_dl_phich_t phich[MAX_GRANTS];
    uint32_t nof_grants; 
    uint32_t nof_phich; 
  } ul_sched_t; */

/* see lib/include/srslte/interfaces/sched_interface.h
  typedef struct {
    uint32_t nof_dci_elems; 
    uint32_t nof_phich_elems; 
    ul_sched_data_t  pusch[MAX_DATA_LIST];
    ul_sched_phich_t phich[MAX_PHICH_LIST];
  } ul_sched_res_t; */
inline std::string ul_sched_t_to_string(const srsenb::mac_interface_phy::ul_sched_t   * ul_sched,
                                        const srsenb::sched_interface::ul_sched_res_t * ulsched_res)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "n_ulgrants %u, n_ul_phich %u",
                     ul_sched->nof_grants,
                     ul_sched->nof_phich);

  for(uint32_t i = 0; i < ul_sched->nof_grants; ++i)
    {
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t grant[%d] %s",
                      i,
                      enb_ul_pusch_t_to_string(&ul_sched->sched_grants[i]).c_str());
    }

  for(uint32_t i = 0; i < ul_sched->nof_phich; ++i)
    {
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t phich[%d] %s",
                      i,
                      enb_dl_phich_t_to_string(&ul_sched->phich[i]).c_str());
    }

  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t ulsched_res: n_dci_elems %u, n_phich_elems %u",
                  ulsched_res->nof_dci_elems,
                  ulsched_res->nof_phich_elems);

  for(uint32_t i = 0; i < ulsched_res->nof_dci_elems; ++i)
    {
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  pusch[%d] %s",
                      i,
                      enb_ul_sched_data_t_to_string(&ulsched_res->pusch[i]).c_str());
    }

  for(uint32_t i = 0; i < ulsched_res->nof_phich_elems; ++i)
    {
      len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t\t  phich[%d] %s",
                      i,
                      ul_sched_phich_t_to_string(&ulsched_res->phich[i]).c_str());
    }
 
  return buf;
}


/* see lib/include/srslte/phy/phch/pusch_cfg.h
typedef struct SRSLTE_API {
  srslte_cbsegm_t cb_segm; 
  srslte_ra_ul_grant_t grant; 
  srslte_ra_nbits_t nbits; 
  srslte_uci_cfg_t uci_cfg; 
  uint32_t rv; 
  uint32_t sf_idx;
  uint32_t tti; 
  srslte_cp_t cp; 
  uint32_t last_O_cqi;
} srslte_pusch_cfg_t; */
inline std::string pusch_cfg_t_to_string(const srslte_pusch_cfg_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "rv %u, sf_idx %u, tti %u, cp_type %s, last_O_cqi %u, Ioff[cqi %u, ri %u, ack %u]"
                                       "\n\t\t\t  cb_segm: %s"
                                       "\n\t\t\t  grant:   %s"
                                       "\n\t\t\t  ra_nbits:%s",
                     arg->rv,
                     arg->sf_idx,
                     arg->tti,
                     cp_t_to_name(arg->cp).c_str(),
                     arg->last_O_cqi,
                     arg->uci_cfg.I_offset_cqi,
                     arg->uci_cfg.I_offset_ri,
                     arg->uci_cfg.I_offset_ack,
                     cbsegm_t_to_string(&arg->cb_segm).c_str(),
                     ra_ul_grant_t_to_string(&arg->grant).c_str(),
                     ra_nbits_t_to_string(&arg->nbits).c_str());

  return buf;
}


/* see lib/include/srslte/phy/phch/pusch.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  bool is_ue;
  uint16_t ue_rnti;
  uint32_t max_re;
  srslte_dft_precoding_t dft_precoding;  
  cf_t *ce;
  cf_t *z;
  cf_t *d;
  void *q;
  void *g;
  srslte_modem_table_t mod[4];
  srslte_sequence_t seq_type2_fo; 
  srslte_pusch_user_t **users;
  srslte_sequence_t tmp_seq;
  srslte_sch_t ul_sch;
  bool shortened;
}srslte_pusch_t; */
inline std::string pusch_t_to_string(const srslte_pusch_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "max_re %u, ue_rnti 0x%hx, is_ue %s"
                                       "\n\t\t\t ul_sch:%s",
                     arg->max_re,
                     arg->ue_rnti,
                     YES_NO(arg->is_ue),
                     sch_t_to_string(&arg->ul_sch).c_str());                   

  return buf;
}

 
/* see lib/include/srslte/phy/phch/pucch.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  srslte_pucch_cfg_t pucch_cfg;
  srslte_modem_table_t mod; 
  srslte_uci_cqi_pucch_t cqi; 
  srslte_pucch_user_t **users;
  uint8_t bits_scram[SRSLTE_PUCCH_MAX_BITS];
  cf_t d[SRSLTE_PUCCH_MAX_BITS/2];
  uint32_t n_cs_cell[SRSLTE_NSLOTS_X_FRAME][SRSLTE_CP_NORM_NSYMB]; 
  uint32_t f_gh[SRSLTE_NSLOTS_X_FRAME];
  float tmp_arg[SRSLTE_PUCCH_N_SEQ];
  cf_t *z;
  cf_t *z_tmp;
  cf_t *ce;
  bool shortened; 
  bool group_hopping_en;
  float threshold_format1;
  float last_corr;
  uint32_t last_n_prb;
  uint32_t last_n_pucch;
}srslte_pucch_t; */
inline std::string pucch_t_to_string(const srslte_pucch_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "grp_hop_en %s, last_corr %f, thresh_f1 %f, last_n_prb %u, last_n_pucch %u",
                     YES_NO(arg->group_hopping_en),
                     arg->last_corr,
                     arg->threshold_format1,
                     arg->last_n_prb,
                     arg->last_n_pucch);
  return buf;
}


/* see lib/include/srslte/phy/phch/prach.h
typedef struct SRSLTE_API {
  // Parameters from higher layers (extracted from SIB2)
  uint32_t config_idx; 
  uint32_t f;               // preamble format
  uint32_t rsi;             // rootSequenceIndex
  bool hs;                  // highSpeedFlag
  uint32_t zczc;            // zeroCorrelationZoneConfig
  uint32_t N_ifft_ul;       // IFFT size for uplink
  uint32_t N_ifft_prach;    // IFFT size for PRACH generation
  uint32_t max_N_ifft_ul;
  // Working parameters
  uint32_t N_zc;  // PRACH sequence length
  uint32_t N_cs;  // Cyclic shift size
  uint32_t N_seq; // Preamble length
  float    T_seq; // Preamble length in seconds
  float    T_tot; // Total sequence length in seconds
  uint32_t N_cp;  // Cyclic prefix length
  // Generated tables
  cf_t seqs[64][839];         // Our set of 64 preamble sequences
  cf_t dft_seqs[64][839];     // DFT-precoded seqs
  uint32_t root_seqs_idx[64]; // Indices of root seqs in seqs table
  uint32_t N_roots;           // Number of root sequences used in this configuration
  // Containers
  cf_t *ifft_in;
  cf_t *ifft_out;
  cf_t *prach_bins;
  cf_t *corr_spec;
  float *corr;
  // PRACH IFFT
  srslte_dft_plan_t fft;
  srslte_dft_plan_t ifft;
  // ZC-sequence FFT and IFFT
  srslte_dft_plan_t zc_fft;
  srslte_dft_plan_t zc_ifft;
  cf_t *signal_fft; 
  float detect_factor; 
  uint32_t deadzone; 
  float    peak_values[65];
  uint32_t peak_offsets[65];
} srslte_prach_t; */
inline std::string enb_prach_t_to_string(const srslte_prach_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "config_idx %u, preamble_fmat %u, rsi %u, n_roots %u, dz %u",
                     arg->config_idx,
                     arg->f,
                     arg->rsi,
                     arg->N_roots,
                     arg->deadzone);
  return buf;
}


/* see lib/include/srslte/phy/phch/pusch.h
typedef struct {
  enum {
    SRSLTE_PUSCH_HOP_MODE_INTER_SF = 1,
    SRSLTE_PUSCH_HOP_MODE_INTRA_SF = 0
  } hop_mode; 
  uint32_t hopping_offset;
  uint32_t n_sb;
} srslte_pusch_hopping_cfg_t; */
inline std::string enb_pusch_hopping_cfg_t_to_string(const srslte_pusch_hopping_cfg_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "hop_mode %s, hop_offset %u, n_sb %u",
                     arg->hop_mode ? "INTER_SF" : "INTRA_SF",
                     arg->hopping_offset,
                     arg->n_sb);

  return buf;
}


/* see lib/include/srslte/phy/enb/enb_ul.h
typedef struct SRSLTE_API {
  srslte_cell_t cell;
  cf_t *sf_symbols; 
  cf_t *ce; 
  srslte_ofdm_t     fft;
  srslte_chest_ul_t chest;
  srslte_pusch_t    pusch;
  srslte_pucch_t    pucch;
  srslte_prach_t    prach;
  srslte_pusch_cfg_t         pusch_cfg; 
  srslte_pusch_hopping_cfg_t hopping_cfg;
  srslte_enb_ul_user_t **users; 
} srslte_enb_ul_t; */
inline std::string enb_ul_t_to_string(const srslte_enb_ul_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "enb_ul_t: pilot_power %f, noise_est %f"
                                       "\n\t\t\t cell:     %s"
                                       "\n\t\t\t pusch_cfg:%s"
                                       "\n\t\t\t pusch:    %s"
                                       "\n\t\t\t pucch:    %s"
                                       "\n\t\t\t prach:    %s"
                                       "\n\t\t\t pusch_hop_cfg:%s",
                     arg->chest.pilot_power,
                     arg->chest.noise_estimate,
                     cell_t_to_string(&arg->cell).c_str(),
                     pusch_cfg_t_to_string(&arg->pusch_cfg).c_str(),
                     pusch_t_to_string(&arg->pusch).c_str(),
                     pucch_t_to_string(&arg->pucch).c_str(),
                     enb_prach_t_to_string(&arg->prach).c_str(),
                     enb_pusch_hopping_cfg_t_to_string(&arg->hopping_cfg).c_str());

  return buf;
}

/* see lib/include/srslte/phy/sync/sync.h
typedef struct SRSLTE_API {
  srslte_pss_t pss;
  srslte_pss_t pss_i[2];
  srslte_sss_t sss;
  srslte_cp_synch_t cp_synch;
  cf_t *cfo_i_corr[2];
  int decimate;
  float threshold;
  float peak_value;
  uint32_t N_id_2;
  uint32_t N_id_1;
  uint32_t sf_idx;
  uint32_t fft_size;
  uint32_t frame_size;
  uint32_t max_offset;
  uint32_t nof_symbols;
  uint32_t cp_len;
  float current_cfo_tol;
  sss_alg_t sss_alg; 
  bool detect_cp;
  bool sss_en;
  srslte_cp_t cp;
  uint32_t m0;
  uint32_t m1;
  float m0_value;
  float m1_value;
  float M_norm_avg; 
  float M_ext_avg; 
  cf_t  *temp;
  uint32_t max_frame_size;
  // variables for various CFO estimation methods
  bool cfo_cp_enable;
  bool cfo_pss_enable;
  bool cfo_i_enable;
  bool cfo_cp_is_set;
  bool cfo_pss_is_set;
  bool cfo_i_initiated;
  float cfo_cp_mean;
  float cfo_pss;
  float cfo_pss_mean;
  int   cfo_i_value;
  float cfo_ema_alpha;
  uint32_t cfo_cp_nsymbols;
  srslte_cfo_t cfo_corr_frame;
  srslte_cfo_t cfo_corr_symbol;
  bool sss_channel_equalize;
  bool pss_filtering_enabled;
  cf_t sss_filt[SRSLTE_SYMBOL_SZ_MAX];
  cf_t pss_filt[SRSLTE_SYMBOL_SZ_MAX];
}srslte_sync_t; */
inline std::string sync_t_to_string(const srslte_sync_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "sync_t: N_id_1 %u, N_id_1 %u, sf_idx %u, fr_len %u, max_off %u, n_sym %u, cp_len %u",
                     arg->N_id_2,
                     arg->N_id_1,
                     arg->sf_idx,
                     arg->frame_size,
                     arg->max_offset,
                     arg->nof_symbols,
                     arg->cp_len);

  return buf;
}


/* see lib/include/srslte/phy/ue/ue_mib.h
typedef struct SRSLTE_API {
  srslte_sync_t sfind;
  cf_t *sf_symbols;
  cf_t *ce[SRSLTE_MAX_PORTS];
  srslte_ofdm_t fft;
  srslte_chest_dl_t chest; 
  srslte_pbch_t pbch;
  uint8_t bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  uint32_t nof_tx_ports; 
  uint32_t sfn_offset; 
  uint32_t frame_cnt; 
} srslte_ue_mib_t; */
inline std::string ue_mib_t_to_string(const srslte_ue_mib_t * arg)
{
  char buf[2048] = {0};

  int len = snprintf(buf, sizeof(buf), "ue_mib_t: n_txports %u, sfn_offset %u, fr_cnt %u"
                                       "\n\t\t  sfind:%s"
                                       "\n\t\t  pbch: %s"
                                       "\n\t\t  chest->cell:%s",
                     arg->nof_tx_ports,
                     arg->sfn_offset,
                     arg->frame_cnt,
                     sync_t_to_string(&arg->sfind).c_str(),
                     pbch_t_to_string(&arg->pbch).c_str(),
                     cell_t_to_string(&arg->chest.cell).c_str());


  return buf;
}

/* see lib/include/srslte/phy/ue/ue_sync.h
typedef struct SRSLTE_API {
  srslte_sync_t sfind;
  srslte_sync_t strack;
  uint32_t max_prb;
  srslte_agc_t agc; 
  bool do_agc; 
  uint32_t agc_period; 
  int decimate;
  void *stream; 
  void *stream_single; 
  int (*recv_callback)(void*, cf_t*[SRSLTE_MAX_PORTS], uint32_t, srslte_timestamp_t*);
  int (*recv_callback_single)(void*, void*, uint32_t, srslte_timestamp_t*); 
  srslte_timestamp_t last_timestamp;
  uint32_t nof_rx_antennas; 
  srslte_filesource_t file_source; 
  bool file_mode; 
  float file_cfo;
  bool file_wrap_enable;
  srslte_cfo_t file_cfo_correct; 
  srslte_ue_sync_state_t state;
  uint32_t frame_len; 
  uint32_t fft_size;
  uint32_t nof_recv_sf;  // Number of subframes received each call to srslte_ue_sync_get_buffer
  uint32_t nof_avg_find_frames;
  uint32_t frame_find_cnt;
  uint32_t sf_len;
  // These count half frames (5ms)
  uint64_t frame_ok_cnt;
  uint32_t frame_no_cnt; 
  uint32_t frame_total_cnt; 
  // this is the system frame number (SFN) 
  uint32_t frame_number; JG IS THIS USED ???
  srslte_cell_t cell; 
  uint32_t sf_idx;
  bool decode_sss_on_track; 
  bool  cfo_is_copied;
  bool  cfo_correct_enable_track;
  bool  cfo_correct_enable_find;
  float cfo_current_value;
  float cfo_loop_bw_pss;
  float cfo_loop_bw_ref;
  float cfo_pss_min;
  float cfo_ref_min;
  float cfo_ref_max;
  uint32_t pss_stable_cnt;
  uint32_t pss_stable_timeout;
  bool     pss_is_stable;
  uint32_t peak_idx;
  int next_rf_sample_offset;
  int last_sample_offset; 
  float mean_sample_offset; 
  float mean_sfo; 
  uint32_t sample_offset_correct_period; 
  float sfo_ema; 
  #ifdef MEASURE_EXEC_TIME
  float mean_exec_time;
  #endif
} srslte_ue_sync_t; */
inline std::string ue_sync_t_to_string(const srslte_ue_sync_t * arg)
{
  char buf[4096] = {0};

  int len = snprintf(buf, sizeof(buf), "ue_sync_t: state %s, n_rxports %u, fr_len %u, n_rxsf %u, n_avg_findfr %u, sf_len %u, sf_idx %u, decimate %d"
                                       "\n\t\t\t dec_sss_on_track %s, fr_find_cnt %u, fr_ok_cnt %lu, fr_no_cnt %u, fr_tot_cnt %u, last_ts %06ld:%0.6lf"
                                       "\n\t\t\t pss_stab_cnt %u, pss_stab_timeout %u, pss_is_stable %s, peak_idx %u, next_samp_offset %u, last_samp_offset %u"
                                       "\n\t\t\t curr_cfo %f, cfo_en_track %s, cfo_en_find %s"
                                       "\n\t\t\t cell:  %s"
                                       "\n\t\t\t sfind: %s"
                                       "\n\t\t\t strack:%s",
                     ue_sync_state_t_to_name(arg->state).c_str(),
                     arg->nof_rx_antennas,
                     arg->frame_len, 
                     arg->nof_recv_sf,
                     arg->nof_avg_find_frames,
                     arg->sf_len,
                     arg->sf_idx,
                     arg->decimate,
                     YES_NO(arg->decode_sss_on_track),
                     arg->frame_find_cnt,
                     arg->frame_ok_cnt,
                     arg->frame_no_cnt,
                     arg->frame_total_cnt,
                     arg->last_timestamp.full_secs,
                     arg->last_timestamp.frac_secs,
                     arg->pss_stable_cnt,
                     arg->pss_stable_timeout,
                     YES_NO(arg->pss_is_stable),
                     arg->peak_idx,
                     arg->next_rf_sample_offset,
                     arg->last_sample_offset,
                     arg->cfo_current_value,
                     YES_NO(arg->cfo_correct_enable_track),
                     YES_NO(arg->cfo_correct_enable_find),
                     cell_t_to_string(&arg->cell).c_str(),
                     sync_t_to_string(&arg->sfind).c_str(),
                     sync_t_to_string(&arg->strack).c_str());
  return buf;
}


/* see lib/include/srslte/phy/ue/ue_mib.h
 typedef struct {
  srslte_ue_mib_t ue_mib; 
  srslte_ue_sync_t ue_sync; 
  cf_t *sf_buffer[SRSLTE_MAX_PORTS];
  uint32_t nof_rx_antennas;
} srslte_ue_mib_sync_t; */
inline std::string ue_mib_sync_t_to_string(const srslte_ue_mib_sync_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "ue_mib_sync_t: n_rxports %u"
                                       "\n\t\tue_mib: %s"
                                       "\n\t\tue_sync:%s",
                     arg->nof_rx_antennas,
                     ue_mib_t_to_string(&arg->ue_mib).c_str(),
                     ue_sync_t_to_string(&arg->ue_sync).c_str());

  return buf;
}


/* see lib/include/srslte/phy/ue/ue_cell_search.h
typedef struct SRSLTE_API {
  uint32_t cell_id;
  srslte_cp_t cp; 
  float peak; 
  float mode; 
  float psr;
  float cfo; 
} srslte_ue_cellsearch_result_t; */
inline std::string ue_cellsearch_result_t_to_string(const srslte_ue_cellsearch_result_t * arg)
{
  char buf[256] = {0};

  int len = snprintf(buf, sizeof(buf), "ue_cs_result_t: cell_id %u, cp_type %s, peak %f, mode %f, psr %f, cfo %f",
                     arg->cell_id,
                     cp_t_to_name(arg->cp).c_str(),
                     arg->peak,
                     arg->mode,
                     arg->psr,
                     arg->cfo);
    
  return buf;
}


/* see lib/include/srslte/phy/ue/ue_cell_search.h
typedef struct SRSLTE_API {
  srslte_ue_sync_t ue_sync;
  cf_t *sf_buffer[SRSLTE_MAX_PORTS];
  uint32_t nof_rx_antennas; 
  uint32_t max_frames;
  uint32_t nof_valid_frames;  // number of 5 ms frames to scan 
  uint32_t *mode_ntimes;
  uint8_t *mode_counted; 
  srslte_ue_cellsearch_result_t *candidates; 
} srslte_ue_cellsearch_t; */
inline std::string ue_cellsearch_t_to_string(const srslte_ue_cellsearch_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "\n\t\tue_cs_t: n_rxports %u, max_fr %u, n_valid_fr %u"
                                       "\n\t\t ue_sync: %s",
                     arg->nof_rx_antennas,
                     arg->max_frames,
                     arg->nof_valid_frames,
                     ue_sync_t_to_string(&arg->ue_sync).c_str());

#if 0
  len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t candidates");

  for(uint32_t i = 0; i < arg->max_frames; ++i)
    {
       len += snprintf(buf + len, sizeof(buf) - len, "\n\t\t [%u] nt %u, nc %u, %s", 
                       i,
                       arg->mode_ntimes[i],
                       arg->mode_counted[i],
                       ue_cellsearch_result_t_to_string(&arg->candidates[i]).c_str());
    }
#endif
    
  return buf;
}

/* see lib/include/srslte/phy/ue/ue_ul.h
typedef struct SRSLTE_API {
  srslte_ofdm_t fft;
  srslte_cfo_t cfo; 
  srslte_cell_t cell;
  bool normalize_en; 
  bool cfo_en; 
  float current_cfo_tol;
  float current_cfo; 
  srslte_pucch_format_t last_pucch_format;
  srslte_pusch_cfg_t pusch_cfg; 
  srslte_refsignal_ul_t signals; 
  srslte_refsignal_ul_dmrs_pregen_t pregen_drms;
  srslte_refsignal_srs_pregen_t pregen_srs;
  srslte_softbuffer_tx_t softbuffer;
  srslte_pusch_t pusch; 
  srslte_pucch_t pucch; 
  srslte_pucch_sched_t              pucch_sched; 
  srslte_refsignal_srs_cfg_t        srs_cfg;
  srslte_uci_cfg_t                  uci_cfg;
  srslte_pusch_hopping_cfg_t        hopping_cfg;
  srslte_ue_ul_powerctrl_t          power_ctrl;
  cf_t *refsignal; 
  cf_t *srs_signal; 
  cf_t *sf_symbols;
  float last_amplitude;
  uint16_t current_rnti;  
  bool signals_pregenerated;
}srslte_ue_ul_t; */
inline std::string ue_ul_t_to_string(const srslte_ue_ul_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "\n\t\t\tue_ul_t: cur_rnti 0x%x, last_amplitude %f"
                                       "\n\t\t\t cell   %s"
                                       "\n\t\t\t pusch_cfg:%s"
                                       "\n\t\t\t pusch: %s"
                                       "\n\t\t\t pucch: %s\n",
                     arg->current_rnti,
                     arg->last_amplitude,
                     cell_t_to_string(&arg->cell).c_str(),
                     pusch_cfg_t_to_string(&arg->pusch_cfg).c_str(),
                     pusch_t_to_string(&arg->pusch).c_str(),
                     pucch_t_to_string(&arg->pucch).c_str());

  // XXX TODO power and more
  
  return buf;
}




/* see lib/include/srslte/phy/ue/ue_dl.h
typedef struct SRSLTE_API {
  srslte_pcfich_t pcfich;
  srslte_pdcch_t pdcch;
  srslte_pdsch_t pdsch;
  srslte_pmch_t  pmch;
  srslte_phich_t phich; 
  srslte_regs_t regs;
  srslte_ofdm_t fft[SRSLTE_MAX_PORTS];
  srslte_ofdm_t fft_mbsfn;
  srslte_chest_dl_t chest;
  srslte_pdsch_cfg_t pdsch_cfg;
  srslte_pdsch_cfg_t pmch_cfg;
  srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_CODEWORDS];
  srslte_ra_dl_dci_t dl_dci;
  srslte_cell_t cell;
  uint32_t nof_rx_antennas;
  cf_t *sf_symbols;  // this is for backwards compatibility
  cf_t *sf_symbols_m[SRSLTE_MAX_PORTS]; 
  cf_t *ce[SRSLTE_MAX_PORTS]; // compatibility
  cf_t *ce_m[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  // RI, PMI and SINR for MIMO statistics
  float sinr[SRSLTE_MAX_LAYERS][SRSLTE_MAX_CODEBOOKS];
  uint32_t pmi[SRSLTE_MAX_LAYERS];
  uint32_t ri;
  // Power allocation parameter 3GPP 36.213 Clause 5.2 Rho_b 
  float rho_b;
  srslte_dci_format_t dci_format;
  uint64_t pkt_errors; 
  uint64_t pkts_total;
  uint64_t pdsch_pkt_errors;
  uint64_t pdsch_pkts_total;
  uint64_t pmch_pkt_errors;
  uint64_t pmch_pkts_total;
  uint64_t nof_detected; 
  uint16_t current_rnti;
  uint16_t current_mbsfn_area_id;
  dci_blind_search_t current_ss_ue[3][10];
  dci_blind_search_t current_ss_common[3];
  srslte_dci_location_t last_location;
  srslte_dci_location_t last_location_ul;
  srslte_dci_msg_t pending_ul_dci_msg; 
  uint16_t pending_ul_dci_rnti; 
  float last_phich_corr;
}srslte_ue_dl_t; */
inline std::string ue_dl_t_to_string(const srslte_ue_dl_t * arg)
{
  char buf[8192] = {0};

  int len = snprintf(buf, sizeof(buf), "\n\t\t\tue_dl_t: n_rxprts %u, cur_rnti 0x%x, pndg_ul_dci_rnti %hu, pkt(err %lu, tot %lu), last_phich_corr %f"
                                       "\n\t\t\t pdsch(err %lu, tot %lu), pmch(err %lu, tot %lu), n_dect %lu, mbsfn_area %hu"
                                       "\n\t\t\t last_dci_location %s"
                                       "\n\t\t\t last_dci_location_ul %s"
                                       "\n\t\t\t dl_fmt %s"
                                       "\n\t\t\t dl_dci %s"
                                       "\n\t\t\t cell   %s"
                                       "\n\t\t\t pdsch: %s"
                                       "\n\t\t\t pcfich:%s"
                                       "\n\t\t\t phich: %s"
                                       "\n\t\t\t pdcch: %s"
                                       "\n\t\t\t pmch:  %s"
                                       "\n\t\t\t pdsch_cfg:%s"
                                       "\n\t\t\t pmch_cfg:%s\n",
                     arg->nof_rx_antennas,
                     arg->current_rnti,
                     arg->pending_ul_dci_rnti,
                     arg->pkt_errors, 
                     arg->pkts_total,
                     arg->last_phich_corr,
                     arg->pdsch_pkt_errors,
                     arg->pdsch_pkts_total,
                     arg->pmch_pkt_errors,
                     arg->pmch_pkts_total,
                     arg->nof_detected,
                     arg->current_mbsfn_area_id,
                     dci_location_t_to_string(&arg->last_location).c_str(),
                     dci_location_t_to_string(&arg->last_location_ul).c_str(),
                     dci_format_t_to_name(arg->dci_format).c_str(),
                     ra_dl_dci_t_to_string(&arg->dl_dci).c_str(),
                     cell_t_to_string(&arg->cell).c_str(),
                     pdsch_t_to_string(&arg->pdsch).c_str(),
                     pcfich_t_to_string(&arg->pcfich).c_str(),
                     phich_t_to_string(&arg->phich).c_str(),
                     pdcch_t_to_string(&arg->pdcch).c_str(),
                     pmch_t_to_string(&arg->pmch).c_str(),
                     pdsch_cfg_t_to_string(&arg->pdsch_cfg).c_str(),
                     pdsch_cfg_t_to_string(&arg->pmch_cfg).c_str());

  return buf;
}

#endif 
