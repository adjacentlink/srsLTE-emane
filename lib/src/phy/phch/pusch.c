/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include <math.h>
#include <srslte/srslte.h>
#include <srslte/phy/phch/pusch.h>

#include "srslte/phy/ch_estimation/refsignal_ul.h"
#include "srslte/phy/phch/pusch.h"
#include "srslte/phy/phch/pusch_cfg.h"
#include "srslte/phy/phch/uci.h"
#include "srslte/phy/common/phy_common.h"
#include "srslte/phy/utils/bit.h"
#include "srslte/phy/utils/debug.h"
#include "srslte/phy/utils/vector.h"
#include "srslte/phy/dft/dft_precoding.h"

#define MAX_PUSCH_RE(cp) (2 * SRSLTE_CP_NSYMB(cp) * 12)



const static srslte_mod_t modulations[4] =
    { SRSLTE_MOD_BPSK, SRSLTE_MOD_QPSK, SRSLTE_MOD_16QAM, SRSLTE_MOD_64QAM };
    
static int f_hop_sum(srslte_pusch_t *q, uint32_t i) {
  uint32_t sum = 0;
  for (uint32_t k=i*10+1;k<i*10+9;i++) {
    sum += (q->seq_type2_fo.c[k]<<(k-(i*10+1)));
  }
  return sum; 
}
    
static int f_hop(srslte_pusch_t *q, srslte_pusch_hopping_cfg_t *hopping, int i) {
  if (i == -1) {
    return 0; 
  } else {
    if (hopping->n_sb == 1) {
      return 0;
    } else if (hopping->n_sb == 2) {
      return (f_hop(q, hopping, i-1) + f_hop_sum(q, i))%2;
    } else {
      return (f_hop(q, hopping, i-1) + f_hop_sum(q, i)%(hopping->n_sb-1)+1)%hopping->n_sb;   
    }    
  }
}

static int f_m(srslte_pusch_t *q, srslte_pusch_hopping_cfg_t *hopping, uint32_t i, uint32_t current_tx_nb) {
  if (hopping->n_sb == 1) {
    if (hopping->hop_mode == SRSLTE_PUSCH_HOP_MODE_INTER_SF) {
      return current_tx_nb%2;
    } else {
      return i%2;      
    }
  } else {
    return q->seq_type2_fo.c[i*10];
  }
}

/* Computes PUSCH frequency hopping as defined in Section 8.4 of 36.213 */
void compute_freq_hopping(srslte_pusch_t *q, srslte_ra_ul_grant_t *grant, 
                               srslte_pusch_hopping_cfg_t *hopping, 
                               uint32_t sf_idx, uint32_t current_tx_nb) 
{
  
  for (uint32_t slot=0;slot<2;slot++) {    

    INFO("PUSCH Freq hopping: %d\n", grant->freq_hopping);
    uint32_t n_prb_tilde = grant->n_prb[slot]; 
    
    if (grant->freq_hopping == 1) {
      if (hopping->hop_mode == SRSLTE_PUSCH_HOP_MODE_INTER_SF) {
        n_prb_tilde = grant->n_prb[current_tx_nb%2];      
      } else {
        n_prb_tilde = grant->n_prb[slot];
      }
    }
    if (grant->freq_hopping == 2) {
      /* Freq hopping type 2 as defined in 5.3.4 of 36.211 */
      uint32_t n_vrb_tilde = grant->n_prb[0];
      if (hopping->n_sb > 1) {
        n_vrb_tilde -= (hopping->hopping_offset-1)/2+1;
      }
      int i=0;
      if (hopping->hop_mode == SRSLTE_PUSCH_HOP_MODE_INTER_SF) {
        i = sf_idx;
      } else {
        i = 2*sf_idx+slot;
      }
      uint32_t n_rb_sb = q->cell.nof_prb;
      if (hopping->n_sb > 1) {
        n_rb_sb = (n_rb_sb-hopping->hopping_offset-hopping->hopping_offset%2)/hopping->n_sb;
      }
      n_prb_tilde = (n_vrb_tilde+f_hop(q, hopping, i)*n_rb_sb+
        (n_rb_sb-1)-2*(n_vrb_tilde%n_rb_sb)*f_m(q, hopping, i, current_tx_nb))%(n_rb_sb*hopping->n_sb);
      
      INFO("n_prb_tilde: %d, n_vrb_tilde: %d, n_rb_sb: %d, n_sb: %d\n", 
           n_prb_tilde, n_vrb_tilde, n_rb_sb, hopping->n_sb);
      if (hopping->n_sb > 1) {
        n_prb_tilde += (hopping->hopping_offset-1)/2+1;
      }
      
    }
    grant->n_prb_tilde[slot] = n_prb_tilde; 
  }
}


/* Allocate/deallocate PUSCH RBs to the resource grid
 */
int pusch_cp(srslte_pusch_t *q, srslte_ra_ul_grant_t *grant, cf_t *input, cf_t *output, bool advance_input) 
{
  cf_t *in_ptr = input; 
  cf_t *out_ptr = output; 
  
  uint32_t L_ref = 3;
  if (SRSLTE_CP_ISEXT(q->cell.cp)) {
    L_ref = 2; 
  }
  for (uint32_t slot=0;slot<2;slot++) {        
    uint32_t N_srs = 0; 
    if (q->shortened && slot == 1) {
      N_srs = 1; 
    }
    INFO("%s PUSCH %d PRB to index %d at slot %d\n",advance_input?"Allocating":"Getting",grant->L_prb, grant->n_prb_tilde[slot], slot);
    for (uint32_t l=0;l<SRSLTE_CP_NSYMB(q->cell.cp)-N_srs;l++) {
      if (l != L_ref) {
        uint32_t idx = SRSLTE_RE_IDX(q->cell.nof_prb, l+slot*SRSLTE_CP_NSYMB(q->cell.cp), 
                              grant->n_prb_tilde[slot]*SRSLTE_NRE);
        if (advance_input) {
          out_ptr = &output[idx]; 
        } else {
          in_ptr = &input[idx];
        }              
        memcpy(out_ptr, in_ptr, grant->L_prb * SRSLTE_NRE * sizeof(cf_t));                       
        if (advance_input) {
          in_ptr += grant->L_prb*SRSLTE_NRE;
        } else {
          out_ptr += grant->L_prb*SRSLTE_NRE; 
        }
      }
    }        
  }
  if (advance_input) {
    return in_ptr - input;
  } else {
    return out_ptr - output; 
  }
}

int pusch_put(srslte_pusch_t *q, srslte_ra_ul_grant_t *grant, cf_t *input, cf_t *output) {
  return pusch_cp(q, grant, input, output, true);
}

int pusch_get(srslte_pusch_t *q, srslte_ra_ul_grant_t *grant, cf_t *input, cf_t *output) {
  return pusch_cp(q, grant, input, output, false);
}


/** Initializes the PDCCH transmitter and receiver */
int pusch_init(srslte_pusch_t *q, uint32_t max_prb, bool is_ue) {
  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  int i;

 if (q != NULL)
  {   
    
    bzero(q, sizeof(srslte_pusch_t));
    ret = SRSLTE_ERROR;
    q->max_re = max_prb * MAX_PUSCH_RE(SRSLTE_CP_NORM);

    INFO("Init PUSCH: %d PRBs\n", max_prb);

    for (i = 0; i < 4; i++) {
      if (srslte_modem_table_lte(&q->mod[i], modulations[i])) {
        goto clean;
      }
      srslte_modem_table_bytes(&q->mod[i]);
    }

    q->is_ue = is_ue;

    q->users = calloc(sizeof(srslte_pusch_user_t*), q->is_ue?1:(1+SRSLTE_SIRNTI));
    if (!q->users) {
      perror("malloc");
      goto clean;
    }

    if (srslte_sequence_init(&q->tmp_seq, q->max_re * srslte_mod_bits_x_symbol(SRSLTE_MOD_64QAM))) {
      goto clean;
    }

    srslte_sch_init(&q->ul_sch);

    if (srslte_dft_precoding_init(&q->dft_precoding, max_prb, is_ue)) {
      fprintf(stderr, "Error initiating DFT transform precoding\n");
      goto clean; 
    }

    // Allocate int16 for reception (LLRs). Buffer casted to uint8_t for transmission
    q->q = srslte_vec_malloc(sizeof(int16_t) * q->max_re * srslte_mod_bits_x_symbol(SRSLTE_MOD_64QAM));
    if (!q->q) {
      goto clean;
    }

    // Allocate int16 for reception (LLRs). Buffer casted to uint8_t for transmission
    q->g = srslte_vec_malloc(sizeof(int16_t) * q->max_re * srslte_mod_bits_x_symbol(SRSLTE_MOD_64QAM));
    if (!q->g) {
      goto clean;
    }
    q->d = srslte_vec_malloc(sizeof(cf_t) * q->max_re);
    if (!q->d) {
      goto clean; 
    }

    if (!q->is_ue) {
      q->ce = srslte_vec_malloc(sizeof(cf_t) * q->max_re);
      if (!q->ce) {
        goto clean;
      }
    }
    q->z = srslte_vec_malloc(sizeof(cf_t) * q->max_re);
    if (!q->z) {
      goto clean;
    }

    ret = SRSLTE_SUCCESS;
  }
  clean: 
  if (ret == SRSLTE_ERROR) {
    srslte_pusch_free(q);
  }
  return ret;
}

int srslte_pusch_init_ue(srslte_pusch_t *q, uint32_t max_prb) {
  return pusch_init(q, max_prb, true);
}

int srslte_pusch_init_enb(srslte_pusch_t *q, uint32_t max_prb) {
  return pusch_init(q, max_prb, false);
}

void srslte_pusch_free(srslte_pusch_t *q) {
  int i;

  if (q->q) {
    free(q->q);
  }
  if (q->d) {
    free(q->d);
  }
  if (q->g) {
    free(q->g);
  }
  if (q->ce) {
    free(q->ce);
  }
  if (q->z) {
    free(q->z);
  }
  
  srslte_dft_precoding_free(&q->dft_precoding);

  if (q->users) {
    if (q->is_ue) {
      srslte_pusch_free_rnti(q, 0);
    } else {
      for (int rnti=0;rnti<=SRSLTE_SIRNTI;rnti++) {
        srslte_pusch_free_rnti(q, rnti);
      }
    }
    free(q->users);
  }

  srslte_sequence_free(&q->seq_type2_fo);
  
  srslte_sequence_free(&q->tmp_seq);
  
  for (i = 0; i < 4; i++) {
    srslte_modem_table_free(&q->mod[i]);
  }
  srslte_sch_free(&q->ul_sch);

  bzero(q, sizeof(srslte_pusch_t));

}

int srslte_pusch_set_cell(srslte_pusch_t *q, srslte_cell_t cell) {
  int ret = SRSLTE_ERROR_INVALID_INPUTS;

  if (q != NULL && srslte_cell_isvalid(&cell))
  {

    q->max_re = cell.nof_prb * MAX_PUSCH_RE(q->cell.cp);

    INFO("PUSCH: Cell config PCI=%d, %d ports %d PRBs, max_symbols: %d\n",
         q->cell.id, q->cell.nof_ports, q->cell.nof_prb, q->max_re);

    if (q->cell.id != cell.id || q->cell.nof_prb == 0) {
      memcpy(&q->cell, &cell, sizeof(srslte_cell_t));
      /* Precompute sequence for type2 frequency hopping */
      if (srslte_sequence_LTE_pr(&q->seq_type2_fo, 210, q->cell.id)) {
        fprintf(stderr, "Error initiating type2 frequency hopping sequence\n");
        return SRSLTE_ERROR;
      }

    }
    ret = SRSLTE_SUCCESS;
  }
  return ret;
}



/* Configures the structure srslte_pusch_cfg_t from the UL DCI allocation dci_msg. 
 * If dci_msg is NULL, the grant is assumed to be already stored in cfg->grant
 */
int srslte_pusch_cfg(srslte_pusch_t             *q, 
                     srslte_pusch_cfg_t         *cfg, 
                     srslte_ra_ul_grant_t       *grant, 
                     srslte_uci_cfg_t           *uci_cfg, 
                     srslte_pusch_hopping_cfg_t *hopping_cfg, 
                     srslte_refsignal_srs_cfg_t *srs_cfg, 
                     uint32_t tti, 
                     uint32_t rv_idx, 
                     uint32_t current_tx_nb) 
{
  if (q && cfg && grant) {
    memcpy(&cfg->grant, grant, sizeof(srslte_ra_ul_grant_t));
    
    if (srslte_cbsegm(&cfg->cb_segm, cfg->grant.mcs.tbs)) {
      fprintf(stderr, "Error computing Codeblock segmentation for TBS=%d\n", cfg->grant.mcs.tbs);
      return SRSLTE_ERROR; 
    }
    
    /* Compute PUSCH frequency hopping */
    if (hopping_cfg) {
      compute_freq_hopping(q, &cfg->grant, hopping_cfg, tti%10, current_tx_nb);
    } else {
      cfg->grant.n_prb_tilde[0] = cfg->grant.n_prb[0];
      cfg->grant.n_prb_tilde[1] = cfg->grant.n_prb[1];
    }
    if (srs_cfg) {
      q->shortened = false; 
      if (srs_cfg->configured) {
        // If UE-specific SRS is configured, PUSCH is shortened every time UE transmits SRS even if overlaping in the same RB or not
        if (srslte_refsignal_srs_send_cs(srs_cfg->subframe_config, tti%10) == 1 && 
            srslte_refsignal_srs_send_ue(srs_cfg->I_srs, tti) == 1)
        {
          q->shortened = true; 
          /* If RBs are contiguous, PUSCH is not shortened */
          uint32_t k0_srs = srslte_refsignal_srs_rb_start_cs(srs_cfg->bw_cfg, q->cell.nof_prb);
          uint32_t nrb_srs = srslte_refsignal_srs_rb_L_cs(srs_cfg->bw_cfg, q->cell.nof_prb);
          for (uint32_t ns=0;ns<2 && q->shortened;ns++) {
            if (cfg->grant.n_prb_tilde[ns] == k0_srs + nrb_srs ||         // If PUSCH is contiguous on the right-hand side of SRS
                cfg->grant.n_prb_tilde[ns] + cfg->grant.L_prb == k0_srs)  // If SRS is contiguous on the left-hand side of PUSCH
            {
              q->shortened = false; 
            }
          }
        }
        // If not coincides with UE transmission. PUSCH shall be shortened if cell-specific SRS transmission RB 
        //coincides with PUSCH allocated RB
        if (!q->shortened) {
          if (srslte_refsignal_srs_send_cs(srs_cfg->subframe_config, tti%10) == 1) {
            uint32_t k0_srs = srslte_refsignal_srs_rb_start_cs(srs_cfg->bw_cfg, q->cell.nof_prb);
            uint32_t nrb_srs = srslte_refsignal_srs_rb_L_cs(srs_cfg->bw_cfg, q->cell.nof_prb);
            for (uint32_t ns=0;ns<2 && !q->shortened;ns++) {
              if ((cfg->grant.n_prb_tilde[ns] >= k0_srs && cfg->grant.n_prb_tilde[ns] < k0_srs + nrb_srs) || 
                  (cfg->grant.n_prb_tilde[ns] + cfg->grant.L_prb >= k0_srs && 
                        cfg->grant.n_prb_tilde[ns] + cfg->grant.L_prb < k0_srs + nrb_srs) ||
                  (cfg->grant.n_prb_tilde[ns] <= k0_srs && cfg->grant.n_prb_tilde[ns] + cfg->grant.L_prb >= k0_srs + nrb_srs))
              {            
                q->shortened = true; 
              }
            }
          }
        }
      }    
    } 
    
    /* Compute final number of bits and RE */
    srslte_ra_ul_grant_to_nbits(&cfg->grant, q->cell.cp, q->shortened?1:0, &cfg->nbits);

    cfg->sf_idx = tti%10; 
    cfg->tti    = tti; 
    cfg->rv     = rv_idx; 
    cfg->cp     = q->cell.cp; 
    
    // Save UCI configuration 
    if (uci_cfg) {
      memcpy(&cfg->uci_cfg, uci_cfg, sizeof(srslte_uci_cfg_t));
    }
    
    return SRSLTE_SUCCESS;      
  } else {
    return SRSLTE_ERROR_INVALID_INPUTS;
  }
}

/* Precalculate the PUSCH scramble sequences for a given RNTI. This function takes a while 
 * to execute, so shall be called once the final C-RNTI has been allocated for the session.
 * For the connection procedure, use srslte_pusch_encode() functions */
int srslte_pusch_set_rnti(srslte_pusch_t *q, uint16_t rnti) {
  uint32_t i;

  uint32_t rnti_idx = q->is_ue?0:rnti;

  if (!q->users[rnti_idx] || q->is_ue) {
    if (!q->users[rnti_idx]) {
      q->users[rnti_idx] = calloc(1, sizeof(srslte_pusch_user_t));
      if (!q->users[rnti_idx]) {
        perror("calloc");
        return -1;
      }
    }
    q->users[rnti_idx]->sequence_generated = false;
    for (i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; i++) {
      if (srslte_sequence_pusch(&q->users[rnti_idx]->seq[i], rnti, 2 * i, q->cell.id,
                                q->max_re * srslte_mod_bits_x_symbol(SRSLTE_MOD_64QAM)))
      {
        fprintf(stderr, "Error initializing PUSCH scrambling sequence\n");
        srslte_pusch_free_rnti(q, rnti);
        return SRSLTE_ERROR;
      }
    }
    q->ue_rnti = rnti;
    q->users[rnti_idx]->cell_id = q->cell.id;
    q->users[rnti_idx]->sequence_generated = true;
  } else {
    fprintf(stderr, "Error generating PUSCH sequence: rnti=0x%x already generated\n", rnti);
  }
  return SRSLTE_SUCCESS;
}

void srslte_pusch_free_rnti(srslte_pusch_t *q, uint16_t rnti) {

  uint32_t rnti_idx = q->is_ue?0:rnti;

  if (q->users[rnti_idx]) {
    for (int i = 0; i < SRSLTE_NSUBFRAMES_X_FRAME; i++) {
      srslte_sequence_free(&q->users[rnti_idx]->seq[i]);
    }    
    free(q->users[rnti_idx]);
    q->users[rnti_idx] = NULL;
    q->ue_rnti = 0;
  }
}

static srslte_sequence_t *get_user_sequence(srslte_pusch_t *q, uint16_t rnti, uint32_t sf_idx, uint32_t len)
{
  uint32_t rnti_idx = q->is_ue?0:rnti;

  if (rnti >= SRSLTE_CRNTI_START && rnti < SRSLTE_CRNTI_END) {
    // The scrambling sequence is pregenerated for all RNTIs in the eNodeB but only for C-RNTI in the UE
    if (q->users[rnti_idx]                          &&
        q->users[rnti_idx]->sequence_generated      &&
        q->users[rnti_idx]->cell_id == q->cell.id   &&
        (!q->is_ue || q->ue_rnti == rnti))
    {
      return &q->users[rnti_idx]->seq[sf_idx];
    } else {
      if (srslte_sequence_pusch(&q->tmp_seq, rnti, 2 * sf_idx, q->cell.id, len)) {
        fprintf(stderr, "Error generating temporal scrambling sequence\n");
        return NULL;
      }
      return &q->tmp_seq;
    }
  } else {
    fprintf(stderr, "Invalid RNTI=0x%x\n", rnti);
    return NULL;
  }
}

/** Converts the PUSCH data bits to symbols mapped to the slot ready for transmission
 */
int srslte_pusch_encode(srslte_pusch_t *q, srslte_pusch_cfg_t *cfg, srslte_softbuffer_tx_t *softbuffer,
                        uint8_t *data, srslte_uci_data_t uci_data, uint16_t rnti, 
                        cf_t *sf_symbols) 
{
  int ret = SRSLTE_ERROR_INVALID_INPUTS; 
   
  if (q    != NULL &&
      cfg  != NULL)
  {

    if (cfg->nbits.nof_re > q->max_re) {
      fprintf(stderr, "Error too many RE per subframe (%d). PUSCH configured for %d RE (%d PRB)\n",
          cfg->nbits.nof_re, q->max_re, q->cell.nof_prb);
      return SRSLTE_ERROR_INVALID_INPUTS;
    }

    INFO("Encoding PUSCH SF: %d, Mod %s, RNTI: %d, TBS: %d, NofRE: %d, NofSymbols=%d, NofBitsE: %d, rv_idx: %d\n",
         cfg->sf_idx, srslte_mod_string(cfg->grant.mcs.mod), rnti, 
         cfg->grant.mcs.tbs, cfg->nbits.nof_re, cfg->nbits.nof_symb, cfg->nbits.nof_bits, cfg->rv);
    
    if (srslte_ulsch_uci_encode(&q->ul_sch, cfg, softbuffer, data, uci_data, q->g, q->q)) {
      fprintf(stderr, "Error encoding TB\n");
      return SRSLTE_ERROR;
    }

    // Generate scrambling sequence if not pre-generated
    srslte_sequence_t *seq = get_user_sequence(q, rnti, cfg->sf_idx, cfg->nbits.nof_bits);

    // Run scrambling
    if (!seq) {
      fprintf(stderr, "Error getting scrambling sequence\n");
      return SRSLTE_ERROR;
    }
    srslte_scrambling_bytes(seq, (uint8_t*) q->q, cfg->nbits.nof_bits);

    // Correct UCI placeholder/repetition bits
    uint8_t *d = q->q; 
    for (int i = 0; i < q->ul_sch.nof_ri_ack_bits; i++) {     
      if (q->ul_sch.ack_ri_bits[i].type == UCI_BIT_PLACEHOLDER) {
        d[q->ul_sch.ack_ri_bits[i].position/8] |= (1<<(7-q->ul_sch.ack_ri_bits[i].position%8)); 
      } else if (q->ul_sch.ack_ri_bits[i].type == UCI_BIT_REPETITION) {
        if (q->ul_sch.ack_ri_bits[i].position > 1) {
          uint32_t p=q->ul_sch.ack_ri_bits[i].position;
          uint8_t bit = d[(p-1)/8] & (1<<(7-(p-1)%8)); 
          if (bit) {
            d[p/8] |= 1<<(7-p%8);
          } else {
            d[p/8] &= ~(1<<(7-p%8));
          }
        } 
      }
    }
    
    // Bit mapping
    srslte_mod_modulate_bytes(&q->mod[cfg->grant.mcs.mod], (uint8_t*) q->q, q->d, cfg->nbits.nof_bits);

    // DFT precoding
    srslte_dft_precoding(&q->dft_precoding, q->d, q->z, cfg->grant.L_prb, cfg->nbits.nof_symb);
    
    // Mapping to resource elements
    pusch_put(q, &cfg->grant, q->z, sf_symbols);
    
    ret = SRSLTE_SUCCESS;
  } 
  return ret; 
}


/** Decodes the PUSCH from the received symbols
 */
int srslte_pusch_decode(srslte_pusch_t *q, 
                        srslte_pusch_cfg_t *cfg, srslte_softbuffer_rx_t *softbuffer, 
                        cf_t *sf_symbols, 
                        cf_t *ce, float noise_estimate, uint16_t rnti,                        
                        uint8_t *data, srslte_cqi_value_t *cqi_value, srslte_uci_data_t *uci_data)
{
  int ret = SRSLTE_ERROR_INVALID_INPUTS;
  uint32_t n;
  
  if (q           != NULL &&
      sf_symbols  != NULL &&
      data        != NULL &&
      cfg         != NULL)
  {
    
    INFO("Decoding PUSCH SF: %d, Mod %s, NofBits: %d, NofRE: %d, NofSymbols=%d, NofBitsE: %d, rv_idx: %d\n",
        cfg->sf_idx, srslte_mod_string(cfg->grant.mcs.mod), cfg->grant.mcs.tbs, 
          cfg->nbits.nof_re, cfg->nbits.nof_symb, cfg->nbits.nof_bits, cfg->rv);

    /* extract symbols */
    n = pusch_get(q, &cfg->grant, sf_symbols, q->d);
    if (n != cfg->nbits.nof_re) {
      fprintf(stderr, "Error expecting %d symbols but got %d\n", cfg->nbits.nof_re, n);
      return SRSLTE_ERROR;
    }
    
    /* extract channel estimates */
    n = pusch_get(q, &cfg->grant, ce, q->ce);
    if (n != cfg->nbits.nof_re) {
      fprintf(stderr, "Error expecting %d symbols but got %d\n", cfg->nbits.nof_re, n);
      return SRSLTE_ERROR;
    }

    // Equalization
    srslte_predecoding_single(q->d, q->ce, q->z, NULL, cfg->nbits.nof_re, 1.0f, noise_estimate);
    
    // DFT predecoding
    srslte_dft_precoding(&q->dft_precoding, q->z, q->d, cfg->grant.L_prb, cfg->nbits.nof_symb);
    
    // Soft demodulation
    if (q->llr_is_8bit) {
      srslte_demod_soft_demodulate_b(cfg->grant.mcs.mod, q->d, q->q, cfg->nbits.nof_re);
    } else {
      srslte_demod_soft_demodulate_s(cfg->grant.mcs.mod, q->d, q->q, cfg->nbits.nof_re);
    }

    // Generate scrambling sequence if not pre-generated
    srslte_sequence_t *seq = get_user_sequence(q, rnti, cfg->sf_idx, cfg->nbits.nof_bits);

    // Set CQI len assuming RI = 1 (3GPP 36.212 Clause 5.2.4.1. Uplink control information on PUSCH without UL-SCH data)
    if (cqi_value) {
      if (cqi_value->type == SRSLTE_CQI_TYPE_SUBBAND_HL && cqi_value->subband_hl.ri_present) {
        cqi_value->subband_hl.rank_is_not_one = false;
        uci_data->uci_ri_len = (q->cell.nof_ports == 4) ? 2 : 1;
      }
      uci_data->uci_cqi_len = (uint32_t) srslte_cqi_size(cqi_value);
    }

    // Decode RI/HARQ bits before descrambling
    if (srslte_ulsch_uci_decode_ri_ack(&q->ul_sch, cfg, softbuffer, q->q, seq->c, uci_data)) {
      fprintf(stderr, "Error decoding RI/HARQ bits\n");
      return SRSLTE_ERROR; 
    }

    // Set CQI len with corresponding RI
    if (cqi_value) {
      if (cqi_value->type == SRSLTE_CQI_TYPE_SUBBAND_HL) {
        cqi_value->subband_hl.rank_is_not_one = (uci_data->uci_ri != 0);
      }
      uci_data->uci_cqi_len = (uint32_t) srslte_cqi_size(cqi_value);
    }
    
    // Descrambling
    if (q->llr_is_8bit) {
      srslte_scrambling_sb_offset(seq, q->q, 0, cfg->nbits.nof_bits);
    } else {
      srslte_scrambling_s_offset(seq, q->q, 0, cfg->nbits.nof_bits);
    }

    // Decode
    ret = srslte_ulsch_uci_decode(&q->ul_sch, cfg, softbuffer, q->q, q->g, data, uci_data);

    // Unpack CQI value if available
    if (cqi_value) {
      srslte_cqi_value_unpack(uci_data->uci_cqi, cqi_value);
    }
  }

  return ret;
}

uint32_t srslte_pusch_last_noi(srslte_pusch_t *q) {
  return q->ul_sch.nof_iterations;
}


  
