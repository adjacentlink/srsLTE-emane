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

#include <string>
#include <sstream>
#include <string.h>
#include <strings.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/mman.h>

#include "srslte/common/threads.h"
#include "srslte/common/log.h"
#include "srsenb/hdr/phy/phy.h"

#define Error(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->error(fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (SRSLTE_DEBUG_ENABLED) log_h->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (SRSLTE_DEBUG_ENABLED) log_h->info(fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (SRSLTE_DEBUG_ENABLED) log_h->debug(fmt, ##__VA_ARGS__)

using namespace std;
using namespace asn1::rrc;

namespace srsenb {

phy::phy() : workers_pool(MAX_WORKERS), 
             workers(MAX_WORKERS), 
             workers_common(MAX_WORKERS),
             nof_workers(0)
{
  radio_handler = NULL;
  bzero(&prach_cfg, sizeof(prach_cfg));
}

void phy::parse_config(phy_cfg_t* cfg)
{
  
  // PRACH configuration
  prach_cfg.config_idx     = cfg->prach_cnfg.prach_cfg_info.prach_cfg_idx;
  prach_cfg.hs_flag        = cfg->prach_cnfg.prach_cfg_info.high_speed_flag;
  prach_cfg.root_seq_idx   = cfg->prach_cnfg.root_seq_idx;
  prach_cfg.zero_corr_zone = cfg->prach_cnfg.prach_cfg_info.zero_correlation_zone_cfg;
  prach_cfg.freq_offset    = cfg->prach_cnfg.prach_cfg_info.prach_freq_offset;

  // PUSCH DMRS configuration
  workers_common.pusch_cfg.cyclic_shift        = cfg->pusch_cnfg.ul_ref_sigs_pusch.cyclic_shift;
  workers_common.pusch_cfg.delta_ss            = cfg->pusch_cnfg.ul_ref_sigs_pusch.group_assign_pusch;
  workers_common.pusch_cfg.group_hopping_en    = cfg->pusch_cnfg.ul_ref_sigs_pusch.group_hop_enabled;
  workers_common.pusch_cfg.sequence_hopping_en = cfg->pusch_cnfg.ul_ref_sigs_pusch.seq_hop_enabled;

  // PUSCH hopping configuration
  workers_common.hopping_cfg.hop_mode =
      cfg->pusch_cnfg.pusch_cfg_basic.hop_mode ==
              asn1::rrc::pusch_cfg_common_s::pusch_cfg_basic_s_::hop_mode_e_::intra_and_inter_sub_frame
          ? srslte_pusch_hopping_cfg_t::SRSLTE_PUSCH_HOP_MODE_INTRA_SF
          : srslte_pusch_hopping_cfg_t::SRSLTE_PUSCH_HOP_MODE_INTER_SF;
  ;
  workers_common.hopping_cfg.n_sb           = cfg->pusch_cnfg.pusch_cfg_basic.n_sb;
  workers_common.hopping_cfg.hopping_offset = cfg->pusch_cnfg.pusch_cfg_basic.pusch_hop_offset;

  // PUCCH configuration
  workers_common.pucch_cfg.delta_pucch_shift =
      cfg->pucch_cnfg.delta_pucch_shift.to_number(); // FIXME: Why was it a % operator before?
  workers_common.pucch_cfg.N_cs               = cfg->pucch_cnfg.n_cs_an;
  workers_common.pucch_cfg.n_rb_2             = cfg->pucch_cnfg.n_rb_cqi;
  workers_common.pucch_cfg.srs_configured     = false;
  workers_common.pucch_cfg.n1_pucch_an        = cfg->pucch_cnfg.n1_pucch_an;

  // PDSCH configuration
  workers_common.pdsch_p_b                    = cfg->pdsch_cnfg.p_b;
}

bool phy::init(phy_args_t *args, 
               phy_cfg_t *cfg, 
               srslte::radio* radio_handler_, 
               mac_interface_phy *mac,
               srslte::log_filter* log_h)
{

  std::vector<srslte::log_filter*> log_vec;
  this->log_h = log_h;
  for (int i=0;i<args->nof_phy_threads;i++) {
    log_vec.push_back(log_h);
  }
  init(args, cfg, radio_handler_, mac, log_vec);
  return true; 
}

bool phy::init(phy_args_t *args, 
               phy_cfg_t *cfg, 
               srslte::radio* radio_handler_, 
               mac_interface_phy *mac, 
               std::vector<srslte::log_filter*> log_vec)
{

  mlockall(MCL_CURRENT | MCL_FUTURE);
  
  radio_handler = radio_handler_;
  nof_workers = args->nof_phy_threads; 
  this->log_h = (srslte::log*)log_vec[0];
  workers_common.params = *args; 

  workers_common.init(&cfg->cell, radio_handler, mac);
  
  parse_config(cfg);
  
  // Add workers to workers pool and start threads
  for (uint32_t i=0;i<nof_workers;i++) {
    workers[i].init(&workers_common, (srslte::log*) log_vec[i]);
    workers_pool.init_worker(i, &workers[i], WORKERS_THREAD_PRIO);    
  }
  
  prach.init(&cfg->cell, &prach_cfg, mac, (srslte::log*) log_vec[0], PRACH_WORKER_THREAD_PRIO);
  prach.set_max_prach_offset_us(args->max_prach_offset_us);
  
  // Warning this must be initialized after all workers have been added to the pool
  tx_rx.init(radio_handler, &workers_pool, &workers_common, &prach, (srslte::log*) log_vec[0], SF_RECV_THREAD_PRIO);
    
  return true; 
}

void phy::stop()
{  
  tx_rx.stop();  
  for (uint32_t i=0;i<nof_workers;i++) {
    workers[i].stop();
  }
  workers_common.stop();
  workers_pool.stop();
  prach.stop();
}

uint32_t phy::tti_to_SFN(uint32_t tti) {
  return tti/10; 
}

uint32_t phy::tti_to_subf(uint32_t tti) {
  return tti%10; 
}

/***** MAC->PHY interface **********/
int phy::add_rnti(uint16_t rnti)
{
  if (rnti >= SRSLTE_CRNTI_START && rnti <= SRSLTE_CRNTI_END) {
    workers_common.ue_db_add_rnti(rnti);
  }
  for (uint32_t i=0;i<nof_workers;i++) {
    if (workers[i].add_rnti(rnti)) {
      return SRSLTE_ERROR; 
    }
  }
  return SRSLTE_SUCCESS;
}

void phy::rem_rnti(uint16_t rnti)
{
  if (rnti >= SRSLTE_CRNTI_START && rnti <= SRSLTE_CRNTI_END) {
    workers_common.ue_db_rem_rnti(rnti);
  }
  for (uint32_t i=0;i<nof_workers;i++) {
    workers[i].rem_rnti(rnti);
  }
}

void phy::get_metrics(phy_metrics_t metrics[ENB_METRICS_MAX_USERS])
{
  phy_metrics_t metrics_tmp[ENB_METRICS_MAX_USERS];

  uint32_t nof_users = workers[0].get_nof_rnti(); 
  bzero(metrics, sizeof(phy_metrics_t)*ENB_METRICS_MAX_USERS);
  int n_tot = 0; 
  for (uint32_t i=0;i<nof_workers;i++) {
    workers[i].get_metrics(metrics_tmp);
    for (uint32_t j=0;j<nof_users;j++) {
      metrics[j].dl.n_samples   += metrics_tmp[j].dl.n_samples; 
      metrics[j].dl.mcs         += metrics_tmp[j].dl.n_samples*metrics_tmp[j].dl.mcs;
      
      metrics[j].ul.n_samples   += metrics_tmp[j].ul.n_samples; 
      metrics[j].ul.mcs         += metrics_tmp[j].ul.n_samples*metrics_tmp[j].ul.mcs;
      metrics[j].ul.n           += metrics_tmp[j].ul.n_samples*metrics_tmp[j].ul.n;
      metrics[j].ul.rssi        += metrics_tmp[j].ul.n_samples*metrics_tmp[j].ul.rssi;
      metrics[j].ul.sinr        += metrics_tmp[j].ul.n_samples*metrics_tmp[j].ul.sinr;
      metrics[j].ul.turbo_iters += metrics_tmp[j].ul.n_samples*metrics_tmp[j].ul.turbo_iters;
    }
  }
  for (uint32_t j=0;j<nof_users;j++) {
    metrics[j].dl.mcs         /= metrics[j].dl.n_samples;
    metrics[j].ul.mcs         /= metrics[j].ul.n_samples;
    metrics[j].ul.n           /= metrics[j].ul.n_samples;
    metrics[j].ul.rssi        /= metrics[j].ul.n_samples;
    metrics[j].ul.sinr        /= metrics[j].ul.n_samples;
    metrics[j].ul.turbo_iters /= metrics[j].ul.n_samples;
  }
}


/***** RRC->PHY interface **********/

void phy::set_conf_dedicated_ack(uint16_t rnti, bool ack)
{
  for (uint32_t i = 0; i < nof_workers; i++) {
    workers[i].set_conf_dedicated_ack(rnti, ack);
  }
}

void phy::set_config_dedicated(uint16_t rnti, phys_cfg_ded_s* dedicated)
{
  for (uint32_t i=0;i<nof_workers;i++) {
    workers[i].set_config_dedicated(rnti, NULL, dedicated);
  }
}

void phy::configure_mbsfn(sib_type2_s* sib2, sib_type13_r9_s* sib13, mcch_msg_s mcch)
{
  if (sib2->mbsfn_sf_cfg_list_present) {
    if (sib2->mbsfn_sf_cfg_list.size() == 0) {
      Warning("SIB2 does not have any MBSFN config although it was set as present\n");
    } else {
      if (sib2->mbsfn_sf_cfg_list.size() > 1) {
        Warning("SIB2 has %d MBSFN subframe configs - only 1 supported\n", sib2->mbsfn_sf_cfg_list.size());
      }
      phy_rrc_config.mbsfn.mbsfn_subfr_cnfg = sib2->mbsfn_sf_cfg_list[0];
    }
  }

  phy_rrc_config.mbsfn.mbsfn_notification_cnfg = sib13->notif_cfg_r9;
  if (sib13->mbsfn_area_info_list_r9.size() > 0) {
    if (sib13->mbsfn_area_info_list_r9.size() > 1) {
      Warning("SIB13 has %d MBSFN area info elements - only 1 supported\n", sib13->mbsfn_area_info_list_r9.size());
    }
    phy_rrc_config.mbsfn.mbsfn_area_info = sib13->mbsfn_area_info_list_r9[0];
  }

  phy_rrc_config.mbsfn.mcch = mcch;

  workers_common.configure_mbsfn(&phy_rrc_config.mbsfn);
}

// Start GUI 
void phy::start_plot() {
  ((phch_worker) workers[0]).start_plot();
}

}
