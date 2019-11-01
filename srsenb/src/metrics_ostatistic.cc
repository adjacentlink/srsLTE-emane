/*
 * Copyright 2013-2019 Software Radio Systems Limited
 *
 * This file is part of srsLTE.
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

#include "libemanelte/enbstatisticmanager.h"
#include "srsenb/hdr/metrics_ostatistic.h"

namespace srsenb{

metrics_ostatistic::metrics_ostatistic():
  enb(NULL)
{ }

void metrics_ostatistic::set_handle(enb_metrics_interface *enb_)
{
  enb = enb_;
}

void metrics_ostatistic::set_metrics(enb_metrics_t &m, const uint32_t)
{
  const auto & stack = m.stack;
  const auto & rrc   = stack.rrc;
  const auto & rlc   = stack.rlc;
  const auto & mac   = stack.mac;
  const auto & s1ap  = stack.s1ap;

  ENBSTATS::MACMetrics    macMetrics;
  ENBSTATS::RLCMetrics    rlcMetrics;
  ENBSTATS::RLCMRBMetrics rlcMRBMetrics;

  ENBSTATS::setS1State(s1ap.status == S1AP_ATTACHING ? "ATTACHING" :
                       s1ap.status == S1AP_READY     ? "READY"     : "ERROR");

  for(uint16_t user = 0; user < ENB_METRICS_MAX_USERS; ++user)
   {
     // track ue's
     if(user < stack.rrc.n_ues)
      {
        const std::string state = rrc.ues[user].state == RRC_STATE_IDLE                            ? "IDLE" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_CON_SETUP_COMPLETE     ? "WAIT_SETUP_COMP" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_SECURITY_MODE_COMPLETE ? "WAIT_SECMD_COMP" :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_UE_CAP_INFO            ? "WAIT_CAP_INFO"   :
                                  rrc.ues[user].state == RRC_STATE_WAIT_FOR_CON_RECONF_COMPLETE    ? "WAIT_CON_RECONF" :
                                  rrc.ues[user].state == RRC_STATE_REGISTERED                      ? "REGISTERED"      :
                                  rrc.ues[user].state == RRC_STATE_RELEASE_REQUEST                 ? "RELEASE_REQUEST" : "ERROR";

        macMetrics.emplace_back(ENBSTATS::MACMetric(mac[user].rnti,
                                                    mac[user].tx_pkts,
                                                    mac[user].tx_errors,
                                                    mac[user].tx_brate,
                                                    mac[user].rx_pkts,
                                                    mac[user].rx_errors,
                                                    mac[user].rx_brate,
                                                    mac[user].ul_buffer,
                                                    mac[user].dl_buffer,
                                                    mac[user].dl_cqi,
                                                    mac[user].dl_ri,
                                                    mac[user].dl_pmi,
                                                    mac[user].phr,
                                                    state));
      }

     ENBSTATS::RLCBearerMetrics rlcBearerMetrics;

     // for each bearer
     for(uint16_t n = 0; n < SRSLTE_N_RADIO_BEARERS; ++n)
      {
        // use capacity to determine if lcid is active
        if(rlc[user].metrics[n].qmetrics.capacity > 0)
         {
           const ENBSTATS::RLCQueueMetric queueMetric((int)rlc[user].metrics[n].mode,
                                                      rlc[user].metrics[n].qmetrics.capacity,
                                                      rlc[user].metrics[n].qmetrics.currsize,
                                                      rlc[user].metrics[n].qmetrics.highwater,
                                                      rlc[user].metrics[n].qmetrics.num_cleared,
                                                      rlc[user].metrics[n].qmetrics.num_push,
                                                      rlc[user].metrics[n].qmetrics.num_push_fail,
                                                      rlc[user].metrics[n].qmetrics.num_pop,
                                                      rlc[user].metrics[n].qmetrics.num_pop_fail);

           rlcBearerMetrics.emplace_back(ENBSTATS::RLCBearerMetric(rlc[user].dl_tput_mbps[n], 
                                                                   rlc[user].ul_tput_mbps[n],
                                                                   queueMetric));
         }
      }

     if(! rlcBearerMetrics.empty())
      {
        // save entry on unique rnti
        rlcMetrics[rlc[user].rnti] = rlcBearerMetrics;
      }

     ENBSTATS::RLCMRBBearerMetrics rlcMRBBearerMetrics;

     // for each mrb bearer
     for(uint16_t n = 0; n < SRSLTE_N_MCH_LCIDS; ++n)
      {
        // use capacity to determine if lcid is active
        if(rlc[user].mrb_metrics[n].qmetrics.capacity > 0)
         {
           ENBSTATS::RLCQueueMetric queueMetric((int)rlc[user].mrb_metrics[n].mode,
                                                rlc[user].mrb_metrics[n].qmetrics.capacity,
                                                rlc[user].mrb_metrics[n].qmetrics.currsize,
                                                rlc[user].mrb_metrics[n].qmetrics.highwater,
                                                rlc[user].mrb_metrics[n].qmetrics.num_cleared,
                                                rlc[user].mrb_metrics[n].qmetrics.num_push,
                                                rlc[user].mrb_metrics[n].qmetrics.num_push_fail,
                                                rlc[user].mrb_metrics[n].qmetrics.num_pop,
                                                rlc[user].mrb_metrics[n].qmetrics.num_pop_fail);

           rlcMRBBearerMetrics.emplace_back(ENBSTATS::RLCMRBBearerMetric(rlc[user].dl_tput_mrb_mbps[n], 
                                                                         queueMetric));
         }
      }

     if(! rlcMRBBearerMetrics.empty())
      {
        // save entry on unique rnti
        rlcMRBMetrics[rlc[user].rnti] = rlcMRBBearerMetrics;
      }
   }

  ENBSTATS::setMACMetrics(macMetrics);
  ENBSTATS::setRLCMetrics(rlcMetrics);
  ENBSTATS::setRLCMetrics(rlcMRBMetrics);
}

} // end namespace srsenb
