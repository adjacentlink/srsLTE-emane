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

#include <stdio.h>

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

  ENBSTATS::setS1State(m.s1ap.status == S1AP_ATTACHING ? "ATTACHING" :
                       m.s1ap.status == S1AP_READY     ? "READY"     : "ERROR");

  ENBSTATS::MACMetrics macMetrics;
  ENBSTATS::RLCMetrics rlcMetrics;

  // track ue's
  for(uint16_t ue = 0; ue < m.rrc.n_ues; ++ue)
   {
     fprintf(stderr, "%s ue %d\n", __func__, ue);

     const std::string state = m.rrc.ues[ue].state == RRC_STATE_IDLE                            ? "IDLE" :
                               m.rrc.ues[ue].state == RRC_STATE_WAIT_FOR_CON_SETUP_COMPLETE     ? "WAIT_SETUP_COMP" :
                               m.rrc.ues[ue].state == RRC_STATE_WAIT_FOR_SECURITY_MODE_COMPLETE ? "WAIT_SECMD_COMP" :
                               m.rrc.ues[ue].state == RRC_STATE_WAIT_FOR_UE_CAP_INFO            ? "WAIT_CAP_INFO"   :
                               m.rrc.ues[ue].state == RRC_STATE_WAIT_FOR_CON_RECONF_COMPLETE    ? "WAIT_CON_RECONF" :
                               m.rrc.ues[ue].state == RRC_STATE_REGISTERED                      ? "REGISTERED"      :
                               m.rrc.ues[ue].state == RRC_STATE_RELEASE_REQUEST                 ? "RELEASE_REQUEST" : "ERROR";

     macMetrics.push_back(ENBSTATS::MACMetric(m.mac[ue].rnti,
                                              m.mac[ue].tx_pkts,
                                              m.mac[ue].tx_errors,
                                              m.mac[ue].tx_brate,
                                              m.mac[ue].rx_pkts,
                                              m.mac[ue].rx_errors,
                                              m.mac[ue].rx_brate,
                                              m.mac[ue].ul_buffer,
                                              m.mac[ue].dl_buffer,
                                              m.mac[ue].dl_cqi,
                                              m.mac[ue].dl_ri,
                                              m.mac[ue].dl_pmi,
                                              m.mac[ue].phr,
                                              state));

     ENBSTATS::RLCMetric rlcMetric;

     // for each bearer
     for(uint16_t bearer = 0; bearer < SRSLTE_N_RADIO_BEARERS; ++bearer)
      {
        // use capacity to determine if lcid is active
        if(m.rlc[ue].metrics[bearer].qmetrics.capacity)
         {
           const ENBSTATS::RLCQueueMetric queueMetric(m.rlc[ue].metrics[bearer].mode,
                                                      m.rlc[ue].metrics[bearer].qmetrics.capacity,
                                                      m.rlc[ue].metrics[bearer].qmetrics.currsize,
                                                      m.rlc[ue].metrics[bearer].qmetrics.highwater,
                                                      m.rlc[ue].metrics[bearer].qmetrics.num_cleared,
                                                      m.rlc[ue].metrics[bearer].qmetrics.num_push,
                                                      m.rlc[ue].metrics[bearer].qmetrics.num_push_fail,
                                                      m.rlc[ue].metrics[bearer].qmetrics.num_pop,
                                                      m.rlc[ue].metrics[bearer].qmetrics.num_pop_fail);

           rlcMetric.first.push_back(ENBSTATS::RLCBearerMetric(m.rlc[ue].dl_tput_mbps[bearer], 
                                                               m.rlc[ue].ul_tput_mbps[bearer],
                                                               queueMetric));
         }
      }

     // for each mrb bearer
     for(uint16_t bearer = 0; bearer < SRSLTE_N_MCH_LCIDS; ++bearer)
      {
        // use capacity to determine if lcid is active
        if(m.rlc[ue].mrb_metrics[bearer].qmetrics.capacity)
         {
           ENBSTATS::RLCQueueMetric queueMetric(m.rlc[ue].mrb_metrics[bearer].mode,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.capacity,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.currsize,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.highwater,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.num_cleared,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.num_push,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.num_push_fail,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.num_pop,
                                                m.rlc[ue].mrb_metrics[bearer].qmetrics.num_pop_fail);

           rlcMetric.second.push_back(ENBSTATS::RLCMRBBearerMetric(m.rlc[ue].dl_tput_mrb_mbps[bearer], 
                                                                   queueMetric));
         }
      }

     // save entry on unique rnti
     rlcMetrics[m.mac[ue].rnti] = rlcMetric;
   }

  ENBSTATS::setMACMetrics(macMetrics);
  ENBSTATS::setRLCMetrics(rlcMetrics);


}

} // end namespace srsenb
