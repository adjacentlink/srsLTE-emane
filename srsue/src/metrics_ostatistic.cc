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

#include "libemanelte/uestatisticmanager.h"
#include "srsue/hdr/metrics_ostatistic.h"

namespace srsue{

metrics_ostatistic::metrics_ostatistic():
  ue(NULL)
{ }

void metrics_ostatistic::set_ue_handle(ue_metrics_interface *ue_)
{
  ue = ue_;
}

void metrics_ostatistic::set_metrics(ue_metrics_t &m, const uint32_t)
{
   UESTATS::RLCQueueMetricsList rlcQueueMetrics;
   UESTATS::RLCQueueMetricsList rlcMrbQueueMetrics;

#warning "metrics to do"
#if 0
   for(size_t n = 0; n < SRSLTE_N_RADIO_BEARERS; ++n)
     {
       // use capacity to determine if lcid is active
       if(m.rlc.metrics[n].qmetrics.capacity)
        {
          rlcQueueMetrics.push_back(
             UESTATS::RLCQueueMetrics(m.rlc.metrics[n].mode,
                                      m.rlc.metrics[n].qmetrics.capacity,
                                      m.rlc.metrics[n].qmetrics.currsize,
                                      m.rlc.metrics[n].qmetrics.highwater,
                                      m.rlc.metrics[n].qmetrics.num_cleared,
                                      m.rlc.metrics[n].qmetrics.num_push,
                                      m.rlc.metrics[n].qmetrics.num_push_fail,
                                      m.rlc.metrics[n].qmetrics.num_pop,
                                      m.rlc.metrics[n].qmetrics.num_pop_fail));
         }
      }

    for(size_t n = 0; n < SRSLTE_N_MCH_LCIDS; ++n)
      {
        // use capacity to determine if lcid is active
        if(m.rlc.mrb_metrics[n].qmetrics.capacity)
         {
           rlcMrbQueueMetrics.push_back(
             UESTATS::RLCQueueMetrics(m.rlc.mrb_metrics[n].mode,
                                      m.rlc.mrb_metrics[n].qmetrics.capacity,
                                      m.rlc.mrb_metrics[n].qmetrics.currsize,
                                      m.rlc.mrb_metrics[n].qmetrics.highwater,
                                      m.rlc.mrb_metrics[n].qmetrics.num_cleared,
                                      m.rlc.mrb_metrics[n].qmetrics.num_push,
                                      m.rlc.mrb_metrics[n].qmetrics.num_push_fail,
                                      m.rlc.mrb_metrics[n].qmetrics.num_pop,
                                      m.rlc.mrb_metrics[n].qmetrics.num_pop_fail));
         }
      }

  UESTATS::setRLCMetrics(UESTATS::RLCMetrics(m.rlc.dl_tput_mbps, 
                                             m.rlc.ul_tput_mbps,
                                             rlcQueueMetrics,
                                             m.rlc.dl_tput_mrb_mbps,
                                             rlcMrbQueueMetrics));

  UESTATS::setGWMetrics(UESTATS::GWMetrics(m.gw.dl_tput_mbps, m.gw.ul_tput_mbps));

  for(int cc = 0; cc < 1; ++cc)
   {
     UESTATS::setMACMetrics(
      UESTATS::MACMetrics(m.mac[cc].tx_pkts,
                          m.mac[cc].tx_errors,
                          m.mac[cc].tx_brate,
                          m.mac[cc].rx_pkts,
                          m.mac[cc].rx_errors,
                          m.mac[cc].rx_brate,
                          m.mac[cc].ul_buffer,
                          m.mac[cc].dl_retx_avg,
                          m.mac[cc].ul_retx_avg));

      UESTATS::setPHYMetrics(
       UESTATS::PHYMetrics(m.phy.sync.ta_us,
                           m.phy.sync.cfo,      
                           m.phy.sync.sfo,        
                           m.phy.dl[cc].n,
                           m.phy.dl[cc].sinr,
                           m.phy.dl[cc].rsrp,
                           m.phy.dl[cc].rsrq,
                           m.phy.dl[cc].rssi,
                           m.phy.dl[cc].turbo_iters,
                           m.phy.dl[cc].mcs,
                           m.phy.dl[cc].pathloss,
                           0,   // was mabr_mbps, now unused
                           m.phy.ul[cc].mcs,
                           m.phy.ul[cc].power,
                           0)); // was mabr_mbps, now unused
    }
#endif
}

} // end namespace srsue
