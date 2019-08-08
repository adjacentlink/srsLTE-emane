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
  const auto & mac   = stack.mac;
  const auto & s1ap  = stack.s1ap;

  ENBSTATS::MACMetrics macMetrics;

  ENBSTATS::setS1State(s1ap.status == S1AP_ATTACHING ? "ATTACHING" :
                       s1ap.status == S1AP_READY     ? "READY"     : "ERROR");

  // track ue's
  for(uint16_t ue = 0; ue < stack.rrc.n_ues; ++ue)
   {
     const std::string state = rrc.ues[ue].state == RRC_STATE_IDLE                            ? "IDLE" :
                               rrc.ues[ue].state == RRC_STATE_WAIT_FOR_CON_SETUP_COMPLETE     ? "WAIT_SETUP_COMP" :
                               rrc.ues[ue].state == RRC_STATE_WAIT_FOR_SECURITY_MODE_COMPLETE ? "WAIT_SECMD_COMP" :
                               rrc.ues[ue].state == RRC_STATE_WAIT_FOR_UE_CAP_INFO            ? "WAIT_CAP_INFO"   :
                               rrc.ues[ue].state == RRC_STATE_WAIT_FOR_CON_RECONF_COMPLETE    ? "WAIT_CON_RECONF" :
                               rrc.ues[ue].state == RRC_STATE_REGISTERED                      ? "REGISTERED"      :
                               rrc.ues[ue].state == RRC_STATE_RELEASE_REQUEST                 ? "RELEASE_REQUEST" : "ERROR";

     macMetrics.push_back(ENBSTATS::MACMetric(mac[ue].rnti,
                                              mac[ue].tx_pkts,
                                              mac[ue].tx_errors,
                                              mac[ue].tx_brate,
                                              mac[ue].rx_pkts,
                                              mac[ue].rx_errors,
                                              mac[ue].rx_brate,
                                              mac[ue].ul_buffer,
                                              mac[ue].dl_buffer,
                                              mac[ue].dl_cqi,
                                              mac[ue].dl_ri,
                                              mac[ue].dl_pmi,
                                              mac[ue].phr,
                                              state));
   }

  ENBSTATS::setMACMetrics(macMetrics);
}

} // end namespace srsenb
