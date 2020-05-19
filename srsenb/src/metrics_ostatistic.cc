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

void metrics_ostatistic::set_metrics(const enb_metrics_t &m, const uint32_t)
{
/* 
typedef struct {
  srslte::rf_metrics_t rf;
  phy_metrics_t        phy[ENB_METRICS_MAX_USERS];
  stack_metrics_t      stack;
  bool                 running;
} enb_metrics_t; 

	typedef struct {
	  uint32_t rf_o;
	  uint32_t rf_u;
	  uint32_t rf_l;
	  bool     rf_error;
	} rf_metrics_t;

	struct phy_metrics_t {
	  dl_metrics_t dl;
	  ul_metrics_t ul;
	};
		 struct ul_metrics_t {
		  float n;
		  float sinr;
		  float rssi;
		  float turbo_iters;
		  float mcs;
		  int   n_samples;
		};

		struct dl_metrics_t {
		  float mcs;
		  int   n_samples;
		};

	struct stack_metrics_t {
	  mac_metrics_t  mac[ENB_METRICS_MAX_USERS];
	  rrc_metrics_t  rrc;
	  s1ap_metrics_t s1ap; 
	};
		struct mac_metrics_t {
		  uint16_t rnti;
		  uint32_t nof_tti;
		  int      tx_pkts;
		  int      tx_errors;
		  int      tx_brate;
		  int      rx_pkts;
		  int      rx_errors;
		  int      rx_brate;
		  int      ul_buffer;
		  int      dl_buffer;
		  float    dl_cqi;
		  float    dl_ri;
		  float    dl_pmi;
		  float    phr;
		};


		struct rrc_metrics_t {
		  uint16_t         n_ues;
		  rrc_ue_metrics_t ues[ENB_METRICS_MAX_USERS];
		};
			struct rrc_ue_metrics_t {
			  rrc_state_t state;
			};
				 typedef enum {
				  RRC_STATE_IDLE = 0,
				  RRC_STATE_WAIT_FOR_CON_SETUP_COMPLETE,
				  RRC_STATE_WAIT_FOR_SECURITY_MODE_COMPLETE,
				  RRC_STATE_WAIT_FOR_UE_CAP_INFO,
				  RRC_STATE_WAIT_FOR_CON_RECONF_COMPLETE,
				  RRC_STATE_REGISTERED,
				  RRC_STATE_RELEASE_REQUEST,
				  RRC_STATE_N_ITEMS,
				} rrc_state_t;

		struct s1ap_metrics_t {
		  S1AP_STATUS_ENUM status;
		};
			typedef enum {
			  S1AP_ATTACHING = 0, // Attempting to create S1 connection
			  S1AP_READY,         // S1 connected
			  S1AP_ERROR          // Failure
			} S1AP_STATUS_ENUM;
*/
 

  const auto & stack = m.stack;
  const auto & rrc   = stack.rrc;
  const auto & mac   = stack.mac;
  const auto & s1ap  = stack.s1ap;

  ENBSTATS::MACMetrics    macMetrics;

  ENBSTATS::setS1State(s1ap.status == S1AP_ATTACHING ? "ATTACHING" :
                       s1ap.status == S1AP_READY     ? "READY"     : 
                                                       "ERROR");

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
   }

  ENBSTATS::setMACMetrics(macMetrics);
}

} // end namespace srsenb
