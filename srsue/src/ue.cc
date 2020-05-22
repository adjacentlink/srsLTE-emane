/*
 * Copyright 2013-2020 Software Radio Systems Limited
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

#include "srsue/hdr/ue.h"
#include "srslte/build_info.h"
#include "srslte/radio/radio.h"
#include "srslte/srslte.h"
#include "srsue/hdr/phy/phy.h"
#include "srsue/hdr/stack/ue_stack_lte.h"
#include <algorithm>
#include <iostream>
#include <iterator>
#include <pthread.h>
#include <sstream>
#include <string>

#ifdef PHY_ADAPTER_ENABLE
#include "srsue/hdr/phy/phy_adapter.h"
#include "libemanelte/uestatisticmanager.h"
#endif

using namespace srslte;

namespace srsue {

ue::ue() : logger(nullptr)
{
  // print build info
  std::cout << std::endl << get_build_string() << std::endl;
  pool = byte_buffer_pool::get_instance();
}

ue::~ue()
{
  // destruct stack components before cleaning buffer pool
  stack.reset();
  byte_buffer_pool::cleanup();
}

int ue::init(const all_args_t& args_, srslte::logger* logger_)
{
  int ret = SRSLTE_SUCCESS;
  logger = logger_;

  // Init UE log
  log.init("UE  ", logger);
  log.set_level(srslte::LOG_LEVEL_INFO);
  log.info("%s", get_build_string().c_str());

  // Validate arguments
  if (parse_args(args_)) {
    log.console("Error processing arguments. Please check %s for more details.\n", args_.log.filename.c_str());
    return SRSLTE_ERROR;
  }

  // Instantiate layers and stack together our UE
  if (args.stack.type == "lte") {
    std::unique_ptr<ue_stack_lte> lte_stack(new ue_stack_lte());
    if (!lte_stack) {
      log.console("Error creating LTE stack instance.\n");
      return SRSLTE_ERROR;
    }

    std::unique_ptr<gw> gw_ptr(new gw());
    if (!gw_ptr) {
      log.console("Error creating a GW instance.\n");
      return SRSLTE_ERROR;
    }

    std::unique_ptr<srsue::phy> lte_phy = std::unique_ptr<srsue::phy>(new srsue::phy(logger));
    if (!lte_phy) {
      log.console("Error creating LTE PHY instance.\n");
      return SRSLTE_ERROR;
    }

    std::unique_ptr<srslte::radio> lte_radio = std::unique_ptr<srslte::radio>(new srslte::radio(logger));
    if (!lte_radio) {
      log.console("Error creating radio multi instance.\n");
      return SRSLTE_ERROR;
    }

    // init layers (do not exit immedietly if something goes wrong as sub-layers may already use interfaces)
    if (lte_radio->init(args.rf, lte_phy.get())) {
      log.console("Error initializing radio.\n");
      ret = SRSLTE_ERROR;
    }

    if (lte_phy->init(args.phy, lte_stack.get(), lte_radio.get())) {
      log.console("Error initializing PHY.\n");
      ret = SRSLTE_ERROR;
    }

    if (lte_stack->init(args.stack, logger, lte_phy.get(), gw_ptr.get())) {
      log.console("Error initializing stack.\n");
      ret = SRSLTE_ERROR;
    }

    if (gw_ptr->init(args.gw, logger, lte_stack.get())) {
      log.console("Error initializing GW.\n");
      ret = SRSLTE_ERROR;
    }

    // move ownership
    stack   = std::move(lte_stack);
    gw_inst = std::move(gw_ptr);
    phy     = std::move(lte_phy);
    radio   = std::move(lte_radio);
  } else {
    log.console("Invalid stack type %s. Supported values are [lte].\n", args.stack.type.c_str());
    ret = SRSLTE_ERROR;
  }

  if (phy) {
    log.console("Waiting PHY to initialize ... ");
    phy->wait_initialize();
    log.console("done!\n");
  }

  // ALINK_XXX set log level to prevent info level logs srslte issue #393
  // log.set_level(srslte::LOG_LEVEL_WARNING);

  return ret;
}

int ue::parse_args(const all_args_t& args_)
{
  // set member variable
  args = args_;

  // carry out basic sanity checks
  if (args.stack.rrc.mbms_service_id > -1) {
    if (!args.phy.interpolate_subframe_enabled) {
      log.error("interpolate_subframe_enabled = %d, While using MBMS, "
                "please set interpolate_subframe_enabled to true\n",
                args.phy.interpolate_subframe_enabled);
      return SRSLTE_ERROR;
    }
    if (args.phy.nof_phy_threads > 2) {
      log.error("nof_phy_threads = %d, While using MBMS, please set "
                "number of phy threads to 1 or 2\n",
                args.phy.nof_phy_threads);
      return SRSLTE_ERROR;
    }
    if ((0 == args.phy.snr_estim_alg.find("refs"))) {
      log.error("snr_estim_alg = refs, While using MBMS, please set "
                "algorithm to pss or empty \n");
      return SRSLTE_ERROR;
    }
  }

  if (args.rf.nof_antennas > SRSLTE_MAX_PORTS) {
    fprintf(stderr, "Maximum number of antennas exceeded (%d > %d)\n", args.rf.nof_antennas, SRSLTE_MAX_PORTS);
    return SRSLTE_ERROR;
  }

  if (args.rf.nof_carriers > SRSLTE_MAX_CARRIERS) {
    fprintf(stderr, "Maximum number of carriers exceeded (%d > %d)\n", args.rf.nof_carriers, SRSLTE_MAX_CARRIERS);
    return SRSLTE_ERROR;
  }

  // replicate some RF parameter to make them available to PHY
  args.phy.nof_carriers = args.rf.nof_carriers;
  args.phy.nof_rx_ant   = args.rf.nof_antennas;
  args.phy.agc_enable   = args.rf.rx_gain < 0.0f;

  // populate DL EARFCN list
  if (!args.phy.dl_earfcn.empty()) {
    args.phy.dl_earfcn_list.clear();
    std::stringstream ss(args.phy.dl_earfcn);
    uint32_t          idx = 0;
    while (ss.good()) {
      std::string substr;
      getline(ss, substr, ',');
      uint32_t earfcn                     = (uint32_t)strtoul(substr.c_str(), nullptr, 10);
      args.stack.rrc.supported_bands[idx] = srslte_band_get_band(earfcn);
      args.stack.rrc.nof_supported_bands  = ++idx;
      args.phy.dl_earfcn_list.push_back(earfcn);
    }
  } else {
    log.error("Error: dl_earfcn list is empty\n");
    log.console("Error: dl_earfcn list is empty\n");
    return SRSLTE_ERROR;
  }

  // populate UL EARFCN list
  if (!args.phy.ul_earfcn.empty()) {
    args.phy.ul_earfcn_map.clear();
    std::stringstream ss(args.phy.ul_earfcn);
    uint32_t          idx = 0;
    while (ss.good()) {
      std::string substr;
      getline(ss, substr, ',');
      uint32_t ul_earfcn = (uint32_t)strtoul(substr.c_str(), nullptr, 10);

      if (idx < args.phy.dl_earfcn_list.size()) {
        // If it can be matched with a DL EARFCN, otherwise ignore entry
        uint32_t dl_earfcn                = args.phy.dl_earfcn_list[idx];
        args.phy.ul_earfcn_map[dl_earfcn] = ul_earfcn;
        idx++;
      }
    }
  }

  // Set UE category
  args.stack.rrc.ue_category = (uint32_t)strtoul(args.stack.rrc.ue_category_str.c_str(), nullptr, 10);

#ifdef PHY_ADAPTER_ENABLE
  UESTATS::initialize(args.general.metrics_period_secs);

  phy_adapter::ue_initialize(&log, 1, args.mhal);
  phy_adapter::ue_start();
#endif

  // Consider Carrier Aggregation support if more than one
  args.stack.rrc.support_ca = (args.rf.nof_carriers > 1);

  return SRSLTE_SUCCESS;
}

void ue::stop()
{
  // tear down UE in reverse order
  if (stack) {
    stack->stop();
#ifdef PHY_ADAPTER_ENABLE
    phy_adapter::ue_stop();
#endif
  }

  if (gw_inst) {
    gw_inst->stop();
  }

  if (phy) {
    phy->stop();
  }

  if (radio) {
    radio->stop();
  }
}

bool ue::switch_on()
{
  return stack->switch_on();
}

bool ue::switch_off()
{
  if (gw_inst) {
    gw_inst->stop();
  }
  return stack->switch_off();
}

void ue::start_plot()
{
  phy->start_plot();
}

bool ue::get_metrics(ue_metrics_t* m)
{
  bzero(m, sizeof(ue_metrics_t));
  phy->get_metrics(&m->phy);
  radio->get_metrics(&m->rf);
  stack->get_metrics(&m->stack);
  gw_inst->get_metrics(m->gw);

  UESTATS::setRRCState(rrc_state_text[m->stack.rrc.state]);
  UESTATS::setEMMState(emm_state_text[m->stack.nas.state]);

  return true;
}

std::string ue::get_build_mode()
{
  return std::string(srslte_get_build_mode());
}

std::string ue::get_build_info()
{
  if (std::string(srslte_get_build_info()).find("  ") != std::string::npos) {
    return std::string(srslte_get_version());
  }
  return std::string(srslte_get_build_info());
}

std::string ue::get_build_string()
{
  std::stringstream ss;
  ss << "Built in " << get_build_mode() << " mode using " << get_build_info() << "." << std::endl;
  return ss.str();
}

} // namespace srsue
