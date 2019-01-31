/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsUE library.
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

#ifndef SRSLTE_RLC_METRICS_H
#define SRSLTE_RLC_METRICS_H

#include "srslte/common/common.h"
#include "srslte/common/queue_metrics.h"
#include "srslte/upper/rlc_interface.h"

namespace srslte {

struct rlc_queue_metrics_t { 
 queue_metrics_t qmetrics;
 rlc_mode_t      mode;
};


struct rlc_metrics_t
{
  float dl_tput_mbps[SRSLTE_N_RADIO_BEARERS];
  float ul_tput_mbps[SRSLTE_N_RADIO_BEARERS];
  float dl_tput_mrb_mbps[SRSLTE_N_MCH_LCIDS];

  rlc_queue_metrics_t metrics[SRSLTE_N_MCH_LCIDS];
  rlc_queue_metrics_t mrb_metrics[SRSLTE_N_MCH_LCIDS];
};

} // namespace srslte

#endif // SRSLTE_RLC_METRICS_H
