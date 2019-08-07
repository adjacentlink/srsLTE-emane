/*
 * Copyright 2019 Software Radio Systems Limited
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

/******************************************************************************
 * File:        metrics_ostatistic.h
 * Description: Metrics class for open statistic
 *****************************************************************************/

#ifndef SRSENB_METRICS_OSTATISTIC_H
#define SRSENB_METRICS_OSTATISTIC_H

#include "srslte/interfaces/enb_metrics_interface.h"

namespace srsenb {

class metrics_ostatistic : public srslte::metrics_listener<enb_metrics_t>
{
public:
  metrics_ostatistic();

  void set_metrics(enb_metrics_t &m, const uint32_t period_usec);
  void set_handle(enb_metrics_interface *enb_);
  void stop() {};

private:
  enb_metrics_interface* enb;
};

} // namespace srsenb

#endif // SRSENB_METRICS_OSTATISTIC_H
