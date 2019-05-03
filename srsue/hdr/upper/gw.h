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

#ifndef SRSUE_GW_H
#define SRSUE_GW_H

#include <net/if.h>
#include "srslte/common/buffer_pool.h"
#include "srslte/common/log.h"
#include "srslte/common/common.h"
#include "srslte/common/interfaces_common.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/common/threads.h"
#include "gw_metrics.h"

namespace srsue {

class gw
    :public gw_interface_pdcp
    ,public gw_interface_nas
    ,public gw_interface_rrc
    ,public thread
{
public:
  gw();
  void init(pdcp_interface_gw *pdcp_, nas_interface_gw *nas_, srslte::log *gw_log_, srslte::srslte_gw_config_t);
  void stop();

  void get_metrics(gw_metrics_t &m);
  void set_netmask(std::string netmask);
  void set_tundevname(const std::string & devname);

  // PDCP interface
  void write_pdu(uint32_t lcid, srslte::byte_buffer_t *pdu);
  void write_pdu_mch(uint32_t lcid, srslte::byte_buffer_t *pdu);

  // NAS interface
  srslte::error_t setup_if_addr(uint8_t pdn_type, uint32_t ip_addr, uint8_t* ipv6_if_addr, char *err_str);

  // RRC interface
  void add_mch_port(uint32_t lcid, uint32_t port);

private:

  bool default_netmask;
  std::string netmask;
  std::string tundevname;

  static const int GW_THREAD_PRIO = 7;

  pdcp_interface_gw  *pdcp;
  nas_interface_gw   *nas;

  srslte::byte_buffer_pool   *pool;
  srslte::log                *gw_log;

  srslte::srslte_gw_config_t cfg;

  bool                running;
  bool                run_enable;
  int32_t             tun_fd;
  struct ifreq        ifr;
  int32_t             sock;
  bool                if_up;

  uint32_t            current_ip_addr;
  uint8_t             current_if_id[8];

  long                ul_tput_bytes;
  long                dl_tput_bytes;
  struct timeval      metrics_time[3];

  void                run_thread();
  srslte::error_t     init_if(char *err_str);
  srslte::error_t setup_if_addr4(uint32_t ip_addr, char *err_str);
  srslte::error_t setup_if_addr6(uint8_t *ipv6_if_id, char *err_str);
  bool find_ipv6_addr(struct in6_addr *in6_out);
  void del_ipv6_addr(struct in6_addr *in6p);

  // MBSFN
  int      mbsfn_sock_fd;                   // Sink UDP socket file descriptor
  struct   sockaddr_in mbsfn_sock_addr;     // Target address
  uint32_t mbsfn_ports[SRSLTE_N_MCH_LCIDS]; // Target ports for MBSFN data

};

} // namespace srsue


#endif // SRSUE_GW_H
