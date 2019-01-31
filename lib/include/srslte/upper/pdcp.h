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

#ifndef SRSLTE_PDCP_H
#define SRSLTE_PDCP_H

#include "srslte/common/log.h"
#include "srslte/common/common.h"
#include "srslte/interfaces/ue_interfaces.h"
#include "srslte/upper/pdcp_entity.h"

namespace srslte {

class pdcp
    :public srsue::pdcp_interface_gw
    ,public srsue::pdcp_interface_rlc
    ,public srsue::pdcp_interface_rrc
{
public:
  pdcp();
  virtual ~pdcp();
  void init(srsue::rlc_interface_pdcp *rlc_,
            srsue::rrc_interface_pdcp *rrc_,
            srsue::gw_interface_pdcp *gw_,
            log *pdcp_log_,
            uint32_t lcid_,
            uint8_t direction_);
  void stop();

  // GW interface
  bool is_lcid_enabled(uint32_t lcid);

  // RRC interface
  void reestablish();
  void reset();
  void write_sdu(uint32_t lcid, byte_buffer_t *sdu, bool blocking = true);
  void write_sdu_mch(uint32_t lcid, byte_buffer_t *sdu);
  void add_bearer(uint32_t lcid, srslte_pdcp_config_t cnfg = srslte_pdcp_config_t());
  void add_bearer_mrb(uint32_t lcid, srslte_pdcp_config_t cnfg = srslte_pdcp_config_t());
  void del_bearer(uint32_t lcid);
  void change_lcid(uint32_t old_lcid, uint32_t new_lcid);
  void config_security(uint32_t lcid,
                       uint8_t *k_enc,
                       uint8_t *k_int,
                       CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                       INTEGRITY_ALGORITHM_ID_ENUM integ_algo);
  void config_security_all(uint8_t *k_enc,
                           uint8_t *k_int,
                           CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                           INTEGRITY_ALGORITHM_ID_ENUM integ_algo);
  void enable_integrity(uint32_t lcid);
  void enable_encryption(uint32_t lcid);
  uint32_t get_dl_count(uint32_t lcid);
  uint32_t get_ul_count(uint32_t lcid);

  // RLC interface
  void write_pdu(uint32_t lcid, byte_buffer_t *sdu);
  void write_pdu_mch(uint32_t lcid, byte_buffer_t *sdu);
  void write_pdu_bcch_bch(byte_buffer_t *sdu);
  void write_pdu_bcch_dlsch(byte_buffer_t *sdu);
  void write_pdu_pcch(byte_buffer_t *sdu);

private:
  srsue::rlc_interface_pdcp *rlc;
  srsue::rrc_interface_pdcp *rrc;
  srsue::gw_interface_pdcp  *gw;

  typedef std::map<uint16_t, pdcp_entity_interface*> pdcp_map_t;
  typedef std::pair<uint16_t, pdcp_entity_interface*> pdcp_map_pair_t;

  log         *pdcp_log;
  pdcp_map_t  pdcp_array, pdcp_array_mrb;
  pthread_rwlock_t rwlock;

  // default PDCP entity that is maintained active by PDCP instance
  srslte_pdcp_config_t default_cnfg;
  uint32_t             default_lcid;

  bool valid_lcid(uint32_t lcid);
  bool valid_mch_lcid(uint32_t lcid);
};

} // namespace srslte


#endif // SRSLTE_PDCP_H
