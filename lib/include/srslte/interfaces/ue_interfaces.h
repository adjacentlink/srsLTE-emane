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

/******************************************************************************
 * File:        interfaces.h
 * Description: Abstract base class interfaces provided by layers
 *              to other layers.
 *****************************************************************************/

#ifndef SRSLTE_UE_INTERFACES_H
#define SRSLTE_UE_INTERFACES_H

#include <string>

#include "srslte/asn1/liblte_mme.h"
#include "srslte/asn1/rrc_asn1.h"
#include "srslte/common/common.h"
#include "srslte/common/interfaces_common.h"
#include "srslte/common/security.h"
#include "srslte/upper/rlc_interface.h"

namespace srsue {

typedef enum {
  AUTH_OK,
  AUTH_FAILED,
  AUTH_SYNCH_FAILURE
} auth_result_t;

// UE interface
class ue_interface
{
};

// USIM interface for NAS
class usim_interface_nas
{
public:
  virtual std::string get_imsi_str() = 0;
  virtual std::string get_imei_str() = 0;
  virtual bool get_imsi_vec(uint8_t* imsi_, uint32_t n) = 0;
  virtual bool get_imei_vec(uint8_t* imei_, uint32_t n) = 0;
  virtual bool          get_home_plmn_id(asn1::rrc::plmn_id_s* home_plmn_id)              = 0;
  virtual auth_result_t generate_authentication_response(uint8_t  *rand,
                                                uint8_t  *autn_enb,
                                                uint16_t  mcc,
                                                uint16_t  mnc,
                                                uint8_t  *res,
                                                int      *res_len,
                                                uint8_t  *k_asme) = 0;
  virtual void generate_nas_keys(uint8_t *k_asme,
                                 uint8_t *k_nas_enc,
                                 uint8_t *k_nas_int,
                                 srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                                 srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo) = 0;
};

// USIM interface for RRC
class usim_interface_rrc
{
public:
  virtual void generate_as_keys(uint8_t *k_asme,
                                uint32_t count_ul,
                                uint8_t *k_rrc_enc,
                                uint8_t *k_rrc_int,
                                uint8_t *k_up_enc,
                                uint8_t *k_up_int,
                                srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                                srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo) = 0;
  virtual void generate_as_keys_ho(uint32_t pci,
                                   uint32_t earfcn,
                                   int ncc,
                                   uint8_t *k_rrc_enc,
                                   uint8_t *k_rrc_int,
                                   uint8_t *k_up_enc,
                                   uint8_t *k_up_int,
                                   srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo,
                                   srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo) = 0;
};

// GW interface for NAS
class gw_interface_nas
{
public:
  virtual srslte::error_t setup_if_addr(uint8_t pdn_type, uint32_t ip_addr, uint8_t *ipv6_if_id, char *err_str) = 0;
};

// GW interface for RRC
class gw_interface_rrc
{
public:
  virtual void add_mch_port(uint32_t lcid, uint32_t port) = 0;
};

// GW interface for PDCP
class gw_interface_pdcp
{
public:
  virtual void write_pdu(uint32_t lcid, srslte::byte_buffer_t *pdu) = 0;
  virtual void write_pdu_mch(uint32_t lcid, srslte::byte_buffer_t *pdu) = 0;
};

// NAS interface for RRC
class nas_interface_rrc
{
public:
  typedef enum {
    BARRING_NONE = 0,
    BARRING_MO_DATA,
    BARRING_MO_SIGNALLING,
    BARRING_MT,
    BARRING_ALL
  } barring_t;
  virtual void      set_barring(barring_t barring) = 0;
  virtual void      paging(asn1::rrc::s_tmsi_s* ue_identiy)              = 0;
  virtual bool      is_attached() = 0;
  virtual void      write_pdu(uint32_t lcid, srslte::byte_buffer_t *pdu) = 0;
  virtual uint32_t  get_k_enb_count() = 0;
  virtual bool      get_k_asme(uint8_t *k_asme_, uint32_t n) = 0;
  virtual uint32_t  get_ipv4_addr() = 0;
  virtual bool      get_ipv6_addr(uint8_t *ipv6_addr) = 0;
};

// NAS interface for UE
class nas_interface_ue
{
public:
  virtual bool attach_request() = 0;
  virtual bool detach_request() = 0;
};

// NAS interface for UE
class nas_interface_gw
{
public:
  virtual bool attach_request() = 0;
};

// RRC interface for MAC
class rrc_interface_mac_common
{
public:
  virtual void ra_problem() = 0;
};

class rrc_interface_mac : public rrc_interface_mac_common
{
public:
  virtual void ho_ra_completed(bool ra_successful) = 0;
  virtual void release_pucch_srs() = 0;
  virtual void run_tti(uint32_t tti) = 0;
};

// RRC interface for PHY
class rrc_interface_phy
{
public:
  virtual void in_sync() = 0;
  virtual void out_of_sync() = 0;
  virtual void new_phy_meas(float rsrp, float rsrq, uint32_t tti, int earfcn = -1, int pci = -1) = 0;
};

// RRC interface for NAS
class rrc_interface_nas
{
public:
  typedef struct {
    asn1::rrc::plmn_id_s plmn_id;
    uint16_t             tac;
  } found_plmn_t;

  const static int MAX_FOUND_PLMNS = 16;

  virtual void write_sdu(srslte::byte_buffer_t *sdu) = 0;
  virtual uint16_t get_mcc() = 0;
  virtual uint16_t get_mnc() = 0;
  virtual void enable_capabilities() = 0;
  virtual int plmn_search(found_plmn_t found_plmns[MAX_FOUND_PLMNS]) = 0;
  virtual void     plmn_select(asn1::rrc::plmn_id_s plmn_id)                                                       = 0;
  virtual bool connection_request(asn1::rrc::establishment_cause_e cause, srslte::byte_buffer_t* dedicatedInfoNAS) = 0;
  virtual void set_ue_idenity(asn1::rrc::s_tmsi_s s_tmsi)                                                          = 0;
  virtual bool is_connected() = 0;
  virtual std::string get_rb_name(uint32_t lcid) = 0;
};

// RRC interface for PDCP
class rrc_interface_pdcp
{
public:
  virtual void write_pdu(uint32_t lcid, srslte::byte_buffer_t *pdu) = 0;
  virtual void write_pdu_bcch_bch(srslte::byte_buffer_t *pdu) = 0;
  virtual void write_pdu_bcch_dlsch(srslte::byte_buffer_t *pdu) = 0;
  virtual void write_pdu_pcch(srslte::byte_buffer_t *pdu) = 0;
  virtual void write_pdu_mch(uint32_t lcid, srslte::byte_buffer_t *pdu) = 0;
  virtual std::string get_rb_name(uint32_t lcid) = 0;
};

// RRC interface for RLC
class rrc_interface_rlc
{
public:
  virtual void max_retx_attempted() = 0;
  virtual std::string get_rb_name(uint32_t lcid) = 0;
};

// PDCP interface for GW
class pdcp_interface_gw
{
public:
  virtual void write_sdu(uint32_t lcid, srslte::byte_buffer_t *sdu, bool blocking) = 0;
  virtual bool is_lcid_enabled(uint32_t lcid) = 0;
};

// PDCP interface for RRC
class pdcp_interface_rrc
{
public:
  virtual void reestablish() = 0;
  virtual void reset() = 0;
  virtual void write_sdu(uint32_t lcid, srslte::byte_buffer_t *sdu, bool blocking = true) = 0;
  virtual void add_bearer(uint32_t lcid, srslte::srslte_pdcp_config_t cnfg = srslte::srslte_pdcp_config_t()) = 0;
  virtual void change_lcid(uint32_t old_lcid, uint32_t new_lcid) = 0;
  virtual void config_security(uint32_t lcid,
                               uint8_t *k_enc_,
                               uint8_t *k_int_,
                               srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo_,
                               srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo_) = 0;
  virtual void config_security_all(uint8_t *k_enc_,
                                   uint8_t *k_int_,
                                   srslte::CIPHERING_ALGORITHM_ID_ENUM cipher_algo_,
                                   srslte::INTEGRITY_ALGORITHM_ID_ENUM integ_algo_) = 0;
  virtual void enable_integrity(uint32_t lcid) = 0;
  virtual void enable_encryption(uint32_t lcid) = 0;
  virtual uint32_t get_dl_count(uint32_t lcid) = 0;
  virtual uint32_t get_ul_count(uint32_t lcid) = 0;
};

// PDCP interface for RLC
class pdcp_interface_rlc
{
public:
  /* RLC calls PDCP to push a PDCP PDU. */
  virtual void write_pdu(uint32_t lcid, srslte::byte_buffer_t *sdu) = 0;
  virtual void write_pdu_bcch_bch(srslte::byte_buffer_t *sdu) = 0;
  virtual void write_pdu_bcch_dlsch(srslte::byte_buffer_t *sdu) = 0;
  virtual void write_pdu_pcch(srslte::byte_buffer_t *sdu) = 0;
  virtual void write_pdu_mch(uint32_t lcid, srslte::byte_buffer_t *sdu) = 0;
};

// RLC interface for RRC
class rlc_interface_rrc
{
public:
  virtual void reset() = 0;
  virtual void reestablish() = 0;
  virtual void reestablish(uint32_t lcid) = 0;
  virtual void add_bearer(uint32_t lcid) = 0;
  virtual void add_bearer(uint32_t lcid, srslte::srslte_rlc_config_t cnfg) = 0;
  virtual void add_bearer_mrb(uint32_t lcid) = 0;
  virtual void del_bearer(uint32_t lcid) = 0;
  virtual void change_lcid(uint32_t old_lcid, uint32_t new_lcid) = 0;
  virtual bool has_bearer(uint32_t lcid) = 0;
  virtual bool has_data(const uint32_t lcid)                               = 0;
};

// RLC interface for PDCP
class rlc_interface_pdcp
{
public:
  /* PDCP calls RLC to push an RLC SDU. SDU gets placed into the RLC buffer and MAC pulls
   * RLC PDUs according to TB size. */
  virtual void write_sdu(uint32_t lcid,  srslte::byte_buffer_t *sdu, bool blocking = true) = 0;
  virtual bool rb_is_um(uint32_t lcid) = 0;
};

//RLC interface for MAC
class rlc_interface_mac : public srslte::read_pdu_interface
{
public:
  /* MAC calls has_data() to query whether a logical channel has data to transmit (without
   * knowing how much. This function should return quickly. */
  virtual bool has_data(const uint32_t lcid) = 0;

  /* MAC calls RLC to get the buffer state for a logical channel. */
  virtual uint32_t get_buffer_state(const uint32_t lcid) = 0;

  const static int MAX_PDU_SEGMENTS = 20;

  /* MAC calls RLC to get RLC segment of nof_bytes length.
   * Segmentation happens in this function. RLC PDU is stored in payload. */
  virtual int     read_pdu(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes) = 0;

  /* MAC calls RLC to push an RLC PDU. This function is called from an independent MAC thread.
   * PDU gets placed into the buffer and higher layer thread gets notified. */
  virtual void write_pdu(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes) = 0;
  virtual void write_pdu_bcch_bch(uint8_t *payload, uint32_t nof_bytes) = 0;
  virtual void write_pdu_bcch_dlsch(uint8_t *payload, uint32_t nof_bytes) = 0;
  virtual void write_pdu_pcch(uint8_t *payload, uint32_t nof_bytes) = 0;
  virtual void write_pdu_mch(uint32_t lcid, uint8_t *payload, uint32_t nof_bytes) = 0;
};


//BSR interface for MUX
class bsr_interface_mux
{
public:
  typedef enum {
    LONG_BSR,
    SHORT_BSR,
    TRUNC_BSR
  } bsr_format_t;

  typedef struct {
    bsr_format_t format;
    uint32_t buff_size[4];
  } bsr_t;

  /* MUX calls BSR to check if it can fit a BSR into PDU */
  virtual bool need_to_send_bsr_on_ul_grant(uint32_t grant_size, bsr_t *bsr) = 0;

  /* MUX calls BSR to let it generate a padding BSR if there is space in PDU */
  virtual bool generate_padding_bsr(uint32_t nof_padding_bytes, bsr_t *bsr) = 0;

  /* MAX calls BSR to set the Tx TTI */
  virtual void set_tx_tti(uint32_t tti) = 0;
};


/** MAC interface 
 *
 */
/* Interface PHY -> MAC */
class mac_interface_phy
{
public:
  typedef struct {
      uint32_t nof_mbsfn_services;
  } mac_phy_cfg_mbsfn_t; 
    
    
    
  typedef struct {
    uint32_t    pid;    
    uint32_t    tti;
    uint32_t    last_tti;
    bool        ndi[SRSLTE_MAX_CODEWORDS];
    bool        last_ndi[SRSLTE_MAX_CODEWORDS];
    uint32_t    n_bytes[SRSLTE_MAX_CODEWORDS];
    int         rv[SRSLTE_MAX_CODEWORDS];
    bool        tb_en[SRSLTE_MAX_CODEWORDS];
    bool        tb_cw_swap;
    uint16_t    rnti; 
    bool        is_from_rar;
    bool        is_sps_release;
    bool        has_cqi_request;
    srslte_rnti_type_t rnti_type; 
    srslte_phy_grant_t phy_grant; 
  } mac_grant_t; 
  
  typedef struct {
    bool                    decode_enabled[SRSLTE_MAX_TB];
    int                     rv[SRSLTE_MAX_TB];
    uint16_t                rnti; 
    bool                    generate_ack; 
    bool                    default_ack[SRSLTE_MAX_TB];
    // If non-null, called after tb_decoded_ok to determine if ack needs to be sent
    bool                  (*generate_ack_callback)(void*); 
    void                   *generate_ack_callback_arg;
    uint8_t                *payload_ptr[SRSLTE_MAX_TB];
    srslte_softbuffer_rx_t *softbuffers[SRSLTE_MAX_TB];
    srslte_phy_grant_t      phy_grant;
  } tb_action_dl_t;

  typedef struct {
    bool                    tx_enabled;
    bool                    expect_ack;
    uint32_t                rv[SRSLTE_MAX_TB];
    uint16_t                rnti; 
    uint32_t                current_tx_nb;
    int32_t                 tti_offset;     // relative offset between grant and UL tx/HARQ rx
    srslte_softbuffer_tx_t *softbuffers;
    srslte_phy_grant_t      phy_grant;
    uint8_t                *payload_ptr[SRSLTE_MAX_TB];
  } tb_action_ul_t;
  
  /* Indicate reception of UL grant. 
   * payload_ptr points to memory where MAC PDU must be written by MAC layer */
  virtual void new_grant_ul(mac_grant_t grant, tb_action_ul_t *action) = 0;

  /* Indicate reception of UL grant + HARQ information throught PHICH in the same TTI. */
  virtual void new_grant_ul_ack(mac_grant_t grant, bool ack, tb_action_ul_t *action) = 0;

  /* Obtain action for a new MCH subframe. */
  virtual void new_mch_dl(srslte_ra_dl_grant_t phy_grant, tb_action_dl_t *action) = 0;

  /* Indicate reception of HARQ information only through PHICH.   */
  virtual void harq_recv(uint32_t tti, bool ack, tb_action_ul_t *action) = 0;
  
  /* Indicate reception of DL grant. */ 
  virtual void new_grant_dl(mac_grant_t grant, tb_action_dl_t *action) = 0;
  
  /* Indicate successful decoding of PDSCH TB. */
  virtual void tb_decoded(bool ack, uint32_t tb_idx, srslte_rnti_type_t rnti_type, uint32_t harq_pid) = 0;
  
  /* Indicate successful decoding of BCH TB through PBCH */
  virtual void bch_decoded_ok(uint8_t *payload, uint32_t len) = 0;  
  
  /* Indicate successful decoding of PCH TB through PDSCH */
  virtual void pch_decoded_ok(uint32_t len) = 0;  

  /* Indicate successful decoding of MCH TB through PMCH */
  virtual void mch_decoded_ok(uint32_t len) = 0;
  
  /* Communicate the number of mbsfn services available  */
  virtual void set_mbsfn_config(uint32_t nof_mbsfn_services) = 0;
  
  /* Function called every start of a subframe (TTI). Warning, this function is called 
   * from a high priority thread and should terminate asap 
   */


};

/* Interface RRC -> MAC shared between different RATs */
class mac_interface_rrc_common
{
public:
  // Class to handle UE specific RNTIs between RRC and MAC
  typedef struct {
    uint16_t crnti;
    uint16_t temp_rnti;
    uint16_t tpc_rnti;
    uint16_t sps_rnti;
    uint64_t contention_id;
  } ue_rnti_t;

  typedef struct {
    uint32_t max_harq_msg3_tx;
    uint32_t max_harq_tx;
  } ul_harq_params_t;
};

/* Interface RRC -> MAC */
class mac_interface_rrc : public mac_interface_rrc_common
{
public:
  
  typedef struct {
    asn1::rrc::mac_main_cfg_s      main;
    asn1::rrc::rach_cfg_common_s   rach;
    asn1::rrc::sched_request_cfg_c sr;
    ul_harq_params_t               ul_harq_params;
    uint32_t                       prach_config_index;
  } mac_cfg_t;

  virtual void    clear_rntis() = 0;

  /* Instructs the MAC to start receiving BCCH */
  virtual void    bcch_start_rx(int si_window_start, int si_window_length) = 0;

  /* Instructs the MAC to start receiving PCCH */
  virtual void    pcch_start_rx() = 0; 

  /* RRC configures a logical channel */
  virtual void    setup_lcid(uint32_t lcid, uint32_t lcg, uint32_t priority, int PBR_x_tti, uint32_t BSD) = 0;

  /* Instructs the MAC to start receiving an MCH */
  virtual void    mch_start_rx(uint32_t lcid) = 0;

  virtual uint32_t get_current_tti() = 0;

  virtual void set_config(mac_cfg_t *mac_cfg) = 0;
  virtual void set_config_main(asn1::rrc::mac_main_cfg_s* main_cfg)                                 = 0;
  virtual void set_config_rach(asn1::rrc::rach_cfg_common_s* rach_cfg, uint32_t prach_config_index) = 0;
  virtual void set_config_sr(asn1::rrc::sched_request_cfg_c* sr_cfg)                                = 0;
  virtual void get_config(mac_cfg_t *mac_cfg) = 0;
  
  virtual void get_rntis(ue_rnti_t *rntis) = 0;
  virtual void set_contention_id(uint64_t uecri) = 0;
  virtual void set_ho_rnti(uint16_t crnti, uint16_t target_pci) = 0;

  virtual void start_noncont_ho(uint32_t preamble_index, uint32_t prach_mask) = 0;
  virtual void start_cont_ho() = 0;

  virtual void reconfiguration() = 0;
  virtual void reset() = 0;
  virtual void wait_uplink() = 0;
};


/** PHY interface 
 *
 */

typedef struct {
  bool ul_pwr_ctrl_en; 
  float prach_gain;
  int pdsch_max_its;
  bool attach_enable_64qam; 
  int nof_phy_threads;
  
  int worker_cpu_mask;
  int sync_cpu_affinity;
  
  uint32_t nof_rx_ant;
  std::string equalizer_mode;
  int cqi_max; 
  int cqi_fixed; 
  float snr_ema_coeff; 
  std::string snr_estim_alg;
  bool cfo_is_doppler;
  bool cfo_integer_enabled; 
  float cfo_correct_tol_hz;
  float cfo_pss_ema;
  float cfo_ref_ema;
  float cfo_loop_bw_pss;
  float cfo_loop_bw_ref;
  float cfo_loop_ref_min;
  float cfo_loop_pss_tol;
  float sfo_ema;
  uint32_t sfo_correct_period;
  uint32_t cfo_loop_pss_conv;
  uint32_t cfo_ref_mask;
  bool average_subframe_enabled;
  bool estimator_fil_auto;
  float estimator_fil_stddev;
  uint32_t estimator_fil_order;
  std::string sss_algorithm;
  bool rssi_sensor_enabled;
  bool sic_pss_enabled;
  float rx_gain_offset;
  bool pdsch_csi_enabled;
  bool pdsch_8bit_decoder;
  uint32_t intra_freq_meas_len_ms;
  uint32_t intra_freq_meas_period_ms;
} phy_args_t; 


/* RAT agnostic Interface MAC -> PHY */
class phy_interface_mac_common
{
public:

  /* Sets a C-RNTI allowing the PHY to pregenerate signals if necessary */
  virtual void set_crnti(uint16_t rnti) = 0;

  /* Time advance commands */
  virtual void set_timeadv_rar(uint32_t ta_cmd) = 0;
  virtual void set_timeadv(uint32_t ta_cmd) = 0;

  /* Sets RAR grant payload */
  virtual void set_rar_grant(uint32_t tti, uint8_t grant_payload[SRSLTE_RAR_GRANT_LEN]) = 0;

  virtual uint32_t get_current_tti() = 0;

  virtual float get_phr() = 0;
  virtual float get_pathloss_db() = 0;
};

/* Interface MAC -> PHY */
class phy_interface_mac : public phy_interface_mac_common
{
public:
      
  /* Configure PRACH using parameters written by RRC */
  virtual void configure_prach_params() = 0;

  virtual void prach_send(uint32_t preamble_idx, int allowed_subframe, float target_power_dbm) = 0;  
  virtual int  prach_tx_tti() = 0;   
  /* Indicates the transmission of a SR signal in the next opportunity */
  virtual void sr_send() = 0;  
  virtual int  sr_last_tx_tti() = 0; 

  /* Instruct the PHY to decode PDCCH with the CRC scrambled with given RNTI */
  virtual void pdcch_ul_search(srslte_rnti_type_t rnti_type, uint16_t rnti, int tti_start = -1, int tti_end = -1) = 0;
  virtual void pdcch_dl_search(srslte_rnti_type_t rnti_type, uint16_t rnti, int tti_start = -1, int tti_end = -1) = 0;
  virtual void pdcch_ul_search_reset() = 0;
  virtual void pdcch_dl_search_reset() = 0;
  
  virtual void set_mch_period_stop(uint32_t stop) = 0;
  
};

class phy_interface_rrc
{
public:
  struct phy_cfg_common_t {
    asn1::rrc::prach_cfg_sib_s      prach_cnfg;
    asn1::rrc::pdsch_cfg_common_s   pdsch_cnfg;
    asn1::rrc::pusch_cfg_common_s   pusch_cnfg;
    asn1::rrc::phich_cfg_s          phich_cnfg;
    asn1::rrc::pucch_cfg_common_s   pucch_cnfg;
    asn1::rrc::srs_ul_cfg_common_c  srs_ul_cnfg;
    asn1::rrc::ul_pwr_ctrl_common_s ul_pwr_ctrl;
    asn1::rrc::tdd_cfg_s            tdd_cnfg;
    asn1::rrc::srs_ant_port_e       ant_info;
  };

  struct phy_cfg_mbsfn_t {
    asn1::rrc::mbsfn_sf_cfg_s       mbsfn_subfr_cnfg;
    asn1::rrc::mbms_notif_cfg_r9_s  mbsfn_notification_cnfg;
    asn1::rrc::mbsfn_area_info_r9_s mbsfn_area_info;
    asn1::rrc::mcch_msg_s           mcch;
  };

  typedef struct {
    asn1::rrc::phys_cfg_ded_s dedicated;
    phy_cfg_common_t          common;
    phy_cfg_mbsfn_t           mbsfn;
    bool                      enable_64qam;
  } phy_cfg_t; 

  virtual void get_current_cell(srslte_cell_t *cell, uint32_t *current_earfcn = NULL) = 0;
  virtual uint32_t get_current_earfcn() = 0;
  virtual uint32_t get_current_pci() = 0;

  virtual void get_config(phy_cfg_t *phy_cfg) = 0;
  virtual void set_config(phy_cfg_t *phy_cfg) = 0;
  virtual void set_config_dedicated(asn1::rrc::phys_cfg_ded_s* dedicated) = 0;
  virtual void set_config_common(phy_cfg_common_t *common) = 0;
  virtual void set_config_tdd(asn1::rrc::tdd_cfg_s* tdd)                  = 0;
  virtual void set_config_64qam_en(bool enable) = 0;
  virtual void set_config_mbsfn_sib2(asn1::rrc::sib_type2_s* sib2)        = 0;
  virtual void set_config_mbsfn_sib13(asn1::rrc::sib_type13_r9_s* sib13)  = 0;
  virtual void set_config_mbsfn_mcch(asn1::rrc::mcch_msg_s* mcch)         = 0;

  /* Measurements interface */
  virtual void meas_reset() = 0;
  virtual int  meas_start(uint32_t earfcn, int pci = -1) = 0;
  virtual int  meas_stop(uint32_t earfcn, int pci = -1) = 0;

  typedef struct {
    enum {CELL_FOUND = 0, CELL_NOT_FOUND, ERROR} found;
    enum {MORE_FREQS = 0, NO_MORE_FREQS} last_freq;
  } cell_search_ret_t;

  typedef struct {
    srslte_cell_t cell;
    uint32_t      earfcn;
  } phy_cell_t;

  /* Cell search and selection procedures */
  virtual cell_search_ret_t  cell_search(phy_cell_t *cell) = 0;
  virtual bool cell_select(phy_cell_t *cell = NULL) = 0;
  virtual bool cell_is_camping() = 0;

  /* Configure UL using parameters written with set_param() */
  virtual void configure_ul_params(bool pregen_disabled = false) = 0;

  virtual void reset() = 0;

};
  

} // namespace srsue

#endif // SRSLTE_UE_INTERFACES_H
