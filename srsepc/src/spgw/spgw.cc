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

#include "srsepc/hdr/spgw/spgw.h"
#include "srsepc/hdr/mme/mme_gtpc.h"
#include "srsepc/hdr/spgw/gtpc.h"
#include "srsepc/hdr/spgw/gtpu.h"
#include "srslte/upper/gtpu.h"
#include <inttypes.h> // for printing uint64_t

#ifdef PHY_ADAPTER_ENABLE
#include "libemanelte/epcstatisticmanager.h"
#endif
namespace srsepc {

spgw*           spgw::m_instance    = NULL;
pthread_mutex_t spgw_instance_mutex = PTHREAD_MUTEX_INITIALIZER;

spgw::spgw() : m_running(false)
{
  m_gtpc = new spgw::gtpc;
  m_gtpu = new spgw::gtpu;
  return;
}

spgw::~spgw()
{
  delete m_gtpc;
  delete m_gtpu;
  return;
}

spgw* spgw::get_instance()
{
  pthread_mutex_lock(&spgw_instance_mutex);
  if (NULL == m_instance) {
    m_instance = new spgw();
  }
  pthread_mutex_unlock(&spgw_instance_mutex);
  return (m_instance);
}

void spgw::cleanup()
{
  pthread_mutex_lock(&spgw_instance_mutex);
  if (NULL != m_instance) {
    delete m_instance;
    m_instance = NULL;
  }
  pthread_mutex_unlock(&spgw_instance_mutex);
}

int spgw::init(spgw_args_t*                           args,
               srslte::log_filter*                    gtpu_log,
               srslte::log_filter*                    gtpc_log,
               srslte::log_filter*                    spgw_log,
               const std::map<std::string, uint64_t>& ip_to_imsi)
{
  srslte::error_t err;
  m_pool = srslte::byte_buffer_pool::get_instance();

  // Init log
  m_spgw_log = spgw_log;

  // Init GTP-U
  if (m_gtpu->init(args, this, m_gtpc, gtpu_log) != 0) {
    m_spgw_log->console("Could not initialize the SPGW's GTP-U.\n");
    return -1;
  }

  // Init GTP-C
  if (m_gtpc->init(args, this, m_gtpu, gtpc_log, ip_to_imsi) != 0) {
    m_spgw_log->console("Could not initialize the S1-U interface.\n");
    return -1;
  }

  m_spgw_log->info("SP-GW Initialized.\n");
  m_spgw_log->console("SP-GW Initialized.\n");
  return 0;
}

void spgw::stop()
{
  if (m_running) {
    m_running = false;
    thread_cancel();
    wait_thread_finish();
  }

  m_gtpu->stop();
  m_gtpc->stop();
  return;
}

void spgw::run_thread()
{
  // Mark the thread as running
  m_running = true;
  srslte::byte_buffer_t *sgi_msg, *s1u_msg, *s11_msg;
  s1u_msg = m_pool->allocate("spgw::run_thread::s1u");
  s11_msg = m_pool->allocate("spgw::run_thread::s11");

  struct sockaddr_in src_addr_in;
  struct sockaddr_un src_addr_un;
  socklen_t       addrlen;
  struct iphdr*   ip_pkt;

  int sgi = m_gtpu->get_sgi();
  int s1u = m_gtpu->get_s1u();
  int s11 = m_gtpc->get_s11();

  size_t buf_len = SRSLTE_MAX_BUFFER_SIZE_BYTES - SRSLTE_BUFFER_HEADER_OFFSET;

  fd_set set;
  int    max_fd = std::max(s1u, sgi);
  max_fd        = std::max(max_fd, s11);
  while (m_running) {

    s1u_msg->reset();
    s11_msg->reset();

    FD_ZERO(&set);
    FD_SET(s1u, &set);
    FD_SET(sgi, &set);
    FD_SET(s11, &set);

    int n = select(max_fd + 1, &set, NULL, NULL, NULL);
    if (n == -1) {
      m_spgw_log->error("Error from select\n");
    } else if (n) {
      if (FD_ISSET(sgi, &set)) {
        /*
         * SGi messages may need to be queued when waiting for UE Paging procedure.
         * For this reason, buffers for SGi pdus are allocated here and deallocated
         * at the gtpu::send_s1u_pdu() when the PDU is sent, at handle_sgi_pdu() when the PDU is dropped or at
         * gtpc::free_all_queued_packets, which is called when the Downlink Data Notification
         * procedure fails (see handle_downlink_data_notification_acknowledgment and
         * handle_downlink_data_notification_failure)
         */
        m_spgw_log->debug("Message received at SPGW: SGi Message\n");
        sgi_msg      = m_pool->allocate("spgw::run_thread::sgi_msg");
        sgi_msg->N_bytes = read(sgi, sgi_msg->msg, buf_len);
        m_gtpu->handle_sgi_pdu(sgi_msg);
      }
      if (FD_ISSET(s1u, &set)) {
        m_spgw_log->debug("Message received at SPGW: S1-U Message\n");
        s1u_msg->N_bytes = recvfrom(s1u, s1u_msg->msg, buf_len, 0, (struct sockaddr*)&src_addr_in, &addrlen);
        m_gtpu->handle_s1u_pdu(s1u_msg);
      }
      if (FD_ISSET(s11, &set)) {
        m_spgw_log->debug("Message received at SPGW: S11 Message\n");
        s11_msg->N_bytes = recvfrom(s11, s11_msg->msg, buf_len, 0, (struct sockaddr*)&src_addr_un, &addrlen);
        m_gtpc->handle_s11_pdu(s11_msg);
      }
    } else {
      m_spgw_log->debug("No data from select.\n");
    }
  }
  m_pool->deallocate(s1u_msg);
  m_pool->deallocate(s11_msg);
  return;
}

void
spgw::handle_sgi_pdu(srslte::byte_buffer_t *msg)
{
  uint8_t version=0;
  uint32_t dest_ip;
  struct in_addr dest_addr;
  std::map<uint32_t,srslte::gtpc_f_teid_ie>::iterator gtp_fteid_it;
  bool ip_found = false;
  srslte::gtpc_f_teid_ie enb_fteid;

  struct iphdr *iph = (struct iphdr *) msg->msg;
  if (iph->version != 4) {
    m_spgw_log->warning("IPv6 not supported yet.\n");
    return;
  }
  if (iph->tot_len < 20) {
    m_spgw_log->warning("Invalid IP header length.\n");
    return;
  }

  pthread_mutex_lock(&m_mutex);
  gtp_fteid_it = m_ip_to_teid.find(iph->daddr);
  if (gtp_fteid_it != m_ip_to_teid.end()) {
    ip_found = true;
    enb_fteid = gtp_fteid_it->second;
  }
  pthread_mutex_unlock(&m_mutex);

  if (ip_found == false) {
    m_spgw_log->info("IP Packet dst %s is not for any UE\n", inet_ntoa(*(struct in_addr*)(&iph->daddr)));
#ifdef PHY_ADAPTER_ENABLE
    EPCSTATS::updateDstNotFound(iph->saddr, iph->daddr, msg->N_bytes);
#endif
    return;
  }
#ifdef PHY_ADAPTER_ENABLE
    EPCSTATS::updateDownlinkTraffic(iph->saddr, iph->daddr, msg->N_bytes);
#endif
  struct sockaddr_in enb_addr;
  enb_addr.sin_family = AF_INET;
  enb_addr.sin_port = htons(GTPU_RX_PORT);
  enb_addr.sin_addr.s_addr = enb_fteid.ipv4;

  //Setup GTP-U header
  srslte::gtpu_header_t header;
  header.flags        = GTPU_FLAGS_VERSION_V1 | GTPU_FLAGS_GTP_PROTOCOL;
  header.message_type = GTPU_MSG_DATA_PDU;
  header.length       = msg->N_bytes;
  header.teid         = enb_fteid.teid;

  //Write header into packet
  if (!srslte::gtpu_write_header(&header, msg, m_spgw_log)) {
    m_spgw_log->console("Error writing GTP-U header on PDU\n");
  }


  //Send packet to destination
  int n = sendto(m_s1u,msg->msg,msg->N_bytes,0,(struct sockaddr*) &enb_addr,sizeof(enb_addr));
  if (n<0) {
    m_spgw_log->error("Error sending packet to eNB\n");
    return;
  } else if((unsigned int) n!=msg->N_bytes) {
    m_spgw_log->error("Mis-match between packet bytes and sent bytes: Sent: %d, Packet: %d \n",n,msg->N_bytes);
  }

  return;
}


void
spgw::handle_s1u_pdu(srslte::byte_buffer_t *msg)
{
  //m_spgw_log->console("Received PDU from S1-U. Bytes=%d\n",msg->N_bytes);
  srslte::gtpu_header_t header;
  srslte::gtpu_read_header(msg, &header, m_spgw_log);

  //m_spgw_log->console("TEID 0x%x. Bytes=%d\n", header.teid, msg->N_bytes);
  int n = write(m_sgi_if, msg->msg, msg->N_bytes);
  if (n<0) {
    m_spgw_log->error("Could not write to TUN interface.\n");
  } else {
#ifdef PHY_ADAPTER_ENABLE
    struct iphdr *iph = (struct iphdr *) msg->msg;
    EPCSTATS::updateUplinkTraffic(iph->saddr, iph->daddr, msg->N_bytes);
#endif
    //m_spgw_log->console("Forwarded packet to TUN interface. Bytes= %d/%d\n", n, msg->N_bytes);
  }
  return;
}

/*
 * Helper Functions
 */
uint64_t
spgw::get_new_ctrl_teid()
{
  return m_next_ctrl_teid++;
}

uint64_t
spgw::get_new_user_teid()
{
  return m_next_user_teid++;
}

in_addr_t
spgw::get_new_ue_ipv4()
{
  m_h_next_ue_ip++;
  return ntohl(m_h_next_ue_ip);//FIXME Tmp hack
}

spgw_tunnel_ctx_t*
spgw::create_gtp_ctx(struct srslte::gtpc_create_session_request *cs_req)
{
  //Setup uplink control TEID
  uint64_t spgw_uplink_ctrl_teid = get_new_ctrl_teid();
  //Setup uplink user TEID
  uint64_t spgw_uplink_user_teid = get_new_user_teid();
  //Allocate UE IP
  in_addr_t ue_ip = get_new_ue_ipv4();
  //in_addr_t ue_ip = inet_addr("172.16.0.2");
  uint8_t default_bearer_id = 5;

  m_spgw_log->console("SPGW: Allocated Ctrl TEID %" PRIu64 "\n", spgw_uplink_ctrl_teid);
  m_spgw_log->console("SPGW: Allocated User TEID %" PRIu64 "\n", spgw_uplink_user_teid);
  struct in_addr ue_ip_;
  ue_ip_.s_addr=ue_ip;
  m_spgw_log->console("SPGW: Allocate UE IP %s\n", inet_ntoa(ue_ip_));


  //Save the UE IP to User TEID map 
  spgw_tunnel_ctx_t *tunnel_ctx = new spgw_tunnel_ctx_t;
  bzero(tunnel_ctx,sizeof(spgw_tunnel_ctx_t));

  tunnel_ctx->imsi = cs_req->imsi;
  tunnel_ctx->ebi = default_bearer_id;
  tunnel_ctx->up_user_fteid.teid = spgw_uplink_user_teid;
  tunnel_ctx->up_user_fteid.ipv4 = m_s1u_addr.sin_addr.s_addr;
  tunnel_ctx->dw_ctrl_fteid.teid = cs_req->sender_f_teid.teid;
  tunnel_ctx->dw_ctrl_fteid.ipv4 = cs_req->sender_f_teid.ipv4;

  tunnel_ctx->up_ctrl_fteid.teid = spgw_uplink_ctrl_teid;
  tunnel_ctx->ue_ipv4 = ue_ip;
  m_teid_to_tunnel_ctx.insert(std::pair<uint32_t,spgw_tunnel_ctx_t*>(spgw_uplink_ctrl_teid,tunnel_ctx));
  m_imsi_to_ctr_teid.insert(std::pair<uint64_t,uint32_t>(cs_req->imsi,spgw_uplink_ctrl_teid));
  return tunnel_ctx; 
}

bool
spgw::delete_gtp_ctx(uint32_t ctrl_teid)
{
  spgw_tunnel_ctx_t *tunnel_ctx;
  if (!m_teid_to_tunnel_ctx.count(ctrl_teid)) {
    m_spgw_log->error("Could not find GTP context to delete.\n");
    return false;
  }
  tunnel_ctx = m_teid_to_tunnel_ctx[ctrl_teid];

  //Remove GTP-U connections, if any.
  if (m_ip_to_teid.count(tunnel_ctx->ue_ipv4)) {
    pthread_mutex_lock(&m_mutex);
    m_ip_to_teid.erase(tunnel_ctx->ue_ipv4);
#ifdef PHY_ADAPTER_ENABLE
    EPCSTATS::delBearer(tunnel_ctx->ue_ipv4);
#endif
    pthread_mutex_unlock(&m_mutex);
  }

  //Remove Ctrl TEID from IMSI to control TEID map
  m_imsi_to_ctr_teid.erase(tunnel_ctx->imsi);

  //Remove GTP context from control TEID mapping
  m_teid_to_tunnel_ctx.erase(ctrl_teid);
  delete tunnel_ctx; 
  return true;
}

void
spgw::handle_create_session_request(struct srslte::gtpc_create_session_request *cs_req, struct srslte::gtpc_pdu *cs_resp_pdu)
{
  m_spgw_log->info("Received Create Session Request\n");
  spgw_tunnel_ctx_t *tunnel_ctx;
  int default_bearer_id = 5;
  //Check if IMSI has active GTP-C and/or GTP-U
  bool gtpc_present = m_imsi_to_ctr_teid.count(cs_req->imsi);
  if (gtpc_present) {
    m_spgw_log->console("SPGW: GTP-C context for IMSI %015" PRIu64 " already exists.\n", cs_req->imsi);
    delete_gtp_ctx(m_imsi_to_ctr_teid[cs_req->imsi]);
    m_spgw_log->console("SPGW: Deleted previous context.\n");
  }

  m_spgw_log->info("Creating new GTP-C context\n");
  tunnel_ctx = create_gtp_ctx(cs_req);

  //Create session response message
  srslte::gtpc_header *header = &cs_resp_pdu->header;
  srslte::gtpc_create_session_response *cs_resp = &cs_resp_pdu->choice.create_session_response;

  //Setup GTP-C header
  header->piggyback = false;
  header->teid_present = true;
  header->teid = tunnel_ctx->dw_ctrl_fteid.teid;  //Send create session requesponse to the UE's MME Ctrl TEID
  header->type = srslte::GTPC_MSG_TYPE_CREATE_SESSION_RESPONSE;

  //Initialize to zero
  bzero(cs_resp,sizeof(struct srslte::gtpc_create_session_response));
  //Setup Cause
  cs_resp->cause.cause_value = srslte::GTPC_CAUSE_VALUE_REQUEST_ACCEPTED;
  //Setup sender F-TEID (ctrl)
  cs_resp->sender_f_teid.ipv4_present = true;
  cs_resp->sender_f_teid = tunnel_ctx->up_ctrl_fteid;

  //Bearer context created
  cs_resp->eps_bearer_context_created.ebi = default_bearer_id;
  cs_resp->eps_bearer_context_created.cause.cause_value = srslte::GTPC_CAUSE_VALUE_REQUEST_ACCEPTED;
  cs_resp->eps_bearer_context_created.s1_u_sgw_f_teid_present=true;
  cs_resp->eps_bearer_context_created.s1_u_sgw_f_teid =  tunnel_ctx->up_user_fteid;

  //Fill in the PAA
  cs_resp->paa_present = true;
  cs_resp->paa.pdn_type = srslte::GTPC_PDN_TYPE_IPV4;
  cs_resp->paa.ipv4_present = true;
  cs_resp->paa.ipv4 = tunnel_ctx->ue_ipv4;
  m_spgw_log->info("Sending Create Session Response\n");
  m_mme_gtpc->handle_create_session_response(cs_resp_pdu);
  return;
}

void
spgw::handle_modify_bearer_request(struct srslte::gtpc_pdu *mb_req_pdu, struct srslte::gtpc_pdu *mb_resp_pdu)
{
  m_spgw_log->info("Received Modified Bearer Request\n");

  //Get control tunnel info from mb_req PDU
  uint32_t ctrl_teid = mb_req_pdu->header.teid;
  std::map<uint32_t,spgw_tunnel_ctx_t*>::iterator tunnel_it = m_teid_to_tunnel_ctx.find(ctrl_teid);
  if (tunnel_it == m_teid_to_tunnel_ctx.end()) {
    m_spgw_log->warning("Could not find TEID %d to modify\n",ctrl_teid);
    return;
  }
  spgw_tunnel_ctx_t *tunnel_ctx = tunnel_it->second;

  m_spgw_log->info("%s XXX found ctx, ctrl_teid 0x%x, header.teid 0x%lx, imsi 0x%lx, ipv4 %s, ebi 0x%hhx\n",
                   __func__,
                   ctrl_teid, 
                   mb_req_pdu->header.teid,
                   tunnel_ctx->imsi,
                   inet_ntoa(*(struct in_addr*) &tunnel_ctx->ue_ipv4),
                   tunnel_ctx->ebi);

  //Store user DW link TEID
  srslte::gtpc_modify_bearer_request *mb_req = &mb_req_pdu->choice.modify_bearer_request;
  tunnel_ctx->dw_user_fteid.teid = mb_req->eps_bearer_context_to_modify.s1_u_enb_f_teid.teid;
  tunnel_ctx->dw_user_fteid.ipv4 = mb_req->eps_bearer_context_to_modify.s1_u_enb_f_teid.ipv4;
  //Set up actual tunnel
  m_spgw_log->info("Setting Up GTP-U tunnel. Tunnel info: \n");
  struct in_addr addr;
  addr.s_addr = tunnel_ctx->ue_ipv4;
  m_spgw_log->info("IMSI: %lu, UE IP, %s \n",tunnel_ctx->imsi, inet_ntoa(addr));
  m_spgw_log->info("S-GW Rx Ctrl TEID 0x%x, MME Rx Ctrl TEID 0x%x\n", tunnel_ctx->up_ctrl_fteid.teid, tunnel_ctx->dw_ctrl_fteid.teid);
  m_spgw_log->info("S-GW Rx Ctrl IP (NA), MME Rx Ctrl IP (NA)\n");

  struct in_addr addr2;
  addr2.s_addr = tunnel_ctx->up_user_fteid.ipv4;
  m_spgw_log->info("S-GW Rx User TEID 0x%x, S-GW Rx User IP %s\n", tunnel_ctx->up_user_fteid.teid, inet_ntoa(addr2));

  struct in_addr addr3;
  addr3.s_addr = tunnel_ctx->dw_user_fteid.ipv4;
  m_spgw_log->info("eNB Rx User TEID 0x%x, eNB Rx User IP %s\n", tunnel_ctx->dw_user_fteid.teid, inet_ntoa(addr3));

  //Setup IP to F-TEID map
  //bool ret = false;
  pthread_mutex_lock(&m_mutex);
  m_ip_to_teid[tunnel_ctx->ue_ipv4]=tunnel_ctx->dw_user_fteid;
  //ret = m_ip_to_teid.insert(std::pair<uint32_t,srslte::gtpc_f_teid_ie>(tunnel_ctx->ue_ipv4, tunnel_ctx->dw_user_fteid));
#ifdef PHY_ADAPTER_ENABLE
  EPCSTATS::addBearer(tunnel_ctx->ue_ipv4, 
                      tunnel_ctx->dw_user_fteid.teid, 
                      tunnel_ctx->dw_user_fteid.ipv4,
                      tunnel_ctx->imsi,
                      tunnel_ctx->ebi);
#endif
  pthread_mutex_unlock(&m_mutex);

  //Setting up Modify bearer response PDU
  //Header
  srslte::gtpc_header *header = &mb_resp_pdu->header;
  header->piggyback = false;
  header->teid_present = true;
  header->teid = tunnel_ctx->dw_ctrl_fteid.teid;  //
  header->type = srslte::GTPC_MSG_TYPE_MODIFY_BEARER_RESPONSE;

  //PDU
  srslte::gtpc_modify_bearer_response *mb_resp = &mb_resp_pdu->choice.modify_bearer_response;
  mb_resp->cause.cause_value = srslte::GTPC_CAUSE_VALUE_REQUEST_ACCEPTED;
  mb_resp->eps_bearer_context_modified.ebi = tunnel_ctx->ebi;
  mb_resp->eps_bearer_context_modified.cause.cause_value = srslte::GTPC_CAUSE_VALUE_REQUEST_ACCEPTED;

  return;
}

void
spgw::handle_delete_session_request(struct srslte::gtpc_pdu *del_req_pdu, struct srslte::gtpc_pdu *del_resp_pdu)
{
  //Find tunel ctxt
  uint32_t ctrl_teid = del_req_pdu->header.teid;
  std::map<uint32_t,spgw_tunnel_ctx_t*>::iterator tunnel_it = m_teid_to_tunnel_ctx.find(ctrl_teid);
  if (tunnel_it == m_teid_to_tunnel_ctx.end()) {
    m_spgw_log->warning("Could not find TEID %d to delete\n",ctrl_teid);
    return;
  }
  spgw_tunnel_ctx_t *tunnel_ctx = tunnel_it->second;
  in_addr_t ue_ipv4 = tunnel_ctx->ue_ipv4;

  //Delete data tunnel
  pthread_mutex_lock(&m_mutex);
  std::map<in_addr_t,srslte::gtp_fteid_t>::iterator data_it = m_ip_to_teid.find(tunnel_ctx->ue_ipv4);
  if (data_it != m_ip_to_teid.end()) {
    m_ip_to_teid.erase(data_it);
#ifdef PHY_ADAPTER_ENABLE
  EPCSTATS::delBearer(tunnel_ctx->ue_ipv4);
#endif
  }
  pthread_mutex_unlock(&m_mutex);
  m_teid_to_tunnel_ctx.erase(tunnel_it);

  delete tunnel_ctx; 
  return;
}

void
spgw::handle_release_access_bearers_request(struct srslte::gtpc_pdu *rel_req_pdu, struct srslte::gtpc_pdu *rel_resp_pdu)
{
  //Find tunel ctxt
  uint32_t ctrl_teid = rel_req_pdu->header.teid;
  std::map<uint32_t,spgw_tunnel_ctx_t*>::iterator tunnel_it = m_teid_to_tunnel_ctx.find(ctrl_teid);
  if (tunnel_it == m_teid_to_tunnel_ctx.end()) {
    m_spgw_log->warning("Could not find TEID %d to release bearers from\n",ctrl_teid);
    return;
  }
  spgw_tunnel_ctx_t *tunnel_ctx = tunnel_it->second;
  in_addr_t ue_ipv4 = tunnel_ctx->ue_ipv4;

  //Delete data tunnel
  pthread_mutex_lock(&m_mutex);
  std::map<in_addr_t,srslte::gtpc_f_teid_ie>::iterator data_it = m_ip_to_teid.find(tunnel_ctx->ue_ipv4);
  if (data_it != m_ip_to_teid.end()) {
    m_ip_to_teid.erase(data_it);
#ifdef PHY_ADAPTER_ENABLE
  EPCSTATS::delBearer(tunnel_ctx->ue_ipv4);
#endif
  }
  pthread_mutex_unlock(&m_mutex);

  //Do NOT delete control tunnel
  return;
}

} //namespace srsepc
