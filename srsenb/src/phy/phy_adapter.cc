/**
 *
 * \section COPYRIGHT
 *
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 *
 * \section LICENSE
 *
 * This file is part of srsLTE.
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

#include "srslte/config.h"

#ifdef PHY_ADAPTER_ENABLE_PENDING

#define Error(fmt, ...)   if (log_h_) log_h_->error  (fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (log_h_) log_h_->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (log_h_) log_h_->info   (fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (log_h_) log_h_->debug  (fmt, ##__VA_ARGS__)
#define Console(fmt, ...) if (log_h_) log_h_->console(fmt, ##__VA_ARGS__)
#define InfoHex(p,l,fmt, ...) if (log_h_) log_h_->info_hex  (p,l, fmt, ##__VA_ARGS__)

extern "C" {
#include "srslte/phy/phch/ra.h"
}

#include "lib/include/srslte/phy/phch/pdsch_cfg.h"
#include "srsenb/hdr/phy/phy_adapter.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"

#include "libemanelte/mhalenb.h"
#include "libemanelte/enbstatisticmanager.h"

#include <vector>
#include <set>


// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
  EMANELTE::MHAL::ENB_DL_Message   enb_dl_msg_;
  EMANELTE::MHAL::TxControlMessage tx_control_;
  EMANELTE::MHAL::DownlinkMessage * downlink_control_message_;

  typedef std::pair<EMANELTE::MHAL::UE_UL_Message, EMANELTE::MHAL::RxControl> UE_UL_Message;
  typedef std::vector<UE_UL_Message> UE_UL_Messages;

  UE_UL_Messages ue_ul_msgs_;

  uint64_t tx_seqnum_ = 0;
  uint8_t my_pci_     = 0;
  uint32_t curr_tti_  = 0;
  uint32_t tti_tx_    = 0;

  // referenceSignalPower as set by sib.conf sib2.rr_config_common_sib.pdsch_cnfg.rs_power
  float pdsch_rs_power_milliwatt = 0.0;

  // scaling between pdsch res in symbols with reference signals to symbols without reference signals
  float pdsch_rho_b_over_rho_a = 1.0;

  // scaling between reference signal res and pdsch res in symbols without reference signals, by tti and rnti
  typedef std::map<uint16_t, float> rho_a_db_map_t; // map of rnti to rho_a
  rho_a_db_map_t rho_a_db_map[10];                     // vector of rho_a maps by subframe number

  // cyclic prefix normal or extended for this cell
  srslte_cp_t cell_cp = SRSLTE_CP_NORM;

  inline bool rnti_is_user_i(uint32_t rnti)
   {
     return(rnti == SRSLTE_SIRNTI || 
            rnti == SRSLTE_PRNTI  || 
           (rnti >= SRSLTE_RARNTI_START && rnti <= SRSLTE_RARNTI_END));
   }

  pthread_mutex_t dl_mutex_;
  pthread_mutex_t ul_mutex_;

  srslte::log *log_h_ = NULL;

  const uint8_t zeros_[0xffff] = {0};

  inline std::string GetDebugString(const std::string & str)
   {
#ifdef ENABLE_DEBUG_STRING
       return str;
#else
       return "";
#endif
   }


  void
  initDownlinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channel_message,
                             EMANELTE::MHAL::CHANNEL_TYPE ctype,
                             EMANELTE::MHAL::MOD_TYPE modType,
                             uint32_t infoBits,
                             float txPowerScaledB=0.0)
  {
    channel_message->set_channel_type(ctype);

    channel_message->set_modulation_type(modType);

    channel_message->set_number_of_bits(infoBits);

    channel_message->set_tx_power_scale_db(txPowerScaledB);
  }
}


namespace srsenb {
namespace phy_adapter {

static EMANELTE::MHAL::DCI_FORMAT convert(srslte_dci_format_t format)
 {
   switch(format)
    {
      case SRSLTE_DCI_FORMAT0:
        return(EMANELTE::MHAL::DCI_FORMAT_0);

      case SRSLTE_DCI_FORMAT1:
        return(EMANELTE::MHAL::DCI_FORMAT_1);

      case SRSLTE_DCI_FORMAT1A:
        return(EMANELTE::MHAL::DCI_FORMAT_1A);

      case SRSLTE_DCI_FORMAT1C:
        return(EMANELTE::MHAL::DCI_FORMAT_1C);

      case SRSLTE_DCI_FORMAT1B:
        return(EMANELTE::MHAL::DCI_FORMAT_1B);

      case SRSLTE_DCI_FORMAT1D:
        return(EMANELTE::MHAL::DCI_FORMAT_1D);

      case SRSLTE_DCI_FORMAT2:
        return(EMANELTE::MHAL::DCI_FORMAT_2);

      case SRSLTE_DCI_FORMAT2A:
        return(EMANELTE::MHAL::DCI_FORMAT_2A);

      case SRSLTE_DCI_FORMAT2B:
        return(EMANELTE::MHAL::DCI_FORMAT_2B);

      default:
       throw("PHY_ADPT:convert: invalid dci format");

      return (EMANELTE::MHAL::DCI_FORMAT_ERR);
   }
}

static EMANELTE::MHAL::MOD_TYPE convert(srslte_mod_t type)
{
   switch(type)
    {
       case SRSLTE_MOD_BPSK: 
         return (EMANELTE::MHAL::MOD_BPSK);

       case SRSLTE_MOD_QPSK:
         return (EMANELTE::MHAL::MOD_QPSK);

       case SRSLTE_MOD_16QAM:
         return (EMANELTE::MHAL::MOD_16QAM);

       case SRSLTE_MOD_64QAM:
         return (EMANELTE::MHAL::MOD_64QAM);

       default:
         throw("PHY_ADPT:convert: invalid mod type");

       return (EMANELTE::MHAL::MOD_ERR);
    }
}


// set pdcch and pdsch dl
static void enb_dl_put_dl_grant_i(const srslte_enb_dl_pdsch_t * grant,
                                  const srslte_dci_msg_t * dci,
                                  const srslte_ra_dl_grant_t * phy_grant,
                                  const srslte_enb_dl_t * enb_dl,
                                  int tb,
                                  uint32_t refid)
 {
   const uint32_t sf_idx = (tti_tx_ % 10);

   // pdcch
   EMANELTE::MHAL::ENB_DL_Message_PDCCH * pdcch = enb_dl_msg_.add_pdcch();

   EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->add_pdcch();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PDCCH,
                              EMANELTE::MHAL::MOD_QPSK,
                              dci->nof_bits);

   channel_message->set_rnti(grant->rnti);

   const srslte_pdcch_t * ppdcch = &enb_dl->pdcch;
   const srslte_regs_t * h = ppdcch->regs;

   uint32_t start_reg = grant->location.ncce * 9;
   //uint32_t nof_regs = PDCCH_FORMAT_NOF_REGS(grant->location.L);
   uint32_t nof_regs = (1<<grant->location.L)*9;

  for (uint32_t i=start_reg;i<start_reg+nof_regs;i++) {
    srslte_regs_reg_t *reg = h->pdcch[enb_dl->cfi-1].regs[i];

    uint32_t k0 = reg->k0;
    uint32_t l = reg->l;
    const uint32_t * k = &reg->k[0];

    uint32_t rb = k0 / 12;

    Debug("PDCCH DL DCI group sf_idx=%d, reg=%d, rnti=%d placement: "
          "(l=%u, "
          "k0=%u, "
          "k[0]=%u "
          "k[1]=%u "
          "k[2]=%u "
          "k[3]=%u) in rb=%u\n", sf_idx, i, grant->rnti, l, k0, k[0], k[1], k[2], k[3], rb);

    channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
  }

   // dci
   EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI * dl_dci = pdcch->mutable_dl_dci();

   dl_dci->set_rnti(grant->rnti);

   dl_dci->set_refid(refid);

   // dci msg
   EMANELTE::MHAL::ENB_DL_Message_DCI_MSG * dl_dci_msg = dl_dci->mutable_dci_msg();

   dl_dci_msg->set_num_bits(dci->nof_bits);

   dl_dci_msg->set_l_level(grant->location.L);

   dl_dci_msg->set_l_ncce(grant->location.ncce);

   dl_dci_msg->set_data(dci->data, dci->nof_bits);

   dl_dci_msg->set_format(convert(grant->dci_format));

   // pdsch
   float rho_a_db = 0.0;

   rho_a_db_map_t::iterator riter = rho_a_db_map[sf_idx].find(grant->rnti);
   if(riter != rho_a_db_map[sf_idx].end())
     {
       rho_a_db = riter->second;
     }

   Debug("PHY_ADPT:enb_dl_put_dl_grant_i "
         "sf_idx %d, "
         "rnti %d, "
         "rho_a_db %0.2f\n",
         sf_idx,
         grant->rnti,
         rho_a_db);

   channel_message = downlink_control_message_->add_pdsch();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PDSCH,
                              convert(phy_grant->mcs[0].mod),
                              phy_grant->mcs[0].tbs,
                              rho_a_db);

   channel_message->set_rnti(grant->rnti);

   // Add resource block assignment from the phy_grant
   for(uint32_t i=0; i<enb_dl->cell.nof_prb; ++i)
     {
       if(phy_grant->prb_idx[0][i])
         {
           channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(i));
         }

       if(phy_grant->prb_idx[1][i])
         {
           channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(i));
         }
     }

   EMANELTE::MHAL::ENB_DL_Message_PDSCH * pdsch = enb_dl_msg_.mutable_pdsch();

   // pdsch data
   EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data* dl_data = pdsch->add_data();

   // data len is in transfer blocks (bits)
   const uint32_t len = phy_grant->mcs[tb].tbs/8;

   dl_data->set_refid(refid);

   dl_data->set_tb(tb);

   dl_data->set_tbs(len);

   // data has been verified at mac
   dl_data->set_data(grant->data[tb], len);

   ENBSTATS::putDLGrant(grant->rnti);
 
   InfoHex(grant->data[tb], phy_grant->mcs[tb].tbs/8,
           "PHY_ADPT:enb_dl_put_dl_grant: tb[%d], rnti 0x%x, refid %u, len %u"
           "\n\t\t\t dci_msg %s"
           "\n\t\t\t phy_grant %s\n",
           tb,
           grant->rnti,
           refid,
           len, 
           dci_msg_t_to_string(dci).c_str(),
           ra_dl_grant_t_to_string(phy_grant).c_str());
}


void enb_initialize(srslte::log * log_h, uint32_t sf_interval_msec, uint32_t physical_cell_id, srslte_cp_t cp, float ul_freq, float dl_freq, int n_prb,  EMANELTE::MHAL::mhal_config_t & mhal_config, rrc_cfg_t * rrc_cfg)
{
  log_h_ = log_h;

  pdsch_rs_power_milliwatt = pow(10.0, static_cast<float>(rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr) / 10.0);
  uint8_t p_b = rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b;

  if(p_b < 4)
    {
      pdsch_rho_b_over_rho_a = pdsch_cfg_cell_specific_ratio_table[0][p_b];
    }

  cell_cp = cp;

  Info("PHY_ADPT:enb_initialize sf_interval "
       "%u msec, "
       "ul_freq %6.4f MHz, "
       "fl_freq %6.4f MHz, "
       "n_prb %d, "
       "rs_power=%d "
       "pdsch_rs_power_milliwatt=%0.2f "
       "p_b=%d "
       "pdsch_rho_b_over_rho_a=%.02f\n",
       sf_interval_msec,
       ul_freq/1e6,
       dl_freq/1e6,
       n_prb,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.ref_sig_pwr,
       pdsch_rs_power_milliwatt,
       rrc_cfg->sibs[1].sib2().rr_cfg_common.pdsch_cfg_common.p_b,
       pdsch_rho_b_over_rho_a);

  EMANELTE::MHAL::ENB::initialize(
     mhal_config,
     EMANELTE::MHAL::ENB::mhal_enb_config_t(physical_cell_id,
                                            sf_interval_msec,
                                            cp == SRSLTE_CP_NORM ? SRSLTE_CP_NORM_NSYMB : SRSLTE_CP_EXT_NSYMB,
                                            ul_freq,
                                            dl_freq,
                                            n_prb,
                                            pdsch_rs_power_milliwatt,
                                            pdsch_rho_b_over_rho_a));
}


void enb_start()
{
  Info("PHY_ADPT:enb_start\n");

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("PHY_ADPT:enb_start pthread_mutexattr_init error %s, exit\n", strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("PHY_ADPT:enb_start pthread_mutexattr_setprotocol error %s, exit\n", strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&dl_mutex_, &mattr) < 0)
       {
         Error("PHY_ADPT:enb_start pthread_mutex_init error %s, exit\n", strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&ul_mutex_, &mattr) < 0)
       {
         Error("PHY_ADPT:enb_start pthread_mutex_init error %s, exit\n", strerror(errno));

         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  enb_dl_msg_.Clear();

  tx_control_.Clear();

  downlink_control_message_ = tx_control_.mutable_downlink();

  downlink_control_message_->Clear();

  EMANELTE::MHAL::ENB::start();
}


void enb_stop()
{
  Info("PHY_ADPT:enb_start\n");

  EMANELTE::MHAL::ENB::stop();

  pthread_mutex_destroy(&dl_mutex_);

  pthread_mutex_destroy(&ul_mutex_);
}



void enb_dl_tx_init(const srslte_enb_dl_t *enb_dl,
                    uint32_t tti_tx,
                    uint32_t cfi)
{
  // lock here, unlocked after tx_end to prevent any worker thread(s)
  // from attempting to start a new tx sequence before the current tx sequence
  // is finished
  pthread_mutex_lock(&dl_mutex_);

  enb_dl_msg_.Clear();

  tx_control_.Clear();

  downlink_control_message_ = tx_control_.mutable_downlink();

  downlink_control_message_->Clear();

  downlink_control_message_->set_num_resource_blocks(enb_dl->cell.nof_prb);

  Info("PHY_ADPT:enb_dl_tx_init: curr_tti %u, tti_tx %u\n", curr_tti_, tti_tx);

  // subframe index
  const uint32_t sf_idx = (tti_tx % 10);

  rho_a_db_map[sf_idx].clear();

  // always set tti for timing tracking
  enb_dl_msg_.set_tti(tti_tx);

  // always set cfi
  // note - cfi should be nof_ctrl_symbols on regular frames and
  //        non_mbsfn_region_length (from sib13) on mbsfn frames
  enb_dl_msg_.set_cfi(cfi);
  downlink_control_message_->set_cfi(enb_dl_msg_.cfi());

  // always set phy_cell_id to distinguish multiple cells
  enb_dl_msg_.set_phy_cell_id(enb_dl->cell.id);
  tx_control_.set_phy_cell_id(enb_dl->cell.id);

  // save our pci
  my_pci_ = enb_dl->cell.id;

  // save the tti_tx
  tti_tx_ = tti_tx;

  // PCFICH encoding
  EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->mutable_pcfich();

  initDownlinkChannelMessage(channel_message,
                             EMANELTE::MHAL::CHAN_PCFICH,
                             EMANELTE::MHAL::MOD_QPSK,
                             2);  // 2 bit to encode dfi

  for(int i=0; i<3; ++i)
    {
      const srslte_pcfich_t * p1 = &enb_dl->pcfich;
      const srslte_regs_t * p2 = p1->regs;
      const srslte_regs_ch_t *rch = &p2->pcfich;
      const srslte_regs_reg_t *reg = rch->regs[i];

      uint32_t k0 = reg->k0;
      uint32_t l = reg->l;
      const uint32_t * k = &reg->k[0];

      //srslte_regs_ch_t * pcfich = &((enb_dl->pcfich.regs)->pcfich);
      uint32_t rb = k0 / 12;
      Debug("PCFICH group i=%d on this subframe placed at resource starting at "
            "(l=%u, "
            "k0=%u, "
            "k[0]=%u "
            "k[1]=%u "
            "k[2]=%u "
            "k[3]=%u) in resource block=%u\n", i, l, k0, k[0], k[1], k[2], k[3], rb);

      channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
    }

  // Set side chain PSS, SSS and MIB information on appropriate subframes
  if(sf_idx == 0 || sf_idx == 5) 
    {
      // physical cell group and id derived from pci

      // set cyclical prefix mode
      enb_dl_msg_.mutable_pss_sss()->set_cp_mode(enb_dl->cell.cp == SRSLTE_CP_NORM ? 
                                                 EMANELTE::MHAL::CP_NORM : 
                                                 EMANELTE::MHAL::CP_EXTD);

      // MIB on first subframe
      if(sf_idx == 0)
       {
         EMANELTE::MHAL::ENB_DL_Message_PBCH * pbch = enb_dl_msg_.mutable_pbch();

         EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->mutable_pbch();

         initDownlinkChannelMessage(channel_message,
                                    EMANELTE::MHAL::CHAN_PBCH,
                                    EMANELTE::MHAL::MOD_QPSK,
                                    40);  // MIB + 16 bit CRC

         // MIB occupies the middle 72 resource elements of the second slot of subframe 0, which
         // is the middle 6 or 7 resource blocks depending on nof_prb being even or odd.
         // Approximate this by sending a segment for each fullly occupied resource block,
         // So 5 blocks when num_prb is odd.
         int first_prb = enb_dl->cell.nof_prb / 2 - 3 + (enb_dl->cell.nof_prb % 2);

         int num_prb = enb_dl->cell.nof_prb % 2 ? 5 : 6;

         for(int i=0; i<num_prb; ++i)
           {
             channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(first_prb + i));
           }

         switch(enb_dl->cell.phich_resources) 
          {
            case SRSLTE_PHICH_R_1_6:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_SIXTH);
            break;

            case SRSLTE_PHICH_R_1_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE_HALF);
            break;

            case SRSLTE_PHICH_R_1:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_ONE);
            break;

            case SRSLTE_PHICH_R_2:
               pbch->set_phich_resources(EMANELTE::MHAL::PR_TWO);
            break;

            default:
             throw("PHY_ADPT:enb_dl_put_base: unhandled cell phich_resources type");
          }

         switch(enb_dl->cell.phich_length) 
          {
            case SRSLTE_PHICH_NORM:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM);
            break;

            case SRSLTE_PHICH_EXT:
               pbch->set_phich_length(EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD);
            break;

            default:
             throw("PHY_ADPT:enb_dl_put_base: unhandled cell phich_length type");
          }

         pbch->set_num_prb(enb_dl->cell.nof_prb);

         pbch->set_num_antennas(enb_dl->cell.nof_ports);
      }
   }
}


// send msg to mhal
bool enb_dl_send_signal(time_t sot_sec, float frac_sec)
{
  bool result = false;

  EMANELTE::MHAL::Data data;
  
  if(enb_dl_msg_.SerializeToString(&data))
    {
      tx_control_.set_reference_signal_power_milliwatt(pdsch_rs_power_milliwatt);

      // align sot to sf time
      const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
      EMANELTE::MHAL::Timestamp * const ts = tx_control_.mutable_sf_time();
      ts->set_ts_sec(tv_sf_time.tv_sec);
      ts->set_ts_usec(tv_sf_time.tv_usec);

      tx_control_.set_message_type(EMANELTE::MHAL::DOWNLINK);
      tx_control_.set_tx_seqnum(tx_seqnum_++);
      tx_control_.set_tti_tx(tti_tx_);

      Info("PHY_ADPT:enb_dl_send_signal tx_ctrl:%s\n \t\tmsg:%s\n",
           GetDebugString(tx_control_.DebugString()).c_str(),
           GetDebugString(enb_dl_msg_.DebugString()).c_str());

      EMANELTE::MHAL::ENB::send_msg(data, tx_control_);
    }
  else
    {
      Error("PHY_ADPT:TX enb_dl_send_signal: SerializeToString ERROR len %zu\n", data.length());
    }

  return result;
}


// tx sequence is done
void enb_dl_tx_end()
{
  pthread_mutex_unlock(&dl_mutex_);
}


// set the power scaling on a per rnti basis
void enb_dl_set_power_allocation(uint32_t tti, uint16_t rnti, float rho_a_db,  float rho_b_db)
{
  const uint32_t sf_idx = (tti % 10);

  rho_a_db_map[sf_idx].insert(std::make_pair<uint16_t, float>(rnti, rho_a_db));

  Debug("PHY_ADPT:enb_dl_set_power_allocation "
        "sf_idx %d, "
        "rnti %d, "
        "rho_a_db %0.2f\n",
        sf_idx,
        rnti,
        rho_a_db);
}


// handles pdcch and pdsch dl
int enb_dl_put_pdcch_dl(srslte_enb_dl_pdsch_t * grant,
                        const srslte_enb_dl_t * enb_dl,
                        uint32_t & refid)
{
  // for each transfer block
  for(int tb = 0; tb < SRSLTE_RA_DL_GRANT_NOF_TB(&grant->grant); ++tb)
    {
      if(grant->data[tb] != NULL)
        {
          srslte_dci_msg_t dci_msg;
          bzero(&dci_msg, sizeof(dci_msg));

          if(srslte_dci_msg_pack_pdsch(&grant->grant,
                                       grant->dci_format,
                                       &dci_msg,
                                       enb_dl->cell.nof_prb,
                                       enb_dl->cell.nof_ports,
                                       !rnti_is_user_i(grant->rnti)) == SRSLTE_SUCCESS)
            {
              srslte_ra_dl_grant_t phy_grant;
              bzero(&phy_grant, sizeof(phy_grant));

              // get phy grant info
              if(srslte_ra_dl_dci_to_grant(&grant->grant,
                                           enb_dl->cell.nof_prb,
                                           grant->rnti,
                                           &phy_grant) == SRSLTE_SUCCESS)
                {
                  // check tb is enabled
                  if(phy_grant.tb_en[tb])
                    {
                      enb_dl_put_dl_grant_i(grant,
                                            &dci_msg,
                                            &phy_grant,
                                            enb_dl,
                                            tb,
                                            ++refid);
                    }
                  else
                    {
                      Info("PHY_ADPT:enb_dl_put_pdcch_dl: skip grant tb[%d], rnti 0x%x"
                           "\n\t\t\t dci_msg %s"
                           "\n\t\t\t phy_grant %s\n",
                           tb,
                           grant->rnti,
                           dci_msg_t_to_string(&dci_msg).c_str(),
                           ra_dl_grant_t_to_string(&phy_grant).c_str());
                    }
                }
            }
          else
            {
              Error("PHY_ADPT:enb_dl_put_pdcch_dl: grant rnti=0x%x error converting dci to grant\n", grant->rnti);
              return SRSLTE_ERROR;
            }
        }
      else
        {
          Info("PHY_ADPT:enb_dl_put_pdcch_dl: skip tb=%d for rnti=0x%x\n", tb, grant->rnti);
        }
    }

  return SRSLTE_SUCCESS;
}


void enb_dl_put_pmch(const srslte_enb_dl_pdsch_t *grant, const srslte_ra_dl_grant_t *phy_grant)
{
   if(grant->rnti == 0)
    {
      return;
    }

   const int tb = 0;

   // ppmch
   EMANELTE::MHAL::ENB_DL_Message_PMCH * pmch = enb_dl_msg_.mutable_pmch();

   // data len is in transfer blocks (bits)
   const uint32_t len = phy_grant->mcs[tb].tbs/8;

   // see srslte_enb_dl_put_pmch() hard coded to 1 as of rev 18.06
   pmch->set_area_id(1); // XXX TODO

   pmch->set_tbs(len);

   pmch->set_rnti(grant->rnti);

   // data has NOT been verified at mac yet, inquire made 7/12/2018 JG
   pmch->set_data(grant->data[tb] ? grant->data[tb] : zeros_, len);

   EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->mutable_pmch();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PMCH,
                              convert(phy_grant->mcs[tb].mod),
                              phy_grant->mcs[tb].tbs);

   // channel_message.add_resource_blocks();
   for(uint32_t rb=0; rb<phy_grant->nof_prb; ++rb)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
     }

   channel_message->set_rnti(grant->rnti);

   ENBSTATS::putDLGrant(grant->rnti);

   Info("PHY_ADPT:enb_dl_put_pmch: tb[%d], rnti 0x%x, len %u"
        "\n\t\t\t phy_grant %s\n",
        tb,
        grant->rnti,
        len,
        ra_dl_grant_t_to_string(phy_grant).c_str());
}


// handles pdcch ul
int enb_dl_put_pdcch_ul(srslte_enb_ul_pusch_t * ul_pusch,
                        const srslte_enb_dl_t * enb_dl)
{
  const uint32_t sf_idx = (tti_tx_ % 10);

  if((ul_pusch->needs_pdcch) && (ul_pusch->rnti > 0))
    {
      srslte_ra_ul_dci_t * ra_ul_dci = &ul_pusch->grant;

      EMANELTE::MHAL::ENB_DL_Message_PDCCH * pdcch = enb_dl_msg_.add_pdcch();

      EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI * ul_dci = pdcch->mutable_ul_dci();

      EMANELTE::MHAL::ENB_DL_Message_DCI_MSG * ul_dci_msg = ul_dci->mutable_dci_msg();

      srslte_dci_msg_t dci_msg;
      bzero(&dci_msg, sizeof(dci_msg));

      if(srslte_dci_msg_pack_pusch(ra_ul_dci, &dci_msg, enb_dl->cell.nof_prb) == SRSLTE_SUCCESS)
        {
          EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->add_pdcch();

          initDownlinkChannelMessage(channel_message,
                                     EMANELTE::MHAL::CHAN_PDCCH,
                                     EMANELTE::MHAL::MOD_QPSK,
                                     dci_msg.nof_bits);

          channel_message->set_rnti(ul_pusch->rnti);

          const srslte_pdcch_t * ppdcch = &enb_dl->pdcch;
          const srslte_regs_t * h = ppdcch->regs;

          uint32_t start_reg = ul_pusch->location.ncce * 9;

          uint32_t nof_regs = (1<<ul_pusch->location.L)*9;

          for (uint32_t i=start_reg;i<start_reg+nof_regs;i++) {
            srslte_regs_reg_t *reg = h->pdcch[enb_dl->cfi-1].regs[i];

            uint32_t k0 = reg->k0;
            uint32_t l = reg->l;
            const uint32_t * k = &reg->k[0];

            uint32_t rb = k0 / 12;

            Debug("PDCCH UL DCI group sf_idx=%d reg=%d rnti=%d placement: "
                  "(l=%u, "
                  "k0=%u, "
                  "k[0]=%u "
                  "k[1]=%u "
                  "k[2]=%u "
                  "k[3]=%u) in rb=%u\n", sf_idx, i, ul_pusch->rnti, l, k0, k[0], k[1], k[2], k[3], rb);

            channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
          }

          ul_dci->set_rnti(ul_pusch->rnti);

          ul_dci_msg->set_num_bits(dci_msg.nof_bits);

          ul_dci_msg->set_l_level(ul_pusch->location.L);

          ul_dci_msg->set_l_ncce(ul_pusch->location.ncce);

          ul_dci_msg->set_data(dci_msg.data, dci_msg.nof_bits);

          ul_dci_msg->set_format(EMANELTE::MHAL::DCI_FORMAT_0);

          Info("PHY_ADPT:enb_dl_put_pdcch_ul:"
               "\n\t\t\t dci_msg %s"
               "\n\t\t\t ul_pusch %s\n",
               dci_msg_t_to_string(&dci_msg).c_str(),
               enb_ul_pusch_t_to_string(ul_pusch).c_str());
        }
      else
        {
          Error("PHY_ADPT:enb_dl_put_pdcch_ul: rnti=%d error packing dci_msg\n", ul_pusch->rnti);
          return SRSLTE_ERROR;
        }
    }

  return SRSLTE_SUCCESS;
}


void enb_dl_put_phich(const srslte_enb_dl_t *enb_dl,
                      const srslte_enb_dl_phich_t * ack,
                      uint32_t n_prb_L,
                      uint32_t n_dmrs)
{
   const uint32_t sf_idx = (tti_tx_ % 10);

   EMANELTE::MHAL::ENB_DL_Message_PHICH * phich = enb_dl_msg_.mutable_phich();

   uint32_t ngroup, nseq;

   srslte_phich_calc(const_cast<srslte_phich_t *>(&enb_dl->phich), n_prb_L, n_dmrs, &ngroup, &nseq);

   phich->set_rnti(ack->rnti);
   phich->set_ack(ack->ack);
   phich->set_num_prb_low(n_prb_L);
   phich->set_num_dmrs(n_dmrs);

   EMANELTE::MHAL::ChannelMessage * channel_message = downlink_control_message_->add_phich();

   initDownlinkChannelMessage(channel_message,
                              EMANELTE::MHAL::CHAN_PHICH,
                              EMANELTE::MHAL::MOD_BPSK,
                              3);  // phich is 000 for nak, 111 for ack. each bit is BPSK modulated to a symbol, and each symbol spread to 4 REs (12 REs total)

   channel_message->set_rnti(ack->rnti);

   srslte_regs_t *h = enb_dl->phich.regs;
   if (SRSLTE_CP_ISEXT(h->cell.cp)) {
     ngroup /= 2;
   }

   srslte_regs_ch_t *rch = &h->phich[ngroup];

   Debug("PHY_ADPT:enb_dl_put_phich msg: ngroup=%d nof_regs=%d\n", ngroup, rch->nof_regs);

   // nof_regs is 3 for phich groups (12 REs total per group).
   // l should always be 0 for Normal PHICH duration and [0,2] for Extended
   for (uint32_t i = 0; i < rch->nof_regs; i++) {
     uint32_t k0 = rch->regs[i]->k0;
     uint32_t rb = k0 / 12;
     uint32_t l = rch->regs[i]->l;

     Debug("enb_dl_put_phich: for rnti=%d, "
           "putting PHICH ack=%d in "
           "l=%u "
           "k0=%u "
           "rb=%u\n", ack->rnti, ack->ack, l, k0, rb);

     channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::ENB::get_tx_prb_frequency(rb));
   }

   Info("PHY_ADPT:enb_dl_put_phich msg:\n%s\n", GetDebugString(phich->DebugString()).c_str());
}




bool enb_ul_get_signal(uint32_t tti, srslte_timestamp_t * ts)
{
  struct timeval tv_sor;

  pthread_mutex_lock(&ul_mutex_);

  curr_tti_ = tti;

  EMANELTE::MHAL::ENB::set_tti(tti);

  for(UE_UL_Messages::iterator ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end(); ++ul_msg)
    {
      ul_msg->second.SINRTester_.release();
    }

  ue_ul_msgs_.clear();

  EMANELTE::MHAL::RxMessages messages;

  const bool bInStep = EMANELTE::MHAL::ENB::get_messages(messages, tv_sor);

  // set rx time 
  ts->full_secs = tv_sor.tv_sec;
  ts->frac_secs = tv_sor.tv_usec/1e6;

  // for each msg rx ota
  for(EMANELTE::MHAL::RxMessages::const_iterator iter = messages.begin(); iter != messages.end(); ++iter)
   {
     EMANELTE::MHAL::UE_UL_Message ue_ul_msg;

     if(ue_ul_msg.ParseFromString(iter->first))
      {
        const EMANELTE::MHAL::RxControl & rx_control = iter->second;

        Info("PHY_ADPT:RX enb_ul_get_signal: %s, sf_time %ld:%06ld, msg:%s\n",
              bInStep ? "in-step" : "late",
              rx_control.rxData_.sf_time_.tv_sec,
              rx_control.rxData_.sf_time_.tv_usec,
              GetDebugString(ue_ul_msg.DebugString()).c_str());

        const uint32_t & pci = ue_ul_msg.transmitter().phy_cell_id();

        // plenty of noise checking todo here
        
        // check ul msg src vs our pci
        if(pci != my_pci_)
         {
           Debug("PHY_ADPT:enb_ul_get_signal: pci 0x%x != my_pci 0x%x, ignore\n", pci, my_pci_);
         }
        else
         {
           ue_ul_msgs_.push_back(UE_UL_Message(ue_ul_msg, rx_control));
         }
      }
    else
      {
        Error("PHY_ADPT:RX enb_ul_get_signal: ParseFromString ERROR\n");
      }
   }

  pthread_mutex_unlock(&ul_mutex_);

  return (! ue_ul_msgs_.empty());
}


int enb_ul_get_prach(uint32_t * indices, float * offsets, float * p2avg, uint32_t max_entries, uint32_t & num_entries)
{
  int result = SRSLTE_SUCCESS;

  num_entries = 0;

  pthread_mutex_lock(&ul_mutex_);

  Debug("PHY_ADPT:RX enb_ul_get_prach: check %zu messages of %u max\n", 
         ue_ul_msgs_.size(), max_entries);

  std::set<uint32_t> unique;

  for(UE_UL_Messages::iterator ul_msg = ue_ul_msgs_.begin(); 
       (ul_msg != ue_ul_msgs_.end()) && (num_entries < max_entries); 
         ++ul_msg)
    {
      if(ul_msg->first.has_prach())
       {
         EMANELTE::MHAL::RxControl & rx_control = ul_msg->second;

         if(!rx_control.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PRACH))
           {
             continue;
           }

         const EMANELTE::MHAL::UE_UL_Message_PRACH & prach = ul_msg->first.prach();

         const EMANELTE::MHAL::UE_UL_Message_PRACH_Preamble & preamble = prach.preamble();

         if(unique.count(preamble.index()) == 0) // unique
           {
             unique.insert(preamble.index());

             indices[num_entries] = preamble.index();

             // timing offset estimation not currently implemented
             offsets[num_entries] = 0.0;
             p2avg[num_entries] = 0.0;

             ++num_entries;

             Info("PHY_ADPT:enb_ul_get_prach: entry[%u], accept index %d\n",
                  num_entries, 
                  preamble.index());
           }
         else
          {
             Info("PHY_ADPT:enb_ul_get_prach: entry[%u], ignore duplicate index %d\n",
                  num_entries, 
                  preamble.index());
          }

         // see srsenb/src/mac/scheduler.cc sched::dl_sched_rar
         Warning("PHY_ADPT:enb_ul_get_prach: XXX only supporting 1 parch/frame\n");

         break;
       }
     else
       {
         Debug("PHY_ADPT:enb_ul_get_prach: no preambles\n");
       }
    }

  pthread_mutex_unlock(&ul_mutex_);

  return result;
}


int enb_ul_get_pucch(srslte_enb_ul_t * q,
                     uint16_t rnti, 
                     srslte_uci_data_t *uci_data)
{
  pthread_mutex_lock(&ul_mutex_);

  Info("PHY_ADPT:RX enb_ul_get_pucch: check %zu messages for rnti %hx\n", ue_ul_msgs_.size(), rnti);

  for(UE_UL_Messages::iterator ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end(); ++ul_msg)
   {
     q->pucch.last_corr = 0.0;
  
     if(ul_msg->first.has_pucch())
      {
        const EMANELTE::MHAL::UE_UL_Message_PUCCH & pucch = ul_msg->first.pucch();

        for(int n = 0; n < pucch.grant_size(); ++n)
         {
           const EMANELTE::MHAL::UE_UL_Message_PUCCH_Grant & grant = pucch.grant(n);

           const uint16_t ul_rnti = grant.rnti();

           // XXX ue crnti might not be set when CR pdu is sent so allow ul_rnti 0
           if(ul_rnti == 0 || ul_rnti == rnti)
            {
              // check sinr
              EMANELTE::MHAL::RxControl & rx_control = ul_msg->second;

              if(rx_control.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PUCCH, ul_rnti))
                {
                  const std::string & uci = grant.uci();

                  memcpy(uci_data, uci.data(), uci.length());

                  q->pucch.last_n_pucch = grant.num_pucch();

                  q->pucch.last_n_prb = grant.num_prb();

                  q->pucch.last_corr = 1.0;

                  Info("PHY_ADPT:enb_ul_get_pucch: grant %d of %d, found pucch_ul_rnti %hx\n",
                       n+1, pucch.grant_size(), rnti);

                  // pass
                  ENBSTATS::getPUCCH(rnti, true);
                }
              else
                {
                  // PUCCH failed sinr, ignore
                  ENBSTATS::getPUCCH(rnti, false);
                }

              pthread_mutex_unlock(&ul_mutex_);

              return SRSLTE_SUCCESS;
            }
         }
      }
   }

  pthread_mutex_unlock(&ul_mutex_);

  return SRSLTE_SUCCESS;
}


int enb_ul_get_pusch(srslte_enb_ul_t * q,
                     srslte_ra_ul_grant_t *ra_ul_grant,
                     uint16_t rnti, 
                     uint32_t rv_idx, 
                     uint32_t current_tx_nb,
                     uint8_t *data, 
                     srslte_cqi_value_t *cqi_value, 
                     srslte_uci_data_t *uci_data, 
                     uint32_t tti)
{
  int result = SRSLTE_ERROR;

  q->pucch.last_corr = 0.0;

  if (q->users[rnti]) {
    if (srslte_pusch_cfg(&q->pusch, 
                         &q->pusch_cfg, 
                         ra_ul_grant, 
                         q->users[rnti]->uci_cfg_en?&q->users[rnti]->uci_cfg:NULL, 
                         &q->hopping_cfg, 
                         q->users[rnti]->srs_cfg_en?&q->users[rnti]->srs_cfg:NULL, 
                         tti, rv_idx, current_tx_nb)) {
      Error("enb_ul_get_pusch: Error configuring PUSCH\n");

      return result;
    }
  } else {
      if (srslte_pusch_cfg(&q->pusch, 
                           &q->pusch_cfg, 
                           ra_ul_grant, 
                           NULL, 
                           &q->hopping_cfg, 
                           NULL, 
                           tti, rv_idx, current_tx_nb)) {
      Error("enb_ul_get_pusch: Error configuring PUSCH\n");

      return result;
    }
  }

  pthread_mutex_lock(&ul_mutex_);

  Info("PHY_ADPT:RX enb_ul_get_pusch: check %zu messages for rnti %hx\n", ue_ul_msgs_.size(), rnti);

  for(UE_UL_Messages::iterator ul_msg = ue_ul_msgs_.begin(); ul_msg != ue_ul_msgs_.end(); ++ul_msg)
   {
     if(ul_msg->first.has_pusch())
      {
        const EMANELTE::MHAL::UE_UL_Message_PUSCH & pusch = ul_msg->first.pusch();

        for(int n = 0; n < pusch.grant_size(); ++n)
         {
           const EMANELTE::MHAL::UE_UL_Message_PUSCH_Grant & grant = pusch.grant(n);

           const uint16_t ul_rnti = grant.rnti();

           if(ul_rnti == rnti)
            {
              // check sinr
              EMANELTE::MHAL::RxControl & rx_control = ul_msg->second;

              if(rx_control.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PUSCH, ul_rnti))
                {
                  const std::string & ul_grant = grant.ul_grant();

                  const std::string & uci = grant.uci();

                  const std::string & payload = grant.payload();

                  memcpy(ra_ul_grant, ul_grant.data(), ul_grant.length());

                  memcpy(uci_data, uci.data(), uci.length());

                  memcpy(data, payload.data(), payload.length());

                  q->pusch.ul_sch.nof_iterations = 1;
 
                  q->pucch.last_corr = 1.0;
 
                  Info("PHY_ADPT:enb_ul_get_pusch: grant %d of %d, found pusch_ul_rnti %hx\n",
                       n+1, pusch.grant_size(), rnti);

                  if(cqi_value)
                    {
                      srslte_cqi_value_unpack(uci_data->uci_cqi, cqi_value);
                    }

                  // pass
                  ENBSTATS::getPUSCH(rnti, true);

                  result = SRSLTE_SUCCESS;
                }
              else
                {
                  // PUSCH failed sinr, ignore
                  ENBSTATS::getPUSCH(rnti, false);

                  result = SRSLTE_ERROR;
                }

              pthread_mutex_unlock(&ul_mutex_);

              // only expect 1 at most
              return result;
            }
           else
            {
              Info("PHY_ADPT:enb_ul_get_pusch: ignore pusch_ul_rnti %hx\n", ul_rnti); 
            }
         }
      }
   }

  Info("PHY_ADPT:RX enb_ul_get_pusch: nothing found for rnti %hx\n", rnti);

  pthread_mutex_unlock(&ul_mutex_);

  return SRSLTE_ERROR;
}

} // end namespace phy_adapter
} // end namespace srsenb

#endif
