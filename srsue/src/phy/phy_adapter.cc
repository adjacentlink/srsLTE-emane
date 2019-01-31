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

#ifdef PHY_ADAPTER_ENABLE

#include "srsue/hdr/phy/phy_adapter.h"
#include "srslte/msg_format_utils.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"
#include "libemanelte/mhalue.h"
#include "libemanelte/uestatisticmanager.h"

#include <vector>
#include <map>
#include <set>

#define Error(fmt, ...)   if (log_h_) log_h_->error  (fmt, ##__VA_ARGS__)
#define Warning(fmt, ...) if (log_h_) log_h_->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)    if (log_h_) log_h_->info   (fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)   if (log_h_) log_h_->debug  (fmt, ##__VA_ARGS__)
#define Console(fmt, ...) if (log_h_) log_h_->console(fmt, ##__VA_ARGS__)
#define InfoHex(p,l,fmt, ...) if (log_h_) log_h_->info_hex  (p,l, fmt, ##__VA_ARGS__)

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
 EMANELTE::MHAL::UE_UL_Message    ue_ul_msg_;
 EMANELTE::MHAL::TxControlMessage tx_control_;
 EMANELTE::MHAL::UplinkMessage * uplink_control_message_;

 // enb dl msg and rx control info
 typedef std::pair<EMANELTE::MHAL::ENB_DL_Message, EMANELTE::MHAL::RxControl> DL_ENB_Signal;

 // vector of dl signals from a specific enb
 typedef std::vector<DL_ENB_Signal> DL_ENB_Signals;

 // map of dl signals to enb(pci)
 typedef std::map<uint16_t, DL_ENB_Signals> DL_Signals;

 EMANELTE::MHAL::ENB_DL_Message            enb_dl_msg_;
 EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data enb_dl_grant_;

 EMANELTE::MHAL::RxControl rx_control_;

 uint64_t tx_seqnum_ = 0;
 uint16_t crnti_     = 0;
 uint32_t earfcn_    = 0;
 uint32_t tti_tx_    = 0;

 srslte::log * log_h_ = NULL;

 pthread_mutex_t ul_mutex_;
 pthread_mutex_t dl_mutex_;

 bool is_valid_n_id_2(int n_id_2)
  {
   return(n_id_2 >= 0 && n_id_2 < 3);
  }

 srslte_dci_format_t get_msg_format(EMANELTE::MHAL::DCI_FORMAT format)
  {
   switch(format)
    {
     case EMANELTE::MHAL::DCI_FORMAT_0:
       return  SRSLTE_DCI_FORMAT0;

     case EMANELTE::MHAL::DCI_FORMAT_1:
       return  SRSLTE_DCI_FORMAT1;
 
     case EMANELTE::MHAL::DCI_FORMAT_1A:
       return  SRSLTE_DCI_FORMAT1A;
 
     case EMANELTE::MHAL::DCI_FORMAT_1B:
       return  SRSLTE_DCI_FORMAT1B;

     case EMANELTE::MHAL::DCI_FORMAT_1C:
       return  SRSLTE_DCI_FORMAT1C;

     case EMANELTE::MHAL::DCI_FORMAT_1D:
       return  SRSLTE_DCI_FORMAT1D;

     case EMANELTE::MHAL::DCI_FORMAT_2:
       return  SRSLTE_DCI_FORMAT2;

     case EMANELTE::MHAL::DCI_FORMAT_2A:
       return  SRSLTE_DCI_FORMAT2A;

     case EMANELTE::MHAL::DCI_FORMAT_2B:
       return  SRSLTE_DCI_FORMAT2B;

     default:
       return SRSLTE_DCI_NOF_FORMATS;
    }
  }

 inline std::string GetDebugString(const std::string & str)
  {
#ifdef ENABLE_DEBUG_STRING
       return str;
#else
       return "";
#endif
  }

  void initUplinkChannelMessage(EMANELTE::MHAL::ChannelMessage * channel_message,
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

namespace srsue {
namespace phy_adapter {

typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI> DL_DCIList;

typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI> UL_DCIList;

typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data> PDSCH_DataList;

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


// get all ota messages (all pci enb dl messages and rx control info)
static DL_Signals ue_dl_get_signals_i(srslte_timestamp_t * ts)
{
  timeval tv_sor;

  EMANELTE::MHAL::RxMessages messages;

  const bool bInStep = EMANELTE::MHAL::UE::get_messages(messages, tv_sor);

  // all signals from all enb(s)
  DL_Signals dl_signals;

  ts->full_secs = tv_sor.tv_sec;
  ts->frac_secs = tv_sor.tv_usec / 1e6;

  // for each message rx ota
  for(EMANELTE::MHAL::RxMessages::const_iterator iter = messages.begin(); iter != messages.end(); ++iter)
   {
     EMANELTE::MHAL::ENB_DL_Message enb_dl_msg;

     if(enb_dl_msg.ParseFromString(iter->first))
      {
       const EMANELTE::MHAL::RxControl & rxControl = iter->second;

       const uint32_t & pci = enb_dl_msg.phy_cell_id();

       Info("PHY_ADPT:RX ue_dl_get_signals_i: %s, rx_seq %lu, pci %u, sf_time %ld:%06ld, msg:%s\n",
            bInStep ? "in-step" : "late",
            rxControl.rxData_.rx_seqnum_,
            pci,
            rxControl.rxData_.sf_time_.tv_sec,
            rxControl.rxData_.sf_time_.tv_usec,
            GetDebugString(enb_dl_msg.DebugString()).c_str());

       DL_Signals::iterator iter = dl_signals.find(pci);

       // existing append
       if(iter != dl_signals.end())
         {
           iter->second.push_back(DL_ENB_Signal(enb_dl_msg, rxControl));
         }
       else
         {
           dl_signals.insert(std::make_pair(pci, DL_ENB_Signals(1, DL_ENB_Signal(enb_dl_msg, rxControl))));
         }
     }
   else
     {
       Error("PHY_ADPT:RX ue_dl_get_signals_i:ParseFromString ERROR\n");
     }
   }

  return (dl_signals);
}


// return signals for a specific pci (enb)
static DL_ENB_Signals ue_dl_enb_subframe_search_i(srslte_ue_sync_t * ue_sync, const uint32_t * tti)
{
   const DL_Signals dl_signals = ue_dl_get_signals_i(&ue_sync->last_timestamp);

   DL_Signals::const_iterator iter = dl_signals.find(ue_sync->cell.id);

   if(iter != dl_signals.end())
    {
      Info("PHY_ADPT:ue_dl_enb_subframe_search_i: sf available sot %ld%0.6f, pci %hu\n",
           ue_sync->last_timestamp.full_secs,
           ue_sync->last_timestamp.frac_secs,
           iter->first);

      const uint32_t sf_idx = tti ? (*tti) % 10 : 0;

      ue_sync->sf_idx        = sf_idx;
      ue_sync->strack.sf_idx = sf_idx;
      ue_sync->sfind.sf_idx  = sf_idx;

      ++ue_sync->nof_recv_sf;

      if(sf_idx == 0)
       {
         ++ue_sync->frame_find_cnt;
         ++ue_sync->frame_ok_cnt;
         ++ue_sync->frame_total_cnt;
       }

      return iter->second;
    }
   else
    {
      Debug("PHY_ADPT:ue_dl_enb_subframe_search_i: nothing found for pci %hu\n", ue_sync->cell.id);

      return DL_ENB_Signals();
    }
}


static UL_DCIList get_ul_dci_list_i(uint16_t rnti)
{
  UL_DCIList dci_list;

  if(enb_dl_msg_.pdcch_size() > 0)
    {
      for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
       {
         const EMANELTE::MHAL::ENB_DL_Message_PDCCH & pdcch = enb_dl_msg_.pdcch(n);

         const EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI & ul_ota_dci = pdcch.ul_dci();

         if(ul_ota_dci.rnti() == rnti)
           {
             if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDCCH, rnti))
               {
                 Info("PHY_ADPT:ue_ul_dci_search: found dci for rnti 0x%hx\n", rnti);

                 dci_list.push_back(ul_ota_dci);
               }
           }
       }
    }

  return dci_list;
}


static DL_DCIList get_dl_dci_list_i(uint16_t rnti)
{
  DL_DCIList dci_list;

  if(enb_dl_msg_.pdcch_size() > 0)
    {
      for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
       {
         const EMANELTE::MHAL::ENB_DL_Message_PDCCH & pdcch = enb_dl_msg_.pdcch(n);

         const EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI & dl_dci = pdcch.dl_dci();

         if(dl_dci.rnti() == rnti)
           {
             if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDCCH, rnti))
               {
                 Info("PHY_ADPT:ue_dl_dci_search: found dci for rnti 0x%hx\n", rnti);

                 dci_list.push_back(dl_dci);
               }
           }
       }
    }

  return dci_list;
}



static PDSCH_DataList ue_dl_get_pdsch_data_list_i(uint32_t refid, uint16_t rnti)
{
  PDSCH_DataList dataList;

  if(enb_dl_msg_.has_pdsch())
    {
      const EMANELTE::MHAL::ENB_DL_Message_PDSCH & pdsch = enb_dl_msg_.pdsch();

      if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDSCH, rnti))
        {
          for(int n = 0; n < pdsch.data().size(); ++n)
            {
              const EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data & data = pdsch.data(n);

              if(data.refid() == refid)
                {
                  Info("PHY_ADPT:ue_dl_get_pdsch_data_list_i: found data for refid %u\n", refid);

                  dataList.push_back(data);
                }
            }
        }
    }

  return dataList;
}


void ue_initialize(srslte::log * log_h, uint32_t sf_interval_msec, EMANELTE::MHAL::mhal_config_t & mhal_config)
{
  log_h_ = log_h;

  Info("PHY_ADPT:ue_initialize sf_interval %u msec\n", sf_interval_msec);

  EMANELTE::MHAL::UE::initialize(sf_interval_msec, mhal_config);
}


void ue_set_frequencies(float ul_freq, float dl_freq, uint32_t earfcn)
{
  Info("PHY_ADPT:ue_set_frequencies ul_freq %6.4f MHz, fl_freq %6.4f MHz, earfcn %u -> %u\n",
       ul_freq/1e6,
       dl_freq/1e6,
       earfcn_,
       earfcn);

  earfcn_ = earfcn;

  EMANELTE::MHAL::UE::set_frequencies(ul_freq, dl_freq);
}


void ue_set_bandwidth(int n_prb)
{
  Info("PHY_ADPT:ue_set_bandwidth n_prb %d\n", 
       n_prb);

  EMANELTE::MHAL::UE::set_num_resource_blocks(n_prb);
}


void ue_start()
{
  Info("PHY_ADPT:ue_start\n");

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("PHY_ADPT:ue_start pthread_mutexattr_init error %s, exit\n", strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("PHY_ADPT:ue_start pthread_mutexattr_setprotocol error %s, exit\n", strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&dl_mutex_, &mattr) < 0)
       {
         Error("PHY_ADPT:ue_start pthread_mutex_init error %s, exit\n", strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&ul_mutex_, &mattr) < 0)
       {
         Error("PHY_ADPT:ue_start pthread_mutex_init error %s, exit\n", strerror(errno));

         exit(1);
       }

     pthread_mutexattr_destroy(&mattr);
  }

  ue_ul_msg_.Clear();

  tx_control_.Clear();

  uplink_control_message_ = tx_control_.mutable_uplink();

  uplink_control_message_->Clear();

  EMANELTE::MHAL::UE::start();
}


void ue_stop()
{
  Info("PHY_ADPT:ue_start\n");

  EMANELTE::MHAL::UE::stop();

  pthread_mutex_destroy(&dl_mutex_);

  pthread_mutex_destroy(&ul_mutex_);
}


void ue_set_crnti(uint16_t crnti)
{
  Info("PHY_ADPT:ue_set_crnti from 0x%hx to 0x%hx\n", crnti_, crnti);

  crnti_ = crnti;

  UESTATS::setCrnti(crnti);
}


// 1 initial state cell search
int ue_dl_cell_search(srslte_ue_cellsearch_t * cs,
                      srslte_ue_cellsearch_result_t * csr,
                      int force_nid_2,
                      uint32_t *max_peak)
{
  // n_id_2's
  std::set<uint32_t> n_id2s;

  UESTATS::Cells cells;

  // num 5ms frames or 5sf
  const uint32_t max_tries = cs->max_frames * 5;

  uint32_t num_pss_sss_found = 0;

  uint32_t try_num = 0;

  EMANELTE::MHAL::UE::begin_cell_search();

  while(try_num++ <= max_tries)
   {
     // cell search is typically done in blocks of 5 sf's
     // we handle the radio recv call here one at a time
     const DL_Signals dl_signals = ue_dl_get_signals_i(&cs->ue_sync.last_timestamp);

     // for each enb
     for(DL_Signals::const_iterator iter = dl_signals.begin(); iter != dl_signals.end(); ++iter)
      {
        const uint32_t pci = iter->first;

        const uint32_t n_id_1 = pci / 3;
 
        const uint32_t n_id_2 = pci % 3;

        Debug("PHY_ADPT:ue_dl_cell_search: try %u/%u, pci %hu, %zu signals\n", 
              try_num, max_tries, pci, iter->second.size());

        // force is enabled, but this cell id group does not match
        if(is_valid_n_id_2(force_nid_2) && n_id_2 != (uint32_t)force_nid_2)
         {
           Info("PHY_ADPT:ue_dl_cell_search: n_id_1 %u, n_id_2 %u != %d, ignore\n",
                n_id_1,
                n_id_2,
                force_nid_2);
 
            continue;
         }

       if(! iter->second.empty())
         {
           float peak_sum = 0.0;

           srslte_cp_t cp = SRSLTE_CP_NORM;

           size_t num_samples = 0;

           // for all gathered signals
           for(size_t n = 0; n < iter->second.size(); ++n)
             {
               const EMANELTE::MHAL::ENB_DL_Message & enb_dl_msg = iter->second[n].first;

               // looking for pss/sss
               if(enb_dl_msg.has_pss_sss())
                {
                  const EMANELTE::MHAL::ENB_DL_Message_PSS_SSS & pss_sss = enb_dl_msg.pss_sss();
 
                  // should all be the same
                  cp = pss_sss.cp_mode() == EMANELTE::MHAL::CP_NORM ? SRSLTE_CP_NORM : SRSLTE_CP_EXT;

                  peak_sum = iter->second[n].second.rxData_.peak_sum_;

                  num_samples = iter->second[n].second.rxData_.num_samples_;

                  ++num_pss_sss_found;

                  Info("PHY_ADPT:ue_dl_cell_search: found pss_sss %s, peak_sum %0.1f, num_samples %u\n",
                       GetDebugString(pss_sss.DebugString()).c_str(),
                       iter->second[n].second.rxData_.peak_sum_,
                       iter->second[n].second.rxData_.num_samples_);
                }
             }

          if(num_samples > 0)
            {
              const float peak_avg = peak_sum / num_samples;

              // save cell info
              cells[pci] = peak_avg;

              // cell id [0,1,2]
              if(n_id2s.insert(n_id_2).second == true)
               {
                 csr[n_id_2].cell_id = pci;
                 csr[n_id_2].cp      = cp;
                 csr[n_id_2].peak    = peak_avg;

                 csr[n_id_2].mode    = 1.0;
                 csr[n_id_2].psr     = 0.0;
                 csr[n_id_2].cfo     = 0.0;

                 Info("PHY_ADPT:ue_dl_cell_search: new PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                      pci,
                      n_id_1,
                      n_id_2,
                      peak_avg);
               }
             else
               {
                // tie goes to the first entry (numeric lowest id)
                if(peak_avg > csr[n_id_2].peak)
                  {
                     Info("PHY_ADPT:ue_dl_cell_search: replace PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                           pci,
                           n_id_1,
                           n_id_2,
                           peak_avg);

                     csr[n_id_2].cell_id = pci;
                     csr[n_id_2].cp      = cp;
                     csr[n_id_2].peak    = peak_avg;
                  }
               }
            }
         }
      }
   }

  cs->ue_sync.pss_stable_cnt = num_pss_sss_found;
  cs->ue_sync.pss_is_stable  = num_pss_sss_found > 0 ? true : false;

  float max_avg = 0.0f;

  // now find best
  for(std::set<uint32_t>::iterator iter = n_id2s.begin(); iter != n_id2s.end(); ++iter)
    {
      if(csr[*iter].peak > max_avg)
        {
          *max_peak = *iter;

          max_avg = csr[*iter].peak;
        }
    }

  Warning("PHY_ADPT:ue_dl_cell_search: sf_idx %u, done, num_cells %zu, max_peak id %u, max_avg %f\n",
       cs->ue_sync.sf_idx,
       n_id2s.size(),
       *max_peak,
       max_avg);

  UESTATS::enterCellSearch(cells, earfcn_);

  return n_id2s.size();
}


// 2 mib search
int ue_dl_mib_search(const srslte_ue_cellsearch_t * cs,
                     srslte_ue_mib_sync_t * ue_mib_sync,
                     srslte_cell_t * cell)
{
  const uint32_t max_tries = 40;

  uint32_t try_num = 0;

  while(try_num++ < max_tries)
    {
      // we handle the radio recv call here one at a time
      const DL_ENB_Signals dl_enb_signals = ue_dl_enb_subframe_search_i(&ue_mib_sync->ue_sync, NULL);

      Debug("PHY_ADPT:ue_dl_mib_search: pci %hu, try %d/%u, %zu signals\n", 
            ue_mib_sync->ue_sync.cell.id, try_num, max_tries, dl_enb_signals.size());

      if(! dl_enb_signals.empty())
        {
          const EMANELTE::MHAL::ENB_DL_Message enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              EMANELTE::MHAL::RxControl rxControl = dl_enb_signals[0].second;

              if(rxControl.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PBCH))
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const EMANELTE::MHAL::ENB_DL_Message_PSS_SSS & pss_sss = enb_dl_msg.pss_sss();

                      const EMANELTE::MHAL::ENB_DL_Message_PBCH & pbch = enb_dl_msg.pbch();

                      Info("PHY_ADPT:ue_dl_mib_search: found pbch %s\n", GetDebugString(pbch.DebugString()).c_str());

                      cell->nof_prb   = pbch.num_prb();
                      cell->nof_ports = pbch.num_antennas();

                      ue_mib_sync->ue_sync.state          = SF_TRACK;
                      ue_mib_sync->ue_sync.pss_stable_cnt = 1;
                      ue_mib_sync->ue_sync.pss_is_stable  = true;

                      switch(pbch.phich_resources())
                        {
                        case EMANELTE::MHAL::PR_ONE_SIXTH:
                          cell->phich_resources = SRSLTE_PHICH_R_1_6;
                          break;

                        case EMANELTE::MHAL::PR_ONE_HALF:
                          cell->phich_resources = SRSLTE_PHICH_R_1_2;
                          break;

                        case EMANELTE::MHAL::PR_ONE:
                          cell->phich_resources = SRSLTE_PHICH_R_1;
                          break;

                        case EMANELTE::MHAL::PR_TWO:
                          cell->phich_resources = SRSLTE_PHICH_R_2;
                          break;
                        }

                      switch(pbch.phich_length())
                        {
                        case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_NORM:
                          cell->phich_length = SRSLTE_PHICH_NORM;
                          break;

                        case EMANELTE::MHAL::ENB_DL_Message_PBCH_PHICH_LENGTH_LEN_EXTD:
                          cell->phich_length = SRSLTE_PHICH_EXT;
                          break;
                        }

                      UESTATS::enterMibSearch(true);

                      return 1;
                }
             }
          }
       }
    }

  UESTATS::enterMibSearch(false);

  return 0;
}


// 3 system frame search
int ue_dl_system_frame_search(srslte_ue_sync_t * ue_sync, uint32_t * sfn)
{
  const uint32_t max_tries = 50;

  uint32_t try_num = 0;

  while(try_num++ < max_tries)
    {
      const DL_ENB_Signals dl_enb_signals = ue_dl_enb_subframe_search_i(ue_sync, NULL);

      Debug("PHY_ADPT:ue_dl_system_frame_search: try %d/%u, %zu signals\n", 
            try_num, max_tries, dl_enb_signals.size());

      if(! dl_enb_signals.empty())
        {
          const EMANELTE::MHAL::ENB_DL_Message enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              EMANELTE::MHAL::RxControl rxControl = dl_enb_signals[0].second;

              // check for PSS SSS if PBCH is good
              if(rxControl.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PBCH))
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const EMANELTE::MHAL::ENB_DL_Message_PSS_SSS & pss_sss = enb_dl_msg.pss_sss();

                      const EMANELTE::MHAL::ENB_DL_Message_PBCH & pbch = enb_dl_msg.pbch();

                      Info("PHY_ADPT:ue_dl_system_frame_search: found pbch %s, try %u/%u\n",
                           GetDebugString(pbch.DebugString()).c_str(), try_num, max_tries);

                      ue_sync->state           = SF_TRACK;
                      ue_sync->pss_stable_cnt  = 1;
                      ue_sync->pss_is_stable   = true;

                      // set system frame number
                      *sfn = enb_dl_msg.tti();
                
                      UESTATS::enterSysFrameSearch(true);

                      return 1;
                    }
                }
            }
        }
    }

  UESTATS::enterSysFrameSearch(false);

  return 0;
}


// 4 called by emu_srsue/src/phy/phch_recv.cc
int ue_dl_sync_search(srslte_ue_sync_t * ue_sync, uint32_t tti)
{
   // set next tx tti
   tti_tx_ = (tti+4)%10240;

   EMANELTE::MHAL::UE::set_tti(tti);

   const DL_ENB_Signals dl_enb_signals = ue_dl_enb_subframe_search_i(ue_sync, &tti);

   enb_dl_msg_.Clear();

   rx_control_.SINRTester_.release();

   enb_dl_grant_.Clear();

   // single antenna mode expect 1 msg for our cell id
   if(dl_enb_signals.size() != 1)
     {
       Info("PHY_ADPT:ue_dl_sync_search: found %zu signals, cell_id %u, expected 1, tti %u\n", 
            dl_enb_signals.size(), ue_sync->cell.id, tti);
 
       UESTATS::enterSyncSearch(false);

       return 0;
     }
   else
     {
       // save enb_dl_msg and rxControl for further processing
       enb_dl_msg_ = dl_enb_signals[0].first;

       rx_control_ = dl_enb_signals[0].second;

       UESTATS::enterSyncSearch(true);

       return 1;
     }
}



float ue_dl_decode_signal(srslte_chest_dl_t * q,
                          uint32_t cell_id,
                          uint32_t &cfi, 
                          uint32_t tti_rx)
{
   float rssi = 0.0;

   // xxx this was added to code base to help with chest
   q->snr = 0.0;

   if(enb_dl_msg_.IsInitialized())
     {
       const uint32_t & pci = enb_dl_msg_.phy_cell_id();

       const uint32_t & dl_tti = enb_dl_msg_.tti();

       // check dl msg src vs our selected cell
       if(pci != cell_id)
         {
           Info("PHY_ADPT:ue_dl_decode_signal: pci 0x%x != cell_id 0x%x, ignore\n", pci, cell_id);
         }
       else
         {
          cfi = enb_dl_msg_.cfi();

          rssi = 3.0;
 
          q->snr = 40;

          Info("PHY_ADPT:ue_dl_decode_signal: dl_tti %u, pci %u, cfi %u, rssi %f, snr %f\n",
                dl_tti, pci, cfi, rssi, q->snr);
        }
     }
   else
     {
       Info("PHY_ADPT:ue_dl_decode_signal: empty msg\n");
     }

   return rssi;
}



int ue_dl_find_dl_dci(srslte_ue_dl_t *q, 
                      uint16_t rnti, 
                      srslte_dci_msg_t *dci_msg)
{
  const DL_DCIList dci_list = get_dl_dci_list_i(rnti);

  int result = 0;

  // expecting 1 dci/rnti
  if(dci_list.size() == 1)
    {
      const EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI & dl_dci = dci_list[0];

      const PDSCH_DataList dl_data = ue_dl_get_pdsch_data_list_i(dl_dci.refid(), rnti);

      // XXX TODO pass/fail
      UESTATS::getPDCCH(rnti, true);

      // expecting 1 pdsch/dci
      if(dl_data.size() == 1)
        {
          // XXX TODO pass/fail
          UESTATS::getPDSCH(rnti, true);

          enb_dl_grant_ = dl_data[0];

          const EMANELTE::MHAL::ENB_DL_Message_DCI_MSG & ota_dl_dci_msg = dl_dci.dci_msg();

          dci_msg->nof_bits = ota_dl_dci_msg.num_bits();

          dci_msg->format = get_msg_format(ota_dl_dci_msg.format());

          memcpy(dci_msg->data, ota_dl_dci_msg.data().data(), ota_dl_dci_msg.data().length());

          const srslte_dci_location_t dci_location = {ota_dl_dci_msg.l_level(), ota_dl_dci_msg.l_ncce()};

          // from lib/src/phy/ue/ue_dl.c dci_blind_search()
          memcpy(&q->last_location, &dci_location, sizeof(srslte_dci_location_t));

          InfoHex((const uint8_t*)ota_dl_dci_msg.data().data(), 
                  ota_dl_dci_msg.data().length(),
                  "PHY_ADPT:ue_dl_find_dl_dci found pdsch for ref id %u, rnti 0x%hx\n", 
                  dl_dci.refid(), rnti);

          result = 1;
       }
      else
       {
         if(dl_data.size() > 1)
          {
            Warning("PHY_ADPT:ue_dl_find_dl_dci found multiple dci_data (%zu) pdcch for rnti 0x%hx\n", 
                            dl_data.size(), rnti);
          }
         else
          {
            Info("PHY_ADPT:ue_dl_find_dl_dci no data for rnti 0x%hx\n", rnti);
          }
       }
    }
   else
    {
      if(dci_list.size() > 1)
       {
         Warning("PHY_ADPT:ue_dl_find_dl_dci_type found multiple dci (%zu) pdcch for rnti 0x%hx\n", 
                         dci_list.size(), rnti);
       }
      else
       {
         Info("PHY_ADPT:ue_dl_find_dl_dci_type no dci for rnti 0x%hx\n", rnti);
       }
    }

  return result;
}


int ue_dl_find_ul_dci(srslte_ue_dl_t *q, 
                      uint16_t rnti, 
                      srslte_dci_msg_t *dci_msg)
{
  int result = 0;

  const UL_DCIList dci_list = get_ul_dci_list_i(rnti);

  // expecting 1 dci/rnti
  if(dci_list.size() == 1)
    {
      const EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI & ul_ota_dci = dci_list[0];

      const EMANELTE::MHAL::ENB_DL_Message_DCI_MSG & ota_ul_dci_msg = ul_ota_dci.dci_msg();

      dci_msg->nof_bits = ota_ul_dci_msg.num_bits();

      dci_msg->format = get_msg_format(ota_ul_dci_msg.format());

      const srslte_dci_location_t dci_location = {ota_ul_dci_msg.l_level(), ota_ul_dci_msg.l_ncce()};

      // from lib/src/phy/ue/ue_dl.c dci_blind_search
      memcpy(&q->last_location_ul, &dci_location, sizeof(srslte_dci_location_t));

      memcpy(dci_msg->data, ota_ul_dci_msg.data().data(), ota_ul_dci_msg.data().length());

      Info("PHY_ADPT:ue_dl_find_ul_dci found pdsch for rnti 0x%hx\n", rnti);

      result = 1;
    }
   else
    {
      if(dci_list.size() > 1)
       {
         Warning("PHY_ADPT:ue_dl_find_ul_dci found multiple (%zu) pdcch for rnti 0x%hx\n", 
                         dci_list.size(), rnti);
       }
      else
       {
         Info("PHY_ADPT:ue_dl_find_ul_dci no pdcch for rnti 0x%hx\n", rnti);
       }
    }

  return result;
}

   
void ue_dl_decode_pdsch(srsue::mac_interface_phy::tb_action_dl_t * dl_action,
                        bool acks[SRSLTE_MAX_CODEWORDS])
{
   const int tb = enb_dl_grant_.tb();

   if(dl_action->decode_enabled[tb])
    {
      if(dl_action->payload_ptr[tb])
        {
          acks[tb] = true;

          memcpy(dl_action->payload_ptr[tb], enb_dl_grant_.data().data(), enb_dl_grant_.data().size());

          InfoHex((const uint8_t *)enb_dl_grant_.data().data(), 
                  enb_dl_grant_.data().size(),
                  "PHY_ADPT:ue_dl_decode_pdsch: rnti 0x%hx, tb[%d], payload len %zu\n",
                  dl_action->rnti, 
                  tb, 
                  enb_dl_grant_.data().size());
        }
      else
        {
          Error("PHY_ADPT:ue_dl_decode_pdsch: tb[%d], no payload_ptr\n", tb);
        } 
    }
   else
    {
      Error("PHY_ADPT:ue_dl_decode_pdsch: tb[%d], decode not enabled\n", tb);
    }
}


bool ue_dl_decode_phich(srslte_ue_dl_t *q, 
                        uint32_t sfn,
                        uint16_t rnti,
                        uint32_t n_prb_L,
                        uint32_t n_dmrs)
{
   if(enb_dl_msg_.has_phich())
     {
       const EMANELTE::MHAL::ENB_DL_Message_PHICH & phich = enb_dl_msg_.phich();

       Debug("PHY_ADPT:ue_dl_decode_phich: sfn %u, rnti 0x%hx, n_prb_L %u, n_dmrs %u, msg:\n%s\n", 
                   sfn,
                   rnti,
                   n_prb_L,
                   n_dmrs,
                   GetDebugString(phich.DebugString()).c_str());

         if(rnti    == phich.rnti()        && 
            n_prb_L == phich.num_prb_low() && 
            n_dmrs  == phich.num_dmrs())
         {
           if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PHICH, rnti))
             {
               // from lib/src/phy/ue/ue_dl.c srslte_ue_dl_decode_phich()
               q->last_phich_corr = 1.0;

               return true;
             }
         }
     }

   return false;
}


bool ue_dl_decode_pmch(srslte_ue_dl_t * q, 
                       uint16_t area_id,
                       uint8_t * payload)
{
  if(enb_dl_msg_.has_pmch())
    {
      const EMANELTE::MHAL::ENB_DL_Message_PMCH & pmch = enb_dl_msg_.pmch();

      if(area_id == pmch.area_id())
       {
         Info("PHY_ADPT:ue_dl_decode_pmch: area_id %u, tbs %u\n",
           pmch.area_id(),
           pmch.tbs());

         if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PMCH))
           {
             memset(payload, 0x0, SRSLTE_MAX_BUFFER_SIZE_BYTES);

             memcpy(payload, pmch.data().data(), pmch.data().length());

             return true;
           }
       }
     else
       {
         Debug("PHY_ADPT:ue_dl_decode_pmch: area_id %u != our area_id %hu, tbs %u\n",
              pmch.area_id(),
              area_id,
              pmch.tbs());

         return false;
       }
    }

  Debug("PHY_ADPT:ue_dl_decode_pmch: no pmch\n");

  return false;
}


// tx init
void ue_ul_tx_init()
{
  // no op may not tx on ever sf so let messages accumulate
}
 

// send to mhal
void ue_ul_send_signal(time_t sot_sec, float frac_sec, const srslte_cell_t & cell)
{
  // end of tx sequence, tx_end will release to lock
  pthread_mutex_lock(&ul_mutex_);

  EMANELTE::MHAL::UE_UL_Message_Transmitter * txinfo = ue_ul_msg_.mutable_transmitter();

  txinfo->set_crnti(crnti_);

  txinfo->set_phy_cell_id(cell.id);
  tx_control_.set_phy_cell_id(cell.id);

  txinfo->set_tti(tti_tx_);

  EMANELTE::MHAL::Data data;

  if(ue_ul_msg_.SerializeToString(&data))
    {
      // align sot to sf time
      const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
      EMANELTE::MHAL::Timestamp * const ts = tx_control_.mutable_sf_time();
      ts->set_ts_sec(tv_sf_time.tv_sec);
      ts->set_ts_usec(tv_sf_time.tv_usec);

      tx_control_.set_message_type(EMANELTE::MHAL::UPLINK);
      tx_control_.set_tx_seqnum(tx_seqnum_++);
      tx_control_.set_tti_tx(tti_tx_);

      Info("PHY_ADPT:ue_ul_send_signal tx_ctrl:%s\n \t\tmsg:%s\n",
           GetDebugString(tx_control_.DebugString()).c_str(),
           GetDebugString(ue_ul_msg_.DebugString()).c_str());

      EMANELTE::MHAL::UE::send_msg(data, tx_control_);
    }
  else
    {
      Error("PHY_ADPT:TX ue_ul_set_signal: SerializeToString ERROR len %zu\n", data.length());
    }
}


// tx sequence ended
void ue_ul_tx_end()
{
  ue_ul_msg_.Clear();

  tx_control_.Clear();

  uplink_control_message_ = tx_control_.mutable_uplink();

  uplink_control_message_->Clear();

  pthread_mutex_unlock(&ul_mutex_);
}


void ue_ul_put_prach(int index, uint32_t prach_freq_offset)
{
   pthread_mutex_lock(&ul_mutex_);

   EMANELTE::MHAL::ChannelMessage * channel_message = uplink_control_message_->mutable_prach();

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PRACH,
                            EMANELTE::MHAL::MOD_BPSK,   // modtype
                            839);                       // PRACH sequence is 839 for formats 0-3 (all allowed by FDD) 

   // The upstream PRACH message is not really a slotted message
   // and can span 2 or 3 subframes. Set slot1 and slot2 resource blocks the same.
   // prach spans the 6 resource blocks starting from prach_freq_offset
   for(int i=0; i<6; ++i)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset + i));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset + i));
     }

   EMANELTE::MHAL::UE_UL_Message_PRACH * prach = ue_ul_msg_.mutable_prach();

   EMANELTE::MHAL::UE_UL_Message_PRACH_Preamble * preamble = prach->mutable_preamble();

   preamble->set_index(index);

   Info("PHY_ADPT:ue_ul_put_prach: index %d, msg:%s\n",
        index,
        GetDebugString(prach->DebugString()).c_str());

   pthread_mutex_unlock(&ul_mutex_);
}


bool ue_ul_put_pucch(srslte_ue_ul_t * q,
                     srslte_uci_data_t * uci_data,
                     uint32_t ncce)
{
   pthread_mutex_lock(&ul_mutex_);

   EMANELTE::MHAL::UE_UL_Message_PUCCH * pucch = ue_ul_msg_.mutable_pucch();

   EMANELTE::MHAL::UE_UL_Message_PUCCH_Grant * grant = pucch->add_grant();

   q->last_pucch_format = srslte_pucch_get_format(uci_data, q->cell.cp);
   
   // from lib/src/phy/ue/ue_ul.c srslte_ue_ul_pucch_encode()
   q->pucch.last_n_pucch = srslte_pucch_get_npucch(ncce, 
                                                   q->last_pucch_format,
                                                   uci_data->scheduling_request,
                                                   &q->pucch_sched);

   // default: SRSLTE_PUCCH_FORMAT_1
   EMANELTE::MHAL::MOD_TYPE modType = EMANELTE::MHAL::MOD_BPSK;
   uint32_t bits = 0;

   switch(q->last_pucch_format)
     {
     case SRSLTE_PUCCH_FORMAT_1:   // 1 HARQ ACK
       bits = 0;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_1A:  // 1 HARQ ACK
       bits = 1;
       modType = EMANELTE::MHAL::MOD_BPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_1B:  // 2 HARQ ACK
       bits = 2;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_2:   // CSI
       bits = 20;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_2A:  // CSI + 1 HARQ ACK
       bits = 21;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_2B:  // CSI + 2 HARQ ACK
       bits = 22;
       modType = EMANELTE::MHAL::MOD_QPSK;
       break;
     case SRSLTE_PUCCH_FORMAT_ERROR:
     default:
       Error("PHY_ADPT:ue_ul_put_pucch: unknown pucch format: %d\n",
             q->last_pucch_format);
     }

   EMANELTE::MHAL::ChannelMessage * channel_message = uplink_control_message_->add_pucch();

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PUCCH,
                            modType,
                            bits);

   channel_message->set_rnti(q->current_rnti);

   // from lib/src/phy/phch/pucch.c  pucch_encode_()
   uint16_t n_prb[2] = {0};

   for(int n = 0; n < 2; ++n)
    {
     n_prb[n] = srslte_pucch_n_prb(&q->pucch.pucch_cfg, 
                                   q->last_pucch_format, 
                                   q->pucch.last_n_pucch,
                                   q->cell.nof_prb,
                                   q->cell.cp, n);
    }

   // flag when resource blocks are different on slot 1 and 2 of the subframe
   channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[0]));
   channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[1]));

   q->pucch.last_n_prb = n_prb[1];

   grant->set_num_pucch(q->pucch.last_n_pucch);

   grant->set_num_prb(n_prb[1]);

   grant->set_rnti(q->current_rnti);

   grant->set_uci(uci_data, sizeof(srslte_uci_data_t));

   Info("PHY_ADPT:ue_ul_put_pucch: ncce %u, format %s, msg: %s\n",
               ncce,
               pucch_format_t_to_name(q->last_pucch_format).c_str(),
               GetDebugString(pucch->DebugString()).c_str());

   pthread_mutex_unlock(&ul_mutex_);

   return SRSLTE_SUCCESS;
}


bool ue_ul_put_pusch(uint16_t rnti,
                     srslte_ra_ul_grant_t *ul_grant,
                     srslte_uci_data_t * uci_data,
                     uint8_t *payload)
{
   pthread_mutex_lock(&ul_mutex_);

   EMANELTE::MHAL::ChannelMessage * channel_message = uplink_control_message_->add_pusch();

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PUSCH,
                            convert(ul_grant->mcs.mod),
                            ul_grant->mcs.tbs);

   channel_message->set_rnti(rnti);

   for (size_t i=0; i<ul_grant->L_prb; ++i)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(ul_grant->n_prb[0] + i));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(ul_grant->n_prb[1] + i));
     }

   EMANELTE::MHAL::UE_UL_Message_PUSCH * pusch = ue_ul_msg_.mutable_pusch();

   EMANELTE::MHAL::UE_UL_Message_PUSCH_Grant * grant = pusch->add_grant();

   grant->set_rnti(rnti);

   grant->set_ul_grant(ul_grant, sizeof(srslte_ra_ul_grant_t));

   grant->set_uci(uci_data, sizeof(srslte_uci_data_t));

   grant->set_payload(payload, ul_grant->mcs.tbs/8);

   Info("PHY_ADPT:ue_ul_put_pusch: %s\n", GetDebugString(pusch->DebugString()).c_str());

   UESTATS::putULGrant(rnti);

   pthread_mutex_unlock(&ul_mutex_);

   return SRSLTE_SUCCESS;
}

} // end namespace phy_adapter
} // end namepsace srsue

#endif
