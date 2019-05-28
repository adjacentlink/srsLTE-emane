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
 EMANELTE::MHAL::UplinkMessage*   uplink_control_message_;

 // enb dl msg and rx control info
 typedef std::pair<EMANELTE::MHAL::ENB_DL_Message, EMANELTE::MHAL::RxControl> DL_ENB_Signal;

 // vector of dl signals from a specific enb
 typedef std::vector<DL_ENB_Signal> DL_ENB_Signals;

 // map of dl signals to enb(pci)
 typedef std::map<uint16_t, DL_ENB_Signals> DL_Signals;

 EMANELTE::MHAL::ENB_DL_Message            enb_dl_msg_;
 EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data enb_dl_grant_;

 EMANELTE::MHAL::RxControl rx_control_;

 uint64_t tx_seqnum_         = 0;
 uint16_t crnti_             = 0;
 uint32_t earfcn_            = 0;
 uint32_t tti_tx_            = 0;
 uint32_t prach_freq_offset_ = 0;

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

  inline int bits_to_bytes(int bits)
    {  return ceil(bits/8.0); }

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
      throw("MHAL:convert: invalid mod type");

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
  for(auto iter = messages.begin(); iter != messages.end(); ++iter)
   {
     EMANELTE::MHAL::ENB_DL_Message enb_dl_msg;

     if(enb_dl_msg.ParseFromString(iter->first))
      {
       const auto & rxControl = iter->second;

       const uint32_t & pci = enb_dl_msg.phy_cell_id();

       Debug("TX:%s %s, rx_seq %lu, pci %u, sf_time %ld:%06ld, msg:%s\n",
            __func__,
            bInStep ? "in-step" : "late",
            rxControl.rxData_.rx_seqnum_,
            pci,
            rxControl.rxData_.sf_time_.tv_sec,
            rxControl.rxData_.sf_time_.tv_usec,
            GetDebugString(enb_dl_msg.DebugString()).c_str());

       auto signal = dl_signals.find(pci);

       // existing append
       if(signal != dl_signals.end())
         {
           signal->second.emplace_back(enb_dl_msg, rxControl);
         }
       else
         {
           dl_signals.emplace(pci, DL_ENB_Signals(1, DL_ENB_Signal(enb_dl_msg, rxControl)));
         }
     }
   else
     {
       Error("MHAL:%s ParseFromString ERROR\n", __func__);
     }
   }

  return (dl_signals);
}


// return signals for a specific pci (enb)
static DL_ENB_Signals ue_dl_enb_subframe_search_i(srslte_ue_sync_t * ue_sync, const uint32_t * tti)
{
   const auto dl_signals = ue_dl_get_signals_i(&ue_sync->last_timestamp);

   auto iter = dl_signals.find(ue_sync->cell.id);

   if(iter != dl_signals.end())
    {
      Debug("RX:%s sf available sot %ld%0.6f, pci %hu\n",
            __func__,
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
      Debug("RX:%s nothing found for pci %hu\n", __func__, ue_sync->cell.id);

      return DL_ENB_Signals();
    }
}


static UL_DCIList get_ul_dci_list_i(uint16_t rnti)
{
  UL_DCIList dci_message_list;

  if(enb_dl_msg_.pdcch_size() > 0)
    {
      for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
       {
         const auto & pdcch_message = enb_dl_msg_.pdcch(n);

         const auto & dci_message = pdcch_message.ul_dci();

         if(dci_message.rnti() == rnti)
           {
             if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDCCH, rnti))
               {
                 Warning("RX:%s: found dci for rnti 0x%hx\n", __func__, rnti);

                 dci_message_list.emplace_back(dci_message);
               }
           }
       }
    }

  return dci_message_list;
}


static DL_DCIList get_dl_dci_list_i(uint16_t rnti)
{
  DL_DCIList dci_message_list;

  if(enb_dl_msg_.pdcch_size() > 0)
    {
      for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
       {
         const auto & pdcch_message = enb_dl_msg_.pdcch(n);

         const auto & dci_message = pdcch_message.dl_dci();

         if(dci_message.rnti() == rnti)
           {
             if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDCCH, rnti))
               {
                 Warning("RX:%s: found dci for rnti 0x%hx\n", __func__, rnti);

                 dci_message_list.emplace_back(dci_message);
               }
           }
       }
    }

  return dci_message_list;
}



static PDSCH_DataList ue_dl_get_pdsch_data_list_i(uint32_t refid, uint16_t rnti)
{
  PDSCH_DataList data_message_list;

  if(enb_dl_msg_.has_pdsch())
    {
      const auto & pdsch_message = enb_dl_msg_.pdsch();

      if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PDSCH, rnti))
        {
          for(int n = 0; n < pdsch_message.data().size(); ++n)
            {
              const auto & data_message = pdsch_message.data(n);

              if(data_message.refid() == refid)
                {
                  Info("RX:%s: found data for refid %u\n", __func__, refid);

                  data_message_list.emplace_back(data_message);
                }
            }
        }
    }

  return data_message_list;
}


void ue_initialize(srslte::log * log_h, uint32_t sf_interval_msec, EMANELTE::MHAL::mhal_config_t & mhal_config)
{
  log_h_ = log_h;

  Info("INIT:%s sf_interval %u msec\n", __func__, sf_interval_msec);

  EMANELTE::MHAL::UE::initialize(sf_interval_msec, mhal_config);
}


void ue_set_frequencies(float ul_freq, float dl_freq, uint32_t earfcn)
{
  Info("INIT:%s ul_freq %6.4f MHz, fl_freq %6.4f MHz, earfcn %u -> %u\n",
       __func__,
       ul_freq/1e6,
       dl_freq/1e6,
       earfcn_,
       earfcn);

  earfcn_ = earfcn;

  EMANELTE::MHAL::UE::set_frequencies(ul_freq, dl_freq);
}


void ue_set_bandwidth(int n_prb)
{
  Info("INIT:%s n_prb %d\n", __func__, n_prb);

  EMANELTE::MHAL::UE::set_num_resource_blocks(n_prb);
}


void ue_start()
{
  Info("START:%s\n", __func__);

  pthread_mutexattr_t mattr;

  if(pthread_mutexattr_init(&mattr) < 0)
   {
     Error("START:%s pthread_mutexattr_init error %s, exit\n", __func__, strerror(errno));

     exit(1);
   }
  else
   {
     if(pthread_mutexattr_setprotocol(&mattr, PTHREAD_PRIO_INHERIT) < 0)
       {
         Error("START:%s pthread_mutexattr_setprotocol error %s, exit\n", __func__, strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&dl_mutex_, &mattr) < 0)
       {
         Error("START:%s pthread_mutex_init error %s, exit\n", __func__, strerror(errno));

         exit(1);
       }

     if(pthread_mutex_init(&ul_mutex_, &mattr) < 0)
       {
         Error("START:%s pthread_mutex_init error %s, exit\n", __func__, strerror(errno));

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
  Info("STOP:%s\n", __func__);

  EMANELTE::MHAL::UE::stop();

  pthread_mutex_destroy(&dl_mutex_);

  pthread_mutex_destroy(&ul_mutex_);
}


void ue_set_crnti(uint16_t crnti)
{
  Info("MHAL:%s from 0x%hx to 0x%hx\n", __func__, crnti_, crnti);

  crnti_ = crnti;

  UESTATS::setCrnti(crnti);
}

void ue_set_prach_freq_offset(uint32_t freq_offset)
{
  Info("MHAL:%s %u\n", __func__, freq_offset);

  prach_freq_offset_ = freq_offset;
}

// 1 initial state cell search
/*typedef struct SRSLTE_API {
  uint32_t            cell_id;
  srslte_cp_t         cp;
  srslte_frame_type_t frame_type;
  float               peak; 
  float               mode; 
  float               psr;
  float               cfo; 
} srslte_ue_cellsearch_result_t;

typedef enum SRSLTE_API { SRSLTE_FDD = 0, SRSLTE_TDD = 1 } srslte_frame_type_t;

typedef struct SRSLTE_API {
  srslte_ue_sync_t ue_sync;
  cf_t             *sf_buffer[SRSLTE_MAX_PORTS];
  uint32_t         nof_rx_antennas;
  uint32_t         max_frames;
  uint32_t         nof_valid_frames;  // number of 5 ms frames to scan 
  uint32_t         *mode_ntimes;
  uint8_t          *mode_counted; 
  srslte_ue_cellsearch_result_t *candidates; 
} srslte_ue_cellsearch_t; */
int ue_dl_cellsearch_scan(srslte_ue_cellsearch_t * cs,
                          srslte_ue_cellsearch_result_t * res,
                          int force_nid_2,
                          uint32_t *max_peak)
{
  // n_id_2's
  std::set<uint32_t> n_id2s;

  UESTATS::Cells cells;

  // 40 sf
  const uint32_t max_tries = cs->max_frames * 5;

  uint32_t num_pss_sss_found = 0;
  uint32_t try_num           = 0;

  EMANELTE::MHAL::UE::begin_cell_search();

  while(try_num++ <= max_tries)
   {
     // cell search is typically done in blocks of 5 sf's
     // we handle the radio recv call here one at a time
     const auto dl_signals = ue_dl_get_signals_i(&cs->ue_sync.last_timestamp);

     // for each enb
     for(auto iter = dl_signals.begin(); iter != dl_signals.end(); ++iter)
      {
        const uint32_t pci = iter->first;

        const uint32_t n_id_1 = pci / 3;
 
        const uint32_t n_id_2 = pci % 3;

        Debug("RX:%s: try %u/%u, pci %hu, %zu signals\n", 
              __func__, 
              try_num,
              max_tries,
              pci,
              iter->second.size());

        // force is enabled, but this cell id group does not match
        if(is_valid_n_id_2(force_nid_2) && n_id_2 != (uint32_t)force_nid_2)
         {
           Info("RX:%s: n_id_1 %u, n_id_2 %u != %d, ignore\n",
                __func__,
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
               const auto & enb_dl_msg = iter->second[n].first;

               // looking for pss/sss
               if(enb_dl_msg.has_pss_sss())
                {
                  const auto & pss_sss = enb_dl_msg.pss_sss();
 
                  // should all be the same
                  cp = pss_sss.cp_mode() == EMANELTE::MHAL::CP_NORM ? SRSLTE_CP_NORM : SRSLTE_CP_EXT;

                  peak_sum = iter->second[n].second.rxData_.peak_sum_;

                  num_samples = iter->second[n].second.rxData_.num_samples_;

                  ++num_pss_sss_found;

                  Info("RX:%s: found pss_sss %s, peak_sum %0.1f, num_samples %u\n",
                       __func__,
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
                 res[n_id_2].cell_id     = pci;
                 res[n_id_2].cp          = cp;
                 res[n_id_2].peak        = peak_avg;
                 res[n_id_2].mode        = 1.0;
                 res[n_id_2].psr         = 0.0;
                 res[n_id_2].cfo         = 0.0;
                 res[n_id_2].frame_type  = SRSLTE_FDD;

                 Info("RX:%s: new PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                      __func__,
                      pci,
                      n_id_1,
                      n_id_2,
                      peak_avg);
               }
             else
               {
                // tie goes to the first entry (numeric lowest id)
                if(peak_avg > res[n_id_2].peak)
                  {
                     Info("RX:%s: replace PCI %u, n_id_1 %u, n_id_2 %u, peak_avg %f\n",
                           __func__,
                           pci,
                           n_id_1,
                           n_id_2,
                           peak_avg);

                     res[n_id_2].cell_id = pci;
                     res[n_id_2].cp      = cp;
                     res[n_id_2].peak    = peak_avg;
                  }
               }
            }
         }
      }
   }

  cs->ue_sync.pss_stable_cnt = num_pss_sss_found;
  cs->ue_sync.pss_is_stable  = num_pss_sss_found > 0 ? true : false;

  float max_avg = 0.0f;

  // now find the best
  for(auto iter = n_id2s.begin(); iter != n_id2s.end(); ++iter)
    {
      if(res[*iter].peak > max_avg)
        {
          *max_peak = *iter;

          max_avg = res[*iter].peak;
        }
    }

  Info("RX:%s: sf_idx %u, done, num_cells %zu, max_peak id %u, max_avg %f\n",
          __func__,
          cs->ue_sync.sf_idx,
          n_id2s.size(),
          *max_peak,
          max_avg);

  UESTATS::enterCellSearch(cells, earfcn_);

  return n_id2s.size();
}

// 2 mib search
/* typedef struct SRSLTE_API {
  srslte_ue_sync_t              ue_sync;
  cf_t                          *sf_buffer[SRSLTE_MAX_PORTS];
  uint32_t                      nof_rx_antennas;
  uint32_t                      max_frames;
  uint32_t                      nof_valid_frames;  // number of 5 ms frames to scan 
  uint32_t                      *mode_ntimes;
  uint8_t                       *mode_counted; 
  srslte_ue_cellsearch_result_t *candidates; 
} srslte_ue_cellsearch_t;

typedef struct {
  srslte_ue_mib_t  ue_mib; 
  srslte_ue_sync_t ue_sync; 
  cf_t             *sf_buffer[SRSLTE_MAX_PORTS];
  uint32_t         nof_rx_antennas;
} srslte_ue_mib_sync_t;

typedef struct SRSLTE_API {
  srslte_sync_t         sfind;
  cf_t*                 sf_symbols[SRSLTE_MAX_PORTS];
  srslte_ofdm_t         fft;
  srslte_pbch_t         pbch;
  srslte_chest_dl_t     chest;
  srslte_chest_dl_res_t chest_res;
  uint8_t               bch_payload[SRSLTE_BCH_PAYLOAD_LEN];
  uint32_t              nof_tx_ports; 
  uint32_t              sfn_offset; 
  uint32_t              frame_cnt; 
} srslte_ue_mib_t;

typedef struct SRSLTE_API {
  cf_t*    ce[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  uint32_t nof_re;
  float    noise_estimate;
  float    noise_estimate_dbm;
  float    snr_db;
  float    snr_ant_port_db[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  float    rsrp;
  float    rsrp_dbm;
  float    rsrp_neigh_dbm;
  float    rsrp_port_dbm[SRSLTE_MAX_PORTS];
  float    rsrp_ant_port_dbm[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  float    rsrq;
  float    rsrq_db;
  float    rsrq_ant_port_db[SRSLTE_MAX_PORTS][SRSLTE_MAX_PORTS];
  float    rssi_dbm;
  float    cfo;
  float    sync_error;
} srslte_chest_dl_res_t; */
int ue_dl_mib_search(const srslte_ue_cellsearch_t * cs,
                     srslte_ue_mib_sync_t * ue_mib_sync,
                     srslte_cell_t * cell)
{
  // 40 sf
  const uint32_t max_tries = cs->max_frames * 5;

  uint32_t try_num = 0;

  while(try_num++ < max_tries)
    {
      // we handle the radio recv call here one at a time
      const auto dl_enb_signals = ue_dl_enb_subframe_search_i(&ue_mib_sync->ue_sync, NULL);

      Debug("RX:ue_dl_mib_search: pci %hu, try %d/%u, %zu signals\n", 
            ue_mib_sync->ue_sync.cell.id, try_num, max_tries, dl_enb_signals.size());

      if(! dl_enb_signals.empty())
        {
          const auto enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              auto rxControl = dl_enb_signals[0].second;

              if(rxControl.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PBCH))
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const auto & pss_sss = enb_dl_msg.pss_sss();

                      const auto & pbch = enb_dl_msg.pbch();

                      Info("RX:ue_dl_mib_search: found pbch %s\n", GetDebugString(pbch.DebugString()).c_str());

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
/*typedef struct SRSLTE_API {
  srslte_sync_t s        find;
  srslte_sync_t          strack;
  uint32_t               max_prb;
  srslte_agc_t           agc; 
  bool                   do_agc; 
  uint32_t               agc_period; 
  int                    decimate;
  srslte_timestamp_t     last_timestamp;
  uint32_t               nof_rx_antennas; 
  srslte_ue_sync_state_t state;
  uint32_t               frame_len; 
  uint32_t               fft_size;
  uint32_t               nof_recv_sf;
  uint32_t               nof_avg_find_frames;
  uint32_t               frame_find_cnt;
  uint32_t               sf_len;
  uint64_t               frame_ok_cnt;
  uint32_t               frame_no_cnt; 
  uint32_t               frame_total_cnt; 
  uint32_t               frame_number; // not used
  srslte_cell_t          cell; 
  uint32_t               sf_idx;
  bool                   cfo_is_copied;
  bool                   cfo_correct_enable_track;
  bool                   cfo_correct_enable_find;
  float                  cfo_current_value;
  float                  cfo_loop_bw_pss;
  float                  cfo_loop_bw_ref;
  float                  cfo_pss_min;
  float                  cfo_ref_min;
  float                  cfo_ref_max;
  uint32_t               pss_stable_cnt;
  uint32_t               pss_stable_timeout;
  bool                   pss_is_stable;
  uint32_t               peak_idx;
  int                    next_rf_sample_offset;
  int                    last_sample_offset; 
  float                  mean_sample_offset; 
  uint32_t               sample_offset_correct_period;
  float                  sfo_ema; 
} srslte_ue_sync_t; */
int ue_dl_system_frame_search(srslte_ue_sync_t * ue_sync, uint32_t * sfn)
{
  const uint32_t max_tries = 50;

  uint32_t try_num = 0;

  while(try_num++ < max_tries)
    {
      const auto dl_enb_signals = ue_dl_enb_subframe_search_i(ue_sync, NULL);

      Debug("RX:%s: try %d/%u, %zu signals\n", 
            __func__,
            try_num, 
            max_tries,
            dl_enb_signals.size());

      if(! dl_enb_signals.empty())
        {
          const auto enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              auto rxControl = dl_enb_signals[0].second;

              // check for PSS SSS if PBCH is good
              if(rxControl.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PBCH))
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const auto & pss_sss = enb_dl_msg.pss_sss();

                      const auto & pbch = enb_dl_msg.pbch();

                      Info("RX:%s: found pbch %s, try %u/%u\n",
                           __func__,
                           GetDebugString(pbch.DebugString()).c_str(),
                           try_num,
                           max_tries);

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

   const auto dl_enb_signals = ue_dl_enb_subframe_search_i(ue_sync, &tti);

   enb_dl_msg_.Clear();

   rx_control_.SINRTester_.release();

   enb_dl_grant_.Clear();

   // single antenna mode expect 1 msg for our cell id
   if(dl_enb_signals.size() != 1)
     {
       Info("RX:%s: found %zu signals, cell_id %u, expected 1, tti %u\n", 
            __func__,
            dl_enb_signals.size(),
            ue_sync->cell.id,
            tti);
 
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


float ue_dl_decode_signal(uint32_t cell_id)
{
   float rssi = 0.0;

   if(enb_dl_msg_.IsInitialized())
     {
       const uint32_t & pci = enb_dl_msg_.phy_cell_id();

       // check dl msg src vs our selected cell
       if(pci != cell_id)
         {
           Info("RX:%s: pci 0x%x != cell_id 0x%x, ignore\n",
                __func__, pci, cell_id);
         }
       else
         {
          rssi = 3.0; // XXX TODO
 
          Debug("RX:%s: pci %u, rssi %f\n", __func__, pci, rssi);
        }
     }
   else
     {
       Info("RX:%s: empty msg\n", __func__);
     }

   return rssi;
}

/*
typedef struct SRSLTE_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srslte_dci_location_t;

typedef struct SRSLTE_API {
  uint8_t               payload[SRSLTE_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srslte_dci_location_t location;
  srslte_dci_format_t   format;
  uint16_t              rnti;
} srslte_dci_msg_t; */

// see ue_dl_find_dl_dc
// int srslte_ue_dl_find_dl_dci(srslte_ue_dl_t*     q,
//                              srslte_dl_sf_cfg_t* sf,
//                              srslte_ue_dl_cfg_t* cfg,
//                              uint16_t            rnti,
//                              srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG])
int ue_dl_find_dl_dci(srslte_ue_dl_t*     q,
                      srslte_dl_sf_cfg_t* sf,
                      srslte_ue_dl_cfg_t* cfg,
                      uint16_t            rnti,
                      srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG])

{
  srslte_dci_msg_t dci_msg[SRSLTE_MAX_DCI_MSG] = {};

  int nof_msg = 0;

  const auto dci_message_list = get_dl_dci_list_i(rnti);

  // expecting 1 dci/rnti
  if(dci_message_list.size() == 1)
    {
      const auto & dci_message = dci_message_list[0];

      const auto data_message_list = ue_dl_get_pdsch_data_list_i(dci_message.refid(), rnti);

      // XXX TODO pass/fail
      UESTATS::getPDCCH(rnti, true);

      // expecting 1 pdsch/dci
      if(data_message_list.size() == 1)
        {
          // XXX TODO pass/fail
          UESTATS::getPDSCH(rnti, true);

          // save the dci grant for get_pdsch
          enb_dl_grant_ = data_message_list[0];

          const auto & dl_dci_message      = dci_message.dci_msg();
          const auto & dl_dci_message_data = dl_dci_message.data();

          dci_msg[0].nof_bits      = dl_dci_message.num_bits();
          dci_msg[0].rnti          = rnti;
          dci_msg[0].format        = get_msg_format(dl_dci_message.format());
          dci_msg[0].location.L    = dl_dci_message.l_level();
          dci_msg[0].location.ncce = dl_dci_message.l_ncce();

          memcpy(dci_msg[0].payload, dl_dci_message_data.data(), dl_dci_message_data.size());

          Info("RX:%s found dl_dci ref id %u, rnti 0x%hx\n", 
                __func__, dci_message.refid(), rnti);

          ++nof_msg;

          // Unpack DCI messages see lib/src/phy/phch/dci.c
          for (int i = 0; i < nof_msg; i++) {
            if (srslte_dci_msg_unpack_pdsch(&q->cell, sf, &cfg->dci_cfg, &dci_msg[i], &dci_dl[i])) {
               Error("RX:%s Unpacking DL DCI\n", __func__);
               return SRSLTE_ERROR;
            }
          }

         q->chest_res.snr_db = 111; // XXX TODO
       }
      else
       {
         if(data_message_list.size() > 1)
          {
            Warning("RX:%s found multiple dci_data (%zu) for rnti 0x%hx\n", 
                    __func__, data_message_list.size(), rnti);
          }
         else
          {
            Warning("RX:%s no data for rnti 0x%hx\n", __func__, rnti);
          }
       }
    }
   else
    {
      if(dci_message_list.size() > 1)
       {
         Warning("RX:%s found multiple dl_dci (%zu) for rnti 0x%hx\n", 
                 __func__, dci_message_list.size(), rnti);
       }
    }

  return nof_msg;
}

/*
typedef struct SRSLTE_API {
  uint32_t L;    // Aggregation level
  uint32_t ncce; // Position of first CCE of the dci
} srslte_dci_location_t;

typedef struct SRSLTE_API {
  uint8_t               payload[SRSLTE_DCI_MAX_BITS];
  uint32_t              nof_bits;
  srslte_dci_location_t location;
  srslte_dci_format_t   format;
  uint16_t              rnti;
} srslte_dci_msg_t; */

// see lib/src/phy/ue/ue_dl.c
int ue_dl_find_ul_dci(srslte_ue_dl_t*     q,
                      srslte_dl_sf_cfg_t* sf,
                      srslte_ue_dl_cfg_t* cfg,
                      uint16_t            rnti,
                      srslte_dci_ul_t     dci_ul[SRSLTE_MAX_DCI_MSG])
{
  srslte_dci_msg_t dci_msg[SRSLTE_MAX_DCI_MSG] = {};
  int nof_msg = 0;

  if(rnti) 
   {
     const auto dci_message_list = get_ul_dci_list_i(rnti);

     // expecting 1 dci/rnti
     if(dci_message_list.size() == 1)
      {
        const auto & dci_message         = dci_message_list[0];
        const auto & ul_dci_message      = dci_message.dci_msg();
        const auto & ul_dci_message_data = ul_dci_message.data();

        dci_msg[0].nof_bits      = ul_dci_message.num_bits();
        dci_msg[0].rnti          = rnti;
        dci_msg[0].format        = get_msg_format(ul_dci_message.format());
        dci_msg[0].location.L    = ul_dci_message.l_level();
        dci_msg[0].location.ncce = ul_dci_message.l_ncce();

        memcpy(dci_msg[0].payload, ul_dci_message_data.data(), ul_dci_message_data.size());

        Info("RX:%s found ul_dci rnti 0x%hx\n", __func__, rnti);

        ++nof_msg;

        // Unpack DCI messages
        for (int i = 0; i < nof_msg; i++) {
          if (srslte_dci_msg_unpack_pusch(&q->cell, sf, &cfg->dci_cfg, &dci_msg[i], &dci_ul[i])) {
            Error("RX:%s Unpacking UL DCI\n", __func__);
            return SRSLTE_ERROR;
          }
        }

        q->chest_res.snr_db = 111; // XXX TODO
      }
   else
    {
      if(dci_message_list.size() > 1)
       {
         Warning("RX:%s found multiple (%zu) ul_dci for rnti 0x%hx\n", 
                  __func__, dci_message_list.size(), rnti);
       }
    }
  }
 
  return nof_msg;
}


/* typedef struct {
    bool     enabled;
    uint32_t rv;
    uint8_t* payload;
    union {
      srslte_softbuffer_rx_t* rx;
      srslte_softbuffer_tx_t* tx;
    } softbuffer;
 } tb_action_t;

 typedef struct {
   tb_action_t tb[SRSLTE_MAX_TB];

   bool generate_ack;
 } tb_action_dl_t; */
void ue_dl_decode_pdsch(srsue::mac_interface_phy::tb_action_dl_t * dl_action,
                        bool acks[SRSLTE_MAX_CODEWORDS])
{
   const int tb = enb_dl_grant_.tb();

   if(dl_action->tb[tb].enabled)
    {
      if(dl_action->tb[tb].payload)
        {
          acks[tb] = true;

          const auto & grant_data = enb_dl_grant_.data();

          memcpy(dl_action->tb[tb].payload, grant_data.data(), grant_data.size());

          Info("PDSCH:%s: tb[%d], payload len %zu\n",
               __func__,
               tb, 
               grant_data.size());
        }
      else
        {
          Error("PDSCH:%s: tb[%d], no payload_ptr\n", __func__, tb);
        } 
    }
   else
    {
      Error("PDSCH:%s: tb[%d], decode not enabled\n", __func__, tb);
    }
}


/* typedef struct SRSLTE_API {
  uint32_t n_prb_lowest;
  uint32_t n_dmrs;
  uint32_t I_phich;
} srslte_phich_grant_t;

typedef struct SRSLTE_API {
  uint32_t ngroup;
  uint32_t nseq;
} srslte_phich_resource_t;

typedef struct SRSLTE_API {
  bool  ack_value;
  float distance;
} srslte_phich_res_t; */

// see lib/src/phy/ue/ue_dl.c
int ue_dl_decode_phich(srslte_ue_dl_t*       q,
                       srslte_dl_sf_cfg_t*   sf,
                       srslte_ue_dl_cfg_t*   cfg,
                       srslte_phich_grant_t* grant,
                       srslte_phich_res_t*   result,
                       uint16_t rnti)
{
  srslte_phich_resource_t n_phich;

  const uint32_t sf_idx = sf->tti % 10;

  srslte_phich_calc(&q->phich, grant, &n_phich);

  if(enb_dl_msg_.has_phich())
   {
     const auto & phich_message = enb_dl_msg_.phich();

     if(rx_control_.SINRTester_.sinrCheck(EMANELTE::MHAL::CHAN_PHICH, rnti))
      {
       if(rnti                == phich_message.rnti()        && 
          grant->n_prb_lowest == phich_message.num_prb_low() &&
          grant->n_dmrs       == phich_message.num_dmrs())
         {
           result->ack_value = phich_message.ack();
           result->distance  = 1.0;
         }

        Info("PHICH:%s sf_idx=%d, n_prb_l=%d, n_dmrs=%d, I_phich=%d, rnti %hu, ack %d, dist %f\n",
             __func__,
             sf_idx,
             grant->n_prb_lowest,
             grant->n_dmrs,
             grant->I_phich,
             rnti,
             result->ack_value,
             result->distance);

         q->chest_res.snr_db = 111; // XXX TODO
       }
   }

   return SRSLTE_SUCCESS;
}


#if 0 // XXX TODO

bool ue_dl_decode_pmch(srslte_ue_dl_t * q, 
                       uint16_t area_id,
                       uint8_t * payload)
{
  if(enb_dl_msg_.has_pmch())
    {
      const EMANELTE::MHAL::ENB_DL_Message_PMCH & pmch = enb_dl_msg_.pmch();

      if(area_id == pmch.area_id())
       {
         Info("MHAL:ue_dl_decode_pmch: area_id %u, tbs %u\n",
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
         Debug("MHAL:ue_dl_decode_pmch: area_id %u != our area_id %hu, tbs %u\n",
              pmch.area_id(),
              area_id,
              pmch.tbs());

         return false;
       }
    }

  Debug("MHAL:ue_dl_decode_pmch: no pmch\n");

  return false;
}
#endif


void ue_ul_tx_init()
{
  Debug("TX:%s: \n", __func__);
}

// send to mhal
void ue_ul_send_signal(time_t sot_sec, float frac_sec, const srslte_cell_t & cell)
{
  // end of tx sequence, tx_end will release to lock
  pthread_mutex_lock(&ul_mutex_);

  auto txinfo = ue_ul_msg_.mutable_transmitter();

  txinfo->set_crnti(crnti_);

  txinfo->set_phy_cell_id(cell.id);
  tx_control_.set_phy_cell_id(cell.id);

  txinfo->set_tti(tti_tx_);

  EMANELTE::MHAL::Data data;

  if(ue_ul_msg_.SerializeToString(&data))
    {
      // align sot to sf time
      const timeval tv_sf_time = {sot_sec, (time_t)(round(frac_sec * 1e3)*1e3)};
     
      auto ts = tx_control_.mutable_sf_time();
      ts->set_ts_sec(tv_sf_time.tv_sec);
      ts->set_ts_usec(tv_sf_time.tv_usec);

      tx_control_.set_message_type(EMANELTE::MHAL::UPLINK);
      tx_control_.set_tx_seqnum(tx_seqnum_++);
      tx_control_.set_tti_tx(tti_tx_);

      Info("TX:%s tx_ctrl:%s\n \t\tmsg:%s\n",
           __func__,
           GetDebugString(tx_control_.DebugString()).c_str(),
           GetDebugString(ue_ul_msg_.DebugString()).c_str());

      EMANELTE::MHAL::UE::send_msg(data, tx_control_);
    }
  else
    {
      Error("TX:%s: SerializeToString ERROR len %zu\n", __func__, data.length());
    }

  ue_ul_msg_.Clear();

  tx_control_.Clear();

  uplink_control_message_ = tx_control_.mutable_uplink();

  uplink_control_message_->Clear();

  pthread_mutex_unlock(&ul_mutex_);
}


void ue_ul_put_prach(int index)
{
   pthread_mutex_lock(&ul_mutex_);

   auto channel_message = uplink_control_message_->mutable_prach();

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PRACH,
                            EMANELTE::MHAL::MOD_BPSK,   // modtype
                            839);                       // PRACH sequence is 839 for formats 0-3 (all allowed by FDD) 

   // The upstream PRACH message is not really a slotted message
   // and can span 2 or 3 subframes. Set slot1 and slot2 resource blocks the same.
   // prach spans the 6 resource blocks starting from prach_freq_offset
   for(int i = 0; i < 6; ++i)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(prach_freq_offset_ + i));
     }

   auto prach = ue_ul_msg_.mutable_prach();

   auto preamble = prach->mutable_preamble();

   preamble->set_index(index);

   Info("PRACH:%s: index %d\n", __func__, index);

   pthread_mutex_unlock(&ul_mutex_);
}


/*typedef struct SRSLTE_API {
  srslte_cell_t    cell;
  uint16_t         current_rnti;
  bool             signals_pregenerated;
  srslte_pusch_t   pusch;
  srslte_pucch_t   pucch;
  srslte_ra_ul_pusch_hopping_t hopping;
} srslte_ue_ul_t;

 typedef struct SRSLTE_API {
  uint16_t         rnti;
  srslte_uci_cfg_t uci_cfg;

  // Common configuration
  uint32_t delta_pucch_shift;
  uint32_t n_rb_2;
  uint32_t N_cs;
  uint32_t N_pucch_1;
  bool     group_hopping_en; // common pusch config

  // Dedicated PUCCH configuration
  uint32_t I_sr;
  bool     sr_configured;
  uint32_t n_pucch_1[4]; // 4 n_pucch resources specified by RRC
  uint32_t n_pucch_2;
  uint32_t n_pucch_sr;
  bool     simul_cqi_ack;
  bool     tdd_ack_bundle; // if false, multiplex
  bool     sps_enabled;
  uint32_t tpc_for_pucch;

  // Release 10 CA specific
  srslte_ack_nack_feedback_mode_t ack_nack_feedback_mode;
  uint32_t                        n1_pucch_an_cs[SRSLTE_PUCCH_SIZE_AN_CS][SRSLTE_PUCCH_NOF_AN_CS];
  uint32_t                        n3_pucch_an_list[SRSLTE_PUCCH_SIZE_AN_CS];

  // Other configuration
  float threshold_format1;
  float threshold_data_valid_format1a;
  float threshold_data_valid_format2;

  // PUCCH configuration generated during a call to encode/decode
  srslte_pucch_format_t format;
  uint32_t              n_pucch;
  uint8_t               pucch2_drs_bits[SRSLTE_PUCCH2_MAX_DMRS_BITS];
} srslte_pucch_cfg_t; 

typedef struct SRSLTE_API {
  // Uplink config (includes common and dedicated variables)
  srslte_pucch_cfg_t                pucch;
  srslte_pusch_cfg_t                pusch;
  srslte_pusch_hopping_cfg_t        hopping;
  srslte_ue_ul_powerctrl_t          power_ctrl;
  srslte_refsignal_dmrs_pusch_cfg_t dmrs;
  srslte_refsignal_srs_cfg_t        srs;
} srslte_ul_cfg_t;

typedef struct SRSLTE_API {
  srslte_ul_cfg_t ul_cfg;
  bool            grant_available;
  uint32_t        cc_idx;
  bool            normalize_en;
  bool            cfo_en;
  float           cfo_tol;
  float           cfo_value;
} srslte_ue_ul_cfg_t; */

int ue_ul_put_pucch_i(srslte_ue_ul_t* q, srslte_ul_sf_cfg_t* sf, srslte_ue_ul_cfg_t* cfg, srslte_uci_value_t* uci_data)
{
   pthread_mutex_lock(&ul_mutex_);

   auto pucch_message = ue_ul_msg_.mutable_pucch();

   auto grant_message = pucch_message->add_grant();

   auto pucch_cfg = cfg->ul_cfg.pucch;

   const auto rnti = pucch_cfg.rnti;

   // see lib/src/phy/ue/ue_ul.c
   srslte_ue_ul_pucch_resource_selection(&q->cell, &cfg->ul_cfg.pucch, &cfg->ul_cfg.pucch.uci_cfg, uci_data);

   // default: SRSLTE_PUCCH_FORMAT_1
   EMANELTE::MHAL::MOD_TYPE modType = EMANELTE::MHAL::MOD_BPSK;

   uint32_t bits = 0;

   switch(pucch_cfg.format)
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
       Error("PUCCH:ue_ul_put_pucch: unknown pucch format: %d\n",
             pucch_cfg.format);
     }

   auto channel_message = uplink_control_message_->add_pucch();

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PUCCH,
                            modType,
                            bits);

   channel_message->set_rnti(pucch_cfg.rnti);

   // see lib/src/phy/phch/pucch.c 
   // pucch_cp(srslte_pucch_t* q, 
   //          srslte_ul_sf_cfg_t* sf, 
   //          srslte_pucch_cfg_t* cfg, 
   //          cf_t* source, cf_t* dest,
   //          bool source_is_grid)
   // Determine n_prb
   uint16_t n_prb[2] = {0};

   for(int ns = 0; ns < 2; ++ns)
     {
       if(! ((n_prb[ns] = srslte_pucch_n_prb(&q->cell, &pucch_cfg, ns)) < q->cell.nof_prb))
         {
           Error("PUCCH:%s ns %d, n_prb=%d > cell_nof_prb %d\n", 
                 __func__, ns, n_prb[ns], q->cell.nof_prb);

           return SRSLTE_ERROR;
         }
     }

   // flag when resource blocks are different on slot 1 and 2 of the subframe
   channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[0]));
   channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(n_prb[1]));

   grant_message->set_num_prb(n_prb[1]);

   grant_message->set_num_pucch(pucch_cfg.n_pucch);

   grant_message->set_rnti(rnti);

   grant_message->set_uci(uci_data, sizeof(srslte_uci_value_t));

   Info("PUCCH:%s: rnti %hu\n", __func__, rnti);

   pthread_mutex_unlock(&ul_mutex_);

   // signal ready
   return 1;
}



/* typedef struct SRSLTE_API {
  srslte_mod_t mod;
  int          tbs;
  int          rv;
  uint32_t     nof_bits;
  uint32_t     cw_idx;
  bool         enabled;
} srslte_ra_tb_t;

 typedef struct SRSLTE_API {
  bool           is_from_rar;
  uint32_t       L_prb;
  uint32_t       n_prb[2];       // rb_start after frequency hopping
  uint32_t       n_prb_tilde[2]; // rb_start after frequency hopping per retx
  uint32_t       freq_hopping;
  uint32_t       nof_re;
  uint32_t       nof_symb;
  srslte_ra_tb_t tb;
  srslte_ra_tb_t last_tb;
  uint32_t       n_dmrs;
} srslte_pusch_grant_t;

typedef struct SRSLTE_API {
  srslte_uci_cfg_ack_t ack;
  srslte_cqi_cfg_t     cqi;
  bool                 is_scheduling_request_tti;
} srslte_uci_cfg_t;

typedef struct SRSLTE_API {
  uint16_t                rnti;
  srslte_uci_cfg_t        uci_cfg;
  srslte_uci_offset_cfg_t uci_offset;
  srslte_pusch_grant_t    grant;

  uint32_t max_nof_iterations;
  uint32_t last_O_cqi;
  uint32_t K_segm;
  uint32_t current_tx_nb;
  bool     csi_enable;
  bool     enable_64qam;

  union {
    srslte_softbuffer_tx_t* tx;
    srslte_softbuffer_rx_t* rx;
  } softbuffers;

  bool     meas_time_en;
  uint32_t meas_time_value;
} srslte_pusch_cfg_t; 

typedef struct SRSLTE_API {
  bool                   scheduling_request;
  srslte_cqi_value_t     cqi;
  srslte_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srslte_uci_value_t;

typedef struct SRSLTE_API {
  uint8_t*           ptr;
  srslte_uci_value_t uci;
} srslte_pusch_data_t; */
static int ue_ul_put_pusch_i(srslte_pusch_cfg_t* cfg, srslte_pusch_data_t* data)
{
   pthread_mutex_lock(&ul_mutex_);

   auto channel_message = uplink_control_message_->add_pusch();

   const auto grant = &cfg->grant;
 
   const auto rnti = cfg->rnti;

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PUSCH,
                            convert(grant->tb.mod),
                            grant->tb.tbs);

   channel_message->set_rnti(rnti);

   for (size_t i = 0; i < grant->L_prb; ++i)
     {
       channel_message->add_resource_block_frequencies_slot1(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[0] + i));
       channel_message->add_resource_block_frequencies_slot2(EMANELTE::MHAL::UE::get_tx_prb_frequency(grant->n_prb[1] + i));
     }

   auto pusch_message = ue_ul_msg_.mutable_pusch();

   auto grant_message = pusch_message->add_grant();

   grant_message->set_rnti(rnti);

   // srslte_pusch_grant_t
   grant_message->set_ul_grant(grant, sizeof(srslte_pusch_grant_t));

   // srslte_uci_value_t
   grant_message->set_uci(&data->uci, sizeof(srslte_uci_value_t));

   // payload
   grant_message->set_payload(data->ptr, bits_to_bytes(grant->tb.tbs));

   UESTATS::putULGrant(rnti);

   pthread_mutex_unlock(&ul_mutex_);

   // signal ready
   return 1;
}


/* typedef struct SRSLTE_API {
  srslte_cell_t    cell;
  uint16_t         current_rnti;
  bool             signals_pregenerated;
  srslte_pusch_t   pusch;
  srslte_pucch_t   pucch;
  srslte_ra_ul_pusch_hopping_t hopping;
} srslte_ue_ul_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  bool                shortened;
} srslte_ul_sf_cfg_t;

typedef struct SRSLTE_API {
  // Uplink config (includes common and dedicated variables)
  srslte_pucch_cfg_t                pucch;
  srslte_pusch_cfg_t                pusch;
  srslte_pusch_hopping_cfg_t        hopping;
  srslte_ue_ul_powerctrl_t          power_ctrl;
  srslte_refsignal_dmrs_pusch_cfg_t dmrs;
  srslte_refsignal_srs_cfg_t        srs;
} srslte_ul_cfg_t;

typedef struct SRSLTE_API {
  srslte_ul_cfg_t ul_cfg;
  bool            grant_available;
  uint32_t        cc_idx;
  bool            normalize_en;
  bool            cfo_en;
  float           cfo_tol;
  float           cfo_value;
} srslte_ue_ul_cfg_t;

typedef struct SRSLTE_API {
  bool                   scheduling_request;
  srslte_cqi_value_t     cqi;
  srslte_uci_value_ack_t ack;
  uint8_t                ri; // Only 1-bit supported for RI
} srslte_uci_value_t;

typedef struct SRSLTE_API {
  uint8_t*           ptr;
  srslte_uci_value_t uci;
} srslte_pusch_data_t; */

// see lib/src/phy/ue/ue_ul.c
// srslte_ue_ul_encode(srslte_ue_ul_t* q, 
//                     srslte_ul_sf_cfg_t* sf,
//                     srslte_ue_ul_cfg_t* cfg,
//                     srslte_pusch_data_t* data);
//
int ue_ul_encode(srslte_ue_ul_t* q, srslte_ul_sf_cfg_t* sf, srslte_ue_ul_cfg_t* cfg, srslte_pusch_data_t* data)
{
  /* Convert DTX to NACK in channel-selection mode (Release 10 only)*/
  if(cfg->ul_cfg.pucch.ack_nack_feedback_mode != SRSLTE_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL) {
    uint32_t dtx_count = 0;
    for(uint32_t a = 0; a < cfg->ul_cfg.pusch.uci_cfg.ack.nof_acks; a++) {
      if(data->uci.ack.ack_value[a] == 2) {
        data->uci.ack.ack_value[a] = 0;
        dtx_count++;
      }
    }

    /* If all bits are DTX, do not transmit HARQ */
    if(dtx_count == cfg->ul_cfg.pusch.uci_cfg.ack.nof_acks) {
      cfg->ul_cfg.pusch.uci_cfg.ack.nof_acks = 0;
    }
  }

   // see lib/src/phy/ue/ue_ul.c
#define uci_pending(cfg) (cfg.ack.nof_acks > 0 || cfg.cqi.data_enable || cfg.cqi.ri_len > 0)
   if(cfg->grant_available) 
    {
      return ue_ul_put_pusch_i(&cfg->ul_cfg.pusch, data);
    } 
   else if((uci_pending(cfg->ul_cfg.pucch.uci_cfg) || 
             data->uci.scheduling_request) && cfg->cc_idx == 0)
    // Send PUCCH over PCell only
    {
      if(!cfg->ul_cfg.pucch.rnti)
       {
         if(! (cfg->ul_cfg.pucch.rnti = q->current_rnti))
          {
            Error("TX:%s Warning PUCCH rnti or current_rnti are not set\n", __func__);
          }
       }

      return ue_ul_put_pucch_i(q, sf, cfg, &data->uci);
    }
   else
    {
       // XXX SRS
    }

  // nothing pending
  return 0;
}

} // end namespace phy_adapter
} // end namepsace srsue

#endif
