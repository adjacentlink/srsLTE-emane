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

#include <stdint.h> // radio.h needs this
#include "srslte/config.h"
#include "srslte/radio/radio.h"

#ifdef PHY_ADAPTER_ENABLE

#include "srsue/hdr/phy/phy_adapter.h"
#include "srsue/hdr/phy/sync.h"

#include "libemanelte/enbotamessage.pb.h"
#include "libemanelte/ueotamessage.pb.h"
#include "libemanelte/mhalue.h"
#include "libemanelte/uestatisticmanager.h"

#include <vector>
#include <map>
#include <set>

#define Error(fmt, ...)          if (log_h_) log_h_->error  (fmt, ##__VA_ARGS__)
#define Warning(fmt, ...)        if (log_h_) log_h_->warning(fmt, ##__VA_ARGS__)
#define Info(fmt, ...)           if (log_h_) log_h_->info   (fmt, ##__VA_ARGS__)
#define Debug(fmt, ...)          if (log_h_) log_h_->debug  (fmt, ##__VA_ARGS__)
#define Console(fmt, ...)        if (log_h_) log_h_->console(fmt, ##__VA_ARGS__)

#undef DEBUG_HEX

#ifdef DEBUG_HEX
#define InfoHex(p,l,fmt, ...)    if (log_h_) log_h_->info_hex ((const uint8_t*)p, l, fmt, ##__VA_ARGS__)
#endif

// private namespace for misc helpers and state for PHY_ADAPTER
namespace {
 EMANELTE::MHAL::UE_UL_Message     ue_ul_msg_;
 EMANELTE::MHAL::TxControlMessage  tx_control_;
 EMANELTE::MHAL::UplinkMessage    *uplink_control_message_;

 // enb dl msg and rx control info
 using DL_ENB_Signal = std::pair<EMANELTE::MHAL::ENB_DL_Message, EMANELTE::MHAL::RxControl>;

 // vector of dl signals from a specific enb
 using DL_ENB_Signals = std::vector<DL_ENB_Signal>;

 // map of dl signals to enb(pci)
 using DL_ENB_Signal_Map = std::map<uint16_t, DL_ENB_Signals>;

 using DL_ENB_Messages = std::vector<std::pair<struct timeval, EMANELTE::MHAL::RxMessages>>;

 srslte::rf_buffer_t buffer_(1);

 DL_ENB_Messages dl_enb_messages_;

 EMANELTE::MHAL::ENB_DL_Message enb_dl_msg_;

 struct SignalQuality {
  double sinr_dB_;
  double noiseFloor_dBm_;

  SignalQuality(double sinr, double noiseFloor) :
   sinr_dB_(sinr),
   noiseFloor_dBm_(noiseFloor)
  { }
};


 // pdsch rnti/messages with signal quality
 using ENB_DL_Message_PDSCH_Entry = std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data, SignalQuality>;

 using ENB_DL_PDSCH_MESSAGES = std::map<uint16_t, ENB_DL_Message_PDSCH_Entry>;

 ENB_DL_PDSCH_MESSAGES enb_dl_pdsch_messages_;

 EMANELTE::MHAL::RxControl rx_control_;

 uint64_t tx_seqnum_         = 0;
 uint16_t crnti_             = 0;
 uint32_t earfcn_            = 0;
 uint32_t tti_tx_            = 0;
 uint32_t prach_freq_offset_ = 0;
 srsue::sync * sync_         = nullptr;

 srslte::log * log_h_ = NULL;

 pthread_mutex_t ul_mutex_;

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

  inline int bits_to_bytes(int bits) { return bits/8; }

}

namespace srsue {
namespace phy_adapter {


typedef std::vector<EMANELTE::MHAL::ENB_DL_Message_PDCCH_DL_DCI> DL_DCI_Results;

// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDCCH_UL_DCI, SignalQuality> UL_DCI_Result;

typedef std::vector<UL_DCI_Result> UL_DCI_Results;


// message, sinr
typedef std::pair<EMANELTE::MHAL::ENB_DL_Message_PDSCH_Data, SignalQuality> PDSCH_Result;

typedef std::vector<PDSCH_Result> PDSCH_Results;


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

// see sf_worker::update_measurements() -> cc_worker::update_measurements()
static void ue_dl_update_chest_i(srslte_chest_dl_res_t * chest_res, float snr_db, float noise_db)
{
    //  from faux_rf
    //  cc_idx avg_noise 0.000, acg_rsrp_dbm -20.313, avg_rsrq_db -3.519, avg_rssi_dbm 2.536, pathloss 20.313, avg_snr_db_cqi  141.119
    chest_res->sync_error         = 0;
    chest_res->snr_db             = snr_db;
    chest_res->noise_estimate_dbm = noise_db;
    chest_res->noise_estimate     = noise_db;
}

// get all ota messages (all pci enb dl messages and rx control info)
static DL_ENB_Signal_Map ue_dl_get_signals_i(srslte_timestamp_t * ts)
{
  if(dl_enb_messages_.empty())
   {
      Error("No Messages:%s: \n", __func__);

      return DL_ENB_Signal_Map{};
   }

  // we expect 1 and only 1 message
  const auto message = dl_enb_messages_.front();

  if(ts)
   {
     ts->full_secs = message.first.tv_sec;
     ts->frac_secs = message.first.tv_usec / 1e6;
   }

  // all signals from all enb(s)
  DL_ENB_Signal_Map dl_signals;

  // for each message rx ota
  for(auto msg_iter = message.second.begin(); 
       msg_iter != message.second.end();
         ++msg_iter)
   {
     EMANELTE::MHAL::ENB_DL_Message enb_dl_msg;

     if(enb_dl_msg.ParseFromString(msg_iter->first))
      {
       const auto & rxControl = msg_iter->second;

       const uint32_t & pci = enb_dl_msg.phy_cell_id();

       Debug("RX:%s rx_seq %lu, pci %u, sf_time %ld:%06ld, msg:%s\n",
            __func__,
            rxControl.rxData_.rx_seqnum_,
            pci,
            rxControl.rxData_.sf_time_.tv_sec,
            rxControl.rxData_.sf_time_.tv_usec,
            GetDebugString(enb_dl_msg.DebugString()).c_str());

       auto signal_iter = dl_signals.find(pci);

       // existing append
       if(signal_iter != dl_signals.end())
        {
          signal_iter->second.emplace_back(enb_dl_msg, rxControl);
        }
       else
        {
          dl_signals.emplace(pci, DL_ENB_Signals{1, {enb_dl_msg, rxControl}});
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
static DL_ENB_Signals ue_dl_enb_subframe_get_i(srslte_ue_sync_t * ue_sync, const uint32_t * tti)
{
   Debug("Enter:%s: \n", __func__);

   const auto dl_signals = ue_dl_get_signals_i(&ue_sync->last_timestamp);

   const auto iter = dl_signals.find(ue_sync->cell.id);

   if(iter != dl_signals.end())
    {
      Debug("RX:%s sf available sot %12.06f, pci %hu\n",
            __func__,
            ue_sync->last_timestamp.full_secs +
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
      Info("RX:%s nothing found for pci %hu\n", __func__, ue_sync->cell.id);

      return DL_ENB_Signals{};
    }
}


static UL_DCI_Results get_ul_dci_list_i(uint16_t rnti, uint32_t cc_idx)
{
  UL_DCI_Results ul_dci_results;

  for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
   {
     const auto & pdcch_message = enb_dl_msg_.pdcch(n);

     if(pdcch_message.has_ul_dci())
      {
         const auto & ul_dci_message = pdcch_message.ul_dci();

         if(ul_dci_message.rnti() == rnti)
           {
             const auto sinrResult = rx_control_.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH, rnti, cc_idx);

             if(sinrResult.bPassed_)
               {
                 Info("PUCCH:%s: found dci rnti 0x%hx\n", __func__, rnti);

                 // message, sinr
                 ul_dci_results.emplace_back(ul_dci_message, SignalQuality(sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_));

                 break; // we expect 1 and only 1 match
               }
             else
               {
                 Info("PUCCH:%s: fail snr rnti 0x%hx\n", __func__, rnti);
               }
           }
         else
           {
             Debug("PUCCH:%s: rnti 0x%hx != ul_dci_rnti 0x%hx, skip\n", 
                    __func__, rnti, ul_dci_message.rnti());
           }
       }
   }

  return ul_dci_results;
}


static DL_DCI_Results get_dl_dci_list_i(uint16_t rnti, uint32_t cc_idx)
{
  DL_DCI_Results dl_dci_results;

  for(int n = 0; n < enb_dl_msg_.pdcch_size(); ++n)
   {
     const auto & pdcch_message = enb_dl_msg_.pdcch(n);

     if(pdcch_message.has_dl_dci())
       {
         const auto & dl_dci_message = pdcch_message.dl_dci();

         if(dl_dci_message.rnti() == rnti)
           {
             if(rx_control_.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PDCCH, rnti, cc_idx).bPassed_)
               {
                 Info("PDSCH:%s: found dci rnti 0x%hx, refid %u\n", 
                        __func__, rnti, dl_dci_message.refid());

                 dl_dci_results.emplace_back(dl_dci_message);

                 break; // XXX expect 1 and only 1 match
               }
             else
               {
                 Info("PDSCH:%s: fail snr rnti 0x%hx\n", __func__, rnti);
               }
           }
         else
           {
             Debug("PDSCH:%s: rnti 0x%hx != dl_dci_rnti 0x%hx, refid %u, skip\n", 
                    __func__, rnti, dl_dci_message.rnti(), dl_dci_message.refid());
           }
       }
   }

  return dl_dci_results;
}



static PDSCH_Results ue_dl_get_pdsch_data_list_i(uint32_t refid, uint16_t rnti, uint32_t cc_idx)
{
  PDSCH_Results pdsch_results;

  if(enb_dl_msg_.has_pdsch())
    {
      const auto & pdsch_message = enb_dl_msg_.pdsch();

      Info("PDSCH:%s: have %u messages\n", __func__, pdsch_message.data().size());

      const auto sinrResult = rx_control_.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PDSCH, rnti, cc_idx);

      if(sinrResult.bPassed_)
        {
          for(int n = 0; n < pdsch_message.data().size(); ++n)
            {
              const auto & data_message = pdsch_message.data(n);

              if(data_message.refid() == refid)
                {
                  pdsch_results.emplace_back(data_message, SignalQuality(sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_));
                }
            }
        }
      else
        {
          Info("PDSCH:%s: fail snr rnti 0x%hx\n", __func__, rnti);
        }
    }

  return pdsch_results;
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

void ue_set_sync(srsue::sync * sync)
{
  sync_ = sync;
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


// read frame for this tti common to all states
int ue_dl_read_frame(srslte_timestamp_t* rx_time)
{
  timeval tv_tti = {0,0};

  EMANELTE::MHAL::RxMessages messages;

  EMANELTE::MHAL::UE::get_messages(messages, tv_tti);

  dl_enb_messages_.clear();
  dl_enb_messages_.emplace_back(tv_tti, messages);

  if(rx_time)
   {
     rx_time->full_secs = tv_tti.tv_sec; 
     rx_time->frac_secs = tv_tti.tv_usec / 1e6;
   }

  return 1;
}


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

// 1 initial state cell search
int ue_dl_cellsearch_scan(srslte_ue_cellsearch_t * cs,
                          srslte_ue_cellsearch_result_t * res,
                          int force_nid_2,
                          uint32_t *max_peak)
{
  // n_id_2's
  std::set<uint32_t> n_id2s;

  UESTATS::Cells cells;

  // cell search is typically done in blocks of 5 sf's
  const uint32_t max_tries   = cs->max_frames * 5; // 40 sf
  uint32_t num_pss_sss_found = 0;
  uint32_t try_num           = 0;

  EMANELTE::MHAL::UE::begin_cell_search();

  while(++try_num <= max_tries)
   {
     // radio recv called here
     sync_->radio_recv_fnc(buffer_, 0, 0);

     const auto dl_signals = ue_dl_get_signals_i(&cs->ue_sync.last_timestamp);

     // for each enb
     for(auto iter = dl_signals.begin(); iter != dl_signals.end(); ++iter)
      {
        const uint32_t pci    = iter->first;
        const uint32_t n_id_1 = pci / 3;
        const uint32_t n_id_2 = pci % 3;

        // force is enabled, but this cell id group does not match
        if(is_valid_n_id_2(force_nid_2) && n_id_2 != (uint32_t)force_nid_2)
         {
           Info("RX:%s: n_id_1 %u, n_id_2 %u != %d, ignore\n",
                __func__, n_id_1, n_id_2, force_nid_2);
 
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

               // search for pss/sss
               if(enb_dl_msg.has_pss_sss())
                {
                  const auto & pss_sss = enb_dl_msg.pss_sss();
 
                  // should all be the same
                  cp = pss_sss.cp_mode() == EMANELTE::MHAL::CP_NORM ? SRSLTE_CP_NORM : SRSLTE_CP_EXT;

                  peak_sum = iter->second[n].second.rxData_.peak_sum_;

                  num_samples = iter->second[n].second.rxData_.num_samples_;

                  ++num_pss_sss_found;

                  Debug("RX:%s: found pss_sss %s, peak_sum %0.1f, num_samples %u\n",
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

  Info("RX:%s: sf_idx %u, DONE, num_cells %zu, max_peak id %u, max_avg %f\n",
          __func__,
          cs->ue_sync.sf_idx,
          n_id2s.size(),
          *max_peak,
          max_avg);

  UESTATS::enterCellSearch(cells, earfcn_);

  return n_id2s.size();
}

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

// 2 mib search
int ue_dl_mib_search(const srslte_ue_cellsearch_t * cs,
                     srslte_ue_mib_sync_t * ue_mib_sync,
                     srslte_cell_t * cell)
{
  // 40 sf
  const uint32_t max_tries = cs->max_frames * 5;
  uint32_t try_num         = 0;

  while(++try_num <= max_tries)
    {
      // radio recv called here
      sync_->radio_recv_fnc(buffer_, 0, 0);

      const auto dl_enb_signals = ue_dl_enb_subframe_get_i(&ue_mib_sync->ue_sync, NULL);

      Debug("RX:ue_dl_mib_search: pci %hu, try %d/%u, %zu signals\n", 
            ue_mib_sync->ue_sync.cell.id, try_num, max_tries, dl_enb_signals.size());

      if(! dl_enb_signals.empty())
        {
          const auto enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              auto rxControl = dl_enb_signals[0].second;

              if(rxControl.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PBCH, 0).bPassed_)
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const auto & pss_sss = enb_dl_msg.pss_sss();

                      const auto & pbch = enb_dl_msg.pbch();

                      Info("RX:ue_dl_mib_search: found PBCH %s\n", GetDebugString(pbch.DebugString()).c_str());

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
            else
             {
               Info("MIB:%s: fail snr\n", __func__);
             }
          }
       }
    }

  UESTATS::enterMibSearch(false);

  return 0;
}


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

// 3 system frame search
int ue_dl_system_frame_search(srslte_ue_sync_t * ue_sync, uint32_t * sfn)
{
  const uint32_t max_tries = 1;

  uint32_t try_num = 0;

  while(++try_num <= max_tries)
    {
      // radio recv called here
      sync_->radio_recv_fnc(buffer_, 0, 0);

      const auto dl_enb_signals = ue_dl_enb_subframe_get_i(ue_sync, NULL);

      if(! dl_enb_signals.empty())
        {
          const auto enb_dl_msg = dl_enb_signals[0].first;

          if(enb_dl_msg.has_pbch())
            {
              auto rxControl = dl_enb_signals[0].second;

              // check for PSS SSS if PBCH is good
              if(rxControl.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PBCH, 0).bPassed_)
                {
                  if(enb_dl_msg.has_pss_sss())
                    {
                      const auto & pss_sss = enb_dl_msg.pss_sss();

                      const auto & pbch = enb_dl_msg.pbch();

                      Info("RX:%s: found PBCH %s, try %u/%u\n",
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
              else
                {
                  Info("PBCH:%s: fail snr\n", __func__);
                }
            }
        }
    }

  UESTATS::enterSysFrameSearch(false);

  return 0;
}


// 4 
int ue_dl_sync_search(srslte_ue_sync_t * ue_sync, uint32_t tti)
{
   // set next tx tti
   tti_tx_ = (tti+4)%10240;

   EMANELTE::MHAL::UE::set_tti(tti);

   // radio recv called here
   sync_->radio_recv_fnc(buffer_, 0, 0);

   const auto dl_enb_signals = ue_dl_enb_subframe_get_i(ue_sync, &tti);

   enb_dl_msg_.Clear();

   rx_control_.SINRTester_.release();

   enb_dl_pdsch_messages_.clear();

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


float ue_dl_get_rssi(uint32_t cell_id)
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
          rssi = 10.0; // ALINK_XXX TODO
 
          Debug("RX:%s: pci %u, rssi %f\n", __func__, pci, rssi);
        }
     }
   else
     {
       Info("RX:%s: empty msg, rssi %f\n", __func__, rssi);
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
} srslte_dci_msg_t;

typedef struct SRSLTE_API {
  srslte_dl_cfg_t       cfg;
  srslte_chest_dl_cfg_t chest_cfg;
  uint32_t              last_ri;
  float                 snr_to_cqi_offset;
} srslte_ue_dl_cfg_t; */

// see ue_dl_find_dl_dc
// int srslte_ue_dl_find_dl_dci(srslte_ue_dl_t*     q,
//                              srslte_dl_sf_cfg_t* sf,
//                              srslte_ue_dl_cfg_t* dl_cfg,
//                              uint16_t            rnti,
//                              srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG])
int ue_dl_cc_find_dl_dci(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_ue_dl_cfg_t* dl_cfg,
                         uint16_t            rnti,
                         srslte_dci_dl_t     dci_dl[SRSLTE_MAX_DCI_MSG],
                         uint32_t            cc_idx)

{
  srslte_dci_msg_t dci_msg[SRSLTE_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  const auto dl_dci_results = get_dl_dci_list_i(rnti, cc_idx);

  // expecting 1 dci/rnti
  if(dl_dci_results.size() == 1)
    {
      const auto & dci_message = dl_dci_results[0];

      const auto pdsch_results = ue_dl_get_pdsch_data_list_i(dci_message.refid(), rnti, cc_idx);

      // XXX TODO pass/fail
      UESTATS::getPDCCH(rnti, true);

      // expecting 1 pdsch/dci
      if(pdsch_results.size() == 1)
        {
          const auto & pdsch_result = pdsch_results.front();

          // XXX TODO pass/fail
          UESTATS::getPDSCH(rnti, true);

          // save the grant for pdsch_decode
          enb_dl_pdsch_messages_.emplace(rnti, 
                                         ENB_DL_Message_PDSCH_Entry{pdsch_result.first, 
                                                                    pdsch_result.second});

          const auto & dl_dci_message      = dci_message.dci_msg();
          const auto & dl_dci_message_data = dl_dci_message.data();

          auto & dci_entry = dci_msg[0];

          dci_entry.nof_bits      = dl_dci_message.num_bits();
          dci_entry.rnti          = rnti;
          dci_entry.format        = get_msg_format(dl_dci_message.format());
          dci_entry.location.L    = dl_dci_message.l_level();
          dci_entry.location.ncce = dl_dci_message.l_ncce();

          memcpy(dci_entry.payload, dl_dci_message_data.data(), dl_dci_message_data.size());
          ++nof_msg;

          ue_dl_update_chest_i(&q->chest_res, pdsch_result.second.sinr_dB_, pdsch_result.second.noiseFloor_dBm_);

#ifdef DEBUG_HEX
          InfoHex(dl_dci_message_data.data(), dl_dci_message_data.size(),
                  "PDCCH:%s dl_dci ref id %u, rnti 0x%hx, dci_len %zu\n", 
                  __func__, dci_message.refid(), rnti, dl_dci_message_data.size());
#endif

          // Unpack DCI messages see lib/src/phy/phch/dci.c
          for (int i = 0; i < nof_msg; i++) {
            if (srslte_dci_msg_unpack_pdsch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_msg[i], &dci_dl[i])) {
               Error("PDCCH:%s Unpacking DL DCI\n", __func__);
               return SRSLTE_ERROR;
            }
          }
       }
      else
       {
         if(pdsch_results.size() > 1)
          {
            Info("PDCCH:%s found %zu dl_dci for rnti 0x%hx\n", 
                    __func__, pdsch_results.size(), rnti);
          }
         else
          {
            Error("PDCCH:%s no pdsch data for rnti 0x%hx, refid %u\n",
                  __func__, rnti, dci_message.refid());
          }
       }
    }
   else
    {
      if(dl_dci_results.size() > 1)
       {
         Warning("PDCCH:%s found %zu dl_dci for rnti 0x%hx\n", 
                 __func__, dl_dci_results.size(), rnti);
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
int ue_dl_cc_find_ul_dci(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_ue_dl_cfg_t* dl_cfg,
                         uint16_t            rnti,
                         srslte_dci_ul_t     dci_ul[SRSLTE_MAX_DCI_MSG],
                         uint32_t cc_idx)
{
  srslte_dci_msg_t dci_msg[SRSLTE_MAX_DCI_MSG] = {{}};

  int nof_msg = 0;

  if(rnti) 
   {
     const auto ul_dci_results = get_ul_dci_list_i(rnti, cc_idx);

     // expecting 1 dci/rnti
     if(ul_dci_results.size() == 1)
      {
        const auto & dci_message         = ul_dci_results[0].first;
        const auto & ul_dci_message      = dci_message.dci_msg();
        const auto & ul_dci_message_data = ul_dci_message.data();
 
        ue_dl_update_chest_i(&q->chest_res, ul_dci_results[0].second.sinr_dB_, ul_dci_results[0].second.noiseFloor_dBm_);

        auto & dci_entry = dci_msg[0];

        dci_entry.nof_bits      = ul_dci_message.num_bits();
        dci_entry.rnti          = rnti;
        dci_entry.format        = get_msg_format(ul_dci_message.format());
        dci_entry.location.L    = ul_dci_message.l_level();
        dci_entry.location.ncce = ul_dci_message.l_ncce();

        memcpy(dci_entry.payload, ul_dci_message_data.data(), ul_dci_message_data.size());

        Info("PUCCH:%s found ul_dci rnti 0x%hx\n", __func__, rnti);

        ++nof_msg;

        // Unpack DCI messages
        for (int i = 0; i < nof_msg; i++) {
          if (srslte_dci_msg_unpack_pusch(&q->cell, sf, &dl_cfg->cfg.dci, &dci_msg[i], &dci_ul[i])) {
            Error("PUCCH:%s Unpacking UL DCI\n", __func__);
            return SRSLTE_ERROR;
          }
        }

      }
   else
    {
      if(ul_dci_results.size() > 1)
       {
         Warning("PUCCH:%s found %zu ul_dci for rnti 0x%hx\n", 
                  __func__, ul_dci_results.size(), rnti);
       }
    }
  }
 
  return nof_msg;
}

/* typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t      nof_rx_antennas;
  uint16_t      current_mbsfn_area_id;
  uint16_t      pregen_rnti;
   
  srslte_pcfich_t pcfich;
  srslte_pdcch_t  pdcch;
  srslte_pdsch_t  pdsch;
  srslte_pmch_t   pmch;
  srslte_phich_t  phich;

  srslte_regs_t regs[MI_MAX_REGS];
  uint32_t      mi_manual_index; 
  bool          mi_auto;

  srslte_chest_dl_t     chest;
  srslte_chest_dl_res_t chest_res;
  srslte_ofdm_t         fft[SRSLTE_MAX_PORTS];
  srslte_ofdm_t         fft_mbsfn;

  // Variables for blind DCI search
  dci_blind_search_t current_ss_ue[MI_MAX_REGS][3][10];
  dci_blind_search_t current_ss_common[MI_MAX_REGS][3]; 
  srslte_dci_msg_t   pending_ul_dci_msg[SRSLTE_MAX_DCI_MSG];
  uint32_t           pending_ul_dci_count;
} srslte_ue_dl_t;

typedef struct SRSLTE_API {
  srslte_tdd_config_t tdd_config;
  uint32_t            tti;
  uint32_t            cfi;
  srslte_sf_t         sf_type;
  uint32_t            non_mbsfn_region;
} srslte_dl_sf_cfg_t;

typedef struct SRSLTE_API {
  srslte_tx_scheme_t tx_scheme;
  uint32_t           pmi;
  bool               prb_idx[2][SRSLTE_MAX_PRB];
  uint32_t           nof_prb;
  uint32_t           nof_re;
  uint32_t           nof_symb_slot[2];
  srslte_ra_tb_t     tb[SRSLTE_MAX_CODEWORDS];
  int                last_tbs[SRSLTE_MAX_CODEWORDS];
  uint32_t           nof_tb;
  uint32_t           nof_layers;
} srslte_pdsch_grant_t;

typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
} srslte_pdsch_cfg_t;

typedef struct {
  uint8_t* payload;
  bool     crc;
  float    avg_iterations_block;
} srslte_pdsch_res_t; */

// see lib/src/phy/phch/pdsch.c srslte_pdsch_decode()
int ue_dl_cc_decode_pdsch(srslte_ue_dl_t*     q,
                          srslte_dl_sf_cfg_t* sf,
                          srslte_pdsch_cfg_t* cfg,
                          srslte_pdsch_res_t  data[SRSLTE_MAX_CODEWORDS],
                          uint32_t cc_idx)
{
   const auto rnti = cfg->rnti;

   for(uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; ++tb)
    {
     if(cfg->grant.tb[tb].enabled)
       {
         const auto iter = enb_dl_pdsch_messages_.find(rnti);

         if(iter != enb_dl_pdsch_messages_.end())
           {
             const auto & pdsch_result  = iter->second;
             const auto & pdsch_message = pdsch_result.first;
             const auto & pdsch_data    = pdsch_message.data();

             memcpy(data[tb].payload, pdsch_data.data(), pdsch_data.size());

             data[tb].avg_iterations_block = 1;
             data[tb].crc = true;

             ue_dl_update_chest_i(&q->chest_res, pdsch_result.second.sinr_dB_, pdsch_result.second.noiseFloor_dBm_);

#ifdef DEBUG_HEX
             InfoHex(pdsch_data.data(), pdsch_data.size(),
                     "PDSCH:%s: rnti 0x%hx, refid %d, tb[%d], payload %zu bytes, snr %f\n",
                     __func__, rnti, pdsch_message.refid(), tb, pdsch_data.size(), q->chest_res.snr_db);
#endif
           }
         else
           {
             Error("PDSCH:%s: rnti %0xhx, no pdsch data\n", __func__, rnti);
           }
       }
    }

  return SRSLTE_SUCCESS;
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
} srslte_phich_res_t;

typedef struct SRSLTE_API {
  srslte_dl_cfg_t       cfg;
  srslte_chest_dl_cfg_t chest_cfg;
  uint32_t              last_ri;
  float                 snr_to_cqi_offset;
} srslte_ue_dl_cfg_t; */

// see lib/src/phy/ue/ue_dl.c
int ue_dl_cc_decode_phich(srslte_ue_dl_t*       q,
                          srslte_dl_sf_cfg_t*   sf,
                          srslte_ue_dl_cfg_t*   cfg,
                          srslte_phich_grant_t* grant,
                          srslte_phich_res_t*   result,
                          uint16_t rnti,
                          uint32_t cc_idx)
{
  srslte_phich_resource_t n_phich;

  srslte_phich_calc(&q->phich, grant, &n_phich);

  if(enb_dl_msg_.has_phich())
   {
     const auto & phich_message = enb_dl_msg_.phich();

     const auto sinrResult = rx_control_.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PHICH, rnti, cc_idx);

     ue_dl_update_chest_i(&q->chest_res, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

     if(sinrResult.bPassed_)
      {
       if(rnti                == phich_message.rnti()        && 
          grant->n_prb_lowest == phich_message.num_prb_low() &&
          grant->n_dmrs       == phich_message.num_dmrs())
         {
           result->ack_value = phich_message.ack();
           result->distance  = 1.0;
         }

        Info("PHICH:%s sf_idx=%d, n_prb_l=%d, n_dmrs=%d, I_phich=%d, rnti 0x%hx, ack %d, dist %f\n",
             __func__,
             sf->tti % 10,
             grant->n_prb_lowest,
             grant->n_dmrs,
             grant->I_phich,
             rnti,
             result->ack_value,
             result->distance);

       }
     else
       {
         Info("PHICH:%s: fail snr\n", __func__);
       }
   }

   return SRSLTE_SUCCESS;
}


/* typedef struct SRSLTE_API {
  srslte_cell_t cell;
  uint32_t      nof_rx_antennas;
  uint16_t      current_mbsfn_area_id;
  uint16_t      pregen_rnti;
   
  srslte_pcfich_t pcfich;
  srslte_pdcch_t  pdcch;
  srslte_pdsch_t  pdsch;
  srslte_pmch_t   pmch;
  srslte_phich_t  phich;

  srslte_regs_t regs[MI_MAX_REGS];
  uint32_t      mi_manual_index; 
  bool          mi_auto;

  srslte_chest_dl_t     chest;
  srslte_chest_dl_res_t chest_res;
  srslte_ofdm_t         fft[SRSLTE_MAX_PORTS];
  srslte_ofdm_t         fft_mbsfn;

  // Variables for blind DCI search
  dci_blind_search_t current_ss_ue[MI_MAX_REGS][3][10];
  dci_blind_search_t current_ss_common[MI_MAX_REGS][3]; 
  srslte_dci_msg_t   pending_ul_dci_msg[SRSLTE_MAX_DCI_MSG];
  uint32_t           pending_ul_dci_count;
} srslte_ue_dl_t;

typedef struct SRSLTE_API {
  srslte_tx_scheme_t tx_scheme;
  uint32_t           pmi;
  bool               prb_idx[2][SRSLTE_MAX_PRB];
  uint32_t           nof_prb;
  uint32_t           nof_re;
  uint32_t           nof_symb_slot[2];
  srslte_ra_tb_t     tb[SRSLTE_MAX_CODEWORDS];
  int                last_tbs[SRSLTE_MAX_CODEWORDS];
  uint32_t           nof_tb;
  uint32_t           nof_layers;
} srslte_pdsch_grant_t;

typedef struct SRSLTE_API {
  srslte_pdsch_grant_t  grant;
  uint16_t              rnti;
  uint32_t              max_nof_iterations;
  srslte_mimo_decoder_t decoder_type;
  float                 p_a;
  uint32_t              p_b;
  float                 rs_power;
  bool                  power_scale;
  bool                  csi_enable;
} srslte_pdsch_cfg_t;

typedef struct SRSLTE_API {
  srslte_pdsch_cfg_t pdsch_cfg;
  uint16_t           area_id;
} srslte_pmch_cfg_t;

typedef struct {
  uint8_t* payload;
  bool     crc;
  float    avg_iterations_block;
} srslte_pdsch_res_t; */

int ue_dl_cc_decode_pmch(srslte_ue_dl_t*     q,
                         srslte_dl_sf_cfg_t* sf,
                         srslte_pmch_cfg_t*  cfg,
                         srslte_pdsch_res_t  data[SRSLTE_MAX_CODEWORDS],
                         uint32_t cc_idx)
{
   const auto area_id = cfg->area_id;

   for(uint32_t tb = 0; tb < SRSLTE_MAX_CODEWORDS; ++tb)
    {
      if(cfg->pdsch_cfg.grant.tb[tb].enabled)
       {
         if(enb_dl_msg_.has_pmch())
          {
            const auto & pmch = enb_dl_msg_.pmch();

            if(area_id == pmch.area_id())
             {
               const auto sinrResult = rx_control_.SINRTester_.sinrCheck2(EMANELTE::MHAL::CHAN_PMCH, cc_idx);

               ue_dl_update_chest_i(&q->chest_res, sinrResult.sinr_dB_, sinrResult.noiseFloor_dBm_);

               if(sinrResult.bPassed_)
                 {
                   memcpy(data[tb].payload, pmch.data().data(), pmch.data().length());
 
                   data[tb].avg_iterations_block = 1;

                   data[tb].crc = true;
#ifdef DEBUG_HEX
                   InfoHex(pmch.data().data(), pmch.data().size(),
                           "PMCH:%s: areaid %d, tb[%d], payload %zu bytes, snr %f\n",
                           __func__, area_id, tb, pmch.data().size(), q->chest_res.snr_db);
#endif
                 }
               else
                 {
                   Info("PMCH:%s: area_id %d, fail snr\n", __func__, area_id);
                 }
             }
            else
             {
               Info("MHAL:%s: dl_area_id %u != area_id %hu, skip\n",
                       __func__, pmch.area_id(), area_id);
             }
          }
       }
    }

  return SRSLTE_SUCCESS;
}


void ue_ul_tx_init()
{
  Debug("TX:%s: \n", __func__);
}

// send to mhal
void ue_ul_send_signal(time_t sot_sec, float frac_sec, const srslte_cell_t & cell)
{
  // end of tx sequence, tx_end will release lock
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

      Debug("TX:%s tx_ctrl:%s\n",
           __func__,
           tx_control_.DebugString().c_str());

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

  auto prach    = ue_ul_msg_.mutable_prach();
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

int ue_ul_put_pucch_i(srslte_ue_ul_t* q, 
                      srslte_ul_sf_cfg_t* sf,
                      srslte_ue_ul_cfg_t* cfg,
                      srslte_uci_value_t* uci_data,
                      uint32_t cc_idx)
{
   pthread_mutex_lock(&ul_mutex_);

   auto pucch_message = ue_ul_msg_.mutable_pucch();
   auto grant_message = pucch_message->add_grant();
   auto pucch_cfg     = cfg->ul_cfg.pucch;
   const auto rnti    = pucch_cfg.rnti;

   srslte_uci_value_t uci_value2 = *uci_data;

   // see lib/src/phy/ue/ue_ul.c
   srslte_ue_ul_pucch_resource_selection(&q->cell, &cfg->ul_cfg.pucch, &cfg->ul_cfg.pucch.uci_cfg, uci_data, uci_value2.ack.ack_value);

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
   grant_message->set_uci(&uci_value2, sizeof(srslte_uci_value_t));

#ifdef DEBUG_HEX
   InfoHex(&uci_value2, sizeof(srslte_uci_value_t), 
           "PUCCH:%s: rnti 0x%hx\n", __func__, rnti);
#endif

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
static int ue_ul_put_pusch_i(srslte_pusch_cfg_t* cfg, srslte_pusch_data_t* data, uint32_t cc_idx)
{
   pthread_mutex_lock(&ul_mutex_);

   auto channel_message = uplink_control_message_->add_pusch();

   const auto grant = &cfg->grant;
   const auto rnti  = cfg->rnti;

   initUplinkChannelMessage(channel_message,
                            EMANELTE::MHAL::CHAN_PUSCH,
                            convert(grant->tb.mod),
                            grant->tb.tbs);

   channel_message->set_rnti(rnti);

   for(size_t i = 0; i < grant->L_prb; ++i)
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

#ifdef DEBUG_HEX
   InfoHex(data->ptr, bits_to_bytes(grant->tb.tbs), 
           "PUSCH:%s: rnti 0x%hx\n", __func__, rnti);
#endif

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
int ue_ul_encode(srslte_ue_ul_t* q, srslte_ul_sf_cfg_t* sf, srslte_ue_ul_cfg_t* cfg, srslte_pusch_data_t* data, uint32_t cc_idx)
{
  /* Convert DTX to NACK in channel-selection mode (Release 10 only)*/
  if(cfg->ul_cfg.pucch.ack_nack_feedback_mode != SRSLTE_PUCCH_ACK_NACK_FEEDBACK_MODE_NORMAL) {
    uint32_t dtx_count = 0;
    for(uint32_t a = 0; a < srslte_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg); a++) {
      if(data->uci.ack.ack_value[a] == 2) {
        data->uci.ack.ack_value[a] = 0;
        dtx_count++;
      }
    }

    /* If all bits are DTX, do not transmit HARQ */
    if(dtx_count == srslte_uci_cfg_total_ack(&cfg->ul_cfg.pusch.uci_cfg)) {
      for (int i = 0; i < 2; i++) { // Format 1b-CS only supports 2 CC
       cfg->ul_cfg.pusch.uci_cfg.ack[i].nof_acks = 0;
      }
    }
  }

   // see lib/src/phy/ue/ue_ul.c
#define uci_pending(cfg) (srslte_uci_cfg_total_ack(&cfg) > 0 || cfg.cqi.data_enable || cfg.cqi.ri_len > 0)
   if(cfg->grant_available) 
    {
      return ue_ul_put_pusch_i(&cfg->ul_cfg.pusch, data, cc_idx);
    } 
   else if((uci_pending(cfg->ul_cfg.pucch.uci_cfg) || 
            data->uci.scheduling_request) && cfg->cc_idx == 0)
    // Send PUCCH over PCell only
    {
      return ue_ul_put_pucch_i(q, sf, cfg, &data->uci, cc_idx);
    }
   else
    {
      return 0;
    }
}

} // end namespace phy_adapter
} // end namepsace srsue

#endif
