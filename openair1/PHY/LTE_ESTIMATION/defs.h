/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

#ifndef __LTE_ESTIMATION_DEFS__H__
#define __LTE_ESTIMATION_DEFS__H__

#include "PHY/defs.h"
/** @addtogroup _PHY_PARAMETER_ESTIMATION_BLOCKS_
 * @{
 */

/*!\brief Timing drift hysterisis in samples*/
#define SYNCH_HYST 2

/*!
\brief This function is used for time-frequency scanning prior to complete cell search.  It scans
over the entire LTE band for maximum correlation and keeps the 10 best scores and the correspoding frequency offset (5 kHz granularity) for each of the 3 PSS sequences.
\param ue Pointer to UE variables
\param band index of lte band
\param DL_freq Central RF Frequency in Hz
*/
/*!
\brief This function allocates memory needed for the synchronization.
\param frame_parms LTE DL frame parameter structure

 */


 /*为时间同步分配内存
 @param frame_parms 下行帧结构
*/
int lte_sync_time_init(LTE_DL_FRAME_PARMS *frame_parms); //LTE_UE_COMMON *common_vars

/*! \fn void lte_sync_time_free()
\brief This function frees the memory allocated by lte_sync_time_init.
 */
 /* 释放同步初始化过程中的内存
*/
void lte_sync_time_free(void);

/*!
\brief This function performs the coarse timing synchronization.
The algorithm uses a time domain correlation with a downsampled version of the received signal.
\param rxdata Received time domain data for all rx antennas
\param frame_parms LTE DL frame parameter structure
\param eNB_id return value with the eNb_id
\return sync_pos Position of the sync within the frame (downsampled) if successfull and -1 if there was an error or no peak was detected.
 */
/* 完成粗略的时间同步函数，算法使用接收信号下行符号的时域相关性
@param rxdata 所有接收端天线收到的时域信号
@param frame_parms LTE下行帧结构
@param eNB_id 基站ID
@return sync_pos 成功返回同步偏移量，失败则返回-1.
*/
int lte_sync_time(int **rxdata,
                  LTE_DL_FRAME_PARMS *frame_parms,
                  int *eNB_id);

/*!
\brief This function performs the coarse frequency and PSS synchronization.
The algorithm uses a frequency-domain correlation.  It scans over 20 MHz/10ms signal chunks using each of the 3 PSS finding the most likely (strongest) carriers and their frequency offset (+-2.5 kHz).
\param ue Pointer to UE data structure
\param band index of band in scan_info structure, used to store statistics
\param DL_freq center frequency of band being scanned, used when storing statistics
*/
/*  实现频率和PSS同步，使用了频率相关性，
@param ue 指向UE物理层变量的结构
@param band 使用的频带数
@param DL_freq 扫描频带的中心频率
*/
void lte_sync_timefreq(PHY_VARS_UE *ue,
                       int band,
                       unsigned int DL_freq);


/*!
\brief This function performs detection of the PRACH (=SRS) at the eNb to estimate the timing advance
The algorithm uses a time domain correlation with a downsampled version of the received signal.
\param rxdata Received time domain data for all rx antennas
\param frame_parms LTE DL frame parameter structure
\param length Length for correlation
\param peak_val pointer to value of returned peak
\param sync_corr_eNb pointer to correlation buffer
\return sync_pos Position of the sync within the frame (downsampled) if successfull and -1 if there was an error or no peak was detected.
 */
 /*! 基站侧探测PRACH探测参考信号来估计时域偏移量.
 @param rxdata 所有接收天线接收到的时域数据
 @param frame_parms LTE下行帧结构
 @param length 相关长度
 @param peak_val 最大值
 @param sync_corr_eNb eNB的同步缓冲
 @return sync_pos 同步位在帧中的位置
  */
int lte_sync_time_eNB(int32_t **rxdata,
                      LTE_DL_FRAME_PARMS *frame_parms,
                      uint32_t length,
                      uint32_t *peak_val,
                      uint32_t *sync_corr_eNb);

/*! 基站emul的时域同步
@param phy_vars_eNb 基站物理层变量
@param sect_id 区域ID
@param sync_val 同步值
只有声明，没有实现。
 */
int lte_sync_time_eNB_emul(PHY_VARS_eNB *phy_vars_eNb,
                           uint8_t sect_id,
                           int32_t *sync_val);

/*!
\brief This function performs channel estimation including frequency and temporal interpolation
\param phy_vars_ue Pointer to UE PHY variables
\param eNB_id Index of target eNB
\param eNB_offset Offset for interfering eNB (in terms cell ID mod 3)
\param Ns slot number (0..19)
\param p antenna port
\param l symbol within slot
\param symbol symbol within frame
*/
/*! 时域和频域的下行信道估计
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 目标基站ID
@param eNB_offset 基站偏移量
@param Ns 时隙数(0..19)
@param p 天线端口
@param l 时隙中ODFM符号数
@param symbol 帧中的符号数
*/
int lte_dl_channel_estimation(PHY_VARS_UE *phy_vars_ue,
                              module_id_t eNB_id,
                              uint8_t eNB_offset,
                              uint8_t Ns,
                              uint8_t p,
                              uint8_t l,
                              uint8_t symbol);

/*! 时域和频域的下行波束赋形信道估计
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 目标基站ID
@param eNB_offset 基站偏移量
@param Ns 时隙数(0..19)
@param p 天线端口
@param symbol 帧中的符号数
*/
int lte_dl_bf_channel_estimation(PHY_VARS_UE *phy_vars_ue,
                                 module_id_t eNB_id,
                                 uint8_t eNB_offset,
                                 uint8_t Ns,
                                 uint8_t p,
                                 uint8_t symbol);

/*! 时域和频域的下行MSBFN信道估计
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 目标基站ID
@param eNB_offset 基站偏移量
@param Ns 时隙数(0..19)
@param subframe 子帧数
@param p 天线端口
@param symbol 帧中的符号数
// 没有使用
*/
int lte_dl_msbfn_channel_estimation(PHY_VARS_UE *phy_vars_ue,
                                    module_id_t eNB_id,
                                    uint8_t eNB_offset,
                                    int subframe,
                                    unsigned char l,
                                    unsigned char symbol);

/*! 时域和频域的下行MSBFN信道估计
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 目标基站ID
@param eNB_offset 基站偏移量
@param subframe 子帧数
@param l 时隙中ODFM符号数
*/
int lte_dl_mbsfn_channel_estimation(PHY_VARS_UE *phy_vars_ue,
                                    module_id_t eNB_id,
                                    uint8_t eNB_offset,
                                    int subframe,
                                    unsigned char l);

/*!
\brief Frequency offset estimation for LTE
We estimate the frequency offset by calculating the phase difference between channel estimates for symbols carrying pilots (l==0 or l==3/4). We take a moving average of the phase difference.
\param dl_ch_estimates pointer to structure that holds channel estimates (one slot)
\param frame_parms pointer to LTE frame parameters
\param l symbol within slot
\param freq_offset pointer to the returned frequency offset
\param reset When non-zer it resets the filter to the initial value (set whenever tuning has been changed or for a one-shot estimate)
 */
 /*! 频域偏移量的估计
 @param dl_ch_estimates 现行信道估计结果
 @param frame_parms 帧结构
 @param l 时隙中ODFM符号数
 @param freq_offset 频偏
 @param reset 初始值复位
  */
int lte_est_freq_offset(int **dl_ch_estimates,
                        LTE_DL_FRAME_PARMS *frame_parms,
                        int l,
                        int* freq_offset,
            int reset);

int lte_mbsfn_est_freq_offset(int **dl_ch_estimates,
                              LTE_DL_FRAME_PARMS *frame_parms,
                              int l,
                              int* freq_offset);

/*! \brief Tracking of timing for LTE
This function computes the time domain channel response, finds the peak and adjusts the timing in pci_interface.offset accordingly.
\param frame_parms LTE DL frame parameter structure
\param phy_vars_ue Pointer to UE PHY data structure
\param eNb_id
\param clear If clear==1 moving average filter is reset
\param coef Coefficient of the moving average filter (Q1.15)
 */
 /*! 时域同步调整
 @param frame_parms 帧结构
 @param phy_vars_ue UE物理层变量
 @param eNb_id 基站ID
 @param clear 移除过滤器
 @param coef 过滤器的相关系数
*/
void lte_adjust_synch(LTE_DL_FRAME_PARMS *frame_parms,
                      PHY_VARS_UE *phy_vars_ue,
                      module_id_t eNb_id,
                      uint8_t subframe,
                      unsigned char clear,
                      short coef);

// UE 相关
//! \brief this function fills the PHY_VARS_UE->PHY_measurement structure
void lte_ue_measurements(PHY_VARS_UE *phy_vars_ue,
                         unsigned int subframe_offset,
                         unsigned char N0_symbol,
                         unsigned char abstraction_flag,
                         unsigned char rank_adaptation,
                         uint8_t subframe);

//! \brief This function performance RSRP/RSCP measurements
void ue_rrc_measurements(PHY_VARS_UE *phy_vars_ue,
                         uint8_t slot,
                         uint8_t abstraction_flag);

void lte_ue_measurements_emul(PHY_VARS_UE *phy_vars_ue,uint8_t last_slot,uint8_t eNB_id);

/*! \brief Function to return the path-loss based on the UE cell-specific reference signal strength and transmission power of eNB
@param Mod_id Module ID for UE
@param eNB_index Index of eNB on which to act
@returns Path loss in dB
 */
 /*! 获取路径损耗，根据参考信号强度和基站的传输功率
 @param Mod_id UE模块ID
 @param eNB_index 基站ID
 @param CC_id 成员载波ID
 @returns 路径损耗的DB值
  */
int16_t get_PL(module_id_t Mod_id,
               uint8_t CC_id,
               uint8_t eNB_index);

/*! 获取参考信号接收功率
@param Mod_id UE模块ID
@param eNB_index 基站ID
@param CC_id 成员载波ID
@returns 路径损耗的DB值
 */
double get_RSRP(module_id_t Mod_id,
                uint8_t CC_id,
                uint8_t eNB_index);


/*! 获取参考信号接收质量
@param Mod_id UE模块ID
@param eNB_index 基站ID
@param CC_id 成员载波ID
@returns 路径损耗的DB值
 */
uint32_t get_RSRQ(module_id_t Mod_id,
                  uint8_t CC_id,
                  uint8_t eNB_index);

/*! 获取相邻小区
@param Mod_id UE模块ID
@param CC_id 成员载波ID
 */
uint8_t get_n_adj_cells(module_id_t Mod_id,
                        uint8_t CC_id);
uint32_t get_rx_total_gain_dB(module_id_t Mod_id,uint8_t CC_id);
uint32_t get_RSSI(module_id_t Mod_id,uint8_t CC_id);
int8_t set_RSRP_filtered(module_id_t Mod_id,uint8_t CC_id,uint8_t eNB_index,float rsrp);
int8_t set_RSRQ_filtered(module_id_t Mod_id,uint8_t CC_id,uint8_t eNB_index,float rstq);

//! Automatic gain control
/* 物理层增益调整
@param phy_vars_ue 用户侧物理层变量
@param rx_power_fil_dB 接收功率
@param eNB_id 基站ID
*/
void phy_adjust_gain (PHY_VARS_UE *phy_vars_ue,
                      uint32_t rx_power_fil_dB,
                      unsigned char eNB_id);

/* LTE上行信道估计
@param phy_vars_eNB 基站物理层变量
@param proc 基站收发进程
@param UE_id UE ID
@param l 时隙中ODFM符号数
@param Ns 时隙数 0-19
*/
int lte_ul_channel_estimation(PHY_VARS_eNB *phy_vars_eNB,
			                        eNB_rxtx_proc_t *proc 基站收发进程,
                              module_id_t UE_id,
                              uint8_t l,
                              uint8_t Ns);


/* LTE上行信道频频估计
@param phy_vars_eNB 基站物理层变量
@param proc 基站收发进程
@param UE_id UE ID
@param l 时隙中ODFM符号数
@param Ns 时隙数 0-19
*/
int16_t lte_ul_freq_offset_estimation(LTE_DL_FRAME_PARMS *frame_parms,
                                      int32_t *ul_ch_estimates,
                                      uint16_t nb_rb);

/* LTE探测参考信号的信道估计
@param frame_parms 帧结构
@param eNB_common_vars 基站通用变量
@param eNB_srs_vars 基站探测参考信号变量
@param soundingrs_ul_config_dedicated SRS上行配置专有变量
@param sub_frame_number 子帧值
*/
int lte_srs_channel_estimation(LTE_DL_FRAME_PARMS *frame_parms,
                               LTE_eNB_COMMON *eNB_common_vars,
                               LTE_eNB_SRS *eNB_srs_vars,
                               SOUNDINGRS_UL_CONFIG_DEDICATED *soundingrs_ul_config_dedicated,
                               unsigned char sub_frame_number,
			       unsigned char eNB_id);

/* LTE估计信道的时域便宜
@param frame_parms 帧结构
@param lte_eNb_srs 基站SRS数据
@param eNb_id 基站ID
@param number_of_cards
@param coef
// 没有被使用
*/
int lte_est_timing_advance(LTE_DL_FRAME_PARMS *frame_parms,
                           LTE_eNB_SRS *lte_eNb_srs,
                           unsigned int *eNb_id,
                           unsigned char clear,
                           unsigned char number_of_cards,
                           short coef);

/* LTEPUSCH估计信道的时域便宜
@param phy_vars_eNB 基站物理层变量
@param UE_id UE id
*/
int lte_est_timing_advance_pusch(PHY_VARS_eNB* phy_vars_eNB,
                                module_id_t UE_id);


/* 基站IO参数
@param phy_vars_eNB 基站物理层变量
@param subframe 子帧数
@param eNB_id 基站ID
@param clear 复位
*/
void lte_eNB_I0_measurements(PHY_VARS_eNB *phy_vars_eNB,
                            int subframe,
                             module_id_t eNB_id,
                             unsigned char clear);

/* 基站IO eMule 参数
@param phy_vars_eNB 基站物理层变量
@param sect_id 区域ID
// 没有使用
*/
void lte_eNB_I0_measurements_emul(PHY_VARS_eNB *phy_vars_eNB,
                                  uint8_t sect_id);



/* 基站SRS参数
@param phy_vars_eNB 基站物理层变量
@param eNB_id 基站ID
@param UE_id 用户ID
@param init_averaging 初始化平均值函数？？？
// 没有使用
*/
void lte_eNB_srs_measurements(PHY_VARS_eNB *phy_vars_eNBy,
                              module_id_t eNB_id,
                              module_id_t UE_id,
                              unsigned char init_averaging);


/* 频域均衡技术
@param frame_parms 帧结构
@param rxdataF_comp 频域接收端符合数据
@param ul_ch_mag 上行信道
@param ul_ch_mag_b 上行信道BR?
@param symbol 帧中的符号数
@param Msc_RS
@param Qm
*/
void freq_equalization(LTE_DL_FRAME_PARMS *frame_parms,
                       int **rxdataF_comp,
                       int **ul_ch_mag,
                       int **ul_ch_mag_b,
                       unsigned char symbol,
                       unsigned short Msc_RS,
                       unsigned char Qm);


/** @} */
#endif
