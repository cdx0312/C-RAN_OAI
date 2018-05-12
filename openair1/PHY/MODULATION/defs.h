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

#ifndef __MODULATION_DEFS__H__
#define __MODULATION_DEFS__H__
#include "PHY/defs.h"
/** @addtogroup _PHY_MODULATION_
 * @{
*/

/**
\fn void PHY_ofdm_mod(int *input,int *output,int fftsize,unsigned char nb_symbols,unsigned short nb_prefix_samples,Extension_t etype)
This function performs OFDM modulation with cyclic extension or zero-padding.

@param input The sequence input samples in the frequency-domain.  This is a concatenation of the input symbols in SIMD redundant format
@param output The time-domain output signal
@param fftsize size of OFDM symbol size (\f$N_d\f$)
@param nb_symbols The number of OFDM symbols in the block
@param nb_prefix_samples The number of prefix/suffix/zero samples
@param etype Type of extension (CYCLIC_PREFIX,CYCLIC_SUFFIX,ZEROS)
*/

/** 物理层OFDM调制，结尾以循环冗余校验位或者0
@param input 频域输入的样本
@param output 时域输出信号
@param fftsize OFDM符号的长度
@param nb_symbols 一个资源块中OFDM符号的数量
@param nb_prefix_samples 前缀、后缀、0的样本个数
@param etype 扩展类型，0、循环前缀或者循环后缀
*/
void PHY_ofdm_mod(int *input,
                  int *output,
                  int fftsize,
                  unsigned char nb_symbols,
                  unsigned short nb_prefix_samples,
                  Extension_t etype
                 );

#ifdef OPENAIR_LTE

/*!
\brief This function implements the OFDM front end processor on reception (FEP)
\param phy_vars_ue Pointer to PHY variables
\param l symbol within slot (0..6/7)
\param Ns Slot number (0..19)
\param sample_offset offset within rxdata (points to beginning of subframe)
\param no_prefix if 1 prefix is removed by HW
\param reset_freq_est if non-zero it resets the frequency offset estimation loop
*/
/*! FEP， OFDM符号接收过程中的前端处理器
\param phy_vars_ue 指向用户侧物理层变量
\param l 时隙内的符号数
\param Ns 时隙序号(0..19)
\param sample_offset rxdata中的偏移量，子帧的位置
\param no_prefix 1表示没有prefix
\param reset_freq_est 不为0，则重置循环估计的频偏
*/
int slot_fep(PHY_VARS_UE *phy_vars_ue,
             unsigned char l,
             unsigned char Ns,
             int sample_offset,
             int no_prefix,
	     int reset_freq_est);

/*! 前端处理器的多播组播单频网络过程
\param phy_vars_ue 指向用户侧物理层变量
\param l 时隙内的符号数
\param sample_offset rxdata中的偏移量，子帧的位置
\param no_prefix 1表示没有prefix
*/
int slot_fep_mbsfn(PHY_VARS_UE *phy_vars_ue,
                   unsigned char l,
                   int subframe,
                   int sample_offset,
                   int no_prefix);

/*! FEP上行过程
\param ru RU节点的数据
\param l 时隙内的符号数
\param Ns 时隙序号(0..19)
\param no_prefix 1表示没有prefix
*/
int slot_fep_ul(RU_t *ru,
                unsigned char l,
                unsigned char Ns,
                int no_prefix);

/*! 前端FFT过程
\param phy_vars_ue 指向用户侧物理层变量
\param l 时隙内的符号数
\param Ns 时隙序号(0..19)
\param sample_offset rxdata中的偏移量，子帧的位置
\param no_prefix 1表示没有prefix
*/
int front_end_fft(PHY_VARS_UE *ue,
             unsigned char l,
             unsigned char Ns,
             int sample_offset,
             int no_prefix);

/*! 前端信道估计过程？？？
\param phy_vars_ue 指向用户侧物理层变量
\param l 时隙内的符号数
\param Ns 时隙序号(0..19)
\param reset_freq_est 重置信道估计的频率
*/
int front_end_chanEst(PHY_VARS_UE *ue,
             unsigned char l,
             unsigned char Ns,
            int reset_freq_est);

/** 正常前缀调制
    @param txdataF 频域接收的数据指针表
    @param txdata 时域接收的数据指针表
    @param nsymb
    @param frame_parms 帧结构
*/
void normal_prefix_mod(int32_t *txdataF,
                       int32_t *txdata,
                       uint8_t nsymb,
                       LTE_DL_FRAME_PARMS *frame_parms);

/** OFDM调制过程
    @param txdataF 频域接收的数据指针表
    @param txdata 时域接收的数据指针表
    @param frame 帧索引
    @param next_slot 下一个时隙
    @param frame_parms 帧结构
*/
void do_OFDM_mod(int32_t **txdataF,
                 int32_t **txdata,
                 uint32_t frame,
                 uint16_t next_slot,
                 LTE_DL_FRAME_PARMS *frame_parms);


/** OFDM符号调制
    @param eNB_common_vars 基站通用的变量结构
    @param common RU通用的变量结构
    @param next_slot 下一个时隙
    @param frame_parms 帧结构
    @param do_precoding 是否做预编码
    实际上这个函数并没有在工程中调用
*/
void do_OFDM_mod_symbol(LTE_eNB_COMMON *eNB_common_vars,
                        RU_COMMON *common,
                        uint16_t next_slot,
                        LTE_DL_FRAME_PARMS *frame_parms,
                        int do_precoding);


/** 频域偏移7.5K
    @param ru RU数据结构
    @param subframe 子帧的位置
*/
void remove_7_5_kHz(RU_t *ru,
                    uint8_t subframe);

/** 应用7.5k
    @param phy_vars_ue 用户物理层变量
    @param txdata 时域接收的数据指针表
    @param subframe 子帧的位置
*/
void apply_7_5_kHz(PHY_VARS_UE *phy_vars_ue,
                   int32_t*txdata,
                   uint8_t subframe);
/* 初始化625的PRACH信道
@param frame_parms 帧结构
*/
void init_prach625(LTE_DL_FRAME_PARMS *frame_parms);

// 移除信道
void remove_625_Hz(PHY_VARS_eNB *phy_vars_eNB,
                   int16_t *prach);

void apply_625_Hz(PHY_VARS_UE *phy_vars_ue,
                  int16_t *prach);

/** \brief This function performs beamforming precoding for common
 * data
    @param txdataF Table of pointers for frequency-domain TX signals
    @param txdataF_BF Table of pointers for frequency-domain TX signals
    @param frame_parms Frame descriptor structure
after beamforming
    @param beam_weights Beamforming weights applied on each
antenna element and each carrier
    @param slot Slot number
    @param symbol Symbol index on which to act
    @param aa physical antenna index*/
/* 波束赋形预编码
@param txdataF 发送频域数据
@param txdataF_BF 发送频域数据波束赋形之后
@param frame_parms 下行帧结构
@param beam_weights 波束权重矩阵
@param slot 时隙
@param symbol 符号
@param aa
*/
int beam_precoding(int32_t **txdataF,
	           int32_t **txdataF_BF,
                   LTE_DL_FRAME_PARMS *frame_parms,
	           int32_t ***beam_weights,
                   int slot,
                   int symbol,
                   int aa);

/* 读取文件
@param calibF_fname 读取的文件名
@param nb_ant 天线数
@param nb_freq 频率数
@param tdd_calib_coeffs tdd校准系数
*/
int f_read(char *calibF_fname,
           int nb_ant,
           int nb_freq,
           int32_t **tdd_calib_coeffs);

/* 通过上行信道状态信息来估算下行信道状态信息
@param calib_dl_ch_estimates 估计的下行信道校准因子
@param ul_ch_estimates 上行信道的估计值
@param tdd_calib_coeffs tdd的校准相关系数
@param nb_ant 天线数 ？？
@param nb_freq 频率数 ？？
// 实际上，这个函数是空的，是否需要使用还不知道
*/
int estimate_DLCSI_from_ULCSI(int32_t **calib_dl_ch_estimates,
                              int32_t **ul_ch_estimates,
                              int32_t **tdd_calib_coeffs,
                              int nb_ant,
                              int nb_freq);

/* 计算波束赋形权重
@param beam_weights 波束权重矩阵
@param calib_dl_ch_estimates 估计的下行信道校准因子
@param precode_type 预编码类型
@param nb_ant 天线数 ？？
@param nb_freq 频率数 ？？
// 实际上，这个函数是空的，具体实现为空
*/
int compute_BF_weights(int32_t **beam_weights,
                       int32_t **calib_dl_ch_estimates,
                       PRECODE_TYPE_t precode_type,
                       int nb_ant,
                       int nb_freq);


#endif
/** @}*/
#endif
