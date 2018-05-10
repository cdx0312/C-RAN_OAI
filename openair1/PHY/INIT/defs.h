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

#ifndef __INIT_DEFS__H__
#define __INIT_DEFS__H__

#include "PHY/defs.h"


#include "SystemInformationBlockType2.h"
//#include "RadioResourceConfigCommonSIB.h"
#include "RadioResourceConfigDedicated.h"
#include "TDD-Config.h"
#include "PHICH-Config.h"
#include "MBSFN-SubframeConfigList.h"
#include "MobilityControlInfo.h"
#if defined(Rel10) || defined(Rel14)
#include "SCellToAddMod-r10.h"
#endif
/** @addtogroup _PHY_STRUCTURES_
 * @{
 */

/*!
\fn int l1_top_init_eNB(void)
\brief Initialize north interface for L1
@returns 0 on success
 */
// 初始化基站L1层北向接口，在lte_init.c中
int l1_north_init_eNB(void);

/*!
\fn int phy_init_top(LTE_DL_FRAME_PARMS *frame_parms)
\brief Allocate and Initialize the PHY variables after receiving static configuration
@param frame_parms Pointer to LTE_DL_FRAME_PARMS (common configuration)
@returns 0 on success
 */
 // 接收配置文件之后，初始化物理层变量，恩，这个函数没有找到，可能不用了
int phy_init_top(LTE_DL_FRAME_PARMS *frame_parms);


/*!
\brief Allocate and Initialize the PHY variables relevant to the LTE ue signal buffers.
\details Only a subset of phy_vars_ue is initialized.
@param[out] phy_vars_ue Pointer to UE Variables
@param nb_connected_eNB Number of eNB that UE can process in one PDSCH demodulation subframe
@param abstraction_flag 1 indicates memory should be allocated for abstracted MODEM
@returns 0 on success
@returns -1 if any memory allocation failed
@note The current implementation will never return -1, but segfault.
 */
 // 初始化用户侧与UE信号缓冲相关物理层变量， lte_init_ue.c中
int init_lte_ue_signal(PHY_VARS_UE *phy_vars_ue,
			   int          nb_connected_eNB,
			   uint8_t         abstraction_flag);

/*!
\brief Allocate and initialize the PHY variables releated to the transport channel buffers (UL/DL)
@param ue Pointer to UE L1 context
@param abstraction flag Indicates that abstraction is used in L1
*/
// 初始化用户侧传输相关的物理层变量， lte_init_ue.c中
void init_lte_ue_transport(PHY_VARS_UE *ue,int absraction_flag);

/*!
\brief Allocate and initialize the PHY variables relevant to the LTE implementation (eNB).
\details Only a subset of phy_vars_eNb is initialized.
@param[out] phy_vars_eNb Pointer to eNB Variables
@param is_secondary_eNb Flag to indicate this eNB gets synch from another
@param abstraction_flag 1 indicates memory should be allocated for abstracted MODEM
@returns 0 on success
@returns -1 if any memory allocation failed
@note The current implementation will never return -1, but segfault.
 */
 // 初始化基站侧的物理层变量， lte_init.c中
int phy_init_lte_eNB(PHY_VARS_eNB *phy_vars_eNb,
                     unsigned char is_secondary_eNb,
                     unsigned char abstraction_flag);

/*!
\brief Free the PHY variables relevant to the LTE implementation (eNB).
\details Only a subset of phy_vars_eNb is freed (those who have been allocated with phy_init_lte_eNB()).
@param[in] phy_vars_eNb Pointer to eNB Variables
 */
 // 释放基站侧的物理层变量， lte_init.c中
void phy_free_lte_eNB(PHY_VARS_eNB *phy_vars_eNb);

/** \brief Configure LTE_DL_FRAME_PARMS with components derived after initial synchronization (MIB decoding + primary/secondary synch).
\details The basically allows configuration of \f$N_{\mathrm{RB}}^{\mathrm{DL}}\f$, the cell id  \f$N_{\mathrm{ID}}^{\mathrm{cell}}\f$, the normal/extended prefix mode, the frame type (FDD/TDD), \f$N_{\mathrm{cp}}\f$, the number of TX antennas at eNB (\f$p\f$) and the number of PHICH groups, \f$N_{\mathrm{group}}^{\mathrm{PHICH}}\f$
@param lte_frame_parms pointer to LTE parameter structure
@param N_RB_DL Number of DL resource blocks
@param Nid_cell Cell ID
@param Ncp Normal/Extended Prefix flag
@param p_eNB Number of eNB TX antennas
@param phich_config Pointer to PHICH_CONFIG_COMMON
 */
 // MIB解码之后 配置LTE下行帧结构的相关参数
void phy_config_mib_eNB(int                    Mod_id,
			int                    CC_id,
			int                    eutra_band,
			int                    N_RB_DL,
			PHICH_Config_t         *phich_config,
			int                    Nid_cell,
			int                    Ncp,
			int                    p_eNB,
			uint32_t               dl_CarrierFreq,
			uint32_t               ul_CarrierFreq);



/** \brief Configure LTE_DL_FRAME_PARMS with components derived after reception of SIB1.
\details From a PHY perspective this allows configuration of TDD framing parameters and SI reception.
@param Mod_id Instance ID of eNB
@param CC_id Component Carrier index
@param tdd_Config TDD UL/DL and S-subframe configurations
@param SIwindowsize Size of a SI window in frames where repetitions of a unique System Information message block is repeated
@param SIperiod Periodicity of System Information Messages (in multiples of a frame)*/
// 基站侧接收到SIB1之后，配置下行帧结构参数
void phy_config_sib1_eNB(module_id_t    Mod_id,
                         int CC_id,
                         TDD_Config_t  *tdd_Config,
                         uint8_t           SIwindowsize,
                         uint16_t            SIperiod);

/** \brief Configure LTE_DL_FRAME_PARMS with components derived after reception of SIB1.
\details From a PHY perspective this allows configuration of TDD framing parameters and SI reception.
@param Mod_id Instance ID of UE
@param CC_id Component Carrier index
@param CH_index Index of eNB for this configuration
@param tdd_Config TDD UL/DL and S-subframe configurations
@param SIwindowsize Size of a SI window in frames where repetitions of a unique System Information message block is repeated
@param SIperiod Periodicity of System Information Messages (in multiples of a frame)*/
// 用户侧接收到SIB1之后，配置下行帧结构参数
void phy_config_sib1_ue(module_id_t   Mod_id,
                        int CC_id,
                        uint8_t          CH_index,
                        TDD_Config_t *tdd_Config,
                        uint8_t          SIwindowsize,
                        uint16_t           SIperiod);


/*!
  \fn void phy_config_sib2_ue(module_id_t Mod_id,uint8_t CC_id,uint8_t CH_index,
      RadioResourceConfigCommonSIB_t *radioResourceConfigCommon,
      ARFCN_ValueEUTRA_t *ul_CArrierFreq,
            long *ul_Bandwidth,
      AdditionalSpectrumEmission_t additionalSpectrumEmission,
      struct MBSFN_SubframeConfigList *mbsfn_SubframeConfigList)
  \brief Configure LTE_DL_FRAME_PARMS with components derived after reception of SIB2 (at UE).
  @param Mod_id Instance id
  @param CC_id
  @param CH_index Index of CH to which UE is connected
  @param CC_id Component Carrier Index
  @param radioResourceConfigCommon Radio Configuration from SIB2
  @param ul_CarrierFreq UL carrier ARFCN, null if optional (i.e. implicit from DL)
  @param ul_Bandwidth UL bandwidth, null if optional (i.e. same as DL)
  @param additionalSpectrumEmission UL parameter (see 36.101)
  @param mbsfn_SubframeConfigList MBSFN subframe configuration
 */
 // 基站侧接收到SIB2之后，配置下行帧结构参数
void phy_config_sib2_ue(module_id_t                     Mod_id,
                        int                         CC_id,
                        uint8_t                         CH_index,
                        RadioResourceConfigCommonSIB_t  *radioResourceConfigCommon,
                        ARFCN_ValueEUTRA_t              *ul_CArrierFreq,
                        long                            *ul_Bandwidth,
                        AdditionalSpectrumEmission_t    *additionalSpectrumEmission,
                        struct MBSFN_SubframeConfigList *mbsfn_SubframeConfigList);


/*!
  \fn phy_config_afterHO_ue
  \brief Configure Common PHY parameters from mobilityControlInfo
  @param Mod_id
  @param CC_id
  @param eNB_index
  @param mobilityControlInfo pointer to the mobility control information for handover
  @param ho_failed flag to indicated whether the ho was successful or not
 */
 // 配置物理层移动性管理相关的参数
void phy_config_afterHO_ue(module_id_t Mod_id,
                           uint8_t CC_id,
                           uint8_t eNB_index,
                           MobilityControlInfo_t *mobilityControlInfo,
                           uint8_t ho_failed);
/*!
  \fn void phy_config_sib2_eNB(module_id_t Mod_id,
                               RadioResourceConfigCommonSIB_t *radioResourceConfigCommon,
             ARFCN_ValueEUTRA_t *ul_CArrierFreq,
             long *ul_Bandwidth,
             AdditionalSpectrumEmission_t additionalSpectrumEmission,
             struct MBSFN_SubframeConfigList  *mbsfn_SubframeConfigList)
  \brief Configure LTE_DL_FRAME_PARMS with components of SIB2 (at eNB).
  @param Mod_id Instance id
  @param Mod_id Component Carrier index
  @param radioResourceConfigCommon Radio Configuration from SIB2
  @param ul_CarrierFreq UL carrier ARFCN, null if optional (i.e. implicit from DL)
  @param ul_Bandwidth UL bandwidth, null if optional (i.e. same as DL)
  @param additionalSpectrumEmission UL parameter (see 36.101)
  @param mbsfn_SubframeConfigList MBSFN subframe configuration
 */
 // 基站侧接收到SIB2之后，配置下行帧结构参数
void phy_config_sib2_eNB(module_id_t                            Mod_id,
                         int                                CC_id,
                         RadioResourceConfigCommonSIB_t         *radioResourceConfigCommon,
                         ARFCN_ValueEUTRA_t                     *ul_CArrierFreq,
                         long                                   *ul_Bandwidth,
                         AdditionalSpectrumEmission_t           *additionalSpectrumEmission,
                         struct MBSFN_SubframeConfigList        *mbsfn_SubframeConfigList);


/*!
\fn void phy_config_dedicated_ue(module_id_t Mod_id,uint8_t CC_id,uint8_t CH_index,
               struct PhysicalConfigDedicated *physicalConfigDedicated)
\brief Configure UE dedicated parameters.
\details Invoked upon reception of RRCConnectionSetup or RRCConnectionReconfiguration from eNB.
@param Mod_id Instance ID for eNB
@param CC_id Component Carrier index
@param CH_index Index of eNB for this configuration
@param physicalConfigDedicated PHY Configuration information

 */
 // 配置用户dedicated 参数
void phy_config_dedicated_ue(module_id_t Mod_id,
                             int CC_id,
                             uint8_t CH_index,
                             struct PhysicalConfigDedicated *physicalConfigDedicated);

/*!
\fn void phy_config_harq_ue(module_id_t Mod_id,uint8_t CC_id,uint8_t CH_index,
               uint16_t max_harq_tx)
\brief Configure UE UL max harq Tx.
\details Invoked upon reception of RRCConnectionSetup or RRCConnectionReconfiguration from eNB.
@param Mod_id Instance ID for eNB
@param CC_id Component Carrier index
@param CH_index Index of eNB for this configuration
@param max_harq_tx max harq tx information

 */
 // 配置用户侧上行最大重传
void phy_config_harq_ue(module_id_t Mod_id,int CC_id,uint8_t CH_index,
                           uint16_t max_harq_tx);
/**
\brief Configure UE MBSFN common parameters.
\details Invoked upon reception of SIB13 from eNB.
@param Mod_id Instance ID for UE
@param CC_id Component Carrier Index
@param CH_index eNB id (for multiple eNB reception)
@param mbsfn_Area_idx Index of MBSFN-Area for which this command operates
@param mbsfn_AreaId_r9 MBSFN-Area Id
 */

// 配置用户侧MBSFN通用参数
void phy_config_sib13_ue(module_id_t Mod_id,
                         int CC_id,uint8_t CH_index,int mbsfn_Area_idx,
                         long mbsfn_AreaId_r9);

/**
\brief Configure eNB MBSFN common parameters.
\details Invoked upon transmission of SIB13 from eNB.
@param Mod_id Instance ID for eNB
@param CC_id Component Carrier index
@param mbsfn_Area_idx Index of MBSFN-Area for which this command operates
@param mbsfn_AreaId_r9 MBSFN-Area Id
 */
 // 配置基站MBSFN通用参数
void phy_config_sib13_eNB(module_id_t Mod_id,
                          int CC_id,
                          int mbsfn_Area_idx,
                          long mbsfn_AreaId_r9);

/**
\brief Configure cba rnti for .
@param Mod_id Instance ID for eNB
@param CC_id Component Carrier Index
@param eNB_flag flag indicating whether the nodeis eNB (1) or UE (0)
@param index index of the node
@param cba_rnti rnti for the cba transmission
@param num_active_cba_groups num active cba group
 */
 // 配置cba rnti
void  phy_config_cba_rnti (module_id_t Mod_id,int CC_id,eNB_flag_t eNB_flag, uint8_t index, rnti_t cba_rnti, uint8_t cba_group_id, uint8_t num_active_cba_groups);

/** \brief Configure RRC inter-cell measurements procedures
@param Mod_id Index of UE
@param CC_id
@param eNB_index Index of corresponding eNB
@param n_adj_cells Number of adjacent cells on which to perform the measuremnts
@param adj_cell_id Array of cell ids of adjacent cells
 */
 // 配置RRC小区内估计过程
void phy_config_meas_ue(module_id_t Mod_id,
                        uint8_t CC_id,
                        uint8_t eNB_index,
                        uint8_t n_adj_cells,
                        uint32_t *adj_cell_id);

/*!
\fn void phy_config_dedicated_eNB(module_id_t Mod_id,uint16_t rnti,
                                  struct PhysicalConfigDedicated *physicalConfigDedicated)
\brief Prepare for configuration of PHY with dedicated parameters.
\details Invoked just prior to transmission of RRCConnectionSetup or RRCConnectionReconfiguration at eNB.
@param Mod_id Instance ID for eNB
@param CC_id Component Carrier index
@param rnti rnti for UE context
@param physicalConfigDedicated PHY Configuration information
 */
 // 基站侧特定数据配置
void phy_config_dedicated_eNB(module_id_t Mod_id,
                              int CC_id,
                              rnti_t rnti,
                              struct PhysicalConfigDedicated *physicalConfigDedicated);

/*!
\fn void phy_config_dedicated_eNB_step2(PHY_VARS_eNB *phy_vars_eNB)
\brief Configure PHY with dedicated parameters between configuration of DLSCH (n) and ULSCH (n+4) in current subframe (n).
@param phy_vars_eNB Pointer to PHY_VARS_eNB structure
 */
 // 基站侧特定数据配置2
void phy_config_dedicated_eNB_step2(PHY_VARS_eNB *phy_vars_eNB);

/*
  \fn int phy_init_secsys_eNB(PHY_VARS_eNB *phy_vars_eNb)
\brief Allocate and Initialize the PHY variables relevant to the LTE implementation.
@param phy_vars_eNb pointer to LTE parameter structure for the eNb
 */
 // 没有找到,不知道怎么初始化
int phy_init_secsys_eNB(PHY_VARS_eNB *phy_vars_eNb);

// 释放LTE占用的资源
void free_lte_top(void);

// 初始化LTE主要参数,内部嵌套了很多函数
void init_lte_top(LTE_DL_FRAME_PARMS *lte_frame_parms);

//void copy_lte_parms_to_phy_framing(LTE_DL_FRAME_PARMS *frame_parm, PHY_FRAMING *phy_framing);

// 初始化LTE下行帧结构中的参数
void lte_param_init(PHY_VARS_eNB **eNBp,
		    PHY_VARS_UE **UEp,
		    RU_t **rup,
		    unsigned char N_tx_port_eNB,
                    unsigned char N_tx_phy,
		    unsigned char N_rx_ru,
		    unsigned char N_rx_ue,
		    unsigned char transmission_mode,
		    uint8_t extended_prefix_flag,
		    frame_t frame_type,
		    uint16_t Nid_cell,
		    uint8_t tdd_config,
		    uint8_t N_RB_DL,
		    uint8_t pa,
		    uint8_t threequarter_fs,
                    uint8_t osf,
		    uint32_t perfect_ce);


#if defined(Rel10) || defined(Rel14)
void phy_config_dedicated_scell_ue(uint8_t Mod_id,
                                   uint8_t eNB_index,
                                   SCellToAddMod_r10_t *sCellToAddMod_r10,
                                   int CC_id);

void phy_config_dedicated_scell_eNB(uint8_t Mod_id,
                                    uint16_t rnti,
                                    SCellToAddMod_r10_t *sCellToAddMod_r10,
                                    int CC_id);

#endif



/*! !\fn void phy_cleanup(void)
\brief Cleanup the PHY variables*/
// 清除物理层变量
void phy_cleanup(void);

// 物理层配置请求初始化
void phy_config_request(PHY_Config_t *phy_config);

// 初始化帧结构数据
int init_frame_parms(LTE_DL_FRAME_PARMS *frame_parms,uint8_t osf);
// 记录帧结构数据
void dump_frame_parms(LTE_DL_FRAME_PARMS *frame_parms);

/** @} */
#endif
