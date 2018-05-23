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

/*
  \author R. Knopp, F. Kaltenberger
  \company EURECOM
  \email knopp@eurecom.fr
*/

// L1层调度相关的头文件

#ifndef __openair_SCHED_H__
#define __openair_SCHED_H__

#include "PHY/defs.h"
#include "PHY_INTERFACE/defs.h"

// 线程索引枚举类
enum THREAD_INDEX {
  OPENAIR_THREAD_INDEX = 0,
  TOP_LEVEL_SCHEDULER_THREAD_INDEX,
  DLC_SCHED_THREAD_INDEX,
  openair_SCHED_NB_THREADS
}; // do not modify this line


#define OPENAIR_THREAD_PRIORITY        255


#define OPENAIR_THREAD_STACK_SIZE     PTHREAD_STACK_MIN //4096 //RTL_PTHREAD_STACK_MIN*6
//#define DLC_THREAD_STACK_SIZE        4096 //DLC stack size
//#define UE_SLOT_PARALLELISATION

//调度状态枚举类，stopped,starting,started,stopping
enum openair_SCHED_STATUS {
  openair_SCHED_STOPPED=1,
  openair_SCHED_STARTING,
  openair_SCHED_STARTED,
  openair_SCHED_STOPPING
};

// 错误类型枚举类：
enum openair_ERROR {
  // HARDWARE CAUSES
  // 连接建立过程中的错误
  openair_ERROR_HARDWARE_CLOCK_STOPPED= 1,

  // SCHEDULER CAUSE
  // 调度引起的错误
  openair_ERROR_OPENAIR_RUNNING_LATE,
  openair_ERROR_OPENAIR_SCHEDULING_FAILED,

  // OTHERS
  // 其他错误
  openair_ERROR_OPENAIR_TIMING_OFFSET_OUT_OF_BOUNDS,
};

// 同步装填枚举类：未同步，已同步，退出
enum openair_SYNCH_STATUS {
  openair_NOT_SYNCHED=1,
  openair_SYNCHED,
  openair_SCHED_EXIT
};

// HARQ 类型枚举类，上行，下行，RA
enum openair_HARQ_TYPE {
  openair_harq_DL = 0,
  openair_harq_UL,
  openair_harq_RA
};

#define DAQ_AGC_ON 1
#define DAQ_AGC_OFF 0


/** @addtogroup _PHY_PROCEDURES_
 * @{
 */



/*! \brief Top-level entry routine for eNB procedures.  Called every slot by process scheduler. In even slots, it performs RX functions from previous subframe (if required).  On odd slots, it generate TX waveform for the following subframe.
  @param subframe Index of current subframe (0-9)
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
  @param *phy_vars_rn pointer to RN variables
*/
/* 基站物理层调度过程
@param subframe 子帧下标
@param phy_vars_eNB 基站物理层变量
@param abstraction_flag 物理层是否抽象标识
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
@param *phy_vars_rn RN 物理层变量
没有使用
*/
void phy_procedures_eNB_lte(uint8_t subframe,
                            PHY_VARS_eNB **phy_vars_eNB,
                            uint8_t abstraction_flag,
                            relaying_type_t r_type,
                            PHY_VARS_RN *phy_vars_rn);

/*! \brief Top-level entry routine for UE procedures.  Called every slot by process scheduler. In even slots, it performs RX functions from previous subframe (if required).  On odd slots, it generate TX waveform for the following subframe.
  @param phy_vars_ue Pointer to UE variables on which to act
  @param eNB_id ID of eNB on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param mode calibration/debug mode
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
  @param *phy_vars_rn pointer to RN variables
*/
/* UE物理层调度过程
@param phy_vars_ue UE物理层变量
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标识
@param mode 运行模式
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
@param *phy_vars_rn RN 物理层变量
*/
void phy_procedures_UE_lte(PHY_VARS_UE *phy_vars_ue,
                           UE_rxtx_proc_t *proc,
                           uint8_t eNB_id,
                           uint8_t abstraction_flag,
                           uint8_t do_pdcch_flag,
                           runmode_t mode,
                           relaying_type_t r_type,PHY_VARS_RN *phy_vars_rn);

#if defined(Rel10) || defined(Rel14)
/*! \brief Top-level entry routine for relay node procedures when acting as eNB. This proc will make us of the existing eNB procs.
  @param last_slot Index of last slot (0-19)
  @param next_slot Index of next_slot (0-19)
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* 中继节点的物理层过程
@param last_slot 上一个时隙的下标
@param next_slot 下一个时隙的下标
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
没有使用
*/
int phy_procedures_RN_eNB_TX(unsigned char last_slot,
                             unsigned char next_slot,
                             relaying_type_t r_type);

/*! \brief Top-level entry routine for relay node procedures actinf as UE. This proc will make us of the existing UE procs.
  @param last_slot Index of last slot (0-19)
  @param next_slot Index of next_slot (0-19)
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* 中继节点UE侧物理层过程
@param last_slot 上一个时隙的下标
@param next_slot 下一个时隙的下标
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
int phy_procedures_RN_UE_RX(unsigned char last_slot, unsigned char next_slot, relaying_type_t r_type);
#endif

/*! \brief Scheduling for UE TX procedures in normal subframes.
  @param phy_vars_ue Pointer to UE variables on which to act
  @param proc Pointer to RXn-TXnp4 proc information
  @param eNB_id Local id of eNB on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param mode calib/normal mode
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/

/* UE侧调度过程
@param phy_vars_ue UE物理层变量
@param proc UE收发过程信息
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标志
@param mode 运行模式 cab、normal
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
void phy_procedures_UE_TX(PHY_VARS_UE *phy_vars_ue,
                          UE_rxtx_proc_t *proc,
                          uint8_t eNB_id,
                          uint8_t abstraction_flag,
                          runmode_t mode,
                          relaying_type_t r_type);

/*! \brief Scheduling for UE RX procedures in normal subframes.
  @param last_slot Index of last slot (0-19)
  @param phy_vars_ue Pointer to UE variables on which to act
  @param proc Pointer to RXn_TXnp4 proc information
  @param eNB_id Local id of eNB on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param mode calibration/debug mode
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
  @param phy_vars_rn pointer to RN variables
*/
/* UE侧接收调度过程
@param last_slot 上一个时隙的下标
@param phy_vars_ue UE物理层变量
@param proc UE收发过程信息
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标记
@param mode 运行模式 calibration/debug mode
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
@param phy_vars_rn 中继物理层变量
*/
int phy_procedures_UE_RX(PHY_VARS_UE *phy_vars_ue,
                         UE_rxtx_proc_t *proc,
                         uint8_t eNB_id,
                         uint8_t abstraction_flag,
                         uint8_t do_pdcch_flag,
                         runmode_t mode,
                         relaying_type_t r_type,
                         PHY_VARS_RN *phy_vars_rn);

/* UE侧并行接收时隙处理过程
@param ue UE物理层变量
@param proc UE收发过程信息
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标记
@param do_pdcch_flag PDCCH标识
@param mode 运行模式 calibration/debug mode
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
@param phy_vars_rn 中继物理层变量
*/
int phy_procedures_slot_parallelization_UE_RX(PHY_VARS_UE *ue,
                                              UE_rxtx_proc_t *proc,
                                              uint8_t eNB_id,
                                              uint8_t abstraction_flag,
                                              uint8_t do_pdcch_flag,
                                              runmode_t mode,
                                              relaying_type_t r_type,
                                              PHY_VARS_RN *phy_vars_rn);

#ifdef UE_SLOT_PARALLELISATION
/* 用户线程时隙1下行处理函数
*/
void *UE_thread_slot1_dl_processing(void *arg);
#endif

/*! \brief Scheduling for UE TX procedures in TDD S-subframes.
  @param phy_vars_ue Pointer to UE variables on which to act
  @param eNB_id Local id of eNB on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* TDD同步子帧下，UE侧发送调度过程
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标识
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
void phy_procedures_UE_S_TX(PHY_VARS_UE *phy_vars_ue,
                            uint8_t eNB_id,
                            uint8_t abstraction_flag,
                            relaying_type_t r_type);

/*! \brief Scheduling for UE RX procedures in TDD S-subframes.
  @param phy_vars_ue Pointer to UE variables on which to act
  @param eNB_id Local id of eNB on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* TDD同步子帧下UE侧接收调度过程
@param phy_vars_ue 用户侧物理层变量
@param eNB_id 基站ID
@param abstraction_flag 物理层抽象标识
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
只有函数声明，没有实现
*/
void phy_procedures_UE_S_RX(PHY_VARS_UE *phy_vars_ue,uint8_t eNB_id,uint8_t abstraction_flag, relaying_type_t r_type);

/*! \brief Scheduling for eNB TX procedures in normal subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param abstraction_flag Indicator of PHY abstraction
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
  @param phy_vars_rn pointer to the RN variables
  @param do_meas Do inline timing measurement
*/
/* 正常子帧下基站侧发送过程调度
@param phy_vars_eNB 基站侧物理层变量
@param abstraction_flag 物理层抽象标识
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
@param phy_vars_rn 中继网络物理层变量
@param do_meas 时序计算timing measurement
*/
void phy_procedures_eNB_TX(PHY_VARS_eNB *phy_vars_eNB,
                           eNB_rxtx_proc_t *proc,
                           relaying_type_t r_type,
                           PHY_VARS_RN *phy_vars_rn,
                           int do_meas);

/*! \brief Scheduling for eNB RX UE-specific procedures in normal subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param proc Pointer to RXn-TXnp4 proc information
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* 基站侧接收UE专有信号调度过程
@param phy_vars_eNB 基站物理层变量
@param proc rxtx收发信息
@param r_type 中继类型: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
void phy_procedures_eNB_uespec_RX(PHY_VARS_eNB *phy_vars_eNB,
                                  eNB_rxtx_proc_t *proc,
                                  relaying_type_t r_type);

/*! \brief Scheduling for eNB TX procedures in TDD S-subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param proc Pointer to RXn-TXnp4 proc information
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/

/*! \brief Scheduling for eNB RX common procedures in normal subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param abstraction_flag Indicator of PHY abstraction
*/
/* 基站侧接收通用调度过程
@param phy_vars_eNB 基站物理层变量
@param proc 基站收发过程数据
没有被调用过，也没有函数实现
*/
void phy_procedures_eNB_common_RX(PHY_VARS_eNB *phy_vars_eNB,
                                  eNB_rxtx_proc_t *proc);

/*! \brief Scheduling for eNB TX procedures in TDD S-subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* TDD同步子帧中，基站侧发送调度过程
@param phy_vars_eNB 基站物理层变量
@param r_type 中继类型
没有实现，没有调用
*/
void phy_procedures_eNB_S_TX(PHY_VARS_eNB *phy_vars_eNB,
                             relaying_type_t r_type);

/*! \brief Scheduling for eNB RX procedures in TDD S-subframes.
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param r_type indicates the relaying operation: 0: no_relaying, 1: unicast relaying type 1, 2: unicast relaying type 2, 3: multicast relaying
*/
/* TDD同步子帧中，基站侧接收调度过程
@param phy_vars_eNB 基站侧物理层变量
@param proc 基站侧收发过程数据
@param r_type 中继类型
没有实现，没有调用
*/
void phy_procedures_eNB_S_RX(PHY_VARS_eNB *phy_vars_eNB,
                             eNB_rxtx_proc_t *proc,
                             relaying_type_t r_type);

/*! \brief Scheduling for eNB PRACH RX procedures
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param br_flag indicator for eMTC PRACH
*/
#ifdef Rel14
/* 基站PRACH接收调度过程
@param eNB 基站侧物理层变量
@param br_flag PRACH BR标识
*/
void prach_procedures(PHY_VARS_eNB *eNB,
		      int br_flag);
#else
/* 基站PRACH接收调度过程
@param eNB 基站侧物理层变量
*/
void prach_procedures(PHY_VARS_eNB *eNB);
#endif

/*! \brief Function to compute subframe Number(DL and S) as a function of Frame type and TDD Configuration
  @param frame_parms Pointer to DL frame parameter descriptor
  @returns Subframe Number (DL,S)
*/
/* 子帧数量计算函数
@param frame_parms 帧结构
*/
int subframe_num(LTE_DL_FRAME_PARMS *frame_parms);

/*! \brief Function to compute subframe type as a function of Frame type and TDD Configuration (implements Table 4.2.2 from 36.211, p.11 from version 8.6) and subframe index.
  @param frame_parms Pointer to DL frame parameter descriptor
  @param subframe Subframe index
  @returns Subframe type (DL,UL,S)
*/

/* 计算子帧类型
@param frame_parms 帧结构
@param subframe 子帧下标
@returns 子帧类型(DL,UL,S)
*/
lte_subframe_t subframe_select(LTE_DL_FRAME_PARMS *frame_parms,
                               uint8_t subframe);


/*! \brief Function to compute which type of DCIs to detect in the given subframe
  @param frame_parms Pointer to DL frame parameter descriptor
  @param subframe Subframe index
  @returns DCI detetion mode type (no DCIs, uplink DCIs, downlink DCIs, both uplink and downlink DCIs)
 */
/* 检测子帧中的DCI类型
@param frame_parms 帧结构
@param subframe 子帧下标
@returns DCI检测类型(no DCIs, uplink DCIs, downlink DCIs, both uplink and downlink DCIs)
*/
dci_detect_mode_t dci_detect_mode_select(LTE_DL_FRAME_PARMS *frame_parms,
                                         uint8_t subframe);

/*! \brief Function to compute subframe type as a function of Frame type and TDD Configuration (implements Table 4.2.2 from 36.211, p.11 from version 8.6) and subframe index.  Same as subframe_select, except that it uses the Mod_id and is provided as a service to the MAC scheduler.
  @param Mod_id Index of eNB
  @param CC_id Component Carrier Index
  @param subframe Subframe index
  @returns Subframe type (DL,UL,S)
*/
/* 计算子帧类型
@param Mod_id 基站ID
@param CC_id 成员载波ID
@param subframe 子帧ID
@returns 子帧类型(DL,UL,S)
*/
lte_subframe_t get_subframe_direction(uint8_t Mod_id,
                                      uint8_t CC_id,
                                      uint8_t subframe);

/*! \brief Function to indicate PHICH transmission subframes.  Implements Table 9.1.2-1 for TDD.
  @param frame_parms Pointer to DL frame parameter descriptor
  @param subframe Subframe index
  @returns 1 if PHICH can be transmitted in subframe (always 1 for FDD)
*/
/* 判断当前之中呢是否为PHICH子帧
@param frame_parms 帧结构
@param subframe 子帧下标
@returns true-1，FDD一直为true
*/
uint32_t is_phich_subframe(LTE_DL_FRAME_PARMS *frame_parms,
                           uint8_t subframe);

/*! \brief Function to compute timing of Msg3 transmission on UL-SCH (first UE transmission in RA procedure). This implements the timing in paragraph a) from Section 6.1.1 in 36.213 (p. 17 in version 8.6).  Used by eNB upon transmission of random-access response (RA_RNTI) to program corresponding ULSCH reception procedure.  Used by UE upon reception of random-access response (RA_RNTI) to program corresponding ULSCH transmission procedure.  This does not support the UL_delay field in RAR (always assumed to be 0).
  @param frame_parms Pointer to DL frame parameter descriptor
  @param current_subframe Index of subframe where RA_RNTI was received
  @param current_frame Index of frame where RA_RNTI was received
  @param frame Frame index where Msg3 is to be transmitted (n+6 mod 10 for FDD, different for TDD)
  @param subframe subframe index where Msg3 is to be transmitted (n, n+1 or n+2)
*/
/* 计算Msg3上传输的时序同步信息
@param frame_parms 帧结构
@param current_subframe 当前子帧ID
@param current_frame 当前帧ID
@param frame Msg3传输时的帧ID
@param subframe Msg3传输时的子帧ID
*/
void get_Msg3_alloc(LTE_DL_FRAME_PARMS *frame_parms,
                    uint8_t current_subframe,
                    uint32_t current_frame,
                    uint32_t *frame,
                    uint8_t *subframe);

/*! \brief Function to compute timing of Msg3 retransmission on UL-SCH (first UE transmission in RA procedure).
  @param frame_parms Pointer to DL frame parameter descriptor
  @param current_subframe Index of subframe where RA_RNTI was received
  @param current_frame Index of frame where RA_RNTI was received
  @param frame Frame index where Msg3 is to be transmitted (n+6 mod 10 for FDD, different for TDD)
  @param subframe subframe index where Msg3 is to be transmitted (n, n+1 or n+2)
*/
/* 计算Msg3在ULSCH重传时的时序同步信息
@param frame_parms 帧结构
@param current_subframe 当前子帧ID
@param current_frame 当前帧ID
@param frame Msg3传输时的帧ID
@param subframe Msg3传输时的子帧ID
*/
void get_Msg3_alloc_ret(LTE_DL_FRAME_PARMS *frame_parms,
                        uint8_t current_subframe,
                        uint32_t current_frame,
                        uint32_t *frame,
                        uint8_t *subframe);

/*! \brief Get ULSCH harq_pid for Msg3 from RAR subframe.  This returns n+k mod 10 (k>6) and corresponds to the rule in Section 6.1.1 from 36.213
   @param frame_parms Pointer to DL Frame Parameters
   @param frame Frame index
   @param current_subframe subframe of RAR transmission
   @returns harq_pid (0 ... 7)
 */
 /* 从RAR子帧上获取上行共享信道的HARQ_PID
 @param frame_parms 帧结构
 @param frame 帧ID
 @param current_frame RAR传输是的子帧ID
 */
uint8_t get_Msg3_harq_pid(LTE_DL_FRAME_PARMS *frame_parms,
                          uint32_t frame,
                          uint8_t current_subframe);

/*! \brief Get ULSCH harq_pid from PHICH subframe
   @param frame_parms Pointer to DL Frame Parameters
   @param subframe subframe of PHICH
   @returns harq_pid (0 ... 7)
 */

/*! \brief Function to indicate failure of contention resolution or RA procedure.  It places the UE back in PRACH mode.
    @param Mod_id Instance index of UE
    @param CC_id Component Carrier Index
    @param eNB_index Index of eNB
 */
/* RA随机接入失败函数
@param Mod_id UE ID
@param CC_id 成员载波ID
@param eNB_index 基站ID
*/
void ra_failed(uint8_t Mod_id,
               uint8_t CC_id,
               uint8_t eNB_index);

/*! \brief Function to indicate success of contention resolution or RA procedure.
    @param Mod_id Instance index of UE
    @param CC_id Component Carrier Index
    @param eNB_index Index of eNB
 */
 /* RA随机接入成功函数
 @param Mod_id UE ID
 @param CC_id 成员载波ID
 @param eNB_index 基站ID
 */
void ra_succeeded(uint8_t Mod_id,
                  uint8_t CC_id,
                  uint8_t eNB_index);

/* 从重传指示信道指针中获取HARQ_pid
@param frame_parms 帧结构
@param frame 帧编号
@param subframe 子帧编号
*/
uint8_t phich_subframe_to_harq_pid(LTE_DL_FRAME_PARMS *frame_parms,
                                   uint32_t frame,
                                   uint8_t subframe);

/*! \brief Get PDSCH subframe (n+k) from PDCCH subframe n using relationship from Table 8-2 from 36.213
   @param frame_parms Pointer to DL Frame Parameters
   @param n subframe of PDCCH
   @returns PDSCH subframe (0 ... 7) (note: this is n+k from Table 8-2)
 */
/* 从PDCCH子帧n中获取PDSCH子帧
@param frame_parms 帧结构
@param n PDCCH子帧n
*/
uint8_t pdcch_alloc2ul_subframe(LTE_DL_FRAME_PARMS *frame_parms,
                                uint8_t n);


/*! \brief Compute ACK/NACK information for PUSCH/PUCCH for UE transmission in subframe n. This function implements table 10.1-1 of 36.213, p. 69.
  @param frame_parms Pointer to DL frame parameter descriptor
  @param harq_ack Pointer to dlsch_ue harq_ack status descriptor
  @param subframe Subframe for UE transmission (n in 36.213)
  @param o_ACK Pointer to ACK/NAK payload for PUCCH/PUSCH
  @returns status indicator for PUCCH/PUSCH transmission
*/
/* 计算子帧n中PUSCH、PUCCH的ACK、NACK信息
@param frame_parms 帧结构
@param harq_ack 下行共享信道 HARQ ack信息
@param subframe UE传输的子帧
@param o_ACK PUCCH/PUSCH的ack、nak信息
@returns PUCCH/PUSCH状态信息
*/
uint8_t get_ack(LTE_DL_FRAME_PARMS *frame_parms,
                harq_status_t *harq_ack,
                uint8_t subframe_tx,
                uint8_t subframe_rx,
                uint8_t *o_ACK,
                uint8_t cw_idx);

/*! \brief Reset ACK/NACK information
  @param frame_parms Pointer to DL frame parameter descriptor
  @param harq_ack Pointer to dlsch_ue harq_ack status descriptor
  @param subframe Subframe for UE transmission (n in 36.213)
  @param o_ACK Pointer to ACK/NAK payload for PUCCH/PUSCH
  @returns status indicator for PUCCH/PUSCH transmission
*/
/* 重置ACK，NACK信息
@param frame_parms 帧结构
@param harq_ack 下行共享信道 HARQ ack信息
@param subframe UE传输的子帧
@param o_ACK  ACK/NAK payload for PUCCH/PUSCH
@returns status indicator for PUCCH/PUSCH transmission
*/
uint8_t reset_ack(LTE_DL_FRAME_PARMS *frame_parms,
                harq_status_t *harq_ack,
                unsigned char subframe_tx,
                unsigned char subframe_rx,
                unsigned char *o_ACK,
                uint8_t *pN_bundled,
                uint8_t cw_idx);

/*! \brief Compute UL ACK subframe from DL subframe. This is used to retrieve corresponding DLSCH HARQ pid at eNB upon reception of ACK/NAK information on PUCCH/PUSCH.  Derived from Table 10.1-1 in 36.213 (p. 69 in version 8.6)
  @param frame_parms Pointer to DL frame parameter descriptor
  @param subframe Subframe for UE transmission (n in 36.213)
  @param ACK_index TTI bundling index (0,1)
  @returns Subframe index for corresponding DL transmission
*/
/* 通过现行子帧来计算上行ACK子帧的值
@param frame_parms 帧结构
@param subframe UE传输的子帧信息
@param ACK_index TTI绑定index (0,1)
@returns 下行传输的对应子帧下标
*/
uint8_t ul_ACK_subframe2_dl_subframe(LTE_DL_FRAME_PARMS *frame_parms,
                                     uint8_t subframe,
                                     uint8_t ACK_index);

/*! \brief Computes number of DL subframes represented by a particular ACK received on UL (M from Table 10.1-1 in 36.213, p. 69 in version 8.6)
  @param frame_parms Pointer to DL frame parameter descriptor
  @param subframe Subframe for UE transmission (n in 36.213)
  @returns Number of DL subframes (M)
*/
/* 通过上行接收的ACK，计算下行子帧的数量
@param frame_parms 帧结构
@param subframe UE传输的子帧信息
*/
uint8_t ul_ACK_subframe2_M(LTE_DL_FRAME_PARMS *frame_parms,
                           unsigned char subframe);

/*! \brief Indicates the SR TXOp in current subframe.  Implements Table 10.1-5 from 36.213.
  @param phy_vars_ue Pointer to UE variables
  @param proc Pointer to RXn_TXnp4 thread context
  @param eNB_id ID of eNB which is to receive the SR
  @returns 1 if TXOp is active.
*/
/* 调度请求SR是否启用了TXOp
@param phy_vars_ue UE物理层变量
@param proc 收发过程信息
@param eNB_id 接收SR的基站ID
@returns 1 if TXOp is active.
*/
uint8_t is_SR_TXOp(PHY_VARS_UE *phy_vars_ue,
                   UE_rxtx_proc_t *proc,
                   uint8_t eNB_id);

/*! \brief Indicates the SR TXOp in current subframe for eNB and particular UE index.  Implements Table 10.1-5 from 36.213.
  @param phy_vars_eNB Pointer to eNB variables
  @param UE_id ID of UE which may be issuing the SR
  @returns 1 if TXOp is active.
*/
/* 当前子帧是否包含了SR TXOp
@param phy_vars_eNB 基站物理层变量
@param proc 收发过程信息
@param UE_id 发送SR的用户ID
@returns 1 if TXOp is active.
没有实现
*/
uint8_t is_SR_subframe(PHY_VARS_eNB *phy_vars_eNB,
                       eNB_rxtx_proc_t *proc,
                       uint8_t UE_id);

/*! \brief Gives the UL subframe corresponding to a PDDCH order in subframe n
  @param frame_parms Pointer to DL frame parameters
  @param proc Pointer to RXn-TXnp4 proc information
  @param n subframe of PDCCH
  @returns UL subframe corresponding to pdcch order
*/
/* 通过上行子帧信息，返回对应的PDDCH顺序的子帧n
@param frame_parms 下行帧结构
@param n PDDCH子帧下标
@returns UL subframe corresponding to pdcch order
*/
uint8_t pdcch_alloc2ul_subframe(LTE_DL_FRAME_PARMS *frame_parms,
                                uint8_t n);

/*! \brief Gives the UL frame corresponding to a PDDCH order in subframe n
  @param frame_parms Pointer to DL frame parameters
  @param frame Frame of received PDCCH
  @param n subframe of PDCCH
  @returns UL frame corresponding to pdcch order
*/
/* 通过上行帧信息，返回对应的PDDCH顺序的子帧n
@param frame_parms 下行帧结构
@param frame PDCCH接收的帧结构
@param n PDDCH子帧下标
@returns UL subframe corresponding to pdcch order
*/
uint32_t pdcch_alloc2ul_frame(LTE_DL_FRAME_PARMS *frame_parms,
                              uint32_t frame,
                              uint8_t n);


/* 返回Np数组
@param N_RB_DL 下行资源块的数量
@param nCCE
@param plus1
*/
uint16_t get_Np(uint8_t N_RB_DL,
                uint8_t nCCE,
                uint8_t plus1);

/*未实现，未使用*/
int8_t find_ue_dlsch(uint16_t rnti, PHY_VARS_eNB *phy_vars_eNB);
/*未实现，未使用*/
int8_t find_ue_ulsch(uint16_t rnti, PHY_VARS_eNB *phy_vars_eNB);

//int32_t add_ue(int16_t rnti, PHY_VARS_eNB *phy_vars_eNB);
//int mac_phy_remove_ue(module_id_t Mod_idP,rnti_t rnti);
/* 定时提前过程
@param Mod_id 基站ID
@param CC_id 载波ID
@param timing_advance 时间提前量
*/
void process_timing_advance(module_id_t Mod_id,
                            uint8_t CC_id,
                            int16_t timing_advance);

/* RAR的定时提前过程
@param phy_vars_ue 用户物理层变量
@param proc 用户收发过程变量
@param timing_advance 时间提前量
*/
void process_timing_advance_rar(PHY_VARS_UE *phy_vars_ue,
                                UE_rxtx_proc_t *proc,
                                uint16_t timing_advance);

/* 获取发送的振幅
@param power_dBm dbm功率
@param power_max_dBm 发送总功率
@param N_RB_UL 上行资源块的数量
@param nb_rb 资源块的数量
*/
unsigned int get_tx_amp(int power_dBm,
                        int power_max_dBm,
                        int N_RB_UL,
                        int nb_rb);

/* 重置UE物理层*/
void phy_reset_ue(module_id_t Mod_id,uint8_t CC_id,uint8_t eNB_index);

/*! \brief This function retrives the resource (n1_pucch) corresponding to a PDSCH transmission in
subframe n-4 which is acknowledged in subframe n (for FDD) according to n1_pucch = Ncce + N1_pucch.  For
TDD, this routine computes the complex procedure described in Section 10.1 of 36.213 (through tables 10.1-1,10.1-2)
@param phy_vars_ue Pointer to UE variables
@param proc Pointer to RXn-TXnp4 proc information
@param harq_ack Pointer to dlsch_ue harq_ack status descriptor
@param eNB_id Index of eNB
@param b Pointer to PUCCH payload (b[0],b[1])
@param SR 1 means there's a positive SR in parallel to ACK/NAK
@returns n1_pucch
*/
/* 获取n1 PUCCH 信息
@param phy_vars_ue 用户物理层变量
@param proc 收发过程信息
@param harq_ack 用户DLSCH HARQ ACK信息
@param eNB_id 基站ID
@param b PUCCH payload (b[0],b[1])
@param SR 1表示SR启用
@returns n1_pucch
*/
uint16_t get_n1_pucch(PHY_VARS_UE *phy_vars_ue,
		                  UE_rxtx_proc_t *proc,
                      harq_status_t *harq_ack,
                      uint8_t eNB_id,
                      uint8_t *b,
                      uint8_t SR);

/*! \brief This function retrives the resource (n1_pucch) corresponding to a PDSCH transmission in
subframe n-4 which is acknowledged in subframe n (for FDD) according to n1_pucch = Ncce + N1_pucch.  For
TDD, this routine computes the procedure described in Section 10.1 of 36.213 (through tables 10.1-1,10.1-2)
@param phy_vars_eNB Pointer to eNB variables
@param proc Pointer to RXn-TXnp4 proc information
@param eNB_id Index of eNB
@param subframe Index of subframe
@param b Pointer to PUCCH payload (b[0],b[1])
@param n1_pucch0 Pointer to n1_pucch0
@param n1_pucch1 Pointer to n1_pucch1
@param n1_pucch2 Pointer to n1_pucch2
@param n1_pucch3 Pointer to n1_pucch3
*/
/* 基站侧获取n1 PUCCH 信息
@param phy_vars_eNB 基站侧物理层变量
@param proc 基站收发过程信息
@param UE_id 用户ID
@param n1_pucch0
@param n1_pucch1
@param n1_pucch2
@param n1_pucch3
没有实现，没有使用
*/
void get_n1_pucch_eNB(PHY_VARS_eNB *phy_vars_eNB,
		                  eNB_rxtx_proc_t *proc,
                      uint8_t UE_id,
                      int16_t *n1_pucch0,
                      int16_t *n1_pucch1,
                      int16_t *n1_pucch2,
                      int16_t *n1_pucch3);


/*! \brief This function retrieves the harq_pid of the corresponding DLSCH process and updates the error statistics of the DLSCH based on the received ACK info from UE along with the round index.  It also performs the fine-grain rate-adaptation based on the error statistics derived from the ACK/NAK process.
  @param UE_id Local UE index on which to act
  @param phy_vars_eNB Pointer to eNB variables on which to act
  @param proc Pointer to RXn-TXnp4 proc information
  @param pusch_flag Indication that feedback came from PUSCH
  @param pucch_payload Resulting payload from pucch
  @param pucch_sel Selection of n1_pucch0 or n1_pucch1 (TDD specific)
  @param SR_payload Indication of SR presence (TDD specific)
*/
// 没有实现没有使用
void process_HARQ_feedback(uint8_t UE_id,
                           PHY_VARS_eNB *phy_vars_eNB,
			                     eNB_rxtx_proc_t *proc,
                           uint8_t pusch_flag,
                           uint8_t *pucch_payload,
                           uint8_t pucch_sel,
                           uint8_t SR_payload);

/*! \brief This function retrieves the PHY UE mode. It is used as a helper function for the UE MAC.
  @param Mod_id Local UE index on which to act
  @param CC_id Component Carrier Index
  @param eNB_index ID of eNB
  @returns UE mode
*/
/* 获取UE的运行模式
@param Mod_id UEID
@param CC_id 成员载波ID
@param eNB_index 基站ID
@returns UE mode
*/
UE_MODE_t get_ue_mode(uint8_t Mod_id,
                      uint8_t CC_id,
                      uint8_t eNB_index);

/*! \brief This function implements the power control mechanism for PUCCH from 36.213.
    @param phy_vars_ue PHY variables
    @param proc Pointer to proc descriptor
    @param eNB_id Index of eNB
    @param pucch_fmt Format of PUCCH that is being transmitted
    @returns Transmit power
 */
/* 实现PUCCH的功率控制机制
@param phy_vars_ue 用户物理层变量
@param proc 收发过程信息
@param eNB_id 基站ID
@param pucch_fmt 传输的PUCCH的格式
@returns 传输功率
*/
int16_t pucch_power_cntl(PHY_VARS_UE *ue,
                         UE_rxtx_proc_t *proc,
                         uint8_t subframe,
                         uint8_t eNB_id,
                         PUCCH_FMT_t pucch_fmt);

/*! \brief This function implements the power control mechanism for PUCCH from 36.213.
    @param phy_vars_ue PHY variables
    @param proc Pointer to proc descriptor
    @param eNB_id Index of eNB
    @param j index of type of PUSCH (SPS, Normal, Msg3)
    @returns Transmit power
 */
/* PUSCH功率控制
@param phy_vars_ue 用户物理层变量
@param proc 收发过程信息
@param eNB_id 基站ID
@param j PUSCH类型下标 (SPS, Normal, Msg3)
@returns 传输功率
*/
void pusch_power_cntl(PHY_VARS_UE *phy_vars_ue,
                      UE_rxtx_proc_t *proc,
                      uint8_t eNB_id,
                      uint8_t j,
                      uint8_t abstraction_flag);

/*! \brief This function implements the power control mechanism for SRS from 36.213.
    @param phy_vars_ue PHY variables
    @param proc Pointer to proc descriptor
    @param eNB_id Index of eNB
    @param j index of type of PUSCH (SPS, Normal, Msg3)
    @returns Transmit power
 */
/* 探测参考信号功率控制
@param phy_vars_ue 用户侧物理层变量
@param proc 收发过程信息
@param eNB_id 基站ID
@param j PUSCH类型下标(SPS, Normal, Msg3)
@returns 发送功率
*/
void srs_power_cntl(PHY_VARS_UE *ue,
                    UE_rxtx_proc_t *proc,
                    uint8_t eNB_id,
                    uint8_t *pnb_rb_srs,
                    uint8_t abstraction_flag);

/* 获取CQI PMIRI参数
@param ue 用户物理层变量
@eNB_id 基站ID
*/
void get_cqipmiri_params(PHY_VARS_UE *ue,
                         uint8_t eNB_id);

/* 获取PHR信息
@param Mod_id 模块ID
@param CC_id 成员载波ID
@param eNB_index 基站ID
*/
int8_t get_PHR(uint8_t Mod_id,
               uint8_t CC_id,
               uint8_t eNB_index);

/* 调度响应函数
@param Sched_INFO 调度信息
*/
void schedule_response(Sched_Rsp_t *Sched_INFO);

/* 获取用户状态信息
@param Mod_id 模块ID
@param CC_id 成员载波ID
@param rnti 无线网络临时标识
// 实现被注释了
*/
LTE_eNB_UE_stats* get_UE_stats(uint8_t Mod_id,
                               uint8_t CC_id,
                               uint16_t rnti);


/* 获取下行帧结构
@param Mod_id 模块ID
@param CC_id 成员载波ID
实现被注释了
*/
LTE_DL_FRAME_PARMS *get_lte_frame_parms(module_id_t Mod_id,
                                        uint8_t CC_id);
// 实现被注释了
MU_MIMO_mode* get_mu_mimo_mode (module_id_t Mod_id, uint8_t CC_id, rnti_t rnti);

/*功率控制相关的函数*/
int16_t get_hundred_times_delta_IF(PHY_VARS_UE *phy_vars_ue,uint8_t eNB_id,uint8_t harq_pid);

int16_t get_hundred_times_delta_IF_eNB(PHY_VARS_eNB *phy_vars_eNB,uint8_t UE_id,uint8_t harq_pid, uint8_t bw_factor);

int16_t get_hundred_times_delta_IF_mac(module_id_t module_idP, uint8_t CC_id, rnti_t rnti, uint8_t harq_pid);

int16_t get_target_pusch_rx_power(module_id_t module_idP, uint8_t CC_id);
int16_t get_target_pucch_rx_power(module_id_t module_idP, uint8_t CC_id);
未使用
int get_ue_active_harq_pid(uint8_t Mod_id,uint8_t CC_id,uint16_t rnti,int frame, uint8_t subframe,uint8_t *harq_pid,uint8_t *round,uint8_t ul_flag);

// dump 日志文件相关
void dump_dlsch(PHY_VARS_UE *phy_vars_ue,UE_rxtx_proc_t *proc,uint8_t eNB_id,uint8_t subframe,uint8_t harq_pid);
void dump_dlsch_SI(PHY_VARS_UE *phy_vars_ue,UE_rxtx_proc_t *proc,uint8_t eNB_id,uint8_t subframe);
void dump_dlsch_ra(PHY_VARS_UE *phy_vars_ue,UE_rxtx_proc_t *proc,uint8_t eNB_id,uint8_t subframe);

void dump_dlsch2(PHY_VARS_UE *phy_vars_ue,uint8_t eNB_id,uint8_t subframe, unsigned int *coded_bits_per_codeword,int round, unsigned char harq_pid);

/* 判定是否需要进行SRS过程
@param frame_parms 帧结构
@param frame_tx 接收的帧数
@param subframe_tx 接收的子帧数
*/
int is_srs_occasion_common(LTE_DL_FRAME_PARMS *frame_parms,
                           int frame_tx,
                           int subframe_tx);
/* 计算SRS的位置
@param frameType 帧类型
@param isrs ISRS？？？
@param psrsPeriodicity
@param psrsOffset
*/
void compute_srs_pos(lte_frame_type_t frameType,
                     uint16_t isrs,
                     uint16_t *psrsPeriodicity,
                     uint16_t *psrsOffset);

/*@}*/


#endif
