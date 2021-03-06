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

/*! \file PHY/defs.h
 \brief Top-level defines and structure definitions
 \author R. Knopp, F. Kaltenberger
 \date 2011
 \version 0.1
 \company Eurecom
 \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr
 \note
 \warning
*/

/************************************最基本的定义和结构体**************************************************/
#ifndef __PHY_DEFS__H__
#define __PHY_DEFS__H__


#define _GNU_SOURCE
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <linux/sched.h>
#include <signal.h>
#include <execinfo.h>
#include <getopt.h>
#include <sys/sysinfo.h>


#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <string.h>
#include <math.h>
#include "common_lib.h"
#include "msc.h"

#include "openair2/PHY_INTERFACE/IF_Module.h"

//#include <complex.h>
#include "assertions.h"
#ifdef MEX
# define msg mexPrintf
#endif
//use msg in the real-time thread context
#define msg_nrt printf
//use msg_nrt in the non real-time context (for initialization, ...)
#ifndef malloc16
#  ifdef __AVX2__
#    define malloc16(x) memalign(32,x)
#  else
#    define malloc16(x) memalign(16,x)
#  endif
#endif
#define free16(y,x) free(y)
#define bigmalloc malloc
#define bigmalloc16 malloc16
#define openair_free(y,x) free((y))
#define PAGE_SIZE 4096
#define free_and_zero(PtR) do { \
      if (PtR) {           \
        free(PtR);         \
        PtR = NULL;        \
      }                    \
    } while (0)

#define RX_NB_TH_MAX 2
#define RX_NB_TH 2


//! \brief Allocate \c size bytes of memory on the heap with alignment 16 and zero it afterwards.
//! If no more memory is available, this function will terminate the program with an assertion error.
static inline void* malloc16_clear( size_t size )
{
#ifdef __AVX2__
  void* ptr = memalign(32, size);
#else
  void* ptr = memalign(16, size);
#endif
  DevAssert(ptr);
  memset( ptr, 0, size );
  return ptr;
}



#define PAGE_MASK 0xfffff000
#define virt_to_phys(x) (x)

#define openair_sched_exit() exit(-1)


#define max(a,b)  ((a)>(b) ? (a) : (b))
#define min(a,b)  ((a)<(b) ? (a) : (b))


#define bzero(s,n) (memset((s),0,(n)))

#define cmax(a,b)  ((a>b) ? (a) : (b))
#define cmin(a,b)  ((a<b) ? (a) : (b))

#define cmax3(a,b,c) ((cmax(a,b)>c) ? (cmax(a,b)) : (c))

/// suppress compiler warning for unused arguments
#define UNUSED(x) (void)x;


#include "impl_defs_top.h"
#include "impl_defs_lte.h"

#include "PHY/TOOLS/time_meas.h"
#include "PHY/CODING/defs.h"
#include "PHY/TOOLS/defs.h"
#include "platform_types.h"
#define MAX_NUM_RU_PER_eNB 64

#include "PHY/LTE_TRANSPORT/defs.h"
#include <pthread.h>

#include "targets/ARCH/COMMON/common_lib.h"
#include "targets/COMMON/openairinterface5g_limits.h"

#if defined(EXMIMO) || defined(OAI_USRP)
//#define NUMBER_OF_eNB_MAX 1
//#define NUMBER_OF_UE_MAX 16

//#define NUMBER_OF_CONNECTED_eNB_MAX 3
#else
#ifdef LARGE_SCALE
//#define NUMBER_OF_eNB_MAX 2
//#define NUMBER_OF_UE_MAX 120
//#define NUMBER_OF_CONNECTED_eNB_MAX 1 // to save some memory
#else
//#define NUMBER_OF_eNB_MAX 3
//#define NUMBER_OF_UE_MAX 16
//#define NUMBER_OF_RU_MAX 64
//#define NUMBER_OF_CONNECTED_eNB_MAX 1
#endif
#endif
#define NUMBER_OF_SUBBANDS_MAX 13
#define NUMBER_OF_HARQ_PID_MAX 8

#define MAX_FRAME_NUMBER 0x400



#define NUMBER_OF_RN_MAX 3
typedef enum {no_relay=1,unicast_relay_type1,unicast_relay_type2, multicast_relay} relaying_type_t;



#define MCS_COUNT 28
#define MCS_TABLE_LENGTH_MAX 64


#define NUM_DCI_MAX 32

#define NUMBER_OF_eNB_SECTORS_MAX 3

#define NB_BANDS_MAX 8

#define MAX_BANDS_PER_RRU 4


#ifdef OCP_FRAMEWORK
#include <enums.h>
#else
typedef enum {normal_txrx=0,rx_calib_ue=1,rx_calib_ue_med=2,rx_calib_ue_byp=3,debug_prach=4,no_L2_connect=5,calib_prach_tx=6,rx_dump_frame=7,loop_through_memory=8} runmode_t;

/*! \brief Extension Type */
// 扩展类型，包含循环前缀，循环后缀，ZEROS和空
typedef enum {
  CYCLIC_PREFIX,
  CYCLIC_SUFFIX,
  ZEROS,
  NONE
} Extension_t;

//传输接收模式
enum transmission_access_mode {
  NO_ACCESS=0,
  POSTPONED_ACCESS,
  CANCELED_ACCESS,
  UNKNOWN_ACCESS,
  SCHEDULED_ACCESS,
  CBA_ACCESS};
// 节点功能划分，包括传统的eNodeB功能，IF5接口的RAU和NGFI_RRU, IF4p5接口的RAU和NGFI_RRU，Mobipass RRU
typedef enum  {
  eNodeB_3GPP=0,   // classical eNodeB function
  NGFI_RAU_IF5,    // RAU with NGFI IF5
  NGFI_RAU_IF4p5,  // RAU with NFGI IF4p5
  NGFI_RRU_IF5,    // NGFI_RRU (NGFI remote radio-unit,IF5)
  NGFI_RRU_IF4p5,  // NGFI_RRU (NGFI remote radio-unit,IF4p5)
  MBP_RRU_IF5      // Mobipass RRU
} node_function_t;

//节点同步方案
typedef enum {
  // 与外部设备同步
  synch_to_ext_device=0,  // synch to RF or Ethernet device
  // 与其他的节点（时钟源）同步
  synch_to_other,          // synch to another source_(timer, other RU)
  synch_to_mobipass_standalone  // special case for mobipass in standalone mode
} node_timing_t;
#endif

typedef struct UE_SCAN_INFO_s {
  /// 10 best amplitudes (linear) for each pss signals
  int32_t amp[3][10];
  /// 10 frequency offsets (kHz) corresponding to best amplitudes, with respect do minimum DL frequency in the band
  int32_t freq_offset_Hz[3][10];
} UE_SCAN_INFO_t;

/// Top-level PHY Data Structure for RN
// 中继网络物理层数据结构体
typedef struct {
  /// Module ID indicator for this instance
  // 实例的模块ID
  uint8_t Mod_id;
  // 帧， 4字节
  uint32_t frame;
  // phy_vars_eNB
  // phy_vars ue
  // cuurently only used to store and forward the PMCH
  uint8_t mch_avtive[10];
  uint8_t sync_area[10]; // num SF
  LTE_UE_DLSCH_t   *dlsch_rn_MCH[10];

} PHY_VARS_RN;

/// Context data structure for RX/TX portion of subframe processing
// 收发过程中子帧处理中的结构体
typedef struct {
  /// Component Carrier index
  // 成员载波index
  uint8_t              CC_id;
  /// timestamp transmitted to HW
  // 发送的时间戳，8个字节
  openair0_timestamp timestamp_tx;
  /// subframe to act upon for transmission
  // 发送子帧个数
  int subframe_tx;
  /// subframe to act upon for reception
  // 接收子帧个数
  int subframe_rx;
  /// frame to act upon for transmission
  // 发送端帧
  int frame_tx;
  /// frame to act upon for reception
  // 接收的帧数
  int frame_rx;
  /// \brief Instance count for RXn-TXnp4 processing thread.
  /// \internal This variable is protected by \ref mutex_rxtx.
  int instance_cnt_rxtx;
  /// pthread structure for RXn-TXnp4 processing thread
  // 线程结构
  pthread_t pthread_rxtx;
  /// pthread attributes for RXn-TXnp4 processing thread
  // 线程参数
  pthread_attr_t attr_rxtx;
  /// condition variable for tx processing thread
  // 发送线程的环境变量
  pthread_cond_t cond_rxtx;
  /// mutex for RXn-TXnp4 processing thread
  // 收发线程的互斥量
  pthread_mutex_t mutex_rxtx;
  /// scheduling parameters for RXn-TXnp4 thread
  // 调度参数
  struct sched_param sched_param_rxtx;
} eNB_rxtx_proc_t;

// td_params
typedef struct {
  // 基站物理层变量结构
  struct PHY_VARS_eNB_s *eNB;
  // 用户Id
  int UE_id;
  // HARQ PID
  int harq_pid;
  // 左移标志
  int llr8_flag;
  // 结果
  int ret;
} td_params;

typedef struct {
  struct PHY_VARS_eNB_s *eNB;
  LTE_eNB_DLSCH_t *dlsch;
  int G;
  int harq_pid;
} te_params;

// RU线程启动过程中的结构体
typedef struct RU_proc_t_s {
  /// Pointer to associated RU descriptor
  // 指向对应的RU结构体
  struct RU_t_s *ru;
  /// timestamp received from HW
  // 接收的时间戳
  openair0_timestamp timestamp_rx;
  /// timestamp to send to "slave rru"
  // 发送的时间戳
  openair0_timestamp timestamp_tx;
  /// subframe to act upon for reception
  int subframe_rx;
  /// subframe to act upon for transmission
  int subframe_tx;
  /// subframe to act upon for reception of prach
  // prach接收过程中的子帧结构
  int subframe_prach;
#ifdef Rel14
  /// subframe to act upon for reception of prach BL/CE UEs
  // prach BL/CE接收过程中的子帧结构
  int subframe_prach_br;
#endif
  /// frame to act upon for reception
  int frame_rx;
  /// frame to act upon for transmission
  int frame_tx;
  /// unwrapped frame count
  // 没有被封装的帧数
  int frame_tx_unwrap;
  /// frame to act upon for reception of prach
  int frame_prach;
#ifdef Rel14
  /// frame to act upon for reception of prach
  int frame_prach_br;
#endif
  /// frame offset for slave RUs (to correct for frame asynchronism at startup)
  // 从RUs的帧偏移量，为了在开始时矫正偏移量
  int frame_offset;
  /// \brief Instance count for FH processing thread.
  /// \internal This variable is protected by \ref mutex_FH.
  // 前传网络处理线程的数量，内部受保护变量
  int instance_cnt_FH;
  /// \internal This variable is protected by \ref mutex_prach.
  int instance_cnt_prach;
#ifdef Rel14
  /// \internal This variable is protected by \ref mutex_prach.
  int instance_cnt_prach_br;
#endif
  /// \internal This variable is protected by \ref mutex_synch.
  int instance_cnt_synch;
  /// \internal This variable is protected by \ref mutex_eNBs.
  // 基站实例的数量
  int instance_cnt_eNBs;
  /// \brief Instance count for rx processing thread.
  /// \internal This variable is protected by \ref mutex_asynch_rxtx.
  // rxtx线程的实例数量
  int instance_cnt_asynch_rxtx;
  /// \internal This variable is protected by \ref mutex_fep
  int instance_cnt_fep;
  /// \internal This variable is protected by \ref mutex_fep
  int instance_cnt_feptx;
  /// \internal This variable is protected by \ref mutex_ru_thread
  int instance_cnt_ru;
  /// pthread structure for RU FH processing thread
  // RU 前传线程
  pthread_t pthread_FH;
  /// pthread structure for RU control thread
  // RU 控制线程
  pthread_t pthread_ctrl;
  /// pthread structure for RU prach processing thread
  // RU prach线程
  pthread_t pthread_prach;
#ifdef Rel14
  /// pthread structure for RU prach processing thread BL/CE UEs
  pthread_t pthread_prach_br;
#endif
  /// pthread struct for RU synch thread
  // RU 同步线程
  pthread_t pthread_synch;
  /// pthread struct for RU RX FEP worker thread
  // RU接收端front-end Processing 线程
  pthread_t pthread_fep;
  /// pthread struct for RU RX FEPTX worker thread
  pthread_t pthread_feptx;
  /// pthread structure for asychronous RX/TX processing thread
  // 异步收发线程
  pthread_t pthread_asynch_rxtx;
  /// flag to indicate first RX acquisition
  // 第一个获得的RX
  int first_rx;
  /// flag to indicate first TX transmission
  int first_tx;
  /// pthread attributes for RU FH processing thread
  pthread_attr_t attr_FH;
  /// pthread attributes for RU control thread
  pthread_attr_t attr_ctrl;
  /// pthread attributes for RU prach
  pthread_attr_t attr_prach;
#ifdef Rel14
  /// pthread attributes for RU prach BL/CE UEs
  pthread_attr_t attr_prach_br;
#endif
  /// pthread attributes for RU synch thread
  pthread_attr_t attr_synch;
  /// pthread attributes for asynchronous RX thread
  pthread_attr_t attr_asynch_rxtx;
  /// pthread attributes for worker fep thread
  pthread_attr_t attr_fep;
  /// pthread attributes for worker feptx thread
  pthread_attr_t attr_feptx;
  /// scheduling parameters for RU FH thread
  struct sched_param sched_param_FH;
  /// scheduling parameters for RU prach thread
  struct sched_param sched_param_prach;
#ifdef Rel14
  /// scheduling parameters for RU prach thread BL/CE UEs
  struct sched_param sched_param_prach_br;
#endif
  /// scheduling parameters for RU synch thread
  struct sched_param sched_param_synch;
  /// scheduling parameters for asynch_rxtx thread
  struct sched_param sched_param_asynch_rxtx;
  /// condition variable for RU FH thread
  pthread_cond_t cond_FH;
  /// condition variable for RU prach thread
  pthread_cond_t cond_prach;
#ifdef Rel14
  /// condition variable for RU prach thread BL/CE UEs
  pthread_cond_t cond_prach_br;
#endif
  /// condition variable for RU synch thread
  pthread_cond_t cond_synch;
  /// condition variable for asynch RX/TX thread
  pthread_cond_t cond_asynch_rxtx;
  /// condition varaible for RU RX FEP thread
  pthread_cond_t cond_fep;
  /// condition varaible for RU RX FEPTX thread
  pthread_cond_t cond_feptx;
  /// condition variable for eNB signal
  pthread_cond_t cond_eNBs;
  /// condition variable for ru_thread
  pthread_cond_t cond_ru_thread;
  /// mutex for RU FH
  pthread_mutex_t mutex_FH;
  /// mutex for RU prach
  pthread_mutex_t mutex_prach;
#ifdef Rel14
  /// mutex for RU prach BL/CE UEs
  pthread_mutex_t mutex_prach_br;
#endif
  /// mutex for RU synch
  pthread_mutex_t mutex_synch;
  /// mutex for eNB signal
  pthread_mutex_t mutex_eNBs;
  /// mutex for asynch RX/TX thread
  pthread_mutex_t mutex_asynch_rxtx;
  /// mutex for fep RX worker thread
  pthread_mutex_t mutex_fep;
  /// mutex for fep TX worker thread
  pthread_mutex_t mutex_feptx;
  /// mutex for ru_thread
  pthread_mutex_t mutex_ru;
  /// symbol mask for IF4p5 reception per subframe
  uint32_t symbol_mask[10];
  /// number of slave threads
  int                  num_slaves;
  /// array of pointers to slaves
  struct RU_proc_t_s           **slave_proc;
} RU_proc_t;

/// Context data structure for eNB subframe processing
// eNB子帧处理过程中的上下文数据结构体
typedef struct eNB_proc_t_s {
  /// Component Carrier index
  // 成员载波ID
  uint8_t              CC_id;
  /// thread index
  // 线程ID
  int thread_index;
  /// timestamp received from HW
  // 接收的信号的时间戳
  openair0_timestamp timestamp_rx;
  /// timestamp to send to "slave rru"
  // 发送的信号的时间戳
  openair0_timestamp timestamp_tx;
  /// subframe to act upon for reception
  // 子帧接收
  int subframe_rx;
  /// subframe to act upon for PRACH
  // 子帧PRACH个数
  int subframe_prach;
#ifdef Rel14
  /// subframe to act upon for reception of prach BL/CE UEs
  // 子帧数目，PRACH_BR
  int subframe_prach_br;
#endif
  /// frame to act upon for reception
  // 接收帧数
  int frame_rx;
  /// frame to act upon for transmission
  // 发送的帧数
  int frame_tx;
  /// frame to act upon for PRACH
  // PRACH过程中收发的帧数
  int frame_prach;
#ifdef Rel14
  /// frame to act upon for PRACH BL/CE UEs
  // PRACH_BR中收发的帧数
  int frame_prach_br;
#endif
  /// \internal This variable is protected by \ref mutex_td.
  // tubor解码实例数
  int instance_cnt_td;
  /// \internal This variable is protected by \ref mutex_te.
  // turbo编码的实例数
  int instance_cnt_te;
  /// \internal This variable is protected by \ref mutex_prach.
  // PRACH的实例数
  int instance_cnt_prach;
#ifdef Rel14
  /// \internal This variable is protected by \ref mutex_prach for BL/CE UEs.
  int instance_cnt_prach_br;
#endif
  // instance count for over-the-air eNB synchronization
  int instance_cnt_synch;
  /// \internal This variable is protected by \ref mutex_asynch_rxtx.
  int instance_cnt_asynch_rxtx;
  /// pthread structure for eNB single processing thread
  // 基站处理单线程
  pthread_t pthread_single;
  /// pthread structure for asychronous RX/TX processing thread
  // 基站异步收发线程
  pthread_t pthread_asynch_rxtx;
  /// flag to indicate first RX acquisition
  int first_rx;
  /// flag to indicate first TX transmission
  int first_tx;
  /// pthread attributes for parallel turbo-decoder thread
  // 并行turbo解码线程的线程变量
  pthread_attr_t attr_td;
  /// pthread attributes for parallel turbo-encoder thread
  // 并行turbo编码线程的线程变量
  pthread_attr_t attr_te;
  /// pthread attributes for single eNB processing thread
  // 单线程处理过程中
  pthread_attr_t attr_single;
  /// pthread attributes for prach processing thread
  pthread_attr_t attr_prach;
#ifdef Rel14
  /// pthread attributes for prach processing thread BL/CE UEs
  pthread_attr_t attr_prach_br;
#endif
  /// pthread attributes for asynchronous RX thread
  pthread_attr_t attr_asynch_rxtx;
  /// scheduling parameters for parallel turbo-decoder thread
  // 并行Turbo编码的调度参数
  struct sched_param sched_param_td;
  /// scheduling parameters for parallel turbo-encoder thread
  // 并行Turbo解码的调度参数
  struct sched_param sched_param_te;
  /// scheduling parameters for single eNB thread
  struct sched_param sched_param_single;
  /// scheduling parameters for prach thread
  struct sched_param sched_param_prach;
#ifdef Rel14
  /// scheduling parameters for prach thread
  struct sched_param sched_param_prach_br;
#endif
  /// scheduling parameters for asynch_rxtx thread
  struct sched_param sched_param_asynch_rxtx;
  /// pthread structure for parallel turbo-decoder thread
  // 线程结构相关
  pthread_t pthread_td;
  /// pthread structure for parallel turbo-encoder thread
  pthread_t pthread_te;
  /// pthread structure for PRACH thread
  pthread_t pthread_prach;
#ifdef Rel14
  /// pthread structure for PRACH thread BL/CE UEs
  pthread_t pthread_prach_br;
#endif
  /// condition variable for parallel turbo-decoder thread
  // 环境变量相关
  pthread_cond_t cond_td;
  /// condition variable for parallel turbo-encoder thread
  pthread_cond_t cond_te;
  /// condition variable for PRACH processing thread;
  pthread_cond_t cond_prach;
#ifdef Rel14
  /// condition variable for PRACH processing thread BL/CE UEs;
  pthread_cond_t cond_prach_br;
#endif
  /// condition variable for asynch RX/TX thread
  pthread_cond_t cond_asynch_rxtx;
  /// mutex for parallel turbo-decoder thread
  // 互斥量相关
  pthread_mutex_t mutex_td;
  /// mutex for parallel turbo-encoder thread
  pthread_mutex_t mutex_te;
  /// mutex for PRACH thread
  pthread_mutex_t mutex_prach;
#ifdef Rel14
  /// mutex for PRACH thread for BL/CE UEs
  pthread_mutex_t mutex_prach_br;
#endif
  /// mutex for asynch RX/TX thread
  pthread_mutex_t mutex_asynch_rxtx;
  /// mutex for RU access to eNB processing (PDSCH/PUSCH)
  pthread_mutex_t mutex_RU;
  /// mutex for RU access to eNB processing (PRACH)
  pthread_mutex_t mutex_RU_PRACH;
  /// mutex for RU access to eNB processing (PRACH BR)
  pthread_mutex_t mutex_RU_PRACH_br;
  /// mask for RUs serving eNB (PDSCH/PUSCH)
  int RU_mask[10];
  /// time measurements for RU arrivals
  struct timespec t[10];
  /// Timing statistics (RU_arrivals)
  // RU 到达的统计信息
  time_stats_t ru_arrival_time;
  /// mask for RUs serving eNB (PRACH)
  // PRACH中RU的掩码
  int RU_mask_prach;
#ifdef Rel14
  /// mask for RUs serving eNB (PRACH)
  // RU的掩码
  int RU_mask_prach_br;
#endif
  /// parameters for turbo-decoding worker thread
  // tubor解码线程
  td_params tdp;
  /// parameters for turbo-encoding worker thread
  // turbo编码线程
  te_params tep;
  /// set of scheduling variables RXn-TXnp4 threads
  // 基站收发过程结构体，指向proc_rxtx[2]
  eNB_rxtx_proc_t proc_rxtx[2];
} eNB_proc_t;


/// Context data structure for RX/TX portion of subframe processing
// UE收发帧结构
typedef struct {
  /// index of the current UE RX/TX proc
  // 收发进程ID
  int                  proc_id;
  /// Component Carrier index
  // 成员载波索引
  uint8_t              CC_id;
  /// timestamp transmitted to HW
  // 时间戳
  openair0_timestamp timestamp_tx;
  /// subframe to act upon for transmission
  // 子帧相关
  int subframe_tx;
  /// subframe to act upon for reception
  int subframe_rx;
  /// frame to act upon for transmission
  int frame_tx;
  /// frame to act upon for reception
  // 帧相关
  int frame_rx;
  /// \brief Instance count for RXn-TXnp4 processing thread.
  /// \internal This variable is protected by \ref mutex_rxtx.
  int instance_cnt_rxtx;
  /// pthread structure for RXn-TXnp4 processing thread
  // 线程相关的数据
  pthread_t pthread_rxtx;
  /// pthread attributes for RXn-TXnp4 processing thread
  pthread_attr_t attr_rxtx;
  /// condition variable for tx processing thread
  pthread_cond_t cond_rxtx;
  /// mutex for RXn-TXnp4 processing thread
  pthread_mutex_t mutex_rxtx;
  /// scheduling parameters for RXn-TXnp4 thread
  // 调度参数
  struct sched_param sched_param_rxtx;

  /// internal This variable is protected by ref mutex_fep_slot1.
  //int instance_cnt_slot0_dl_processing;
  int instance_cnt_slot1_dl_processing;
  /// pthread descriptor fep_slot1 thread
  //pthread_t pthread_slot0_dl_processing;
  pthread_t pthread_slot1_dl_processing;
  /// pthread attributes for fep_slot1 processing thread
 // pthread_attr_t attr_slot0_dl_processing;
  pthread_attr_t attr_slot1_dl_processing;
  /// condition variable for UE fep_slot1 thread;
  //pthread_cond_t cond_slot0_dl_processing;
  pthread_cond_t cond_slot1_dl_processing;
  /// mutex for UE synch thread
  //pthread_mutex_t mutex_slot0_dl_processing;
  pthread_mutex_t mutex_slot1_dl_processing;
  //
  uint8_t chan_est_pilot0_slot1_available;
  uint8_t chan_est_slot1_available;
  uint8_t llr_slot1_available;
  uint8_t dci_slot0_available;
  uint8_t first_symbol_available;
  //uint8_t channel_level;
  /// scheduling parameters for fep_slot1 thread
  struct sched_param sched_param_fep_slot1;

  int sub_frame_start;
  int sub_frame_step;
  unsigned long long gotIQs;
} UE_rxtx_proc_t;

/// Context data structure for eNB subframe processing
// eNB子帧处理过程中的结构体
typedef struct {
  /// Component Carrier index
  // 成员载波ID
  uint8_t              CC_id;
  /// Last RX timestamp
  // RX时间戳
  openair0_timestamp timestamp_rx;
  /// pthread attributes for main UE thread
  // UE主线程的线程参数
  pthread_attr_t attr_ue;
  /// scheduling parameters for main UE thread
  // 主线程调度结构体
  struct sched_param sched_param_ue;
  /// pthread descriptor main UE thread
  // UE主线程
  pthread_t pthread_ue;
  /// \brief Instance count for synch thread.
  /// \internal This variable is protected by \ref mutex_synch.
  // 实例同步数量
  int instance_cnt_synch;
  /// pthread attributes for synch processing thread
  pthread_attr_t attr_synch;
  /// scheduling parameters for synch thread
  // 同步线程的调度参数
  struct sched_param sched_param_synch;
  /// pthread descriptor synch thread
  // 同步线程
  pthread_t pthread_synch;
  /// condition variable for UE synch thread;
  // 环境变量
  pthread_cond_t cond_synch;
  /// mutex for UE synch thread
  // UE 同步线程
  pthread_mutex_t mutex_synch;
  /// instance count for eNBs
  // eNB的实例数
  int instance_cnt_eNBs;
  /// set of scheduling variables RXn-TXnp4 threads
  // 用户收发进程中的调度变量
  UE_rxtx_proc_t proc_rxtx[RX_NB_TH];
} UE_proc_t;

// 枚举类型，定义RU南向接口的类型，6个可选类型，C-RAN中选用的是3
typedef enum {
  LOCAL_RF        =0,
  REMOTE_IF5      =1,
  REMOTE_MBP_IF5  =2,
  REMOTE_IF4p5    =3,
  REMOTE_IF1pp    =4,
  MAX_RU_IF_TYPES =5
} RU_if_south_t;

// RRU状态的枚举类型，0-5， 从IDLE-SYNC， ERROR则退出
typedef enum {
  RU_IDLE   = 0,
  RU_CONFIG = 1,
  RU_READY  = 2,
  RU_RUN    = 3,
  RU_ERROR  = 4,
  RU_SYNC   = 5
} rru_state_t;

/// Some commamds to RRU. Not sure we should do it like this !
// 对RRU执行的一些操作，不确定是否操作
typedef enum {
  EMPTY     = 0,
  STOP_RU   = 1,
  RU_FRAME_RESYNCH = 2,
  WAIT_RESYNCH = 3
} rru_cmd_t;

//RU数据结构体，包含RU的index，配置文件等
typedef struct RU_t_s{
  /// index of this ru
  // RU的index
  uint32_t idx;
 /// Pointer to configuration file
  // 指向配置文件的指针
  char *rf_config_file;
  /// southbound interface
  // RU南向接口类型，C-RAN中为REMOTE_IF4p5
  RU_if_south_t if_south;
  /// timing
  // RU节点同步方案，C-RAN中与Oclock-G来同步，因此选择的为 synch_to_other
  node_timing_t if_timing;
  /// function
  // 节点功能，C-RAN中RRU的节点功能为NGFI_RRU_IF4p5
  node_function_t function;
  /// Ethernet parameters for fronthaul interface
  // RU网络接口信息
  eth_params_t eth_params;
  /// flag to indicate the RU is in synch with a master reference
  // 是否与主参考信号同步
  int in_synch;
  /// timing offset
  // 同步偏移量
  int rx_offset;
  /// flag to indicate the RU is a slave to another source
  // 是否为slave RU
  int is_slave;
  /// counter to delay start of processing of RU until HW settles
  // 等待时间？？？
  int wait_cnt;
  /// Total gain of receive chain
  // 接收总增益
  uint32_t             rx_total_gain_dB;
  /// number of bands that this device can support
  // 支持的band数，C-RAN中为band7
  int num_bands;
  /// band list
  // band列表
  int band[MAX_BANDS_PER_RRU];
  /// number of RX paths on device
  // 接收路径是数量，也就是接收信道数？
  int nb_rx;
  /// number of TX paths on device
  // 发送路径的数量，也就是发送信道数
  int nb_tx;
  /// maximum PDSCH RS EPRE
  // PDSCH 参考信号功率的最大值
  int max_pdschReferenceSignalPower;
  /// maximum RX gain
  // 接收端最大增益
  int max_rxgain;
  /// Attenuation of RX paths on device
  // 接收天线数
  int att_rx;
  /// Attenuation of TX paths on device
  // 发送天线数
  int att_tx;
  /// flag to indicate precoding operation in RU
  // 标志位中指示是否需要进行预编码
  int do_precoding;
  /// Frame parameters
  // LTE下行帧结构参数
  LTE_DL_FRAME_PARMS frame_parms;
  ///timing offset used in TDD
  // TDD 模式中的频率偏移量
  int              N_TA_offset;
  /// RF device descriptor
  // 射频设备的信息，结构体common_lib.h中
  openair0_device rfdevice;
  /// HW configuration
  // 硬件配置信息，common_lib.h中
  openair0_config_t openair0_cfg;
  /// Number of eNBs using this RU
  // RU 中的基站实例的个数，当前为1个
  int num_eNB;
  /// list of eNBs using this RU
  // RU中使用的基站列表
  struct PHY_VARS_eNB_s *eNB_list[NUMBER_OF_eNB_MAX];
  /// Mapping of antenna ports to RF chain index
  // 天线端口和射频链之间的映射， common_lib.h中
  openair0_rf_map      rf_map;
  /// IF device descriptor
  // ifdevice结构体信息，比如USRP， commons_lib.h中
  openair0_device ifdevice;
  /// Pointer for ifdevice buffer struct
  // IF设备的缓冲结构指针
  if_buffer_t ifbuffer;
  /// if prach processing is to be performed in RU
  // PRACH是否由RU来完成
  int                  do_prach;
  /***************************************函数指针**********************************************/
  /// function pointer to synchronous RX fronthaul function (RRU,3GPP_eNB)
  // 前传接收同步函数
  void                 (*fh_south_in)(struct RU_t_s *ru,int *frame, int *subframe);
  /// function pointer to synchronous TX fronthaul function
  // 前传发送同步函数
  void                 (*fh_south_out)(struct RU_t_s *ru);
  /// function pointer to synchronous RX fronthaul function (RRU)
  // 前传发送同步函数，RRU
  void                 (*fh_north_in)(struct RU_t_s *ru,int *frame, int *subframe);
  /// function pointer to synchronous RX fronthaul function (RRU)
  // 前传接收同步函数，RRU
  void                 (*fh_north_out)(struct RU_t_s *ru);
  /// function pointer to asynchronous fronthaul interface
  // 异步前传接口函数
  void                 (*fh_north_asynch_in)(struct RU_t_s *ru,int *frame, int *subframe);
  /// function pointer to asynchronous fronthaul interface
  void                 (*fh_south_asynch_in)(struct RU_t_s *ru,int *frame, int *subframe);
  /// function pointer to initialization function for radio interface
  // 射频启动函数
  int                  (*start_rf)(struct RU_t_s *ru);
  /// function pointer to release function for radio interface
  // 射频终止函数
  int                  (*stop_rf)(struct RU_t_s *ru);
  /// function pointer to initialization function for radio interface
  // IF设备启动函数
  int                  (*start_if)(struct RU_t_s *ru,struct PHY_VARS_eNB_s *eNB);
  /// function pointer to RX front-end processing routine (DFTs/prefix removal or NULL)
  // 接收前端处理过程（完成DFTs，循环前缀的移除或者什么都不做）
  void                 (*feprx)(struct RU_t_s *ru);
  /// function pointer to TX front-end processing routine (IDFTs and prefix removal or NULL)
  // 发送前端处理过程（完成IDFTs，添加循环前缀，或者什么都不做）
  void                 (*feptx_ofdm)(struct RU_t_s *ru);
  /// function pointer to TX front-end processing routine (PRECODING)
  // 接收前端处理过程，预编码
  void                 (*feptx_prec)(struct RU_t_s *ru);
  /// function pointer to wakeup routine in lte-enb.
  // 唤醒LTE-enb中的收发线程
  int (*wakeup_rxtx)(struct PHY_VARS_eNB_s *eNB, struct RU_t_s *ru);
  /// function pointer to wakeup routine in lte-enb.
  // 唤醒lte-enb中的prach线程
  void (*wakeup_prach_eNB)(struct PHY_VARS_eNB_s *eNB,struct RU_t_s *ru,int frame,int subframe);
#ifdef Rel14
  /// function pointer to wakeup routine in lte-enb.
  // 唤醒lte-enb中 的prach_br线程
  void (*wakeup_prach_eNB_br)(struct PHY_VARS_eNB_s *eNB,struct RU_t_s *ru,int frame,int subframe);
#endif
  /// function pointer to eNB entry routine
  // 基站实体过程
  void (*eNB_top)(struct PHY_VARS_eNB_s *eNB, int frame_rx, int subframe_rx, char *string);
  /*********************************枚举类型*********************************************************/
  /// Timing statistics，时间统计信息
  // OFDM解调
  time_stats_t ofdm_demod_stats;
  /// Timing statistics (TX)
  // OFDM 调制
  time_stats_t ofdm_mod_stats;
  /// Timing statistics (RX Fronthaul + Compression)
  // 接收前端和压缩
  time_stats_t rx_fhaul;
  /// Timing statistics (TX Fronthaul + Compression)
  // 发送前传和压缩
  time_stats_t tx_fhaul;
  /// Timong statistics (Compression)
  // 压缩过程
  time_stats_t compression;
  /// Timing statistics (Fronthaul transport)
  // 前传传输过程
  time_stats_t transport;
  /// RX and TX buffers for precoder output
  // 收发缓冲，impl_defs_lte.h中
  RU_COMMON            common;
  /// beamforming weight vectors per eNB
  // 每个基站的波束赋形权重
  int32_t **beam_weights[NUMBER_OF_eNB_MAX+1][15];

  /// received frequency-domain signal for PRACH (IF4p5 RRU)
  // PRACH 接收的频域信号
  int16_t              **prach_rxsigF;
  /// received frequency-domain signal for PRACH BR (IF4p5 RRU)
  // PRACH_BR 接收的频域信号
  int16_t              **prach_rxsigF_br[4];
  /// sequence number for IF5
  // IF5的序列号
  uint8_t seqno;
  /// initial timestamp used as an offset make first real timestamp 0
  // 第一个时间戳
  openair0_timestamp   ts_offset;
  /// Current state of the RU
  // RU 当前的状态 IDLE-SYNC
  rru_state_t state;
  /// Command to do
  // RU命令
  rru_cmd_t cmd;
  /// value to be passed using command
  // CMD 的指令
  uint16_t cmdval;
  /// process scheduling variables
  // RU 调度变量
  RU_proc_t            proc;
  /// stats thread pthread descriptor
  // RU stats 线程
  pthread_t            ru_stats_thread;
} RU_t;

// 物理参数相关的结构体，包含参数估计的一些值，天线配置情况，以及接收功率的统计信息等
typedef struct {
  //unsigned int   rx_power[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];     //! estimated received signal power (linear)
  //unsigned short rx_power_dB[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];  //! estimated received signal power (dB)
  //unsigned short rx_avg_power_dB[NUMBER_OF_CONNECTED_eNB_MAX];              //! estimated avg received signal power (dB)

  // RRC measurements
  // RRC 参数
  uint32_t rssi;
  int n_adj_cells;
  unsigned int adj_cell_id[6];
  uint32_t rsrq[7];
  uint32_t rsrp[7];
  float rsrp_filtered[7]; // after layer 3 filtering
  float rsrq_filtered[7];
  // common measurements
  /**********************************噪声********************************************/
  //! estimated noise power (linear)
  // 估计的噪声功率
  unsigned int   n0_power[NB_ANTENNAS_RX];
  //! estimated noise power (dB)
  // 估计的噪声功率的DB值
  unsigned short n0_power_dB[NB_ANTENNAS_RX];
  //! total estimated noise power (linear)
  // 总噪声功率
  unsigned int   n0_power_tot;
  //! total estimated noise power (dB)
  // 总噪声功率
  unsigned short n0_power_tot_dB;
  //! average estimated noise power (linear)
  // 平均噪声功率
  unsigned int   n0_power_avg;
  // 平均噪声功率dB
  //! average estimated noise power (dB)
  unsigned short n0_power_avg_dB;
  //! total estimated noise power (dBm)
  // 总噪声功率， DB
  short n0_power_tot_dBm;

  // UE measurements
  /****************************************************UE***************************************/
  //! estimated received spatial signal power (linear)
  // 接收空分信号功率
  int            rx_spatial_power[NUMBER_OF_CONNECTED_eNB_MAX][2][2];
  //! estimated received spatial signal power (dB)
  // DB
  unsigned short rx_spatial_power_dB[NUMBER_OF_CONNECTED_eNB_MAX][2][2];

  /// estimated received signal power (sum over all TX antennas)
  //int            wideband_cqi[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  // 接收信号的总功率,发送天线
  int            rx_power[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  /// estimated received signal power (sum over all TX antennas)
  //int            wideband_cqi_dB[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  unsigned short rx_power_dB[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  // 接收信号的总功率，收发天线的总和
  /// estimated received signal power (sum over all TX/RX antennas)
  int            rx_power_tot[NUMBER_OF_CONNECTED_eNB_MAX]; //NEW
  /// estimated received signal power (sum over all TX/RX antennas)
  unsigned short rx_power_tot_dB[NUMBER_OF_CONNECTED_eNB_MAX]; //NEW

  // 接收信号总功率在时域的平均值
  //! estimated received signal power (sum of all TX/RX antennas, time average)
  int            rx_power_avg[NUMBER_OF_CONNECTED_eNB_MAX];
  //! estimated received signal power (sum of all TX/RX antennas, time average, in dB)
  unsigned short rx_power_avg_dB[NUMBER_OF_CONNECTED_eNB_MAX];

  // SINR
  /// SINR (sum of all TX/RX antennas, in dB)
  int            wideband_cqi_tot[NUMBER_OF_CONNECTED_eNB_MAX];
  /// SINR (sum of all TX/RX antennas, time average, in dB)
  int            wideband_cqi_avg[NUMBER_OF_CONNECTED_eNB_MAX];

  //! estimated rssi (dBm)
  // rssi dbm值
  short          rx_rssi_dBm[NUMBER_OF_CONNECTED_eNB_MAX];
  //! estimated correlation (wideband linear) between spatial channels (computed in dlsch_demodulation)
  // 空分信道相关性估计值
  int            rx_correlation[NUMBER_OF_CONNECTED_eNB_MAX][2];
  //! estimated correlation (wideband dB) between spatial channels (computed in dlsch_demodulation)
  int            rx_correlation_dB[NUMBER_OF_CONNECTED_eNB_MAX][2];

  // 预编码CQI
  /// Wideband CQI (sum of all RX antennas, in dB, for precoded transmission modes (3,4,5,6), up to 4 spatial streams)
  int            precoded_cqi_dB[NUMBER_OF_CONNECTED_eNB_MAX+1][4];
  /// Subband CQI per RX antenna (= SINR)
  // 子频带的CQI
  int            subband_cqi[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX][NUMBER_OF_SUBBANDS_MAX];
  /// Total Subband CQI  (= SINR)
  // SINR
  int            subband_cqi_tot[NUMBER_OF_CONNECTED_eNB_MAX][NUMBER_OF_SUBBANDS_MAX];
  /// Subband CQI in dB (= SINR dB)
  int            subband_cqi_dB[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX][NUMBER_OF_SUBBANDS_MAX];
  /// Total Subband CQI
  int            subband_cqi_tot_dB[NUMBER_OF_CONNECTED_eNB_MAX][NUMBER_OF_SUBBANDS_MAX];
  /// Wideband PMI for each RX antenna
  // 接收天线的PMI， i
  int            wideband_pmi_re[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  /// Wideband PMI for each RX antenna
  // 接收天线的PMI, j
  int            wideband_pmi_im[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];
  // 子带宽的的PMI
  ///Subband PMI for each RX antenna
  int            subband_pmi_re[NUMBER_OF_CONNECTED_eNB_MAX][NUMBER_OF_SUBBANDS_MAX][NB_ANTENNAS_RX];
  ///Subband PMI for each RX antenna
  int            subband_pmi_im[NUMBER_OF_CONNECTED_eNB_MAX][NUMBER_OF_SUBBANDS_MAX][NB_ANTENNAS_RX];
  /// chosen RX antennas (1=Rx antenna 1, 2=Rx antenna 2, 3=both Rx antennas)
  // 接收天线选择，1-RX1， 2-RX2， 3-BOTH
  unsigned char           selected_rx_antennas[NUMBER_OF_CONNECTED_eNB_MAX][NUMBER_OF_SUBBANDS_MAX];
  /// Wideband Rank indication
  // RANK
  unsigned char  rank[NUMBER_OF_CONNECTED_eNB_MAX];
  /// Number of RX Antennas
  // 接收天线的数量
  unsigned char  nb_antennas_rx;
  /// DLSCH error counter
  // short          dlsch_errors;

} PHY_MEASUREMENTS;

// 基站侧物理参数相关的结构体，包含参数估计的一些值，天线配置情况，以及接收功率的统计信息等
typedef struct {
  //unsigned int   rx_power[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];     //! estimated received signal power (linear)
  //unsigned short rx_power_dB[NUMBER_OF_CONNECTED_eNB_MAX][NB_ANTENNAS_RX];  //! estimated received signal power (dB)
  //unsigned short rx_avg_power_dB[NUMBER_OF_CONNECTED_eNB_MAX];              //! estimated avg received signal power (dB)

  // common measurements
  //! estimated noise power (linear)
  // 噪声功率
  unsigned int   n0_power[MAX_NUM_RU_PER_eNB];
  //! estimated noise power (dB)
  unsigned short n0_power_dB[MAX_NUM_RU_PER_eNB];

  // 总噪声功率
  //! total estimated noise power (linear)
  unsigned int   n0_power_tot;
  //! estimated avg noise power (dB)
  unsigned short n0_power_tot_dB;
  //! estimated avg noise power (dB)
  short n0_power_tot_dBm;

  // 每个资源块每个天线的平均噪声功率
  //! estimated avg noise power per RB per RX ant (lin)
  unsigned short n0_subband_power[MAX_NUM_RU_PER_eNB][100];
  //! estimated avg noise power per RB per RX ant (dB)
  unsigned short n0_subband_power_dB[MAX_NUM_RU_PER_eNB][100];

  // 每个资源块的平均噪声功率
  //! estimated avg noise power per RB (dB)
  short n0_subband_power_tot_dB[100];
  //! estimated avg noise power per RB (dBm)
  short n0_subband_power_tot_dBm[100];

  /**********************************************基站参数**************************************/
  // eNB measurements (per user)
  // 接收到的空分信号功率
  //! estimated received spatial signal power (linear)
  unsigned int   rx_spatial_power[NUMBER_OF_UE_MAX][2][2];
  //! estimated received spatial signal power (dB)
  unsigned short rx_spatial_power_dB[NUMBER_OF_UE_MAX][2][2];
  // RSSI信号
  //! estimated rssi (dBm)
  short          rx_rssi_dBm[NUMBER_OF_UE_MAX];
  //! estimated correlation (wideband linear) between spatial channels (computed in dlsch_demodulation)
  // 信道相关性分析，dlsch_demodulation中计算
  int            rx_correlation[NUMBER_OF_UE_MAX][2];
  //! estimated correlation (wideband dB) between spatial channels (computed in dlsch_demodulation)
  int            rx_correlation_dB[NUMBER_OF_UE_MAX][2];

  // SINR
  /// Wideband CQI (= SINR)
  int            wideband_cqi[NUMBER_OF_UE_MAX][MAX_NUM_RU_PER_eNB];
  /// Wideband CQI in dB (= SINR dB)
  int            wideband_cqi_dB[NUMBER_OF_UE_MAX][MAX_NUM_RU_PER_eNB];
  /// Wideband CQI (sum of all RX antennas, in dB)
  char           wideband_cqi_tot[NUMBER_OF_UE_MAX];
  /// Subband CQI per RX antenna and RB (= SINR)
  int            subband_cqi[NUMBER_OF_UE_MAX][MAX_NUM_RU_PER_eNB][100];
  /// Total Subband CQI and RB (= SINR)
  int            subband_cqi_tot[NUMBER_OF_UE_MAX][100];
  /// Subband CQI in dB and RB (= SINR dB)
  int            subband_cqi_dB[NUMBER_OF_UE_MAX][MAX_NUM_RU_PER_eNB][100];
  /// Total Subband CQI and RB
  int            subband_cqi_tot_dB[NUMBER_OF_UE_MAX][100];
  /// PRACH background noise level
  int            prach_I0;
} PHY_MEASUREMENTS_eNB;


/// Top-level PHY Data Structure for eNB
// 基站物理层变量结构体
typedef struct PHY_VARS_eNB_s {
  /// Module ID indicator for this instance
  // 当前实例的模块ID
  module_id_t          Mod_id;
  uint8_t              CC_id;
  uint8_t              configured;
  // 基站子帧处理过程中的结构体
  eNB_proc_t           proc;
  int                  single_thread_flag;
  int                  abstraction_flag;
  // RU 数量
  int                  num_RU;
  // RU传输的数据结构
  RU_t                 *RU_list[MAX_NUM_RU_PER_eNB];
  /// Ethernet parameters for northbound midhaul interface
  //北向接口的网络参数
  eth_params_t         eth_params_n;
  // 前传网络的参数
  /// Ethernet parameters for fronthaul interface
  eth_params_t         eth_params;
  int                  rx_total_gain_dB;
  int                  (*td)(struct PHY_VARS_eNB_s *eNB,int UE_id,int harq_pid,int llr8_flag);
  int                  (*te)(struct PHY_VARS_eNB_s *,uint8_t *,uint8_t,LTE_eNB_DLSCH_t *,int,uint8_t,time_stats_t *,time_stats_t *,time_stats_t *);
  int                  (*start_if)(struct RU_t_s *ru,struct PHY_VARS_eNB_s *eNB);
  uint8_t              local_flag;
  // LTE下行帧结构参数
  LTE_DL_FRAME_PARMS   frame_parms;
  // 基站信道估计相关参数
  PHY_MEASUREMENTS_eNB measurements;
  // IF 模块信息
  IF_Module_t          *if_inst;
  // 上行？？？？
  UL_IND_t             UL_INFO;
  // 上行信息的线程互斥量
  pthread_mutex_t      UL_INFO_mutex;
  /// NFAPI RX ULSCH information
  // RX ULSCH信号，属于NFAPI
  nfapi_rx_indication_pdu_t  rx_pdu_list[NFAPI_RX_IND_MAX_PDU];
  /// NFAPI RX ULSCH CRC information
  // RX ULSCH CRC信息
  nfapi_crc_indication_pdu_t crc_pdu_list[NFAPI_CRC_IND_MAX_PDU];
  /// NFAPI HARQ information
  // HARQ 信息
  nfapi_harq_indication_pdu_t harq_pdu_list[NFAPI_HARQ_IND_MAX_PDU];
  /// NFAPI SR information
  // SR 信息
  nfapi_sr_indication_pdu_t sr_pdu_list[NFAPI_SR_IND_MAX_PDU];
  /// NFAPI CQI information
  // CQI 信息，PDU和RAW
  nfapi_cqi_indication_pdu_t cqi_pdu_list[NFAPI_CQI_IND_MAX_PDU];
  /// NFAPI CQI information (raw component)
  nfapi_cqi_indication_raw_pdu_t cqi_raw_pdu_list[NFAPI_CQI_IND_MAX_PDU];
  /// NFAPI PRACH information
  // PRACH信息，PDU
  nfapi_preamble_pdu_t preamble_list[MAX_NUM_RX_PRACH_PREAMBLES];
#ifdef Rel14
  /// NFAPI PRACH information BL/CE UEs
  nfapi_preamble_pdu_t preamble_list_br[MAX_NUM_RX_PRACH_PREAMBLES];
#endif
  Sched_Rsp_t          Sched_INFO;
  // PDCCH相关
  LTE_eNB_PDCCH        pdcch_vars[2];
  // PHICH相关
  LTE_eNB_PHICH        phich_vars[2];
#ifdef Rel14
  // EPDCCH
  LTE_eNB_EPDCCH       epdcch_vars[2];
  // MPDCCH
  LTE_eNB_MPDCCH       mpdcch_vars[2];
  // PRACH
  LTE_eNB_PRACH        prach_vars_br;
#endif
  // 基站COMMON，基站侧收发数据
  LTE_eNB_COMMON       common_vars;
  // 基站上行控制信息
  LTE_eNB_UCI          uci_vars[NUMBER_OF_UE_MAX];
  // 基站探测参考信号，进行上行信道估计
  LTE_eNB_SRS          srs_vars[NUMBER_OF_UE_MAX];
  // 物理层广播信道
  LTE_eNB_PBCH         pbch;
  // 物理层上行共享信道
  LTE_eNB_PUSCH       *pusch_vars[NUMBER_OF_UE_MAX];
  // 物理层随机接入信道
  LTE_eNB_PRACH        prach_vars;
  /************************LTE_TRANSPORT中******************************************************/
  // 下行共享信道的传输
  LTE_eNB_DLSCH_t     *dlsch[NUMBER_OF_UE_MAX][2];   // Nusers times two spatial streams
  // 上行共享信道传输
  LTE_eNB_ULSCH_t     *ulsch[NUMBER_OF_UE_MAX+1];      // Nusers + number of RA
  // 下行共享信道，SI，RA，P
  LTE_eNB_DLSCH_t     *dlsch_SI,*dlsch_ra,*dlsch_p;
  // MCH
  LTE_eNB_DLSCH_t     *dlsch_MCH;
  // PCH
  LTE_eNB_DLSCH_t     *dlsch_PCH;
  // 用户的统计信息
  LTE_eNB_UE_stats     UE_stats[NUMBER_OF_UE_MAX];
  LTE_eNB_UE_stats    *UE_stats_ptr[NUMBER_OF_UE_MAX];

  /// cell-specific reference symbols
  // 参考信号的特定小区
  uint32_t         lte_gold_table[20][2][14];

  /// UE-specific reference symbols (p=5), TM 7
  // 参考信号的有用户
  uint32_t         lte_gold_uespec_port5_table[NUMBER_OF_UE_MAX][20][38];

  /// UE-specific reference symbols (p=7...14), TM 8/9/10
  // TM8/9/10
  uint32_t         lte_gold_uespec_table[2][20][2][21];

  /// mbsfn reference symbols
  // MBSFN 参考信号
  uint32_t         lte_gold_mbsfn_table[10][3][42];

  uint32_t X_u[64][839];
#ifdef Rel14
  uint32_t X_u_br[4][64][839];
#endif
  uint8_t pbch_configured;
  uint8_t pbch_pdu[4]; //PBCH_PDU_SIZE
  char eNB_generate_rar;

  /// Indicator set to 0 after first SR
  uint8_t first_sr[NUMBER_OF_UE_MAX];

  uint32_t max_peak_val;
  int max_eNB_id, max_sync_pos;

  /// \brief sinr for all subcarriers of the current link (used only for abstraction).
  /// first index: ? [0..N_RB_DL*12[
  // 子载波的SINR
  double *sinr_dB;

  /// N0 (used for abstraction)
  double N0;

  unsigned char first_run_timing_advance[NUMBER_OF_UE_MAX];
  unsigned char first_run_I0_measurements;

  unsigned char    is_secondary_eNB; // primary by default
  // 初始同步标志
  unsigned char    is_init_sync;     /// Flag to tell if initial synchronization is performed. This affects how often the secondary eNB will listen to the PSS from the primary system.
  // 预编码和信道估计
  unsigned char    has_valid_precoder; /// Flag to tell if secondary eNB has channel estimates to create NULL-beams from, and this B/F vector is created.
  // 当前基站的ID值
  unsigned char    PeNB_id;          /// id of Primary eNB

  /// hold the precoder for NULL beam to the primary user
  int              **dl_precoder_SeNB[3];
  char             log2_maxp; /// holds the maximum channel/precoder coefficient

  /// if ==0 enables phy only test mode
  // 是否加载mac层
  int mac_enabled;
  /// counter to average prach energh over first 100 prach opportunities
  // 前100个PRACH功率
  int prach_energy_counter;

  /************************物理信道变量***********************************/
  // PDSCH Varaibles
  // 物理层下行共享信道 变量
  PDSCH_CONFIG_DEDICATED pdsch_config_dedicated[NUMBER_OF_UE_MAX];

  // PUSCH Varaibles
  // 物理层上行共享信道变量
  PUSCH_CONFIG_DEDICATED pusch_config_dedicated[NUMBER_OF_UE_MAX];

  // PUCCH variables
  // 物理层上行控制信道变量
  PUCCH_CONFIG_DEDICATED pucch_config_dedicated[NUMBER_OF_UE_MAX];

  // UL-POWER-Control
  // 上行功率控制
  UL_POWER_CONTROL_DEDICATED ul_power_control_dedicated[NUMBER_OF_UE_MAX];

  // TPC
  TPC_PDCCH_CONFIG tpc_pdcch_config_pucch[NUMBER_OF_UE_MAX];
  TPC_PDCCH_CONFIG tpc_pdcch_config_pusch[NUMBER_OF_UE_MAX];

  // CQI reporting
  CQI_REPORT_CONFIG cqi_report_config[NUMBER_OF_UE_MAX];

  // SRS Variables
  // SRS变量
  SOUNDINGRS_UL_CONFIG_DEDICATED soundingrs_ul_config_dedicated[NUMBER_OF_UE_MAX];
  uint8_t ncs_cell[20][7];

  // Scheduling Request Config
  // 调度Request配置
  SCHEDULING_REQUEST_CONFIG scheduling_request_config[NUMBER_OF_UE_MAX];

  // Transmission mode per UE
  // UE 传输模式选择
  uint8_t transmission_mode[NUMBER_OF_UE_MAX];

  /// cba_last successful reception for each group, used for collision detection
  // 碰撞检测中使用
  uint8_t cba_last_reception[4];

  // Pointers for active physicalConfigDedicated to be applied in current subframe
  // 物理层配置检测
  struct PhysicalConfigDedicated *physicalConfigDedicated[NUMBER_OF_UE_MAX];


  uint32_t rb_mask_ul[4];

  /// Information regarding TM5
  MU_MIMO_mode mu_mimo_mode[NUMBER_OF_UE_MAX];

  /************************debug only************************************/
  /// target_ue_dl_mcs : only for debug purposes
  uint32_t target_ue_dl_mcs;
  /// target_ue_ul_mcs : only for debug purposes
  uint32_t target_ue_ul_mcs;
  /// target_ue_dl_rballoc : only for debug purposes
  uint32_t ue_dl_rb_alloc;
  /// target ul PRBs : only for debug
  uint32_t ue_ul_nb_rb;

  ///check for Total Transmissions
  uint32_t check_for_total_transmissions;

  ///check for MU-MIMO Transmissions
  // 多用户MIMO传输
  uint32_t check_for_MUMIMO_transmissions;

  ///check for SU-MIMO Transmissions
  // 单用户MIMO传输
  uint32_t check_for_SUMIMO_transmissions;

  ///check for FULL MU-MIMO Transmissions
  uint32_t  FULL_MUMIMO_transmissions;

  /// Counter for total bitrate, bits and throughput in downlink
  // 比特率，比特数，吞吐量
  uint32_t total_dlsch_bitrate;
  uint32_t total_transmitted_bits;
  uint32_t total_system_throughput;

  int hw_timing_advance;
  /*****************************时间相关的信息************************************************/
  time_stats_t phy_proc;
  time_stats_t phy_proc_tx;
  time_stats_t phy_proc_rx;
  time_stats_t rx_prach;

  time_stats_t ofdm_mod_stats;
  time_stats_t dlsch_encoding_stats;
  time_stats_t dlsch_modulation_stats;
  time_stats_t dlsch_scrambling_stats;
  time_stats_t dlsch_rate_matching_stats;
  time_stats_t dlsch_turbo_encoding_stats;
  time_stats_t dlsch_interleaving_stats;

  time_stats_t rx_dft_stats;
  time_stats_t ulsch_channel_estimation_stats;
  time_stats_t ulsch_freq_offset_estimation_stats;
  time_stats_t ulsch_decoding_stats;
  time_stats_t ulsch_demodulation_stats;
  time_stats_t ulsch_rate_unmatching_stats;
  time_stats_t ulsch_turbo_decoding_stats;
  time_stats_t ulsch_deinterleaving_stats;
  time_stats_t ulsch_demultiplexing_stats;
  time_stats_t ulsch_llr_stats;
  time_stats_t ulsch_tc_init_stats;
  time_stats_t ulsch_tc_alpha_stats;
  time_stats_t ulsch_tc_beta_stats;
  time_stats_t ulsch_tc_gamma_stats;
  time_stats_t ulsch_tc_ext_stats;
  time_stats_t ulsch_tc_intl1_stats;
  time_stats_t ulsch_tc_intl2_stats;

#ifdef LOCALIZATION
  /// time state for localization
  time_stats_t localization_stats;
#endif

  int32_t pucch1_stats_cnt[NUMBER_OF_UE_MAX][10];
  int32_t pucch1_stats[NUMBER_OF_UE_MAX][10*1024];
  int32_t pucch1_stats_thres[NUMBER_OF_UE_MAX][10*1024];
  int32_t pucch1ab_stats_cnt[NUMBER_OF_UE_MAX][10];
  int32_t pucch1ab_stats[NUMBER_OF_UE_MAX][2*10*1024];
  int32_t pusch_stats_rb[NUMBER_OF_UE_MAX][10240];
  int32_t pusch_stats_round[NUMBER_OF_UE_MAX][10240];
  int32_t pusch_stats_mcs[NUMBER_OF_UE_MAX][10240];
  int32_t pusch_stats_bsr[NUMBER_OF_UE_MAX][10240];
  int32_t pusch_stats_BO[NUMBER_OF_UE_MAX][10240];
} PHY_VARS_eNB;

#define debug_msg if (((mac_xface->frame%100) == 0) || (mac_xface->frame < 50)) msg

/// Top-level PHY Data Structure for UE
// UE的物理层数据结构体
typedef struct {
  /// \brief Module ID indicator for this instance
  uint8_t Mod_id;
  /// \brief Component carrier ID for this PHY instance
  uint8_t CC_id;
  /// \brief Mapping of CC_id antennas to cards
  openair0_rf_map      rf_map;
  //uint8_t local_flag;
  /// \brief Indicator of current run mode of UE (normal_txrx, rx_calib_ue, no_L2_connect, debug_prach)
  runmode_t mode;
  /// \brief Indicator that UE should perform band scanning
  int UE_scan;
  /// \brief Indicator that UE should perform coarse scanning around carrier
  int UE_scan_carrier;
  /// \brief Indicator that UE is synchronized to an eNB
  int is_synchronized;
  /// Data structure for UE process scheduling
  UE_proc_t proc;
  /// Flag to indicate the UE shouldn't do timing correction at all
  int no_timing_correction;
  /// \brief Total gain of the TX chain (16-bit baseband I/Q to antenna)
  uint32_t tx_total_gain_dB;
  /// \brief Total gain of the RX chain (antenna to baseband I/Q) This is a function of rx_gain_mode (and the corresponding gain) and the rx_gain of the card.
  uint32_t rx_total_gain_dB;
  /// \brief Total gains with maximum RF gain stage (ExpressMIMO2/Lime)
  uint32_t rx_gain_max[4];
  /// \brief Total gains with medium RF gain stage (ExpressMIMO2/Lime)
  uint32_t rx_gain_med[4];
  /// \brief Total gains with bypassed RF gain stage (ExpressMIMO2/Lime)
  uint32_t rx_gain_byp[4];
  /// \brief Current transmit power
  int16_t tx_power_dBm[10];
  /// \brief Total number of REs in current transmission
  int tx_total_RE[10];
  /// \brief Maximum transmit power
  int8_t tx_power_max_dBm;
  /// \brief Number of eNB seen by UE
  uint8_t n_connected_eNB;
  /// \brief indicator that Handover procedure has been initiated
  uint8_t ho_initiated;
  /// \brief indicator that Handover procedure has been triggered
  uint8_t ho_triggered;
  /// \brief Measurement variables.
  PHY_MEASUREMENTS measurements;
  LTE_DL_FRAME_PARMS  frame_parms;
  /// \brief Frame parame before ho used to recover if ho fails.
  LTE_DL_FRAME_PARMS  frame_parms_before_ho;
  LTE_UE_COMMON    common_vars;

  // point to the current rxTx thread index
  uint8_t current_thread_id[10];

  LTE_UE_PDSCH     *pdsch_vars[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX+1]; // two RxTx Threads
  LTE_UE_PDSCH_FLP *pdsch_vars_flp[NUMBER_OF_CONNECTED_eNB_MAX+1];
  LTE_UE_PDSCH     *pdsch_vars_SI[NUMBER_OF_CONNECTED_eNB_MAX+1];
  LTE_UE_PDSCH     *pdsch_vars_ra[NUMBER_OF_CONNECTED_eNB_MAX+1];
  LTE_UE_PDSCH     *pdsch_vars_p[NUMBER_OF_CONNECTED_eNB_MAX+1];
  LTE_UE_PDSCH     *pdsch_vars_MCH[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_PBCH      *pbch_vars[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_PDCCH     *pdcch_vars[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_PRACH     *prach_vars[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_DLSCH_t   *dlsch[RX_NB_TH_MAX][NUMBER_OF_CONNECTED_eNB_MAX][2]; // two RxTx Threads
  LTE_UE_ULSCH_t   *ulsch[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_DLSCH_t   *dlsch_SI[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_DLSCH_t   *dlsch_ra[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_DLSCH_t   *dlsch_p[NUMBER_OF_CONNECTED_eNB_MAX];
  LTE_UE_DLSCH_t   *dlsch_MCH[NUMBER_OF_CONNECTED_eNB_MAX];
  // This is for SIC in the UE, to store the reencoded data
  LTE_eNB_DLSCH_t  *dlsch_eNB[NUMBER_OF_CONNECTED_eNB_MAX];

  //Paging parameters
  uint32_t              IMSImod1024;
  uint32_t              PF;
  uint32_t              PO;

  // For abstraction-purposes only
  uint8_t               sr[10];
  uint8_t               pucch_sel[10];
  uint8_t               pucch_payload[22];

  UE_MODE_t        UE_mode[NUMBER_OF_CONNECTED_eNB_MAX];
  /// cell-specific reference symbols
  uint32_t lte_gold_table[7][20][2][14];

  /// UE-specific reference symbols (p=5), TM 7
  uint32_t lte_gold_uespec_port5_table[20][38];

  /// ue-specific reference symbols
  uint32_t lte_gold_uespec_table[2][20][2][21];

  /// mbsfn reference symbols
  uint32_t lte_gold_mbsfn_table[10][3][42];

  uint32_t X_u[64][839];

  uint32_t high_speed_flag;
  uint32_t perfect_ce;
  int16_t ch_est_alpha;
  int generate_ul_signal[NUMBER_OF_CONNECTED_eNB_MAX];

  UE_SCAN_INFO_t scan_info[NB_BANDS_MAX];

  char ulsch_no_allocation_counter[NUMBER_OF_CONNECTED_eNB_MAX];



  unsigned char ulsch_Msg3_active[NUMBER_OF_CONNECTED_eNB_MAX];
  uint32_t  ulsch_Msg3_frame[NUMBER_OF_CONNECTED_eNB_MAX];
  unsigned char ulsch_Msg3_subframe[NUMBER_OF_CONNECTED_eNB_MAX];
  PRACH_RESOURCES_t *prach_resources[NUMBER_OF_CONNECTED_eNB_MAX];
  int turbo_iterations, turbo_cntl_iterations;
  /// \brief ?.
  /// - first index: eNB [0..NUMBER_OF_CONNECTED_eNB_MAX[ (hard coded)
  uint32_t total_TBS[NUMBER_OF_CONNECTED_eNB_MAX];
  /// \brief ?.
  /// - first index: eNB [0..NUMBER_OF_CONNECTED_eNB_MAX[ (hard coded)
  uint32_t total_TBS_last[NUMBER_OF_CONNECTED_eNB_MAX];
  /// \brief ?.
  /// - first index: eNB [0..NUMBER_OF_CONNECTED_eNB_MAX[ (hard coded)
  uint32_t bitrate[NUMBER_OF_CONNECTED_eNB_MAX];
  /// \brief ?.
  /// - first index: eNB [0..NUMBER_OF_CONNECTED_eNB_MAX[ (hard coded)
  uint32_t total_received_bits[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_errors[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_errors_last[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_received[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_received_last[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_fer[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_SI_received[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_SI_errors[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_ra_received[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_ra_errors[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_p_received[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_p_errors[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mch_received_sf[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mch_received[NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mcch_received[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mtch_received[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mcch_errors[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mtch_errors[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mcch_trials[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int dlsch_mtch_trials[MAX_MBSFN_AREA][NUMBER_OF_CONNECTED_eNB_MAX];
  int current_dlsch_cqi[NUMBER_OF_CONNECTED_eNB_MAX];
  unsigned char first_run_timing_advance[NUMBER_OF_CONNECTED_eNB_MAX];
  uint8_t               generate_prach;
  uint8_t               prach_cnt;
  uint8_t               prach_PreambleIndex;
  //  uint8_t               prach_timer;
  uint8_t               decode_SIB;
  uint8_t               decode_MIB;
  int              rx_offset; /// Timing offset
  int              rx_offset_diff; /// Timing adjustment for ofdm symbol0 on HW USRP
  int              time_sync_cell;
  int              timing_advance; ///timing advance signalled from eNB
  int              hw_timing_advance;
  int              N_TA_offset; ///timing offset used in TDD
  /// Flag to tell if UE is secondary user (cognitive mode)
  unsigned char    is_secondary_ue;
  /// Flag to tell if secondary eNB has channel estimates to create NULL-beams from.
  unsigned char    has_valid_precoder;
  /// hold the precoder for NULL beam to the primary eNB
  int              **ul_precoder_S_UE;
  /// holds the maximum channel/precoder coefficient
  char             log2_maxp;

  /// if ==0 enables phy only test mode
  int mac_enabled;

  /// Flag to initialize averaging of PHY measurements
  int init_averaging;

  /// \brief sinr for all subcarriers of the current link (used only for abstraction).
  /// - first index: ? [0..12*N_RB_DL[
  double *sinr_dB;

  /// \brief sinr for all subcarriers of first symbol for the CQI Calculation.
  /// - first index: ? [0..12*N_RB_DL[
  double *sinr_CQI_dB;

  /// sinr_effective used for CQI calulcation
  double sinr_eff;

  /// N0 (used for abstraction)
  double N0;

  /// PDSCH Varaibles
  PDSCH_CONFIG_DEDICATED pdsch_config_dedicated[NUMBER_OF_CONNECTED_eNB_MAX];

  /// PUSCH Varaibles
  PUSCH_CONFIG_DEDICATED pusch_config_dedicated[NUMBER_OF_CONNECTED_eNB_MAX];

  /// PUSCH contention-based access vars
  PUSCH_CA_CONFIG_DEDICATED  pusch_ca_config_dedicated[NUMBER_OF_eNB_MAX]; // lola

  /// PUCCH variables

  PUCCH_CONFIG_DEDICATED pucch_config_dedicated[NUMBER_OF_CONNECTED_eNB_MAX];

  uint8_t ncs_cell[20][7];

  /// UL-POWER-Control
  UL_POWER_CONTROL_DEDICATED ul_power_control_dedicated[NUMBER_OF_CONNECTED_eNB_MAX];

  /// TPC
  TPC_PDCCH_CONFIG tpc_pdcch_config_pucch[NUMBER_OF_CONNECTED_eNB_MAX];
  TPC_PDCCH_CONFIG tpc_pdcch_config_pusch[NUMBER_OF_CONNECTED_eNB_MAX];

  /// CQI reporting
  CQI_REPORT_CONFIG cqi_report_config[NUMBER_OF_CONNECTED_eNB_MAX];

  /// SRS Variables
  SOUNDINGRS_UL_CONFIG_DEDICATED soundingrs_ul_config_dedicated[NUMBER_OF_CONNECTED_eNB_MAX];

  /// Scheduling Request Config
  SCHEDULING_REQUEST_CONFIG scheduling_request_config[NUMBER_OF_CONNECTED_eNB_MAX];

  /// Transmission mode per eNB
  uint8_t transmission_mode[NUMBER_OF_CONNECTED_eNB_MAX];

  time_stats_t phy_proc[RX_NB_TH];
  time_stats_t phy_proc_tx;
  time_stats_t phy_proc_rx[RX_NB_TH];

  uint32_t use_ia_receiver;

  time_stats_t ofdm_mod_stats;
  time_stats_t ulsch_encoding_stats;
  time_stats_t ulsch_modulation_stats;
  time_stats_t ulsch_segmentation_stats;
  time_stats_t ulsch_rate_matching_stats;
  time_stats_t ulsch_turbo_encoding_stats;
  time_stats_t ulsch_interleaving_stats;
  time_stats_t ulsch_multiplexing_stats;

  time_stats_t generic_stat;
  time_stats_t generic_stat_bis[RX_NB_TH][LTE_SLOTS_PER_SUBFRAME];
  time_stats_t ue_front_end_stat[RX_NB_TH];
  time_stats_t ue_front_end_per_slot_stat[RX_NB_TH][LTE_SLOTS_PER_SUBFRAME];
  time_stats_t pdcch_procedures_stat[RX_NB_TH];
  time_stats_t pdsch_procedures_stat[RX_NB_TH];
  time_stats_t pdsch_procedures_per_slot_stat[RX_NB_TH][LTE_SLOTS_PER_SUBFRAME];
  time_stats_t dlsch_procedures_stat[RX_NB_TH];

  time_stats_t ofdm_demod_stats;
  time_stats_t dlsch_rx_pdcch_stats;
  time_stats_t rx_dft_stats;
  time_stats_t dlsch_channel_estimation_stats;
  time_stats_t dlsch_freq_offset_estimation_stats;
  time_stats_t dlsch_decoding_stats[2];
  time_stats_t dlsch_demodulation_stats;
  time_stats_t dlsch_rate_unmatching_stats;
  time_stats_t dlsch_turbo_decoding_stats;
  time_stats_t dlsch_deinterleaving_stats;
  time_stats_t dlsch_llr_stats;
  time_stats_t dlsch_llr_stats_parallelization[RX_NB_TH][LTE_SLOTS_PER_SUBFRAME];
  time_stats_t dlsch_unscrambling_stats;
  time_stats_t dlsch_rate_matching_stats;
  time_stats_t dlsch_turbo_encoding_stats;
  time_stats_t dlsch_interleaving_stats;
  time_stats_t dlsch_tc_init_stats;
  time_stats_t dlsch_tc_alpha_stats;
  time_stats_t dlsch_tc_beta_stats;
  time_stats_t dlsch_tc_gamma_stats;
  time_stats_t dlsch_tc_ext_stats;
  time_stats_t dlsch_tc_intl1_stats;
  time_stats_t dlsch_tc_intl2_stats;
  time_stats_t tx_prach;

  /// RF and Interface devices per CC

  openair0_device rfdevice;
} PHY_VARS_UE;

/* this structure is used to pass both UE phy vars and
 * proc to the function UE_thread_rxn_txnp4
 */
 // UE线程收发数据
struct rx_tx_thread_data {
  // UE物理层数据
  PHY_VARS_UE    *UE;
  // UE收发过程的数据
  UE_rxtx_proc_t *proc;
};

void exit_fun(const char* s);

#include "UTIL/LOG/log_extern.h"
extern pthread_cond_t sync_cond;
extern pthread_mutex_t sync_mutex;
extern int sync_var;


#define MODE_DECODE_NONE         0
#define MODE_DECODE_SSE          1
#define MODE_DECODE_C            2
#define MODE_DECODE_AVX2         3

#define DECODE_INITTD8_SSE_FPTRIDX   0
#define DECODE_INITTD16_SSE_FPTRIDX  1
#define DECODE_INITTD_AVX2_FPTRIDX   2
#define DECODE_TD8_SSE_FPTRIDX       3
#define DECODE_TD16_SSE_FPTRIDX      4
#define DECODE_TD_C_FPTRIDX          5
#define DECODE_TD16_AVX2_FPTRIDX     6
#define DECODE_FREETD8_FPTRIDX       7
#define DECODE_FREETD16_FPTRIDX      8
#define DECODE_FREETD_AVX2_FPTRIDX   9
#define ENCODE_SSE_FPTRIDX           10
#define ENCODE_C_FPTRIDX             11
#define ENCODE_INIT_SSE_FPTRIDX      12
#define DECODE_NUM_FPTR              13

// IF格式解码
typedef uint8_t(*decoder_if_t)(int16_t *y,
                               int16_t *y2,
    		               uint8_t *decoded_bytes,
    		               uint8_t *decoded_bytes2,
	   		       uint16_t n,
	   		       uint16_t f1,
	   		       uint16_t f2,
	   		       uint8_t max_iterations,
	   		       uint8_t crc_type,
	   		       uint8_t F,
	   		       time_stats_t *init_stats,
	   		       time_stats_t *alpha_stats,
	   		       time_stats_t *beta_stats,
	   		       time_stats_t *gamma_stats,
	   		       time_stats_t *ext_stats,
	   		       time_stats_t *intl1_stats,
                               time_stats_t *intl2_stats);
// IF 格式编码
typedef uint8_t(*encoder_if_t)(uint8_t *input,
                               uint16_t input_length_bytes,
                               uint8_t *output,
                               uint8_t F,
                               uint16_t interleaver_f1,
                               uint16_t interleaver_f2);

#define MAX_RRU_CONFIG_SIZE 1024

// RRU配置类型信息
typedef enum {
  RAU_tick=0,
  RRU_capabilities=1,
  RRU_config=2,
  RRU_MSG_max_num=3,
  RRU_config_ok=4,
  RRU_start=5,
  RRU_stop=6,
  RRU_sync_ok=7,
  RRU_frame_resynch=8
} rru_config_msg_type_t;

// RRU配置信息
typedef struct RRU_CONFIG_msg_s {
  // 状态信息
  rru_config_msg_type_t type;
  // 长度
  ssize_t len;
  // RRU最长的配置长度
  uint8_t msg[MAX_RRU_CONFIG_SIZE];
} RRU_CONFIG_msg_t;

// 前传格式类型 0-4
typedef enum {
  OAI_IF5_only      =0,
  OAI_IF4p5_only    =1,
  OAI_IF5_and_IF4p5 =2,
  MBP_IF5           =3,
  MAX_FH_FMTs       =4
} FH_fmt_options_t;

// 每个RRU最大band数为4
#define MAX_BANDS_PER_RRU 4

// RRU容量信息
typedef struct RRU_capabilities_s {
  /// Fronthaul format
  // 前传网络类型
  FH_fmt_options_t FH_fmt;
  /// number of EUTRA bands (<=4) supported by RRU
  // RRU 支持的band数
  uint8_t          num_bands;
  /// EUTRA band list supported by RRU
  // RRU 支持的band列表
  uint8_t          band_list[MAX_BANDS_PER_RRU];
  /// Number of concurrent bands (component carriers)
  // 并发band数
  uint8_t          num_concurrent_bands;
  /// Maximum TX EPRE of each band
  // 最大物理下行参考信号功率
  int8_t           max_pdschReferenceSignalPower[MAX_BANDS_PER_RRU];
  /// Maximum RX gain of each band
  // 每个band的接收增益
  uint8_t          max_rxgain[MAX_BANDS_PER_RRU];
  /// Number of RX ports of each band
  // 每个Band的接收天线数
  uint8_t          nb_rx[MAX_BANDS_PER_RRU];
  /// Number of TX ports of each band
  // 每个Band的接收天线数
  uint8_t          nb_tx[MAX_BANDS_PER_RRU];
  /// max DL bandwidth (1,6,15,25,50,75,100)
  // 下行带宽的最大值
  uint8_t          N_RB_DL[MAX_BANDS_PER_RRU];
  /// max UL bandwidth (1,6,15,25,50,75,100)
  // 上行带宽的最大值
  uint8_t          N_RB_UL[MAX_BANDS_PER_RRU];
} RRU_capabilities_t;

typedef struct RRU_config_s {

  /// Fronthaul format
  // 前传类型
  RU_if_south_t FH_fmt;
  /// number of EUTRA bands (<=4) configured in RRU
  // RRU 中的band数
  uint8_t num_bands;
  /// EUTRA band list configured in RRU
  // RRU中配置的band列表
  uint8_t band_list[MAX_BANDS_PER_RRU];
  /// TDD configuration (0-6)
  // TDD配置
  uint8_t tdd_config[MAX_BANDS_PER_RRU];
  /// TDD special subframe configuration (0-10)
  // TDD 子帧配置
  uint8_t tdd_config_S[MAX_BANDS_PER_RRU];
  /// TX frequency
  // 发送频率
  uint32_t tx_freq[MAX_BANDS_PER_RRU];
  /// RX frequency
  // 接收频率
  uint32_t rx_freq[MAX_BANDS_PER_RRU];
  /// TX attenation w.r.t. max
  // 发送衰减
  uint8_t att_tx[MAX_BANDS_PER_RRU];
  /// RX attenuation w.r.t. max
  // 接收衰减参数
  uint8_t att_rx[MAX_BANDS_PER_RRU];
  /// DL bandwidth
  // 下行带宽
  uint8_t N_RB_DL[MAX_BANDS_PER_RRU];
  /// UL bandwidth
  // 上行带宽
  uint8_t N_RB_UL[MAX_BANDS_PER_RRU];
  /// 3/4 sampling rate
  // 3/4采样率
  uint8_t threequarter_fs[MAX_BANDS_PER_RRU];
  /// prach_FreqOffset for IF4p5
  // IF4p5 PRACH信道频偏
  int prach_FreqOffset[MAX_BANDS_PER_RRU];
  /// prach_ConfigIndex for IF4p5
  // PRACH配置索引
  int prach_ConfigIndex[MAX_BANDS_PER_RRU];
#ifdef Rel14
  int emtc_prach_CElevel_enable[MAX_BANDS_PER_RRU][4];
  /// emtc_prach_FreqOffset for IF4p5 per CE Level
  int emtc_prach_FreqOffset[MAX_BANDS_PER_RRU][4];
  /// emtc_prach_ConfigIndex for IF4p5 per CE Level
  int emtc_prach_ConfigIndex[MAX_BANDS_PER_RRU][4];
#endif
} RRU_config_t;

/*static函数只在当前文件中执行，inline可以提高函数执行的效率，
  完成同步函数
  返回值为空，输入参数为线程名
*/
static inline void wait_sync(char *thread_name) {

  printf( "waiting for sync (%s)\n",thread_name);
  // 对互斥量加锁
  pthread_mutex_lock( &sync_mutex );
  // 等待完成同步
  while (sync_var<0)
    pthread_cond_wait( &sync_cond, &sync_mutex );
  // 释放锁
  pthread_mutex_unlock(&sync_mutex);

  printf( "got sync (%s)\n", thread_name);

}

/* 等待环境变量
@param mutex 线程互斥量
@param cond 线程条件变量
@param instance_cnt 实例数量
@Parma name 线程名称
*/
static inline int wait_on_condition(pthread_mutex_t *mutex,
                                    pthread_cond_t *cond,
                                    int *instance_cnt,
                                    char *name) {
  // 加锁失败，直接返回-1
  if (pthread_mutex_lock(mutex) != 0) {
    LOG_E( PHY, "[SCHED][eNB] error locking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  // 当同步的实例数量不小于0时，处于等待状态
  while (*instance_cnt < 0) {
    // most of the time the thread is waiting here
    // proc->instance_cnt_rxtx is -1
    // 等待条件成立，然后在锁定
    pthread_cond_wait(cond,mutex); // this unlocks mutex_rxtx while waiting and then locks it again
  }
  // 释放互斥量，如果失败，直接退出
  if (pthread_mutex_unlock(mutex) != 0) {
    LOG_E(PHY,"[SCHED][eNB] error unlocking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  return(0);
}

/* 繁忙时等待函数
*/
static inline int wait_on_busy_condition(pthread_mutex_t *mutex,pthread_cond_t *cond,int *instance_cnt,char *name) {
  // 加锁失败，直接退出
  if (pthread_mutex_lock(mutex) != 0) {
    LOG_E( PHY, "[SCHED][eNB] error locking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  // 当实例数量为0时，先解锁mutex，然后等待一户再加锁
  while (*instance_cnt == 0) {
    // most of the time the thread will skip this
    // waits only if proc->instance_cnt_rxtx is 0
    pthread_cond_wait(cond,mutex); // this unlocks mutex_rxtx while waiting and then locks it again
  }

  if (pthread_mutex_unlock(mutex) != 0) {
    LOG_E(PHY,"[SCHED][eNB] error unlocking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  return(0);
}

/* 释放线程，返回值为int，参数为互斥量，实例数量，线程名
*/
static inline int release_thread(pthread_mutex_t *mutex,int *instance_cnt,char *name) {
  // 加锁
  if (pthread_mutex_lock(mutex) != 0) {
    LOG_E( PHY, "[SCHED][eNB] error locking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  // 实例数量-1
  *instance_cnt=*instance_cnt-1;
  // 解锁
  if (pthread_mutex_unlock(mutex) != 0) {
    LOG_E( PHY, "[SCHED][eNB] error unlocking mutex for %s\n",name);
    exit_fun("nothing to add");
    return(-1);
  }
  return(0);
}


#include "PHY/INIT/defs.h"
#include "PHY/LTE_REFSIG/defs.h"
#include "PHY/MODULATION/defs.h"
#include "PHY/LTE_TRANSPORT/proto.h"
#include "PHY/LTE_ESTIMATION/defs.h"

#include "SIMULATION/ETH_TRANSPORT/defs.h"
#endif //  __PHY_DEFS__H__
