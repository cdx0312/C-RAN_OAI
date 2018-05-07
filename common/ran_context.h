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

/*! \file PHY/impl_defs_lte.h
* \brief LTE Physical channel configuration and variable structure definitions
* \author R. Knopp, F. Kaltenberger
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr
* \note
* \warning
* RAN侧的上下文环境，包含一些结构体的定义等内容，
*/

#ifndef __RAN_CONTEXT_H__
#define __RAN_CONTEXT_H__

#include <pthread.h>
#include "COMMON/platform_constants.h"
#include "PHY/defs.h"
#include "PHY/types.h"
#include "PHY/impl_defs_top.h"
#include "PHY/impl_defs_lte.h"
#include "RRC/LITE/defs.h"
#include "flexran_agent_defs.h"

#include "gtpv1u.h"
#include "NwGtpv1u.h"
#include "NwGtpv1uMsg.h"
#include "NwGtpv1uPrivate.h"
#include "gtpv1u_eNB_defs.h"

#include "PHY/defs_L1_NB_IoT.h"
#include "RRC/LITE/defs_NB_IoT.h"
/* RAN_CONTEXT_t, 一般称为RC，记录
*/
typedef struct {
  /// RAN context config file name
  // RAN侧 配置文件的名称
  char *config_file_name;
  /// Number of RRC instances in this node
  // RRC实体数量
  int nb_inst;
  /// Number of Component Carriers per instance in this node
  // 每个RCC实体所包含的子载波数量
  int *nb_CC;
  /// Number of NB_IoT instances in this node
  // 当前节点中的nb-iot实例数量
  int nb_nb_iot_rrc_inst;
  /// Number of MACRLC instances in this node
  // 节点中的MACRLC实例
  int nb_macrlc_inst;
  /// Number of NB_IoT MACRLC instances in this node
  // 节点中的NBIOT中MACRLC实例数量
  int nb_nb_iot_macrlc_inst;
  /// Number of component carriers per instance in this node
  int *nb_mac_CC;
  /// Number of L1 instances in this node
  // 节点中的L1实例数量
  int nb_L1_inst;
  /// Number of NB_IoT L1 instances in this node
  int nb_nb_iot_L1_inst;
  /// Number of Component Carriers per instance in this node
  // 每个L1实体中子载波数量
  int *nb_L1_CC;
  /// Number of RU instances in this node
  // 节点中RU实例的数量
  int nb_RU;
  /// FlexRAN context variables
  // 可扩展RAN侧上下文环境变量（并没有理解这个是什么东西，后面会补充）
  flexran_agent_info_t **flexran;
  /// eNB context variables
  // 基站物理层变量
  struct PHY_VARS_eNB_s ***eNB;
  /// NB_IoT L1 context variables
  struct PHY_VARS_eNB_NB_IoT_s **L1_NB_IoT;
  /// RRC context variables
  // RRC 上下文变量
  struct eNB_RRC_INST_s **rrc;
  /// NB_IoT RRC context variables
  // struct eNB_RRC_INST_NB_IoT_s **nb_iot_rrc;
  /// MAC context variables
  // 基站MACRLC实例环境变量
  struct eNB_MAC_INST_s **mac;
  /// NB_IoT MAC context variables
  struct eNB_MAC_INST_NB_IoT_s **nb_iot_mac;
  /// GTPu descriptor
  // GTPU 指示器
  gtpv1u_data_t *gtpv1u_data_g;
  /// RU descriptors. These describe what each radio unit is supposed to do and contain the necessary functions for fronthaul interfaces
  // RU 结构体，射频单元需要做的事情，及前传接口必要的函数
  struct RU_t_s **ru;
  /// Mask to indicate fronthaul setup status of RU (hard-limit to 64 RUs)
  // RU的前传网络的状态标志位
  uint64_t ru_mask;
  /// Mutex for protecting ru_mask
  // ru_mast的互斥量
  pthread_mutex_t ru_mutex;
  /// condition variable for signaling setup completion of an RU
  // ru的环境变量
  pthread_cond_t ru_cond;
} RAN_CONTEXT_t;


#endif
