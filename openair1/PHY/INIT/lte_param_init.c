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

#include <string.h>
#include <math.h>
#include <unistd.h>
#include <execinfo.h>
#include <signal.h>

#include "SIMULATION/TOOLS/defs.h"
#include "PHY/types.h"
#include "PHY/defs.h"
#include "PHY/extern.h"

extern PHY_VARS_eNB *eNB;
extern PHY_VARS_UE *UE;
extern RU_t *ru;
extern void  phy_init_RU(RU_t*);

/* LTE参数初始化
@param eNBp 基站侧物理层变量
@param UEp 用户侧物理层变量
@param **rup RU的进程
@param N_tx_port_eNB 基站侧发送端口数
@param N_tx_phy 物理层发送天线数量
@param N_rx_ru RU侧接收天线数量
@param N_rx_ue UE侧接收天线数量
@param transmission_mode 传输模式
@param extended_prefix_flag 扩展前缀标识
@param frame_type 帧类型
@param Nid_cell 小区ID号
@param tdd_config TDD配置
@param N_RB_DL 下行资源块数量
@param pa 功率
@param threequarter_fs 3/4采样率
@param osf 过采样倍数
@param perfect_ce
 */
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
		    uint32_t perfect_ce)
{
	// LTE下行帧结构参数
  LTE_DL_FRAME_PARMS *frame_parms;
  int i;
	// 基站物理层变量
  PHY_VARS_eNB *eNB;
	// 用户物理层变量
  PHY_VARS_UE  *UE;
	// RU数据
  RU_t         *ru;
  printf("Start lte_param_init\n");
	// 为基站，UE和RU初始化分配内存并赋值
  *eNBp = malloc(sizeof(PHY_VARS_eNB));
  *UEp = malloc(sizeof(PHY_VARS_UE));
  *rup = malloc(sizeof(RU_t));
  eNB = *eNBp;
  UE  = *UEp;
  ru  = *rup;
  printf("eNB %p, UE %p, ru %p\n",eNB,UE,ru);



  memset((void*)eNB,0,sizeof(PHY_VARS_eNB));
  memset((void*)UE,0,sizeof(PHY_VARS_UE));
  memset((void*)ru,0,sizeof(RU_t));
	// RU，基站之间的依赖
  ru->eNB_list[0] = eNB;
  eNB->RU_list[0] = ru;
  ru->num_eNB=1;

  srand(0);
	// 随机生成seed
  randominit(0);
	// 设置taus seed
  set_taus_seed(0);

	/*************************** 帧结构参数的初始化 ********************************/
  frame_parms = &(eNB->frame_parms);
	// 上下行资源块数量
  frame_parms->N_RB_DL            = N_RB_DL;   //50 for 10MHz and 25 for 5 MHz
  frame_parms->N_RB_UL            = N_RB_DL;
	// 3/4采样率
  frame_parms->threequarter_fs    = threequarter_fs;
	// Ncp 上下行
  frame_parms->Ncp                = extended_prefix_flag;
  frame_parms->Ncp_UL             = extended_prefix_flag;
  frame_parms->Nid_cell           = Nid_cell;
  frame_parms->nushift            = Nid_cell%6;
	// 天线端口数量
  frame_parms->nb_antennas_tx     = N_tx_phy;
  frame_parms->nb_antennas_rx     = N_rx_ru;
  frame_parms->nb_antenna_ports_eNB = N_tx_port_eNB;
	// 物理层HARQ信道相关
  frame_parms->phich_config_common.phich_resource         = oneSixth;
  frame_parms->phich_config_common.phich_duration         = normal;
	// TDD设置和帧类型
  frame_parms->tdd_config         = tdd_config;
  frame_parms->frame_type         = frame_type;
  //  frame_parms->Csrs = 2;
  //  frame_parms->Bsrs = 0;
  //  frame_parms->kTC = 0;44
  //  frame_parms->n_RRC = 0;

	// 初始化帧结构参数
  init_frame_parms(frame_parms,osf);

  //copy_lte_parms_to_phy_framing(frame_parms, &(PHY_config->PHY_framing));

  //  phy_init_top(frame_parms); //allocation
	// 设置UE相关的参数
  UE->is_secondary_ue = 0;
  UE->frame_parms = *frame_parms;
  UE->frame_parms.nb_antennas_rx=N_rx_ue;
  //  eNB->frame_parms = *frame_parms;
	// 设置RU相关的参数
  ru->frame_parms = *frame_parms;
  ru->nb_tx = N_tx_phy;
  ru->nb_rx = N_rx_ru;
  ru->if_south = LOCAL_RF;
	//配置方式和传输模式设定
  eNB->configured=1;

  eNB->transmission_mode[0] = transmission_mode;
  UE->transmission_mode[0] = transmission_mode;

	// 将帧结构的参数dump到日志中去
  dump_frame_parms(frame_parms);

  UE->measurements.n_adj_cells=0;
  UE->measurements.adj_cell_id[0] = Nid_cell+1;
  UE->measurements.adj_cell_id[1] = Nid_cell+2;
	// GOLD编码
  for (i=0; i<3; i++)
    lte_gold(frame_parms,UE->lte_gold_table[i],Nid_cell+i);

  printf("Calling init_lte_ue_signal\n");
	// 初始化UE信号
  init_lte_ue_signal(UE,1,0);
  printf("Calling phy_init_lte_eNB\n");
	// 初始化基站物理层
  phy_init_lte_eNB(eNB,0,0);
  printf("Calling phy_init_RU (%p)\n",ru);
	// 初始化RU
  phy_init_RU(ru);
	// 生成物理层控制格式指示信道映射关系，该信道用于指示一个子帧中用于传输PDCCH的OFDM符号数，该信道属于下行物理信
  generate_pcfich_reg_mapping(&UE->frame_parms);
	// 生成物理混合自动重传指示信道映射关系，物理混合自动重传指示信道。PHICH用于对PUSCH传输的数据回应HARQ ACK/NACK。每个TTI中的每个上行TB对应一个PHICH，也就是说，当UE在某小区配置了上行空分复用时，需要2个PHICH。
  generate_phich_reg_mapping(&UE->frame_parms);

  // DL power control init
  //if (transmission_mode == 1) {
	// 初始化下行控制功率
  UE->pdsch_config_dedicated->p_a  = pa;
	// 1 or 7
  if (transmission_mode == 1 || transmission_mode ==7) {
    ((eNB->frame_parms).pdsch_config_common).p_b = 0;
    ((UE->frame_parms).pdsch_config_common).p_b = 0;
  } else { // rho_a = rhob
    ((eNB->frame_parms).pdsch_config_common).p_b = 1;
    ((UE->frame_parms).pdsch_config_common).p_b = 1;
  }

  UE->perfect_ce = perfect_ce;

  /* the UE code is multi-thread "aware", we need to setup this array */
  for (i = 0; i < 10; i++)
		UE->current_thread_id[i] = i % 2;

	// TDD 模式初始化，根据下行资源块的数量来设置RU的TA便宜
  if (eNB->frame_parms.frame_type == TDD) {
    if (eNB->frame_parms.N_RB_DL == 100)
				ru->N_TA_offset = 624;
    else if (eNB->frame_parms.N_RB_DL == 50)
				ru->N_TA_offset = 624/2;
    else if (eNB->frame_parms.N_RB_DL == 25)
				ru->N_TA_offset = 624/4;
  } else // FDD模式
		ru->N_TA_offset=0;

  printf("Done lte_param_init\n");

}
