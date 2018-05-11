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

/*!\brief Initilization and reconfiguration routines for LTE PHY */
#include "defs.h"
#include "PHY/extern.h"
#include "PHY/CODING/extern.h"
/* 初始化并配置LTE物理层
@param frame_parms, LTE下行帧结构
*/
void init_lte_top(LTE_DL_FRAME_PARMS *frame_parms)
{
  // 循环冗余校验表的吃书画
  crcTableInit();
  // 长度为7的编码表的初始化
  ccodedot11_init();
  // Input in LSB, followed by state in 6 MSBs
  ccodedot11_init_inv();
  // lte 长度为7的编码表初始化
  ccodelte_init();
  ccodelte_init_inv();


  // 维特比表的生成
  phy_generate_viterbi_tables();
  phy_generate_viterbi_tables_lte();
  // 加载编码类库
  load_codinglib();
  // 同步初始化
  lte_sync_time_init(frame_parms);
  // 生成上行参考信号
  generate_ul_ref_sigs();
  generate_ul_ref_sigs_rx();

  // 生成64QAM表
  generate_64qam_table();
  generate_16qam_table();
  generate_RIV_tables();
  // 初始化下行共享新到scrambling_lut
  init_unscrambling_lut();
  init_scrambling_lut();
  //set_taus_seed(1328);


}

// 释放编码库和同步库
void free_lte_top(void)
{
  free_codinglib();
  lte_sync_time_free();

  /* free_ul_ref_sigs() is called in phy_free_lte_eNB() */
}


/*
 * @}*/
