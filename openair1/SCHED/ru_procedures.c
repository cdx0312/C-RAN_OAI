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

/*! \file ru_procedures.c
 * \brief Implementation of RU procedures
 * \author R. Knopp, F. Kaltenberger, N. Nikaein, X. Foukas
 * \date 2011
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr,navid.nikaein@eurecom.fr, x.foukas@sms.ed.ac.uk
 * \note
 * \warning
 */

#include "PHY/defs.h"
#include "PHY/extern.h"
#include "SCHED/defs.h"
#include "SCHED/extern.h"

#include "PHY/LTE_TRANSPORT/if4_tools.h"
#include "PHY/LTE_TRANSPORT/if5_tools.h"

#include "LAYER2/MAC/extern.h"
#include "LAYER2/MAC/defs.h"
#include "UTIL/LOG/log.h"
#include "UTIL/LOG/vcd_signal_dumper.h"

#include "T.h"

#include "assertions.h"
#include "msc.h"

#include <time.h>

#include "targets/RT/USER/rt_wrapper.h"

// RU OFDM Modulator, used in IF4p5 RRU, RCC/RAU with IF5, eNodeB
// RU OFDM调制，C-RAN中使用

extern openair0_config_t openair0_cfg[MAX_CARDS];

extern int oai_exit;

/* front-end process 前端发送过程
@param ru RU数据信息
@param slot 时隙数
*/
void feptx0(RU_t *ru,
            int slot)
{
  // 帧结构
  LTE_DL_FRAME_PARMS *fp = &ru->frame_parms;
  //int dummy_tx_b[7680*2] __attribute__((aligned(32)));

  unsigned int aa,slot_offset;
  int i, tx_offset;
  // 频域时隙数
  int slot_sizeF = (fp->ofdm_symbol_size)*
                   ((fp->Ncp==1) ? 6 : 7);
  // 接收子帧数
  int subframe = ru->proc.subframe_tx;


  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM+slot , 1 );

  slot_offset = subframe*fp->samples_per_tti + (slot*(fp->samples_per_tti>>1));

  //LOG_D(PHY,"SFN/SF:RU:TX:%d/%d Generating slot %d\n",ru->proc.frame_tx, ru->proc.subframe_tx,slot);
  // 遍历基站发送天线
  for (aa=0; aa<ru->nb_tx; aa++) {
    if (fp->Ncp == EXTENDED)
      // NCPWieEXTENDED的物理层OFDM调制
      PHY_ofdm_mod(&ru->common.txdataF_BF[aa][slot*slot_sizeF],
					            (int*)&ru->common.txdata[aa][slot_offset],
					            fp->ofdm_symbol_size,
					            6,
					            fp->nb_prefix_samples,
					            CYCLIC_PREFIX);
    else
      // NORMAL 前缀调制
      normal_prefix_mod(&ru->common.txdataF_BF[aa][slot*slot_sizeF],
					             (int*)&ru->common.txdata[aa][slot_offset],
					             7,
					             fp);

   /*
    len = fp->samples_per_tti>>1;


    if ((slot_offset+len)>(LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti)) {
      tx_offset = (int)slot_offset;
      txdata = (int16_t*)&ru->common.txdata[aa][tx_offset];
      len2 = -tx_offset+LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;
      for (i=0; i<(len2<<1); i++) {
	txdata[i] = ((int16_t*)dummy_tx_b)[i];
      }
      txdata = (int16_t*)&ru->common.txdata[aa][0];
      for (j=0; i<(len<<1); i++,j++) {
	txdata[j++] = ((int16_t*)dummy_tx_b)[i];
      }
    }
    else {
      tx_offset = (int)slot_offset;
      txdata = (int16_t*)&ru->common.txdata[aa][tx_offset];
      memcpy((void*)txdata,(void*)dummy_tx_b,len<<2);
    }
   */
    // TDD: turn on tx switch N_TA_offset before by setting buffer in these samples to 0
    if ((slot == 0) &&
        (fp->frame_type == TDD) &&
        ((fp->tdd_config==0) ||
         (fp->tdd_config==1) ||
         (fp->tdd_config==2) ||
         (fp->tdd_config==6)) &&
        ((subframe==0) || (subframe==5))) {
      for (i=0; i<ru->N_TA_offset; i++) {
         // 设置偏移量
	       tx_offset = (int)slot_offset+i-ru->N_TA_offset/2;
	       if (tx_offset<0)
            tx_offset += LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;

	       if (tx_offset>=(LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti))
	          tx_offset -= LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;

	       ru->common.txdata[aa][tx_offset] = 0x00000000;
       }
     }
   }
   VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM+slot , 0);
}

/* 前端发送线程
@param param RU数据信息
*/
static void *feptx_thread(void *param) {
  // ru和proc的赋值
  RU_t *ru = (RU_t *)param;
  RU_proc_t *proc  = &ru->proc;
  // 通用线程初始化函数
  thread_top_init("feptx_thread",0,870000,1000000,1000000);

  while (!oai_exit) {
    // 获取线程锁，开始执行
    if (wait_on_condition(&proc->mutex_feptx,&proc->cond_feptx,&proc->instance_cnt_feptx,"feptx thread")<0)
      break;
    //调用前传发送函数
    feptx0(ru,1);
    // 释放线程
    if (release_thread(&proc->mutex_feptx,&proc->instance_cnt_feptx,"feptx thread")<0)
      break;
    // 唤醒线程
    if (pthread_cond_signal(&proc->cond_feptx) != 0) {
      printf("[eNB] ERROR pthread_cond_signal for feptx thread exit\n");
      exit_fun( "ERROR pthread_cond_signal" );
      return NULL;
    }
  }



  return(NULL);
}

/* 前传发送调制2线程
@param ru RU数据信息
*/
void feptx_ofdm_2thread(RU_t *ru) {
  // fp，proc，subframe赋值
  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;
  RU_proc_t *proc = &ru->proc;
  struct timespec wait;
  int subframe = ru->proc.subframe_tx;

  wait.tv_sec=0;
  wait.tv_nsec=5000000L;

  start_meas(&ru->ofdm_mod_stats);
  // 子帧类型为上行子帧，退出
  if (subframe_select(fp,subframe) == SF_UL)
    return;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM , 1 );
  // 子帧为下行子帧
  if (subframe_select(fp,subframe)==SF_DL) {
    // If this is not an S-subframe
    // 同步子帧，退出
    if (pthread_mutex_timedlock(&proc->mutex_feptx,&wait) != 0) {
      printf("[RU] ERROR pthread_mutex_lock for feptx thread (IC %d)\n", proc->instance_cnt_feptx);
      exit_fun( "error locking mutex_feptx" );
      return;
    }
    // 前端传输实例为0.则退出并解锁proc前传互斥量
    if (proc->instance_cnt_feptx==0) {
      printf("[RU] FEPtx thread busy\n");
      exit_fun("FEPtx thread busy");
      pthread_mutex_unlock( &proc->mutex_feptx );
      return;
    }
    // 前传实例+1
    ++proc->instance_cnt_feptx;


    if (pthread_cond_signal(&proc->cond_feptx) != 0) {
      printf("[RU] ERROR pthread_cond_signal for feptx thread\n");
      exit_fun( "ERROR pthread_cond_signal" );
      return;
    }

    pthread_mutex_unlock( &proc->mutex_feptx );
  }

  // call first slot in this thread
  // 调用线程的第一个接收时隙
  feptx0(ru,0);
  wait_on_busy_condition(&proc->mutex_feptx,&proc->cond_feptx,&proc->instance_cnt_feptx,"feptx thread");

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM , 0 );

  stop_meas(&ru->ofdm_mod_stats);

}

/* 前端发送OFDM函数
@param ru RU数据类型
*/
void feptx_ofdm(RU_t *ru) {
  // 帧结构
  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;

  unsigned int aa,slot_offset, slot_offset_F;
  int dummy_tx_b[7680*4] __attribute__((aligned(32)));
  int i,j, tx_offset;
  int slot_sizeF = (fp->ofdm_symbol_size)*
                   ((fp->Ncp==1) ? 6 : 7);
  int len,len2;
  int16_t *txdata;
  int subframe = ru->proc.subframe_tx;

  //  int CC_id = ru->proc.CC_id;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM , 1 );

  slot_offset_F = 0;

  slot_offset = subframe*fp->samples_per_tti;
  // 下行子帧或者同步子帧
  if ((subframe_select(fp,subframe)==SF_DL)||
      ((subframe_select(fp,subframe)==SF_S))) {
    //    LOG_D(HW,"Frame %d: Generating slot %d\n",frame,next_slot);

    start_meas(&ru->ofdm_mod_stats);
    // 遍历天线数，根据循环前缀进行不同 的调制
    for (aa=0; aa<ru->nb_tx; aa++) {
      if (fp->Ncp == EXTENDED) {
        PHY_ofdm_mod(&ru->common.txdataF_BF[aa][0],
                     dummy_tx_b,
                     fp->ofdm_symbol_size,
                     6,
                     fp->nb_prefix_samples,
                     CYCLIC_PREFIX);
        PHY_ofdm_mod(&ru->common.txdataF_BF[aa][slot_sizeF],
                     dummy_tx_b+(fp->samples_per_tti>>1),
                     fp->ofdm_symbol_size,
                     6,
                     fp->nb_prefix_samples,
                     CYCLIC_PREFIX);
      } else {
        normal_prefix_mod(&ru->common.txdataF_BF[aa][slot_offset_F],
                          dummy_tx_b,
                          7,
                          fp);
      	// if S-subframe generate first slot only
        // 下行子帧，生成第一个时隙
      	if (subframe_select(fp,subframe) == SF_DL)
      	  normal_prefix_mod(&ru->common.txdataF_BF[aa][slot_offset_F+slot_sizeF],
      			    dummy_tx_b+(fp->samples_per_tti>>1),
      			    7,
      			    fp);
      }

      // if S-subframe generate first slot only
      // 同步子帧，只生成第一个时隙
      if (subframe_select(fp,subframe) == SF_S)
	       len = fp->samples_per_tti>>1;
      else
	       len = fp->samples_per_tti;
          /*
          for (i=0;i<len;i+=4) {
    	dummy_tx_b[i] = 0x100;
    	dummy_tx_b[i+1] = 0x01000000;
    	dummy_tx_b[i+2] = 0xff00;
    	dummy_tx_b[i+3] = 0xff000000;
    	}*/
      // 时隙偏移量小于0
      if (slot_offset<0) {
	       txdata = (int16_t*)&ru->common.txdata[aa][(LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti)+tx_offset];
         len2 = -(slot_offset);
	       len2 = (len2>len) ? len : len2;
	       for (i=0; i<(len2<<1); i++) {
	          txdata[i] = ((int16_t*)dummy_tx_b)[i];
          }
      	 if (len2<len) {
      	   txdata = (int16_t*)&ru->common.txdata[aa][0];
      	   for (j=0; i<(len<<1); i++,j++) {
      	     txdata[j++] = ((int16_t*)dummy_tx_b)[i];
      	   }
      	 }
       } else if ((slot_offset+len)>(LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti)) {
         // 时隙偏移量+长度大于TTI
         tx_offset = (int)slot_offset;
	       txdata = (int16_t*)&ru->common.txdata[aa][tx_offset];
         len2 = -tx_offset+LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;
        	for (i=0; i<(len2<<1); i++) {
        	  txdata[i] = ((int16_t*)dummy_tx_b)[i];
        	}
          txdata = (int16_t*)&ru->common.txdata[aa][0];
          for (j=0; i<(len<<1); i++,j++) {
            txdata[j++] = ((int16_t*)dummy_tx_b)[i];
          }
        } else {
        	//LOG_D(PHY,"feptx_ofdm: Writing to position %d\n",slot_offset);
        	tx_offset = (int)slot_offset;
        	txdata = (int16_t*)&ru->common.txdata[aa][tx_offset];

        	for (i=0; i<(len<<1); i++) {
        	  txdata[i] = ((int16_t*)dummy_tx_b)[i];
        	}
      }

     // if S-subframe switch to RX in second subframe
      /*
     if (subframe_select(fp,subframe) == SF_S) {
       for (i=0; i<len; i++) {
	 ru->common_vars.txdata[0][aa][tx_offset++] = 0x00010001;
       }
     }
      */
      // TDD,0126,子帧为0,5，计算发送偏移量
     if ((fp->frame_type == TDD) &&
         ((fp->tdd_config==0) ||
	   (fp->tdd_config==1) ||
	   (fp->tdd_config==2) ||
	   (fp->tdd_config==6)) &&
	     ((subframe==0) || (subframe==5))) {
       // turn on tx switch N_TA_offset before
       //LOG_D(HW,"subframe %d, time to switch to tx (N_TA_offset %d, slot_offset %d) \n",subframe,ru->N_TA_offset,slot_offset);
       for (i=0; i<ru->N_TA_offset; i++) {
         tx_offset = (int)slot_offset+i-ru->N_TA_offset/2;
         if (tx_offset<0)
           tx_offset += LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;

         if (tx_offset>=(LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti))
           tx_offset -= LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_tti;

         ru->common.txdata[aa][tx_offset] = 0x00000000;
       }
     }

     stop_meas(&ru->ofdm_mod_stats);
     LOG_D(PHY,"feptx_ofdm (TXPATH): frame %d, subframe %d: txp (time %p) %d dB, txp (freq) %d dB\n",
	   ru->proc.frame_tx,subframe,txdata,dB_fixed(signal_energy((int32_t*)txdata,fp->samples_per_tti)),
	   dB_fixed(signal_energy_nodc(ru->common.txdataF_BF[aa],2*slot_sizeF)));
    }
  }
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_OFDM , 0 );
}

/* 前端发送预编码函数
@param ru RU数据信息
*/
void feptx_prec(RU_t *ru) {

  int l,i,aa;
  // 基站列表
  PHY_VARS_eNB **eNB_list = ru->eNB_list,*eNB;
  LTE_DL_FRAME_PARMS *fp;
  int32_t ***bw;
  // 发送子帧
  int subframe = ru->proc.subframe_tx;
  // 只有一个基站实例
  if (ru->num_eNB == 1) {
    eNB = eNB_list[0];
    fp  = &eNB->frame_parms;

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_PREC , 1);
    // 从RU赋值数据到基站
    for (aa=0;aa<ru->nb_tx;aa++)
      memcpy((void*)ru->common.txdataF_BF[aa],
	     (void*)&eNB->common_vars.txdataF[aa][subframe*fp->symbols_per_tti*fp->ofdm_symbol_size],
	     fp->symbols_per_tti*fp->ofdm_symbol_size*sizeof(int32_t));

    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPTX_PREC , 0);
  } else {
    // 多个基站实例
    for (i=0;i<ru->num_eNB;i++) {
      eNB = eNB_list[i];
      fp  = &eNB->frame_parms;
      bw  = ru->beam_weights[i];

      for (l=0;l<fp->symbols_per_tti;l++) {
	       for (aa=0;aa<ru->nb_tx;aa++) {
            // 预编码函数
        	  beam_precoding(eNB->common_vars.txdataF,
        			 ru->common.txdataF_BF,
        			 fp,
        			 bw,
        			 subframe<<1,
        			 l,
        			 aa);
          }
      }
      #if 0
            LOG_D(PHY,"feptx_prec: frame %d, subframe %d: txp (freq) %d dB\n",
      	    ru->proc.frame_tx,subframe,
      	    dB_fixed(signal_energy_nodc(ru->common.txdataF_BF[0],2*fp->symbols_per_tti*fp->ofdm_symbol_size)));
      #endif
    }
  }
}

/* 前端过程函数
@param ru RU数据信息
@param slot 时隙数
*/
void fep0(RU_t *ru,int slot) {
  // proc， fp赋值
  RU_proc_t *proc       = &ru->proc;
  LTE_DL_FRAME_PARMS *fp = &ru->frame_parms;
  int l;

  //  printf("fep0: slot %d\n",slot);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPRX+slot, 1);
  // remove 7.5khz
  remove_7_5_kHz(ru,(slot&1)+(proc->subframe_rx<<1));
  // 每2个符号，进行此上行前传
  for (l=0; l<fp->symbols_per_tti/2; l++) {
    slot_fep_ul(ru,
		l,
		(slot&1),
		0
		);
  }
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPRX+slot, 0);
}


/* 前端进程
@param param RU数据信息
*/
static void *fep_thread(void *param) {

  RU_t *ru = (RU_t *)param;
  RU_proc_t *proc  = &ru->proc;
  // 通用线程初始化
  thread_top_init("fep_thread",0,870000,1000000,1000000);

  while (!oai_exit) {

    if (wait_on_condition(&proc->mutex_fep,&proc->cond_fep,&proc->instance_cnt_fep,"fep thread")<0) break;
    // fep0，上行
    fep0(ru,0);
    // 释放线程
    if (release_thread(&proc->mutex_fep,&proc->instance_cnt_fep,"fep thread")<0) break;
    // 退出线程
    if (pthread_cond_signal(&proc->cond_fep) != 0) {
      printf("[eNB] ERROR pthread_cond_signal for fep thread exit\n");
      exit_fun( "ERROR pthread_cond_signal" );
      return NULL;
    }
  }



  return(NULL);
}

/* 初始化前端发送线程
@param ru RU数据信息
@param attr_feptx 前端发送线程参数
*/
void init_feptx_thread(RU_t *ru,
                       pthread_attr_t *attr_feptx) {

  RU_proc_t *proc = &ru->proc;

  proc->instance_cnt_feptx         = -1;
  // 初始化互斥量和环境变量
  pthread_mutex_init( &proc->mutex_feptx, NULL);
  pthread_cond_init( &proc->cond_feptx, NULL);
  // 创建feptx线程
  pthread_create(&proc->pthread_feptx, attr_feptx, feptx_thread, (void*)ru);


}

/* 初始化前端线程
@param ru RU数据信息
@param attr_feptx 前端发送线程参数
*/
void init_fep_thread(RU_t *ru,
                     pthread_attr_t *attr_fep) {

  RU_proc_t *proc = &ru->proc;

  proc->instance_cnt_fep         = -1;

  pthread_mutex_init( &proc->mutex_fep, NULL);
  pthread_cond_init( &proc->cond_fep, NULL);

  pthread_create(&proc->pthread_fep, attr_fep, fep_thread, (void*)ru);


}

/* Ru前端full2线程
@param ru RU数据信息
*/
void ru_fep_full_2thread(RU_t *ru) {

  RU_proc_t *proc = &ru->proc;

  struct timespec wait;

  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;
  // TDD帧并且子帧为上行子帧，退出
  if ((fp->frame_type == TDD) &&
     (subframe_select(fp,proc->subframe_rx) != SF_UL)) return;

  if (ru->idx == 0)
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPRX, 1 );

  wait.tv_sec=0;
  wait.tv_nsec=5000000L;

  start_meas(&ru->ofdm_demod_stats);

  if (pthread_mutex_timedlock(&proc->mutex_fep,&wait) != 0) {
    printf("[RU] ERROR pthread_mutex_lock for fep thread (IC %d)\n", proc->instance_cnt_fep);
    exit_fun( "error locking mutex_fep" );
    return;
  }
  // 前传网络实例为0
  if (proc->instance_cnt_fep==0) {
    printf("[RU] FEP thread busy\n");
    exit_fun("FEP thread busy");
    pthread_mutex_unlock( &proc->mutex_fep );
    return;
  }

  ++proc->instance_cnt_fep;


  if (pthread_cond_signal(&proc->cond_fep) != 0) {
    printf("[RU] ERROR pthread_cond_signal for fep thread\n");
    exit_fun( "ERROR pthread_cond_signal" );
    return;
  }

  pthread_mutex_unlock( &proc->mutex_fep );

  // call second slot in this symbol
  // 调用符号中第二个时隙
  fep0(ru,1);

  wait_on_busy_condition(&proc->mutex_fep,&proc->cond_fep,&proc->instance_cnt_fep,"fep thread");

  stop_meas(&ru->ofdm_demod_stats);
}


/* 全前端函数
@param ru RU数据信息
*/
void fep_full(RU_t *ru) {
  // proc，fp赋值
  RU_proc_t *proc = &ru->proc;
  int l;
  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;
  // TDD帧并且子帧为上行子帧，退出
  if ((fp->frame_type == TDD) && (subframe_select(fp,proc->subframe_rx) != SF_UL))
    return;

  start_meas(&ru->ofdm_demod_stats);
  if (ru->idx == 0)
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPRX, 1 );
  // 7.5kHz频移
  remove_7_5_kHz(ru,proc->subframe_rx<<1);
  remove_7_5_kHz(ru,1+(proc->subframe_rx<<1));

  // 每两个符号，做fep上行
  for (l=0; l<fp->symbols_per_tti/2; l++) {
    slot_fep_ul(ru,
		l,
		0,
		0
		);
    slot_fep_ul(ru,
		l,
		1,
		0
		);
  }
  if (ru->idx == 0)
    VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_PHY_PROCEDURES_RU_FEPRX, 0 );
  stop_meas(&ru->ofdm_demod_stats);


}


/* RU PRACH函数
@param ru RU数据信息
*/
void do_prach_ru(RU_t *ru) {
  // proc，，fp赋值
  RU_proc_t *proc = &ru->proc;
  LTE_DL_FRAME_PARMS *fp=&ru->frame_parms;

  // check if we have to detect PRACH first
  // 判断当前子帧为PRACH子帧，进行PRACH过程
  if (is_prach_subframe(fp,proc->frame_prach,proc->subframe_prach)>0) {
    //accept some delay in processing - up to 5ms
    int i;
    // 时延5 ms
    for (i = 0; i < 10 && proc->instance_cnt_prach == 0; i++) {
      LOG_W(PHY,"Frame %d Subframe %d, PRACH thread busy (IC %d)!!\n", proc->frame_prach,proc->subframe_prach,
	    proc->instance_cnt_prach);
      usleep(500);
    }
    // 检查PRACH实例数量
    if (proc->instance_cnt_prach == 0) {
      exit_fun( "PRACH thread busy" );
      return;
    }

    // wake up thread for PRACH RX
    // 唤醒PRACH接受线程
    if (pthread_mutex_lock(&proc->mutex_prach) != 0) {
      LOG_E( PHY, "ERROR pthread_mutex_lock for PRACH thread (IC %d)\n", proc->instance_cnt_prach );
      exit_fun( "error locking mutex_prach" );
      return;
    }

    ++proc->instance_cnt_prach;

    // the thread can now be woken up
    // 唤醒PRACH线程
    if (pthread_cond_signal(&proc->cond_prach) != 0) {
      LOG_E( PHY, "ERROR pthread_cond_signal for PRACH thread\n");
      exit_fun( "ERROR pthread_cond_signal" );
      return;
    }

    pthread_mutex_unlock( &proc->mutex_prach );
  }

}
