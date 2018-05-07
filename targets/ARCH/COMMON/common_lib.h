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

/*! \file common_lib.h
 * \brief common APIs for different RF frontend device
 * \author HongliangXU, Navid Nikaein
 * \date 2015
 * \version 0.2
 * \company Eurecom
 * \maintainer:  navid.nikaein@eurecom.fr
 * \note
 * \warning
 */

#ifndef COMMON_LIB_H
#define COMMON_LIB_H
#include <stdint.h>
#include <sys/types.h>

/* name of shared library implementing the radio front end */
#define OAI_RF_LIBNAME        "oai_device"
/* name of shared library implementing the transport */
#define OAI_TP_LIBNAME        "oai_transpro"

/* flags for BBU to determine whether the attached radio head is local or remote */
#define RAU_LOCAL_RADIO_HEAD  0
#define RAU_REMOTE_RADIO_HEAD 1

#ifndef MAX_CARDS
#define MAX_CARDS 8
#endif

typedef int64_t openair0_timestamp;
typedef volatile int64_t openair0_vtimestamp;


/*!\brief structrue holds the parameters to configure USRP devices*/
typedef struct openair0_device_t openair0_device;






//#define USRP_GAIN_OFFSET (56.0)  // 86 calibrated for USRP B210 @ 2.6 GHz to get equivalent RS EPRE in OAI to SMBV100 output

typedef enum {
  max_gain=0,med_gain,byp_gain
} rx_gain_t;

#if OCP_FRAMEWORK
#include <enums.h>
#else
typedef enum {
  duplex_mode_TDD=1,duplex_mode_FDD=0
} duplex_mode_t;
#endif


/** @addtogroup _GENERIC_PHY_RF_INTERFACE_
 * @{
 */
/*!\brief RF device types
 */
#ifdef OCP_FRAMEWORK
#include <enums.h>
#else
typedef enum {
  MIN_RF_DEV_TYPE = 0,
  /*!\brief device is ExpressMIMO */
  EXMIMO_DEV,
  /*!\brief device is USRP B200/B210*/
  USRP_B200_DEV,
  /*!\brief device is USRP X300/X310*/
  USRP_X300_DEV,
  /*!\brief device is BLADE RF*/
  BLADERF_DEV,
  /*!\brief device is LMSSDR (SoDeRa)*/
  LMSSDR_DEV,
  /*!\brief device is NONE*/
  NONE_DEV,
  MAX_RF_DEV_TYPE

} dev_type_t;
#endif

/*!\brief transport protocol types
 */
typedef enum {
  MIN_TRANSP_TYPE = 0,
  /*!\brief transport protocol ETHERNET */
  ETHERNET_TP,
  /*!\brief no transport protocol*/
  NONE_TP,
  MAX_TRANSP_TYPE

} transport_type_t;


/*!\brief  openair0 device host type */
typedef enum {
  MIN_HOST_TYPE = 0,
 /*!\brief device functions within a RAU */
  RAU_HOST,
 /*!\brief device functions within a RRU */
  RRU_HOST,
  MAX_HOST_TYPE

}host_type_t;


/*! \brief RF Gain clibration */
typedef struct {
  //! Frequency for which RX chain was calibrated
  double freq;
  //! Offset to be applied to RX gain
  double offset;
} rx_gain_calib_table_t;

/*! \brief Clock source types */
typedef enum {
  //! This tells the underlying hardware to use the internal reference
  internal=0,
  //! This tells the underlying hardware to use the external reference
  external=1,
  //! This tells the underlying hardware to use the gpsdo reference
  gpsdo=2
} clock_source_t;

/*! \brief RF frontend parameters set by application */
// 射频前传参数结构体
typedef struct {
  //! Module ID for this configuration
  // 模块ID
  int Mod_id;
  //! device log level
  // 设备日志级别
  int log_level;
  //! duplexing mode
  // 复用模式  duplex_mode_TDD=1,duplex_mode_FDD=0
  duplex_mode_t duplex_mode;
  //! number of downlink resource blocks
  // 下行资源块的数量
  int num_rb_dl;
  //! number of samples per frame
  // 每个帧结构里面的采样数
  unsigned int  samples_per_frame;
  //! the sample rate for both transmit and receive.
  // 采样率
  double sample_rate;
  //! flag to indicate that the device is doing mmapped DMA transfers
  // 标志传输是否是mmapped DMA
  int mmapped_dma;
  //! offset in samples between TX and RX paths
  // 样本信号的偏移量
  int tx_sample_advance;
  //! samples per packet on the fronthaul interface
  // 包中的样本数
  int samples_per_packet;
  //! number of RX channels (=RX antennas)
  // 接收信道数量（天线数）
  int rx_num_channels;
  //! number of TX channels (=TX antennas)
  // 发送信道的数量（天线数）
  int tx_num_channels;
  //! \brief RX base addresses for mmapped_dma
  // 接收端地址
  int32_t* rxbase[4];
  //! \brief TX base addresses for mmapped_dma
  // 发送端地址
  int32_t* txbase[4];
  //! \brief Center frequency in Hz for RX.
  //! index: [0..rx_num_channels[
  // 接收端中心频率
  double rx_freq[4];
  //! \brief Center frequency in Hz for TX.
  //! index: [0..rx_num_channels[ !!! see lte-ue.c:427 FIXME iterates over rx_num_channels
  // 发送端中心频率
  double tx_freq[4];
  //! \brief memory
  //! \brief Pointer to Calibration table for RX gains
  增益校准表
  rx_gain_calib_table_t *rx_gain_calib_table;

  //! mode for rxgain (ExpressMIMO2)
  // 接收端增益
  rx_gain_t rxg_mode[4];
  //! \brief Gain for RX in dB.
  //! index: [0..rx_num_channels]
  // db增益
  double rx_gain[4];
  //! \brief Gain offset (for calibration) in dB
  //! index: [0..rx_num_channels]
  // db 偏移增益
  double rx_gain_offset[4];
  //! gain for TX in dB
  // 发送增益
  double tx_gain[4];
  //! RX bandwidth in Hz
  // 接收带宽
  double rx_bw;
  //! TX bandwidth in Hz
  // 发送带宽
  double tx_bw;
  //! clock source
  // 10MHz C-RAN
  clock_source_t clock_source;
  //! timing_source
  // PPS C-RAN
  clock_source_t time_source;
  //! Auto calibration flag
  // 自动校准标志
  int autocal[4];
  //! rf devices work with x bits iqs when oai have its own iq format
  //! the two following parameters are used to convert iqs
  // IQ 变换
  int iq_txshift;
  int iq_rxrescale;
  //! Configuration file for LMS7002M
  char *configFilename;
#if defined(USRP_REC_PLAY)
  unsigned short sf_mode;           // 1=record, 2=replay
  char           sf_filename[1024]; // subframes file path
  unsigned int   sf_max;            // max number of recorded subframes
  unsigned int   sf_loops;          // number of loops in replay mode
  unsigned int   sf_read_delay;     // read delay in replay mode
  unsigned int   sf_write_delay;    // write delay in replay mode
  unsigned int   eth_mtu;           // ethernet MTU
#endif
} openair0_config_t;

/*! \brief RF mapping */
typedef struct {
  //! card id
  int card;
  //! rf chain id
  int chain;
} openair0_rf_map;

// 网络参数结构体
typedef struct {
  // 远程IP
  char *remote_addr;
  //! remote port number for Ethernet interface (control)
  // 远程端口号,控制端口
  uint16_t remote_portc;
  //! remote port number for Ethernet interface (user)
  // 远程端口号，用户端口
  uint16_t remote_portd;
  //! local IP/MAC addr for Ethernet interface (eNB/RAU, UE)
  // 本地IP地址
  char *my_addr;
  //! local port number (control) for Ethernet interface (eNB/RAU, UE)
  // 本地控制端口
  uint16_t  my_portc;
  //! local port number (user) for Ethernet interface (eNB/RAU, UE)
  // 本地用户端口
  uint16_t  my_portd;
  //! local Ethernet interface (eNB/RAU, UE)
  // 本地网络接口类型 (eNB/RAU, UE)
  char *local_if_name;
  //! transport type preference  (RAW/UDP)
  // 传输类型：(RAW/UDP)
  uint8_t transp_preference;
  //! compression enable (0: No comp/ 1: A-LAW)
  // 压缩：0-no 1-A-law
  uint8_t if_compress;
} eth_params_t;


typedef struct {
  //! Tx buffer for if device, keep one per subframe now to allow multithreading
  void *tx[10];
  //! Tx buffer (PRACH) for if device
  void *tx_prach;
  //! Rx buffer for if device
  void *rx;
} if_buffer_t;


/*!\brief structure holds the parameters to configure USRP devices */
// 配置USRP设备的结构体
struct openair0_device_t {
  /*!brief Module ID of this device */
  // 设备的模块ID
  int Mod_id;

  /*!brief Component Carrier ID of this device */
  // 成员载波id
  int CC_id;

  /*!brief Type of this device */
  // 设备类型，枚举类型
  dev_type_t type;

  /*!brief Transport protocol type that the device suppports (in case I/Q samples need to be transported) */
  // 设备支持的传输协议类型，CRAN采用的传输类型为ETHERNET
  transport_type_t transp_type;

   /*!brief Type of the device's host (RAU/RRU) */
  // 设备的host类型，RAU，RRU
  host_type_t host_type;

  /* !brief RF frontend parameters set by application */
  // 应用设定的射频前端参数
  openair0_config_t *openair0_cfg;

  /* !brief ETH params set by application */
  // 网络参数
  eth_params_t *eth_params;

  /* !brief Indicates if device already initialized */
  // 是否初始化
  int is_init;


  /*!brief Can be used by driver to hold internal structure*/
  // 保存内部结构体
  void *priv;

  /* Functions API, which are called by the application*/

  /*! \brief Called to start the transceiver. Return 0 if OK, < 0 if error
      @param device pointer to the device structure specific to the RF hardware target
  */
  // 收发函数
  int (*trx_start_func)(openair0_device *device);

 /*! \brief Called to configure the device
      @param device pointer to the device structure specific to the RF hardware target
  */

  // 配置硬件函数
  int (*trx_config_func)(openair0_device* device, openair0_config_t *openair0_cfg);

  /*! \brief Called to send a request message between RAU-RRU on control port
      @param device pointer to the device structure specific to the RF hardware target
      @param msg pointer to the message structure passed between RAU-RRU
      @param msg_len length of the message
  */
  // 通过控制端口在RAU和RRU之间发送一个请求信息
  int (*trx_ctlsend_func)(openair0_device *device, void *msg, ssize_t msg_len);

  /*! \brief Called to receive a reply  message between RAU-RRU on control port
      @param device pointer to the device structure specific to the RF hardware target
      @param msg pointer to the message structure passed between RAU-RRU
      @param msg_len length of the message
  */
  // 从控制端口收到发送的控制信息
  int (*trx_ctlrecv_func)(openair0_device *device, void *msg, ssize_t msg_len);

  /*! \brief Called to send samples to the RF target
      @param device pointer to the device structure specific to the RF hardware target
      @param timestamp The timestamp at whicch the first sample MUST be sent
      @param buff Buffer which holds the samples
      @param nsamps number of samples to be sent
      @param antenna_id index of the antenna if the device has multiple anteannas
      @param flags flags must be set to TRUE if timestamp parameter needs to be applied
  */
  // 向射频目标发射信号
  int (*trx_write_func)(openair0_device *device, openair0_timestamp timestamp, void **buff, int nsamps,int antenna_id, int flags);

  /*! \brief Receive samples from hardware.
   * Read \ref nsamps samples from each channel to buffers. buff[0] is the array for
   * the first channel. *ptimestamp is the time at which the first sample
   * was received.
   * \param device the hardware to use
   * \param[out] ptimestamp the time at which the first sample was received.
   * \param[out] buff An array of pointers to buffers for received samples. The buffers must be large enough to hold the number of samples \ref nsamps.
   * \param nsamps Number of samples. One sample is 2 byte I + 2 byte Q => 4 byte.
   * \param antenna_id Index of antenna for which to receive samples
   * \returns the number of sample read
   */
   // 从硬件接收信息
  int (*trx_read_func)(openair0_device *device, openair0_timestamp *ptimestamp, void **buff, int nsamps,int antenna_id);

  /*! \brief print the device statistics
   * \param device the hardware to use
   * \returns  0 on success
   */
   // 打印设备的状态信息
  int (*trx_get_stats_func)(openair0_device *device);

  /*! \brief Reset device statistics
   * \param device the hardware to use
   * \returns 0 in success
   */
   // reset硬件信息
  int (*trx_reset_stats_func)(openair0_device *device);

  /*! \brief Terminate operation of the transceiver -- free all associated resources
   * \param device the hardware to use
   */
   // 停止收发,释放资源
  void (*trx_end_func)(openair0_device *device);

  /*! \brief Stop operation of the transceiver
   */
   // 停止收发
  int (*trx_stop_func)(openair0_device *device);

  /* Functions API related to UE*/

  /*! \brief Set RX feaquencies
   * \param device the hardware to use
   * \param openair0_cfg RF frontend parameters set by application
   * \param exmimo_dump_config  dump EXMIMO configuration
   * \returns 0 in success
   */
  int (*trx_set_freq_func)(openair0_device* device, openair0_config_t *openair0_cfg,int exmimo_dump_config);

  /*! \brief Set gains
   * \param device the hardware to use
   * \param openair0_cfg RF frontend parameters set by application
   * \returns 0 in success
   */
  int (*trx_set_gains_func)(openair0_device* device, openair0_config_t *openair0_cfg);

  /*! \brief RRU Configuration callback
   * \param idx RU index
   * \param arg pointer to capabilities or configuration
   */
  void (*configure_rru)(int idx, void* arg);
};

/* type of device init function, implemented in shared lib */
typedef int(*oai_device_initfunc_t)(openair0_device *device, openair0_config_t *openair0_cfg);
/* type of transport init function, implemented in shared lib */
typedef int(*oai_transport_initfunc_t)(openair0_device *device, openair0_config_t *openair0_cfg, eth_params_t * eth_params);

#ifdef __cplusplus
extern "C"
{
#endif


  /*! \brief Initialize openair RF target. It returns 0 if OK */
  int openair0_device_load(openair0_device *device, openair0_config_t *openair0_cfg);
  /*! \brief Initialize transport protocol . It returns 0 if OK */
  int openair0_transport_load(openair0_device *device, openair0_config_t *openair0_cfg, eth_params_t * eth_params);


 /*! \brief Get current timestamp of USRP
  * \param device the hardware to use
  */
  openair0_timestamp get_usrp_time(openair0_device *device);

 /*! \brief Set RX frequencies
  * \param device the hardware to use
  * \param openair0_cfg RF frontend parameters set by application
  * \returns 0 in success
  */
  int openair0_set_rx_frequencies(openair0_device* device, openair0_config_t *openair0_cfg);

/*@}*/

#ifdef __cplusplus
}
#endif

#endif // COMMON_LIB_H
