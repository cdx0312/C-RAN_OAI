This folder contains the software for the device driver of CBMIMO1 and the physical layer signal processing.

/*************************************************************************************************/
* openair1层主要包含设备软件驱动和物理层信号处理
      * PHY: 包含了所有与物理层相关的信号处理，子文件夹可以看下边的介绍，算了，还是写一下吧
            * CODING: 编码相关，包括turbo及传统编码方式的编码和解码，速率匹配，循环冗余校验，其实用这个来学习LTE中的编码是非常好的，可以自己调函数接口来用，打印编码的结果。
            * INIT: 初始化相关，包括初始化变量和结构体，初始化RU等
            * LTE_ESTIMATION: 参数估计相关，信道估计，基站用户参数估计等
            * LTE_REFSIG: 参考信号相关，同步和导频序列等
            * LTE_TRANSPORT: 传输相关，包括各个物理层信道的传输数据等
            * TOOLS: 常用工具
            * defs.h: 最顶层的.h文件，其实就是讲整个物理层的头文件都引入到这个里面了，做了比较详细的注解
            * 其他就没必要一个一个写了
      * SCHED: 调度物理层函数，OAI项目中的基站和UE是写在一起的，可能考虑到一些函数复用的问题，但其实这样看起来很不直观，虽然使用角度是透明的，但是实际上你要修改什么很繁琐，研究的时候需要记住你看的到底是基站还是用户，我这次主要看的是基站。
            * 主要还是分为了物理层通用调度，物理层基站调度，物理层用户调度，很好理解，不展开
      * SIMULATION: 仿真，没研究过，OAI的仿真系统一直不太好用吧，晓光师姐说她来这边一直在调仿真，不知道进展如何，仿真这块其实就是虚拟物理层，其他上层都是一样的，当你有物理层设备的时候，直接调试比仿真更加直观，可以用这个仿真来做一些算法验证，思路其实就是理清楚调度的流程，替换相关函数，打印输出，恩，适合博士生。
/*************************************************************************************************/

|-- PHY			   // contains all signal processing related to physical layer (used in real-time AND simulation)

General remarks: every directory contains at least defs.h (for declaration of structure and functions), vars.h (for definitions of variables), and extern.h (variables are defined only once in cbmimo1_device.c (for real-time HW operation) or a top-level simulation (in SIMULATION/xxx), if variables are needed in another file, use extern.h)

|   |-- CODING		   // turbo and convolutional (de-)coders, rate-matching, crc generation (implementation of 36-212)
|   |   `-- TESTBENCH      // This contains unitary testbenches for the different channel coders/decoders
|   |-- INIT		   // variables defined in vars.h are initialzed here (memory allocation)
|   |-- LTE_ESTIMATION     // estimation for LTE
|   |-- LTE_REFSIG 	   // reference signals for LTE (sync and pilot sequences) reference signals from 36-211
|   |-- LTE_TRANSPORT      // these are the top level routines for different transport and physical channels (for example DL-SCH, PSS) implements a subset of 36-211,36-212
|   |-- MODULATION	   // modulation and demodulation includes FFT/SC-FDMA front-end processing from 36-211
|   |-- TOOLS		   // tools like (I)FFT, vector operations, matrix multiplication, etc.
|   |   `-- FFTTEST        // unitary testbench for FFT routines
|   |-- defs.h		  // top level defs.h (includes spec_defs, impl_defs and all defs.h of subdirectories)
|   |-- extern.h
|   |-- impl_defs.h       // non-LTE openair implementation
|   |-- impl_defs_lte.h   // lte implementation
|   |-- impl_defs_top.h   // general implementaion defs
|   |-- spec_defs.h
|   |-- spec_defs_top.h
|   |-- types.h
|   `-- vars.h
|-- SCHED		  // schedules the different PHY functions according to the nodes role
|   |-- defs.h
|   |-- extern.h
|   |-- phy_procedures_emos.c    // non-LTE PHY procedures for EMOS
|   |-- phy_procedures_emos.h
|   |-- phy_procedures_lte_eNb.c // LTE PHY procedures for eNB (from 36-213)
|   |-- phy_procedures_lte_ue.c  // LTE PHY procedures for UE (from 36-213)
|   |-- phy_procedures_lte_common.c  // LTE PHY procedures common for UE and eNB (from 36-213)
|   |-- phy_mac_stub.c  //MAC stub that generates channels when used in phy-test-mode
|   |-- pucch_pc.c // power control for PUCCH
|   |-- pusch_pc.c // power control for PUSCH
|   |-- srs_pc.c // power control for SRS
|   |-- rt_compat.h
|   `-- vars.h
|-- SIMULATION		 // contains simulation routines to test PHY
|   |-- LTE_PHY          // LTE simulation testbenches for unitary simulation of transport/physical channels
|   |   |-- pbchsim.c       // PBCH simulation testbench
|   |   |-- pdcchsim.c      // PCFICH/PHICH/PDCCH (DCI) simulation testbench
|   |   |-- dlsim.c         // PDSCH simulation testbench
|   |   |-- ulsim.c         // PUSCH simulation testbench
|   |   |-- pucchsim.c      // PUCCH simulation testbench
|   |-- RF
|   |-- ETH_TRANSPORT
|   `-- TOOLS
