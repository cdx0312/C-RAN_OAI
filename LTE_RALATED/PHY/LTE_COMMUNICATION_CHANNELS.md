# Channel

* Logical Channels: 定义空口传播的数据类型，也就是传输信道，控制信道，系统广播等，逻辑信道处于RLC和MAC之间。

* Transport Channels: 定义空口中数据是如何传播的，PHY和MAC之间

* Physical Channels：空口中实际传播数据的信道。

## Logical Channels

逻辑信道可以分为控制信道和传输信道。控制信道分为通用信道和专用信道，通用信道用于点对多点，专用信道点对点传输。

Channel Name | Acronym | Control channel |Traffic channel
---- | --- | --- | ---
Broadcast Control Channel | BCCH | X | 	 
Paging Control Channel | PCCH | X |	 
Common Control Channel | CCCH |	X | 	 
Dedicated Control Channel | DCCH | X |	 
Multicast Control Channel | MCCH | X |	 
Dedicated Traffic Channel | DTCH |   | X
Multicast Traffic Channel	| MTCH |   | X

## Transport Channels


Channel Name | Acronym | Downlink | Uplink
---- | --- | --- | ---
Broadcast Channel | BCH | X |
Downlink Shared Channel | DL-SCH | X | 	 
Paging Channel | PCH | X | 	 
Multicast Channel | MCH | X |	 
Uplink Shared Channel | UL-SCH |  | X
Random Access Channel | RACH | 	 |	X

## Physical Channels

### Physical data channels

Channel Name | Acronym | Downlink | Uplink
---- | --- | --- | ---
Physical downlink shared channel | PDSCH | X |	 
Physical broadcast channel | PBCH	| X |	 
Physical multicast channel | PMCH |	X	 |
Physical uplink shared channel | PUSCH |	 | X
Physical random access channel	| PRACH	|  | X

### Physical Control Channels

Channel Name	| Acronym |	Downlink |	Uplink
---- | --- | --- | ---
Physical control format indicator channel |	PCFICH	| X |	 
Physical hybrid ARQ indicator channel	| PHICH |	X	 |
Physical downlink control channel	|PDCCH|	X	 |
Relay physical downlink control channel |	R-PDCCH |	X |	 
Physical uplink control channel	| PUCCH	|	| X
