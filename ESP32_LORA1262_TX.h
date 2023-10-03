/********************************************************************************************
*	SX1262.h - Library for SX1262
*	Copyright(C) 2018 NiceRF.All right reserved.  
*	@version 1.0  
*	This library is suit for LoRa1262 in loRa mode
*	please make sure the supply of you board is UNDER 3.3V!! Or the module will be destory!!
*	The configuration of both module should be same.
*********************************************************************************************/
#ifndef __SX1262_H__
#define __SX1262_H__

#include "Arduino.h"

//typedef		boolean			bool;
//typedef 	unsigned char	uint8_t;
//typedef 	unsigned int	uint16_t;
//typedef 	unsigned long	uint32_t;

typedef struct
{
	uint32_t rf_freq;
	int8_t  tx_power;
	uint8_t  lora_sf;
	uint8_t  band_width;
	uint8_t  code_rate;
	uint8_t	 payload_size;
	
}loRa_Para_t;

/**********************************************/
#define SET_SLEEP 			0x84
#define SET_STANDBY			0x80
#define SET_TX				0x83
#define SET_RX				0x82
#define SET_PACKET_TYPE		0x8A
#define SET_RF_FREQUENCY 	0x86
#define SET_TX_PARAMS 		0x8E
#define SET_BUF_BASE_ADDR 	0x8F
#define GET_PACKET_STATUS   0x14

#define SET_RAMP_10U  	0x00
#define SET_RAMP_20U  	0x01
#define SET_RAMP_40U 	0x02
#define SET_RAMP_80U 	0x03
#define SET_RAMP_200U 	0x04
#define SET_RAMP_800U 	0x05
#define SET_RAMP_1700U 	0x06
#define SET_RAMP_3400U 	0x07

#define  TxDone_IRQ  	0x01
#define  RxDone_IRQ  	0x02
#define  Pream_IRQ 		0x04
#define  SyncWord_IRQ 	0x08
#define  Header_IRQ  	0x10
#define  HeaderErr_IRQ 	0x20
#define  CrcErr_IRQ		0x40
#define  CadDone_IRQ	0x80
#define  CadDetec_IRQ	0x0100
#define  Timeout_IRQ	0x0200

#define  LORA_SF5   0x05
#define  LORA_SF6   0x06 
#define  LORA_SF7   0x07
#define  LORA_SF8   0x08
#define  LORA_SF9   0x09
#define  LORA_SF10  0x0A 
#define  LORA_SF11  0x0B 
#define  LORA_SF12  0x0C 

#define  LORA_BW_7	  0x00
#define  LORA_BW_10	  0x08
#define  LORA_BW_15	  0x01 
#define  LORA_BW_20	  0x09 
#define  LORA_BW_31	  0x02
#define  LORA_BW_41	  0x0A 
#define  LORA_BW_62	  0x03 
#define  LORA_BW_125  0x04 
#define  LORA_BW_250  0x05 
#define  LORA_BW_500  0x06

#define  LORA_CR_4_5  0x01
#define  LORA_CR_4_6  0x02
#define  LORA_CR_4_7  0x03
#define  LORA_CR_4_8  0x04

#define  LDRO_ON	  0x01
#define  LDRO_OFF     0x00

#define  DIO3_1_6V  0x00
#define  DIO3_1_7V  0x01
#define  DIO3_1_8V  0x02
#define  DIO3_2_2V  0x03
#define  DIO3_2_4V  0x04
#define  DIO3_2_7V  0x05
#define  DIO3_3_0V  0x06
#define  DIO3_3_3V  0x07

//new changing
#define    RADIO_GET_IRQSTATUS   0x12

class SX1262
{
	
public:
	// Constructor.
	SX1262(uint8_t NSS_Pin = 32, uint8_t NRESET_Pin = 27,uint8_t BUSY_Pin = 34,uint8_t DIO1_Pin = 35);
	bool Init(loRa_Para_t *lp_pt);
	void TxPacket(uint8_t *payload,uint8_t size);
	uint8_t WaitForIRQ_TxDone(void);
	void RxBufferInit(uint8_t *rxpayload,uint16_t *rx_size);
	void RxInit();
	uint8_t WaitForIRQ_RxDone(void);
	void Reset_SX1262(void);
	void SetSleep(void);
	void SetStandby(uint8_t StdbyConfig);
	void SetRfFrequency( uint32_t frequency );
	void GetRSSI(void);
	
	// these are global variables
	uint8_t *rxbuf_pt;
	uint16_t *rxcnt_pt;
	uint8_t RSSIPKT=0;
	uint8_t SNRPKT=0;
	uint32_t freq=0;
	
protected:
	void SPI_Init(void);	// Initialise SPI. @note Use standard Arduino SPI interface
	void Pin_Init(void);	//Initialise other pin.
	uint8_t spi_rw(uint8_t value_w);
	void CheckBusy(void);
	void SetTx(uint32_t timeout);
	void SetRx(uint32_t timeout);
	void SetPacketType(uint8_t PacketType);
	uint8_t GetPacketType(void);
	//void SetRfFrequency( uint32_t frequency );
	void SetPaConfig(void);
	void SetRegulatorMode(void);
	void SetTxParams(int8_t power,uint8_t RampTime);
	void SetBufferBaseAddress(uint8_t TX_base_addr,uint8_t RX_base_addr);
	void WriteRegister(uint16_t address, uint8_t *data, uint8_t length);
	void ReadRegister(uint16_t address, uint8_t *data, uint8_t length);
	void WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length);
	void ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length);
	void GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer);
	void SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro);
	void SetPacketParams(uint8_t payload_len);
	void SetDioIrqParams(uint16_t irq);
	uint16_t GetIrqStatus(void);
	void ClearIrqStatus(uint16_t irq);
	void SetDIO2AsRfSwitchCtrl(void);
	void SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage);
	void SX1262_Config(void);
	bool InitLoRaAs(uint32_t rf_freq_temp, int8_t power_temp, uint8_t sf_temp, uint8_t bw_temp, uint8_t cr_temp, uint8_t size_temp);
	
	
private:
	uint8_t SPI_NSS;	//output,SPI slave select pin
	uint8_t RF_NRESET;	//output,hard reset of rf chip
	uint8_t RF_BUSY;	//input
	uint8_t RF_DIO1;	//input
};
#endif
