/********************************************************************************************
	SX1262.cpp - Library for SX1262
	Copyright(C) 2018 NiceRF.All right reserved.
	@version 1.0
	This library is suited for LoRa1262 in loRa mode
	please make sure the supply of your board is UNDER 3.3V!! Or the module will be destory!!
	The configuration of both module should be same.
*********************************************************************************************/
#include <SPI.h>
//#include <SoftwareSerial.h>
#include "ESP32_LORA1262_TX.h"
#define USE_TCXO
#define SPI_NSS_LOW()  digitalWrite(SPI_NSS, LOW)
#define SPI_NSS_HIGH() digitalWrite(SPI_NSS, HIGH)
/*The TCXO is 32MHz, so the frequency step should be FREQ_STEP = 32e6 / (2^25) Hz*/
#define FREQ_STEP    0.953674
static loRa_Para_t *lora_para_pt;	//pointer to hold lora parameter from app layer
SX1262::SX1262(uint8_t NSS_Pin, uint8_t NRESET_Pin, uint8_t BUSY_Pin, uint8_t DIO1_Pin)
{
  SPI_NSS = NSS_Pin;
  RF_NRESET = NRESET_Pin;
  RF_BUSY = BUSY_Pin;
  RF_DIO1 = DIO1_Pin;
  lora_para_pt->rf_freq;
  lora_para_pt->tx_power;
  lora_para_pt->lora_sf;
  lora_para_pt->band_width;
  lora_para_pt->code_rate;
  lora_para_pt->payload_size;
}
void SX1262::SPI_Init(void)
{
  SPI.begin();
  //  SPI.begin(14, 12, 13, 15);
  // init slave select pin
  pinMode(SPI_NSS, OUTPUT);
  digitalWrite(SPI_NSS, HIGH);
  // depends on LORA spi timing
  SPI.setBitOrder(MSBFIRST);
  // too fast may cause error
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.setDataMode(SPI_MODE0);
}
void SX1262::Pin_Init(void)
{
  pinMode(RF_NRESET, OUTPUT);
  digitalWrite(RF_NRESET, LOW);
  pinMode(RF_BUSY, INPUT_PULLUP);
  digitalWrite(RF_BUSY, LOW);
  pinMode(RF_DIO1, INPUT_PULLUP);
}
bool SX1262::Init(loRa_Para_t *lp_pt)
{
  lora_para_pt = lp_pt;
  Pin_Init();
  SPI_Init();
  Reset_SX1262();	// reset loRa chip
  SX1262_Config();// Set RF parameter,like frequency,data rate etc
  return true;
}
void SX1262::Reset_SX1262(void)
{
  digitalWrite(RF_NRESET, LOW);
  delay(1);		//more thena 100us, delay 2ms
  digitalWrite(RF_NRESET, HIGH);
  delay(1);		//delay 20ms
}
uint8_t SX1262::spi_rw(uint8_t value_w)
{
  uint8_t value_r;
  value_r = SPI.transfer(value_w);
  return (value_r);
}
/***************sx1262*****************/
void SX1262::CheckBusy(void)
{
  uint8_t busy_timeout_cnt;
  busy_timeout_cnt = 0;
  while (digitalRead(RF_BUSY))
  {
    delay(1);
    busy_timeout_cnt++;
    if (busy_timeout_cnt > 2)	//TODO
    {
      SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
      Reset_SX1262();		//reset RF
      SX1262_Config();
      break;
    }
  }
}
void SX1262::SetSleep(void)
{
  uint8_t Opcode, sleepConfig;
  CheckBusy();
  Opcode = SET_SLEEP;	//0x84
  sleepConfig = 0x00;//0x04;	//bit2: 1:warm start; bit0:0: RTC timeout disable
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(sleepConfig);
  SPI_NSS_HIGH();
}
//0:STDBY_RC; 1:STDBY_XOSC
void SX1262::SetStandby(uint8_t StdbyConfig)
{
  uint8_t Opcode;
  CheckBusy();
  Opcode = SET_STANDBY;	//0x80
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(StdbyConfig);
  SPI_NSS_HIGH();
}
void SX1262::SetTx(uint32_t timeout)
{
  uint8_t Opcode;
  uint8_t time_out[3];
  CheckBusy();
  Opcode = SET_TX;	//0x83
  time_out[0] = (timeout >> 16) & 0xFF; //MSB
  time_out[1] = (timeout >> 8) & 0xFF;
  time_out[2] = timeout & 0xFF; //LSB
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(time_out[0]);
  spi_rw(time_out[1]);
  spi_rw(time_out[2]);
  SPI_NSS_HIGH();
}
void SX1262::SetRx(uint32_t timeout)
{
  uint8_t Opcode;
  uint8_t time_out[3];
  CheckBusy();
  Opcode = SET_RX;	//0x82
  time_out[0] = (timeout >> 16) & 0xFF; //MSB
  time_out[1] = (timeout >> 8) & 0xFF;
  time_out[2] = timeout & 0xFF; //LSB
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(time_out[0]);
  spi_rw(time_out[1]);
  spi_rw(time_out[2]);
  SPI_NSS_HIGH();
}
//0:GFSK; 1:LORA
void SX1262::SetPacketType(uint8_t PacketType)
{
  uint8_t Opcode;
  CheckBusy();
  Opcode = SET_PACKET_TYPE;	//0x8A
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(PacketType);
  SPI_NSS_HIGH();
}
uint8_t SX1262::GetPacketType(void)
{
  uint8_t Opcode;
  uint8_t Status;
  uint8_t packetType;
  CheckBusy();
  Opcode = 0x11;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  Status = spi_rw(0xFF);
  packetType = spi_rw(0xFF);
  SPI_NSS_HIGH();
  return packetType;
}
void SX1262::GetRSSI(void)
{
  uint8_t Opcode;
  uint8_t temp;
  uint8_t temp1;
  uint8_t SIGNALRSSIPKT;
  uint8_t Status;
  //uint8_t RSSIPKT; //No need here because it was declared globaly.
  CheckBusy();
  Opcode = GET_PACKET_STATUS;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  Status = spi_rw(0xFF);
  temp = spi_rw(0xFF);
  RSSIPKT = temp / 2;
  temp1 = spi_rw(0xFF);
  SNRPKT = temp1 / 4;
  SIGNALRSSIPKT = spi_rw(0xFF);
  SPI_NSS_HIGH();
  //return RSSIPKT; //Because we declared variables globaly so no need of return.
}
//RF_Freq = freq_reg*32M/(2^25)-----> freq_reg = (RF_Freq * (2^25))/32
void SX1262::SetRfFrequency( uint32_t frequency )
{
  freq = frequency;
  uint8_t Opcode;
  uint8_t Rf_Freq[4];
  uint32_t RfFreq = 0;
  RfFreq = ( uint32_t )( ( double )frequency / ( double )FREQ_STEP );
  CheckBusy();
  Opcode = SET_RF_FREQUENCY;	//0x86
  Rf_Freq[0] = (RfFreq >> 24) & 0xFF; //MSB
  Rf_Freq[1] = (RfFreq >> 16) & 0xFF;
  Rf_Freq[2] = (RfFreq >> 8) & 0xFF;
  Rf_Freq[3] = RfFreq & 0xFF; //LSB
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(Rf_Freq[0]);
  spi_rw(Rf_Freq[1]);
  spi_rw(Rf_Freq[2]);
  spi_rw(Rf_Freq[3]);
  SPI_NSS_HIGH();
}
void SX1262::SetPaConfig(void)
{
  uint8_t Opcode;
  CheckBusy();
  Opcode = 0x95;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(0x04);	//paDutyCycle
  spi_rw(0x07);	//hpMax:0x00~0x07; 7:22dbm
  spi_rw(0x00);	//deviceSel: 0: SX1262; 1: SX1261
  spi_rw(0x01);
  SPI_NSS_HIGH();
}
void SX1262::SetRegulatorMode(void)
{
  uint8_t Opcode;
  CheckBusy();
  Opcode = 0x96;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(0x01);//regModeParam
  SPI_NSS_HIGH();
}
/*@para
  power:
  -17(0xEF) to +14(0x0E) dBm by step of 1 dB if low power PA is selected
  -9(0xF7) to +22(0x16) dBm by step of 1 dB if high power PA is selected
  RampTime:
  -------------------------------------
  RampTime 	  | Value | RampTime(��s)
  -------------------------------------
  SET_RAMP_10U    0x00    10
  SET_RAMP_20U    0x01    20
  SET_RAMP_40U 	0x02	40
  SET_RAMP_80U 	0x03	80
  SET_RAMP_200U 	0x04	200
  SET_RAMP_800U 	0x05	800
  SET_RAMP_1700U 	0x06	1700
  SET_RAMP_3400U 	0x07	3400*/
void SX1262::SetTxParams(int8_t power, uint8_t RampTime)
{
  uint8_t Opcode;
  CheckBusy();
  Opcode = SET_TX_PARAMS;	//0x8E
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(power);
  spi_rw(RampTime);
  SPI_NSS_HIGH();
}
void SX1262::SetBufferBaseAddress(uint8_t TX_base_addr, uint8_t RX_base_addr)
{
  uint8_t Opcode;
  //CheckBusy();
  Opcode = SET_BUF_BASE_ADDR;	//0x8F
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(TX_base_addr);
  spi_rw(RX_base_addr);
  SPI_NSS_HIGH();
}
void SX1262::WriteRegister(uint16_t address, uint8_t *data, uint8_t length)
{
  uint8_t Opcode;
  uint8_t addr_l, addr_h;
  uint8_t i;
  if (length < 1)
    return;
  CheckBusy();
  addr_l = address & 0xff;
  addr_h = address >> 8;
  Opcode = 0x0D;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(addr_h);//MSB
  spi_rw(addr_l);//LSB
  for (i = 0; i < length; i++)
  {
    spi_rw(data[i]);
  }
  SPI_NSS_HIGH();
}
void SX1262::ReadRegister(uint16_t address, uint8_t *data, uint8_t length)
{
  uint8_t Opcode;
  uint8_t addr_l, addr_h;
  uint8_t i;
  if (length < 1)
    return;
  CheckBusy();
  addr_l = address & 0xff;
  addr_h = address >> 8;
  Opcode = 0x1D;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(addr_h);//MSB
  spi_rw(addr_l);//LSB
  for (i = 0; i < length; i++)
  {
    data[i] = spi_rw(0xFF);
  }
  SPI_NSS_HIGH();
}
void SX1262::WriteBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
  uint8_t Opcode;
  uint8_t i;
  if (length < 1)
    return;
  CheckBusy();
  Opcode = 0x0E;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(offset);
  for (i = 0; i < length; i++)
  {
    spi_rw(data[i]);
  }
  SPI_NSS_HIGH();
}
void SX1262::ReadBuffer(uint8_t offset, uint8_t *data, uint8_t length)
{
  uint8_t Opcode;
  uint8_t i;
  if (length < 1)
    return;
  CheckBusy();
  Opcode = 0x1E;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(offset);
  spi_rw(0xFF);
  for (i = 0; i < length; i++)
  {
    data[i] = spi_rw(0xFF);
  }
  SPI_NSS_HIGH();
}
void SX1262::GetRxBufferStatus(uint8_t *payload_len, uint8_t *buf_pointer)
{
  uint8_t Opcode;
  uint8_t Status;
  CheckBusy();
  Opcode = 0x13;
  SPI_NSS_LOW();
  spi_rw(Opcode);
  Status = spi_rw(0xFF);
  *payload_len = spi_rw(0xFF);
  *buf_pointer = spi_rw(0xFF);
  SPI_NSS_HIGH();
}
void SX1262::SetModulationParams(uint8_t sf, uint8_t bw, uint8_t cr, uint8_t ldro)
{
  uint8_t Opcode;

  CheckBusy();
  Opcode = 0x8B;

  SPI_NSS_LOW();
  spi_rw(Opcode);

  spi_rw(sf);//SF=5~12
  spi_rw(bw);//BW
  spi_rw(cr);//CR
  spi_rw(ldro);//LDRO LowDataRateOptimize 0:OFF; 1:ON;

  spi_rw(0XFF);//
  spi_rw(0XFF);//
  spi_rw(0XFF);//
  spi_rw(0XFF);//

  SPI_NSS_HIGH();
}
void SX1262::SetPacketParams(uint8_t payload_len)
{
  uint8_t Opcode;
  uint16_t prea_len;
  uint8_t prea_len_h, prea_len_l;

  CheckBusy();

  Opcode = 0x8C;

  prea_len = 8;
  prea_len_h = prea_len >> 8;
  prea_len_l = prea_len & 0xFF;

  SPI_NSS_LOW();
  spi_rw(Opcode);

  spi_rw(prea_len_h);//PreambleLength MSB
  spi_rw(prea_len_l);//PreambleLength LSB
  spi_rw(0x00);//HeaderType 0:Variable,explicit 1:Fixed,implicit
  //spi_rw(0x01);
  spi_rw(payload_len);//PayloadLength: 0x00 to 0xFF

  spi_rw(0X01);//CRCType 0:OFF 1:ON
  spi_rw(0X00);//InvertIQ 0:Standard 1:Inverted
  spi_rw(0XFF);//
  spi_rw(0XFF);//
  spi_rw(0XFF);//

  SPI_NSS_HIGH();
}
void SX1262::SetDioIrqParams(uint16_t irq)
{
  uint8_t Opcode;
  uint16_t Irq_Mask;
  uint8_t Irq_Mask_h, Irq_Mask_l;
  uint16_t DIO1Mask;
  uint8_t DIO1Mask_h, DIO1Mask_l;
  uint16_t DIO2Mask;
  uint8_t DIO2Mask_h, DIO2Mask_l;
  uint16_t DIO3Mask;
  uint8_t DIO3Mask_h, DIO3Mask_l;

  Irq_Mask = irq;
  DIO1Mask = irq;
  DIO2Mask = 0;
  DIO3Mask = 0;

  Irq_Mask_h = Irq_Mask >> 8;
  Irq_Mask_l = Irq_Mask & 0xFF;
  DIO1Mask_h = DIO1Mask >> 8;
  DIO1Mask_l = DIO1Mask & 0xFF;
  DIO2Mask_h = DIO2Mask >> 8;
  DIO2Mask_l = DIO2Mask & 0xFF;
  DIO3Mask_h = DIO3Mask >> 8;
  DIO3Mask_l = DIO3Mask & 0xFF;

  Opcode = 0x08;

  CheckBusy();
  SPI_NSS_LOW();
  spi_rw(Opcode);

  spi_rw(Irq_Mask_h);//Irq_Mask MSB
  spi_rw(Irq_Mask_l);//Irq_Mask LSB
  spi_rw(DIO1Mask_h);//
  spi_rw(DIO1Mask_l);//

  spi_rw(DIO2Mask_h);//
  spi_rw(DIO2Mask_l);//
  spi_rw(DIO3Mask_h);//
  spi_rw(DIO3Mask_l);//

  SPI_NSS_HIGH();
}
uint16_t SX1262::GetIrqStatus(void)
{
  uint8_t Opcode;
  uint8_t Status;
  uint16_t IrqStatus;
  uint8_t temp;

  CheckBusy();

  Opcode = 0x12;

  SPI_NSS_LOW();
  spi_rw(Opcode);
  Status = spi_rw(0xFF);
  temp = spi_rw(0xFF);
  IrqStatus = temp;
  IrqStatus = IrqStatus << 8;
  temp = spi_rw(0xFF);
  IrqStatus = IrqStatus | temp;
  SPI_NSS_HIGH();

  return IrqStatus;
}
void SX1262::ClearIrqStatus(uint16_t irq)
{
  uint8_t Opcode;
  uint16_t irq_h, irq_l;

  CheckBusy();

  irq_h = irq >> 8;
  irq_l = irq & 0xFF;

  Opcode = 0x02;

  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(irq_h);
  spi_rw(irq_l);
  SPI_NSS_HIGH();
}
void SX1262::SetDIO2AsRfSwitchCtrl(void)
{
  uint8_t Opcode;

  CheckBusy();
  Opcode = 0x9D;

  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(0x01);   //DIO2 is selected to be used to control an RF switch; DIO2 = 1 in TX mode
  SPI_NSS_HIGH();
}
void SX1262::SetDIO3AsTCXOCtrl(uint8_t tcxoVoltage)
{
  uint8_t Opcode;

  CheckBusy();
  Opcode = 0x97;

  SPI_NSS_LOW();
  spi_rw(Opcode);
  spi_rw(tcxoVoltage);   //
  spi_rw(0x00);		   //Timeout MSB ; Timeout duration = Timeout *15.625 ��s
  spi_rw(0x00);
  spi_rw(0x64);          //Timeout LSB

  SPI_NSS_HIGH();
}
void SX1262::TxPacket(uint8_t *payload, uint8_t size)
{
  SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
  SetBufferBaseAddress(0, 0); //(TX_base_addr,RX_base_addr)

  WriteBuffer(0, payload, size); //(offset,*data,length)
  SetPacketParams(size);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ

  SetDioIrqParams(TxDone_IRQ);//TxDone IRQ

  SetTx(0);//timeout = 0

  //Wait for the IRQ TxDone or Timeout (implement in another function)
}
uint8_t SX1262::WaitForIRQ_TxDone(void)
{
  uint16_t time_out;

  time_out = 0;
  while (!digitalRead(RF_DIO1))
  {
    time_out++;
    delay(10);
    if (time_out > 200)			//if timeout , reset the the chip
    {
      ClearIrqStatus(TxDone_IRQ);//Clear the IRQ TxDone flag
      SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
      Reset_SX1262();		//reset RF
      SX1262_Config();
      Serial.println("WaitFor IRQ_TxDone time out\n");
      return 0;
    }
  }
  //Irq_Status = GetIrqStatus();
  ClearIrqStatus(TxDone_IRQ);//Clear the IRQ TxDone flag
  //Serial.println(time_out);
  return 1;
}
void SX1262::RxBufferInit(uint8_t *rxpayload, uint16_t *rx_size)
{
  rxbuf_pt = rxpayload;
  rxcnt_pt = rx_size;
}
void SX1262::RxInit(void)
{
  SetBufferBaseAddress(0, 0); //(TX_base_addr,RX_base_addr)
  //SetPacketParams(payload_length);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
  SetDioIrqParams(RxDone_IRQ);//RxDone IRQ
  SetRx(0);//timeout = 0
}
uint8_t SX1262::WaitForIRQ_RxDone(void)
{
  uint16_t Irq_Status;
  uint8_t packet_size;
  uint8_t buf_offset;

  if (digitalRead(RF_DIO1)) //if IRQ check
  {
    Irq_Status = GetIrqStatus();//read Irq Status
    if ((Irq_Status & 0x02) == RxDone_IRQ)
    {
      GetRxBufferStatus(&packet_size, &buf_offset);
      ReadBuffer(buf_offset, rxbuf_pt, packet_size + 1);
      *rxcnt_pt = packet_size;
      ClearIrqStatus(RxDone_IRQ);//Clear the IRQ RxDone flag
      RxInit();
      return 1;
    }
  }
  return 0;
}
void SX1262::SX1262_Config(void)
{
  uint32_t rf_freq_temp;
  int8_t power_temp;
  uint8_t sf_temp;
  uint8_t bw_temp;
  uint8_t cr_temp;
  uint8_t size_temp;

  rf_freq_temp = lora_para_pt->rf_freq;
  power_temp = lora_para_pt->tx_power;
  sf_temp = lora_para_pt->lora_sf;
  bw_temp = lora_para_pt->band_width;
  cr_temp = lora_para_pt->code_rate;
  size_temp = lora_para_pt->payload_size;

  SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
  SetRegulatorMode();
  SetPaConfig();
#ifdef USE_TCXO
  SetDIO3AsTCXOCtrl(DIO3_3_3V);
#endif
  SetDIO2AsRfSwitchCtrl();
  SetPacketType(1);	//0:GFSK; 1:LORA
  SetRfFrequency(rf_freq_temp);	//RF_Freq = freq_reg*32M/(2^25)
  SetTxParams(power_temp, SET_RAMP_10U);	//set power and ramp_time
  SetModulationParams(sf_temp, bw_temp, cr_temp, LDRO_ON);
  SetPacketParams(size_temp);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
}
bool SX1262::InitLoRaAs(uint32_t rf_freq_temp, int8_t power_temp, uint8_t sf_temp, uint8_t bw_temp, uint8_t cr_temp, uint8_t size_temp)
{
  Pin_Init();
  SPI_Init();
  Reset_SX1262();
  /*uint32_t rf_freq_temp;
    int8_t power_temp;
    uint8_t sf_temp;
    uint8_t bw_temp;
    uint8_t cr_temp;
    uint8_t size_temp;

    rf_freq_temp = lora_para_pt->rf_freq;
    power_temp = lora_para_pt->tx_power;
    sf_temp = lora_para_pt->lora_sf;
    bw_temp = lora_para_pt->band_width;
    cr_temp = lora_para_pt->code_rate;
    size_temp = lora_para_pt->payload_size;*/

  SetStandby(0);//0:STDBY_RC; 1:STDBY_XOSC
  SetRegulatorMode();
  SetPaConfig();
#ifdef USE_TCXO
  SetDIO3AsTCXOCtrl(DIO3_3_3V);
#endif
  SetDIO2AsRfSwitchCtrl();
  SetPacketType(1);	//0:GFSK; 1:LORA
  SetRfFrequency(rf_freq_temp);	//RF_Freq = freq_reg*32M/(2^25)
  SetTxParams(power_temp, SET_RAMP_10U);	//set power and ramp_time
  SetModulationParams(sf_temp, bw_temp, cr_temp, LDRO_ON);
  SetPacketParams(size_temp);//PreambleLength;HeaderType;PayloadLength;CRCType;InvertIQ
  return true;
}
