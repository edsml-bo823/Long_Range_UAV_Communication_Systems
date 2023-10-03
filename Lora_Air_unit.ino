#include <WiFi.h>
//#include <SPI.h>
#include <EEPROM.h>
#include "ESP32_LORA1262_TX.h"

SX1262 LoRa1262;  //define a object of class SX1262.
loRa_Para_t lora_para;  //define a struct to hold lora parameters.
#define LRA_IRQ   35 //*nIRQ Input for LoRa1262 (15.DIO1)
#define LRA_NSEL  32  //*nSELECT Output for LoRa1262 SPI (5.NSS)
#define BUSY_PIN  34 //*BUSY_PIN Input for LoRa1262 (16.BUSY)
#define PA_ENB    26 //TX Mode Output Pin
#define LNA_ENB   13 //RX Mode Output Pin
#define LRA_RST   27 //*nRST Output for LoRa1262 (6.nRST)
#define SPI_SDI   19 //*SDI Output for LoRa1262 SPI (3.MOSI)
#define SPI_SDO   23 //*SDO Input for LoRa1262 SPI (2.MISO)
#define SPI_SCLK  18 //*SCLOCK Output for LoRa1262 SPI (4.SCK)
#define LED_RED   22 //14 //A0 Tx Mode LED/RX_SW_ENB
#define LED_GRN   14 //A1 RX Mode LED/TX_SW_ENB
#define BAT_IN    A3 //A3 IO_17 Battery Volts Reading pin

unsigned char power = 10, lorasf = 5, lora_bw = 4, lora_cr = 1;
double freq1, freq;
unsigned int packetSize = 0, DataLength = 0;
bool StartReadingSerial = false;
uint16_t rx_size = 0;
float Bat_Voltage = 0;
byte rx_buf[250] = {0}, packetBuffer[250] = {0}, DataBuffer[250] = {0}; byte packetLength = 250;

void setup(void)
{
  EEPROM.begin(150);//EEPROM_SIZE);
  Serial.begin(115200);
  pinMode(PA_ENB, OUTPUT); digitalWrite(PA_ENB, LOW);
  pinMode(LNA_ENB, OUTPUT); digitalWrite(LNA_ENB, LOW);
  pinMode(LRA_IRQ, INPUT_PULLUP);
  pinMode(BUSY_PIN, INPUT_PULLUP);
  pinMode(SPI_SDO, INPUT_PULLUP);
  pinMode(LRA_RST, OUTPUT); digitalWrite(LRA_RST, LOW);
  pinMode(LRA_NSEL, OUTPUT); digitalWrite(LRA_NSEL, HIGH);
  pinMode(SPI_SDI, OUTPUT); digitalWrite(SPI_SDI, HIGH);
  pinMode(SPI_SCLK, OUTPUT); digitalWrite(SPI_SCLK, HIGH);
  pinMode(LED_RED, OUTPUT); digitalWrite(LED_RED, LOW);
  pinMode(LED_GRN, OUTPUT); digitalWrite(LED_GRN, LOW);
  //  WriteDefaultSettings();
  //  LoadEEPROMSettings();
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_RED, HIGH); delay(150);
    digitalWrite(LED_RED, LOW); delay(150);
  }
  digitalWrite(LRA_RST, HIGH);
  ParaInit();
}

//____________Main Loop__________________________________________________________________
/*this function continuously checks for incoming data from the serial port. If new data is received, 
it prepares the data for transmission, calls the transmission function, and also handles any received data interrupt from the LoRa module.*/
void loop (void)
{
  while (Serial.available() > 0) // checks if there is any data available to be read from the serial communication 
  {
    StartReadingSerial = true; // reads the incoming data byte by byte from the serial communication 
    DataBuffer[DataLength++] = Serial.read();
  }
  if (StartReadingSerial)
  {
    StartReadingSerial = false;
    memcpy(packetBuffer, DataBuffer, DataLength); //It copies the data from the DataBuffer array to the packetBuffer array using the memcpy() function.
    packetSize = DataLength;
    DataLength = 0;
    ProcessTxData();
  }
  RxInterruptL();
}

//===================================================================================================
/*Here i prepare the LoRa module for transmitting data, waiting for the transmission to finish,
 then switches back to receive mode, and initializes the LoRa module for receiving data again.*/
void ProcessTxData(void)
{
  SetTxMode();
  LoRa1262.TxPacket(packetBuffer, packetSize); // transmits the data contained in the packetBuffer array 
  /* this loop waits for the transmission to complete by repeatedly checking the status of the
   transmission using LoRa1262.WaitForIRQ_TxDone(). The loop continues until the transmission is done.*/
  while (!LoRa1262.WaitForIRQ_TxDone()) {
    ;
  }
  SetRxMode();
  InitLoRaRx();
}

//===================================================================================================
/*this function prepares the LoRa module for receiving data by disabling the transmitter, then indicating 
the receive mode with a red light, and enabling the receiver.*/
void SetRxMode(void)
{
  digitalWrite(PA_ENB, LOW); //sets TX mode control to a low logic level, disabling the transmitter.
  digitalWrite(LED_RED, LOW);
  digitalWrite(LNA_ENB, HIGH); //sets RX mode control to a high logic level, enabling the transmitter.
}



//==================================================================
/*this function prepares the LoRa module for transmitting data by disabling the receiver, indicating 
the transmission mode with a red LED, and enabling the transmitter after a brief delay which allows 
the module settle in TX mode before sending data.*/
void SetTxMode(void)
{
  digitalWrite(LNA_ENB, LOW); // sets RX mode control to a low logic level, disabling the receiver.
  digitalWrite(LED_RED, HIGH);
  digitalWrite(PA_ENB, HIGH); // sets the tx mode to a high logic level, enabling the transmitter
  delay(55);
}
//==================================================================
void InitLoRaRx(void)
{
  LoRa1262.RxBufferInit(rx_buf, &rx_size);
  LoRa1262.RxInit();
}
//==================================================================
/*this function checks if the module has finished receiving a packet by waiting for RxDoneinterrupt. 
If a packet has been received, it writes the received data to the serial port using Serial.write(), 
then prepares the module for the next reception by calling InitLoRaRx()*/
void RxInterruptL (void){
  if (LoRa1262.WaitForIRQ_RxDone())
  {
    Serial.write(rx_buf, rx_size);
    Serial.println("Received");
    InitLoRaRx();
  }
}


//==================================================================
/* initializes the LoRa radio module with specific parameters. If the initialization is successful, 
it sends the current settings via the serial port. After that, the InitLoRaRx() function is then 
called to initialise the module for receiving data and SetRxMode()  sets the loRa into receive mode.*/
void ParaInit (void)
{
  lora_para.rf_freq    = 451500000;//451500000;//451500000;//Main RF Frequency
  lora_para.tx_power   = power;//22;  //-9~22 Output Power Level
  lora_para.lora_sf    = lorasf;//LORA_SF12;//Spreading Factor 7
  lora_para.band_width = lora_bw;//LORA_BW_125;//Bandwith
  lora_para.code_rate  = lora_cr;//LORA_CR_4_8;//Code Rate
  lora_para.payload_size = packetLength; //Length of Packet

  if (!LoRa1262.Init(&lora_para)){
    Serial.println("Init fail");
  }
  else{
    Serial.println("Init done");
    SendSettings1();
  }
  SetRxMode();
  InitLoRaRx();
}


//==================================================================
/* Ill be sedning a string of configuration values via the uart to the ground LoRa
 i do the same (vice versa) for the ground lora.
 each value is appendde to do ALL string, and are separated by the "*" to keep a structured format */
void SendSettings1 (void)
{
  String ALL = "*#A*B*";
  ALL += Bat_Voltage;
  ALL += "*";
  ALL += power;
  ALL += "*";
  ALL += lorasf;
  ALL += "*";
  ALL += lora_bw;
  ALL += "*";
  ALL += lora_cr;
  ALL += "*";
  ALL += freq;
  ALL += "*";
  Serial.println(ALL);
  // this line (line above) sends the string to the serial port, making it available for reading by the ground lora
  // an dvice versa for the ground lora.  
}
//==========================================================================================================
// if the value stored in memory location 100 of EEPROM is not equal to 37. If this condition returns true, 
// it means that default values have not been previously written into the corresponding EEPROM locations.
void WriteDefaultSettings (void)
{
  if (EEPROM.read(100) != 37)
  {
    EEPROM.write(100, 37);
    EEPROM.write(49, power);
    EEPROM.write(50, lorasf);
    EEPROM.write(51, lora_bw);
    EEPROM.write(52, lora_cr);
    EEPROM.commit();
    Serial.println("Default Values Written");
  }
}
void LoadEEPROMSettings (void)
{
  power   = EEPROM.read(49);
  lorasf  = EEPROM.read(50);
  lora_bw = EEPROM.read(51);
  lora_cr = EEPROM.read(52);
  Serial.println("EEPROM Values Loaded.");
}
