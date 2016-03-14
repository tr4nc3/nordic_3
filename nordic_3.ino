//
// TODO
// Set NRF config register
// Disable shockburst according to Travis
//
// See goodfet.nrf AutoTuner class

// the sensor communicates using SPI, so include the library:
#include <SPI.h>

//---------------------------------------------
// Hardware configuration
const int CEPIN = 8;
const int CSPIN = 9;

//MISO PIN gives status Pg 49, Section 8.3.2 nRF24L01+ datasheet
const int MISOPIN = 11; // green on Rajat's board

// MOSI PIN needs to get the command to read MOSI and status
const int MOSIPIN = 10; // yellow on Rajat's board
// CSNPIN is the SPI CS
// RED on Rajat's Board
const int CSNPIN = 10;

const int DEBUG_LED = 13;
//---------------------------------------------

uint8_t channel = 1; 
//---------------------------------------------
//    Radio SPI registers and constants 
//---------------------------------------------

// Configuration
enum CONFIG
{
  PRIM_RX     = 0x01,
  PWR_UP      = 0x02,
  CRC0        = 0x04,
  EN_CRC      = 0x08,
  MASK_MAX_RT = 0x10,
  MASK_TX_DS  = 0x20,
  MASK_RX_DR  = 0x40,
};

// Auto Acknowledgement
enum EN_AA
{
  ENAA_NONE = 0x00,
  ENAA_P0   = 0x01,
  ENAA_P1   = 0x02,
  ENAA_P2   = 0x04,
  ENAA_P3   = 0x08,
  ENAA_P4   = 0x10,
  ENAA_P5   = 0x20,
};

// Enabled RX Addresses
enum EN_RXADDR
{
  ENRX_P0 = 0x01,
  ENRX_P1 = 0x02,
  ENRX_P2 = 0x04,
  ENRX_P3 = 0x08,
  ENRX_P4 = 0x10,
  ENRX_P5 = 0x20,
};

// Address Widths
enum SETUP_AW
{
  AW_2 = 0x00,
  AW_3 = 0x01,
  AW_4 = 0x02,
  AW_5 = 0x03,
};

// RF Setup
enum RF_SETUP
{
  CONT_WAVE = 0x80,
  PLL_LOCK  = 0x10,
  RATE_2M   = 0x08,
  RATE_1M   = 0x00,
  RATE_250K = 0x20,
  RF_PWR_4  = 0x06,
  RF_PWR_3  = 0x04,
  RF_PWR_2  = 0x02,
  RF_PWR_1  = 0x00,
};

// Dynamic payloads
enum DYNPD
{
  DPL_P5 = 0x20,
  DPL_P4 = 0x10,
  DPL_P3 = 0x08,
  DPL_P2 = 0x04,
  DPL_P1 = 0x02,
  DPL_P0 = 0x01,
};

// Features
enum FEATURE
{
  EN_DPL     = 0x04,
  EN_ACK_PAY = 0x02,
  EN_DYN_ACK = 0x01  
};

// Status flags
enum STATUS
{
  RX_DR   = 0x40,
  TX_DS   = 0x20,
  MAX_RT  = 0x10,
  TX_FULL = 0x01,
};

// nRF24 SPI commands
enum nrf24_command
{
  R_REGISTER       = 0x00,
  W_REGISTER       = 0x20,
  R_RX_PL_WID      = 0x60,
  R_RX_PAYLOAD     = 0x61,
  W_TX_PAYLOAD     = 0xA0,
  W_ACK_PAYLOAD_P0 = 0xA8,
  FLUSH_TX         = 0xE1,
  FLUSH_RX         = 0xE2,
  _NOP             = 0xFF,
};

// nRF24 registers
enum nrf24_register
{
  CONFIG      = 0x00,
  EN_AA       = 0x01,
  EN_RXADDR   = 0x02,
  SETUP_AW    = 0x03,
  SETUP_RETR  = 0x04,
  RF_CH       = 0x05,
  RF_SETUP    = 0x06,
  STATUS      = 0x07,
  OBSERVE_TX  = 0x08,
  RPD         = 0x09,
  RX_ADDR_P0  = 0x0A,
  RX_ADDR_P1  = 0x0B,
  RX_ADDR_P2  = 0x0C,
  RX_ADDR_P3  = 0x0D,
  RX_ADDR_P4  = 0x0E,
  RX_ADDR_P5  = 0x0F,
  TX_ADDR     = 0x10,
  RX_PW_P0    = 0x11,
  RX_PW_P1    = 0x12,
  RX_PW_P2    = 0x13,
  RX_PW_P3    = 0x14,
  RX_PW_P4    = 0x15,
  RX_PW_P5    = 0x16,
  FIFO_STATUS = 0x17,
  DYNPD       = 0x1C,
  FEATURE     = 0x1D,
};
//---------------------------------------------
// NRF Registers
const int NRF_AW = 0x03;
const int NRF_EN_RXADDR = 0x02;
const int NRF_RX_ADDR_P0 = 0x0A;
//---------------------------------------------


const int ADDRESS_BUFFER_SIZE = 5;

//-------------------------------------
uint8_t status_rd() {
  uint8_t readstatus;
  readstatus = nrf_read(0x07); //STATUS register also simultaenously to MISO
  return readstatus;
}

//-------------------------------------
// Read the value at register address
//-------------------------------------
int nrf_read(int address) {
  int cmd;

  // address is a 5 bit value
  address = address & 0x1F;
  
  cmd = 0x00 | address;
  
  digitalWrite(CSNPIN, LOW);
  SPI.transfer(cmd);
  int val = SPI.transfer(0);
  digitalWrite(CSNPIN, HIGH);

  return(val);
  
}
//-------------------------------------


//-------------------------------------
// Please NOTE data in NRF is little endian
// so confirm mult-read bytes are in correct order
// See paragraph on page 48 and timing diagrams on page 49
void nrf_multi_read(int address, int size, unsigned char *buffer) {

  int cmd;

  // address is a 5 bit value
  address = address & 0x1F;
  
  cmd = 0x00 | address;
  
  digitalWrite(CSNPIN, LOW);
 
  SPI.transfer(cmd);

  for (int i = 0; i < size; i++) {
    *buffer = SPI.transfer(0);
    buffer++;
  }
  
  digitalWrite(CSNPIN, HIGH);

}
//-------------------------------------


//-------------------------------------
// Please NOTE data in NRF is little endian
// so confirm mult-write bytes are in correct order
// See paragraph on page 48 and timing diagrams on page 49
void nrf_multi_write(int address, int size, unsigned char *buffer) {
  int cmd;

  // address is a 5 bit value
  address = address & 0x1F;

  // cmd format for writes is specified on page 48
  cmd = 0x20 | address;
  
  digitalWrite(CSNPIN, LOW);
 
  SPI.transfer(cmd);

  for (int i = 0; i < size; i++) {
    SPI.transfer(*buffer);
    buffer++;
  }
  
  digitalWrite(CSNPIN, HIGH);
}
//-------------------------------------

//-------------------------------------
// SPI Write value to register at address
//-------------------------------------
void nrf_write(int address, int val) {

  digitalWrite(CSNPIN, LOW);

  // The write command is of form
  // 001b bbbb
  // 5 lower b's are the address of the register to write to
  //
  int write_command = 0x20 | (address & 0x1F);
  SPI.transfer(write_command);
  SPI.transfer(val);

  digitalWrite(CSNPIN, HIGH);

}
//-------------------------------------

//-------------------------------------
//set RF Channel to channel n
//-------------------------------------
void setRFChannel(uint8_t n) {
  nrf_write(0x05,n);
  return;
}
//-------------------------------------


//-------------------------------------
int test_read(void) {
  return(nrf_read(0x0C));
}
//-------------------------------------


//-------------------------------------
void test_write(void) {
  nrf_write(0x0C, 0x09);
}
//-------------------------------------

unsigned short* convert_uint8_array_to_lsb_first(unsigned short* val, unsigned short length) 
{
  unsigned short lsbval[length]; 
  return val;
}
void configure_address(uint8_t* address, unsigned short length)
{
  nrf_write(EN_RXADDR, ENRX_P0);
  nrf_write(SETUP_AW, length - 2);
  nrf_multi_write(TX_ADDR, length, (unsigned char*) address);
  nrf_multi_write(RX_ADDR_P0, length, (unsigned char*)address);  
}

//-------------------------------------
// Configure MAC layer functionality on pipe 0
//-------------------------------------
void configure_mac(unsigned short feature, unsigned short dynpd, unsigned short en_aa)
{
  nrf_write(FEATURE, feature);
  nrf_write(DYNPD, dynpd);
  nrf_write(EN_AA, en_aa);  
}

//-------------------------------------
// Configure PHY layer on pipe 0
//-------------------------------------
void configure_phy(unsigned short config, unsigned short rf_setup, unsigned short rx_pw)
{
  nrf_write(CONFIG, config);
  nrf_write(RF_SETUP, rf_setup);
  nrf_write(RX_PW_P0, rx_pw);
}


//-------------------------------------
void setup() {
  int val;
  int i;
  unsigned char address_buffer[ADDRESS_BUFFER_SIZE] = { 0x01, 0x02, 0x03, 0x04 };

  delay(200);
  
  Serial.begin(115200);
  digitalWrite(CSNPIN, HIGH);

  // start the SPI library:
  SPI.begin();

  // initalize the  data ready and chip select pins:
  pinMode(CEPIN, OUTPUT);
  pinMode(CSPIN, OUTPUT);
  pinMode(CSNPIN, OUTPUT);
  //pinMode(MISOPIN, INPUT);
  //pinMode(MOSIPIN, OUTPUT);
  
  for (int i = 0; i < 5; i++) {
    digitalWrite(DEBUG_LED, HIGH);
    delay(500);
    digitalWrite(DEBUG_LED, LOW);
    delay(500);
  }

  val = test_read();
  Serial.print("Val from read is : "); Serial.println(val, HEX);
  Serial.println("Doing a test write...");
  test_write();

  val = test_read();
  Serial.print("Val from read (after write) is : "); Serial.println(val);


  Serial.println("Setting the RX Address Width 0x00 => (AW <- 2)");
  // See page 55; 0x00 is an illegal value which should result in 2 byte address widths
  // according to Travis
  nrf_write(NRF_AW, 0x00);

  Serial.println("Only enabling receive pipe 0");
  // See page 54 re allowed data pipes.
  nrf_write(NRF_EN_RXADDR, 0x01);

  Serial.println("Doing multi read of RX pipe 0 address - should have default values...");
  nrf_multi_read(NRF_RX_ADDR_P0, ADDRESS_BUFFER_SIZE , &address_buffer[0]);
  for (i = 0; i < ADDRESS_BUFFER_SIZE; i++) {
    Serial.println(address_buffer[i], HEX);
  }
  Serial.println("-----");

  address_buffer[0] = 0x00;
  address_buffer[1] = 0x11;
  address_buffer[2] = 0x22;
  address_buffer[3] = 0x33;
  address_buffer[4] = 0x66;
  Serial.println("Doing multi write of RX pipe 0 address ...");
  nrf_multi_write(NRF_RX_ADDR_P0, ADDRESS_BUFFER_SIZE , &address_buffer[0]);
  
  Serial.println("-----");

  Serial.println("Doing multi read of RX pipe 0 address - should have default values...");
  nrf_multi_read(NRF_RX_ADDR_P0, ADDRESS_BUFFER_SIZE , &address_buffer[0]);
  for (i = 0; i < ADDRESS_BUFFER_SIZE; i++) {
    Serial.println(address_buffer[i], HEX);
  }
  
  delay(2000);
  //while (1) ;
}
//-------------------------------------

void flush_rx() {
  digitalWrite(CSNPIN, LOW) ;
  SPI.transfer(FLUSH_RX);
  digitalWrite(CSNPIN, HIGH); 
}

void flush_tx() {
  digitalWrite(CSNPIN, LOW) ;
  SPI.transfer(FLUSH_TX);
  digitalWrite(CSNPIN, HIGH); 
}

void hexdump(uint8_t* arr, uint8_t len) {
  for (int i = 0; i < len; i++) {
    Serial.print("0x");
    Serial.print(arr[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}
//-------------------------------------
void loop() {
   uint8_t promiscuous_address[2] = { 0x55, 0x00 };
   unsigned long timeout = 100;
   channel = 1; 
   int readval ;
   uint8_t packet[33];
   uint8_t tryno = 0;
   
   while (channel < 85) 
   {
      unsigned long starttime = millis();
      nrf_write(0x07,0x78); //reset status register
      nrf_write(CONFIG, 0x73); //disable crc, power up,  mask interrups
      configure_address(promiscuous_address, 2);
      configure_mac(0,0, ENAA_NONE);
      configure_phy (PRIM_RX | PWR_UP, RATE_2M, 32);
      Serial.print("Tuning to channel: "); Serial.println(channel);
      nrf_write(RF_CH,channel++); //  Set the frequency to channel
      /*nrf_write(SETUP_AW, 0x00); 
      nrf_write(FEATURE, 0); // Disable dyn payload length,
      nrf_write(DYNPD, 0); // Disable dynamic payload
      nrf_write(EN_AA, 0); // Disable auto acknowledgement
      */
      while (millis() - starttime < timeout) 
      {
        readval = nrf_read(R_RX_PL_WID); 
        if (readval <= 32) {
          tryno = status_rd();
          Serial.print("== ");
          Serial.print(channel); 
          Serial.print(" "); 
          Serial.print(readval); 
          Serial.print(" Status: "); 
          Serial.print(tryno);
          nrf_multi_read(R_RX_PAYLOAD, readval, packet);
          packet[readval] = 0x00;
          flush_rx();
          nrf_write(0x07,0x78); //reset status register
          if (readval > 0)
          {  
            Serial.print("Packet obtained of size :"); 
            Serial.print(readval); 
            Serial.print(" ");
            hexdump(packet,readval); 
            //starttime = millis();  // reset the timeout counter if packet received
          }
        }
      }
      //delay(200); //ms
   }
   channel = 1; 
    
}
//-------------------------------------

