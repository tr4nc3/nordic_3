//
// TODO
// Set NRF config register
// Disable shockburst according to Travis
//
// See goodfet.nrf AutoTuner class

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
//#include <HashMap.h>
//---------------------------------------------
// Hardware configuration
// CEPIN is the read enable pin - see p22, section 6.1.4
const int CEPIN = 9;
// const int CSPIN = 9;

//MISO PIN gives status Pg 49, Section 8.3.2 nRF24L01+ datasheet
const int MISOPIN = 11; // green on Rajat's board

// MOSI PIN needs to get the command to read MOSI and status
const int MOSIPIN = 10; // yellow on Rajat's board
// CSNPIN is the SPI CS
// RED on Rajat's Board
const int CSNPIN = 10;

const int DEBUG_LED = 13;
//---------------------------------------------
#if (defined(__GNUC__) && defined(__i386__)) || defined(__WATCOMC__) \
  || defined(_MSC_VER) || defined (__BORLANDC__) || defined (__TURBOC__)
#define get16bits(d) (*((const uint16_t *) (d)))
#endif

#if !defined (get16bits)
#define get16bits(d) ((((uint32_t)(((const uint8_t *)(d))[1])) << 8)\
                       +(uint32_t)(((const uint8_t *)(d))[0]) )
#endif
// Thanks - http://www.azillionmonkeys.com/qed/hash.html
uint32_t SuperFastHash (const char * data, int len) 
{
  uint32_t hash = len, tmp;
  int rem;
  
      if (len <= 0 || data == NULL) return 0;
  
      rem = len & 3;
      len >>= 2;
  
      /* Main loop */
      for (;len > 0; len--) {
          hash  += get16bits (data);
          tmp    = (get16bits (data+2) << 11) ^ hash;
          hash   = (hash << 16) ^ tmp;
          data  += 2*sizeof (uint16_t);
          hash  += hash >> 11;
      }
  
      /* Handle end cases */
      switch (rem) {
          case 3: hash += get16bits (data);
                  hash ^= hash << 16;
                  hash ^= ((signed char)data[sizeof (uint16_t)]) << 18;
                  hash += hash >> 11;
                  break;
          case 2: hash += get16bits (data);
                  hash ^= hash << 11;
                  hash += hash >> 17;
                  break;
          case 1: hash += (signed char)*data;
                  hash ^= hash << 10;
                  hash += hash >> 1;
      }
  
      /* Force "avalanching" of final 127 bits */
      hash ^= hash << 3;
      hash += hash >> 5;
      hash ^= hash << 4;
      hash += hash >> 17;
      hash ^= hash << 25;
      hash += hash >> 6;
  
      return hash;
}

// ---------------------------------------
struct node 
{
  unsigned char* value;
  uint32_t       count; 
  int   valuelen; 
  struct node*   next; 
  node() 
  {
    value = NULL;
    count = 0;
    valuelen = 0 ;
    next = NULL;
  }
  ~node() 
  {
    delete[] value; 
    next = NULL; 
    valuelen = 0; 
    count = 0;
  }
  node(unsigned char* val,int len) 
  {
    value = new unsigned char[len]; 
    valuelen = len; 
    memcpy(value,val,len);
    count = 1; 
    next = NULL; 
  }
}; 
// ---------------------------------------
const int HASH_SIZE = 48; 
class HashTable 
{
  private:
    struct node**  tblbase; 
    uint32_t*      bucketsize; 
    //uint32_t       hash;     // hash code - an int representation of the key
    //unsigned int   keysize; // size of the unsigned char*
    //unsigned int   value;   // value for the key
    //unsigned char* key;     // the key field
    unsigned int   tblsize; // size of the table 
    unsigned int   maxtblsize;
    uint32_t getHashCode(unsigned char* keytosearch,int len) 
    {
      return SuperFastHash((const char*) keytosearch,len);
    }
  public:
    HashTable() 
    {
      maxtblsize = HASH_SIZE; 
      tblbase = new struct node* [HASH_SIZE];  // allocate HASH_SIZE of elements and init each to NULL, allocating pointers
      bucketsize = new uint32_t[HASH_SIZE];    //count of number of values in a bucket
      for (int i=0; i < HASH_SIZE; i++) 
      {
        tblbase[i] = NULL; 
        bucketsize[i] = 0;
      }
      tblsize = 0; 
    }
    ~HashTable() 
    {
      for (int i = 0; i < maxtblsize; i++) 
      {
        if (bucketsize[i] > 1) 
        {
          struct node* tmp = tblbase[i]; 
          struct node* nxt = tmp->next; 
          while (tmp != NULL) 
          {
            delete tmp;
            tmp = nxt; 
            nxt = tmp->next; 
          }
        }
      }
      delete[]  bucketsize;
      delete[]  tblbase; 
    }
    HashTable(int val) 
    {  //size
      maxtblsize = val; 
      tblbase = new struct node* [val];  // allocate HASH_SIZE of elements and init to NULL
      bucketsize = new uint32_t[val];  // count of number of values in that bucket
      for (int i=0; i < val; i++) 
      {
        tblbase[i] = NULL; 
        bucketsize[i] = 0 ; 
      }
    }
    struct node* contains(unsigned char* val2search, uint32_t vallen) 
    {
      uint32_t hashval =  getHashCode(val2search,vallen);  
      if (containsHash(hashval)) 
      {  // Hash is contained ... check if there is a bucket size > 0 (conflicts)
        if (bucketsize[hashval % maxtblsize ] > 0 )
        {
          struct node* temp = tblbase[hashval % maxtblsize ];
          for (int i = 0 ; i < bucketsize[hashval % maxtblsize] ; i++) 
          {
            if (temp != NULL)
            {
              if (temp->valuelen == vallen) 
              { // if string length is not equal ignore comparison
                if (memcmp(val2search, temp->value, vallen) == 0 )
                {
                   return temp;
                }
              }
            }
            temp = temp->next; 
          }
        }
      }
      return NULL;
    }
    int containsHash(uint32_t hash) 
    {
      if (tblbase[hash % maxtblsize] != NULL) 
        return 1;
      return 0;
    }

    // ht["bla"] = 4;  "bla" is 3 chars, value = 4
    void addKey(unsigned char* key, unsigned int keylen, unsigned int value) 
    {
      uint32_t hashinsert  = getHashCode(key, keylen); 
      struct node* entry = NULL ; 
      if ( ( entry = this->contains(key,keylen) ) != NULL) 
      {  // value is found, overwrite that value
        entry->count = value; 
      }
      else 
      {
        struct node* newnode = new struct node(key,keylen); 
        //newnode->next = NULL; 
        tblbase[hashinsert % maxtblsize] = newnode;
        bucketsize[hashinsert % maxtblsize] += 1; 
      }
      this->tblsize++;
    }

    struct node* getValue (unsigned char* key2search, int keylen) 
    {
      uint32_t hashval = SuperFastHash((const char*) key2search,keylen);
      struct node* entry = NULL; 
      if ( ( entry = this->contains(key2search,keylen) ) != NULL )
      {
        //return tblbase[ hashval % maxtblsize ] ; 
        return entry;  // return the node, dereference the pointer at the caller and get values 
      }
      return NULL;
    }

    int deleteKey(unsigned char* val2del, uint32_t vallen) 
    {  // TODO
      return 0;
    }

    void debug() 
    {
      int counter;  
      Serial.print("Max. size is : "); Serial.println(maxtblsize);
      for (int i=0; i < maxtblsize; i++) 
      {
        if (tblbase[i] != NULL) 
        {
          counter = 0; 
          //Serial.print("Row number : ");Serial.print(i);Serial.print(" has ");Serial.print(bucketsize[i]);Serial.println(" values");
          Serial.print("tblsize[");Serial.print(i);Serial.print("] has ");Serial.print(bucketsize[i]);Serial.println(" values");
          struct node* temp = tblbase[i];
          while (temp != NULL) 
          {
            Serial.print("len(bucket[");Serial.print(counter);Serial.print("]) = ");Serial.println(temp->valuelen);
            for (int j = 0 ; j < temp->valuelen; j++) 
            {
              Serial.print("0x");Serial.print(temp->value[j],HEX); 
              Serial.print(" ");
            }
            Serial.print("; Count is : ");  Serial.println(temp->count); 
            temp = temp->next;
            counter++; 
          }
          Serial.println("");
        }
      }
    }
};

// ---------------------------------------

//HashType<uint8_t*,int> hashRawArray[HASH_SIZE]; 
//HashMap<uint8_t*,int> hashMap = HashMap<uint8_t*,int>( hashRawArray , HASH_SIZE );
//uint16_t  hashlastindex = 0 ; 

HashTable ht; 
const uint8_t MAX_LEN_SRCS = 96;
uint8_t validsrcs[MAX_LEN_SRCS][5]; 
uint8_t validcnt = 0;

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
// Search valid entries in the array, size not sent because implicitly assumed to be 5
// TODO: add size param and make it generic
int is_already_valid(uint8_t* candidate) {
  for (int i=0; i < validcnt; i++) {
    if ( !compare_arr(validsrcs[i],candidate,5) ) // identical
       return 1; 
  }
  return 0; // went through the array, no already captured MACs found
}

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
// R_RX_PL_WID Command
//-------------------------------------
uint8_t r_rx_pld_wid() {
  uint8_t rxpayloadwidth;
  uint8_t cmd = 0x60;
  uint8_t junk = 0x00; 
  
  digitalWrite(CSNPIN, LOW);
  SPI.transfer(cmd);
  rxpayloadwidth = SPI.transfer(junk);
  digitalWrite(CSNPIN, HIGH);

  return rxpayloadwidth;
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
  //pinMode(CSPIN, OUTPUT);
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


  nrf_write(CONFIG,0x00); //stop nRF24L01+ 
  nrf_write(EN_AA, 0x00); //disable shockburst
  nrf_write(EN_RXADDR, 0x01); //enable data pipe 0
  nrf_write(DYNPD, 0x00); 
  nrf_write(FEATURE, 0x00); 
  nrf_write(SETUP_AW,AW_2); 
  nrf_write(CONFIG, 0x73); 
  
  delay(2000);
  //while (1) ;
}

void r_rx_payload(uint8_t* data, uint8_t numbytes) {
  uint8_t junk = 0x00;
  
  digitalWrite(CSNPIN, LOW);
  SPI.transfer(R_RX_PAYLOAD);
  for (int i=0; i < numbytes; i++) {
    //data[numbytes-1 -i] = SPI.transfer(junk);
    data[i] = SPI.transfer(junk);
  }
  digitalWrite(CSNPIN, HIGH);    
  return ;  
}
void flush_rx() {
  digitalWrite(CSNPIN, LOW) ;
  SPI.transfer(FLUSH_RX);
  digitalWrite(CSNPIN, HIGH); 
}

void flush_tx() 
{
  digitalWrite(CSNPIN, LOW) ;
  SPI.transfer(FLUSH_TX);
  digitalWrite(CSNPIN, HIGH); 
}

void hexdump(uint8_t* arr, uint8_t len) 
{
  for (int i = 0; i < len; i++) {
    Serial.print("0x");
    Serial.print(arr[i], HEX);
    Serial.print(" ");
  }
  //Serial.println("");
}


// Update a CRC16-CCITT with 1-8 bits from a given byte
uint16_t crc_update(uint16_t crc, uint8_t byt, uint8_t bits)
{
  crc = crc ^ (byt << 8);
  while(bits--)
    if((crc & 0x8000) == 0x8000) crc = (crc << 1) ^ 0x1021;
    else crc = crc << 1;
  crc = crc & 0xFFFF;
  return crc;
}
void subsection(uint8_t* src, uint8_t* dst, uint8_t startindex, uint8_t len, uint8_t arraylen) {
  if (startindex + len < arraylen)
    for (int i = startindex ; i < len; i++) {
      dst[i-startindex] = src[i]; 
    }
    return;
}
void array_lshifter(uint8_t* payload, uint8_t len, uint8_t bitshift) 
{
  uint8_t mask  = 0; 
  if (bitshift > 0 && bitshift < 8 )  // multi-byte bitmap shifting not supported
  {
    for (int i = 1 ; i < bitshift; i++) {
      mask = mask | (2 << i) ;
    }
    for (int i = len; i > 0; i--)  {
      if (i >= 2)
         payload[i-1] = payload[i-1] << bitshift | ( (payload[i-2] & mask) >> (8 - bitshift) ) ;
       else
         payload[i-1] = payload[i-1] << bitshift ;
    }
  }
}

void get_packet_details(uint8_t* payload, uint8_t* mac, int packetlen, int startindex)  
{  // payload is lowest bytes first a[0] == lowest significant byte
  uint16_t crc = 0 ;
  uint8_t payloadlen ;
  if (startindex == -1 ) 
  {
    crc = payload[1] << 8 | payload[0];
    payloadlen = payload[5] >> 2;
  }
  else payloadlen = payload[startindex + 5] >> 2; 
  //array_lshifter(payload, packetlen, 1) ; // left shift by 1-bit
  
  if (payloadlen <= 24) 
  {
    Serial.print("For MAC address: "); hexdump(mac,5); Serial.print(" payload length is : "); Serial.print(payloadlen); Serial.print(" and packet looks like this: ");
    hexdump(&payload[6+payloadlen], packetlen); 

  }
}

void  process_packet_data(uint8_t* payload, uint8_t packetlen) 
{
  /*
  for(int offset=0; offset < 2; offset++)  // do two tries in case the packet header had 0x55 instead of 0xAA as the preamble
  {
    array_lshifter(payload, packetlen, offset) ; 
    // Read the payload length
    int payload_length = payload[5] >> 2;
    
    uint16_t crc_given, crc;
    // Check for a valid payload length, which is less than the usual 32 bytes 
    // because we need to account for the packet header, CRC, and part or all 
    // of the address bytes. 
    if(payload_length <= 24)
    {
      Serial.print("Payload appears to be : ");Serial.print(payload_length); Serial.println(" bytes");
      // Read the given CRC
      crc_given = (payload[packetlen- 1  - 6 - payload_length + 1] << 9) | (payload[packetlen - 7 - payload_length] << 1);  
      crc_given = (crc_given << 8) | (crc_given >> 8);
      if (payload[packetlen - 1 - 8 - payload_length + 1] & 0x80) 
         crc_given |= 0x100;  //???

         // Calculate the CRC
       crc = 0xFFFF;
       for(int x = 2; x < packetlen ; x++) 
         crc = crc_update(crc, payload[x], 8);
       crc = crc_update(crc, payload[packetlen +1 - 6 - payload_length] & 0x80, 1);
       crc = (crc << 8) | (crc >> 8);
       // Verify the CRC
       Serial.print("CRC: "); Serial.print(crc); Serial.print(" CRC_GIVEN: "); Serial.println(crc_given);
       if(crc == crc_given)
       {
          Serial.print("Valid Packet Found!!!!");
          hexdump(payload,5+payload_length);
       }
     }
  }*/
  /*
  uint8_t payloadlens[2] = { 0x00, 0x00 } ;
  payloadlens[0] = payload[5] << 1 & 0xfc;
  payloadlens[0] = payloadlens[0] >> 2;      // 1st possible payload length
  payloadlens[1] = (payload[5] & 0x3f) >> 2; // 2nd possible payload length
  uint16_t crc_given, crc; 
  
  uint8_t prefix_length = 5; // Address length (could be 3, 4 or 5 bytes ... let's go with 5 for now)

  for (int i = 0; i < 2 ; i++) 
  {
    
    crc_given = (payload[6 + payloadlens[i]] << 8) | ((payload[7 + payloadlens[i]]) << 1);  
    crc_given = (crc_given << 8) | (crc_given >> 8);
    if (payload[8 + payloadlens[i]] & 0x80) 
      crc_given |= 0x100;

    // Calculate the CRC
    crc = 0xFFFF;
    for(int x = 0; x < 6 + payloadlens[i]; x++) 
       crc = crc_update(crc, payload[x], 8);
    crc = crc_update(crc, payload[6 + payloadlens[i]] & 0x80, 1);
    crc = (crc << 8) | (crc >> 8);
    // Verify the CRC
    Serial.print("CRC: "); Serial.print(crc); Serial.print(" CRC_GIVEN: "); Serial.println(crc_given);
    if(crc == crc_given)
    {
      Serial.print("Valid Packet Found!!!!");
      hexdump(payload,5+payloadlens[i]);
    }
  } */
  // packetlen is the size of the payload array  
  int delimloc[33] = { -1 } ;
  uint8_t delimlen = 0;
  uint8_t packetsection[5]; 
  uint8_t fakelimit = 6;
  int index = 1 ;
  struct node* elem = NULL; 
  
  delimloc[0] = -1; //0th
  for (int i = 0 ; i < packetlen; i++) {
    if (payload[i] == 0x55 || payload[i] == 0xAA) {
      delimloc[index++] = i;
    }
  }
  delimlen = index ;
  Serial.print("Found "); Serial.print(delimlen); Serial.println(" delimiters");
  index = 0;
  for (int j=0; j < delimlen; j++) 
  {
    if (delimloc[j] + fakelimit > packetlen) 
    {
      fakelimit = packetlen - delimloc[j] - 1;
      if (fakelimit > packetlen)  // limit can't be greater than packetlen
        //return;
        continue;
    } 
    else 
    {
      if (delimloc[j]+6 < packetlen) 
      {
        //subsection(payload, packetsection, i, 5, packetlen );
        subsection(payload, packetsection, delimloc[j]+1, 5, packetlen );
        //hexdump(packetsection,5);
        elem = ht.contains(packetsection,5);
        //index = is_already_valid(packetsection);
        if ( elem != NULL ) 
        {  
        //if (index) {
          Serial.print("Found MAC! ");
          hexdump(packetsection,5); 
          elem->count++;
          
          //hashMap[index](packetsection,hashMap.getValueOf(packetsection) + 1); 
          //hashMap[index].setValue(hashMap.getValueOf(packetsection) + 1);
          //Serial.print("Found at index : " ); Serial.print(index); Serial.print(" with value "); Serial.println(hashMap.getValueOf(packetsection));
          //return packetsection; 
         /*if (! is_already_valid(packetsection)) 
            if (validcnt < MAX_LEN_SRCS) {
              for (int in = 0 ; in < 5 ; in++ ) 
                 validsrcs[validcnt][in] = packetsection[in];
              validcnt++;
            }
            else {
              Serial.println (" ... too bad .. already captured ...");
              get_packet_details(payload, packetsection, packetlen, delimloc[j] ) ; 
            }*/
        }
        else
        {
          /*if (! is_already_valid(packetsection)) {
            if (validcnt < MAX_LEN_SRCS) {
              for (int in = 0 ; in < 5 ; in++ ) 
                 validsrcs[validcnt][in] = packetsection[in];
              validcnt++;
            }
          }*/
          //hashMap[hashlastindex++](packetsection, 1); 
          // Not present ... so add it
          ht.addKey(packetsection,5,1);  // value, len of value, count
        }
        //ht.debug();
      }
      else 
      {
        Serial.println("Delimiter + 5 exceeds packetlen");
        return;
      }
    }
  }
}  // end of for all delims

int compare_arr(uint8_t* src, uint8_t* dst, int len) {

  for (int i = 0; i < len; i++) {
    if (src[i] != dst[i]) 
      return src[i] - dst[i];
  }
  return 0; 
}
//-------------------------------------
void loop() {
   uint8_t promiscuous_address[2] = { 0x55, 0x00 };
   //uint8_t promiscuous_address[3] = { 0x11, 0x22, 0x33 };
   unsigned long timeout = 100;
   channel = 1; 
   int readval ;
   uint8_t packet[37];
   uint8_t tryno = 0;
   uint8_t addr[5]  = { 0xff, 0xff, 0xff, 0xff, 0xff } ; 
   uint8_t fifostatus ; 
   uint16_t packetscaptured = 0 ;
   uint8_t rcvdpowerflag = 0 ;
   uint8_t mouseaddr[5] = { 0xA8, 0xAC, 0x0D, 0x41, 0x66 }; 
   for (int m = 0 ; m < 2 ; m++)  
   {  // try both trailers
     if (m == 1 ) promiscuous_address[0] = 0xAA;
     
     while (channel < 85) 
     {
        unsigned long starttime = millis();
        nrf_write(STATUS,0x70); //reset status register
        nrf_write(CONFIG, 0x73); //disable crc, power up,  mask interrups
        configure_address(promiscuous_address, 2);
        //configure_address(mouseaddr,5);
        configure_mac(0,0, ENAA_NONE);
        
      // Set for transmission
      //  Please note the CE PIN must be driven high.
        configure_phy (PRIM_RX | PWR_UP, RATE_2M, 32);
        //configure_phy (PRIM_RX | PWR_UP, RATE_1M, 32);
        //configure_phy(PRIM_RX | PWR_UP, RATE_250K, 32);
        delay(2);
        digitalWrite(CEPIN, HIGH); // Page 21, nRF24L01+ datasheet (Figure 3)
        delay(1);
        //nrf_write(RX_PW_P0,0x11); // set 32 bytes in RX payload
        Serial.print("Tuning to channel: "); Serial.println(channel);
        nrf_write(RF_CH,channel); //  Set the frequency to channel
        /*nrf_write(SETUP_AW, 0x00); 
        nrf_write(FEATURE, 0); // Disable dyn payload length,
        nrf_write(DYNPD, 0); // Disable dynamic payload
        nrf_write(EN_AA, 0); // Disable auto acknowledgement
        */
        packetscaptured = 0;
        while (millis() - starttime < timeout) 
        {
          readval = r_rx_pld_wid(); 
          if (readval <= 32 && readval > 0) 
          {
            tryno = status_rd();
            r_rx_payload(packet,readval); // lsb at byte [0], msb at byte [len - 1]
            nrf_multi_read(RX_ADDR_P0, 5, addr);
            rcvdpowerflag = nrf_read(RPD);
            nrf_write(STATUS, 0x40); //reset status
            fifostatus = nrf_read(FIFO_STATUS); // Expect the RX_EMPTY flag set
            if (tryno == 0x40) 
            {
              packetscaptured++;
              Serial.print("== ");
              Serial.print(channel); 
              Serial.print(" "); 
              Serial.print(readval); 
              Serial.print("  Status reg: "); 
              Serial.print(tryno, HEX);
              Serial.print(" Address : ");
              for (int i=0; i < 5; i++) 
              {
                 Serial.print(addr[i], HEX); Serial.print(" ");
              } 
              Serial.print(" FIFO_STATUS : ");
              Serial.print(fifostatus,HEX);
              Serial.print(" RPD Flag : "); 
              Serial.print(rcvdpowerflag);
              Serial.println("");
              if (readval > 0)
              {  
                Serial.print("Packet obtained of size :"); 
                Serial.print(readval); 
                Serial.println(" ");
                hexdump(packet,readval);
                process_packet_data(packet, readval);  //readval is packetlen
                starttime = millis();  // reset the timeout counter if packet received
              }
            }
            else 
              if (tryno == 0x00) {
                flush_rx();
              }
          }
         nrf_write(STATUS,0x40); //clear status reg 
        }
        Serial.print(" Packets captured at Channel "); Serial.print(channel); Serial.print(" : "); Serial.println(packetscaptured); 
        channel++;
        packetscaptured = 0 ; 
        //delay(200); //ms
     }
     //channel = 1;
     ht.debug();
     //for (int in = 0 ; in < validcnt; in++) { 
         //hexdump(validsrcs[in],5);Serial.println("");
//         Serial.print(" Occurred ");Serial.print(hashMap.getValueOf(validsrcs[in]),DEC); Serial.println(" times ");
     //}
   }  
}
//-------------------------------------


