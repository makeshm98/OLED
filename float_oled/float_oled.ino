#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define OLED_DC     8   // Data/Command pin
//#define OLED_DC     7
#define OLED_CS    10  // Chip Select pin
#define OLED_RST    7   // Reset pin 

#define PIN_INTERRUPT 10  // Digital pin 10
#define PIN_LED 9        // Digital pin 9 (LED)

// Flag to indicate that an interrupt occurred
volatile bool interruptTriggered = false;

int battery = 0;
int nitrogen = 0;

//uint8_t buffer[1024]; // 128x64 / 8 = 1024 bytes
uint8_t textsize_x = 2; // Text magnification factor in X direction
uint8_t textsize_y = 2; // Text magnification factor in Y direction

float temp_fahrenheit  = 0;
float temp_degrees  = 0;
unsigned int HighByte = 0;
unsigned int LowByte  = 0;
unsigned int Len  = 0;


static const PROGMEM u1_t NWKSKEY[16] = {  0x31, 0x22, 0x47, 0x0F, 0x92, 0x46, 0x00, 0x9A, 0x0A, 0x95, 0x90, 0x18, 0xEC, 0x49, 0x14, 0x34 };
static const u1_t PROGMEM APPSKEY[16] = { 0x86, 0x69, 0xA8, 0x1F, 0xC4, 0x0E, 0x25, 0x1B, 0xC2, 0x58, 0x98, 0xD6, 0x9F, 0x20, 0xCA, 0x53 };
static const u4_t DEVADDR = 0x012C1E92;
//static const PROGMEM u1_t NWKSKEY[16] = {  0x80, 0x35, 0x2D, 0x6F, 0xA1, 0x09, 0x50, 0xF8, 0x72, 0x59, 0x8D, 0xEB, 0x93, 0xF7, 0x82, 0x12 };
//static const u1_t PROGMEM APPSKEY[16] = { 0xB6, 0x84, 0xB6, 0x06, 0x00, 0x5A, 0xE7, 0x04, 0xE5, 0x68, 0x21, 0x25, 0xDB, 0x89, 0x28, 0x99 };
//static const u4_t DEVADDR = 0x012D74E1;
#define SLEEP

#ifdef COMPILE_REGRESSION_TEST
# define CFG_in866 1
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif


void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[7];
static osjob_t sendjob;


const unsigned TX_INTERVAL = 900;

#ifdef SLEEP
#include "LowPower.h" 
bool next = false;
#endif

const lmic_pinmap lmic_pins = {
    .nss = 6,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
      break;
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      #ifndef SLEEP
      os_setTimedCal
      lback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      #else
      next = true;
      #endif
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}





void voltage()                         
{
  double analogvalue = analogRead(A3);
  double bat_temp = ((analogvalue * 3.3) / 1024); 
  double sum = 0;
  double avg = 0;
  short int volt = 0;
  for (byte  i = 0; i < 4; i++)
  {
    sum += bat_temp * 2;
  }
  avg = (sum / 4);
  volt = avg*1000;
 
  Serial.print(avg);
  Serial.println(F(" V avg"));
  Serial.println(F("-------------"));
  Serial.print(F(" Battery Voltage: "));
  Serial.print(volt);
  Serial.println(F(" V"));
  Serial.println(F("-------------"));

  mydata[3]=highByte(volt);
  mydata[4]=lowByte(volt);
  delay(1000);
}

void do_send(osjob_t* j){

  voltage();
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
}

void Nitrogen()
{
  nitrogen = 95;
}

void setup() {

  Serial.begin(115200);
  Serial.println(F("Starting"));

  // Set up the LED pin
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW); // Ensure LED is off initially
  //DDRB |= (1 << PB5);

  // Set up the interrupt pin
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);
  //pinMode(PIN_INTERRUPT, INPUT);

    // Set pin modes
    pinMode(OLED_DC, OUTPUT);
    pinMode(OLED_CS, OUTPUT);
    pinMode(OLED_RST, OUTPUT);

    // Start SPI communication
    //SPI.begin();
    //SPI.setClockDivider(SPI_CLOCK_DIV2);

  // Configure Pin Change Interrupts for Port B (PCINT0 - PCINT7)
  PCICR |= (1 << PCIE0);  // Enable PCINT interrupts for Port B
  PCMSK0 |= (1 << PCINT2); // Enable interrupt for pin 10 (PCINT2)

  #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
  #endif
    


    os_init();
    LMIC_reset();


    #ifdef PROGMEM

    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_in866)
        LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(1, 865232500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(2, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(3, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(4, 866185000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(5, 866385000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(6, 866585000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
        LMIC_setupChannel(7, 866785000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    #elif defined(CFG_us915)

    LMIC_selectSubBand(1);
    #endif

    LMIC_setLinkCheckMode(0);

    LMIC.dn2Dr = DR_SF9;

    LMIC_setDrTxpow(DR_SF12,14);

    do_send(&sendjob);
}

void oledInit() {

    // Start SPI communication
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2);

    oledReset();  // Reset the display

    sendCommand(0xAE); // Display OFF (sleep mode)
    sendCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    sendCommand(0x80); // Set divide ratio
    sendCommand(0xA8); // Set multiplex ratio(1 to 64)
    sendCommand(0x3F); // 1/64 duty
    sendCommand(0xD3); // Set display offset
    sendCommand(0x00); // No offset
    sendCommand(0x40); // Set start line address
    sendCommand(0x8D); // Charge pump
    sendCommand(0x14); // Enable charge pump
    sendCommand(0x20); // Set Memory Addressing Mode
    sendCommand(0x00); // Horizontal addressing mode
    sendCommand(0xA1); // Set Segment Re-map. A0=address mapped; A1=address 127 mapped.
    sendCommand(0xC8); // Set COM Output Scan Direction
    sendCommand(0xDA); // Set COM Pins hardware configuration
    sendCommand(0x12);
    sendCommand(0x81); // Set contrast control register
    sendCommand(0xCF);
    sendCommand(0xD9); // Set pre-charge period
    sendCommand(0xF1);
    sendCommand(0xDB); // Set VCOMH deselect level
    sendCommand(0x40);
    sendCommand(0xA4); // Entire Display ON
    sendCommand(0xA6); // Set normal display (not inverted)
    sendCommand(0x2E); // Deactivate scroll
    sendCommand(0xAF); // Display ON in normal mode
    SPI.end();

}

void sendCommand(uint8_t command) {
    digitalWrite(OLED_DC, LOW);  // Command mode
    //digitalWrite(OLED_CS, LOW);  // Select the OLED
    SPI.transfer(command);       // Send command
    digitalWrite(OLED_CS, HIGH); // Deselect the OLED
}

void sendData(uint8_t data) {
    digitalWrite(OLED_DC, HIGH); // Data mode
    //digitalWrite(OLED_CS, LOW);  // Select the OLED
    SPI.transfer(data);          // Send data
    digitalWrite(OLED_CS, HIGH); // Deselect the OLED
}

void oledReset() {
    digitalWrite(OLED_RST, LOW);  // Reset the OLED
    delay(50);                    // Hold reset for 50ms
    digitalWrite(OLED_RST, HIGH); // Release reset
}

//uint8_t buffer[1024]; // 128x64 / 8 = 1024 bytes
//uint8_t textsize_x = 2; // Text magnification factor in X direction
//uint8_t textsize_y = 2; // Text magnification factor in Y direction

void clearDisplay(uint8_t buffer[1024]) {
    memset(buffer, 0, 1024);
}

void updateDisplay(uint8_t buffer[1024]) {
    sendCommand(0x21); // Set column address
    sendCommand(0);    // Column start address
    sendCommand(127);  // Column end address
    sendCommand(0x22); // Set page address
    sendCommand(0);    // Page start address
    sendCommand(7);    // Page end address

    for (uint16_t i = 0; i < 1024; i++) {
        sendData(buffer[i]);
    }
}

void setPixel(uint8_t buffer[1024], int x, int y, bool color) {
    if (x >= 128 || y >= 64) return; // Ensure coordinates are within bounds
    if (color) {
        buffer[x + (y / 8) * 128] |= (1 << (y & 7));
    } else {
        buffer[x + (y / 8) * 128] &= ~(1 << (y & 7));
    }
}


//Bitmap representation for Capital Letters
const uint8_t PROGMEM font5x7_upper[][5] = {
    //{0x7F, 0x09, 0x09, 0x09, 0x7E},  // A
    //{0x00, 0x00, 0x5F, 0x00, 0x00}, //!
    //{0x7C, 0x12, 0x12, 0x7C, 0x00},  // A
    //{0x7E, 0x49, 0x49, 0x36, 0x00},  // B
    //{0x3E, 0x41, 0x41, 0x22, 0x00},  // C
    //{0x14, 0x7F, 0x14, 0x7F, 0x14},  // #
    //{0x7F, 0x49, 0x49, 0x49, 0x41}
    //{0x7C, 0x12, 0x11, 0x12, 0x7C},  //A
    //{0x7F, 0x49, 0x49, 0x49, 0x36},  //B
    //{0x3E, 0x41, 0x41, 0x41, 0x22},   //C
    //{0x7F, 0x41, 0x41, 0x41, 0x3E},   //D
    //{0x7F, 0x49, 0x49, 0x49, 0x41},   //E
    //{0x7F, 0x49, 0x49, 0x49, 0x41}
    
    {0x7C, 0x12, 0x11, 0x12, 0x7C},  // A
    {0x7F, 0x49, 0x49, 0x49, 0x36},  // B
    {0x3E, 0x41, 0x41, 0x41, 0x22},  // C
    {0x7F, 0x41, 0x41, 0x41, 0x3E},  // D
    {0x7F, 0x49, 0x49, 0x49, 0x41},  // E
    {0x7F, 0x09, 0x09, 0x09, 0x01},  // F
    {0x3E, 0x41, 0x41, 0x51, 0x73},  // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F},  // H
    {0x00, 0x41, 0x7F, 0x41, 0x00},  // I
    {0x20, 0x40, 0x41, 0x3F, 0x01},  // J
    {0x7F, 0x08, 0x14, 0x22, 0x41},  // K
    {0x7F, 0x40, 0x40, 0x40, 0x40},  // L
    {0x7F, 0x02, 0x1C, 0x02, 0x7F},  // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F},  // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E},  // O
    {0x7F, 0x09, 0x09, 0x09, 0x06},  // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E},  // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46},  // R
    {0x26, 0x49, 0x49, 0x49, 0x32},  // S
    {0x03, 0x01, 0x7F, 0x01, 0x03},  // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F},  // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F},  // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F},  // W
    {0x63, 0x14, 0x08, 0x14, 0x63},  // X
    {0x03, 0x04, 0x78, 0x04, 0x03},  // Y
    {0x61, 0x59, 0x49, 0x4D, 0x43},  // Z
/*
    {0x20, 0x54, 0x54, 0x78, 0x40},  // a
    {0x7F, 0x28, 0x44, 0x44, 0x38},  // b
    {0x38, 0x44, 0x44, 0x44, 0x28},  // c
    {0x38, 0x44, 0x44, 0x28, 0x7F},  // d
    {0x38, 0x54, 0x54, 0x54, 0x18},  // e
    {0x00, 0x08, 0x7E, 0x09, 0x02},  // f
    {0x18, 0xA4, 0xA4, 0x9C, 0x78},  // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},  // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},  // i
    {0x20, 0x40, 0x40, 0x3D, 0x00},  // j
    {0x7F, 0x10, 0x28, 0x44, 0x00},  // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},  // l
    {0x7C, 0x04, 0x78, 0x04, 0x78},  // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},  // n
    {0x38, 0x44, 0x44, 0x44, 0x38},  // o
    {0xFC, 0x18, 0x24, 0x24, 0x18},  // p
    {0x18, 0x24, 0x24, 0x18, 0xFC},  // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},  // r
    {0x48, 0x54, 0x54, 0x54, 0x24},  // s
    {0x04, 0x04, 0x3F, 0x44, 0x24},  // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},  // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},  // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},  // w
    {0x44, 0x28, 0x10, 0x28, 0x44},  // x 
    {0x4C, 0x90, 0x90, 0x90, 0x7C},  // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}   // z
    */
};


//Bitmap representation for small letters
const uint8_t PROGMEM font5x7_lower[][5] = {


    {0x20, 0x54, 0x54, 0x78, 0x40},  // a
    {0x7F, 0x28, 0x44, 0x44, 0x38},  // b
    {0x38, 0x44, 0x44, 0x44, 0x28},  // c
    {0x38, 0x44, 0x44, 0x28, 0x7F},  // d
    {0x38, 0x54, 0x54, 0x54, 0x18},  // e
    {0x00, 0x08, 0x7E, 0x09, 0x02},  // f
    {0x18, 0xA4, 0xA4, 0x9C, 0x78},  // g
    {0x7F, 0x08, 0x04, 0x04, 0x78},  // h
    {0x00, 0x44, 0x7D, 0x40, 0x00},  // i
    {0x20, 0x40, 0x40, 0x3D, 0x00},  // j
    {0x7F, 0x10, 0x28, 0x44, 0x00},  // k
    {0x00, 0x41, 0x7F, 0x40, 0x00},  // l
    {0x7C, 0x04, 0x78, 0x04, 0x78},  // m
    {0x7C, 0x08, 0x04, 0x04, 0x78},  // n
    {0x38, 0x44, 0x44, 0x44, 0x38},  // o
    {0xFC, 0x18, 0x24, 0x24, 0x18},  // p
    {0x18, 0x24, 0x24, 0x18, 0xFC},  // q
    {0x7C, 0x08, 0x04, 0x04, 0x08},  // r
    {0x48, 0x54, 0x54, 0x54, 0x24},  // s
    {0x04, 0x04, 0x3F, 0x44, 0x24},  // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C},  // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C},  // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C},  // w
    {0x44, 0x28, 0x10, 0x28, 0x44},  // x 
    {0x4C, 0x90, 0x90, 0x90, 0x7C},  // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}   // z
};


//Bitmap representation for digits
const uint8_t PROGMEM digits[][5] = {
    {0x3E, 0x51, 0x49, 0x45, 0x3E},  //0
    {0x00, 0x42, 0x7F, 0x40, 0x00},  //1
    {0x72, 0x49, 0x49, 0x49, 0x46},  //2
    {0x21, 0x41, 0x49, 0x4D, 0x33},  //3
    {0x18, 0x14, 0x12, 0x7F, 0x10},  //4
    {0x27, 0x45, 0x45, 0x45, 0x39},  //5
    {0x3C, 0x4A, 0x49, 0x49, 0x31},  //6
    {0x41, 0x21, 0x11, 0x09, 0x07},  //7
    {0x36, 0x49, 0x49, 0x49, 0x36},  //8
    {0x46, 0x49, 0x49, 0x29, 0x1E},  //9

    //{0x20, 0x10, 0x08, 0x04, 0x02}   // /

};


//Bitmap representation for special characters
const uint8_t PROGMEM special[][5] = {
    {0x00, 0x00, 0x14, 0x00, 0x00},  // :
    //{0x00, 0x00, 0x60, 0x60, 0x00},  // .
};

//Bitmap representation for extra characters
const uint8_t PROGMEM extra[][5] = {
    //{0x00, 0x00, 0x60, 0x60, 0x00},  // .
    //{0x00, 0x80, 0x70, 0x30, 0x00},  //,
    {0x00, 0x00, 0x60, 0x60, 0x00},  // .
};

/*
void drawChar(uint8_t buffer[1024], int x, int y, char c) {
    if ((c >= 'A' && c <= 'Z')) {
        const uint8_t* charData = font5x7_upper[c - 'A'];

        for (int i = 0; i < 5; i++) {
            uint8_t line = charData[i];
            for (int j = 0; j < 10; j++) {
                if (line & (1 << j)) {
                    // Scale and draw each pixel
                    for (int dx = 0; dx < textsize_x; dx++) {
                        for (int dy = 0; dy < textsize_y; dy++) {
                            setPixel(buffer, x + i * textsize_x + dx, y + j * textsize_y + dy, true);
                        }
                    }
                }
            }
        }
    }

    else if  ((c >= 'a' && c <= 'z'))
    {
        const uint8_t* charData = font5x7_lower[c - 'a'];

        for (int i = 0; i < 5; i++) {
            uint8_t line = charData[i];
            for (int j = 0; j < 10; j++) {
                if (line & (1 << j)) {
                    // Scale and draw each pixel
                    for (int dx = 0; dx < textsize_x; dx++) {
                        for (int dy = 0; dy < textsize_y; dy++) {
                            setPixel(buffer, x + i * textsize_x + dx, y + j * textsize_y + dy, true);
                        }
                    }
                }
            }
        }
    }

    else if ((c >= '0' && c <= '9'))
    {
      const uint8_t* charData = digits[c - '0'];
      for(int i = 0; i < 5; i++)
      {
        uint8_t line =  charData[i];
        for(int j = 0; j < 10; j++)
        {
          if (line & (1 << j)) 
          {
            for (int dx = 0; dx < textsize_x; dx++) 
            {
                for (int dy = 0; dy < textsize_y; dy++) 
                {
                  setPixel(buffer, x + i * textsize_x + dx, y + j * textsize_y + dy, true);
                }
            }
          } 
        }
      }
    }
}
*/

void drawChar(uint8_t buffer[1024], int x, int y, char c) {
    const uint8_t* charData = nullptr;
    
    if (c >= 'A' && c <= 'Z') {
        charData = font5x7_upper[c - 'A'];
    } else if (c >= 'a' && c <= 'z') {
        charData = font5x7_lower[c - 'a'];
    } else if (c >= '0' && c <= '9') {
          if (c >= '.' && c <= '/') {
            charData = extra[c - '.'];
          }
      charData = digits[c - '0'];
    } else if (c >= ':' && c <= '@') {
        charData = special[c - ':'];
    } else if (c >= '.' && c <= '/') {
        charData = extra[c - '.'];
    }

    if (charData) {
        for (int i = 0; i < 5; i++) {
            uint8_t line = pgm_read_byte(&charData[i]);
            for (int j = 0; j < 10; j++) { // Changed to 7 for 5x7 bitmap
                if (line & (1 << j)) {
                    // Scale and draw each pixel
                    for (int dx = 0; dx < textsize_x; dx++) {
                        for (int dy = 0; dy < textsize_y; dy++) {
                            setPixel(buffer, x + i * textsize_x + dx, y + j * textsize_y + dy, true);
                        }
                    }
                }
            }
        }
    }
}

int integer()
{
  int count = 0;
  long num = 25;
  do {
    num /= 10;
    ++count;
  } while (num != 0);

  int a[10];
  for(int i = 0;  i < count; i++)
  {
   a[i] = num % 10; 
   num /= 10;
  }

  char str[10];
  int digitsize = sizeof(a) / sizeof(a[0]);
  for(int i = 0; i < digitsize; i++)
  {
    sprintf(str[i], "%d", a[i]);
  }

  const char* str_digit[10];
  for(int i = 0; i < digitsize; i++)
  {
    str_digit[i] = str[i];
  }
  return str_digit[10];
}

int number()
{
  int temperature = 2345;
  char str2[10];
  sprintf(str2, "%d", temperature);

  char a = str2[0];  
  char b = str2[1];  
  char c = str2[2];  
  char d = str2[3];  
  char e = str2[4];  

  //const char* digit = str2;
  return a;
}
/*
const char* number(int* digit_01, int* digit_02, int* digit_03, int* digit_04, int* digit_05)
{

  int count = 0;
  long long temperature = 25;
  int a, b, c, d, e;

  do {
    temperature /= 10;
    ++count;
  } while (temperature != 0);



  if(count == 1)
  {

    //int temperature = 25;
    char str2[10];
    sprintf(str2, "%d", temperature);
    const char* digit = str2;
    return digit; 
  }

  if(count == 2)
  {
    //int temperature = 25;


    b = temperature % 10;
    temperature /= 10;
    a = temperature % 10;

    char str1[10];
    sprintf(str1, "%d", temperature);
    const char* digit_01 = str1;

    char str2[10];
    sprintf(str2, "%d", temperature);
    const char* digit_02 = str2;

    return digit_01, digit_02;
  }

  if(count == 3)
  {
    //int temperature = 25;

    c = temperature % 10;
    temperature /= 10;
    b = temperature % 10;
    temperature /= 10;
    a = temperature % 10;

    char str1[10];
    sprintf(str1, "%d", a);
    const char* digit_01 = str1;

    char str2[10];
    sprintf(str2, "%d", b);
    const char* digit_02 = str2;

    char str3[10];
    sprintf(str3, "%d", c);
    const char* digit_03 = str3;

    return digit_01, digit_02, digit_03;
  }

  if(count == 4)
  {
    //int temperature = 25;


    d = temperature % 10;
    temperature /= 10;
    c = temperature % 10;
    temperature /= 10;
    b = temperature % 10;
    temperature /= 10;
    a = temperature % 10;

    char str1[10];
    sprintf(str1, "%d", a);
    const char* digit_01 = str1;

    char str2[10];
    sprintf(str2, "%d", b);
    const char* digit_02 = str2;

    char str3[10];
    sprintf(str3, "%d", c);
    const char* digit_03 = str3;

    char str4[10];
    sprintf(str4, "%d", d);
    const char* digit_04 = str4;

    return digit_01, digit_02, digit_03, digit_04;
  }

  if(count == 5)
  {

    e = temperature % 10;          
    temperature /= 10;
    d = temperature % 10;
    temperature /= 10;
    c = temperature % 10;
    temperature /= 10;
    b = temperature % 10;
    temperature /= 10;
    a = temperature % 10;

    char str1[10];
    sprintf(str1, "%d", a);
    const char* digit_01 = str1;

    char str2[10];
    sprintf(str2, "%d", b);
    const char* digit_02 = str2;

    char str3[10];
    sprintf(str3, "%d", c);
    const char* digit_03 = str3;

    char str4[10];
    sprintf(str4, "%d", d);
    const char* digit_04 = str4;

    char str5[10];
    sprintf(str5, "%d", e);
    const char* digit_05 = str5;

    return digit_01, digit_02, digit_03, digit_04;
  }

  /*
  int temperature = 25;
  char str2[10];
  sprintf(str2, "%d", temperature);
  const char* digit = str2;
  return digit;

}*/



void drawString(uint8_t buffer[1024], int x, int y, const char* str) {
    while (*str) {
        drawChar(buffer, x, y, *str);

        x += 6 * textsize_x; // Move to the next character position (5 pixels wide)
        str++;
    }
}

void loop() {

  #ifndef SLEEP
    os_runloop_once();
  #else

  extern volatile unsigned long timer0_overflow_count;
  if (next == false) {
    os_runloop_once();
    if (interruptTriggered) {
      hi();
      interruptTriggered = false;
    }
  } 
  
  else 
  
  {
  hi();
  digitalWrite(PIN_LED, LOW);
  int sleepcycles = TX_INTERVAL / 8;  
  #ifndef DEBUG
      Serial.print(F("Enter sleeping for "));
      Serial.print(sleepcycles);
      Serial.println(F(" cycles of 8 seconds"));
  #endif
  Serial.flush(); 
  for (int i = 0; i < sleepcycles; i++) {
    // Set SCK, MOSI, MISO to low
    //pinMode(SCK, OUTPUT);
    //pinMode(MOSI, OUTPUT);
    //digitalWrite(SCK, LOW);
    //digitalWrite(MOSI, LOW);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);



  if (interruptTriggered) {
/*    
    // Blink the LED
    for (int i = 0; i < 5; i++) {
      digitalWrite(PIN_LED, HIGH);
      delay(200);
      digitalWrite(PIN_LED, LOW);
      delay(200);
    }

    oledInit();

    uint8_t buffer[1024];
    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "T");
    drawString(buffer, 12, 0, "e");
    drawString(buffer, 24, 0, "m");
    drawString(buffer, 36, 0, "p");
    updateDisplay(buffer); 
    delay(1000);     

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "D");
    drawString(buffer, 12, 0, "i");
    drawString(buffer, 24, 0, "s");
    drawString(buffer, 36, 0, "t");
    updateDisplay(buffer);
    delay(1000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "8");
    drawString(buffer, 12, 0, "9");
    drawString(buffer, 24, 0, "6");
    drawString(buffer, 36, 0, "7");
    updateDisplay(buffer);
    delay(1000);
    clearDisplay(buffer);
    sendCommand(0xAE); // Display OFF (sleep mode)
    //SPI.end();
    //digitalWrite(OLED_CS, LOW);
    //DDRB |= (1 << PB5);
    //PORTB &= ~(1 << PB5);

    // Print message to Serial Monitor
    //Serial.println("Interrupt occurred, LED blinked.");*/

    handleinterrupt();
    interruptTriggered = false;

}
    cli();
    timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
    sei();
  }

    
  #ifndef DEBUG
      Serial.println(F("Sleep complete"));
  #endif
    next = false;
    do_send(&sendjob);
  }

#endif
}


/*
// ISR for PCINT0 (Port B pin change interrupt)
ISR(PCINT0_vect) {
  // Check if pin 10 triggered the interrupt
  if (bitRead(PINB, PINB2) == LOW) { // Check if pin 10 is LOW
    interruptTriggered = true; // Set flag
  }
}*/

ISR(PCINT0_vect) {

  static unsigned long lastInterruptTime;
  unsigned long interruptTime = millis();
  // Check if pin 10 triggered the interrupt
  if (bitRead(PINB, PINB2) == LOW &&  interruptTime - lastInterruptTime > 200) { // Check if pin 10 is LOW
    interruptTriggered = true; // Set flag
    lastInterruptTime = interruptTime; 
  }
}



void handleinterrupt()
{
    
    // Blink the LED
    for (int i = 0; i < 5; i++) {
      digitalWrite(PIN_LED, HIGH);
      delay(200);
      digitalWrite(PIN_LED, LOW);
      delay(200);
    }

    oledInit();
/*
    uint8_t buffer[1024];
    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "T");
    drawString(buffer, 12, 0, "e");
    drawString(buffer, 24, 0, "m");
    drawString(buffer, 36, 0, "p");
    updateDisplay(buffer); 
    delay(1000);     
*/
    //const char* num = number()/100;

    // Define a character array with enough space
    //char strArray[10];  // Size is 5 to include the null terminator

    // Copy the string literal into the character array
    //strcpy(strArray, "23.45");

    char deviceID[20];
    strcpy(deviceID, "012C1E92");

    char battery[100];
    strcpy(battery, "4.04");
    /*const char* num[10];
    for(int i = 0; i < 10;  i++)
    {
      num[i] = integer();
    }*/
/*    uint8_t buffer[1024];
    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "T");
    drawString(buffer, 12, 0, "e");
    drawString(buffer, 24, 0, "m");
    drawString(buffer, 36, 0, "p");
    drawString(buffer, 48, 0, ":");
    drawString(buffer, 72, 0, strArray);
    updateDisplay(buffer); 
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "D");
    drawString(buffer, 12, 0, "i");
    drawString(buffer, 24, 0, "s");
    drawString(buffer, 36, 0, "t");
    updateDisplay(buffer);
    delay(1000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "8");
    drawString(buffer, 12, 0, "9");
    drawString(buffer, 24, 0, "6");
    drawString(buffer, 36, 0, "7");
    updateDisplay(buffer);
    delay(1000);
*/

/*    uint8_t buffer[1024];
    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "D");
    drawString(buffer, 12, 0, "e");
    drawString(buffer, 24, 0, "v");
    drawString(buffer, 36, 0, "i");
    drawString(buffer, 48, 0, "c");
    drawString(buffer, 60, 0, "e");
    drawString(buffer, 72, 0, "I");
    drawString(buffer, 84, 0, "D");
    drawString(buffer, 96, 0, ":");
    drawString(buffer, 0, 48, "0");
    drawString(buffer, 12, 48, "1");
    drawString(buffer, 24, 48, "2");
    drawString(buffer, 36, 48, "C");
    drawString(buffer, 48, 48, "1");
    drawString(buffer, 60, 48, "E");
    drawString(buffer, 72, 48, "9");
    drawString(buffer, 84, 48, "2");
    updateDisplay(buffer); 
    delay(4000);



    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "B");
    drawString(buffer, 12, 0, "a");
    drawString(buffer, 24, 0, "t");
    drawString(buffer, 36, 0, "t");
    drawString(buffer, 48, 0, "e");
    drawString(buffer, 60, 0, "r");
    drawString(buffer, 72, 0, "y");
    drawString(buffer, 84, 0, ":");
    drawString(buffer, 0, 48, battery);
    drawString(buffer, 48, 48, "V");
    updateDisplay(buffer); 
    delay(4000);
    clearDisplay(buffer);*/
    Nitrogen();
    char strArr[10];
    char sourceString[10];
    itoa(nitrogen, sourceString, 10);
    strcpy(strArr, sourceString);

    uint8_t buffer[1024];
    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Device ID:");
    drawString(buffer, 0, 48, deviceID);
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Battery:");
    drawString(buffer, 0, 48, battery);
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Nitrogen:");
    drawString(buffer, 0, 48, strArr);
    updateDisplay(buffer);
    delay(4000);

/*    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Phosphorous:");
    drawString(buffer, 0, 48, battery);
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Pottasium:");
    drawString(buffer, 0, 48, battery);
    updateDisplay(buffer);
    delay(4000);
*/
    sendCommand(0xAE);
}

void hi()
{
  if(next == false)
    {
      digitalWrite(PIN_LED, HIGH);
    }
    else if(next == true)
    {
      digitalWrite(PIN_LED, LOW);
    }
}

/*
// Pin change interrupt service routine
ISR(PCINT0_vect) {
  // Debounce logic to avoid repeated triggers
  static unsigned long lastInterruptTime = 0;
  unsigned long interruptTime = millis();
  if (interruptTime - lastInterruptTime > 200) {  // Debounce delay (200ms)
    interruptTriggered = true;
  }
  lastInterruptTime = interruptTime;
}*/