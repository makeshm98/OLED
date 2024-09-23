#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <ModbusMaster.h>

#define OLED_DC     8   // Data/Command pin
//#define OLED_DC     7
#define OLED_CS    10  // Chip Select pin
#define OLED_RST    7   // Reset pin 

#define PIN_INTERRUPT 10  // Digital pin 10
#define PIN_LED 9        // Digital pin 9 (LED)

volatile bool interruptTriggered = false;
uint8_t textsize_x = 2; 
uint8_t textsize_y = 2;

ModbusMaster mode;
int nitrogen, phosphorus, potassium ,err=0 ,Temperature ,moisture ,EC ,PH ;
int sensor_pin = 7;
void npk()
{
 
 
uint8_t result = mode.readHoldingRegisters(0x001E, 7);  // Start address, number of registers

  // Check if reading was successful
  if (result == mode.ku8MBSuccess) {
    // Read the received data
    nitrogen = mode.getResponseBuffer(0);
    phosphorus = mode.getResponseBuffer(1);
    potassium = mode.getResponseBuffer(2);
    Temperature = mode.getResponseBuffer(3);
    moisture = mode.getResponseBuffer(4);
    EC = mode.getResponseBuffer(5);
    PH = mode.getResponseBuffer(6);
    err=0;
   

    // Print the values to Serial Monitor
//    Serial.print("Nitrogen: ");
//    Serial.println(nitrogen);
//    Serial.print("Phosphorus: ");
//    Serial.println(phosphorus);
//    Serial.print("Potassium: ");
//    Serial.println(potassium);
  } else {
    // Print error message and error code
//    Serial.print("Error reading data. Error code: ");
//    Serial.println(result);
      err=1;
  }

  // Delay before next read
  delay(1000);  // Adjust as per your application's requirement  
}

static const PROGMEM u1_t NWKSKEY[16] = { 0xBA, 0xE8, 0x30, 0xAB, 0x85, 0x45, 0xC7, 0x87, 0x08, 0x5A, 0x83, 0x3F, 0xA0, 0x1B, 0x7A, 0x6F };

static const u1_t PROGMEM APPSKEY[16] = { 0xF2, 0x3B, 0x13, 0xC3, 0x48, 0x93, 0x4A, 0x91, 0xD4, 0x27, 0x94, 0x56, 0x62, 0x2D, 0x2D, 0xAC };

static const u4_t DEVADDR = 0x00F2C609; // <-- Change this address for every node!



// show debug statements; comment next line to disable debug statements
#define DEBUG

// use low power sleep; comment next line to not use low power sleep
#define SLEEP

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 900;

//struct {
//int a;
//int16_t b;
//} mydata;

#ifdef SLEEP
#include "LowPower.h"
bool next = false;
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

static uint8_t mydata[100];

// Pin mapping
const lmic_pinmap lmic_pins = {
.nss = 6,
.rxtx = LMIC_UNUSED_PIN,
.rst = 5,
.dio = {2, 3, 4},
};

void onEvent (ev_t ev) {
#ifdef DEBUG
//Serial.println(F("Enter onEvent"));
#endif

switch (ev) {
#ifdef DEBUG
case EV_SCAN_TIMEOUT:
//Serial.println(F("EV_SCAN_TIMEOUT"));
break;
case EV_BEACON_FOUND:
//Serial.println(F("EV_BEACON_FOUND"));
break;
case EV_BEACON_MISSED:
//Serial.println(F("EV_BEACON_MISSED"));
break;
case EV_BEACON_TRACKED:
//Serial.println(F("EV_BEACON_TRACKED"));
break;
case EV_JOINING:
//Serial.println(F("EV_JOINING"));
break;
case EV_JOINED:
//Serial.println(F("EV_JOINED"));
break;
case EV_RFU1:
//Serial.println(F("EV_RFU1"));
break;
case EV_JOIN_FAILED:
//Serial.println(F("EV_JOIN_FAILED"));
break;
case EV_REJOIN_FAILED:
//Serial.println(F("EV_REJOIN_FAILED"));
break;
#endif
case EV_TXCOMPLETE:
//Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
if (LMIC.dataLen) {
// data received in rx slot after tx
//Serial.print(F("Data Received: "));
//Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
//Serial.println();
}
// Schedule next transmission
#ifndef SLEEP
os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
#else
next = true;
#endif

break;

#ifdef DEBUG
case EV_LOST_TSYNC:
//Serial.println(F("EV_LOST_TSYNC"));
break;
case EV_RESET:
//Serial.println(F("EV_RESET"));
break;
case EV_RXCOMPLETE:
// data received in ping slot
//Serial.println(F("EV_RXCOMPLETE"));
break;
case EV_LINK_DEAD:
//Serial.println(F("EV_LINK_DEAD"));
break;
case EV_LINK_ALIVE:
//Serial.println(F("EV_LINK_ALIVE"));
break;
default:
//Serial.println(F("Unknown event"));
break;
#endif
}
#ifdef DEBUG
//Serial.println(F("Leave onEvent"));
#endif

}

void do_send(osjob_t* j) {

int a = batterycalc();
digitalWrite(sensor_pin, HIGH);
//delay(3000);
npk();
npk();
npk();
npk();
npk();
     
      mydata[0] = highByte(nitrogen);
      mydata[1] = lowByte(nitrogen);
     
      mydata[2] = highByte(phosphorus);
      mydata[3] = lowByte(phosphorus);

      mydata[4] = highByte(potassium);
      mydata[5] = lowByte(potassium);

      mydata[10] = highByte(Temperature);
      mydata[11] = lowByte(Temperature);

      mydata[12] = highByte(moisture);
      mydata[13] = lowByte(moisture);

      mydata[14] = highByte(EC);
      mydata[15] = lowByte(EC);

      mydata[16] = highByte(PH);
      mydata[17] = lowByte(PH);

      mydata[6] = highByte(err);
      mydata[7] = lowByte(err);

      mydata[8] = highByte(a);
      mydata[9] = lowByte(a);
digitalWrite(sensor_pin, LOW);
//mydata.b = soil_moisture();
#ifdef DEBUG
//Serial.println(F("Enter do_send"));
#endif

// Check if there is not a current TX/RX job running
if (LMIC.opmode & OP_TXRXPEND) {
//Serial.println(F("OP_TXRXPEND, not sending"));
} else {
// Prepare upstream data transmission at the next possible time.
LMIC_setTxData2(1, (unsigned char *)&mydata, sizeof(mydata) - 1, 0);
//Serial.println(F("Packet queued"));
}
// Next TX is scheduled after TX_COMPLETE event.
#ifdef DEBUG
//Serial.println(F("Leave do_send"));
#endif

}

//Soil Moisture
//int sensor_pin = A5, moisture;
//const int sensorEn = 8;
//
//int soil_moisture()
//{
//digitalWrite(sensorEn, HIGH);
//delay(200);
//moisture = analogRead(sensor_pin);
//moisture = map(moisture, 850, 350, 0, 100);
//digitalWrite(sensorEn, LOW);
//#ifdef DEBUG
//Serial.println("Moisture = ");
//Serial.print(moisture);
//Serial.print("%");
//#endif
//return moisture;
//}

//Battery Monitor
int batv = 0, bat_status = 0;

int batteryv()
{

int val = analogRead(A3) >> 2; // read the input pin
batv = (val * 2.0 * 3.3 / 255) * 1000; //1.85 is the voltage multiplier
delay(0);
return batv;
}

int batterycalc()
{
int diff = 0;
if (bat_status == 0) { //Baseline calibration
for (int j = 0; j < 5; j++) {
bat_status = bat_status + batteryv();
}
bat_status = bat_status / 5;
}
#ifdef DEBUG
//Serial.println("Battery_status = ");
//Serial.print(bat_status);
#endif
for (int i = 0; i < 5; i++) { //Read current average
batv = batv + batteryv();
}
batv = batv / 5;
diff = abs(bat_status - batv);
#ifdef DEBUG
//Serial.println("Now Avg = ");
//Serial.println(batv);
//Serial.println("Value diff");
//Serial.println(diff);
#endif
if (diff > 25 && diff < 70) { //Define Band Pass
bat_status = batv;
}
batv = 0;
#ifdef DEBUG
//Serial.println("Battery Voltage");
//Serial.println(bat_status);
#endif
return bat_status;
}

void Nitrogen()
{
  nitrogen = 98;
}

void setup() {
  while (!Serial);
  Serial.begin(9600);
  mode.begin(1, Serial);
    delay(100);

//Serial.print("Starting..");
//delay(5000);
pinMode(sensor_pin, OUTPUT);
//Serial.println(F("Enter setup"));
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

  PCICR |= (1 << PCIE0);  
  PCMSK0 |= (1 << PCINT2);

#ifdef VCC_ENABLE
// For Pinoccio Scout boards
pinMode(VCC_ENABLE, OUTPUT);
digitalWrite(VCC_ENABLE, HIGH);
delay(1000);
#endif
// LMIC init
os_init();
// Reset the MAC state. Session and pending data transfers will be discarded.
LMIC_reset();

// Set static session parameters. Instead of dynamically establishing a session
// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
// On AVR, these values are stored in flash and only copied to RAM
// once. Copy them to a temporary buffer here, LMIC_setSession will
// copy them into a buffer of its own again.
uint8_t appskey[sizeof(APPSKEY)];
uint8_t nwkskey[sizeof(NWKSKEY)];
memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
LMIC_setSession (0x13, DEVADDR, nwkskey, appskey);
#else
// If not running an AVR with PROGMEM, just use the arrays directly
LMIC_setSession (0x13, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
LMIC_setupChannel(0, 865062500, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
LMIC_setupChannel(1, 865402500, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI); // g-band
LMIC_setupChannel(2, 865985000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI); // g-band
#elif defined(CFG_us915)
LMIC_selectSubBand(1);
#endif

// Disable link check validation
LMIC_setLinkCheckMode(0);

// TTN uses SF9 for its RX2 window.
LMIC.dn2Dr = DR_SF9;

// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
LMIC_setDrTxpow(DR_SF7, 14);

// Start job
do_send(&sendjob);
// Wait a maximum of 10s for Serial Monitor
// while (!debugSerial && millis() < 10000);

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
} else {
  hi();
  digitalWrite(PIN_LED, LOW);
int sleepcycles = TX_INTERVAL / 8; // calculate the number of sleepcycles (8s) given the TX_INTERVAL
#ifdef DEBUG
//Serial.print(F("Enter sleeping for "));
//Serial.print(sleepcycles);
//Serial.println(F(" cycles of 8 seconds"));
#endif
//Serial.flush(); // give the serial print chance to complete
for (int i = 0; i < sleepcycles; i++) {
// Enter power down state for 8 s with ADC and BOD module disabled
LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
  if (interruptTriggered) {
    handleinterrupt();
    interruptTriggered = false;

}
//LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF);

// LMIC uses micros() to keep track of the duty cycle, so
// hack timer0_overflow for a rude adjustment:
cli();
timer0_overflow_count += 8 * 64 * clockCyclesPerMicrosecond();
sei();
}
#ifdef DEBUG
//Serial.println(F("Sleep complete"));
#endif
next = false;
// Start job
do_send(&sendjob);
}

#endif

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

    char deviceID[20];
    strcpy(deviceID, "012C1E92");
    Nitrogen();
    char strArr[10];
    uint8_t buffer[1024];
    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Nitrogen:");
    drawString(buffer, 0, 48, strArr);
    updateDisplay(buffer);
    delay(4000);
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

ISR(PCINT0_vect) {

  static unsigned long lastInterruptTime;
  unsigned long interruptTime = millis();
  // Check if pin 10 triggered the interrupt
  if (bitRead(PINB, PINB2) == LOW &&  interruptTime - lastInterruptTime > 200) { // Check if pin 10 is LOW
    interruptTriggered = true; // Set flag
    lastInterruptTime = interruptTime; 
  }
}