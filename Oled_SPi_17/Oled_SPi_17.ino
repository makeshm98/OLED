#include <SPI.h>
#include <avr/pgmspace.h>

#define OLED_DC     8   // Data/Command pin
//#define OLED_DC     7
#define OLED_CS     10  // Chip Select pin
#define OLED_RST    7   // Reset pin 

//3 on SCK 4 on MOSI 5 on RST 6 on 8 and 7 on 10

//uint8_t buffer[1024]; // 128x64 / 8 = 1024 bytes
uint8_t textsize_x = 2; // Text magnification factor in X direction
uint8_t textsize_y = 2; // Text magnification factor in Y direction

void setup() {
  
    // Set pin modes
    pinMode(OLED_DC, OUTPUT);
    pinMode(OLED_CS, OUTPUT);
    pinMode(OLED_RST, OUTPUT);

    // Start SPI communication
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV2); // Set SPI clock speed

    // Initialize the OLED
    oledInit();
}

void oledInit() {
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
}

void sendCommand(uint8_t command) {
    digitalWrite(OLED_DC, LOW);  // Command mode
    digitalWrite(OLED_CS, LOW);  // Select the OLED
    SPI.transfer(command);       // Send command
    digitalWrite(OLED_CS, HIGH); // Deselect the OLED
}

void sendData(uint8_t data) {
    digitalWrite(OLED_DC, HIGH); // Data mode
    digitalWrite(OLED_CS, LOW);  // Select the OLED
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
    {0x44, 0x64, 0x54, 0x4C, 0x44},  // z
    {0x00, 0x00, 0x14, 0x00, 0x00}   // :
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

    {0x00, 0x00, 0x14, 0x00, 0x00},  //:

    //{0x20, 0x10, 0x08, 0x04, 0x02}   // /

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
        charData = digits[c - '0'];
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

        x += 7 * textsize_x; // Move to the next character position (5 pixels wide)
        str++;
    }
}

void loop() {

  uint8_t buffer[1024];
    clearDisplay(buffer); 
    drawString(buffer, 0, 0, "T");
    drawString(buffer, 12, 0, "e");
    drawString(buffer, 24, 0, "m");
    drawString(buffer, 36, 0, "p");
    updateDisplay(buffer); 
    delay(4000);     

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "D");
    drawString(buffer, 12, 0, "i");
    drawString(buffer, 24, 0, "s");
    drawString(buffer, 36, 0, "t");
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "8");
    drawString(buffer, 12, 0, "9");
    drawString(buffer, 24, 0, "6");
    drawString(buffer, 36, 0, "7");
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "Hello World!");
    updateDisplay(buffer);
    delay(4000);

    clearDisplay(buffer);
    drawString(buffer, 0, 0, "2345");
    updateDisplay(buffer);
    delay(4000);

    sendCommand(0xAE); // Display OFF (sleep mode)

}
