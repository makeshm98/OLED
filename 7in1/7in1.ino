#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

#include <ModbusMaster.h>
BluetoothSerial SerialBT;


// Define Modbus constants
#define MODBUS_ADDRESS 0x01 // Device address
#define MODBUS_FUNCTION 0x03 // Function code for reading registers
#define CRC_POLYNOMIAL 0xA001

// Pin definitions for UART
#define TX_PIN 16 // Change as needed
#define RX_PIN 17 // Change as needed

HardwareSerial modbusSerial(2); // Use UART2 (ESP32 supports UART0, UART1, and UART2)

// Function to calculate CRC
uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= CRC_POLYNOMIAL;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// Function to send Modbus request
void sendModbusRequest(uint16_t startAddress, uint16_t numRegisters) {
  uint8_t request[8]; // Request frame

  request[0] = MODBUS_ADDRESS;            // Address code
  request[1] = MODBUS_FUNCTION;           // Function code
  request[2] = (startAddress >> 8) & 0xFF; // High byte of start address
  request[3] = startAddress & 0xFF;       // Low byte of start address
  request[4] = (numRegisters >> 8) & 0xFF; // High byte of number of registers
  request[5] = numRegisters & 0xFF;       // Low byte of number of registers

  // Calculate CRC
  uint16_t crc = calculateCRC(request, 6);
  request[6] = crc & 0xFF;               // CRC low byte
  request[7] = (crc >> 8) & 0xFF;        // CRC high byte

  // Send the request
  modbusSerial.write(request, 8);
}

// Function to read Modbus response
bool readModbusResponse(uint16_t *dataBuffer, uint8_t numRegisters) {
  uint8_t response[5 + numRegisters * 2]; // Response frame size
  uint8_t responseSize = 5 + numRegisters * 2;

  // Wait for the response
  uint32_t timeout = millis() + 4000; // 2-second timeout
  uint8_t index = 0;

  while (millis() < timeout) {
    if (modbusSerial.available()) {
      response[index++] = modbusSerial.read();
      if (index >= responseSize) {
        break;
      }
    }
  }

  // Validate response length
  if (index != responseSize) {
    return false; // Error: Incomplete response
  }

  // Validate CRC
  uint16_t crc = calculateCRC(response, responseSize - 2);
  if ((response[responseSize - 2] != (crc & 0xFF)) ||
      (response[responseSize - 1] != (crc >> 8))) {
    return false; // Error: CRC mismatch
  }

  // Parse data
  for (uint8_t i = 0; i < numRegisters; i++) {
    dataBuffer[i] = (response[3 + i * 2] << 8) | response[4 + i * 2];
  }

  return true; // Success
}

void setup() {
  // Initialize Serial ports
  //Serial.begin(115200);        // Debugging Serial Monitor
  modbusSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);  // UART2 Modbus Communication

  //Serial.println("Modbus RTU Example for ESP32");
  SerialBT.begin("ESP32test_7in1_soil_moisture"); 
}

/*void loop() {
  uint16_t dataBuffer[3]; // Buffer to hold data
  sendModbusRequest(0x001E, 3); // Request 3 registers starting at 0x001E

  delay(1000); // Wait for response

  if (readModbusResponse(dataBuffer, 3)) {
    // Successfully read data, print it
    SerialBT.print("Nitrogen: ");
    SerialBT.println(dataBuffer[0]);
    SerialBT.print("Phosphorus: ");
    SerialBT.println(dataBuffer[1]);
    SerialBT.print("Potassium: ");
    SerialBT.println(dataBuffer[2]);
  } else {
    // Error in communication
    SerialBT.println("Error reading Modbus data");
  }

  delay(2000); // Wait before the next request
}*/

void loop() {
  uint16_t dataBuffer[3]; // Buffer to hold data for up to 3 registers
  uint16_t registerAddresses[] = {0x0006, 0x0012, 0x0013, 0x0015, 0x001E, 0x001F, 0x0020};  //pH = 0x0006, moisture = 0x0012, temperature = 0x0013, EC = 0x0015, Nitrogen = 0x001E, Phosphorous = 0x001F, Pottassium = 0x0020
  uint8_t numRegisters = 1; // Number of registers to read at a time

  // Loop through each register address
  for (uint8_t i = 0; i < sizeof(registerAddresses) / sizeof(registerAddresses[0]); i++) {
    sendModbusRequest(registerAddresses[i], numRegisters); // Send request for 1 register

    delay(1000); // Wait for the response

    if (readModbusResponse(dataBuffer, numRegisters)) {
      // Successfully read data, print the value
      //SerialBT.print("Register 0x");
      //SerialBT.print(registerAddresses[i], HEX);
      //SerialBT.print(": ");
      //SerialBT.println(dataBuffer[0]);

      if(registerAddresses[i] == 0x6)
      {
        SerialBT.print("pH value: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x12)
      {
        SerialBT.print("Soil Moisture: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x13)
      {
        SerialBT.print("Soil Temperature: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x15)
      {
        SerialBT.print("Soil EC: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x1E)
      {
        SerialBT.print("Soil Nitrogen: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x1F)
      {
        SerialBT.print("Soil Phosphorous: ");
        SerialBT.println(dataBuffer[0]);  
      }

      if(registerAddresses[i] == 0x20)
      {
        SerialBT.print("Soil Pottassium: ");
        SerialBT.println(dataBuffer[0]);  
      }
    } else {
      // Error in communication
      SerialBT.print("Error reading register 0x");
      SerialBT.println(registerAddresses[i], HEX);
    }

    delay(2000); // Wait before the next request
  }
}

