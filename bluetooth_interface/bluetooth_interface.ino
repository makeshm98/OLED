void setup() {
  Serial.begin(9600);                 
  Serial1.begin(9600);                
  Serial.println("Bluetooth AT-09 Module Ready");
}

void loop() {

  String message = "Hello World...!!";
  Serial1.println(message);

  // Check if data is available from the Bluetooth module
  if (Serial1.available()) {
    char btData = Serial1.read();     // Read a character from Bluetooth
    Serial.print("Received from Bluetooth: ");
    Serial.println(btData);           // Print it to Serial Monitor
  }

  // Check if data is available from Serial Monitor (PC)
  if (Serial.available()) {
    char data = Serial.read();        // Read a character from Serial Monitor
    Serial1.print(data);              // Send it to Bluetooth module
    Serial.print("Sent to Bluetooth: ");
    Serial.println(data);             // Print to Serial Monitor for feedback
  }
  delay(5000);

}
