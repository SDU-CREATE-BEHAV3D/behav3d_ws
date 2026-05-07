// --- Modbus map for controlled steps queue ---
// Coils:
//  13 -> QUEUE_PUSH (rising edge enqueues a 32-bit step count from HR3|HR4)
//  14 -> CANCEL_ALL (cancel current job and clear queue)
// Holding Registers:
//  HR3 -> steps_lo (lower 16 bits of 32-bit step count)
//  HR4 -> steps_hi (upper 16 bits of 32-bit step count)
// Discrete/Input registers could later expose queue size / busy flags if needed.

union {
  float asFloat;
  uint16_t asInt[2];
} holding_float;

void setupModbus() {
  for (int i = 0; i < NUMBER_INPUTS_COILS; i++) {
    pinMode(CONTROLLINO_D0 + i, OUTPUT); // Set digital output pins (R0 to R10) as OUTPUT  
    digitalWrite(CONTROLLINO_D0 + i, LOW); // Initialize output pins to LOW (off state)  
  }
  
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet
  
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1);
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  Serial.println("Starting Server...");
  ethernetServer.begin();
  if (!modbusTCPServer.begin()) {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }
  
  // Configure the Modbus Server's coil and register mapping  
  modbusTCPServer.configureCoils(0x00, NUMBER_INPUTS_COILS); // Set up coils starting at address 0 with the specified number  
  modbusTCPServer.configureDiscreteInputs(0x00, NUMBER_INPUTS_COILS); // Configure discrete inputs for monitoring  
  modbusTCPServer.configureHoldingRegisters(0x00, NUMBER_IN_HOLD_REGISTERS); // Set up holding registers for PWM outputs  
  modbusTCPServer.configureInputRegisters(0x00, NUMBER_IN_HOLD_REGISTERS); // Configure input registers for analog readings  
}