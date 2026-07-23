#include <ArduinoBLE.h>


#define DEVICE_NAME "BiWheelBot" 


uint32_t lastCmdMs = 0;


struct Target { int8_t linear; int8_t angular; uint8_t flags; uint8_t seq; };
Target target = {0, 0, 0, 0};

// Custom service + characteristic for movement commands
BLEService cmdService("19b10000-e8f2-537e-4f6c-d104768a1214");
// 4 bytes: [linear_vel_i8, angular_vel_i8, flags_u8, seq_u8]
BLECharacteristic cmdChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLEWriteWithoutResponse, 4);


void onCmdWritten(BLEDevice central, BLECharacteristic ch) {
  const uint8_t* d = ch.value();
  target = { (int8_t)d[0], (int8_t)d[1], d[2], d[3] };
  lastCmdMs = millis();
}


void init_ble() {
    if (!BLE.begin()) { while (1); }
    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(cmdService);
    cmdService.addCharacteristic(cmdChar);
    BLE.addService(cmdService);
    cmdChar.setEventHandler(BLEWritten, onCmdWritten);
    BLE.advertise();
}

void com_poll() {
    BLE.poll();
}