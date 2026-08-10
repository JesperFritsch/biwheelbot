#include <ArduinoBLE.h>

#include "com.h"
#include "types.h"
#define DEVICE_NAME "BiWheelBot" 


static const uint8_t ZEROS[512] = {0};

uint32_t lastCmdMs = 0;


struct Target { int8_t linear; int8_t angular; uint8_t flags; uint8_t seq; };
Target target = {0, 0, 0, 0};

BLEService cmdService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEService configService("19b10002-e8f2-537e-4f6c-d104768a1214");

// 4 bytes: [linear_vel_i8, angular_vel_i8, flags_u8, seq_u8]
BLECharacteristic cmdChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLEWriteWithoutResponse, 4);

// 12 bytes: [kp_f32, ki_f32, kd_f32]
BLECharacteristic balancePIDGains("19b10003-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12);
BLEDescriptor balanceDesc("2901", "balance:kp,ki,kd");

BLECharacteristic speedPIDGains("19b10004-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12);
BLEDescriptor speedDesc("2901", "speed:kp,ki,kd");

BLECharacteristic posPIDGains("19b10005-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12);
BLEDescriptor posDesc("2901", "position:kp,ki,kd");

static ComHooks com_hooks = {};

static void seed_char(BLECharacteristic &ch) {
    int size = ch.valueSize();
    ch.writeValue(ZEROS, size);
}

void com_set_hooks(ComHooks hooks) {
    com_hooks = hooks;
}

bool decode_gains(BLECharacteristic &ch, PIDGains &gains) {
    if (ch.valueLength() != 12) return false;
    float v[3];
    memcpy(v, ch.value(), 12);
    for (int i = 0; i < 3; i++) {
        if (!isfinite(v[i])) return false;
    }
    gains.kp = v[0];
    gains.ki = v[1];
    gains.kd = v[2];
    return true;
}

void on_char_written(BLEDevice central, BLECharacteristic ch) {
    PIDGains gains;
    if (!decode_gains(ch, gains)) return;
    if (strcmp(ch.uuid(), balancePIDGains.uuid()) == 0) {
        if (com_hooks.set_balance_gains) com_hooks.set_balance_gains(gains);
    } else if (strcmp(ch.uuid(), speedPIDGains.uuid()) == 0) {
        if (com_hooks.set_speed_gains) com_hooks.set_speed_gains(gains);
    } else if (strcmp(ch.uuid(), cmdChar.uuid()) == 0) {
        // ignore, handled in on_cmd_written
    } else if (strcmp(ch.uuid(), posPIDGains.uuid()) == 0) {
        if (com_hooks.set_pos_gains) com_hooks.set_pos_gains(gains);
    }
}

void on_char_read(BLEDevice central, BLECharacteristic ch) {
    if (strcmp(ch.uuid(), balancePIDGains.uuid()) == 0) {
        if (com_hooks.get_balance_gains) {
            PIDGains gains = com_hooks.get_balance_gains();
            ch.writeValue((uint8_t*)&gains, sizeof(gains));
        }
    } else if (strcmp(ch.uuid(), speedPIDGains.uuid()) == 0) {
        if (com_hooks.get_speed_gains) {
            PIDGains gains = com_hooks.get_speed_gains();
            ch.writeValue((uint8_t*)&gains, sizeof(gains));
        }
    } else if (strcmp(ch.uuid(), posPIDGains.uuid()) == 0) {
        if (com_hooks.get_pos_gains) {
            PIDGains gains = com_hooks.get_pos_gains();
            ch.writeValue((uint8_t*)&gains, sizeof(gains));
        }
    }
}

void init_ble() {
    if (!BLE.begin()) { while (1); }
    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(cmdService);
    BLE.setAdvertisedService(configService);

    balancePIDGains.addDescriptor(balanceDesc);
    speedPIDGains.addDescriptor(speedDesc);
    posPIDGains.addDescriptor(posDesc);

    cmdService.addCharacteristic(cmdChar);
    configService.addCharacteristic(balancePIDGains);
    configService.addCharacteristic(speedPIDGains);
    configService.addCharacteristic(posPIDGains);

    BLE.addService(cmdService);
    BLE.addService(configService);
    cmdChar.setEventHandler(BLEWritten, on_char_written);
    balancePIDGains.setEventHandler(BLEWritten, on_char_written);
    speedPIDGains.setEventHandler(BLEWritten, on_char_written);
    posPIDGains.setEventHandler(BLEWritten, on_char_written);
    balancePIDGains.setEventHandler(BLERead, on_char_read);
    speedPIDGains.setEventHandler(BLERead, on_char_read);
    posPIDGains.setEventHandler(BLERead, on_char_read);
    seed_char(balancePIDGains);
    seed_char(speedPIDGains);
    seed_char(posPIDGains);
    BLE.advertise();
}

void com_poll() {
    BLE.poll();
}