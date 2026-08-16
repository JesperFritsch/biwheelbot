#include <ArduinoBLE.h>

// Not part of ArduinoBLE's documented API -- setMaxMtu lives on the internal
// ATTClass, reachable only through `extern ATTClass& ATT;`. Needed because the
// default MTU of 23 caps a notification at 20 bytes, and telemetry is 48.
// Pin the library version in platformio.ini while depending on this.
#include "utility/ATT.h"

#include "com.h"
#include "types.h"
#define DEVICE_NAME "BiWheelBot"

// Gives 93 bytes of notification payload (mtu - 3) and 95 for a read response
// (mtu - 1), so both the telemetry packet and its schema string fit in a single
// round trip with room to grow.
//
// Constraint before raising this further, or adding encrypted characteristics:
// ATT.h:102 declares `uint8_t holdBuffer[64]`, the only buffer in the library
// not sized from the negotiated MTU. It is written only on the encryption-hold
// path -- a characteristic declared with BLEPermission::BLEEncryption being read
// before the link is encrypted. None of ours set permissions, so that path is
// unreachable here; if that ever changes, this must come back down to 64.
#define MAX_MTU 96


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

// Telemetry lives in its own service: gaintui treats every 0x2901-carrying
// characteristic in configService as an editable gain block, and this one is
// neither editable nor three floats.
BLEService telemetryService("19b10006-e8f2-537e-4f6c-d104768a1214");

// TELEM_FIELDS little-endian f32, in the order named by the schema below.
// The names and the pack order in main.cpp's loop() must stay in step -- the
// wire format carries no field identifiers, only the schema does.
BLECharacteristic telemetryChar("19b10007-e8f2-537e-4f6c-d104768a1214",
                                BLERead | BLENotify, TELEM_FIELDS * sizeof(float));
BLEDescriptor telemetryDesc("2901", "telem:ang,rate,kfa,cmp,tpos,pos,tang,tspd,eff,spd,duty,bat,en,ovr");

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
    // Must precede begin(): the ceiling is applied when a central negotiates,
    // and everything downstream is already sized from the agreed value.
    ATT.setMaxMtu(MAX_MTU);

    if (!BLE.begin()) { while (1); }
    BLE.setLocalName(DEVICE_NAME);
    BLE.setAdvertisedService(cmdService);
    BLE.setAdvertisedService(configService);
    // Telemetry is deliberately not advertised -- an advertising packet holds
    // only one 128-bit UUID anyway, and gaintui matches on the device name and
    // then discovers every service over the connection.

    // Descriptors and characteristics must be attached before addService():
    // GATT flattens the service into a handle table at that point and never
    // revisits it.
    balancePIDGains.addDescriptor(balanceDesc);
    speedPIDGains.addDescriptor(speedDesc);
    posPIDGains.addDescriptor(posDesc);
    telemetryChar.addDescriptor(telemetryDesc);

    cmdService.addCharacteristic(cmdChar);
    configService.addCharacteristic(balancePIDGains);
    configService.addCharacteristic(speedPIDGains);
    configService.addCharacteristic(posPIDGains);
    telemetryService.addCharacteristic(telemetryChar);

    BLE.addService(cmdService);
    BLE.addService(configService);
    BLE.addService(telemetryService);
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
    // A zero-length value makes a read fail with INVALID_OFFSET before the
    // handler ever runs, so every readable characteristic needs a first value.
    seed_char(telemetryChar);

    // Notifications can only go out as often as the connection interval allows.
    BLE.setConnectionInterval(0x0006, 0x0010);   // 7.5 ms .. 20 ms

    BLE.advertise();
}

// Call from loop(), never from the control thread -- ArduinoBLE is not thread
// safe and com_poll() already drives the stack from the main thread.
void com_publish_telemetry(const float *values, int count) {
    if (count != TELEM_FIELDS) return;   // schema and payload must agree
    telemetryChar.writeValue((const uint8_t *)values, count * sizeof(float));
}

void com_poll() {
    BLE.poll();
}