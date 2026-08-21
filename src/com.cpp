#include <ArduinoBLE.h>

#include "utility/ATT.h"

#include "com.h"
#include "types.h"
#define DEVICE_NAME "BiWheelBot"

#define MAX_MTU 96


static const uint8_t ZEROS[512] = {0};

// The only advertised service, and deliberately empty: it exists to be an
// identity token, not to carry data. An advertising packet has room for exactly
// one 128-bit UUID, so advertising a functional service would mean picking a
// favourite among cmd/gains/telemetry and quietly dropping the rest -- and the
// host's scan filter would then have to change every time the service layout
// did. This UUID never changes; everything behind it is free to.
BLEService idService("19b1000a-e8f2-537e-4f6c-d104768a1214");

BLEService cmdService("19b10000-e8f2-537e-4f6c-d104768a1214");
BLEService gainsService("19b10002-e8f2-537e-4f6c-d104768a1214");

// 4 bytes: [linear_vel_i8, angular_vel_i8, flags_u8, seq_u8]
BLECharacteristic cmdChar("19b10001-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLEWriteWithoutResponse, 4);

// One row per tunable PID block. Adding a loop is this row plus its GainId and
// its defaults in control.cpp -- nothing else in this file needs touching, which
// is the point: the previous hand-rolled version had four separate places per
// gain to keep in step, and the turn block silently missed one of them.
//
// Both string fields must have static storage duration. ArduinoBLE stores them
// by pointer and never copies: BLEUuid keeps `const char* _str`, and
// BLELocalDescriptor keeps `const uint8_t* _value`. String literals qualify;
// anything built into a buffer at init time would dangle.
//
// Value payload is 12 bytes: [kp_f32, ki_f32, kd_f32].
struct GainChar {
    GainId            id;
    BLECharacteristic ch;
    BLEDescriptor     desc;
};

static GainChar gain_chars[] = {
    { GAIN_BALANCE, {"19b10003-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12}, {"2901", "balance:kp,ki,kd"}  },
    { GAIN_SPEED,   {"19b10004-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12}, {"2901", "speed:kp,ki,kd"}    },
    { GAIN_POS,     {"19b10005-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12}, {"2901", "position:kp,ki,kd"} },
    { GAIN_TURN,    {"19b10008-e8f2-537e-4f6c-d104768a1214", BLEWrite | BLERead, 12}, {"2901", "turn:kp,ki,kd"}     },
};

// Telemetry lives in its own service: gaintui treats every 0x2901-carrying
// characteristic in gainsService as an editable gain block, and this one is
// neither editable nor three floats.
BLEService telemetryService("19b10006-e8f2-537e-4f6c-d104768a1214");

// TELEM_FIELDS little-endian f32, in the order named by the schema below.
// The names and the pack order in main.cpp's loop() must stay in step -- the
// wire format carries no field identifiers, only the schema does.
BLECharacteristic telemetryChar("19b10007-e8f2-537e-4f6c-d104768a1214",
                                BLERead | BLENotify, TELEM_FIELDS * sizeof(float));
BLEDescriptor telemetryDesc("2901", "telem:ang,rate,kfa,tpos,pos,tang,tspd,eff,spd,d_a,d_b,t_d,bat,en,ovr");

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

bool decode_drive_cmd(BLECharacteristic &ch, DriveCmd &cmd) {
    if (ch.valueLength() != 4) return false;
    memcpy(&cmd, ch.value(), 4);
    return true;
}

void on_drive_written(BLEDevice central, BLECharacteristic ch) {
    DriveCmd cmd;
    if (!decode_drive_cmd(ch, cmd)) return;
    if (!com_hooks.set_drive) return;
    com_hooks.set_drive(cmd);
}

// The handler receives a BLECharacteristic by value -- a fresh wrapper around
// the same BLELocalCharacteristic -- so the UUID string is the identity we have
// to match on.
static GainChar* find_gain_char(const char* uuid) {
    for (auto& g : gain_chars) {
        if (strcmp(uuid, g.ch.uuid()) == 0) return &g;
    }
    return nullptr;
}

void on_gain_char_written(BLEDevice central, BLECharacteristic ch) {
    GainChar* g = find_gain_char(ch.uuid());
    if (!g || !com_hooks.set_gains) return;
    PIDGains gains;
    if (!decode_gains(ch, gains)) return;
    com_hooks.set_gains(g->id, gains);
}

void on_gain_char_read(BLEDevice central, BLECharacteristic ch) {
    GainChar* g = find_gain_char(ch.uuid());
    if (!g || !com_hooks.get_gains) return;
    PIDGains gains = com_hooks.get_gains(g->id);
    ch.writeValue((uint8_t*)&gains, sizeof(gains));
}

void init_ble() {
    // Must precede begin(): the ceiling is applied when a central negotiates,
    // and everything downstream is already sized from the agreed value.
    ATT.setMaxMtu(MAX_MTU);

    if (!BLE.begin()) { while (1); }
    BLE.setLocalName(DEVICE_NAME);
    // setAdvertisedService is a setter, not an append: only the last call
    // survives. Nothing but idService is advertised, and nothing needs to be --
    // a central connects on the identity match and then discovers cmd, gains
    // and telemetry over the connection.
    BLE.setAdvertisedService(idService);

    // Descriptors and characteristics must be attached before addService():
    // GATT flattens the service into a handle table at that point and never
    // revisits it.
    for (auto& g : gain_chars) {
        g.ch.addDescriptor(g.desc);
        gainsService.addCharacteristic(g.ch);
    }
    telemetryChar.addDescriptor(telemetryDesc);

    cmdService.addCharacteristic(cmdChar);
    telemetryService.addCharacteristic(telemetryChar);

    // Registered with no characteristics so the advertisement is not lying:
    // a central that connects and walks the GATT table actually finds the
    // service it matched on.
    BLE.addService(idService);

    BLE.addService(cmdService);
    BLE.addService(gainsService);
    BLE.addService(telemetryService);

    cmdChar.setEventHandler(BLEWritten, on_drive_written);

    for (auto& g : gain_chars) {
        g.ch.setEventHandler(BLEWritten, on_gain_char_written);
        g.ch.setEventHandler(BLERead, on_gain_char_read);
        seed_char(g.ch);
    }
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