#pragma once
#include "types.h"

struct ComHooks {
    void (*set_balance_gains)(PIDGains);
    PIDGains (*get_balance_gains)();
    void (*set_speed_gains)(PIDGains);
    PIDGains (*get_speed_gains)();
    void (*set_pos_gains)(PIDGains);
    PIDGains (*get_pos_gains)();
};

void com_set_hooks(ComHooks hooks);
void init_ble();
void com_poll();
