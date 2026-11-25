#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t ch[16];
    bool link_up;
    uint32_t last_packet_ms;
} RC_Input_t;

void CRSF_ParseByte(RC_Input_t *rc, uint8_t byte, uint32_t now_ms);
bool CRSF_IsArmed(const RC_Input_t *rc, uint32_t now_ms);
bool CRSF_IsParachuteTriggered(const RC_Input_t *rc);
bool CRSF_IsAutoMode(const RC_Input_t *rc);

#endif