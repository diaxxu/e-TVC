#include "crsf.h"

void CRSF_ParseByte(RC_Input_t *rc, uint8_t byte, uint32_t now_ms) {
    static uint8_t buf[64];
    static uint8_t idx = 0;

    if (idx == 0 && byte != 0xC8) return;
    buf[idx++] = byte;

    if (idx >= 2 && idx == buf[1] + 2) {
        if (buf[2] == 0x16 && idx >= 26) {
            uint8_t *p = &buf[3];
            rc->ch[0]  = ((p[0]    | p[1] << 8) & 0x07FF);
            rc->ch[1]  = ((p[1]>>3 | p[2] << 5) & 0x07FF);
            rc->ch[2]  = ((p[2]>>6 | p[3] << 2 | p[4] << 10) & 0x07FF);
            rc->ch[3]  = ((p[4]>>1 | p[5] << 7) & 0x07FF);
            rc->ch[4]  = ((p[5]>>4 | p[6] << 4) & 0x07FF);
            rc->ch[5]  = ((p[6]>>7 | p[7] << 1 | p[8] << 9) & 0x07FF);
            rc->ch[6]  = ((p[8]>>2 | p[9] << 6) & 0x07FF);
            rc->ch[7]  = ((p[9]>>5 | p[10]<< 3) & 0x07FF);
            rc->link_up = true;
            rc->last_packet_ms = now_ms;
        }
        idx = 0;
    }
    if (idx >= sizeof(buf)) idx = 0;
}

bool CRSF_IsArmed(const RC_Input_t *rc, uint32_t now_ms) {
    if (!rc->link_up || (now_ms - rc->last_packet_ms > 250)) return false;
    return (rc->ch[4] > 1500);
}

bool CRSF_IsParachuteTriggered(const RC_Input_t *rc) {
    return (rc->ch[5] > 1500);
}

bool CRSF_IsAutoMode(const RC_Input_t *rc) {
    return (rc->ch[6] > 1500);
}