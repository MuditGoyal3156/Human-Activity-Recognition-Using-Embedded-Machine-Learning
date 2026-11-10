#ifndef RF_H
#define RF_H

#include <stdint.h>
#include <stdbool.h>

#define RH_ASK_MAX_MESSAGE_LEN 60
#define RH_ASK_MAX_PAYLOAD_LEN 67
#define RH_ASK_HEADER_LEN      4

bool rf_init(uint16_t speed_bps);
bool rf_recv(uint8_t *buf, uint8_t *len);

#endif /* RF_H */
