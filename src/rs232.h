#ifndef __RS232_H__
#define __RS232_H__

#include <stdint.h>
#include "libmfprot.h"

int rs232_config_gpio(void);
int rs232_recv_byte(mfprot_device dev, uint8_t *byte_out);
int rs232_recv_data(mfprot_device dev, void *ptr, int len);
int rs232_send_byte(mfprot_device dev, char *buf, int len);

#endif
