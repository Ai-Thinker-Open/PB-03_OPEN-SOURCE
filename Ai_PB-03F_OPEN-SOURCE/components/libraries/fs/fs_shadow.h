
#ifndef __FS_SHADOW_H__
#define __FS_SHADOW_H__

int fss_assert(uint16_t id); //for debug, trigger crc error


int fss_read(uint16_t id, uint16_t len, void* pbuf);
int fss_write(uint16_t id, uint16_t len, uint8_t* pbuf);
int fss_del(uint16_t id);


#endif /*__FS_SHADOW_H__*/

