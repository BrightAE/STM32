#ifndef __OUTPUTDATA_H
#define __OUTPUTDATA_H

extern float OutData[4];

unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
void Output_Data(void);

#endif
