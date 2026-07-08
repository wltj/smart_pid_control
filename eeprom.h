#ifndef __EEPROM_H
#define __EEPROM_H

int read_params(unsigned char *buf,int len);  //从eeprom中读取数据
int save_params(unsigned char *buf,int len);  //保存数据到eeprom

#endif
