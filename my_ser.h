#ifndef XC_MY_SER_H
#define	XC_MY_SER_H

#include <xc.h> // include processor files - each processor file is guarded.  

void setupSerial(void);
unsigned char is_byte_available(void);//no
unsigned char read_byte_no_lib(void);//no busy waiting
void send_byte_no_lib(unsigned char c);
void send_string_no_lib(unsigned char *p);

#endif	/*  XC_MY_SER_H */

