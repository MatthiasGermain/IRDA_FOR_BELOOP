// irda.h
#ifndef IRDA_H
#define IRDA_H

void irda_init(int txPin, int rxPin);
void irda_send_packet(const char* data);
void irda_receive_packet(char* buffer, int length);
void irda_flush();

#endif

