/*
 * main.h
 * contains global macros
 */
 
static const unsigned long DEFAULT_SYSTEM_CLOCK = 20485760U;
extern char str[100];

#define DEBUG_CAM 0

void delay(int del);

/* Used to debug camera processing */
void matlab_print(void);

