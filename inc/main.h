/*
 * main.h
 * contains global macros
 */
 
#define DEBUG_CAM 0
#define ENABLE_PRINT 0
#define SCAN_LEN 128

extern unsigned long DEFAULT_SYSTEM_CLOCK;

/* Used to debug camera processing */
void matlab_print(void);
static void hardware_init();

