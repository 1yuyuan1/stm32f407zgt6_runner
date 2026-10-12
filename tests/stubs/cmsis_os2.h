#ifndef TEST_CMSIS_OS2_H
#define TEST_CMSIS_OS2_H
#include <stdint.h>
typedef void *osThreadId_t;
typedef struct {
    const char *name;
    uint32_t stack_size;
    int priority;
} osThreadAttr_t;
enum { osPriorityNormal, osPriorityAboveNormal, osPriorityHigh };
osThreadId_t osThreadNew(void (*)(void *), void *, const osThreadAttr_t *);
#endif
