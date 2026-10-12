#ifndef CPP_MAIN_H
#define CPP_MAIN_H

#include "chassis_app.h"

#ifdef __cplusplus
extern "C" {
#endif

// 由默认任务调用一次，初始化底盘与上层逻辑任务。
void cpp_main(void);

#ifdef __cplusplus
}
#endif
#endif
