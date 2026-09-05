#include <Arduino.h>
#include "app_runtime.h"
void setup() { vehicle::beginApplication(); }
// Arduino已启动调度器；业务迁移到静态任务后让出loopTask。
void loop() { delay(1000); }
