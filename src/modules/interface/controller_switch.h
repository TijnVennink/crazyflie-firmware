#ifndef AUX_SWITCH_H_
#define AUX_SWITCH_H_

#include <stdbool.h>
#include <stdint.h>

void switchInit(void);
void auxSwitchTask(void *arg);
bool auxConnected(void);
bool auxState(uint8_t auxNumber);
void armWithAux(void);

#endif // AUX_SWITCH_H_