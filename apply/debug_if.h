#ifndef __DEBUG_IF_H
#define __DEBUG_IF_H

extern uint8_t speed_pid_mode;
extern float debug_rate_target;
extern uint8_t debug_rate_enable;

void DebugIF_FeedChar(uint8_t ch);
void DebugIF_ProcessCmd(void);
void DebugIF_RequestTelemetry(void);
void DebugIF_FlushTelemetry(void);
void DebugIF_NotifyDMADone(void);

#endif
