#ifndef __PATH_RECORD_H
#define __PATH_RECORD_H
#include <stdint.h>

typedef struct __attribute__((packed)) {
    float x, y;   /* cm, 8 bytes */
} PathPoint_t;

#define MAX_PATH_POINTS      500
#define PATH_SAMPLE_DIST_SQ  (4.0f * 4.0f)   /* 16.0 = 4 cm squared */

#define PATH_FLASH_ADDR      0x00001000UL
#define PATH_FLASH_SECTOR    1
#define PATH_FLASH_MAGIC     0x50415448UL     /* "PATH" */

/* ---- Public API ---- */
void PathRecord_Init(void);
void PathRecord_Sample(void);
void PathRecord_RequestSave(void);
void PathRecord_RequestErase(void);
void PathRecord_ProcessFlashOp(void);
void PathRecord_LoadFromFlash(void);

int32_t PathRecord_GetCount(void);
float   PathRecord_GetX(uint32_t idx);
float   PathRecord_GetY(uint32_t idx);
uint8_t PathRecord_IsFlashBusy(void);
uint8_t PathRecord_IsFlashValid(void);
#endif
