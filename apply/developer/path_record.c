#include "headfile.h"
#include "path_record.h"
#include "debug_if.h"
#include <string.h>

/* ---- RAM path buffer ---- */
static PathPoint_t  g_path_buf[MAX_PATH_POINTS];
static uint32_t     g_path_count;
static float        g_last_sample_x, g_last_sample_y;
static uint8_t      g_flash_valid;

/* ---- Non-blocking Flash state machine (driven at 10 Hz) ---- */
#define FLASH_OP_IDLE   0
#define FLASH_OP_ERASE  1
#define FLASH_OP_SAVE   2

static volatile uint8_t  g_flash_op   = FLASH_OP_IDLE;
static volatile uint8_t  g_flash_sub  = 0;
static volatile uint16_t g_flash_page = 0;

/* ---- Page buffer for SAVE (256 B, static to avoid stack pressure in ISR) ---- */
static uint8_t g_page_buf[256];

void PathRecord_Init(void)
{
    g_path_count = 0;
    g_last_sample_x = smartcar_imu.state_estimation.pos.x;
    g_last_sample_y = smartcar_imu.state_estimation.pos.y;
    /* Do NOT touch Flash here — LoadFromFlash is called separately */
}

void PathRecord_Sample(void)
{
    float dx, dy;

    if (g_path_count >= MAX_PATH_POINTS)
        return;

    dx = smartcar_imu.state_estimation.pos.x - g_last_sample_x;
    dy = smartcar_imu.state_estimation.pos.y - g_last_sample_y;

    if ((dx * dx + dy * dy) >= PATH_SAMPLE_DIST_SQ) {
        g_path_buf[g_path_count].x = smartcar_imu.state_estimation.pos.x;
        g_path_buf[g_path_count].y = smartcar_imu.state_estimation.pos.y;
        g_path_count++;
        g_last_sample_x = smartcar_imu.state_estimation.pos.x;
        g_last_sample_y = smartcar_imu.state_estimation.pos.y;
    }
}

void PathRecord_RequestErase(void)
{
    if (g_flash_op != FLASH_OP_IDLE) return;
    g_flash_op  = FLASH_OP_ERASE;
    g_flash_sub = 0;
}

void PathRecord_RequestSave(void)
{
    if (g_flash_op != FLASH_OP_IDLE) return;
    g_flash_op   = FLASH_OP_SAVE;
    g_flash_sub  = 0;
    g_flash_page = 0;
}

uint8_t PathRecord_IsFlashBusy(void)
{
    return (g_flash_op != FLASH_OP_IDLE);
}

uint8_t PathRecord_IsFlashValid(void)
{
    return g_flash_valid;
}

/* ---- Accessors ---- */
int32_t PathRecord_GetCount(void) { return (int32_t)g_path_count; }

float PathRecord_GetX(uint32_t idx)
{
    return (idx < g_path_count) ? g_path_buf[idx].x : 0.0f;
}

float PathRecord_GetY(uint32_t idx)
{
    return (idx < g_path_count) ? g_path_buf[idx].y : 0.0f;
}

/* ---- Load from Flash (blocking, called once on mode entry) ---- */
void PathRecord_LoadFromFlash(void)
{
    uint8_t  header[8];
    uint32_t magic, count;
    uint32_t i, j;
    uint8_t  buf[256];

    g_flash_valid = 0;

    W25Q64_read(header, PATH_FLASH_ADDR, 8);

    magic = ((uint32_t)header[0])
          | ((uint32_t)header[1] << 8)
          | ((uint32_t)header[2] << 16)
          | ((uint32_t)header[3] << 24);
    count = ((uint32_t)header[4])
          | ((uint32_t)header[5] << 8)
          | ((uint32_t)header[6] << 16)
          | ((uint32_t)header[7] << 24);

    if (magic != PATH_FLASH_MAGIC || count == 0 || count > MAX_PATH_POINTS) {
        g_path_count = 0;
        return;
    }

    /* Read points in 256-byte pages (32 points per page) */
    for (i = 0; i < count; i += 32) {
        uint32_t chunk = count - i;
        if (chunk > 32) chunk = 32;
        W25Q64_read(buf, PATH_FLASH_ADDR + 8 + i * 8, chunk * 8);

        for (j = 0; j < chunk; j++) {
            uint32_t ix, iy;
            float    fx, fy;
            uint8_t *px = &buf[j * 8];
            uint8_t *py = &buf[j * 8 + 4];
            ix = ((uint32_t)px[0])
               | ((uint32_t)px[1] << 8)
               | ((uint32_t)px[2] << 16)
               | ((uint32_t)px[3] << 24);
            iy = ((uint32_t)py[0])
               | ((uint32_t)py[1] << 8)
               | ((uint32_t)py[2] << 16)
               | ((uint32_t)py[3] << 24);
            memcpy(&fx, &ix, sizeof(float));
            memcpy(&fy, &iy, sizeof(float));
            g_path_buf[i + j].x = fx;
            g_path_buf[i + j].y = fy;
        }
    }

    g_path_count = count;
    g_flash_valid = 1;

    if (count > 0) {
        g_last_sample_x = g_path_buf[count - 1].x;
        g_last_sample_y = g_path_buf[count - 1].y;
    }
}

/* ---- Pack a float into 4 bytes (little-endian) ---- */
static void pack_float(uint8_t *dst, float val)
{
    uint32_t u;
    memcpy(&u, &val, sizeof(float));
    dst[0] = (uint8_t)(u);
    dst[1] = (uint8_t)(u >> 8);
    dst[2] = (uint8_t)(u >> 16);
    dst[3] = (uint8_t)(u >> 24);
}

/*
 * Non-blocking Flash op state machine.
 * Called from duty_10hz() — each call does ONE small step.
 */
void PathRecord_ProcessFlashOp(void)
{
    uint32_t chunk, j;

    if (g_flash_op == FLASH_OP_IDLE)
        return;

    /* ---- ERASE pipeline ---- */
    if (g_flash_op == FLASH_OP_ERASE) {
        if (g_flash_sub == 0) {
            W25Q64_erase_sector_send(PATH_FLASH_SECTOR);
            g_flash_sub = 1;
            return;
        }
        if (g_flash_sub == 1) {
            if (!W25Q64_is_busy()) {
                g_flash_op  = FLASH_OP_IDLE;
                g_flash_sub = 0;
            }
            return;
        }
    }

    /* ---- SAVE pipeline ---- */
    if (g_flash_op == FLASH_OP_SAVE) {
        switch (g_flash_sub) {

        /* sub=0: Write count (4 bytes at addr+4) */
        case 0: {
            uint32_t cnt = g_path_count;
            g_page_buf[0] = (uint8_t)(cnt);
            g_page_buf[1] = (uint8_t)(cnt >> 8);
            g_page_buf[2] = (uint8_t)(cnt >> 16);
            g_page_buf[3] = (uint8_t)(cnt >> 24);
            W25Q64_write_page_send(g_page_buf, PATH_FLASH_ADDR + 4, 4);
            g_flash_sub = 1;
            return;
        }

        /* sub=1: Poll after count write */
        case 1:
            if (!W25Q64_is_busy()) {
                g_flash_sub  = 2;
                g_flash_page = 0;
            }
            return;

        /* sub=2: Write one data page (256 bytes = 32 points) */
        case 2:
            chunk = g_path_count - g_flash_page * 32;
            if (chunk > 32) chunk = 32;
            for (j = 0; j < chunk; j++) {
                pack_float(&g_page_buf[j * 8],
                           g_path_buf[g_flash_page * 32 + j].x);
                pack_float(&g_page_buf[j * 8 + 4],
                           g_path_buf[g_flash_page * 32 + j].y);
            }
            W25Q64_write_page_send(g_page_buf,
                PATH_FLASH_ADDR + 8 + g_flash_page * 256, chunk * 8);
            g_flash_sub = 3;
            return;

        /* sub=3: Poll after data page write */
        case 3:
            if (!W25Q64_is_busy()) {
                g_flash_page++;
                if (g_flash_page * 32 < g_path_count)
                    g_flash_sub = 2;  /* more pages */
                else
                    g_flash_sub = 4;  /* write magic last */
            }
            return;

        /* sub=4: Write magic (4 bytes at addr+0) — LAST STEP */
        case 4: {
            uint32_t m = PATH_FLASH_MAGIC;
            g_page_buf[0] = (uint8_t)(m);
            g_page_buf[1] = (uint8_t)(m >> 8);
            g_page_buf[2] = (uint8_t)(m >> 16);
            g_page_buf[3] = (uint8_t)(m >> 24);
            W25Q64_write_page_send(g_page_buf, PATH_FLASH_ADDR, 4);
            g_flash_sub = 5;
            return;
        }

        /* sub=5: Poll after magic write → DONE */
        case 5:
            if (!W25Q64_is_busy()) {
                g_flash_op    = FLASH_OP_IDLE;
                g_flash_sub   = 0;
                g_flash_valid = 1;
            }
            return;
        }
    }
}
