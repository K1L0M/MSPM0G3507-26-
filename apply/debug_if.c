#include "ti_msp_dl_config.h"
#include "headfile.h"
#include "debug_if.h"
#include "motor_control.h"
#include "gray_detection.h"
#include "sdk.h"
#include "neeprom.h"
#include <stdio.h>
#include <string.h>

#define RX_BUF_SIZE 64
static char  g_rx_buf[RX_BUF_SIZE];
static int   g_rx_idx = 0;
static int   g_rx_ready = 0;

static char  g_tx_buf[256];

/* ---- Telemetry scheduling (ISR-safe: prepare in ISR, flush in main loop) ---- */
static uint16_t g_telemetry_cnt = 0;
static volatile int g_telemetry_pending = 0;
static char  g_telemetry_buf[384];
static volatile int g_dma_busy = 0;

static char     g_dma_tx_buf[256];
static volatile int g_dma_tx_pending = 0;

static void dma_tx_flush(void)
{
    if (g_dma_busy || !g_dma_tx_pending) return;
    g_dma_tx_pending = 0;
    int len = (int)strlen(g_dma_tx_buf);
    if (len == 0) return;
    g_dma_busy = 1;
    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)g_dma_tx_buf);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_0_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, len);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

float debug_rate_target = 0;
uint8_t debug_rate_enable = 0;

volatile uint8_t g_odo_return_trigger = 0;
volatile uint8_t g_odo_reset_trigger  = 0;

/* ---- UART TX helper (non-blocking: queues via DMA, drops if busy) ---- */
static void uart_send(const char *str)
{
    if (g_dma_tx_pending) return;
    int len = (int)strlen(str);
    if (len >= (int)sizeof(g_dma_tx_buf)) len = (int)sizeof(g_dma_tx_buf) - 1;
    memcpy(g_dma_tx_buf, str, len);
    g_dma_tx_buf[len] = '\0';
    g_dma_tx_pending = 1;
    dma_tx_flush();
}

/* ---- Called from UART0 ISR ---- */
void DebugIF_FeedChar(uint8_t ch)
{
    if (ch == '\n' || ch == '\r') {
        if (g_rx_idx > 0) {
            g_rx_buf[g_rx_idx] = '\0';
            g_rx_ready = 1;
            g_rx_idx = 0;
        }
    } else if (ch >= 0x20 && ch <= 0x7E && g_rx_idx < RX_BUF_SIZE - 1) {
        g_rx_buf[g_rx_idx++] = (char)ch;
    }
}

/* ---- Called from main loop ---- */
void DebugIF_ProcessCmd(void)
{
    if (!g_rx_ready) return;
    g_rx_ready = 0;

    const char *cmd = g_rx_buf;
    char  key[16];
    float val;
    int   n;

    /* ---- No-arg commands ---- */
    if (strcmp(cmd, "RET") == 0) {
        g_odo_return_trigger = 1;
        if (sdk_work_mode == 25 || sdk_work_mode == 26) {
            beep.period = 50;
            beep.light_on_percent = 0.5f;
            beep.reset = 1;
            beep.times = 1;
        }
        return;
    }
    if (strcmp(cmd, "RST") == 0) {
        g_odo_reset_trigger = 1;
        g_odo_return_trigger = 0;
        if (sdk_work_mode == 25 || sdk_work_mode == 26) {
            beep.period = 50;
            beep.light_on_percent = 0.5f;
            beep.reset = 1;
            beep.times = 1;
        }
        return;
    }
    if (strcmp(cmd, "help") == 0) {
        uart_send("help info speed angle trace gyro save lock unlock\r\n"
                  "sp= kp= ki= kd= spmode=\r\n"
                  "pkp= pki= pkd= tkp= tki= tkd=\r\n"
                  "gkp= gki= gkd= ts= gs= dz= mode=\r\n"
                  "sakp= sakd= yaw= rate= turn= radius=\r\n");
        return;
    }
    if (strcmp(cmd, "info") == 0) {
        snprintf(g_tx_buf, sizeof(g_tx_buf),
            "sp=%.1f kp=%.2f ki=%.2f kd=%.2f spmode=%d\r\n"
            "pkp=%.0f pki=%.0f pkd=%.0f ts=%.2f dz=%.1f\r\n"
            "tkp=%.0f tki=%.0f tkd=%.0f\r\n"
            "gkp=%.1f gki=%.2f gkd=%.1f gs=%.2f\r\n"
            "sakp=%.1f sakd=%.1f radius=%.2f\r\n",
            speed_setup, speed_kp, speed_ki, speed_kd, speed_pid_mode,
            seektrack_ctrl[0].kp, seektrack_ctrl[0].ki, seektrack_ctrl[0].kd, turn_scale,
            steer_deadzone,
            seektrack_ctrl[1].kp, seektrack_ctrl[1].ki, seektrack_ctrl[1].kd,
            steergyro_ctrl.kp, steergyro_ctrl.ki, steergyro_ctrl.kd, steer_gyro_scale,
            steerangle_ctrl.kp, steerangle_ctrl.kd, trackless_motor.wheel_radius_cm);
        uart_send(g_tx_buf);
        return;
    }
    if (strcmp(cmd, "gyro") == 0) {
        snprintf(g_tx_buf, sizeof(g_tx_buf),
            "gx=%.3f gy=%.3f gz=%.3f\r\n"
            "gx_raw=%.3f gy_raw=%.3f gz_raw=%.3f\r\n"
            "gx_off=%.4f gy_off=%.4f gz_off=%.4f\r\n"
            "cal=%d conv=%d\r\n",
            smartcar_imu.gyro_dps.x, smartcar_imu.gyro_dps.y, smartcar_imu.gyro_dps.z,
            smartcar_imu.gyro_dps_raw.x, smartcar_imu.gyro_dps_raw.y, smartcar_imu.gyro_dps_raw.z,
            smartcar_imu.gyro_offset.x, smartcar_imu.gyro_offset.y, smartcar_imu.gyro_offset.z,
            smartcar_imu.imu_cal_flag, smartcar_imu.imu_convergence_flag);
        uart_send(g_tx_buf);
        return;
    }
    if (strcmp(cmd, "speed") == 0) {
        snprintf(g_tx_buf, sizeof(g_tx_buf),
            "sp=%.1f fbL=%.1f fbR=%.1f errL=%.1f errR=%.1f\r\n"
            "outL=%.1f outR=%.1f intL=%.1f intR=%.1f\r\n"
            "kp=%.2f ki=%.2f kd=%.2f mode=%d\r\n",
            speed_setup,
            smartcar_imu.left_motor_speed_cmps,
            smartcar_imu.right_motor_speed_cmps,
            speed_error[0], speed_error[1],
            speed_output[0], speed_output[1],
            speed_integral[0], speed_integral[1],
            speed_kp, speed_ki, speed_kd, speed_pid_mode);
        uart_send(g_tx_buf);
        return;
    }
    if (strcmp(cmd, "angle") == 0) {
        snprintf(g_tx_buf, sizeof(g_tx_buf),
            "saex=%.1f sayaw=%.1f saerr=%.1f saout=%.1f sakp=%.1f sakd=%.1f\r\n"
            "gex=%.1f gfb=%.1f gout=%.1f\r\n"
            "gkp=%.1f gki=%.2f gkd=%.1f gs=%.2f mode=%d\r\n",
            steer_angle_expect,
            smartcar_imu.rpy_deg[_YAW],
            steer_angle_error,
            steer_angle_output,
            steerangle_ctrl.kp,
            steerangle_ctrl.kd,
            steer_gyro_expect,
            steergyro_ctrl.feedback,
            steer_gyro_output,
            steergyro_ctrl.kp, steergyro_ctrl.ki, steergyro_ctrl.kd,
            steer_gyro_scale,
            trackless_output.yaw_ctrl_mode);
        uart_send(g_tx_buf);
        return;
    }
    if (strcmp(cmd, "trace") == 0) {
        snprintf(g_tx_buf, sizeof(g_tx_buf),
        "gstat=%.1f tout=%.1f ts=%.2f worse=%u\r\n"
        "pkp=%.0f pki=%.0f pkd=%.0f dz=%.1f\r\n"
        "vstat=%.1f vworse=%u\r\n"
            "tkp=%.0f tki=%.0f tkd=%.0f\r\n",
            gray_status[0], turn_output, turn_scale, gray_status_worse,
            seektrack_ctrl[0].kp, seektrack_ctrl[0].ki, seektrack_ctrl[0].kd, steer_deadzone,
            gray_status[1], vision_status_worse,
            seektrack_ctrl[1].kp, seektrack_ctrl[1].ki, seektrack_ctrl[1].kd);
        uart_send(g_tx_buf);
        return;
    }
    if (strcmp(cmd, "lock") == 0) {
        trackless_output.unlock_flag = LOCK;
        uart_send("OK locked\r\n");
        return;
    }
    if (strcmp(cmd, "unlock") == 0) {
        trackless_output.unlock_flag = UNLOCK;
        uart_send("OK unlocked\r\n");
        return;
    }
    if (strcmp(cmd, "save") == 0) {
        WriteFlashParameter(CTRL_SPEED_KP,  speed_kp,  &Trackless_Params);
        WriteFlashParameter(CTRL_SPEED_KI,  speed_ki,  &Trackless_Params);
        WriteFlashParameter(CTRL_SPEED_KD,  speed_kd,  &Trackless_Params);
        WriteFlashParameter(SPEED_SETUP,    speed_setup, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KP1,  seektrack_ctrl[0].kp, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KI1,  seektrack_ctrl[0].ki, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KD1,  seektrack_ctrl[0].kd, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KP2,  seektrack_ctrl[1].kp, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KI2,  seektrack_ctrl[1].ki, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_KD2,  seektrack_ctrl[1].kd, &Trackless_Params);
        WriteFlashParameter(CTRL_TURN_SCALE, turn_scale, &Trackless_Params);
        WriteFlashParameter(CTRL_GYRO_KP,   steergyro_ctrl.kp, &Trackless_Params);
        WriteFlashParameter(CTRL_GYRO_KI,   steergyro_ctrl.ki, &Trackless_Params);
        WriteFlashParameter(CTRL_GYRO_KD,   steergyro_ctrl.kd, &Trackless_Params);
        WriteFlashParameter(CTRL_GYRO_SCALE, steer_gyro_scale, &Trackless_Params);
        WriteFlashParameter(GYRO_X_OFFSET, smartcar_imu.gyro_offset.x, &Trackless_Params);
        WriteFlashParameter(GYRO_Y_OFFSET, smartcar_imu.gyro_offset.y, &Trackless_Params);
        WriteFlashParameter(GYRO_Z_OFFSET, smartcar_imu.gyro_offset.z, &Trackless_Params);
        WriteFlashParameter(RESERVED_PARAMS_18, steer_deadzone, &Trackless_Params);
        WriteFlashParameter(RESERVED_PARAMS_19, steerangle_ctrl.kd, &Trackless_Params);
        WriteFlashParameter(RESERVED_PARAMS_20, steerangle_ctrl.kp, &Trackless_Params);
        uart_send("OK saved\r\n");
        return;
    }

    /* ---- key=value parsing ---- */
    if (sscanf(cmd, "%15[^=]=%f%n", key, &val, &n) < 2) {
        snprintf(g_tx_buf, sizeof(g_tx_buf), "ERR unknown \"%s\"\r\n", cmd);
        uart_send(g_tx_buf);
        return;
    }
    if (cmd[n] != '\0' && cmd[n] != '\r' && cmd[n] != '\n') {
        snprintf(g_tx_buf, sizeof(g_tx_buf), "ERR trailing \"%s\"\r\n", cmd + n);
        uart_send(g_tx_buf);
        return;
    }

    /* ---- Speed PID ---- */
    if (strcmp(key, "kp") == 0) {
        speed_kp = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK kp=%.2f\r\n", speed_kp);
    }
    else if (strcmp(key, "ki") == 0) {
        speed_ki = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK ki=%.2f\r\n", speed_ki);
    }
    else if (strcmp(key, "kd") == 0) {
        speed_kd = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK kd=%.2f\r\n", speed_kd);
    }
    else if (strcmp(key, "sp") == 0) {
        speed_setup = val;
        speed_expect[0] = val;
        speed_expect[1] = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK sp=%.1f\r\n", speed_setup);
    }
    else if (strcmp(key, "spmode") == 0) {
        speed_pid_mode = (uint8_t)val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK spmode=%d\r\n", speed_pid_mode);
    }
    /* ---- Turn PID (seektrack_ctrl[0]) ---- */
    else if (strcmp(key, "pkp") == 0) {
        seektrack_ctrl[0].kp = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK pkp=%.0f\r\n", seektrack_ctrl[0].kp);
    }
    else if (strcmp(key, "pki") == 0) {
        seektrack_ctrl[0].ki = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK pki=%.0f\r\n", seektrack_ctrl[0].ki);
    }
    else if (strcmp(key, "pkd") == 0) {
        seektrack_ctrl[0].kd = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK pkd=%.0f\r\n", seektrack_ctrl[0].kd);
    }
    /* ---- Track PID2 (seektrack_ctrl[1]) ---- */
    else if (strcmp(key, "tkp") == 0) {
        seektrack_ctrl[1].kp = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK tkp=%.0f\r\n", seektrack_ctrl[1].kp);
    }
    else if (strcmp(key, "tki") == 0) {
        seektrack_ctrl[1].ki = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK tki=%.0f\r\n", seektrack_ctrl[1].ki);
    }
    else if (strcmp(key, "tkd") == 0) {
        seektrack_ctrl[1].kd = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK tkd=%.0f\r\n", seektrack_ctrl[1].kd);
    }
    /* ---- Gyro PID ---- */
    else if (strcmp(key, "gkp") == 0) {
        steergyro_ctrl.kp = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gkp=%.1f\r\n", steergyro_ctrl.kp);
    }
    else if (strcmp(key, "gki") == 0) {
        steergyro_ctrl.ki = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gki=%.2f\r\n", steergyro_ctrl.ki);
    }
    else if (strcmp(key, "gkd") == 0) {
        steergyro_ctrl.kd = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gkd=%.1f\r\n", steergyro_ctrl.kd);
    }
    /* ---- Steer angle P ---- */
    else if (strcmp(key, "sakp") == 0) {
        steerangle_ctrl.kp = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK sakp=%.1f\r\n", steerangle_ctrl.kp);
    }
    /* ---- Steer angle D ---- */
    else if (strcmp(key, "sakd") == 0) {
        steerangle_ctrl.kd = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK sakd=%.1f\r\n", steerangle_ctrl.kd);
    }
    /* ---- Gyro offsets ---- */
    else if (strcmp(key, "gxoff") == 0) {
        smartcar_imu.gyro_offset.x = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gxoff=%.4f\r\n", smartcar_imu.gyro_offset.x);
    }
    else if (strcmp(key, "gyoff") == 0) {
        smartcar_imu.gyro_offset.y = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gyoff=%.4f\r\n", smartcar_imu.gyro_offset.y);
    }
    else if (strcmp(key, "gzoff") == 0) {
        smartcar_imu.gyro_offset.z = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gzoff=%.4f\r\n", smartcar_imu.gyro_offset.z);
    }
    /* ---- Scales ---- */
    else if (strcmp(key, "ts") == 0) {
        turn_scale = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK ts=%.2f\r\n", turn_scale);
    }
    else if (strcmp(key, "gs") == 0) {
        steer_gyro_scale = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK gs=%.2f\r\n", steer_gyro_scale);
    }
    /* ---- Steer deadzone ---- */
    else if (strcmp(key, "dz") == 0) {
        steer_deadzone = val;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK dz=%.1f\r\n", steer_deadzone);
    }
    /* ---- Work mode switch ---- */
    else if (strcmp(key, "mode") == 0) {
        sdk_work_mode = (int16_t)val;
        trackless_output.init = 0;  // 触发初始化复位
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK mode=%d\r\n", sdk_work_mode);
    }
    /* ---- Yaw target: enter AZIMUTH mode ---- */
    else if (strcmp(key, "yaw") == 0) {
        while (val < 0) val += 360.0f;
        while (val >= 360.0f) val -= 360.0f;
        steer_angle_expect = val;
        trackless_output.yaw_outer_control_output = val;
        trackless_output.yaw_ctrl_mode = AZIMUTH;
        trackless_output.yaw_ctrl_start = 1;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK yaw=%.1f (AZIMUTH)\r\n", val);
    }
    /* ---- Gyro rate target: inner-loop test ---- */
    else if (strcmp(key, "rate") == 0) {
        debug_rate_target = val;
        debug_rate_enable = 1;
        sdk_work_mode = 50;
        trackless_output.yaw_outer_control_output = val;
        trackless_output.yaw_ctrl_mode = CLOCKWISE_TURN;
        trackless_output.yaw_ctrl_start = 0;
        trackless_output.unlock_flag = UNLOCK;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK rate=%.1f deg/s (inner-loop)\r\n", val);
    }
    /* ---- Gyro rate advanced: inner-loop with ramp+kick ---- */
    else if (strcmp(key, "rate_adv") == 0) {
        debug_rate_target = val;
        debug_rate_enable = 1;
        sdk_work_mode = 22;
        trackless_output.yaw_outer_control_output = val;
        trackless_output.unlock_flag = UNLOCK;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK rate_adv=%.1f deg/s (inner-adv)\r\n", val);
    }
    /* ---- Cascade: angle outer + gyro inner ---- */
    else if (strcmp(key, "cascade") == 0) {
        while (val < 0) val += 360.0f;
        while (val >= 360.0f) val -= 360.0f;
        sdk_work_mode = 23;
        trackless_output.yaw_outer_control_output = val;
        trackless_output.unlock_flag = UNLOCK;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK cascade=%.1f deg (cascade)\r\n", val);
    }
    // 相对角度转向 (case24)
        // 相对角度转向, 使用AZIMUTH绝对航向模式
        // 相对角度转向 (case24)
        // 相对角度转向 (case24)
        // 相对角度转向 (case24)
    else if (strcmp(key, "turn") == 0) {
        sdk_work_mode = 24;
        trackless_output.yaw_outer_control_output = val;
        trackless_output.unlock_flag = UNLOCK;
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK turn=%.1f deg\r\n", val);
    }
    else if (strcmp(key, "radius") == 0) {
        trackless_motor.wheel_radius_cm = val;
        trackless_motor.wheel_radius_cm = constrain_float(trackless_motor.wheel_radius_cm, 0.1f, 20.0f);
        WriteFlashParameter(TIRE_RADIUS_CM_CFG, trackless_motor.wheel_radius_cm, &Trackless_Params);
        snprintf(g_tx_buf, sizeof(g_tx_buf), "OK radius=%.2f cm\r\n", trackless_motor.wheel_radius_cm);
    }
    else {
        snprintf(g_tx_buf, sizeof(g_tx_buf), "ERR unknown \"%s\"\r\n", key);
    }
    uart_send(g_tx_buf);
}

/* ---- Called from 100Hz ISR: set flag only, no heavy work ---- */
void DebugIF_RequestTelemetry(void)
{
    g_telemetry_cnt++;
    if (g_telemetry_cnt < 10) return; /* 10 * 10ms = 100ms */
    g_telemetry_cnt = 0;
    g_telemetry_pending = 1;
}

/* ---- Called from main loop: format + flush telemetry via DMA ---- */
void DebugIF_FlushTelemetry(void)
{
    dma_tx_flush();  /* drain pending debug/command output first */
    if (!g_telemetry_pending) return;
    if (g_dma_busy) return;


    g_telemetry_pending = 0;

    int len = snprintf(g_telemetry_buf, sizeof(g_telemetry_buf),
        "%.1f,%.1f,%.1f,%.1f,%d,"
        "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,"
        "%.1f,%.1f\r\n",
        speed_setup,
        smartcar_imu.left_motor_speed_cmps,
        smartcar_imu.right_motor_speed_cmps,
        speed_output[0],
        sdk_work_mode,
        smartcar_imu.rpy_deg[_YAW],
        steer_angle_expect,
        steer_angle_error,
        steer_angle_output,
        steergyro_ctrl.feedback,
        steer_gyro_output,
        trackless_output.yaw_ctrl_mode,
        gray_status[0],
        turn_output);
    if (len <= 0 || len >= (int)sizeof(g_telemetry_buf)) return;

    g_dma_busy = 1;

    DL_DMA_disableChannel(DMA, DMA_CH1_CHAN_ID);
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)g_telemetry_buf);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t)(&UART_0_INST->TXDATA));
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, len);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
}

/* ---- Called from UART0 ISR on DMA_DONE_TX ---- */
void DebugIF_NotifyDMADone(void)
{
    g_dma_busy = 0;
}

void DebugIF_Print(const char *str)
{
	if (g_dma_tx_pending) return;
	int len = (int)strlen(str);
	if (len >= (int)sizeof(g_dma_tx_buf)) len = (int)sizeof(g_dma_tx_buf) - 1;
	memcpy(g_dma_tx_buf, str, len);
	g_dma_tx_buf[len] = '\0';
	g_dma_tx_pending = 1;
	dma_tx_flush();
}
