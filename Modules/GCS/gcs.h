#ifndef __GCS_H
#define __GCS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#define ANO_LOG_FONT_MAX_LEN 128
#define ANO_MAX_CMD_NUM 167


/* logger level, the number is compatible for syslog */
#define LOG_LVL_ASSERT 0
#define LOG_LVL_WARNING 1
#define LOG_LVL_INFO 2
#define LOG_LVL_DBG 3

#define ULOG_OUTPUT_LVL LOG_LVL_DBG

#if !defined(LOG_TAG)
/* compatible for rtdbg */
#if defined(DBG_TAG)
#define LOG_TAG DBG_TAG
#else
#define LOG_TAG "NO_TAG"
#endif
#endif /* !defined(LOG_TAG) */

#if !defined(LOG_LVL)
/* compatible for rtdbg */
#if defined(DBG_LVL)
#define LOG_LVL DBG_LVL
#else
#define LOG_LVL LOG_LVL_DBG
#endif
#endif /* !defined(LOG_LVL) */

#if (LOG_LVL >= LOG_LVL_DBG) && (ULOG_OUTPUT_LVL >= LOG_LVL_DBG)
#define ulog_d(TAG, ...) ulog_output(LOG_LVL_DBG, TAG, __VA_ARGS__)
#else
#define ulog_d(TAG, ...)
#endif /* (LOG_LVL >= LOG_LVL_DBG) && (ULOG_OUTPUT_LVL >= LOG_LVL_DBG) */

#if (LOG_LVL >= LOG_LVL_INFO) && (ULOG_OUTPUT_LVL >= LOG_LVL_INFO)
#define ulog_i(TAG, ...) ulog_output(LOG_LVL_INFO, TAG, __VA_ARGS__)
#else
#define ulog_i(TAG, ...)
#endif /* (LOG_LVL >= LOG_LVL_INFO) && (ULOG_OUTPUT_LVL >= LOG_LVL_INFO) */

#if (LOG_LVL >= LOG_LVL_WARNING) && (ULOG_OUTPUT_LVL >= LOG_LVL_WARNING)
#define ulog_w(TAG, ...) ulog_output(LOG_LVL_WARNING, TAG, __VA_ARGS__)
#else
#define ulog_w(TAG, ...)
#endif /* (LOG_LVL >= LOG_LVL_WARNING) && (ULOG_OUTPUT_LVL >= LOG_LVL_WARNING) */

#define LOG_W(...) ulog_w(LOG_TAG, __VA_ARGS__)
#define LOG_I(...) ulog_i(LOG_TAG, __VA_ARGS__)
#define LOG_D(...) ulog_d(LOG_TAG, __VA_ARGS__)
void ulog_output(uint32_t level, const char *tag, const char *format, ...);

#pragma pack(push, 1)
typedef struct
{
    uint16_t par_id;
    int32_t par_val;
} par_t;
typedef struct
{
    uint8_t head;
    uint8_t d_addr;
    uint8_t id;
    uint8_t len;
    void *data;
    uint8_t sum_check;
    uint8_t add_check;
} ano_t;

struct serial_prepare
{
    uint8_t head_len;
    uint8_t addr_len;

    uint8_t head;
    uint8_t d_addr;
    uint8_t cmd;
    uint8_t data_len;
    uint8_t data_buf[0xFF];
    uint8_t sum_check;
    uint8_t add_check;
};
typedef struct serial_prepare serial_prepare_t;

typedef void (*ano_callback)(int32_t *, void *);
typedef struct
{
    int32_t *data_addr;
    uint32_t id;
    ano_callback cb;
} cmd_list_t;
#pragma pack(pop)

void ano_cycle_run(void);
void ano_init(void);
void gcs_tx_buff_flush(void);
void ano_send_attitude(float roll, float pitch, float yaw);
void ano_send_sensor_imu(float ax, float ay, float az, float gx, float gy, float gz);
void ano_send_sensor_mag(float mx, float my, float mz, float temperature);
void ano_seng_mag_cali(uint8_t cmd0, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4, 
                       uint8_t cmd5, uint8_t cmd6, uint8_t cmd7, uint8_t cmd8, uint8_t cmd9);
void ano_send_user_data(float data[10]);

#ifdef __cplusplus
}
#endif

#endif
