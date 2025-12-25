#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "gcs.h"
#include "usart.h"
#include "main.h"

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

static ano_t ano;
static serial_prepare_t receive;
static cmd_list_t read_cmd_list[ANO_MAX_CMD_NUM];
static cmd_list_t write_cmd_list[ANO_MAX_CMD_NUM];
static char str_buf[ANO_LOG_FONT_MAX_LEN];

static uint8_t rec_usart_fifo[128];
static uint8_t send_usart_fifo[512];
static uint16_t send_usart_fifo_num;
static uint16_t rec_usart_fifo_num;
static void ano_check(ano_t *ano)
{
    ano->sum_check = 0;
    ano->add_check = 0;
    for (size_t i = 0; i < 4; i++)
    {
        ano->sum_check += *(uint8_t *)(&ano->head + i);
        ano->add_check += ano->sum_check;
    }

    for (size_t i = 0; i < ano->len; i++)
    {
        ano->sum_check += *((uint8_t *)(ano->data) + i);
        ano->add_check += ano->sum_check;
    }
}

static void ano_send(ano_t *s)
{
    uint8_t buf[128];
    ano_check(s);
    
    if(send_usart_fifo_num + 6 + s->len > 512)
        return;
    
    memcpy(buf, (uint8_t *)s, 4);
    memcpy(buf + 4, (uint8_t *)s->data, s->len);
    memcpy(buf + 4 + s->len, (uint8_t *)(&s->sum_check), 2);
    memcpy(send_usart_fifo + send_usart_fifo_num, buf, 6 + s->len);
    send_usart_fifo_num += 6 + s->len;
}

static void ulog_voutput(uint32_t level, const char *tag, const char *format, va_list args)
{
    switch (level)
    {
    case LOG_LVL_WARNING:
        str_buf[0] = 1;
        break;
    case LOG_LVL_INFO:
        str_buf[0] = 2;
        break;
    case LOG_LVL_DBG:
        str_buf[0] = 0;
        break;

    default:
        break;
    }
    sprintf(str_buf + 1, "[%s]:", tag);
    vsnprintf(str_buf + 1 + strlen(str_buf + 1), ANO_LOG_FONT_MAX_LEN, format, args);
    ano.id = 0xA0;
    ano.len = strlen(str_buf + 1) + 1;
    ano.data = str_buf;
    ano_send(&ano);
}

void ulog_output(uint32_t level, const char *tag, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);

    ulog_voutput(level, tag, format, args);

    va_end(args);
}

static void add_read_cmd_event(uint32_t id, int32_t *data_addr, ano_callback cb)
{
    if (id >= ANO_MAX_CMD_NUM)
        return;
    read_cmd_list[id].data_addr = data_addr;
    read_cmd_list[id].id = id;
    read_cmd_list[id].cb = cb;
}

static void add_write_cmd_event(uint32_t id, int32_t *data_addr, ano_callback cb)
{
    if (id >= ANO_MAX_CMD_NUM)
        return;
    write_cmd_list[id].data_addr = data_addr;
    write_cmd_list[id].id = id;
    write_cmd_list[id].cb = cb;
}

static void ano_check_reply(ano_t *rec)
{
    static uint8_t r_d[3];
    r_d[0] = rec->id;
    r_d[1] = rec->sum_check;
    r_d[2] = rec->add_check;
    ano.id = 0;
    ano.len = 3;
    ano.data = r_d;
    ano_send(&ano);
}

static void ano_processing(void)
{
    static ano_t ano_receive;
    static uint8_t data_buf[128];
    ano_receive.data = data_buf;
    memcpy(&ano_receive, &receive.head, 4);
    memcpy(ano_receive.data, &receive.data_buf, receive.data_len);
    ano_check(&ano_receive);
    if (ano_receive.sum_check != receive.sum_check || ano_receive.add_check != receive.add_check)
        return;

    if(ano_receive.id == 0xE0)
    {
        ano_check_reply(&ano_receive);
        uint8_t cid = *(uint8_t*)ano_receive.data;
        uint8_t cmd1 = *(uint8_t*)(ano_receive.data+2);
        if(cid == 0x01)
        {
            switch(cmd1)
            {
                case 0x02:
                    _sensor_need_cali.gyro = true; 
                    break;
                case 0x04:
                    _sensor_need_cali.mag = true;
                    break;
                case 0x05:
                    _sensor_need_cali.acc = true;
                    break;
                case 0x10:
                    _sensor_need_cali.cancel_mag = true;
                    break;
                default:
                    break;
            }
        }
    }
    if (ano_receive.id == 0xE1)
    {
        for (size_t i = 0; i < ANO_MAX_CMD_NUM; i++)
        {
            uint16_t cmd = *(uint16_t *)ano_receive.data;
            if (cmd == read_cmd_list[i].id)
            {
                read_cmd_list[i].cb(read_cmd_list[i].data_addr, (void *)&ano_receive);
                return;
            }
        }
    }
    else if (ano_receive.id == 0xE2)
    {
        ano_check_reply(&ano_receive);
        for (size_t i = 0; i < ANO_MAX_CMD_NUM; i++)
        {
            uint16_t cmd = *(uint16_t *)ano_receive.data;
            if (cmd == write_cmd_list[i].id)
            {
                write_cmd_list[i].cb(write_cmd_list[i].data_addr, (void *)&ano_receive);
                return;
            }
        }
    }
}

static void _ano_receive(uint8_t data)
{
    static uint16_t buf_len = 0;

    if ((buf_len < (receive.head_len + receive.addr_len) &&
         data == *((uint8_t *)&receive.head + buf_len)) ||
        ((buf_len == receive.head_len) && (data == 0x00)))
    {
        *((uint8_t *)&receive.head + buf_len) = data;
        buf_len++;
    }
    else if (buf_len == (receive.head_len + receive.addr_len))
    {
        receive.cmd = data;
        buf_len++;
    }
    else if (buf_len == (receive.head_len + receive.addr_len) + 1)
    {
        receive.data_len = data;
        buf_len++;
    }
    else if (buf_len > (receive.head_len + receive.addr_len) + 1 &&
             buf_len < (receive.head_len + receive.addr_len) + receive.data_len + 2)
    {
        *((uint8_t *)&receive.head + buf_len) = data;
        buf_len++;
    }
    else if (buf_len == (receive.head_len + receive.addr_len) + receive.data_len + 2)
    {
        receive.sum_check = data;
        buf_len++;
    }
    else if (buf_len == (receive.head_len + receive.addr_len) + receive.data_len + 3)
    {
        receive.add_check = data;
        ano_processing();
        buf_len = 0;
        receive.data_len = 0;
    }
    else
    {
        buf_len = 0;
        receive.data_len = 0;
    }
}

static void ret_default_read_cb(int32_t *addr_num, void *par)
{
    static par_t p;
    uint16_t id = *(uint16_t *)((ano_t *)par)->data;

    ano.id = 0xE2;
    ano.len = 6;

    p.par_id = id;
    p.par_val = 0;

    ano.data = &p;

    ano_send(&ano);
}

static void ret_default_write_cb(int32_t *addr_num, void *par)
{
}

static void ano_cmd_list_init(void)
{
    for (size_t i = 0; i < ANO_MAX_CMD_NUM; i++)
    {
        add_read_cmd_event(i, 0, ret_default_read_cb);
        add_write_cmd_event(i, 0, ret_default_write_cb);
    }
}

void gcs_tx_buff_flush(void)
{
    if (send_usart_fifo_num < 512 && send_usart_fifo_num >0 && _tx_complete)
    {
        memcpy(uart_tx_dma, send_usart_fifo, send_usart_fifo_num);
        _tx_complete = false;
        HAL_UART_Transmit_DMA(&huart2, uart_tx_dma, send_usart_fifo_num);
        send_usart_fifo_num = 0;
    }
}

void ano_cycle_run(void)
{
    uint16_t fifo_num;
    
    gcs_tx_buff_flush();
    
    if(_gcs_receive_cnt > 0)
    {
        rec_usart_fifo_num = _gcs_receive_cnt;
        memcpy(rec_usart_fifo, _gcs_receive_buff, _gcs_receive_cnt);
        _gcs_receive_cnt = 0;
    }
    if (rec_usart_fifo_num > 0)
    {
        fifo_num = rec_usart_fifo_num;
        rec_usart_fifo_num = 0;
        for (size_t i = 0; i < fifo_num; i++)
        {
            _ano_receive(rec_usart_fifo[i]);
        }
    }
}

void ano_init(void)
{
    ano.head = 0xAA;
    ano.d_addr = 0xFF;

    receive.head = 0xAA;
    receive.d_addr = 0xFF;
    receive.head_len = 1;
    receive.addr_len = 1;

    ano_cmd_list_init();

}

void ano_send_attitude(float roll, float pitch, float yaw)
{
    int16_t temp = 0;
    uint8_t cnt = 0;
    uint8_t buff[7] = {0};
    ano.id = 0x03;
    ano.len = 7;

    temp = (int)(roll*100);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    temp = (int)(pitch*100);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    temp = (int)((int)yaw*100);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);    
    buff[cnt] = 0;
    
    ano.data = buff;
    ano_send(&ano);
}

void ano_send_user_data(float data[10])
{
    int32_t temp = 0;
    uint8_t cnt = 0;
    uint8_t buff[40] = {0};
    ano.id = 0xf1;
    ano.len = 40;

    temp = (int32_t)(data[0]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(data[1]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(data[2]*10000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);


    temp = (int32_t)(-data[3]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(-data[4]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(-data[5]*10000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(data[6]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(data[7]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);

    temp = (int32_t)(data[8]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);
    
    temp = (int32_t)(data[9]*1000);
    buff[cnt++] = BYTE0(temp);
    buff[cnt++] = BYTE1(temp);
    buff[cnt++] = BYTE2(temp);
    buff[cnt++] = BYTE3(temp);
    
    ano.data = buff;
    ano_send(&ano);
}

void ano_send_sensor_imu(float ax, float ay, float az, float gx, float gy, float gz)
{
    int16_t temp = 0;
    uint8_t cnt = 0;
    uint8_t buff[13] = {0};
    ano.id = 0x01;
    ano.len = 13;
    
    int16_t accel_x = (int16_t)(ax * 9.81);
    int16_t accel_y = (int16_t)(ay * 9.81);
    int16_t accel_z = (int16_t)(az * 9.81);
    int16_t gyro_x = (int16_t)gx;
    int16_t gyro_y = (int16_t)gy;
    int16_t gyro_z = (int16_t)gz;    
    
    buff[cnt++] = BYTE0(accel_x);
    buff[cnt++] = BYTE1(accel_x);
    buff[cnt++] = BYTE0(accel_y);
    buff[cnt++] = BYTE1(accel_y);
    buff[cnt++] = BYTE0(accel_z);
    buff[cnt++] = BYTE1(accel_z);
    buff[cnt++] = BYTE0(gyro_x);
    buff[cnt++] = BYTE1(gyro_x);
    buff[cnt++] = BYTE0(gyro_y);
    buff[cnt++] = BYTE1(gyro_y);
    buff[cnt++] = BYTE0(gyro_z);
    buff[cnt++] = BYTE1(gyro_z);
    buff[cnt] = 0;
    
    ano.data = buff;
    ano_send(&ano);
}

void ano_send_sensor_mag(float mx, float my, float mz, float temperature)
{
    int16_t temp = 0;
    uint8_t cnt = 0;
    uint8_t buff[14] = {0};
    ano.id = 0x02;
    ano.len = 14;
    
    int16_t mag_x = (int16_t)mx;
    int16_t mag_y = (int16_t)my;
    int16_t mag_z = (int16_t)mz; 

    int16_t temp_1 = ((int16_t)temperature) * 100;
    
    buff[cnt++] = BYTE0(mag_x);
    buff[cnt++] = BYTE1(mag_x);
    buff[cnt++] = BYTE0(mag_y);
    buff[cnt++] = BYTE1(mag_y);
    buff[cnt++] = BYTE0(mag_z);
    buff[cnt++] = BYTE1(mag_z);
    cnt++;cnt++;cnt++;cnt++;
    buff[cnt++] = BYTE0(temp_1);
    buff[cnt++] = BYTE1(temp_1);
    buff[cnt++] = 0;
    buff[cnt++] = 0;
    
    ano.data = buff;
    ano_send(&ano);
}

void ano_seng_mag_cali(uint8_t cmd0, uint8_t cmd1, uint8_t cmd2, uint8_t cmd3, uint8_t cmd4, 
                       uint8_t cmd5, uint8_t cmd6, uint8_t cmd7, uint8_t cmd8, uint8_t cmd9)
{
    int16_t temp = 0;
    uint8_t cnt = 0;
    uint8_t buff[11] = {0x01,cmd0,cmd1,cmd2,cmd3,cmd4,
                        cmd5,cmd6,cmd7,cmd8,cmd9};
    ano.id = 0xA2;
    ano.len = 11;
    
    ano.data = buff;
    ano_send(&ano);
    
}
