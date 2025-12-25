/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
extern "C" {
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "gcs.h"
#include "icm20602.h"
#include "ist8310.h"
#include "Params.h"
}
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "AttitudeEstimator.h"
#include "Ellipsoid_fit.h"
#include "MagCalibration.h"
#include "AccelCalibration.h"
#include "AHRS_Math.h"

/* Private variables ---------------------------------------------------------*/
struct ist8310_t _mag = {};
struct icm20602_t _imu = {};
float _euler[3] = {};
float _quat[4] = {};
float _accel_m_s2[3] = {};
float _gyro_rad[3] = {};
float _mag_gauss[3] = {};
float _user_datas[10] = {};
//Calibration
struct sensor_cali_t _sensor_need_cali = {}; 
bool _off_board_mag_cali = false;
void command_response(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* MCU Configuration--------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();
    /* Configure the system clock */
    SystemClock_Config();
    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();

    ano_init();
    param_init();
    icm20602_init();
    ist8310_init();
    LOG_I("IMU init ok!");

    /* Infinite loop */
    while (1)
    {
		if(_scheduler.cnt_200Hz >= 5)
		{
			_scheduler.cnt_200Hz = 0;
            ist8310_update();
            icm20602_update();
            get_icm20602_accel_gyro(&_imu);
            get_ist8310(&_mag);
            
            _gyro_rad[0] = _imu.gx_cali;
            _gyro_rad[1] = _imu.gy_cali; 
            _gyro_rad[2] = _imu.gz_cali;
            
            _accel_m_s2[0] = _imu.ax_cali * CONSTANTS_ONE_G; 
            _accel_m_s2[1] = _imu.ay_cali * CONSTANTS_ONE_G; 
            _accel_m_s2[2] = _imu.az_cali * CONSTANTS_ONE_G;
            
            //cover from milligauss to gauss
            _mag_gauss[0] = _mag.mx_cali * 0.001f;
            _mag_gauss[1] = _mag.my_cali * 0.001f;
            _mag_gauss[2] = _mag.mz_cali * 0.001f;
            
            attitude_estimator_update(_gyro_rad, _accel_m_s2, _mag_gauss, 0.005);
            get_euler(_euler);
            
        }
        if(_scheduler.cnt_100Hz>= 10)
        {
            _scheduler.cnt_100Hz = 0;
            ano_cycle_run();
            command_response();
        }
        if(_scheduler.cnt_50Hz>= 20)
        {
            _scheduler.cnt_50Hz = 0;
        }
        
		if(_scheduler.cnt_20Hz >= 50)
		{
			_scheduler.cnt_20Hz = 0;
			ano_send_attitude(_euler[0], _euler[1], _euler[2]);
			ano_send_sensor_imu(_imu.ax, _imu.ay, _imu.az, _imu.gx, _imu.gy, _imu.gz);
			ano_send_sensor_mag(_mag.mx, _mag.my, _mag.mz, _imu.temp);
			
			_user_datas[0] = _imu.gx_cali;
			_user_datas[1] = _imu.gy_cali;
			_user_datas[2] = _imu.gz_cali;
            
            float acc_filter[3] = {0};
			get_gyro_bias(&_user_datas[3]);
            get_accel_filter(acc_filter);
            _user_datas[6] = _accel_m_s2[0];
            _user_datas[7] = acc_filter[0];
			ano_send_user_data(_user_datas);
        }
        
        if(_scheduler.cnt_1Hz >= 1000)
        {
            _scheduler.cnt_1Hz = 0;
            led_green_toggle();
        }
    }
}

void command_response(void)
{
    if(_sensor_need_cali.acc != 0)
    {
        do_accel_calibration(&_imu.ax);
    }
    if(_sensor_need_cali.gyro)
    {
        do_gyro_calibration(&_imu.gx);
    }
    //Do mag calibration
    if(_sensor_need_cali.mag)
    {
        do_mag_calibration(&_imu.ax, &_imu.gx, &_mag.mx);
    }
    else
    {
        if(_sensor_need_cali.cancel_mag)
            _sensor_need_cali.cancel_mag = false;
    }
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
