/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "boards.h"
#include "nrf_drv_gpiote.h"

#include "nrf.h"
#include "app_error.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_mpu.h"
#include "nrf_drv_mpu.h"
#include "MadgwickAHRS.h"

APP_TIMER_DEF(data_timer);
APP_TIMER_DEF(initialisation_delay_timer);
APP_TIMER_DEF(calibration_delay_timer);

#define CALIBRATION_CYCLE_LENGTH 10.0f
#define MOVEMENT_FREEDOM 20.0f

typedef struct {
double fs, st, ft;
}inclination_t;

int calibration_counter;
bool calibrated = false;
bool wait = true;
bool hunchback_detected;


/** 
 * Custom error handler to print error messages. 
 * This function overrides the weak app_error_fault_handler() in app_error.c
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_info_t * p_info = (error_info_t *)info;
    NRF_LOG_ERROR("Error 0x%04X (%d) at line %d in file: %s", p_info->err_code, p_info->err_code, p_info->line_num, p_info->p_file_name);
    
    NRF_LOG_FINAL_FLUSH();
    while(1);
}

/**@brief Timeout handler for the single shot timer.
 */
static void initialisation_delay_timer_handler(void * p_context)
{
    nrf_drv_gpiote_in_event_enable(BSP_BUTTON_0, true);
    nrf_gpio_pin_set(LED_1G);
    nrf_gpio_pin_set(LED_1R);
}

/**@brief Timeout handler for the single shot timer.
 */
static void calibration_delay_timer_handler(void * p_context)
{
    wait = false;
    nrf_gpio_pin_set(LED_1G);
    nrf_gpio_pin_set(LED_1R);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */

imu_data_t mpu[3];

inclination_t c_values[4];

EulerAngles angle[3];

static void data_timer_handler(void * p_context)
{
     NRF_LOG_INFO("\033[2J\033[;H");
     
     ret_code_t err_code;
     err_code = app_mpu_read_accel(1, 0x68, &mpu[0].a);
     APP_ERROR_CHECK(err_code);
     err_code = app_mpu_read_accel(1, 0x69, &mpu[1].a);
     APP_ERROR_CHECK(err_code);
     err_code = app_mpu_read_accel(0, 0x68, &mpu[2].a);
     APP_ERROR_CHECK(err_code);
     err_code = app_mpu_read_gyro(1, 0x68, &mpu[0].g);
     APP_ERROR_CHECK(err_code);
     err_code = app_mpu_read_gyro(1, 0x69, &mpu[1].g);
     APP_ERROR_CHECK(err_code);
     err_code = app_mpu_read_gyro(0, 0x68, &mpu[2].g);
     APP_ERROR_CHECK(err_code);


     NRF_LOG_INFO("GX:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].g.x));
     NRF_LOG_INFO("GY:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].g.y));
     NRF_LOG_INFO("GZ:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].g.z));
     NRF_LOG_INFO("AX:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].a.x));
     NRF_LOG_INFO("AY:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].a.y));
     NRF_LOG_INFO("AZ:    "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(mpu[0].a.z));

     MadgwickAHRSupdateIMU(&mpu[0]);
     MadgwickAHRSupdateIMU(&mpu[1]);
     MadgwickAHRSupdateIMU(&mpu[2]);
     angle[0] = ToEulerAngles(mpu[0].q);
     angle[1] = ToEulerAngles(mpu[1].q);
     angle[2] = ToEulerAngles(mpu[2].q);


     NRF_LOG_INFO("Roll:  "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(angle[0].roll));
     NRF_LOG_INFO("Pitch: "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(angle[0].pitch));
     NRF_LOG_INFO("Yaw:   "NRF_LOG_FLOAT_MARKER"",NRF_LOG_FLOAT(angle[0].yaw));
     
     if (wait) return;
     if(calibration_counter){
        calibration_counter--;
        c_values[0].fs += (angle[0].roll-angle[1].roll) / CALIBRATION_CYCLE_LENGTH;
        c_values[0].st += (angle[1].roll-angle[2].roll) / CALIBRATION_CYCLE_LENGTH;
        c_values[0].ft += (angle[0].roll-angle[2].roll) / CALIBRATION_CYCLE_LENGTH;
        c_values[1].fs += (angle[0].pitch-angle[1].pitch) / CALIBRATION_CYCLE_LENGTH;
        c_values[1].st += (angle[1].pitch-angle[2].pitch) / CALIBRATION_CYCLE_LENGTH;
        c_values[1].ft += (angle[0].pitch-angle[2].pitch) / CALIBRATION_CYCLE_LENGTH;
        if(!calibration_counter) nrf_gpio_pin_clear(LED_1G);
     }
     else {

        c_values[2].fs = angle[0].roll-angle[1].roll;
        c_values[2].st = angle[1].roll-angle[2].roll;
        c_values[2].ft = angle[0].roll-angle[2].roll;
        c_values[3].fs = angle[0].pitch-angle[1].pitch;
        c_values[3].st = angle[1].pitch-angle[2].pitch;
        c_values[3].ft = angle[0].pitch-angle[2].pitch;

        if(abs(c_values[2].fs-c_values[0].fs)>MOVEMENT_FREEDOM/2 ||
           abs(c_values[2].st-c_values[0].st)>MOVEMENT_FREEDOM/2 ||
           abs(c_values[2].ft-c_values[0].ft)>MOVEMENT_FREEDOM/2 ||
           abs(c_values[3].fs-c_values[1].fs)>MOVEMENT_FREEDOM/2 ||
           abs(c_values[3].st-c_values[1].st)>MOVEMENT_FREEDOM/2||
           abs(c_values[3].ft-c_values[1].ft)>MOVEMENT_FREEDOM/2){
          if(abs(c_values[2].fs-c_values[0].fs)>MOVEMENT_FREEDOM ||
             abs(c_values[2].st-c_values[0].st)>MOVEMENT_FREEDOM ||
             abs(c_values[2].ft-c_values[0].ft)>MOVEMENT_FREEDOM ||
             abs(c_values[3].fs-c_values[1].fs)>MOVEMENT_FREEDOM ||
             abs(c_values[3].st-c_values[1].st)>MOVEMENT_FREEDOM ||
             abs(c_values[3].ft-c_values[1].ft)>MOVEMENT_FREEDOM){
             if(!hunchback_detected){
             nrf_gpio_pin_set(LED_1G);
             nrf_gpio_pin_clear(LED_1R);
             nrf_gpio_pin_set(VIBRATION_MOTOR);
             hunchback_detected = true;
             }
          } else {
             nrf_gpio_pin_clear(LED_1G);
             nrf_gpio_pin_clear(LED_1R);
             if(hunchback_detected){ 
                nrf_gpio_pin_clear(VIBRATION_MOTOR);
                hunchback_detected = false;
             }
           }
        }
        else{
              nrf_gpio_pin_set(LED_1R);
              nrf_gpio_pin_clear(LED_1G);
           if(hunchback_detected){
              nrf_gpio_pin_clear(VIBRATION_MOTOR);
              hunchback_detected = false;
            }
        }
    }
}


static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&data_timer, APP_TIMER_MODE_REPEATED, data_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&initialisation_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, initialisation_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&calibration_delay_timer, APP_TIMER_MODE_SINGLE_SHOT, calibration_delay_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
    err_code = app_timer_start(data_timer, APP_TIMER_TICKS(20), NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(initialisation_delay_timer, APP_TIMER_TICKS(5000), NULL);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void mpu_init(uint8_t TWI_INSTANCE, uint8_t MPU_ADDRESS)
{
    uint32_t err_code;
    // Initiate MPU driver
    err_code = app_mpu_init(TWI_INSTANCE, MPU_ADDRESS);
    APP_ERROR_CHECK(err_code); // Check for errors in return value
    
    // Setup and configure the MPU with intial values
    app_mpu_config_t p_mpu_config = MPU_DEFAULT_CONFIG();
    p_mpu_config.smplrt_div = 99;   // Change sampelrate. Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV). 199 gives a sample rate of 5Hz
    p_mpu_config.accel_config.afs_sel = AFS_2G; // Set accelerometer full scale range to 2G
   
    err_code = app_mpu_config(TWI_INSTANCE, MPU_ADDRESS, &p_mpu_config); // Configure the MPU with above values
    APP_ERROR_CHECK(err_code); // Check for errors in return value


}



void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    if(hunchback_detected){
      nrf_gpio_pin_clear(VIBRATION_MOTOR);
    }
    ret_code_t err_code;
    nrf_gpio_pin_clear(LED_1G);
    nrf_gpio_pin_clear(LED_1R);
    wait = true;
    err_code = app_timer_start(calibration_delay_timer, APP_TIMER_TICKS(2000), NULL);
    APP_ERROR_CHECK(err_code);

    calibration_counter=CALIBRATION_CYCLE_LENGTH;
    c_values[0].fs = c_values[0].st = c_values[0].ft = 0;
    c_values[1].fs = c_values[1].st = c_values[1].ft = 0;

}

/**
 * @brief Function for configuring BUTTON_0 button for input,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    
    nrf_drv_mpu_init();
    nrf_drv_mpu_init1();

    mpu_init(1, 0x68);
    mpu_init(1, 0x69);
    mpu_init(0, 0x68);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
    out_config.init_state= GPIOTE_CONFIG_OUTINIT_High;

    err_code = nrf_drv_gpiote_out_init(LED_1G, &out_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_out_init(LED_1R, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(LED_1G);
    nrf_gpio_pin_clear(LED_1R);

    out_config.init_state= GPIOTE_CONFIG_OUTINIT_Low;

     err_code = nrf_drv_gpiote_out_init(VIBRATION_MOTOR, &out_config);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BSP_BUTTON_0, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    mpu[0].q.w = mpu[1].q.w = mpu[2].q.w = 1.0f;
    mpu[0].q.x = mpu[1].q.x = mpu[2].q.x = 0.0f;
    mpu[0].q.y = mpu[1].q.y = mpu[2].q.y = 0.0f;
    mpu[0].q.z = mpu[1].q.z = mpu[2].q.z = 0.0f;
    calibration_counter = 0;

    uint32_t err_code;
    bool erase_bonds;

    // Initialize.
    log_init();
    NRF_LOG_INFO("\033[2J\033[;H"); // Clear screen
    timers_init();
    
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);


    gpio_init();
    application_timers_start();
    // Enter main loop.
    while (true)
    {
        NRF_LOG_PROCESS();
        __WFI;
    }
}
