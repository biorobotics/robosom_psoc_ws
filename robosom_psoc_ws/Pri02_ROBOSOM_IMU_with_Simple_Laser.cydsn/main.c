/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"
#include "stdio.h"
#include "usb_comm.h"

// Include BMI160 Library and PSOC HAL
#include "../Library/BMI160/bmi160.h"
#include "../Library/BMI160/bmi160_psoc.h"
#include "../Library/iCHT_Laser_Driver/iCHT_Defines.h"

struct bmi160_dev sensor;
struct bmi160_sensor_data accel;
struct bmi160_sensor_data gyro;
uint16_t step_count = 0;//stores the step counter value


int8_t imu_bmi160_init(void);
int8_t imu_bmi160_config(void);
int8_t imu_bmi160_enable_step_counter(void);
int8_t imu_bmi160_read_acc_gyo(void);
int8_t imu_bmi160_read_steps(void);

// USBUART
#define USBFS_DEVICE    (0u)
#define USBUART_BUFFER_SIZE (256u)
#define MAX_LED_VAL (10)
#define TRIGGER_DELAY_TIMEOUT (250) // Timeout set to 250ms 
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
void USBUART_user_echo(void);
void sys_clock_ms_callback(void);
uint16 USBUART_user_check_read(void);

uint16 read_count;
uint8 buffer_read[USBUART_BUFFER_SIZE];
uint8 serial_input = 0;
bool toggle_finished = true;
bool delayed_trigger = false;

// Exposure activate timestamps
uint32_t seconds = 0;
uint32_t set_secs = 0;
uint32_t set_us = 0;
uint32 t_e_us = 0;
uint32 t_e_s = 0;
uint32_t trigger_timestamps_msecs = 0;
uint32_t mcu_secs = 0;
uint32_t mcu_msecs = 0;

/* Laser/Shutter specific vars */
#define PWM_LASER_OFF (0)

enum shutter_laser_state {
    LSR_DISABLE = 0,
    LSR_ENABLE = 1
} laser_state;

enum shutter_active_state {
    NO_FRAME = 0,
    NEW_FRAME = 1
} shutter_state;

uint8_t frame_status = NO_FRAME;
uint8_t laser_enable = LSR_DISABLE;
uint8_t pulse_enabled = 1;
uint8_t trigger_frame = 1;
uint8_t pwm_led_val = MAX_LED_VAL/2;
bool started = false;
// Testing Functions
void print_imu_via_usbuart(void);
void print_exposure_timestamp(void);

float sys_clock_cur_us_in_ms = 0;
void sys_clock_us_callback(void); // 1ms callback interrupt function

// Interrupt handlers for the ximea camera
void Isr_shutter_handler(void); // Shutter Active interrupt handler

void Isr_second_handler(void); // Timestamp second counter
void toggle_laser_led(void);
void print_skipped_frame(uint8_t led_pwm, bool laser_enable);

int main(void)
{
    uint8_t led_test = 0;
    buffer[0] = 1;

    /* Sets up the GPIO interrupt and enables it */
    isr_EXPOSURE_ACT_StartEx(Isr_shutter_handler);
    /* Clears the pin interrupt */
    //Exposure_Active_ClearInterrupt();
    /* Clears the pending pin interrupt */
    isr_EXPOSURE_ACT_ClearPending();
    
    // USBUART Init
    init_usb_comm();
    USBUART_CDC_Init();

    // I2C Init
    I2C_1_Start();
    us_clock_Start();
    isr_time_StartEx(Isr_second_handler);
    
    // IMU BMI160 Init
    imu_bmi160_init();
    imu_bmi160_config();
    imu_bmi160_enable_step_counter();
    
    // Start system 1ms tick
    CySysTickStart();
    CySysTickSetCallback(0, sys_clock_ms_callback);
    CySysTickEnableInterrupt();
    
    // PWM Block Init
    LED_DRIVER_Start();
    //PWM_LASER_Start();
    //PWM_BUZZER_Start();
    //PWM_BUZZER_EN_Start();
   
    /* Turn off LEDs */
    Led_Red_Write(0);
    Led_Green_Write(0);
    Led_Blue_Write(0);
    //PLED_Write(1);
    CyGlobalIntEnable; /* Enable global interrupts. */
    

    /* For initial testing, establish USB communication before attempting to send first trigger frame */
    while (0u == USBUART_CDCIsReady())
    {
        usb_configuration_reinit();
    }
    // LL comment this out so that we can try to SWD debug without USB
    
    // Trigger first Ximea trigger pulse - Might want to link this to a button for manual triggering.
    Trig_Pulser_Start();

    
    struct ICHT_config config;

    //imu_bmi160_read_steps();
     
    int8_t status;
    ICHT_init_structs(&config);
    // Uncomment and comment above to enable ACC mode
    //ICHT_init_structs_ACCTEST(&config);
    status = ICHT_configure_driver(&config, &(config.regs));

    // Can print out individual registers on reg_list to debug. For now, printout error flags:
    if (status != ICHT_NO_ERR){
        sprintf((char *)buffer, "Failed register set! %d \n", status);
        usb_put_string((char8 *)buffer);
        CyDelay(10);

    }
    /*
    else {
        struct ICHT_Status_Regs_R regs; 
        regs = config.regs.STATUS;
        sprintf((char *)buffer, "Flags: CFGTIMO %x INITRAM %x LDKSAT1 %x LDKSAT2 %x MPAC1 %x MPAC2 %x MEMERR %x MONC1 %x MONC2 %x OSCERR %x OCV1 %x OCV2 %x OVT %x PDOVDD %x\n", 
        regs.CFGTIMO, regs.INITRAM, regs.LDKSAT1, regs.LDKSAT2, regs.MAPC1, regs.MAPC2, regs.MEMERR, regs.MONC1, regs.MONC2, regs.OSCERR, regs.OVC1, regs.OVC2, regs.OVT, regs.PDOVDD);
        usb_put_string((char8 *)buffer);      
        CyDelay(10);
    }
    */
    
    for(;;)
    {
        
        bool reconfigured = false;
        reconfigured = usb_configuration_reinit();
        
        imu_bmi160_read_acc_gyo();
        imu_bmi160_read_steps();

        //USBUART_user_echo();
        print_imu_via_usbuart();
        
        while (frame_status == NEW_FRAME) 
        {
            frame_status = NO_FRAME;
            print_exposure_timestamp();

        }

        if (USBUART_GetCount() > 0) 
        {
            read_count = USBUART_user_check_read();
            serial_input = buffer_read[0];
            //serial_input = usb_get_char(&reconfigured);
            //uint8_t prev_pwm = pwm_led_val;
            //bool prev_laser_enable = laser_enable;
            pwm_led_val = serial_input & 0b0001111; // Mask for first 4 bits
            // Current max LED val
            if (pwm_led_val > MAX_LED_VAL) pwm_led_val = MAX_LED_VAL;
            trigger_frame = (serial_input & 0b00010000) >> 4; // Mask for LSB of 1st byte, hardware frame trigger
            // Mask for 6th position bit, handles state of the laser. LED on when laser off.
            laser_enable = (serial_input & 0b00100000) >> 5;
            bool set_time = (serial_input & 0b01000000) >> 6;
            if (read_count == 9 && set_time) {
                int i;
                uint8_t *buffer_ptr = &(buffer_read[1]);
                set_us = 0;
                set_secs = 0;
                for (i = 0; i < 4; i++) 
                {
                    set_us = set_us << 8;
                    set_us |= *buffer_ptr;
                    buffer_ptr++;
                }
                for (i = 0; i < 4; i++)
                {
                    set_secs = set_secs << 8;
                    set_secs |= *buffer_ptr;
                    buffer_ptr++;
                }
                us_clock_Stop();
                if (set_us >= 1000000)
                {
                    // us counter must be less than 1 sec
                    set_us = 999999;
                }
                us_clock_WriteCounter(set_us);
                seconds = set_secs;
                us_clock_Enable();
            }
            if (trigger_frame){
                __disable_irq(); // Need to atomically check state of toggle + set delayed_trigger
                // Still possible signal could be missed while irq disabled?
                if (toggle_finished) {
                    toggle_finished = false;
                    toggle_laser_led();
                    trigger_timestamps_msecs = (mcu_msecs + TRIGGER_DELAY_TIMEOUT);
                    
                }
                else if (((int32_t)(trigger_timestamps_msecs - mcu_msecs)) <= 0)
                {
                    toggle_finished = false;
                    toggle_laser_led();
                    trigger_timestamps_msecs = (mcu_msecs + TRIGGER_DELAY_TIMEOUT);
                }
                else {
                    // Otherwise, skip this frame- Too soon. Report to PC frame skipped.
                    // If Laser/LED is changed here, OK? PC shouldn't send multiple commands before waiting for frame.
                    print_skipped_frame(pwm_led_val, prev_laser_enable);
                }
               __enable_irq();
            }


            //PWM_LASER_WriteCompare(pwm_laser_val);
            led_test++;
        }
        
        //Led_Red_Write((led_test >> 0) & 0x01);
        Led_Green_Write((led_test >> 1) & 0x01);
        Led_Blue_Write((led_test >> 2) & 0x01);   
    }
}

int8_t imu_bmi160_init(void)
{
    /* IMU init */
    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = bmi160_psoc_i2c_read;
    sensor.write = bmi160_psoc_i2c_write;
    sensor.delay_ms = bmi160_psoc_delay_ms;
    
    int8_t rslt = BMI160_OK;
    rslt = bmi160_soft_reset(&sensor);
    
    rslt = BMI160_OK;
    rslt = bmi160_init(&sensor);
    /*
    // After the above function call, 
    // accel and gyro parameters in the device structure 
    // are set with default values, 
    // found in the datasheet of the sensor
    */

   return rslt;
}

int8_t imu_bmi160_config(void)
{
    int8_t rslt = BMI160_OK;

    /* Select the Output data rate, range of accelerometer sensor */
    // sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_800HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    // sensor.accel_cfg.bw = BMI160_ACCEL_BW_RES_AVG8;
    #define IMU_ACC_SCALE 4

    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    //sensor.gyro_cfg.range = BMI160_GYRO_RANGE_1000_DPS;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
    // sensor.gyro_cfg.bw = BMI160_GYRO_BW_OSR2_MODE;
    #define IMU_GYO_SCALE 500

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);    

    return rslt;
}

int8_t imu_bmi160_enable_step_counter(void)
{
    int8_t rslt = BMI160_OK;
    uint8_t step_enable = 1;//enable the step counter

    rslt = bmi160_set_step_counter(step_enable,  &sensor);

    return rslt;

}

void toggle_laser_led(void)
{
    if (laser_enable == LSR_ENABLE) {
        LED_DRIVER_WriteCompare(PWM_LASER_OFF);
        Laser_En_1_Write(1);
        //light_status = LSR_DISABLE;
        //PWM_LASER_WriteCompare(PWM_LASER_OFF);
    }
    else if (laser_enable == LSR_DISABLE) {
        Laser_En_1_Write(0);
        LED_DRIVER_WriteCompare(pwm_led_val);
        //light_status = LSR_ENABLE;
        //PWM_LASER_WriteCompare(pwm_laser_val);
    }
    Trigger_Reg_Write(1);   
}

int8_t imu_bmi160_read_steps(void)
{
    int8_t rslt = BMI160_OK;
    rslt = bmi160_read_step_counter(&step_count,  &sensor);
    return rslt;
}

int8_t imu_bmi160_read_acc_gyo(void)
{
    int8_t rslt = BMI160_OK;

    /* To read only Accel data */
    //rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);

    /* To read only Gyro data */
    //rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &sensor);

    /* To read both Accel and Gyro data */
    //bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);

    /* To read Accel data along with time */
    //rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL) , &accel, NULL, &sensor);

    /* To read Gyro data along with time */
    //rslt = bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, &gyro, &sensor);

    /* To read both Accel and Gyro data along with time*/
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);

    return rslt;
}

uint16 USBUART_user_check_read(void) {
    /* Service USB CDC when device is configured. */
    if (0u != USBUART_GetConfiguration())
    {
        /* Check for input data from host. */
        if (0u != USBUART_DataIsReady())
        {
            /* Read received data and re-enable OUT endpoint. */
            count = USBUART_GetAll(buffer_read);
            return count;
        }
        
        return 0;
    }
    
    return -1;
}

void print_imu_via_usbuart(void)
{   
    uint32 t_us = (1000000 - us_clock_ReadCounter());//cur_time_us();//second_rounded_us();
    uint32 t_s = seconds;//cur_time_s();//uptime_s();
    while (0u == USBUART_CDCIsReady())
    {
    }

    // sprintf((char *)buffer, "%d\t%d\t%d\t%d\t%ld\t%ld\t%ld\r\n", step_count, accel.x, accel.y, accel.z, (long)(gyro.x + gyro_offset), (long)(gyro.y + gyro_offset), (long)(gyro.z + gyro_offset));
    sprintf((char *)buffer, "I:%lu\t%lu\t%d\t%d\t%d\t%d\t%d\t%d\r\n", t_us, t_s, accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
    //sprintf((char *)buffer, "%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n", (float)sys_clock_cur_ms/1000 + sys_clock_cur_us_in_ms, 
    //                            (float)accel.x/32768*IMU_ACC_SCALE*9.80665, (float)accel.y/32768*IMU_ACC_SCALE*9.80665,(float)accel.z/32768*IMU_ACC_SCALE*9.80665, 
    //                            (float)gyro.x/32768*IMU_GYO_SCALE, (float)gyro.y/32768*IMU_GYO_SCALE, (float)gyro.z/32768*IMU_GYO_SCALE);

    //count = sizeof(buffer);
    /* Send data back to host. */
    //USBUART_PutData(buffer, count);
    usb_put_string((char8 *)buffer);
}

void print_skipped_frame(uint8_t led_pwm, bool laser_enable)
{
    uint32 t_us = (1000000 - us_clock_ReadCounter());//cur_time_us();//second_rounded_us();
    uint32 t_s = seconds;//cur_time_s();//uptime_s();
    while (0u == USBUART_CDCIsReady())
    {
    }
    sprintf((char *)buffer, "E:%lu\t%lu\t%d\t%d\r\n", t_us, t_s, led_pwm, laser_enable);

    usb_put_string((char8 *)buffer);
}

void print_exposure_timestamp(void)
{   
    while (0u == USBUART_CDCIsReady())
    {
    }
    
    sprintf((char *)buffer, "E:%lu\t%lu\r\n", t_e_us, t_e_s);
    
    usb_put_string((char8 *)(buffer));
}


/**
 * @brief Interrupt handler for second counting 
 */
void Isr_second_handler(void)
{
    seconds = seconds + 1;
    isr_time_ClearPending();
    us_clock_ReadStatusRegister();
}

/**
 * @brief Interrupt handler for Shutter Active pin
 */

void Isr_shutter_handler(void)
{
    started = true;
    /* Set interrupt flag */
	frame_status = NEW_FRAME;
    t_e_us = (1000000 - us_clock_ReadCounter());
    t_e_s = seconds;
    toggle_finished = true;

    /* Clears the pending pin interrupt */
    isr_EXPOSURE_ACT_ClearPending();
    /* Clears the pin interrupt */
    //Exposure_Active_ClearInterrupt();
    /* Clears the pending pin interrupt */
    //isr_EXPOSURE_ACT_ClearPending();
}

// 1ms system tick callback interrupt function
void sys_clock_ms_callback(void){
    mcu_msecs ++; // increment ms counter by 1
}
/* [] END OF FILE */
