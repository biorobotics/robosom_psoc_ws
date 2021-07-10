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
uint16 count;
uint8 buffer[USBUART_BUFFER_SIZE];
void USBUART_user_check_init(void);
void USBUART_user_echo(void);
uint16 USBUART_user_check_read(void);

uint16 read_count;
uint8 buffer_read[USBUART_BUFFER_SIZE];
uint8 serial_input = 0;

// Exposure activate timestamps
uint32_t seconds = 0;
uint32 t_e_us = 0;
uint32 t_e_s = 0;

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
uint8_t light_status = LSR_DISABLE;
uint8_t pulse_enabled = 1;
uint8_t pwm_led_val = 50;

// Testing Functions
void print_imu_via_usbuart(void);
void print_exposure_timestamp(void);

float sys_clock_cur_us_in_ms = 0;
void sys_clock_us_callback(void); // 1ms callback interrupt function

// Interrupt handlers for the ximea camera
void Isr_shutter_handler(void); // Shutter Active interrupt handler

void Isr_second_handler(void); // Timestamp second counter

int main(void)
{
    uint8_t led_test = 0;
    buffer[0] = 1;
    
    /* Sets up the GPIO interrupt and enables it */
    isr_EXPOSURE_ACT_StartEx(Isr_shutter_handler);
    
    // USBUART Init
    init_usb_comm();
    USBUART_Start(USBFS_DEVICE, USBUART_5V_OPERATION);
    USBUART_CDC_Init();
    CyGlobalIntEnable; /* Enable global interrupts. */
    // I2C Init
    I2C_1_Start();
    us_clock_Start();
    isr_time_StartEx(Isr_second_handler);
    
    // IMU BMI160 Init
    imu_bmi160_init();
    imu_bmi160_config();
    imu_bmi160_enable_step_counter();
    
    // PWM Block Init
    LED_DRIVER_Start();
    //PWM_LASER_Start();
    //PWM_BUZZER_Start();
    //PWM_BUZZER_EN_Start();
   
    /* Turn off LEDs */
    Led_Red_Write(0);
    Led_Green_Write(0);
    Led_Blue_Write(0);
    PLED_Write(1);
  
    

    /* For initial testing, establish USB communication before attempting to send first trigger frame */
    while (0u == USBUART_CDCIsReady())
    {
        usb_configuration_reinit();
    }
    // LL comment this out so that we can try to SWD debug without USB
    
    // Trigger first Ximea trigger pulse - Might want to link this to a button for manual triggering.
    Trig_Pulser_Start();
    Trigger_Reg_Write(1);
    
    struct ICHT_config config;

    //imu_bmi160_read_steps();
    sprintf((char *)buffer, "Connected!\n");

    usb_put_string((char8 *)buffer);
    
    int8_t status;
    struct ICHT_reg_list reg_list;
    ICHT_init_structs(&config);
    // Uncomment and comment above to enable ACC mode
    //ICHT_init_structs_ACCTEST(&config);
    //status = ICHT_configure_driver(&config, &reg_list);
    status = ICHT_write_each_reg_conf(&config, &reg_list);
    // Can print out individual registers on reg_list to check functionality. For now, printout error flags:
    if (status != ICHT_NO_ERR){
        sprintf((char *)buffer, "Failed register set! %d \n", status);
        usb_put_string((char8 *)buffer);
        CyDelay(10);
        // Enable Laser Enable GPIO

    }
    else {
        struct ICHT_Status_Regs_R regs; regs = reg_list.STATUS;
        sprintf((char *)buffer, "Flags: CFGTIMO %x INITRAM %x LDKSAT1 %x LDKSAT2 %x MPAC1 %x MPAC2 %x MEMERR %x MONC1 %x MONC2 %x OSCERR %x OCV1 %x OCV2 %x OVT %x PDOVDD %x\n", 
        regs.CFGTIMO, regs.INITRAM, regs.LDKSAT1, regs.LDKSAT2, regs.MAPC1, regs.MAPC2, regs.MEMERR, regs.MONC1, regs.MONC2, regs.OSCERR, regs.OVC1, regs.OVC2, regs.OVT, regs.PDOVDD);
        usb_put_string((char8 *)buffer);      
        CyDelay(10);
    }
    
    for(;;)
    {
        
        bool reconfigured = false;
        reconfigured = usb_configuration_reinit();
        CyDelay(1000); // LL reduced this to 1 sec delay
        sprintf((char *)buffer, "TEST!\n");

        usb_put_string((char8 *)buffer);
       
        ICHT_init_test(&config);
        // Should catch for errors before trying to power on, for now I'll 

    
        
        // /* - // LL remote the comment to see if IMU is still functioning
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
        
            pwm_led_val = serial_input & 0b01111111; // Mask for last 7 bits
            pulse_enabled = serial_input & 0b10000000; // Mask for first bit. - Just disables pulse, can change behavior to disable/enable Laser.
            //PWM_LASER_WriteCompare(pwm_laser_val);
            led_test++;
        }
        
        //Led_Red_Write((led_test >> 0) & 0x01);
        Led_Green_Write((led_test >> 1) & 0x01);
        Led_Blue_Write((led_test >> 2) & 0x01);   

        // */ - // LL remote the comment to see if IMU is still functioning
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
    // For testing, alternating ever 1s
    /* If pulse mode enabled, alternate LED/LASER */ 
    if (pulse_enabled) {
       if (light_status == LSR_ENABLE) {
            light_status = LSR_DISABLE;
            LED_DRIVER_WriteCompare(PWM_LASER_OFF);
            Laser_En_1_Write(1);
        }
        else if (light_status == LSR_DISABLE) {
            light_status = LSR_ENABLE;
            LED_DRIVER_WriteCompare(pwm_led_val);
            Laser_En_1_Write(0);
        }
    }
    
    isr_time_ClearPending();
    us_clock_ReadStatusRegister();
}

/**
 * @brief Interrupt handler for Shutter Active pin
 */

void Isr_shutter_handler(void)
{
    /* Set interrupt flag */
	frame_status = NEW_FRAME;
    t_e_us = (1000000 - us_clock_ReadCounter());
    t_e_s = seconds;
    /* If pulse mode enabled, alternate LED/LASER */ 
    /*
    if (pulse_enabled) {
       if (light_status == LSR_ENABLE) {
            light_status = LSR_DISABLE;
            LED_DRIVER_WriteCompare(PWM_LASER_OFF);
            Laser_En_1_Write(1);
            //PWM_LASER_WriteCompare(PWM_LASER_OFF);
        }
        else if (light_status == LSR_DISABLE) {
            light_status = LSR_ENABLE;
            LED_DRIVER_WriteCompare(pwm_led_val);
            Laser_En_1_Write(0);
            //PWM_LASER_WriteCompare(pwm_laser_val);
        }
    }
    */
        
    /* Trigger a new camera frame */
    Trigger_Reg_Write(1);
    
    /* Clears the pin interrupt */
    Exposure_Active_ClearInterrupt();
    /* Clears the pending pin interrupt */
    isr_EXPOSURE_ACT_ClearPending();

}
/* [] END OF FILE */
