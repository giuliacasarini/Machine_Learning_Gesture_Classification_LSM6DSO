/******************************************************************************
* File Name:   sensor.c
*
* Description: This file implements the interface with the motion sensor, as 
*              well as the buffer management of the sensor data.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "sensor.h"

#include "LSM6DSOSensor.h"


#include "cy_fifo.h"

#include "cyhal.h"
#include "cybsp.h"
#include "cyabs_rtos.h"

/*******************************************************************************
* Constants
*******************************************************************************/
#define SENSOR_EVENT_BIT (1u << 2)

#ifdef CY_IMU_I2C_LSM6DSO
    #define IMU_I2C_MASTER_DEFAULT_ADDRESS  0
    #define IMU_I2C_FREQUENCY               1000000
#endif

#define SENSOR_FIFO_ITEM_SIZE (SENSOR_SAMPLE_SIZE / 2)
#define SENSOR_FIFO_POOL_SIZE (2*SENSOR_BATCH_SIZE * SENSOR_SAMPLE_SIZE)

#define SENSOR_SCAN_RATE       128
#define SENSOR_TIMER_FREQUENCY 100000
#define SENSOR_TIMER_PERIOD (SENSOR_TIMER_FREQUENCY/SENSOR_SCAN_RATE)
#define SENSOR_TIMER_PRIORITY  3

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Global timer used for getting data */
cyhal_timer_t sensor_timer;

/* Event set when data is done collecting */
cy_event_t sensor_event;
uint32_t sensor_event_bits = SENSOR_EVENT_BIT;

/* Circle buffer to store IMU data */
static cy_fifo_t sensor_fifo;
int8_t sensor_fifo_pool[SENSOR_FIFO_POOL_SIZE];

#ifdef CY_IMU_I2C_LSM6DSO
    /* LSM6DSO driver structures */
    mtb_lsm6dso_data_t data;

    LSM6DSO_t mMPU;
    /* I2C object for data transmission */
    cyhal_i2c_t i2c;
#endif


/*******************************************************************************
* Local Functions
*******************************************************************************/
void sensor_interrupt_handler(void *callback_arg, cyhal_timer_event_t event);
cy_rslt_t sensor_timer_init(void);

/*******************************************************************************
* Function Name: sensor_init
********************************************************************************
* Summary:
*   Initialize the sensor data.
*
* Parameters:
*     None
*
* Return:
*   The status of the initialization.
*******************************************************************************/
cy_rslt_t sensor_init(void)
{
    cy_rslt_t result;

    /* Setup the circular buffer for data storage */
    cy_fifo_init_static(&sensor_fifo, 
                        sensor_fifo_pool, 
                        sizeof(sensor_fifo_pool), 
                        SENSOR_FIFO_ITEM_SIZE);

#ifdef CY_IMU_I2C_LSM6DSO

    /* Configure the I2C mode, the address, and the data rate */
    cyhal_i2c_cfg_t i2c_config =
    {
            CYHAL_I2C_MODE_MASTER,
            IMU_I2C_MASTER_DEFAULT_ADDRESS,
            IMU_I2C_FREQUENCY
    };

    /* Initialize I2C for IMU communication */
    result = cyhal_i2c_init(&i2c, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }


    /* Configure the I2C */
    result = cyhal_i2c_configure(&i2c, &i2c_config);
    if(CY_RSLT_SUCCESS != result)
    {
        return result;
    }

    /* Initialize the IMU */

    LSM6DSO_Initialize(&mMPU,&i2c);
    begin(&mMPU);
    Enable_X(&mMPU);
    Enable_G(&mMPU);

#endif





    /* Timer for data collection */
    sensor_timer_init();

    /* Create an event that will be set when data collection is done */
    cy_rtos_init_event(&sensor_event);

    return result;
}

/*******************************************************************************
* Function Name: sensor_get_data
********************************************************************************
* Summary:
*   Return sensor data. This function is blocking. Only returns when data is
*   available.
*
* Parameters:
*     None
*
* Return:
*   Returns success if data returned.
*******************************************************************************/
cy_rslt_t sensor_get_data(void* sensor_data)
{
    /* Wait until there is 128 samples from the accelerometer and the
        * gyroscope in the circular buffer */
    cy_rtos_waitbits_event(&sensor_event, &sensor_event_bits, true, true, CY_RTOS_NEVER_TIMEOUT);

    /* Read data from the internal sensor FIFO */
    cy_fifo_read(&sensor_fifo, sensor_data, SENSOR_BATCH_SIZE * SENSOR_DATA_WIDTH);

    return CY_RSLT_SUCCESS;
}

/*******************************************************************************
* Function Name: sensor_interrupt_handler
********************************************************************************
* Summary:
*   Reads accelerometer and gyroscope data at 128 HZ. When there are 128 samples
*   an event is set so the data can be processed.
*
* Parameters:
*     callback_arg: not used
*     event: not used
*
*
*******************************************************************************/
void sensor_interrupt_handler(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;
    static int location = 0;

    /* Read data from IMU sensor */
    cy_rslt_t result;


#ifdef CY_IMU_I2C_LSM6DSO
    int32_t accelerometer[3];
    int32_t gyroscope[3];
    Get_X_Axes(&mMPU, accelerometer);
    Get_G_Axes(&mMPU, gyroscope);
    
	//printf("A]%hd:%hd:%hd\r\n", accelerometer[0],accelerometer[1],accelerometer[2]);
	//printf("G]%hd:%hd:%hd\r\n", gyroscope[0],gyroscope[1],gyroscope[2]);

	data.accel.x=accelerometer[0];
	data.accel.y=accelerometer[1];
	data.accel.z=accelerometer[2];

	data.gyro.x=gyroscope[0];
	data.gyro.y=gyroscope[1];
	data.gyro.z=gyroscope[2];


	//MPU6050_GetCompass(&mMPU,buffer);
	//printf("C]%hd:%hd:%hd\r\n", buffer[0],buffer[1],buffer[2]);

    //result = mtb_bmi160_read(&sensor_bmi160, &data);
	result = CY_RSLT_SUCCESS;

#endif

    if (CY_RSLT_SUCCESS != result)
    {
        CY_ASSERT(0);
    }

    /* Write data to the circle buffer */
    cy_fifo_write(&sensor_fifo, &data.accel, 1);
    cy_fifo_write(&sensor_fifo, &data.gyro, 1);

    /* Once there is enough data to feed the inference, run pre-processing */
    location++;
    if(location == SENSOR_BATCH_SIZE)
    {
        /* Reset the counter */
        location = 0;

        /* Once the event is set the data is processed */
        cy_rtos_setbits_event(&sensor_event, SENSOR_EVENT_BIT, true);
    }
}

/*******************************************************************************
* Function Name: sensor_timer_init
********************************************************************************
* Summary:
*   Sets up an interrupt that triggers at 128Hz.
*
* Parameters:
*     None
*
*
*******************************************************************************/
cy_rslt_t sensor_timer_init(void)
{
    cy_rslt_t rslt;
    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0,                 /* Timer compare value, not used */
        .period = SENSOR_TIMER_PERIOD,      /* Defines the timer period */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .is_continuous = true,              /* Run the timer indefinitely */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    rslt = cyhal_timer_init(&sensor_timer, NC, NULL);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Apply timer configuration such as period, count direction, run mode, etc. */
    rslt = cyhal_timer_configure(&sensor_timer, &timer_cfg);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Set the frequency of timer to 100KHz */
    rslt = cyhal_timer_set_frequency(&sensor_timer, SENSOR_TIMER_FREQUENCY);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&sensor_timer, sensor_interrupt_handler, NULL);
    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&sensor_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT, SENSOR_TIMER_PRIORITY, true);
    /* Start the timer with the configured settings */
    rslt = cyhal_timer_start(&sensor_timer);
    if (CY_RSLT_SUCCESS != rslt)
    {
        return rslt;
    }

    return CY_RSLT_SUCCESS;
}
