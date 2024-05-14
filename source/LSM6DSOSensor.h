/**
 ******************************************************************************
 * @file    LSM6DSOSensor.h
 * @author  SRA
 * @version V1.0.0
 * @date    February 2019
 * @brief   Abstract Class of an LSM6DSO Inertial Measurement Unit (IMU) 3 axes
 *          sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Prevent recursive inclusion -----------------------------------------------*/

#ifndef __LSM6DSOSensor_H__
#define __LSM6DSOSensor_H__


/* Includes ------------------------------------------------------------------*/

#include "cyhal.h"
#include "cyhal_i2c.h"
#include "cybsp.h"
#include "lsm6dso_reg.h"

/* Defines -------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
#ifndef MSBFIRST
#define MSBFIRST SPI_MSBFIRST
#endif
#endif

#define LSM6DSO_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSO_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSO_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSO_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSO_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define LSM6DSO_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSO_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSO_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSO_GYRO_SENSITIVITY_FS_2000DPS  70.000f


/* Typedefs ------------------------------------------------------------------*/

typedef enum
{
  LSM6DSO_OK = 0,
  LSM6DSO_ERROR =-1
} LSM6DSOStatusTypeDef;

typedef enum
{
  LSM6DSO_INT1_PIN,
  LSM6DSO_INT2_PIN,
} LSM6DSO_SensorIntPin_t;

typedef enum
{
  LSM6DSO_ACC_HIGH_PERFORMANCE_MODE,
  LSM6DSO_ACC_LOW_POWER_NORMAL_MODE,
  LSM6DSO_ACC_ULTRA_LOW_POWER_MODE
} LSM6DSO_ACC_Operating_Mode_t;

typedef enum
{
  LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE,
  LSM6DSO_GYRO_LOW_POWER_NORMAL_MODE
} LSM6DSO_GYRO_Operating_Mode_t;

typedef struct
{
  unsigned int FreeFallStatus : 1;
  unsigned int TapStatus : 1;
  unsigned int DoubleTapStatus : 1;
  unsigned int WakeUpStatus : 1;
  unsigned int StepStatus : 1;
  unsigned int TiltStatus : 1;
  unsigned int D6DOrientationStatus : 1;
  unsigned int SleepStatus : 1;
} LSM6DSO_Event_Status_t;

struct lsm6dso_sensor_data
{
    /*! X-axis sensor data */
    int16_t x;

    /*! Y-axis sensor data */
    int16_t y;

    /*! Z-axis sensor data */
    int16_t z;

    /*! sensor time */
    uint32_t sensortime;
};

typedef struct
{
    /** Accelerometer data */
    struct lsm6dso_sensor_data accel;
    /** Gyroscope data */
    struct lsm6dso_sensor_data gyro;
} mtb_lsm6dso_data_t;


typedef struct{
	  cyhal_i2c_t *dev_i2c; ///< Pointer to I2C bus interface
	  //int32_t _sensorID;
    uint8_t address;
    lsm6dso_odr_xl_t acc_odr;
    lsm6dso_odr_g_t gyro_odr;
    uint8_t acc_is_enabled;
    uint8_t gyro_is_enabled;
    lsm6dso_ctx_t reg_ctx;
} LSM6DSO_t;

typedef bool FunctionalState;

void LSM6DSO_Initialize(LSM6DSO_t *pdev, cyhal_i2c_t *pi2c_dev);
LSM6DSOStatusTypeDef begin(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef end(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef ReadID(LSM6DSO_t *pdev, uint8_t *Id);
LSM6DSOStatusTypeDef Enable_X(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Disable_X(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Get_X_Sensitivity(LSM6DSO_t *pdev, float *Sensitivity);
LSM6DSOStatusTypeDef Get_X_ODR(LSM6DSO_t *pdev, float *Odr);
LSM6DSOStatusTypeDef Set_X_ODR(LSM6DSO_t *pdev, float Odr);
LSM6DSOStatusTypeDef Set_X_ODR_With_Mode(LSM6DSO_t *pdev, float Odr, LSM6DSO_ACC_Operating_Mode_t Mode);
LSM6DSOStatusTypeDef Get_X_FS(LSM6DSO_t *pdev, int32_t *FullScale);
LSM6DSOStatusTypeDef Set_X_FS(LSM6DSO_t *pdev, int32_t FullScale);
LSM6DSOStatusTypeDef Get_X_AxesRaw(LSM6DSO_t *pdev, int16_t *Value);
LSM6DSOStatusTypeDef Get_X_Axes(LSM6DSO_t *pdev, int32_t *Acceleration);

LSM6DSOStatusTypeDef Enable_G(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Disable_G(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Get_G_Sensitivity(LSM6DSO_t *pdev, float *Sensitivity);
LSM6DSOStatusTypeDef Get_G_ODR(LSM6DSO_t *pdev, float *Odr);
LSM6DSOStatusTypeDef Set_G_ODR(LSM6DSO_t *pdev, float Odr);
LSM6DSOStatusTypeDef Set_G_ODR_With_Mode(LSM6DSO_t *pdev, float Odr, LSM6DSO_GYRO_Operating_Mode_t Mode);
LSM6DSOStatusTypeDef Get_G_FS(LSM6DSO_t *pdev, int32_t *FullScale);
LSM6DSOStatusTypeDef Set_G_FS(LSM6DSO_t *pdev, int32_t FullScale);
LSM6DSOStatusTypeDef Get_G_AxesRaw(LSM6DSO_t *pdev, int16_t *Value);
LSM6DSOStatusTypeDef Get_G_Axes(LSM6DSO_t *pdev, int32_t *AngularRate);

LSM6DSOStatusTypeDef Read_Reg(LSM6DSO_t *pdev, uint8_t reg, uint8_t *Data);
LSM6DSOStatusTypeDef Write_Reg(LSM6DSO_t *pdev, uint8_t reg, uint8_t Data);
LSM6DSOStatusTypeDef Set_Interrupt_Latch(LSM6DSO_t *pdev, uint8_t Status);
LSM6DSOStatusTypeDef Set_Interrupt_Polarity(LSM6DSO_t *pdev, uint8_t Status);
LSM6DSOStatusTypeDef Set_Interrupt_PinMode(LSM6DSO_t *pdev, uint8_t Status);

LSM6DSOStatusTypeDef Enable_Free_Fall_Detection(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_Free_Fall_Detection(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Set_Free_Fall_Threshold(LSM6DSO_t *pdev, uint8_t Threshold);
LSM6DSOStatusTypeDef Set_Free_Fall_Duration(LSM6DSO_t *pdev, uint8_t Duration);

LSM6DSOStatusTypeDef Enable_Pedometer(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Disable_Pedometer(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Get_Step_Count(LSM6DSO_t *pdev, uint16_t *StepCount);
LSM6DSOStatusTypeDef Step_Counter_Reset(LSM6DSO_t *pdev);

LSM6DSOStatusTypeDef Enable_Tilt_Detection(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_Tilt_Detection(LSM6DSO_t *pdev);

LSM6DSOStatusTypeDef Enable_Wake_Up_Detection(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_Wake_Up_Detection(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Set_Wake_Up_Threshold(LSM6DSO_t *pdev, uint8_t Threshold);
LSM6DSOStatusTypeDef Set_Wake_Up_Duration(LSM6DSO_t *pdev, uint8_t Duration);

LSM6DSOStatusTypeDef Enable_Single_Tap_Detection(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_Single_Tap_Detection(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Enable_Double_Tap_Detection(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_Double_Tap_Detection(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Set_Tap_Threshold(LSM6DSO_t *pdev, uint8_t Threshold);
LSM6DSOStatusTypeDef Set_Tap_Shock_Time(LSM6DSO_t *pdev, uint8_t Time);
LSM6DSOStatusTypeDef Set_Tap_Quiet_Time(LSM6DSO_t *pdev, uint8_t Time);
LSM6DSOStatusTypeDef Set_Tap_Duration_Time(LSM6DSO_t *pdev, uint8_t Time);

LSM6DSOStatusTypeDef Enable_6D_Orientation(LSM6DSO_t *pdev, LSM6DSO_SensorIntPin_t IntPin);
LSM6DSOStatusTypeDef Disable_6D_Orientation(LSM6DSO_t *pdev);
LSM6DSOStatusTypeDef Set_6D_Orientation_Threshold(LSM6DSO_t *pdev, uint8_t Threshold);
LSM6DSOStatusTypeDef Get_6D_Orientation_XL(LSM6DSO_t *pdev, uint8_t *XLow);
LSM6DSOStatusTypeDef Get_6D_Orientation_XH(LSM6DSO_t *pdev, uint8_t *XHigh);
LSM6DSOStatusTypeDef Get_6D_Orientation_YL(LSM6DSO_t *pdev, uint8_t *YLow);
LSM6DSOStatusTypeDef Get_6D_Orientation_YH(LSM6DSO_t *pdev, uint8_t *YHigh);
LSM6DSOStatusTypeDef Get_6D_Orientation_ZL(LSM6DSO_t *pdev, uint8_t *ZLow);
LSM6DSOStatusTypeDef Get_6D_Orientation_ZH(LSM6DSO_t *pdev, uint8_t *ZHigh);

LSM6DSOStatusTypeDef Get_X_DRDY_Status(LSM6DSO_t *pdev, uint8_t *Status);
LSM6DSOStatusTypeDef Get_X_Event_Status(LSM6DSO_t *pdev, LSM6DSO_Event_Status_t *Status);
LSM6DSOStatusTypeDef Set_X_SelfTest(LSM6DSO_t *pdev, uint8_t Status);

LSM6DSOStatusTypeDef Get_G_DRDY_Status(LSM6DSO_t *pdev, uint8_t *Status);
LSM6DSOStatusTypeDef Set_G_SelfTest(LSM6DSO_t *pdev, uint8_t Status);

LSM6DSOStatusTypeDef Get_FIFO_Num_Samples(LSM6DSO_t *pdev, uint16_t *NumSamples);
LSM6DSOStatusTypeDef Get_FIFO_Full_Status(LSM6DSO_t *pdev, uint8_t *Status);
LSM6DSOStatusTypeDef Set_FIFO_INT1_FIFO_Full(LSM6DSO_t *pdev, uint8_t Status);
LSM6DSOStatusTypeDef Set_FIFO_Watermark_Level(LSM6DSO_t *pdev, uint16_t Watermark);
LSM6DSOStatusTypeDef Set_FIFO_Stop_On_Fth(LSM6DSO_t *pdev, uint8_t Status);
LSM6DSOStatusTypeDef Set_FIFO_Mode(LSM6DSO_t *pdev, uint8_t Mode);
LSM6DSOStatusTypeDef Get_FIFO_Tag(LSM6DSO_t *pdev, uint8_t *Tag);
LSM6DSOStatusTypeDef Get_FIFO_Data(LSM6DSO_t *pdev, uint8_t *Data);
LSM6DSOStatusTypeDef Get_FIFO_X_Axes(LSM6DSO_t *pdev, int32_t *Acceleration);
LSM6DSOStatusTypeDef Set_FIFO_X_BDR(LSM6DSO_t *pdev, float Bdr);
LSM6DSOStatusTypeDef Get_FIFO_G_Axes(LSM6DSO_t *pdev, int32_t *AngularVelocity);
LSM6DSOStatusTypeDef Set_FIFO_G_BDR(LSM6DSO_t *pdev, float Bdr);
  
/**
 * @brief Utility function to read data.
 * @param  pBuffer: pointer to data to be read.
 * @param  RegisterAddr: specifies internal address register to be read.
 * @param  NumByteToRead: number of bytes to be read.
 * @retval 0 if ok, an error code otherwise.
 */
uint8_t IO_Read(LSM6DSO_t *pdev, uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead);
/**
 * @brief Utility function to write data.
 * @param  pBuffer: pointer to data to be written.
 * @param  RegisterAddr: specifies internal address register to be written.
 * @param  NumByteToWrite: number of bytes to write.
 * @retval 0 if ok, an error code otherwise.
 */
uint8_t IO_Write(LSM6DSO_t *pdev, uint8_t* pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite);

LSM6DSOStatusTypeDef Set_X_ODR_When_Enabled(LSM6DSO_t *pdev, float Odr);
LSM6DSOStatusTypeDef Set_X_ODR_When_Disabled(LSM6DSO_t *pdev, float Odr);
LSM6DSOStatusTypeDef Set_G_ODR_When_Enabled(LSM6DSO_t *pdev, float Odr);
LSM6DSOStatusTypeDef Set_G_ODR_When_Disabled(LSM6DSO_t *pdev, float Odr);


#ifdef __cplusplus
 extern "C" {
#endif
int32_t LSM6DSO_io_write(void *pdev, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite );
int32_t LSM6DSO_io_read(void *pdev, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead );
#ifdef __cplusplus
  }
#endif

#endif
