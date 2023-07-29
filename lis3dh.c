/**
 * @file lis3dh.c
 * @brief LIS3DH Accelerometer lib
 * @author Jose Martel & ChatGPT
 * @date 30/06/2023
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "hal/gpio_types.h"
#include "hal/i2c_types.h"
#include "lis3dh.h"

/*---------------------------------------------------------------------------*/
/*******************************************************************************
* Function Name		: SPI_MASTER_INIT
* Description		: Function to initialized SPI for LIS3DH				
* Input			: None
* Output		: None
* Return		: Status
*******************************************************************************/
// esp_err_t SPI_MASTER_INIT(void)
// {
//     esp_err_t ret;

//     // Configurar la configuración del controlador SPI
//     spi_bus_config_t bus_config = {
//         .miso_io_num = SPI_PIN_NUM_MISO,
//         .mosi_io_num = SPI_PIN_NUM_MOSI,
//         .sclk_io_num = SPI_PIN_NUM_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .max_transfer_sz = 0,
//     };

//     // Inicializar el controlador SPI
//     ret = spi_bus_initialize(HSPI_HOST, &bus_config, 0);
//     if (ret != ESP_OK) {
//         printf("Error inicializando el bus SPI: %s\n", esp_err_to_name(ret));
//         return ret;
//     }

//     // Configurar la configuración del dispositivo SPI
//     spi_device_interface_config_t device_config = {
//         .command_bits = 0,
//         .address_bits = 8,
//         .dummy_bits = 0,
//         .mode = 0,
//         .duty_cycle_pos = 0,
//         .cs_ena_pretrans = 0,
//         .cs_ena_posttrans = 0,
//         .clock_speed_hz = SPI_CLOCK_SPEED,
//         .input_delay_ns = 0,
//         .spics_io_num = SPI_PIN_NUM_CS,
//         .flags = 0,
//         .queue_size = 1,
//         .pre_cb = NULL,
//         .post_cb = NULL,
//     };

//     // Agregar el dispositivo SPI al bus SPI
//     ret = spi_bus_add_device(HSPI_HOST, &device_config, NULL);
//     if (ret != ESP_OK) {
//         printf("Error agregando el dispositivo SPI: %s\n", esp_err_to_name(ret));
//         return ret;
//     }

//     return ESP_OK;
// }

/*******************************************************************************
* Function Name		: I2C_MASTER_INIT
* Description		: Function to initialized I2C for LIS3DH				
* Input			: None
* Output		: None
* Return		: Status
*******************************************************************************/
esp_err_t I2C_MASTER_INIT(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/*******************************************************************************
* Function Name		: LIS3DH_ReadAccReg
* Description		: Generic Reading function. It must be fullfilled with either			
* Input			: Register address, Data to save 
* Output		: None
* Return		: Status
*******************************************************************************/
u8_t LIS3DH_ReadAccReg(u8_t Reg, u8_t *Data) {

  bool success = false;
  uint8_t value[6];

  success = !(bool)i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DH_MEMS_I2C_ADDRESS, &Reg, 1, value, 6, 1000 / portTICK_PERIOD_MS);
  memcpy(Data, value, 6);

	if (!success) {
		// printf("Error reading LIS3DH %02x register\n", Reg);
		return success;
	} else {
		// LOG_DBG("Getting LIS3DH %02x register OK\n", Reg);
		return success;
	}

}

/*******************************************************************************
* Function Name		: LIS3DH_ReadReg
* Description		: Generic Reading function. It must be fullfilled with either			
* Input			: Register address, Data to save 
* Output		: None
* Return		: Status
*******************************************************************************/
u8_t LIS3DH_ReadReg(u8_t Reg, u8_t *Data) {

//   //To be completed with either I2c or SPI reading function
//   //i.e. *Data = SPI_Mems_Read_Reg( Reg );  

  bool success = false;
  uint8_t value[1] = {0};

  success = !(bool)i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DH_MEMS_I2C_ADDRESS, &Reg, sizeof(value), value, 1, 1000 / portTICK_PERIOD_MS);
  Data[0] = value[0];

	if (!success) {
		// printf("Error reading LIS3DH %02x register\n", Reg);
		return success;
	} else {
		// LOG_DBG("Getting LIS3DH %02x register OK\n", Reg);
		return success;
	}

}


/*******************************************************************************
* Function Name		: LIS3DH_WriteReg
* Description		: Generic Writing function. It must be fullfilled with either
*			: I2C or SPI writing function
* Input			: Register Address, Data to be written
* Output		: None
* Return		: None
*******************************************************************************/
u8_t LIS3DH_WriteReg(u8_t WriteAddr, u8_t Data) {
  //To be completed with either I2c or SPI writing function
  //i.e. SPI_Mems_Write_Reg(WriteAddr, Data);
    
	esp_err_t success = false;
  uint8_t send[2] = {WriteAddr, Data};

  // success = i2c_master_write_to_device(I2C_MASTER_NUM, LIS3DH_MEMS_I2C_ADDRESS, cmd, sizeof(cmd), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
  i2c_cmd_handle_t cmd;
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (LIS3DH_MEMS_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1);
  i2c_master_write(cmd, send,sizeof(send),1);
  i2c_master_stop(cmd);   
  success = !(bool)i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS);
  // if (success != 0) ESP_LOGE("I2C","%d:%s",success,esp_err_to_name(success));
  i2c_cmd_link_delete(cmd);

	if (!success) {
		return 0;
	} else {
		return 1;
	}

}

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : LIS3DH_GetWHO_AM_I
* Description    : Read identification code by WHO_AM_I register
* Input          : Char to empty by Device identification Value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS3DH_GetWHO_AM_I(u8_t* val){
  
  if( !LIS3DH_ReadReg(LIS3DH_WHO_AM_I, val) ){
		printf("Error reading LIS3DH %02x register\n", LIS3DH_WHO_AM_I);
    return MEMS_ERROR;
  }
    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetStatusAUX
* Description    : Read the AUX status register
* Input          : Char to empty by status register buffer
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusAUX(u8_t* val) {
  
  if( !LIS3DH_ReadReg(LIS3DH_STATUS_AUX, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}



/*******************************************************************************
* Function Name  : LIS3DH_GetStatusAUXBIT
* Description    : Read the AUX status register BIT
* Input          : LIS3DH_STATUS_AUX_321OR, LIS3DH_STATUS_AUX_3OR, LIS3DH_STATUS_AUX_2OR, LIS3DH_STATUS_AUX_1OR,
                   LIS3DH_STATUS_AUX_321DA, LIS3DH_STATUS_AUX_3DA, LIS3DH_STATUS_AUX_2DA, LIS3DH_STATUS_AUX_1DA
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusAUXBit(u8_t statusBIT, u8_t* val) {
  u8_t value = 0;  
  
  if( !LIS3DH_ReadReg(LIS3DH_STATUS_AUX, &value) )
    return MEMS_ERROR;
  
  if(statusBIT == LIS3DH_STATUS_AUX_321OR){
    if(value &= LIS3DH_STATUS_AUX_321OR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_3OR){
    if(value &= LIS3DH_STATUS_AUX_3OR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }     
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_2OR){
    if(value &= LIS3DH_STATUS_AUX_2OR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }    
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_1OR){
    if(value &= LIS3DH_STATUS_AUX_1OR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_321DA){
    if(value &= LIS3DH_STATUS_AUX_321DA) {     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_3DA){
    if(value &= LIS3DH_STATUS_AUX_3DA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_2DA){
    if(value &= LIS3DH_STATUS_AUX_2DA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_STATUS_AUX_1DA){
    if(value &= LIS3DH_STATUS_AUX_1DA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }  
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetODR
* Description    : Sets LIS3DH Output Data Rate
* Input          : Output Data Rate
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetODR(LIS3DH_ODR_t ov){
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
    return MEMS_ERROR;
  
  value &= 0x0f;
  value |= ov<<LIS3DH_ODR_BIT;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetTemperature
* Description    : Sets LIS3DH Output Temperature
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Note           : For Read Temperature by LIS3DH_OUT_AUX_3, LIS3DH_SetADCAux and LIS3DH_SetBDU 
				   functions must be ENABLE
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetTemperature(State_t state){
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_TEMP_CFG_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;
  value |= state<<LIS3DH_TEMP_EN;
  
  if( !LIS3DH_WriteReg(LIS3DH_TEMP_CFG_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetADCAux
* Description    : Sets LIS3DH Output ADC
* Input          : MEMS_ENABLE, MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetADCAux(State_t state){
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_TEMP_CFG_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= state<<LIS3DH_ADC_PD;
  
  if( !LIS3DH_WriteReg(LIS3DH_TEMP_CFG_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetAuxRaw
* Description    : Read the Aux Values Output Registers
* Input          : Buffer to empty
* Output         : Aux Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAuxRaw(LIS3DH_Aux123Raw_t* buff) {
  u8_t valueL = 0;
  u8_t valueH = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_1_L, &valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_1_H, &valueH) )
    return MEMS_ERROR;
  
  buff->AUX_1 = (u16_t)( (valueH << 8) | valueL )/16;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_2_L, &valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_2_H, &valueH) )
    return MEMS_ERROR;
  
  buff->AUX_2 = (u16_t)( (valueH << 8) | valueL )/16;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_3_L, &valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_3_H, &valueH) )
    return MEMS_ERROR;
  
  buff->AUX_3 = (u16_t)( (valueH << 8) | valueL )/16;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS3DH_GetTempRaw
* Description    : Read the Temperature Values by AUX Output Registers OUT_3_H
* Input          : Buffer to empty
* Output         : Temperature Values Registers buffer
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetTempRaw(i8_t* buff) {
  u8_t valueL = 0;
  u8_t valueH = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_3_L, &valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_3_H, &valueH) )
    return MEMS_ERROR;
  
  *buff = (i8_t)( valueH );
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS3DH_SetMode
* Description    : Sets LIS3DH Operating Mode
* Input          : Modality (LIS3DH_NORMAL, LIS3DH_LOW_POWER, LIS3DH_POWER_DOWN)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetMode(LIS3DH_Mode_t md) {
  u8_t value = 0;
  u8_t value2 = 0;
  static   u8_t ODR_old_value;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value2) )
    return MEMS_ERROR;
  
  if((value & 0xF0)==0) 
    value = value | (ODR_old_value & 0xF0); //if it comes from POWERDOWN  
  
  switch(md) {
    
  case LIS3DH_POWER_DOWN:
    ODR_old_value = value;
    value &= 0x0F;
    break;
    
  case LIS3DH_NORMAL:
    value &= 0xF7;
    value |= (MEMS_RESET<<LIS3DH_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_SET<<LIS3DH_HR);   //set HighResolution_BIT
    break;
    
  case LIS3DH_LOW_POWER:		
    value &= 0xF7;
    value |=  (MEMS_SET<<LIS3DH_LPEN);
    value2 &= 0xF7;
    value2 |= (MEMS_RESET<<LIS3DH_HR); //reset HighResolution_BIT
    break;
    
  default:
    return MEMS_ERROR;
  }
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
    return MEMS_ERROR;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value2) )
    return MEMS_ERROR;  
  
  return MEMS_SUCCESS;
}

/*******************************************************************************
* Function Name  : LIS3DH_SetAxis
* Description    : Enable/Disable LIS3DH Axis
* Input          : LIS3DH_X_ENABLE/DISABLE | LIS3DH_Y_ENABLE/DISABLE | LIS3DH_Z_ENABLE/DISABLE
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetAxis(LIS3DH_Axis_t axis) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG1, &value) )
    return MEMS_ERROR;
  value &= 0xF8;
  value |= (0x07 & axis);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG1, value) )
    return MEMS_ERROR;   
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetFullScale
* Description    : Sets the LIS3DH FullScale
* Input          : LIS3DH_FULLSCALE_2/LIS3DH_FULLSCALE_4/LIS3DH_FULLSCALE_8/LIS3DH_FULLSCALE_16
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetFullScale(LIS3DH_Fullscale_t fs) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xCF;	
  value |= (fs<<LIS3DH_FS);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetBDU
* Description    : Enable/Disable Block Data Update Functionality
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetBDU(State_t bdu) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  value |= (bdu<<LIS3DH_BDU);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetBLE
* Description    : Set Endianess (MSB/LSB)
* Input          : BLE_LSB / BLE_MSB
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetBLE(LIS3DH_Endianess_t ble) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xBF;	
  value |= (ble<<LIS3DH_BLE);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetSelfTest
* Description    : Set Self Test Modality
* Input          : LIS3DH_SELF_TEST_DISABLE/ST_0/ST_1
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetSelfTest(LIS3DH_SelfTest_t st) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xF9;
  value |= (st<<LIS3DH_ST);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFClick
* Description    : Enable/Disable High Pass Filter for click
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFClickEnable(State_t hpfe) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFB;
  value |= (hpfe<<LIS3DH_HPCLICK);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFAOI1
* Description    : Enable/Disable High Pass Filter for AOI on INT_1
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFAOI1Enable(State_t hpfe) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= (hpfe<<LIS3DH_HPIS1);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_HPFAOI2
* Description    : Enable/Disable High Pass Filter for AOI on INT_2
* Input          : MEMS_ENABLE/MEMS_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_HPFAOI2Enable(State_t hpfe) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xFD;
  value |= (hpfe<<LIS3DH_HPIS2);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetHPFMode
* Description    : Set High Pass Filter Modality
* Input          : LIS3DH_HPM_NORMAL_MODE_RES/LIS3DH_HPM_REF_SIGNAL/
				   LIS3DH_HPM_NORMAL_MODE/LIS3DH_HPM_AUTORESET_INT
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetHPFMode(LIS3DH_HPFMode_t hpm) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F;
  value |= (hpm<<LIS3DH_HPM);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetHPFCutOFF
* Description    : Set High Pass CUT OFF Freq
* Input          : HPFCF [0,3]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetHPFCutOFF(LIS3DH_HPFCutOffFreq_t hpf) {
  u8_t value = 0;
  
  if (hpf > 3)
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xCF;
  value |= (hpf<<LIS3DH_HPCF);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LIS3DH_SetFilterDataSel
* Description    : Set Filter Data Selection bypassed or sent to FIFO OUT register
* Input          : MEMS_SET, MEMS_RESET
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetFilterDataSel(State_t state) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG2, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= (state<<LIS3DH_FDS);
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG2, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
  
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Pin
* Description    : Set Interrupt1 pin Function
* Input          :  LIS3DH_CLICK_ON_PIN_INT1_ENABLE/DISABLE    | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE/DISABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_ENABLE/DISABLE  | LIS3DH_I1_DRDY1_ON_INT1_ENABLE/DISABLE    |              
                    LIS3DH_I1_DRDY2_ON_INT1_ENABLE/DISABLE     | LIS3DH_WTM_ON_INT1_ENABLE/DISABLE         |           
                    LIS3DH_INT1_OVERRUN_ENABLE/DISABLE  
* example        : SetInt1Pin(LIS3DH_CLICK_ON_PIN_INT1_ENABLE | LIS3DH_I1_INT1_ON_PIN_INT1_ENABLE |              
                    LIS3DH_I1_INT2_ON_PIN_INT1_DISABLE | LIS3DH_I1_DRDY1_ON_INT1_ENABLE | LIS3DH_I1_DRDY2_ON_INT1_ENABLE |
                    LIS3DH_WTM_ON_INT1_DISABLE | LIS3DH_INT1_OVERRUN_DISABLE   ) 
* Note           : To enable Interrupt signals on INT1 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Pin(LIS3DH_IntPinConf_t pinConf) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG3, &value) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= pinConf;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG3, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt2Pin
* Description    : Set Interrupt2 pin Function
* Input          : LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE   | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW
* example        : LIS3DH_SetInt2Pin(LIS3DH_CLICK_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_INT1_ON_PIN_INT2_ENABLE/DISABLE |               
                   LIS3DH_I2_INT2_ON_PIN_INT2_ENABLE/DISABLE | LIS3DH_I2_BOOT_ON_INT2_ENABLE/DISABLE |                   
                   LIS3DH_INT_ACTIVE_HIGH/LOW)
* Note           : To enable Interrupt signals on INT2 Pad (You MUST use all input variable in the argument, as example)
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt2Pin(LIS3DH_IntPinConf_t pinConf) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG6, &value) )
    return MEMS_ERROR;
  
  value &= 0x00;
  value |= pinConf;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG6, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}                       


/*******************************************************************************
* Function Name  : LIS3DH_SetClickCFG
* Description    : Set Click Interrupt config Function
* Input          : LIS3DH_ZD_ENABLE/DISABLE | LIS3DH_ZS_ENABLE/DISABLE  | LIS3DH_YD_ENABLE/DISABLE  | 
                   LIS3DH_YS_ENABLE/DISABLE | LIS3DH_XD_ENABLE/DISABLE  | LIS3DH_XS_ENABLE/DISABLE 
* example        : LIS3DH_SetClickCFG( LIS3DH_ZD_ENABLE | LIS3DH_ZS_DISABLE | LIS3DH_YD_ENABLE | 
                               LIS3DH_YS_DISABLE | LIS3DH_XD_ENABLE | LIS3DH_XS_ENABLE)
* Note           : You MUST use all input variable in the argument, as example
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickCFG(u8_t status) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CLICK_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0xC0;
  value |= status;
  
  if( !LIS3DH_WriteReg(LIS3DH_CLICK_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}  


/*******************************************************************************
* Function Name  : LIS3DH_SetClickTHS
* Description    : Set Click Interrupt threshold
* Input          : Click-click Threshold value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickTHS(u8_t ths) {
  
  if(ths>127)     
    return MEMS_ERROR;
  
  if( !LIS3DH_WriteReg(LIS3DH_CLICK_THS, ths) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS3DH_SetClickLIMIT
* Description    : Set Click Interrupt Time Limit
* Input          : Click-click Time Limit value [0-127]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickLIMIT(u8_t t_limit) {
  
  if(t_limit>127)     
    return MEMS_ERROR;
  
  if( !LIS3DH_WriteReg(LIS3DH_TIME_LIMIT, t_limit) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS3DH_SetClickLATENCY
* Description    : Set Click Interrupt Time Latency
* Input          : Click-click Time Latency value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickLATENCY(u8_t t_latency) {
  
  if( !LIS3DH_WriteReg(LIS3DH_TIME_LATENCY, t_latency) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 


/*******************************************************************************
* Function Name  : LIS3DH_SetClickWINDOW
* Description    : Set Click Interrupt Time Window
* Input          : Click-click Time Window value [0-255]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetClickWINDOW(u8_t t_window) {
  
  if( !LIS3DH_WriteReg(LIS3DH_TIME_WINDOW, t_window) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetClickResponse
* Description    : Get Click Interrupt Response by CLICK_SRC REGISTER
* Input          : char to empty by Click Response Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetClickResponse(u8_t* res) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CLICK_SRC, &value) ) 
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  if((value & LIS3DH_IA)==0) {        
    *res = LIS3DH_NO_CLICK;     
    return MEMS_SUCCESS;
  }
  else {
    if (value & LIS3DH_DCLICK){
      if (value & LIS3DH_CLICK_SIGN){
        if (value & LIS3DH_CLICK_Z) {
          *res = LIS3DH_DCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_Y) {
          *res = LIS3DH_DCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_X) {
          *res = LIS3DH_DCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LIS3DH_CLICK_Z) {
          *res = LIS3DH_DCLICK_Z_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_Y) {
          *res = LIS3DH_DCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_X) {
          *res = LIS3DH_DCLICK_X_P;   
          return MEMS_SUCCESS;
        }
      }       
    }
    else{
      if (value & LIS3DH_CLICK_SIGN){
        if (value & LIS3DH_CLICK_Z) {
          *res = LIS3DH_SCLICK_Z_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_Y) {
          *res = LIS3DH_SCLICK_Y_N;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_X) {
          *res = LIS3DH_SCLICK_X_N;   
          return MEMS_SUCCESS;
        }
      }
      else{
        if (value & LIS3DH_CLICK_Z) {
          *res = LIS3DH_SCLICK_Z_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_Y) {
          *res = LIS3DH_SCLICK_Y_P;   
          return MEMS_SUCCESS;
        }
        if (value & LIS3DH_CLICK_X) {
          *res = LIS3DH_SCLICK_X_P;   
          return MEMS_SUCCESS;
        }
      }
    }
  }
  return MEMS_ERROR;
} 


/*******************************************************************************
* Function Name  : LIS3DH_Int1LatchEnable
* Description    : Enable Interrupt 1 Latching function
* Input          : ENABLE/DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_Int1LatchEnable(State_t latch) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
    return MEMS_ERROR;
  
  value &= 0xF7;
  value |= latch<<LIS3DH_LIR_INT1;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_ResetInt1Latch
* Description    : Reset Interrupt 1 Latching function
* Input          : None
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_ResetInt1Latch(void) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetIntConfiguration
* Description    : Interrupt 1 Configuration (without LIS3DH_6D_INT)
* Input          : LIS3DH_INT1_AND/OR | LIS3DH_INT1_ZHIE_ENABLE/DISABLE | LIS3DH_INT1_ZLIE_ENABLE/DISABLE...
* Output         : None
* Note           : You MUST use all input variable in the argument, as example
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetIntConfiguration(LIS3DH_Int1Conf_t ic) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x40; 
  value |= ic;
  
  if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
} 

     
/*******************************************************************************
* Function Name  : LIS3DH_SetIntMode
* Description    : Interrupt 1 Configuration mode (OR, 6D Movement, AND, 6D Position)
* Input          : LIS3DH_INT_MODE_OR, LIS3DH_INT_MODE_6D_MOVEMENT, LIS3DH_INT_MODE_AND, 
				   LIS3DH_INT_MODE_6D_POSITION
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetIntMode(LIS3DH_Int1Mode_t int_mode) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
    return MEMS_ERROR;
  
  value &= 0x3F; 
  value |= (int_mode<<LIS3DH_INT_6D);
  
  if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

    
/*******************************************************************************
* Function Name  : LIS3DH_SetInt6D4DConfiguration
* Description    : 6D, 4D Interrupt Configuration
* Input          : LIS3DH_INT1_6D_ENABLE, LIS3DH_INT1_4D_ENABLE, LIS3DH_INT1_6D_4D_DISABLE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt6D4DConfiguration(LIS3DH_INT_6D_4D_t ic) {
  u8_t value = 0;
  u8_t value2 = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_CFG, &value) )
    return MEMS_ERROR;
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value2) )
    return MEMS_ERROR;
  
  if(ic == LIS3DH_INT1_6D_ENABLE){
    value &= 0xBF; 
    value |= (MEMS_ENABLE<<LIS3DH_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_DISABLE<<LIS3DH_D4D_INT1);
  }
  
  if(ic == LIS3DH_INT1_4D_ENABLE){
    value &= 0xBF; 
    value |= (MEMS_ENABLE<<LIS3DH_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_ENABLE<<LIS3DH_D4D_INT1);
  }
  
  if(ic == LIS3DH_INT1_6D_4D_DISABLE){
    value &= 0xBF; 
    value |= (MEMS_DISABLE<<LIS3DH_INT_6D);
    value2 &= 0xFB; 
    value2 |= (MEMS_DISABLE<<LIS3DH_D4D_INT1);
  }
  
  if( !LIS3DH_WriteReg(LIS3DH_INT1_CFG, value) )
    return MEMS_ERROR;
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value2) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_Get6DPosition
* Description    : 6D, 4D Interrupt Position Detect
* Input          : Byte to empty by POSITION_6D_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_Get6DPosition(u8_t* val){
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
    return MEMS_ERROR;
  
  value &= 0x7F;
  
  switch (value){
  case LIS3DH_UP_SX:   
    *val = LIS3DH_UP_SX;    
    break;
  case LIS3DH_UP_DX:   
    *val = LIS3DH_UP_DX;    
    break;
  case LIS3DH_DW_SX:   
    *val = LIS3DH_DW_SX;    
    break;
  case LIS3DH_DW_DX:   
    *val = LIS3DH_DW_DX;    
    break;
  case LIS3DH_TOP:     
    *val = LIS3DH_TOP;      
    break;
  case LIS3DH_BOTTOM:  
    *val = LIS3DH_BOTTOM;   
    break;
  }
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Threshold
* Description    : Sets Interrupt 1 Threshold
* Input          : Threshold = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Threshold(u8_t ths) {
  if (ths > 127)
    return MEMS_ERROR;
  
  if( !LIS3DH_WriteReg(LIS3DH_INT1_THS, ths) )
    return MEMS_ERROR;    
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetInt1Duration
* Description    : Sets Interrupt 1 Duration
* Input          : Duration value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetInt1Duration(LIS3DH_Int1Conf_t id) {
  
  if (id > 127)
    return MEMS_ERROR;
  
  if( !LIS3DH_WriteReg(LIS3DH_INT1_DURATION, id) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_FIFOModeEnable
* Description    : Sets Fifo Modality
* Input          : LIS3DH_FIFO_DISABLE, LIS3DH_FIFO_BYPASS_MODE, LIS3DH_FIFO_MODE, 
				   LIS3DH_FIFO_STREAM_MODE, LIS3DH_FIFO_TRIGGER_MODE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_FIFOModeEnable(LIS3DH_FifoMode_t fm) {
  u8_t value = 0;  
  
  if(fm == LIS3DH_FIFO_DISABLE) { 
    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1F;
    value |= (LIS3DH_FIFO_BYPASS_MODE<<LIS3DH_FM);                     
    
    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )           //fifo mode bypass
      return MEMS_ERROR;   
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;    
    
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo disable
      return MEMS_ERROR;   
  }
  
  if(fm == LIS3DH_FIFO_BYPASS_MODE)   {  
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DH_FIFO_EN;
    
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DH_FM);                     //fifo mode configuration
    
    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DH_FIFO_MODE)   {
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DH_FIFO_EN;
    
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;  
    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DH_FM);                      //fifo mode configuration
    
    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DH_FIFO_STREAM_MODE)   {  
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DH_FIFO_EN;
    
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;   
    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DH_FM);                      //fifo mode configuration
    
    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  if(fm == LIS3DH_FIFO_TRIGGER_MODE)   {  
    if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG5, &value) )
      return MEMS_ERROR;
    
    value &= 0xBF;
    value |= MEMS_SET<<LIS3DH_FIFO_EN;
    
    if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG5, value) )               //fifo enable
      return MEMS_ERROR;    
    if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
      return MEMS_ERROR;
    
    value &= 0x1f;
    value |= (fm<<LIS3DH_FM);                      //fifo mode configuration
    
    if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
      return MEMS_ERROR;
  }
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetTriggerInt
* Description    : Trigger event liked to trigger signal INT1/INT2
* Input          : LIS3DH_TRIG_INT1/LIS3DH_TRIG_INT2
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetTriggerInt(LIS3DH_TrigInt_t tr) {
  u8_t value = 0;  
  
  if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xDF;
  value |= (tr<<LIS3DH_TR); 
  
  if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_SetWaterMark
* Description    : Sets Watermark Value
* Input          : Watermark = [0,31]
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetWaterMark(u8_t wtm) {
  u8_t value = 0;
  
  if(wtm > 31)
    return MEMS_ERROR;  
  
  if( !LIS3DH_ReadReg(LIS3DH_FIFO_CTRL_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0xE0;
  value |= wtm; 
  
  if( !LIS3DH_WriteReg(LIS3DH_FIFO_CTRL_REG, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}

  
/*******************************************************************************
* Function Name  : LIS3DH_GetStatusReg
* Description    : Read the status register
* Input          : char to empty by Status Reg Value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusReg(u8_t* val) {
  if( !LIS3DH_ReadReg(LIS3DH_STATUS_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;  
}


/*******************************************************************************
* Function Name  : LIS3DH_GetStatusBIT
* Description    : Read the status register BIT
* Input          : LIS3DH_STATUS_REG_ZYXOR, LIS3DH_STATUS_REG_ZOR, LIS3DH_STATUS_REG_YOR, LIS3DH_STATUS_REG_XOR,
                   LIS3DH_STATUS_REG_ZYXDA, LIS3DH_STATUS_REG_ZDA, LIS3DH_STATUS_REG_YDA, LIS3DH_STATUS_REG_XDA, 
				   LIS3DH_DATAREADY_BIT
				   val: Byte to be filled with the status bit	
* Output         : status register BIT
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetStatusBit(u8_t statusBIT, u8_t* val) {
  u8_t value = 0;  
  
  if( !LIS3DH_ReadReg(LIS3DH_STATUS_REG, &value) )
    return MEMS_ERROR;
  
  switch (statusBIT){
  case LIS3DH_STATUS_REG_ZYXOR:     
    if(value &= LIS3DH_STATUS_REG_ZYXOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case LIS3DH_STATUS_REG_ZOR:       
    if(value &= LIS3DH_STATUS_REG_ZOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  case LIS3DH_STATUS_REG_YOR:       
    if(value &= LIS3DH_STATUS_REG_YOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                 
  case LIS3DH_STATUS_REG_XOR:       
    if(value &= LIS3DH_STATUS_REG_XOR){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }     
  case LIS3DH_STATUS_REG_ZYXDA:     
    if(value &= LIS3DH_STATUS_REG_ZYXDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS3DH_STATUS_REG_ZDA:       
    if(value &= LIS3DH_STATUS_REG_ZDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS3DH_STATUS_REG_YDA:       
    if(value &= LIS3DH_STATUS_REG_YDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }   
  case LIS3DH_STATUS_REG_XDA:       
    if(value &= LIS3DH_STATUS_REG_XDA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }                                  
    
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetAccAxesRaw
* Description    : Read the Acceleration Values Output Registers
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAccAxesRaw(AxesRaw_t *buff) {
  i16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_X_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_X_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_X = value;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Y = value;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_L, valueL) )
    return MEMS_ERROR;
  
  if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_H, valueH) )
    return MEMS_ERROR;
  
  buff->AXIS_Z = value;
  
  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : LIS3DH_GetAccAxes
* Description    : Read register and convert raw acceleration data to m/s2
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAccAxes(AxesAccel_t *accel) {
  uint16_t value;
  u8_t *valueL = (u8_t *)(&value);
  u8_t *valueH = ((u8_t *)(&value)+1);
  AxesRaw_t buff;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_X_L, valueL) )
    return MEMS_ERROR;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_X_H, valueH) )
    return MEMS_ERROR;

  buff.AXIS_X = value;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_L, valueL) )
    return MEMS_ERROR;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_Y_H, valueH) )
    return MEMS_ERROR;

  buff.AXIS_Y = value;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_L, valueL) )
    return MEMS_ERROR;

  if( !LIS3DH_ReadReg(LIS3DH_OUT_Z_H, valueH) )
    return MEMS_ERROR;

  buff.AXIS_Z = value;

  accel->axis_x = (((float)buff.AXIS_X)/LIS3DH_ACCEL_SCALE_MODIFIER_8G) * 1000;
  accel->axis_y = (((float)buff.AXIS_Y)/LIS3DH_ACCEL_SCALE_MODIFIER_8G) * 1000;
  accel->axis_z = (((float)buff.AXIS_Z)/LIS3DH_ACCEL_SCALE_MODIFIER_8G) * 1000;

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : LIS3DH_GetAccLP
* Description    : Read only one register per axis and convert raw acceleration data to m/s2
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAccLP(AxesAccel_t *accel) {
  int8_t valueX, valueY, valueZ;

  if (!LIS3DH_ReadReg(LIS3DH_OUT_X_H, (uint8_t *)&valueX))
    return MEMS_ERROR;

  if (!LIS3DH_ReadReg(LIS3DH_OUT_Y_H, (uint8_t *)&valueY))
    return MEMS_ERROR;

  if (!LIS3DH_ReadReg(LIS3DH_OUT_Z_H, (uint8_t *)&valueZ))
    return MEMS_ERROR;

  accel->axis_x = (((float)valueX) / LIS3DH_ACCEL_SCALE_MODIFIER_4G_LP) * 1000.0f;
  accel->axis_y = (((float)valueY) / LIS3DH_ACCEL_SCALE_MODIFIER_4G_LP) * 1000.0f;
  accel->axis_z = (((float)valueZ) / LIS3DH_ACCEL_SCALE_MODIFIER_4G_LP) * 1000.0f;

  return MEMS_SUCCESS;
}




/*******************************************************************************
* Function Name  : LIS3DH_GetAccAxes
* Description    : Read register and convert raw acceleration data to m/s2
* Input          : buffer to empity by AxesRaw_t Typedef
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetAcc(AxesAccel_t *accel) {

  AxesRaw_t buff;
  uint8_t Reg = LIS3DH_OUT_X_L;
  uint8_t value[6];

  i2c_master_write_read_device(I2C_MASTER_NUM, LIS3DH_MEMS_I2C_ADDRESS, &Reg, 1, value, sizeof(value), 1000 / portTICK_PERIOD_MS);

  buff.AXIS_X = (uint16_t)((value[1] << 8) | value[0]);
  buff.AXIS_Y = (uint16_t)((value[3] << 8) | value[2]);
  buff.AXIS_Z = (uint16_t)((value[5] << 8) | value[4]);

  accel->axis_x = (((float)buff.AXIS_X)/LIS3DH_ACCEL_SCALE_MODIFIER_4G) * 1000;
  accel->axis_y = (((float)buff.AXIS_Y)/LIS3DH_ACCEL_SCALE_MODIFIER_4G) * 1000;
  accel->axis_z = (((float)buff.AXIS_Z)/LIS3DH_ACCEL_SCALE_MODIFIER_4G) * 1000;

  return MEMS_SUCCESS; 
}

/*******************************************************************************
* Function Name  : LIS3DH_GetInt1Src
* Description    : Reset Interrupt 1 Latching function
* Input          : Char to empty by Int1 source value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetInt1Src(u8_t* val) {
  
  if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetInt1SrcBit
* Description    : Reset Interrupt 1 Latching function
* Input          : statusBIT: LIS3DH_INT_SRC_IA, LIS3DH_INT_SRC_ZH, LIS3DH_INT_SRC_ZL.....
*                  val: Byte to be filled with the status bit
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetInt1SrcBit(u8_t statusBIT, u8_t* val) {
  u8_t value = 0;  
   
  if( !LIS3DH_ReadReg(LIS3DH_INT1_SRC, &value) )
      return MEMS_ERROR;
   
  if(statusBIT == LIS3DH_INT1_SRC_IA){
    if(value &= LIS3DH_INT1_SRC_IA){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_INT1_SRC_ZH){
    if(value &= LIS3DH_INT1_SRC_ZH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_INT1_SRC_ZL){
    if(value &= LIS3DH_INT1_SRC_ZL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_INT1_SRC_YH){
    if(value &= LIS3DH_INT1_SRC_YH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_INT1_SRC_YL){
    if(value &= LIS3DH_INT1_SRC_YL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LIS3DH_INT1_SRC_XH){
    if(value &= LIS3DH_INT1_SRC_XH){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_INT1_SRC_XL){
    if(value &= LIS3DH_INT1_SRC_XL){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceReg
* Description    : Read Fifo source Register
* Input          : Byte to empty by FIFO source register value
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceReg(u8_t* val) {
  
  if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, val) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceBit
* Description    : Read Fifo WaterMark source bit
* Input          : statusBIT: LIS3DH_FIFO_SRC_WTM, LIS3DH_FIFO_SRC_OVRUN, LIS3DH_FIFO_SRC_EMPTY
*				   val: Byte to fill  with the bit value
* Output         : None
* Return         : Status of BIT [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceBit(u8_t statusBIT,  u8_t* val){
  u8_t value = 0;  
  
  if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  
  if(statusBIT == LIS3DH_FIFO_SRC_WTM){
    if(value &= LIS3DH_FIFO_SRC_WTM){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  
  if(statusBIT == LIS3DH_FIFO_SRC_OVRUN){
    if(value &= LIS3DH_FIFO_SRC_OVRUN){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  if(statusBIT == LIS3DH_FIFO_SRC_EMPTY){
    if(value &= statusBIT == LIS3DH_FIFO_SRC_EMPTY){     
      *val = MEMS_SET;
      return MEMS_SUCCESS;
    }
    else{  
      *val = MEMS_RESET;
      return MEMS_SUCCESS;
    }  
  }
  return MEMS_ERROR;
}


/*******************************************************************************
* Function Name  : LIS3DH_GetFifoSourceFSS
* Description    : Read current number of unread samples stored in FIFO
* Input          : Byte to empty by FIFO unread sample value
* Output         : None
* Return         : Status [value of FSS]
*******************************************************************************/
status_t LIS3DH_GetFifoSourceFSS(u8_t* val){
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_FIFO_SRC_REG, &value) )
    return MEMS_ERROR;
  
  value &= 0x1F;
  
  *val = value;
  
  return MEMS_SUCCESS;
}

      
/*******************************************************************************
* Function Name  : LIS3DH_SetSPIInterface
* Description    : Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* Input          : LIS3DH_SPI_3_WIRE, LIS3DH_SPI_4_WIRE
* Output         : None
* Return         : Status [MEMS_ERROR, MEMS_SUCCESS]
*******************************************************************************/
status_t LIS3DH_SetSPIInterface(LIS3DH_SPIMode_t spi) {
  u8_t value = 0;
  
  if( !LIS3DH_ReadReg(LIS3DH_CTRL_REG4, &value) )
    return MEMS_ERROR;
  
  value &= 0xFE;
  value |= spi<<LIS3DH_SIM;
  
  if( !LIS3DH_WriteReg(LIS3DH_CTRL_REG4, value) )
    return MEMS_ERROR;
  
  return MEMS_SUCCESS;
}