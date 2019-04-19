// SPDX-License-Identifier: GPL-2.0
/*
 * isl29501.c: ISL29501 Time of Flight sensor driver.
 *
 * Copyright (C) 2018
 * Author: Mathieu Othacehe <m.othacehe@gmail.com>
 *
 * 7-bit I2C slave address: 0x57
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
//#include <linux/gpio/consumer.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>


#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/triggered_buffer.h>

/* Control, setting and status registers */
#define ISL29501_DEVICE_ID			0x00
#define ISL29501_ID				0x0A

/* Sampling control registers */
#define ISL29501_INTEGRATION_PERIOD		0x10
#define ISL29501_SAMPLE_PERIOD			0x11

/* Closed loop calibration registers */
#define ISL29501_CROSSTALK_I_MSB		0x24
#define ISL29501_CROSSTALK_I_LSB		0x25
#define ISL29501_CROSSTALK_I_EXPONENT		0x26
#define ISL29501_CROSSTALK_Q_MSB		0x27
#define ISL29501_CROSSTALK_Q_LSB		0x28
#define ISL29501_CROSSTALK_Q_EXPONENT		0x29
#define ISL29501_CROSSTALK_GAIN_MSB		0x2A
#define ISL29501_CROSSTALK_GAIN_LSB		0x2B
#define ISL29501_MAGNITUDE_REF_EXP		0x2C
#define ISL29501_MAGNITUDE_REF_MSB		0x2D
#define ISL29501_MAGNITUDE_REF_LSB		0x2E
#define ISL29501_PHASE_OFFSET_MSB		0x2F
#define ISL29501_PHASE_OFFSET_LSB		0x30

/* Analog control registers */
#define ISL29501_DRIVER_RANGE			0x90
#define ISL29501_EMITTER_DAC			0x91

#define ISL29501_COMMAND_REGISTER		0xB0

/* Commands */
#define ISL29501_EMUL_SAMPLE_START_PIN		0x49
#define ISL29501_RESET_ALL_REGISTERS		0xD7
#define ISL29501_RESET_INT_SM			0xD1

/* Ambiant light and temperature corrections */
#define ISL29501_TEMP_REFERENCE			0x31
#define ISL29501_PHASE_EXPONENT			0x33
#define ISL29501_TEMP_COEFF_A			0x34
#define ISL29501_TEMP_COEFF_B			0x39
#define ISL29501_AMBIANT_COEFF_A		0x36
#define ISL29501_AMBIANT_COEFF_B		0x3B

/* Data output registers */
#define ISL29501_DISTANCE_MSB_DATA		0xD1
#define ISL29501_DISTANCE_LSB_DATA		0xD2
#define ISL29501_PRECISION_MSB			0xD3
#define ISL29501_PRECISION_LSB			0xD4
#define ISL29501_MAGNITUDE_EXPONENT		0xD5
#define ISL29501_MAGNITUDE_MSB			0xD6
#define ISL29501_MAGNITUDE_LSB			0xD7
#define ISL29501_PHASE_MSB			0xD8
#define ISL29501_PHASE_LSB			0xD9
#define ISL29501_I_RAW_EXPONENT			0xDA
#define ISL29501_I_RAW_MSB			0xDB
#define ISL29501_I_RAW_LSB			0xDC
#define ISL29501_Q_RAW_EXPONENT			0xDD
#define ISL29501_Q_RAW_MSB			0xDE
#define ISL29501_Q_RAW_LSB			0xDF
#define ISL29501_DIE_TEMPERATURE		0xE2
#define ISL29501_AMBIENT_LIGHT			0xE3
#define ISL29501_GAIN_MSB			0xE6
#define ISL29501_GAIN_LSB			0xE7

#define ISL29501_MAX_EXP_VAL 15

#define ISL29501_INT_TIME_AVAILABLE \
	"0.00007 0.00014 0.00028 0.00057 0.00114 " \
	"0.00228 0.00455 0.00910 0.01820 0.03640 " \
	"0.07281 0.14561"

#define ISL29501_CURRENT_SCALE_AVAILABLE \
	"0.0039 0.0078 0.0118 0.0157 0.0196 " \
	"0.0235 0.0275 0.0314 0.0352 0.0392 " \
	"0.0431 0.0471 0.0510 0.0549 0.0588"

enum isl29501_correction_coeff {
	COEFF_TEMP_A,
	COEFF_TEMP_B,
	COEFF_LIGHT_A,
	COEFF_LIGHT_B,
	COEFF_MAX,
};

struct isl29501_private {
	struct i2c_client *client;
	struct mutex lock;
	/* Exact representation of correction coefficients. */
	unsigned int shadow_coeffs[COEFF_MAX];
	unsigned int enable_gpio;
	unsigned int irq_gpio;
	unsigned int ss_gpio;
};
enum isl29501_register_name {
	REG_DISTANCE,
	REG_PHASE,
	REG_TEMPERATURE,
	REG_AMBIENT_LIGHT,
	REG_GAIN,
	REG_GAIN_BIAS,
	REG_PHASE_EXP,
	REG_CALIB_PHASE_TEMP_A,
	REG_CALIB_PHASE_TEMP_B,
	REG_CALIB_PHASE_LIGHT_A,
	REG_CALIB_PHASE_LIGHT_B,
	REG_DISTANCE_BIAS,
	REG_TEMPERATURE_BIAS,
	REG_INT_TIME,
	REG_SAMPLE_TIME,
	REG_DRIVER_RANGE,
	REG_EMITTER_DAC,
	CALIBRATE_MAGNITUDE,
	CALIBRATE_XTALK,
	CALIBRATE_DISTANCE,
	DISTANCE_PRECISION,
};

struct isl29501_register_desc {
	u8 msb;
	u8 lsb;
};

static const struct isl29501_register_desc isl29501_registers[] = {
	[REG_DISTANCE] = {
		.msb = ISL29501_DISTANCE_MSB_DATA,
		.lsb = ISL29501_DISTANCE_LSB_DATA,
	},
	[REG_PHASE] = {
		.msb = ISL29501_PHASE_MSB,
		.lsb = ISL29501_PHASE_LSB,
	},
	[REG_TEMPERATURE] = {
		.lsb = ISL29501_DIE_TEMPERATURE,
	},
	[REG_AMBIENT_LIGHT] = {
		.lsb = ISL29501_AMBIENT_LIGHT,
	},
	[REG_GAIN] = {
		.msb = ISL29501_GAIN_MSB,
		.lsb = ISL29501_GAIN_LSB,
	},
	[REG_GAIN_BIAS] = {
		.msb = ISL29501_CROSSTALK_GAIN_MSB,
		.lsb = ISL29501_CROSSTALK_GAIN_LSB,
	},
	[REG_PHASE_EXP] = {
		.lsb = ISL29501_PHASE_EXPONENT,
	},
	[REG_CALIB_PHASE_TEMP_A] = {
		.lsb = ISL29501_TEMP_COEFF_A,
	},
	[REG_CALIB_PHASE_TEMP_B] = {
		.lsb = ISL29501_TEMP_COEFF_B,
	},
	[REG_CALIB_PHASE_LIGHT_A] = {
		.lsb = ISL29501_AMBIANT_COEFF_A,
	},
	[REG_CALIB_PHASE_LIGHT_B] = {
		.lsb = ISL29501_AMBIANT_COEFF_B,
	},
	[REG_DISTANCE_BIAS] = {
		.msb = ISL29501_PHASE_OFFSET_MSB,
		.lsb = ISL29501_PHASE_OFFSET_LSB,
	},
	[REG_TEMPERATURE_BIAS] = {
		.lsb = ISL29501_TEMP_REFERENCE,
	},
	[REG_INT_TIME] = {
		.lsb = ISL29501_INTEGRATION_PERIOD,
	},
	[REG_SAMPLE_TIME] = {
		.lsb = ISL29501_SAMPLE_PERIOD,
	},
	[REG_DRIVER_RANGE] = {
		.lsb = ISL29501_DRIVER_RANGE,
	},
	[REG_EMITTER_DAC] = {
		.lsb = ISL29501_EMITTER_DAC,
	},
};

#if 1 /*porting*/
//      register addresses
#define REG501_ID                                       0x00
#define REG501_MASTER_CTRL                      0x01
#define REG501_STATUS                           0x02
#define REG501_SAMPLE_TIME                      0x10
#define REG501_SAMPLE_PERIOD            0x11
#define REG501_SAMPLE_RANGE                     0x12
#define REG501_SAMPLE_CONTROL           0x13
#define REG501_AGC_CONTROL1                     0x17
#define REG501_AGC_CONTROL2                     0x18
#define REG501_AGC_CONTROL3                     0x19
#define REG501_XTALK_I_EXP                      0x24
#define REG501_XTALK_I_MSB                      0x25
#define REG501_XTALK_I_LSB                      0x26
#define REG501_XTALK_Q_EXP                      0x27
#define REG501_XTALK_Q_MSB                      0x28
#define REG501_XTALK_Q_LSB                      0x29
#define REG501_XTALK_GAIN_MSB           0x2a
#define REG501_XTALK_GAIN_LSB           0x2b
#define REG501_MAG_REF_EXP                      0x2c
#define REG501_MAG_REF_MSB                      0x2d
#define REG501_MAG_REF_LSB                      0x2e
#define REG501_DISTANCE_PHASE_MSB       0x2f
#define REG501_DISTANCE_PHASE_LSB       0x30
#define REG501_INTERRUPT_STATUS         0x58
#define REG501_INTERRUPT_CONTROL        0x60
#define REG501_DETECTION_CONTROL        0x62
#define REG501_DETECTION1_CONTROL       0x63
#define REG501_DETECTION2_CONTROL       0x64
#define REG501_INTERRUPT_FLAG           0x69
#define REG501_DETECTION_FLAG           0x6a
#define REG501_DISTANCE_ZONE_BASE       0x70
#define REG501_MAG_ZONE1_BASE           0x7C
#define REG501_MAG_ZONE2_BASE           0x7e
#define REG501_MAG_ZONE3_BASE           0x82
#define REG501_DISTANCE_REF                     0x84
#define REG501_MAG_REF                          0x86
#define REG501_DRIVER_RANGE                     0x90
#define REG501_EMITTER_CURRENT          0x91
#define REG501_COMMAND_REG                      0xb0

/* data read out section */
#define REG501_READ_OUT_DATA_BASE       0xd0
#define REG501_DATA_INVALID_FLAG        0xd0
#define REG501_DISTANCE_READOUT_H       0xd1
#define REG501_DISTANCE_READOUT_L       0xd2
#define REG501_PRECISION_READOUT_H      0xd3
#define REG501_PRECISION_READOUT_L      0xd4
#define REG501_MAG_EXP_READOUT          0xf6
#define REG501_MAG_READOUT_H            0xf7
#define REG501_MAG_READOUT_L            0xf8
#define REG501_PHASE_READOUT            0xd8
#define REG501_IRAW_EXP_READOUT         0xda
#define REG501_IRAW_READOUT                     0xdb
#define REG501_QRAW_EXP_READOUT         0xdd
#define REG501_QRAW_READOUT                     0xde
#define REG501_VOLTAGE_BEFORE           0xe0
#define REG501_VOLTAGE_AFTER            0xe1
#define REG501_AFE_TEMP                         0xe2
#define REG501_AMBIENT_ADC                      0xe3
#define REG501_VGA1                                     0xe4
#define REG501_VGA2                                     0xe5
#define REG501_GAIN                                     0xe6
#define REG501_I_ADC                            0xe8
#define REG501_Q_ADC                            0xea

typedef union
{
        struct m
        {
                uint8_t cali_flag; //[2:0] = phase|xtalk|mag ; 0x7 means all cailbration done.
                uint8_t mag_ref_EXP;
                uint8_t mag_ref_MSB;
                uint8_t mag_ref_LSB;
                uint8_t xtalk_i_MSB;
                uint8_t xtalk_i_LSB;
                uint8_t xtalk_q_MSB;
                uint8_t xtalk_q_LSB;
                uint8_t xtalk_i_EXP;
                uint8_t xtalk_q_EXP;
                uint8_t xtalk_gain_MSB;
                uint8_t xtalk_gain_LSB;
                uint8_t distance_phase_MSB;
                uint8_t distance_phase_LSB;
                uint8_t afe_temp;
                uint8_t checksum;
        }__attribute__((packed)) meaning;

        uint8_t raw_data[sizeof(struct m)];

}type_cali_data;

typedef enum _REG501_IRQ_MODE
{
        MODE_NO_IRQ=0,
        MODE_IRQ_DATA_READY=1,
}REG501_IRQ_MODE;

typedef struct RO_DATA
{
        uint8_t data_invalid_flag;
        uint16_t distance;
        uint16_t precision;
        uint8_t magnitude_exp;
        uint16_t magnitude;
        uint16_t phase;
        uint8_t i_raw_exp;
        uint16_t i_raw;
        uint8_t q_raw_exp;
        uint16_t q_raw;
        uint8_t voltage_before;
        uint8_t voltage_after;
        uint8_t afe_temp;
        uint8_t ambient_adc;
        uint8_t vga1;
        uint8_t vga2;
        uint16_t        gain;
        uint16_t i_adc;
        uint16_t q_adc;
}RO_DATA;

#define ABS(x) ( (x)>0?(x):-(x) )
#define CALIFILE  "/data/test/califile"

type_cali_data _cdata={0};
struct file *cali_file = NULL;

uint8_t drv_501_i2c_regset(struct isl29501_private *isl29501, uint8_t regaddr, uint8_t bitmask, uint8_t value)
{
        //Read first.
        uint8_t read = 0;
	s32 ret = 0;
	uint8_t set = 0;

	mutex_lock(&isl29501->lock);
	ret = i2c_smbus_read_byte_data(isl29501->client, regaddr);
	if(ret < 0)
		goto err;
	read = ret;	

        set = (read & (~bitmask)) + ( bitmask & value);

	ret = i2c_smbus_write_byte_data(isl29501->client, regaddr, set);
	if(ret <0 )
		goto err;

	/*verify the writting*/
	ret = i2c_smbus_read_byte_data(isl29501->client, regaddr);
	if(ret < 0)
		goto err;
	if((ret !=  set) && (regaddr != REG501_COMMAND_REG)){ //COMMAND_REG do not support readback
		pr_err("[debug] %s verify regaddr 0x%x writting error,w:0x%x,r:0x%x !!!!",__func__,regaddr,set,ret);
	}
	

	mutex_unlock(&isl29501->lock);
        return ret;
err:
	mutex_unlock(&isl29501->lock);
	pr_err("[debug] %s write regaddr 0x%x error",__func__,regaddr);
	return ret;
}

uint8_t drv_501_i2c_read(struct isl29501_private *isl29501, uint8_t regaddr, uint8_t* value)
{
	s32 ret = 0;

	mutex_lock(&isl29501->lock);
	ret = i2c_smbus_read_byte_data(isl29501->client, regaddr);
	if (ret < 0)
		goto err;
	*value = ret;
	mutex_unlock(&isl29501->lock);
	return ret;
err:
	mutex_unlock(&isl29501->lock);
	pr_err("[debug] %s error",__func__);
	return ret;
}

uint8_t drv_501_soft_reset(struct isl29501_private *isl29501)
{
        //REG501_COMMAND_REG = 0xD7 ,Reset all Reg.
        //REG501_COMMAND_REG = 0xD1 ,Resets internal state machine.
        //REG501_COMMAND_REG = 0x49 ,start a measurment.
        //Note: COMMAND_REG do not support readback. use I2C_Set_OneReg()
	return drv_501_i2c_regset(isl29501,REG501_COMMAND_REG,0xff, 0xD7);
}

uint8_t drv_501_soft_ss_start(struct isl29501_private *isl29501)
{
        //REG501_COMMAND_REG = 0xD7 ,Reset all Reg.
        //REG501_COMMAND_REG = 0xD1 ,Resets internal state machine.
        //Note: COMMAND_REG do not support readback. use I2C_Set_OneReg()
	//A soft-start can be initiated by writing 0xB0 to 0x49. This register bit emulates the single shot pin.
	return drv_501_i2c_regset(isl29501,REG501_COMMAND_REG,0xff,0x49);
}

void drv_501_ss_trigger(struct isl29501_private *isl29501)
{
	gpio_direction_output(isl29501->ss_gpio,1);
	gpio_direction_output(isl29501->ss_gpio,0);
	msleep(10);
	gpio_direction_output(isl29501->ss_gpio,1);
	
}


uint16_t drv_501_wait_irq_pin(struct isl29501_private *isl29501, uint16_t timeout_ms)
{
        uint16_t wait_ms=0;

	while(gpio_get_value(isl29501->irq_gpio) == 1){ //ACTIVE LOW
		wait_ms +=1;
		msleep(1);
		if( wait_ms > timeout_ms ) return -1;
	}

        return wait_ms;
}


uint8_t drv_501_read_ro(struct isl29501_private *isl29501, RO_DATA* read_ro)
{
	uint8_t temp=0;

	if(drv_501_i2c_read(isl29501, REG501_DATA_INVALID_FLAG,&temp) < 0) return -1;
	read_ro->data_invalid_flag=temp;

	if(drv_501_i2c_read(isl29501, REG501_DISTANCE_READOUT_H,&temp) < 0) return -1;
	read_ro->distance=(temp<<8);

	if(drv_501_i2c_read(isl29501, REG501_DISTANCE_READOUT_L,&temp) < 0) return -1;
	read_ro->distance+=temp;

	if(drv_501_i2c_read(isl29501, REG501_PRECISION_READOUT_H,&temp) < 0) return -1;
	read_ro->precision=(temp<<8);

	if(drv_501_i2c_read(isl29501, REG501_PRECISION_READOUT_L,&temp) < 0) return -1;
	read_ro->precision+=temp;

	if(drv_501_i2c_read(isl29501, REG501_MAG_EXP_READOUT,&temp) < 0) return -1;
	read_ro->magnitude_exp=temp;

	if(drv_501_i2c_read(isl29501, REG501_MAG_READOUT_H,&temp) < 0) return -1;
	read_ro->magnitude=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_MAG_READOUT_L,&temp) < 0) return -1;
	read_ro->magnitude+=temp;

	if(drv_501_i2c_read(isl29501, REG501_PHASE_READOUT,&temp) < 0) return -1;
	read_ro->phase=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_PHASE_READOUT+1,&temp) < 0) return -1;
	read_ro->phase+=temp;

	if(drv_501_i2c_read(isl29501, REG501_IRAW_EXP_READOUT,&temp) < 0) return -1;
	read_ro->i_raw_exp=temp;

	if(drv_501_i2c_read(isl29501, REG501_IRAW_READOUT,&temp) < 0) return -1;
	read_ro->i_raw=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_IRAW_READOUT+1,&temp) < 0) return -1;
	read_ro->i_raw+=temp;

	if(drv_501_i2c_read(isl29501, REG501_QRAW_EXP_READOUT,&temp) < 0) return -1;
	read_ro->q_raw_exp=temp;
	
	if(drv_501_i2c_read(isl29501, REG501_QRAW_READOUT,&temp) < 0) return -1;
	read_ro->q_raw=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_QRAW_READOUT+1,&temp) < 0) return -1;
	read_ro->q_raw+=temp;

	if(drv_501_i2c_read(isl29501, REG501_VOLTAGE_BEFORE,&temp) < 0) return -1;
	read_ro->voltage_before=temp;

	if(drv_501_i2c_read(isl29501, REG501_VOLTAGE_AFTER,&temp) < 0) return -1;
	read_ro->voltage_after=temp;

	if(drv_501_i2c_read(isl29501, REG501_AFE_TEMP,&temp) < 0) return -1;
	read_ro->afe_temp=temp;

	if(drv_501_i2c_read(isl29501, REG501_AMBIENT_ADC,&temp) < 0) return -1;
	read_ro->ambient_adc=temp;

	if(drv_501_i2c_read(isl29501, REG501_VGA1,&temp) < 0) return -1;
	read_ro->vga1=temp;

	if(drv_501_i2c_read(isl29501, REG501_VGA2,&temp) < 0) return -1;
	read_ro->vga2=temp;

	if(drv_501_i2c_read(isl29501, REG501_GAIN,&temp) < 0) return -1;
	read_ro->gain=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_GAIN+1,&temp) < 0) return -1;
	read_ro->gain+=temp;

	if(drv_501_i2c_read(isl29501, REG501_I_ADC,&temp) < 0) return -1;
	read_ro->i_adc=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_I_ADC+1,&temp) < 0) return -1;
	read_ro->i_adc+=temp;

	if(drv_501_i2c_read(isl29501, REG501_Q_ADC,&temp) < 0) return -1;
	read_ro->q_adc=(temp<<8);
	if(drv_501_i2c_read(isl29501, REG501_Q_ADC+1,&temp) < 0) return -1;
	read_ro->q_adc+=temp;

        return 0;
}

uint8_t drv_501_set_irq_mode(struct isl29501_private *isl29501, REG501_IRQ_MODE mode)
{
	if(drv_501_i2c_regset(isl29501, REG501_INTERRUPT_CONTROL,0x7,mode) < 0) return -1;
	return 0;
}

uint8_t drv_501_irq_soft_check(struct isl29501_private *isl29501)
{
	uint8_t temp=0;
	if(drv_501_i2c_read(isl29501, REG501_INTERRUPT_FLAG,&temp) < 0) return -1;
	if( (temp & 0x1) ==1 ){
		return 0;
	}else{
		return -1;
	}
	return -1;
}

uint8_t drv_501_irq_clean(struct isl29501_private *isl29501)
{
	uint8_t temp=0;
	if(drv_501_i2c_read(isl29501, REG501_INTERRUPT_FLAG,&temp) < 0) return -1;
	return 0;
}

uint8_t drv_501_init_cailbration_magnitude(struct isl29501_private *isl29501)
{
	//sample_len=71.5us*2^SAMPLE_TIME[3:0],max=145.6ms;  sample_num=SAMPLE_TIME[7:4],0 means collect 1 sample each measuremrnt.
	//0x09 = 71.5us*2^9=71*512=36ms
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_TIME,0xff,0x09) < 0 ) return -1;
	//Time of each sample 450us*SAMPLE_PERIOD+1; 0x6E=110*450us=49ms, 0x64=100*450us=450ms
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_PERIOD,0xff,0x6e) < 0 ) return -1;
	//[0]=1 ADC single shot mode. [4]=1 Set to Cailbration mode.
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_CONTROL,0xff,0x61) < 0 ) return -1;
	//AGC setting form vendor code.
	if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL2,0xff,0x1B) < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL3,0xff,0x23) < 0 ) return -1;
	//Emitter Curren= (REG501_DRIVER_RANGE/15) * (REG501_EMITTER_CURRENT/255)*255
	//For example : 0x0d=221mA,0x06=100mA,0x0a = 167mA
	if(drv_501_i2c_regset(isl29501, REG501_DRIVER_RANGE,0xf,0x0d) < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_EMITTER_CURRENT,0xff,0xff) < 0 ) return -1;
	drv_501_set_irq_mode(isl29501,MODE_NO_IRQ);
	return 0;
}
	
	
uint8_t drv_501_init_measurement_setting(struct isl29501_private *isl29501)
{
	int ret = 0;
	//sample_len=71.5us*2^SAMPLE_TIME[3:0],max=145.6ms;  sample_num=SAMPLE_TIME[7:4],0 means collect 1 sample each measuremrnt.
	//0x09 = 71.5us*2^9=71*512=36ms 0x0b=145.6ms
	/*if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_TIME,0xff,0x09)  < 0 ) return -1;*/
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_TIME, 0xff, 0x09) < 0) return -1;

	//Time of each sample 450us*SAMPLE_PERIOD+1; 0x6E=110*450us=49ms, 0x64=100*450us=450ms
	/*if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_PERIOD,0xff,0x6e)  < 0 ) return -1;*/
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_PERIOD, 0xff, 0x6e) < 0) return -1;

	//[0]=1 ADC single shot mode. [4]=0 Set to Measurement mode.
	/*if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_CONTROL,0xff,0x7D)  < 0 ) return -1;*/
	/*jchen?? why use 0x7D. But the distance output is 0 if 0x6D*/
	if(drv_501_i2c_regset(isl29501, REG501_SAMPLE_CONTROL, 0xff, 0x7D) < 0) return -1;

	//AGC setting form vendor code.
	/*if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL2,0xff,0x1B)  < 0 ) return -1;
 	if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL3,0xff,0x23)  < 0 ) return -1;*/
	if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL2,0xff,0x1B) < 0) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_AGC_CONTROL3,0xff,0x23) < 0) return -1;

	//Emitter Curren= (REG501_DRIVER_RANGE/15) * (REG501_EMITTER_CURRENT/255)*255
	//For example : 0x0d=221mA,0x06=100mA,0x0a = 167mA
	/*if(drv_501_i2c_regset(isl29501, REG501_DRIVER_RANGE,0xf,0x0d)  < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_EMITTER_CURRENT,0xff,0xff)  < 0 ) return -1;*/
	if(drv_501_i2c_regset(isl29501, REG501_DRIVER_RANGE,0xf,0x0d) < 0) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_EMITTER_CURRENT,0xff,0xff) < 0) return -1;

	/*drv_501_Set_IRQ_Mode(MODE_NO_IRQ);*/
	if(drv_501_i2c_regset(isl29501, REG501_INTERRUPT_CONTROL,0x7, 0) < 0) return -1; /*N0 IRQ*/

	return ret;
}

uint8_t drv_501_singleshot_capture(struct isl29501_private *isl29501, RO_DATA* ro_data)
{
	if(drv_501_set_irq_mode(isl29501,MODE_IRQ_DATA_READY) < 0) return -1;
	if(drv_501_irq_clean(isl29501) < 0) return -1;
	if(drv_501_soft_ss_start(isl29501) < 0) return -1;
	drv_501_wait_irq_pin(isl29501,200);
	if(drv_501_read_ro(isl29501,ro_data) < 0) return -1;
	if(drv_501_set_irq_mode(isl29501,MODE_NO_IRQ) < 0) return -1;
	return 0;
}

uint8_t drv_501_measure_test(struct isl29501_private *isl29501, uint16_t *distance, uint16_t *precision)
{
	RO_DATA ro_data;

	//pr_err("[debug] %s \n",__func__); 
	if(drv_501_init_measurement_setting(isl29501) < 0) return -1;

	if(drv_501_singleshot_capture(isl29501, &ro_data) < 0) return -1;	
	
	*distance = ro_data.distance;
	*precision = ro_data.precision;
	//pr_err("[debug] %s test dist=%d(%d) precision=%d(%d) data_invalid:0x%x\n",__func__,ro_data.distance,*distance,ro_data.precision,*precision,ro_data.data_invalid_flag); /*ro_data.distance/19.68f ?*/
        return 0;
}

void drv_501_update_checksum(type_cali_data* cali_data)
{
	uint8_t sum=0;
	uint8_t i = 0;

	for(i=0;i<sizeof(cali_data->raw_data)-1;i++){
		sum += cali_data->raw_data[i];
	}
	cali_data->meaning.checksum = sum;

	for(i = 0; i < sizeof(cali_data->raw_data); i++){
		pr_err("raw_data[%d]=0x%x\n",i,cali_data->raw_data[i]);
	}
}

uint8_t drv_501_check_chipstatus(struct isl29501_private *isl29501, uint8_t* read)
{
	int ret = 0;
	ret = drv_501_i2c_read(isl29501, REG501_STATUS,read);
	if(ret < 0){
		pr_err("[debug] %s, read REG501_STATUS err",__func__);
		return ret;
	}
	return ret;
}

uint8_t drv_501_calibrate_data_check(type_cali_data* cali_data)
{
        //check the sum.
	uint8_t sum=0;
	uint8_t i = 0;
	for(i=0;i<sizeof(cali_data->raw_data)-1;i++){
		sum += cali_data->raw_data[i];
	}

	//Data sum check passed and all 3 calibration is vaild.
	if(cali_data->meaning.checksum == sum && cali_data->meaning.cali_flag == 0x7){
		return 0;
	}

	return -1;
}

uint8_t drv_501_calibrate_Magnitude(struct isl29501_private *isl29501, type_cali_data* cali_data)
{
	RO_DATA ro_data;

	if(drv_501_init_cailbration_magnitude(isl29501) < 0) return -1;

	if(drv_501_singleshot_capture(isl29501, &ro_data) < 0 ) return -1;

	if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_EXP,0xff,ro_data.magnitude_exp) < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_MSB,0xff,ro_data.magnitude>>8) < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_LSB,0xff,ro_data.magnitude&0xff) < 0 ) return -1;

	cali_data->meaning.mag_ref_EXP = ro_data.magnitude_exp;
	cali_data->meaning.mag_ref_MSB = ro_data.magnitude>>8;
	cali_data->meaning.mag_ref_LSB = ro_data.magnitude&0xff;
	cali_data->meaning.cali_flag |=  0x01;
	drv_501_update_checksum(cali_data);

	pr_err("Calibrate Magnitude done.\n");
	return 0;
}

uint8_t drv_501_calibrate_xtalk(struct isl29501_private *isl29501, type_cali_data* cali_data)
{
	//must be power of 2 number. becase we use bit shift to divide the sum.
	uint8_t Loop_cnt=32;
	uint32_t i = 0;

	uint8_t i_exp_base=0;
	uint8_t q_exp_base=0;
	uint32_t gain_sum=0;

	//each time i_exp and q_exp may be different. fineout an i_exp_base and q_exp_base.
	uint8_t i_exp_base_B=0;
	uint8_t q_exp_base_B=0;
	uint8_t i_B=0;
 	uint8_t q_B=0;

	uint32_t i_sum=0;
	uint32_t q_sum=0;
	uint8_t sample=0;
	uint8_t time=0;

	RO_DATA ro_data;

	pr_err("Calibrate Xtalk Start...\n");
	if(drv_501_init_measurement_setting(isl29501) < 0) return -1;
	if(drv_501_singleshot_capture(isl29501, &ro_data)< 0 ) return -1;

	i_exp_base = ro_data.i_raw_exp;
	q_exp_base = ro_data.q_raw_exp;

	// x_exp_base_B is a backup value. if it is more ofen. we let it be base value.
        for(i=0;i<Loop_cnt;i++){
		msleep(5);
		if(drv_501_singleshot_capture(isl29501, &ro_data)< 0 ) return -1;
		if(i_exp_base != ro_data.i_raw_exp) {i_exp_base_B=ro_data.i_raw_exp; i_B++; }
		if(q_exp_base != ro_data.q_raw_exp) {q_exp_base_B=ro_data.q_raw_exp; q_B++;}
        }

	// But all i_exp and q_exp will different in +-1. if not. means the value is unstable.
	if( ((i_exp_base_B !=0) && ABS(i_exp_base_B - i_exp_base) > 1)  || \
		((q_exp_base_B !=0) && ABS(q_exp_base_B - q_exp_base) > 1 ) ){
		pr_err("Xtalk data unstable.\n");
		return -1;
	}

	//more then 50% sample with bacisl29501, kup value. we let it be base value.
	if(i_B>(Loop_cnt>>1)) i_exp_base=i_exp_base_B;
	if(q_B>(Loop_cnt>>1)) q_exp_base=q_exp_base_B;

	pr_err("i_exp_base=%d.\n",ro_data.i_raw_exp);
	pr_err("q_exp_base=%d.\n",ro_data.q_raw_exp);

	while(sample<Loop_cnt){
		msleep(5);
        	memset(&ro_data, 0, sizeof(ro_data));
		if(drv_501_singleshot_capture(isl29501,&ro_data)< 0 ) return -1;

		//only collect same exp sample.
		if(ro_data.i_raw_exp==i_exp_base && ro_data.q_raw_exp==q_exp_base){
			sample++;
			i_sum+=ro_data.i_raw;
			q_sum+=ro_data.q_raw;
			gain_sum += ro_data.gain;
		}
		time++;
		//timeout. still no enough sample.
		if(time > (Loop_cnt<<2)) {pr_err("time out.\n"); return -1;}
		//debug
		if(time==1){
			pr_err("i_raw=%d.\n",ro_data.i_raw);
			pr_err("q_raw=%d.\n",ro_data.q_raw);
			pr_err("gain=%d.\n",ro_data.gain);
		}
	}

	pr_err("try=%d.\n",time);

	i_sum>>=5;
	q_sum>>=5;
	gain_sum>>=5;

	if(drv_501_i2c_regset(isl29501, REG501_XTALK_I_MSB,0xff,(uint16_t)i_sum>>8)< 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_XTALK_I_LSB,0xff,(uint16_t)i_sum&0xff)< 0 ) return -1;

	if(drv_501_i2c_regset(isl29501, REG501_XTALK_Q_MSB,0xff,(uint16_t)q_sum>>8)< 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_XTALK_Q_LSB,0xff,(uint16_t)q_sum&0xff)< 0 ) return -1;

	if(drv_501_i2c_regset(isl29501, REG501_XTALK_I_EXP,0xff,(uint8_t)i_exp_base)< 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_XTALK_Q_EXP,0xff,(uint8_t)q_exp_base)< 0 ) return -1;

	if(drv_501_i2c_regset(isl29501, REG501_XTALK_GAIN_MSB,0xff,(uint16_t)gain_sum>>8)< 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_XTALK_GAIN_LSB,0xff,(uint16_t)gain_sum&0xff)< 0 ) return -1;


	cali_data->meaning.xtalk_i_MSB = (uint16_t)i_sum>>8;
	cali_data->meaning.xtalk_i_LSB = (uint16_t)i_sum&0xff;
	cali_data->meaning.xtalk_q_MSB = (uint16_t)q_sum>>8;
	cali_data->meaning.xtalk_q_LSB = (uint16_t)q_sum&0xff;
	cali_data->meaning.xtalk_i_EXP = (uint8_t)i_exp_base;
	cali_data->meaning.xtalk_q_EXP = (uint8_t)q_exp_base;
	cali_data->meaning.xtalk_gain_MSB = (uint16_t)gain_sum>>8;
	cali_data->meaning.xtalk_gain_LSB = (uint16_t)gain_sum&0xff;

	cali_data->meaning.cali_flag |=  0x02;
	drv_501_update_checksum(cali_data);

	pr_err("avg i_exp=%d.\n",i_exp_base);
	pr_err("avg q_exp=%d.\n",q_exp_base);
	pr_err("avg i_raw=%d.\n",(int)i_sum);
	pr_err("avg q_raw=%d.\n",(int)q_sum);
	pr_err("avg gain=%d.\n",(int)gain_sum);
	pr_err("Calibrate_Xtalk Done.\n");

	return 0;
}

//TODO: This step should adding a temperature compensation.
uint8_t drv_501_calibrate_distance(struct isl29501_private *isl29501, type_cali_data* cali_data)
{
	//must be power of 2 number. becase we use bit shift to divide the sum.
	uint8_t Loop_cnt=64;
	uint32_t i = 0;
	uint16_t ref_dist_cm=20;
	uint32_t phase_sum=0;

	pr_err("Calibrate Distance Start...\n");

	if(drv_501_init_measurement_setting(isl29501)  < 0) return -1;

	for(i=0;i<Loop_cnt;i++){
		RO_DATA ro_data;
		if(drv_501_singleshot_capture(isl29501, &ro_data) < 0 ) return -1;

		phase_sum += ro_data.phase;

		//debug
                if(i==0){
			pr_err("phase=%d.\n",ro_data.phase);
		}
	}

	phase_sum>>=6;
	pr_err("avg phase=%d.\n",(int)phase_sum);

	phase_sum = phase_sum-((((uint32_t)ref_dist_cm)*65536/3330));

	pr_err("Correction phase=%d.\n",(int)phase_sum);

	if(drv_501_i2c_regset(isl29501, REG501_DISTANCE_PHASE_MSB,0xff,(uint16_t)phase_sum>>8) < 0 ) return -1;
	if(drv_501_i2c_regset(isl29501, REG501_DISTANCE_PHASE_LSB,0xff,(uint16_t)phase_sum&0xff) < 0 ) return -1;

	cali_data->meaning.distance_phase_MSB = (uint16_t)phase_sum>>8;
	cali_data->meaning.distance_phase_LSB = (uint16_t)phase_sum&0xff;

	cali_data->meaning.cali_flag |=  0x04;

	drv_501_update_checksum(cali_data);

	pr_err("Calibrate Distance Done.\n");

	return 0;
}


uint8_t drv_501_setup_cali_data(struct isl29501_private *isl29501, type_cali_data* cali_data)
{
	if(drv_501_calibrate_data_check(cali_data) < 0) return -1;

	if(cali_data->meaning.cali_flag & 0x01){
		if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_EXP,0xff,cali_data->meaning.mag_ref_EXP) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_MSB,0xff,cali_data->meaning.mag_ref_MSB) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_MAG_REF_LSB,0xff,cali_data->meaning.mag_ref_LSB) < 0 ) return -1;
		pr_err("[debug] %s set mag done.\n",__func__);
	}

	if(cali_data->meaning.cali_flag & 0x02){
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_I_MSB,0xff,cali_data->meaning.xtalk_i_MSB) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_I_LSB,0xff,cali_data->meaning.xtalk_i_LSB) < 0 ) return -1;
	
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_Q_MSB,0xff,cali_data->meaning.xtalk_q_MSB) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_Q_LSB,0xff,cali_data->meaning.xtalk_q_LSB) < 0 ) return -1;

		if(drv_501_i2c_regset(isl29501,REG501_XTALK_I_EXP,0xff,cali_data->meaning.xtalk_i_EXP) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_Q_EXP,0xff,cali_data->meaning.xtalk_q_EXP) < 0 ) return -1;

		if(drv_501_i2c_regset(isl29501,REG501_XTALK_GAIN_MSB,0xff,cali_data->meaning.xtalk_gain_MSB) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_XTALK_GAIN_LSB,0xff,cali_data->meaning.xtalk_gain_LSB) < 0 ) return -1;
		pr_err("[debug] %s set xtalk done.\n",__func__);
	}
	
	if(cali_data->meaning.cali_flag & 0x04){
		if(drv_501_i2c_regset(isl29501,REG501_DISTANCE_PHASE_MSB,0xff,cali_data->meaning.distance_phase_MSB) < 0 ) return -1;
		if(drv_501_i2c_regset(isl29501,REG501_DISTANCE_PHASE_LSB,0xff,cali_data->meaning.distance_phase_LSB) < 0 ) return -1;
		pr_err("[debug] %s set phase done.\n",__func__);
	}

	return 0;
}


#endif


static int isl29501_register_read(struct isl29501_private *isl29501,
				  enum isl29501_register_name name,
				  u32 *val)
{
	const struct isl29501_register_desc *reg = &isl29501_registers[name];
	u8 msb = 0, lsb = 0;
	s32 ret;

	mutex_lock(&isl29501->lock);
	if (reg->msb) {
		ret = i2c_smbus_read_byte_data(isl29501->client, reg->msb);
		if (ret < 0)
			goto err;
		msb = ret;
	}

	if (reg->lsb) {
		ret = i2c_smbus_read_byte_data(isl29501->client, reg->lsb);
		if (ret < 0)
			goto err;
		lsb = ret;
	}
	mutex_unlock(&isl29501->lock);

	*val = (msb << 8) + lsb;

	return 0;
err:
	mutex_unlock(&isl29501->lock);

	return ret;
}

static u32 isl29501_register_write(struct isl29501_private *isl29501,
				   enum isl29501_register_name name,
				   u32 value)
{
	const struct isl29501_register_desc *reg = &isl29501_registers[name];
	u8 msb, lsb;
	int ret;

	if (!reg->msb && value > U8_MAX)
		return -ERANGE;

	if (value > U16_MAX)
		return -ERANGE;

	if (!reg->msb) {
		lsb = value & 0xFF;
	} else {
		msb = (value >> 8) & 0xFF;
		lsb = value & 0xFF;
	}

	mutex_lock(&isl29501->lock);
	if (reg->msb) {
		ret = i2c_smbus_write_byte_data(isl29501->client,
						reg->msb, msb);
		if (ret < 0)
			goto err;
	}

	ret = i2c_smbus_write_byte_data(isl29501->client, reg->lsb, lsb);

err:
	mutex_unlock(&isl29501->lock);
	return ret;
}

static ssize_t isl29501_read_ext(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan,
				 char *buf)
{
	struct isl29501_private *isl29501 = iio_priv(indio_dev);
	enum isl29501_register_name reg = private;
	int ret;
	u32 value, gain, coeff, exp;
	uint16_t distance = 0, precision = 0;

	switch (reg) {
	case REG_GAIN:
	case REG_GAIN_BIAS:
		ret = isl29501_register_read(isl29501, reg, &gain);
		if (ret < 0)
			return ret;

		value = gain;
		break;
	case REG_CALIB_PHASE_TEMP_A:
	case REG_CALIB_PHASE_TEMP_B:
	case REG_CALIB_PHASE_LIGHT_A:
	case REG_CALIB_PHASE_LIGHT_B:
		ret = isl29501_register_read(isl29501, REG_PHASE_EXP, &exp);
		if (ret < 0)
			return ret;

		ret = isl29501_register_read(isl29501, reg, &coeff);
		if (ret < 0)
			return ret;

		value = coeff << exp;
		break;
#if 1
	case CALIBRATE_MAGNITUDE:
		pr_err("[debug] %s CALIBRATE_MAGNITUDE\n",__func__);
		ret = drv_501_calibrate_Magnitude(isl29501, &_cdata);
		if (ret < 0)
			return ret;
		value = 0;
		break;
	case CALIBRATE_XTALK:
		pr_err("[debug] %s CALIBRATE_XTALK\n",__func__);
		ret = drv_501_calibrate_xtalk(isl29501, &_cdata);
		if (ret < 0)
			return ret;
		value = 0;
		break;
	case CALIBRATE_DISTANCE:
		pr_err("[debug] %s CALIBRATE_DISTANCE\n",__func__);
		ret = drv_501_calibrate_distance(isl29501, &_cdata);
		if (ret < 0)
			return ret;
		value = 0;
		break;
	case DISTANCE_PRECISION:
		//pr_err("[debug] %s DISTANCE_PRECISION",__func__);
		ret = drv_501_measure_test(isl29501, &distance, &precision);
		if(ret < 0)
			return ret;
		value = (((uint32_t)distance) << 16) + (uint32_t)precision;	
		//pr_err("[debug]1 %s value:%d distance: %d   precision: %d \n",__func__,value, (value & 0xffff0000) >> 16, value & 0xffff);
		break;
#endif
	default:
		return -EINVAL;
	}

	return sprintf(buf, "%u\n", value);
}

static int isl29501_set_shadow_coeff(struct isl29501_private *isl29501,
				     enum isl29501_register_name reg,
				     unsigned int val)
{
	enum isl29501_correction_coeff coeff;

	switch (reg) {
	case REG_CALIB_PHASE_TEMP_A:
		coeff = COEFF_TEMP_A;
		break;
	case REG_CALIB_PHASE_TEMP_B:
		coeff = COEFF_TEMP_B;
		break;
	case REG_CALIB_PHASE_LIGHT_A:
		coeff = COEFF_LIGHT_A;
		break;
	case REG_CALIB_PHASE_LIGHT_B:
		coeff = COEFF_LIGHT_B;
		break;
	default:
		return -EINVAL;
	}
	isl29501->shadow_coeffs[coeff] = val;

	return 0;
}

static int isl29501_write_coeff(struct isl29501_private *isl29501,
				enum isl29501_correction_coeff coeff,
				int val)
{
	enum isl29501_register_name reg;

	switch (coeff) {
	case COEFF_TEMP_A:
		reg = REG_CALIB_PHASE_TEMP_A;
		break;
	case COEFF_TEMP_B:
		reg = REG_CALIB_PHASE_TEMP_B;
		break;
	case COEFF_LIGHT_A:
		reg = REG_CALIB_PHASE_LIGHT_A;
		break;
	case COEFF_LIGHT_B:
		reg = REG_CALIB_PHASE_LIGHT_B;
		break;
	default:
		return -EINVAL;
	}

	return isl29501_register_write(isl29501, reg, val);
}

static unsigned int isl29501_find_corr_exp(unsigned int val,
					   unsigned int max_exp,
					   unsigned int max_mantissa)
{
	unsigned int exp = 1;

	/*
	 * Correction coefficients are represented under
	 * mantissa * 2^exponent form, where mantissa and exponent
	 * are stored in two separate registers of the sensor.
	 *
	 * Compute and return the lowest exponent such as:
	 *	     mantissa = value / 2^exponent
	 *
	 *  where mantissa < max_mantissa.
	 */
	if (val <= max_mantissa)
		return 0;

	while ((val >> exp) > max_mantissa) {
		exp++;

		if (exp > max_exp)
			return max_exp;
	}

	return exp;
}

static ssize_t isl29501_write_ext(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	struct isl29501_private *isl29501 = iio_priv(indio_dev);
	enum isl29501_register_name reg = private;
	unsigned int val;
	int max_exp = 0;
	int ret;
	int i;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	switch (reg) {
	case REG_GAIN_BIAS:
		if (val > U16_MAX)
			return -ERANGE;

		ret = isl29501_register_write(isl29501, reg, val);
		if (ret < 0)
			return ret;

		break;
	case REG_CALIB_PHASE_TEMP_A:
	case REG_CALIB_PHASE_TEMP_B:
	case REG_CALIB_PHASE_LIGHT_A:
	case REG_CALIB_PHASE_LIGHT_B:

		if (val > (U8_MAX << ISL29501_MAX_EXP_VAL))
			return -ERANGE;

		/* Store the correction coefficient under its exact form. */
		ret = isl29501_set_shadow_coeff(isl29501, reg, val);
		if (ret < 0)
			return ret;

		/*
		 * Find the highest exponent needed to represent
		 * correction coefficients.
		 */
		for (i = 0; i < COEFF_MAX; i++) {
			int corr;
			int corr_exp;

			corr = isl29501->shadow_coeffs[i];
			corr_exp = isl29501_find_corr_exp(corr,
							  ISL29501_MAX_EXP_VAL,
							  U8_MAX / 2);
			dev_dbg(&isl29501->client->dev,
				"found exp of corr(%d) = %d\n", corr, corr_exp);

			max_exp = max(max_exp, corr_exp);
		}

		/*
		 * Represent every correction coefficient under
		 * mantissa * 2^max_exponent form and force the
		 * writing of those coefficients on the sensor.
		 */
		for (i = 0; i < COEFF_MAX; i++) {
			int corr;
			int mantissa;

			corr = isl29501->shadow_coeffs[i];
			if (!corr)
				continue;

			mantissa = corr >> max_exp;

			ret = isl29501_write_coeff(isl29501, i, mantissa);
			if (ret < 0)
				return ret;
		}

		ret = isl29501_register_write(isl29501, REG_PHASE_EXP, max_exp);
		if (ret < 0)
			return ret;

		break;
	default:
		return -EINVAL;
	}

	return len;
}

#define _ISL29501_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = isl29501_read_ext, \
	.write = isl29501_write_ext, \
	.private = _ident, \
	.shared = 0, \
}
       //.shared = IIO_SEPARATE, \   \\ Jack_modifies this field because the kernel doesn't support it 

static const struct iio_chan_spec_ext_info isl29501_ext_info[] = {
	_ISL29501_EXT_INFO("agc_gain", REG_GAIN),
	_ISL29501_EXT_INFO("agc_gain_bias", REG_GAIN_BIAS),
	_ISL29501_EXT_INFO("calib_phase_temp_a", REG_CALIB_PHASE_TEMP_A),
	_ISL29501_EXT_INFO("calib_phase_temp_b", REG_CALIB_PHASE_TEMP_B),
	_ISL29501_EXT_INFO("calib_phase_light_a", REG_CALIB_PHASE_LIGHT_A),
	_ISL29501_EXT_INFO("calib_phase_light_b", REG_CALIB_PHASE_LIGHT_B),
	_ISL29501_EXT_INFO("calib_magnitude", CALIBRATE_MAGNITUDE),
	_ISL29501_EXT_INFO("calib_xtalk", CALIBRATE_XTALK),
	_ISL29501_EXT_INFO("calib_distance", CALIBRATE_DISTANCE),
	_ISL29501_EXT_INFO("distance_precision", DISTANCE_PRECISION),
	{ },
};

#define ISL29501_DISTANCE_SCAN_INDEX 0
#define ISL29501_TIMESTAMP_SCAN_INDEX 1

static const struct iio_chan_spec isl29501_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.scan_index = ISL29501_DISTANCE_SCAN_INDEX,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW)   |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_CALIBBIAS),
		.scan_type = {
			.sign = 'u',
			.realbits = 16,
			.storagebits = 16,
			.endianness = IIO_CPU,
		},
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_INT_TIME) |
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = isl29501_ext_info,
	},
	{
		.type = IIO_PHASE,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_CURRENT,
		.scan_index = -1,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_TEMP,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE)     |
				BIT(IIO_CHAN_INFO_CALIBBIAS),
	},
	{
		.type = IIO_INTENSITY,
		.scan_index = -1,
		.modified = 1,
		.channel2 = IIO_MOD_LIGHT_CLEAR,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE),
	},
	IIO_CHAN_SOFT_TIMESTAMP(ISL29501_TIMESTAMP_SCAN_INDEX),
};

static int isl29501_reset_registers(struct isl29501_private *isl29501)
{
	int ret;

	ret = i2c_smbus_write_byte_data(isl29501->client,
					ISL29501_COMMAND_REGISTER,
					ISL29501_RESET_ALL_REGISTERS);
	if (ret < 0) {
		dev_err(&isl29501->client->dev,
			"cannot reset registers %d\n", ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(isl29501->client,
					ISL29501_COMMAND_REGISTER,
					ISL29501_RESET_INT_SM);
	if (ret < 0)
		dev_err(&isl29501->client->dev,
			"cannot reset state machine %d\n", ret);

	return ret;
}

static int isl29501_begin_acquisition(struct isl29501_private *isl29501)
{
	int ret;

	ret = i2c_smbus_write_byte_data(isl29501->client,
					ISL29501_COMMAND_REGISTER,
					ISL29501_EMUL_SAMPLE_START_PIN);
	if (ret < 0)
		dev_err(&isl29501->client->dev,
			"cannot begin acquisition %d\n", ret);

	return ret;
}

static IIO_CONST_ATTR_INT_TIME_AVAIL(ISL29501_INT_TIME_AVAILABLE);
static IIO_CONST_ATTR(out_current_scale_available,
		      ISL29501_CURRENT_SCALE_AVAILABLE);

static struct attribute *isl29501_attributes[] = {
	&iio_const_attr_integration_time_available.dev_attr.attr,
	&iio_const_attr_out_current_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group isl29501_attribute_group = {
	.attrs = isl29501_attributes,
};

static const int isl29501_current_scale_table[][2] = {
	{0, 3900}, {0, 7800}, {0, 11800}, {0, 15700},
	{0, 19600}, {0, 23500}, {0, 27500}, {0, 31400},
	{0, 35200}, {0, 39200}, {0, 43100}, {0, 47100},
	{0, 51000}, {0, 54900}, {0, 58800},
};

static const int isl29501_int_time[][2] = {
	{0, 70},    /* 0.07 ms */
	{0, 140},   /* 0.14 ms */
	{0, 280},   /* 0.28 ms */
	{0, 570},   /* 0.57 ms */
	{0, 1140},  /* 1.14 ms */
	{0, 2280},  /* 2.28 ms */
	{0, 4550},  /* 4.55 ms */
	{0, 9100},  /* 9.11 ms */
	{0, 18200}, /* 18.2 ms */
	{0, 36400}, /* 36.4 ms */
	{0, 72810}, /* 72.81 ms */
	{0, 145610} /* 145.28 ms */
};

static int isl29501_get_raw(struct isl29501_private *isl29501,
			    const struct iio_chan_spec *chan,
			    int *raw)
{
	int ret;

	switch (chan->type) {
	case IIO_PROXIMITY:
		ret = isl29501_register_read(isl29501, REG_DISTANCE, raw);
		pr_err("[debug] %s, IIO_PROXIMITY, raw:%d,ret:%d\n",__func__,*raw,ret);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_INTENSITY:
		ret = isl29501_register_read(isl29501,
					     REG_AMBIENT_LIGHT,
					     raw);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_PHASE:
		ret = isl29501_register_read(isl29501, REG_PHASE, raw);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CURRENT:
		ret = isl29501_register_read(isl29501, REG_EMITTER_DAC, raw);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_TEMP:
		ret = isl29501_register_read(isl29501, REG_TEMPERATURE, raw);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int isl29501_get_scale(struct isl29501_private *isl29501,
			      const struct iio_chan_spec *chan,
			      int *val, int *val2)
{
	int ret;
	u32 current_scale;

	switch (chan->type) {
	case IIO_PROXIMITY:
		/* distance = raw_distance * 33.31 / 65536 (m) */
		*val = 3331;
		*val2 = 6553600;

		return IIO_VAL_FRACTIONAL;
	case IIO_PHASE:
		/* phase = raw_phase * 2pi / 65536 (rad) */
		*val = 0;
		*val2 = 95874;

		return IIO_VAL_INT_PLUS_NANO;
	case IIO_INTENSITY:
		/* light = raw_light * 35 / 10000 (mA) */
		*val = 35;
		*val2 = 10000;

		return IIO_VAL_FRACTIONAL;
	case IIO_CURRENT:
		ret = isl29501_register_read(isl29501,
					     REG_DRIVER_RANGE,
					     &current_scale);
		if (ret < 0)
			return ret;

		if (current_scale > ARRAY_SIZE(isl29501_current_scale_table))
			return -EINVAL;

		if (!current_scale) {
			*val = 0;
			*val2 = 0;
			return IIO_VAL_INT;
		}

		*val = isl29501_current_scale_table[current_scale - 1][0];
		*val2 = isl29501_current_scale_table[current_scale - 1][1];

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_TEMP:
		/* temperature = raw_temperature * 125 / 100000 (milli °C) */
		*val = 125;
		*val2 = 100000;

		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int isl29501_get_calibbias(struct isl29501_private *isl29501,
				  const struct iio_chan_spec *chan,
				  int *bias)
{
	switch (chan->type) {
	case IIO_PROXIMITY:
		return isl29501_register_read(isl29501,
					      REG_DISTANCE_BIAS,
					      bias);
	case IIO_TEMP:
		return isl29501_register_read(isl29501,
					      REG_TEMPERATURE_BIAS,
					      bias);
	default:
		return -EINVAL;
	}
}

static int isl29501_get_inttime(struct isl29501_private *isl29501,
				int *val, int *val2)
{
	int ret;
	u32 inttime;

	ret = isl29501_register_read(isl29501, REG_INT_TIME, &inttime);
	if (ret < 0)
		return ret;

	if (inttime >= ARRAY_SIZE(isl29501_int_time))
		return -EINVAL;

	*val = isl29501_int_time[inttime][0];
	*val2 = isl29501_int_time[inttime][1];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int isl29501_get_freq(struct isl29501_private *isl29501,
			     int *val, int *val2)
{
	int ret;
	int sample_time;
	unsigned long long freq;
	u32 temp;

	ret = isl29501_register_read(isl29501, REG_SAMPLE_TIME, &sample_time);
	if (ret < 0)
		return ret;

	/* freq = 1 / (0.000450 * (sample_time + 1) * 10^-6) */
	freq = 1000000ULL * 1000000ULL;

	do_div(freq, 450 * (sample_time + 1));

	temp = do_div(freq, 1000000);
	*val = freq;
	*val2 = temp;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int isl29501_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int *val,
			     int *val2, long mask)
{
	struct isl29501_private *isl29501 = iio_priv(indio_dev);
	
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return isl29501_get_raw(isl29501, chan, val);
	case IIO_CHAN_INFO_SCALE:
		return isl29501_get_scale(isl29501, chan, val, val2);
	case IIO_CHAN_INFO_INT_TIME:
		return isl29501_get_inttime(isl29501, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return isl29501_get_freq(isl29501, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return isl29501_get_calibbias(isl29501, chan, val);
	default:
		return -EINVAL;
	}
}

static int isl29501_set_raw(struct isl29501_private *isl29501,
			    const struct iio_chan_spec *chan,
			    int raw)
{
	switch (chan->type) {
	case IIO_CURRENT:
		return isl29501_register_write(isl29501, REG_EMITTER_DAC, raw);
	default:
		return -EINVAL;
	}
}

static int isl29501_set_inttime(struct isl29501_private *isl29501,
				int val, int val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(isl29501_int_time); i++) {
		if (isl29501_int_time[i][0] == val &&
		    isl29501_int_time[i][1] == val2) {
			return isl29501_register_write(isl29501,
						       REG_INT_TIME,
						       i);
		}
	}

	return -EINVAL;
}

static int isl29501_set_scale(struct isl29501_private *isl29501,
			      const struct iio_chan_spec *chan,
			      int val, int val2)
{
	int i;

	if (chan->type != IIO_CURRENT)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(isl29501_current_scale_table); i++) {
		if (isl29501_current_scale_table[i][0] == val &&
		    isl29501_current_scale_table[i][1] == val2) {
			return isl29501_register_write(isl29501,
						       REG_DRIVER_RANGE,
						       i + 1);
		}
	}

	return -EINVAL;
}

static int isl29501_set_calibbias(struct isl29501_private *isl29501,
				  const struct iio_chan_spec *chan,
				  int bias)
{
	switch (chan->type) {
	case IIO_PROXIMITY:
		return isl29501_register_write(isl29501,
					      REG_DISTANCE_BIAS,
					      bias);
	case IIO_TEMP:
		return isl29501_register_write(isl29501,
					       REG_TEMPERATURE_BIAS,
					       bias);
	default:
		return -EINVAL;
	}
}

static int isl29501_set_freq(struct isl29501_private *isl29501,
			     int val, int val2)
{
	int freq;
	unsigned long long sample_time;

	/* sample_freq = 1 / (0.000450 * (sample_time + 1) * 10^-6) */
	freq = val * 1000000 + val2 % 1000000;
	sample_time = 2222ULL * 1000000ULL;
	do_div(sample_time, freq);

	sample_time -= 1;
	
	pr_err("[debug] %s,sample_period:%d",__func__,sample_time);
	if (sample_time > 255)
		return -ERANGE;

	return isl29501_register_write(isl29501, REG_SAMPLE_TIME, sample_time);
}

static int isl29501_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct isl29501_private *isl29501 = iio_priv(indio_dev);

	pr_err("[debug] %s, chan:0x%x,val:%d,val2:%d,mask:%d",__func__,chan,val,val2,mask);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		pr_err("[debug] %s,INFO_RAW: chan:0x%x,val:%d",__func__,chan,val);
		return isl29501_set_raw(isl29501, chan, val);
	case IIO_CHAN_INFO_INT_TIME:
		pr_err("[debug] %s,INT_TIME: val:%d,val2:%d",__func__,val,val2);
		return isl29501_set_inttime(isl29501, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		pr_err("[debug] %s,SAMP_FREQ: val:%d,val2:%d",__func__,val,val2);
		return isl29501_set_freq(isl29501, val, val2);
	case IIO_CHAN_INFO_SCALE:
		pr_err("[debug] %s,INFO_SCALE: chan:0x%x,val:%d,val2:%d",__func__,chan, val, val2);
		return isl29501_set_scale(isl29501, chan, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		pr_err("[debug] %s,INFO_CALIBBIAS: chan:0x%x,val:%d",__func__,chan,val);
		return isl29501_set_calibbias(isl29501, chan, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info isl29501_info = {
	.read_raw = &isl29501_read_raw,
	.write_raw = &isl29501_write_raw,
	.attrs = &isl29501_attribute_group,
};

static int isl29501_init_chip(struct isl29501_private *isl29501, struct device_node *np)
{
	int ret = -1;
	uint8_t sta=0;
	enum of_gpio_flags flags;
	
	//Chip enable GPIO, active low
	isl29501->enable_gpio = of_get_named_gpio_flags(np, "enable-gpios", 0, &flags);
	if (gpio_is_valid(isl29501->enable_gpio)) {
		ret = gpio_request(isl29501->enable_gpio, "enable-gpios");
		if (ret) {
			dev_err(&isl29501->client->dev, "isl29501 enable-gpios request failure!\n");
			return -1;
		}
		//Chip enable pin. Output Low to enable chip I2C intreface.
		gpio_direction_output(isl29501->enable_gpio, 0);
		dev_err(&isl29501->client->dev, "isl29501 enable-gpios request success. output low to enable chip\n");

	} else {
		dev_err(&isl29501->client->dev,"isl29501 enable-gpios invalid!\n");
	}

	//IRQ GPIO, active low
	isl29501->irq_gpio = of_get_named_gpio_flags(np, "irq-gpios", 0, &flags);
	if (gpio_is_valid(isl29501->irq_gpio)) {
		ret = gpio_request(isl29501->irq_gpio, "irq-gpios");
		if (ret) {
			dev_err(&isl29501->client->dev, "isl29501 irq-gpios request failure!\n");
			return -1;
		}
		gpio_direction_input(isl29501->irq_gpio);
		dev_err(&isl29501->client->dev, "isl29501 irq-gpios request success. set input direction\n");

	} else {
		dev_err(&isl29501->client->dev,"isl29501 irq-gpios invalid!\n");
	}

	//SS(Sample start) GPIO, active HIGH->LOW edge
	isl29501->ss_gpio = of_get_named_gpio_flags(np, "ss-gpios", 0, &flags);
	if (gpio_is_valid(isl29501->ss_gpio)) {
		ret = gpio_request(isl29501->ss_gpio, "ss-gpios");
		if (ret) {
			dev_err(&isl29501->client->dev, "isl29501 ss-gpios request failure!\n");
			return -1;
		}
		//Single shot start pin. set to Low to start a measurment.
		gpio_direction_output(isl29501->ss_gpio, 1);
		dev_err(&isl29501->client->dev, "isl29501 ss-gpios request success. set output high\n");

	} else {
		dev_err(&isl29501->client->dev,"isl29501 ss-gpios invalid!\n");
	}

	ret = i2c_smbus_read_byte_data(isl29501->client, ISL29501_DEVICE_ID);
	if (ret < 0) {
		dev_err(&isl29501->client->dev, "Error reading device id\n");
		return ret;
	}


	if (ret != ISL29501_ID) {
		dev_err(&isl29501->client->dev,
			"Wrong chip id, got %x expected %x\n",
			ret, ISL29501_DEVICE_ID);
		return -ENODEV;
	}

	ret = isl29501_reset_registers(isl29501);
	if (ret < 0) {
		return ret;
	}
#if 1
	if(drv_501_soft_reset(isl29501) < 0){
		pr_err("isl29501 Reset fail.!!");
	}
	if (drv_501_init_measurement_setting(isl29501) < 0)
        {
                pr_err("isl29501 measure setting done.\n");
        }
        if (drv_501_check_chipstatus(isl29501, &sta) < 0)
        {
                pr_err("isl29501 status=%d.\n",sta);
        }
#if 0
	if(cali_file == NULL) {
                cali_file = filp_open(CALIFILE, O_RDWR | O_CREAT, 0644);
		if (IS_ERR(cali_file)) {
                	pr_err("[debug] %s error occured while opening file %s,!!!\n", __func__,CALIFILE);
			cali_file == NULL;
                	//return -1;
		}
	}
#endif
        memset(&_cdata.raw_data,0,sizeof(_cdata.raw_data));
#if 0
	if(cali_file != NULL){
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		if(vfs_read(cali_file, _cdata.raw_data, sizeof(_cdata.raw_data), &pos) != sizeof(_cdata.raw_data)){
			pr_err("[debug] %s read cali_file %s error, use default calibration data",__func__,CALIFILE);
        		//Flash_pull(FLASH_ZONE_1,sizeof(_cdata.raw_data),_cdata.raw_data);
			_cdata.raw_data[0] = 0x7;  //cali_flag
			_cdata.raw_data[1] = 0x7;  //mag_ref_EXP
			_cdata.raw_data[2] = 0xbd; //mag_ref_MSB
			_cdata.raw_data[3] = 0x12; //mag_ref_LSB
			_cdata.raw_data[4] = 0x98; //xtalk_i_MSB
			_cdata.raw_data[5] = 0xbc; //xtalk_i_LSB
			_cdata.raw_data[6] = 0x4a; //xtalk_q_MSB
			_cdata.raw_data[7] = 0x83; //xtalk_q_LSB
			_cdata.raw_data[8] = 0x46; //xtalk_i_EXP
			_cdata.raw_data[9] = 0x47; //xtalk_q_EXP
			_cdata.raw_data[10] = 0xff; //xtalk_gain_MSB
			_cdata.raw_data[11] = 0x0; //xtalk_gain_LSB
			_cdata.raw_data[12] = 0x10; //distance_phase_MSB
			_cdata.raw_data[13] = 0x3c; //distance_phase_LSB
			_cdata.raw_data[14] = 0; //afe_temp
			_cdata.raw_data[15] = 0xd6; //afe_temp
		}else{
			pr_err("[debug] %s read cali_file %s success, use default calibration data",__func__,CALIFILE);
			for(i = 0; i < sizeof(_cdata.raw_data); i++){
				pr_err("[debug]:_cdata.raw_data[%d]=0x%d",i,_cdata.raw_data[i]);
			}
			pr_err("[debug] %s write the default calibration data to %s",__func__,CALIFILE);
			pos =0;
    			vfs_write(cali_file,  _cdata.raw_data, sizeof(_cdata.raw_data), &pos);
		}

		set_fs(fs);
	}else{
#endif
        	//Flash_pull(FLASH_ZONE_1,sizeof(_cdata.raw_data),_cdata.raw_data);
		pr_err("[debug] %s no cali_file %s available, use default calibration data",__func__,CALIFILE);
#if 0 //at the board at ASMA5D2 Platform
		_cdata.raw_data[0] = 0x7;  //cali_flag
		_cdata.raw_data[1] = 0x7;         //mag_ref_EXP 0x2c
		_cdata.raw_data[2] = 0xa6;//0xcd; //mag_ref_MSB 0x2d
		_cdata.raw_data[3] = 0x5e;//0x2c; //mag_ref_LSB 0x2e
		_cdata.raw_data[4] = 0x5a;//0x55; //xtalk_i_MSB 0x25
		_cdata.raw_data[5] = 0x73;//0x5a; //xtalk_i_LSB 0x26
		_cdata.raw_data[6] = 0x41;//0x7d; //xtalk_q_MSB 0x28
		_cdata.raw_data[7] = 0x5a;//0x63; //xtalk_q_LSB 0x29
		_cdata.raw_data[8] = 0x45;//0x46; //xtalk_i_EXP 0x24
		_cdata.raw_data[9] = 0x48;//0x47; //xtalk_q_EXP 0x27
		_cdata.raw_data[10] = 0xff;       //xtalk_gain_MSB 0x2a
		_cdata.raw_data[11] = 0x0;        //xtalk_gain_LSB 0x2b
		_cdata.raw_data[12] = 0xf;//0x11; //distance_phase_MSB 0x2f
		_cdata.raw_data[13] = 0xca;//0x6c;//distance_phase_LSB 0x30
		_cdata.raw_data[14] = 0; //afe_temp
		_cdata.raw_data[15] = 0xdf;//0x9f; //afe_temp
#else
                _cdata.raw_data[0] = 0x7;  //cali_flag
                _cdata.raw_data[1] = 0x7;         //mag_ref_EXP 0x2c
                _cdata.raw_data[2] = 0x9c;//0xcd; //mag_ref_MSB 0x2d
                _cdata.raw_data[3] = 0x7e;//0x2c; //mag_ref_LSB 0x2e
                _cdata.raw_data[4] = 0x53;//0x55; //xtalk_i_MSB 0x25
                _cdata.raw_data[5] = 0xd3;//0x5a; //xtalk_i_LSB 0x26
                _cdata.raw_data[6] = 0x4e;//0x7d; //xtalk_q_MSB 0x28
                _cdata.raw_data[7] = 0xb7;//0x63; //xtalk_q_LSB 0x29
                _cdata.raw_data[8] = 0x45;//0x46; //xtalk_i_EXP 0x24
                _cdata.raw_data[9] = 0x47;//0x47; //xtalk_q_EXP 0x27
                _cdata.raw_data[10] = 0xff;       //xtalk_gain_MSB 0x2a
                _cdata.raw_data[11] = 0x0;        //xtalk_gain_LSB 0x2b
                _cdata.raw_data[12] = 0x11;//0x11; //distance_phase_MSB 0x2f
                _cdata.raw_data[13] = 0xd;//0x6c;//distance_phase_LSB 0x30
                _cdata.raw_data[14] = 0; //afe_temp
                _cdata.raw_data[15] = 0xfc;//0x9f; //afe_temp

#if 0
<3>[  208.513500] raw_data[0]=0x7
<3>[  208.513761] raw_data[1]=0x7
<3>[  208.514016] raw_data[2]=0x9c
<3>[  208.514282] raw_data[3]=0x7e
<3>[  208.514547] raw_data[4]=0x53
<3>[  208.514812] raw_data[5]=0xd3
<3>[  208.515076] raw_data[6]=0x4e
<3>[  208.515406] raw_data[7]=0xb7
<3>[  208.515673] raw_data[8]=0x45
<3>[  208.515939] raw_data[9]=0x47
<3>[  208.516204] raw_data[10]=0xff
<3>[  208.516479] raw_data[11]=0x0
<3>[  208.516744] raw_data[12]=0x11
<3>[  208.517009] raw_data[13]=0xd
<3>[  208.517275] raw_data[14]=0x0
<3>[  208.517539] raw_data[15]=0xfc

#endif	
#endif
#if 0
	}
#endif

        if(drv_501_calibrate_data_check(&_cdata) == 0)
        {
                pr_err("[debug] %s check cali data done.\n",__func__);
          	if(drv_501_setup_cali_data(isl29501,&_cdata) == 0) pr_err("[debug] %s set cali data done.\n",__func__);
        }
        else
        {
                memset(&_cdata.raw_data,0,sizeof(_cdata.raw_data));
                pr_err("No vaild cali data yet.!!!\n");
        }
#endif
	return isl29501_begin_acquisition(isl29501);
}

static irqreturn_t isl29501_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct isl29501_private *isl29501 = iio_priv(indio_dev);
	const unsigned long *active_mask = indio_dev->active_scan_mask;
	u32 buffer[4] = {}; /* 1x16-bit + ts */
	pr_err("[debug] %s,active_mask:0x%x",__func__,*active_mask);

	if (test_bit(ISL29501_DISTANCE_SCAN_INDEX, active_mask))
		isl29501_register_read(isl29501, REG_DISTANCE, buffer);

	iio_push_to_buffers_with_timestamp(indio_dev, buffer, pf->timestamp);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int isl29501_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct isl29501_private *isl29501;
	int ret;

	struct device_node *np = client->dev.of_node;

	dev_err(&client->dev, "isl29501_probe 0228\n");
	indio_dev = iio_device_alloc(sizeof(*isl29501));
	if (!indio_dev)
		return -ENOMEM;

	isl29501 = iio_priv(indio_dev);

	i2c_set_clientdata(client, indio_dev);
	isl29501->client = client;

	mutex_init(&isl29501->lock);
       
	ret = isl29501_init_chip(isl29501, np);
    	if (ret < 0)
            return ret;
    	dev_err(&client->dev, "isl29501_init_chip(isl29501) pass\n");
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = &client->dev;
	indio_dev->channels = isl29501_channels;
	indio_dev->num_channels = ARRAY_SIZE(isl29501_channels);
	indio_dev->name = client->name;
	indio_dev->info = &isl29501_info;

	ret = iio_triggered_buffer_setup(indio_dev,
                                              iio_pollfunc_store_time,
                                             isl29501_trigger_handler,
                                             NULL);
	if (ret < 0) {
           dev_err(&client->dev, "unable to setup iio triggered buffer\n");
           return ret;
	}
	
	dev_err(&client->dev, "[debug] isc29501: setup iio triggered buffer !!\n");
	return iio_device_register(indio_dev);
}

static const struct i2c_device_id isl29501_id[] = {
	{"isl29501", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, isl29501_id);

#if defined(CONFIG_OF)
static const struct of_device_id isl29501_i2c_matches[] = {
	{ .compatible = "renesas,isl29501" },
	{ }
};
MODULE_DEVICE_TABLE(of, isl29501_i2c_matches);
#endif

static struct i2c_driver isl29501_driver = {
	.driver = {
		.name	= "isl29501",
	},
	.id_table	= isl29501_id,
	.probe		= isl29501_probe,
};
module_i2c_driver(isl29501_driver);

MODULE_AUTHOR("Mathieu Othacehe <m.othacehe@gmail.com>");
MODULE_DESCRIPTION("ISL29501 Time of Flight sensor driver");
MODULE_LICENSE("GPL v2");
