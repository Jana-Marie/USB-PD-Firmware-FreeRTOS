/*
 * tcpm_driver.c
 *
 * Created: 11/11/2017 18:42:26
 *  Author: jason
 */ 

#include "tcpm_driver.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT];


/* I2C wrapper functions - get I2C port / slave addr from config struct. */
int tcpc_write(int port, int reg, int val)
{
	uint8_t data[2];
	data[0] = (0xFF) & reg;
	data[1] = (0xFF) & val;

	return HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);
}

int tcpc_write16(int port, int reg, int val)
{
	uint8_t data[3];
	data[0] = (0xFF) & reg;
	data[1] = (0xFF) & val;
	data[2] = (0xFF) & (val >> 8);
		
	return HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);//i2c_master_write_packet_wait(&i2c_master_instance, &packet);
}

int tcpc_read(int port, int reg, int *val)
{
	uint8_t data[1];
	data[0] = (0xFF) & reg;
	
	
	HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);//HAL_I2C_Master_Transmit(&hi2c1,  tcpc_config[port].i2c_slave_addr, &packet, sizeof(packet),1);i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);


	HAL_I2C_Master_Receive(&hi2c1,  tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);//i2c_master_read_packet_wait(&i2c_master_instance, &packet);

	*val = data[0];
	
	return 1;
}

int tcpc_read16(int port, int reg, int *val)
{
	uint8_t data[2];
	data[0] = (0xFF) & reg;
	
	
	HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);//i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);


	HAL_I2C_Master_Receive(&hi2c1,  tcpc_config[port].i2c_slave_addr, data, sizeof(data),1);//i2c_master_read_packet_wait(&i2c_master_instance, &packet);

	
	*val  = data[0];
	*val |= (data[1] << 8);
	
	return 1;
}

int tcpc_xfer(int port,
	uint8_t *out, int out_size,
	uint8_t *in, int in_size,
	int flags)
{

	if (out_size != 0)
	{
		if (flags & I2C_XFER_STOP)
		{
			HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, out, sizeof(out_size),1);//i2c_master_write_packet_wait(&i2c_master_instance, &packet);
		}
		else
		{
			HAL_I2C_Master_Transmit(&hi2c1, tcpc_config[port].i2c_slave_addr, out, sizeof(out_size),1);//i2c_master_write_packet_wait_no_stop(&i2c_master_instance, &packet);
		}
	}

	if (in_size != 0)
	{
		// We always send a stop, then another start on the next call. 
		// This avoids trying to read after sending a NACK, which indicates the end of the read. 
		// In the future, this could be optimized by using lower level i2c calls. 
		HAL_I2C_Master_Receive(&hi2c1,  tcpc_config[port].i2c_slave_addr, in, sizeof(in_size),1);//i2c_master_read_packet_wait(&i2c_master_instance, &packet);
	}

	return 1;
}
