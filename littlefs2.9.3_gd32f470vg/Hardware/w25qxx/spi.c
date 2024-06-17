#include "spi.h"

/*
Used for communication with Ww25Q16(Flash)
*/



//SPI1 SPI2  CLK = 60MHz other = 120MHz


//SPI0-FLASH   25Q16
void Spi0_Init(void)
{
	//PA4 - NSS
	//PA5 - CLK
	//PA6 - MISO
	//PB5 - MOSI
	spi_parameter_struct spi_init_struct;
	
	rcu_periph_clock_enable(RCU_SPI0);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOC);
	
	gpio_af_set(GPIOA, GPIO_AF_5, GPIO_PIN_5| GPIO_PIN_6);
	gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5| GPIO_PIN_6);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5| GPIO_PIN_6);
	
	gpio_af_set(GPIOB, GPIO_AF_5, GPIO_PIN_5);
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_5);
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_5);
	
	gpio_mode_set(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_4);
	gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_MAX, GPIO_PIN_4);
	
	spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	spi_init_struct.device_mode          = SPI_MASTER;
	spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
	spi_init_struct.nss                  = SPI_NSS_SOFT;
	spi_init_struct.prescale             = SPI_PSC_2;   //120M/2
	spi_init_struct.endian               = SPI_ENDIAN_MSB;
	spi_init(SPI0, &spi_init_struct);
	
	gpio_bit_set(GPIOC, GPIO_PIN_15);
	
	spi_enable(SPI0);
	
}

uint8_t Spi0_SendByte(uint8_t data)
{
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	
	spi_i2s_data_transmit(SPI0, data);
	
	while(RESET == spi_i2s_flag_get(SPI0, SPI_STAT_RBNE));
	
	return spi_i2s_data_receive(SPI0);
}

uint16_t Spi0_SendHalfWord(uint16_t data)
{
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
	
	spi_i2s_data_transmit(SPI0, data);
	
	while(RESET == spi_i2s_flag_get(SPI0, SPI_STAT_RBNE));
	
	return spi_i2s_data_receive(SPI0);
}






