#ifndef BSP_SPI_H
#define BSP_SPI_H
#include "app_type.h"

#define SPI_DUMMY_BYTE	0xFF

struct spi_op_t
{
	vsf_err_t 	(*init)(uint8_t index, uint32_t speed);
	vsf_err_t 	(*fini)(uint8_t index);
	vsf_err_t 	(*send)(uint8_t index, uint8_t dat);
	uint8_t 	  (*recv)(uint8_t index);
	vsf_err_t 	(*sendmuti)(uint8_t index, uint8_t *buf, uint32_t size);
	vsf_err_t 	(*recvmuti)(uint8_t index, uint8_t *buf, uint32_t size);
	
};

extern const struct spi_op_t spi_op;



#endif
