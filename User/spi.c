

#include "spi.h"
#include "nrf.h"
#include "app_type.h"
#include "bspconfig.h"

//#define USE_DMA

vsf_err_t spi_init(uint8_t index, uint32_t speed)
{
	switch(index)
	{
#ifdef NRF_SPI0_MOSI		
		case 0:
		
		//config spi pins
		//set all pin high
		NRF_GPIO->OUTSET = (1 << NRF_SPI0_SCLK)|(1 << NRF_SPI0_MOSI)|(1 << NRF_SPI0_MISO);
		
		NRF_GPIO->PIN_CNF[NRF_SPI0_SCLK] =
					  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI0_MOSI] =
					  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI0_MISO] =
					  (GPIO_PIN_CNF_DIR_Input      		<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Pulldown     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
					
		
		//disable spi
		NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
		
		
		//config spi
		NRF_SPI0->PSELMISO = NRF_SPI0_MISO;
		NRF_SPI0->PSELMOSI = NRF_SPI0_MOSI;
		NRF_SPI0->PSELSCK = NRF_SPI0_SCLK;
		
		NRF_SPI0->FREQUENCY = speed;
		
		NRF_SPI0->CONFIG = (NRF_SPI0_CPOL	<< SPI_CONFIG_CPOL_Pos)
						|  (NRF_SPI0_CPHA 	<< SPI_CONFIG_CPHA_Pos)
						|  (NRF_SPI0_ORDER	<< SPI_CONFIG_ORDER_Pos);
		
		//enable spi
		NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
		
		
		return VSFERR_NONE;
#endif

#ifdef NRF_SPI1_MOSI
		case 1:
		//config spi pins
		//set all pin high
		NRF_GPIO->OUTSET = (1 << NRF_SPI1_SCLK)|(1 << NRF_SPI1_MOSI)|(1 << NRF_SPI1_MISO);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_SCLK] =
					  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_MOSI] =
					  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_MISO] =
					  (GPIO_PIN_CNF_DIR_Input      		<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Pulldown     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
					
		//disable spi
		NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
		
		
		//config spi
		NRF_SPI1->PSELMISO = NRF_SPI1_MISO;
		NRF_SPI1->PSELMOSI = NRF_SPI1_MOSI;
		NRF_SPI1->PSELSCK = NRF_SPI1_SCLK;
		
		NRF_SPI1->FREQUENCY = speed;
		
		NRF_SPI1->CONFIG = (NRF_SPI1_CPOL	<< SPI_CONFIG_CPOL_Pos)
						|  (NRF_SPI1_CPHA 	<< SPI_CONFIG_CPHA_Pos)
						|  (NRF_SPI1_ORDER	<< SPI_CONFIG_ORDER_Pos);
		
		//enable spi
		NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
		

		return VSFERR_NONE;
#endif

		default:
		return VSFERR_NOT_AVAILABLE;
	}
	
}



vsf_err_t spi_fini(uint8_t index)
{
	switch(index)
	{

#ifdef NRF_SPI0_MOSI
		case 0:
		//disable spi
		NRF_SPI0->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
		//releaseport
		NRF_GPIO->PIN_CNF[NRF_SPI0_SCLK] =
					  (GPIO_PIN_CNF_DIR_Input        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI0_MOSI] =
					  (GPIO_PIN_CNF_DIR_Input        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI0_MISO] =
					  (GPIO_PIN_CNF_DIR_Input      		<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		return VSFERR_NONE;
#endif

#ifdef NRF_SPI1_MOSI
		case 1:
		//disable spi
		NRF_SPI1->ENABLE = (SPI_ENABLE_ENABLE_Disabled << SPI_ENABLE_ENABLE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_SCLK] =
					  (GPIO_PIN_CNF_DIR_Input        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_MOSI] =
					  (GPIO_PIN_CNF_DIR_Input        	<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
		NRF_GPIO->PIN_CNF[NRF_SPI1_MISO] =
					  (GPIO_PIN_CNF_DIR_Input      		<< GPIO_PIN_CNF_DIR_Pos)
					| (GPIO_PIN_CNF_INPUT_Disconnect    << GPIO_PIN_CNF_INPUT_Pos)
					| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
					| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
					| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		return VSFERR_NONE;
#endif
		default:
		return VSFERR_NOT_AVAILABLE;
	}
}

//spi is double buffer so must send twobyte
vsf_err_t spi_send(uint8_t index, uint8_t dat)
{
	switch(index)
	{

#ifdef NRF_SPI0_MOSI
		case 0:
		NRF_SPI0->EVENTS_READY = 0;
		NRF_SPI0->EVENTS_READY;
		
		NRF_SPI0->TXD = dat;
		//checkisready
		while((!NRF_SPI0->EVENTS_READY));
		NRF_SPI0->RXD;
		
		
		return VSFERR_NONE;
#endif

#ifdef NRF_SPI1_MOSI		
		case 1:
				
		NRF_SPI1->EVENTS_READY = 0;
		NRF_SPI1->EVENTS_READY;

		NRF_SPI1->TXD = dat;
		NRF_SPI1->EVENTS_READY;

		//checkisready
		while((!NRF_SPI1->EVENTS_READY));
		NRF_SPI1->RXD;

		return VSFERR_NONE;
#endif
		default:
		return VSFERR_NOT_AVAILABLE;
	}
}


uint8_t spi_recv(uint8_t index)
{
	switch(index)
	{
#ifdef NRF_SPI0_MOSI
		case 0:
		
		NRF_SPI0->EVENTS_READY = 0;
		NRF_SPI0->EVENTS_READY;
		NRF_SPI0->TXD = SPI_DUMMY_BYTE;
		
		while((!NRF_SPI0->EVENTS_READY));
		
		return NRF_SPI0->RXD;
#endif

#ifdef NRF_SPI1_MOSI		
		case 1:
		
		NRF_SPI1->EVENTS_READY = 0;
		NRF_SPI1->EVENTS_READY;
		NRF_SPI1->TXD = SPI_DUMMY_BYTE;
		
		while((!NRF_SPI1->EVENTS_READY));
		
		return NRF_SPI1->RXD;
#endif
		default:
		return SPI_DUMMY_BYTE;
	}
}

vsf_err_t spi_sendmuti(uint8_t index, uint8_t *buf, uint32_t size)
{
	switch(index)
	{
#ifdef NRF_SPI0_MOSI
		case 0:
		while(size--)
		{
			NRF_SPI0->EVENTS_READY = 0;
			NRF_SPI0->EVENTS_READY;
			NRF_SPI0->TXD = *buf;
			buf++;
			//checkisready
			while(!NRF_SPI0->EVENTS_READY);
			NRF_SPI0->RXD;
		
		}
		
		return VSFERR_NONE;
#endif

#ifdef NRF_SPI1_MOSI
		case 1:
		
		while(size--)
		{
			NRF_SPI1->EVENTS_READY = 0;
			NRF_SPI1->EVENTS_READY;
			NRF_SPI1->TXD = *buf;
			buf++;
			//checkisready
			while(!NRF_SPI1->EVENTS_READY);
			NRF_SPI1->RXD;
		
		}
		
		return VSFERR_NONE;
#endif		
		default:
		return VSFERR_NOT_AVAILABLE;
	}
}

vsf_err_t spi_recvmuti(uint8_t index, uint8_t *buf, uint32_t size)
{
	switch(index)
	{
#ifdef NRF_SPI0_MOSI
		case 0:
		while(size--)
		{
			NRF_SPI0->EVENTS_READY = 0;
			NRF_SPI0->TXD = 0xFF;
			
			//checkisready
			while(!NRF_SPI0->EVENTS_READY);
			*buf = NRF_SPI0->RXD;
			buf++;
		}
		return VSFERR_NONE;
#endif

#ifdef NRF_SPI1_MOSI
		case 1:
		
		while(size--)
		{
			NRF_SPI1->EVENTS_READY = 0;
			NRF_SPI1->TXD = 0xFF;
			
			//checkisready
			while(!NRF_SPI1->EVENTS_READY);
			*buf = NRF_SPI1->RXD;
			buf++;
		}
		
		return VSFERR_NONE;
#endif
		default:
		return VSFERR_NOT_AVAILABLE;
	}
}

const struct spi_op_t spi_op = 
{
	spi_init,
	spi_fini,
	spi_send,
	spi_recv,
	spi_sendmuti,
	spi_recvmuti
};


