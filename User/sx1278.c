

#include "timer.h"
#include "nrf.h"
#include "spi.h"
#include "app_type.h"
#include "bspconfig.h"
#include "string.h"
#include "sx1278.h"
#include "radio/sx1276/sx1276Regs-LoRa.h"
//#include "radio/sx1276/sx1276Regs-Fsk.h"

#include "bsp/crc.h"
#include "bsp/exioe.h"

#include "math.h"

#define SX_CS_HIGH		NRF_GPIO->OUTSET = (1<<NRF_SX1278_CS);
#define SX_CS_LOW		NRF_GPIO->OUTCLR = (1<<NRF_SX1278_CS);

#define SX_RES_HIGH		NRF_GPIO->OUTSET = (1<<NRF_SX1278_RES);
#define SX_RES_LOW		NRF_GPIO->OUTCLR = (1<<NRF_SX1278_RES);

#define SX_PWR_HIGH		NRF_GPIO->OUTSET = (1<<NRF_SX1278_PWR);
#define SX_PWR_LOW		NRF_GPIO->OUTCLR = (1<<NRF_SX1278_PWR);



#define SX_MASK_WR		0x80
#define SX_MASK_RD		0x00

#define XTAL_FREQ		32000000
#define FREQ_STEP		61.03515625

#define DIOMAP1			(RFLR_DIOMAPPING1_DIO0_00)

#define FIFO_TX_BASE	0x00
#define FIFO_RX_BASE	0x00

enum sx1278_state
{
	SX1278_STATE_UNINIT = 0,
	SX1278_STATE_UNCONFIG,	
	SX1278_STATE_IDLE,
	SX1278_STATE_TXREQ,
	SX1278_STATE_TXPROCESS,
	SX1278_STATE_POWEROFF,
	SX1278_STATE_CHANGEFREQ,
};

vsf_err_t sx1278_reg_wr(uint8_t addr, uint8_t dat)
{
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, addr|SX_MASK_WR);
	spi_op.send(NRF_SX1278_SPI, dat);
	SX_CS_HIGH;
	
	return VSFERR_NONE;
}

uint8_t sx1278_reg_rd(uint8_t addr)
{
	uint8_t reg = 0xFF;
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, addr|SX_MASK_RD);
	reg = spi_op.recv(NRF_SX1278_SPI);
	SX_CS_HIGH;
	
	return reg;
}

vsf_err_t sx1278_fifo_wr(uint8_t addr, uint8_t *buf, uint8_t size)
{
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, REG_LR_FIFOADDRPTR|SX_MASK_WR);
	spi_op.send(NRF_SX1278_SPI, addr);
	SX_CS_HIGH;
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, REG_LR_FIFO|SX_MASK_WR);
	spi_op.sendmuti(NRF_SX1278_SPI, buf, size);
	SX_CS_HIGH;
	return VSFERR_NONE;
}

vsf_err_t sx1278_fifo_rd(uint8_t addr, uint8_t *buf, uint8_t size)
{
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, REG_LR_FIFOADDRPTR|SX_MASK_WR);
	spi_op.send(NRF_SX1278_SPI, addr);
	SX_CS_HIGH;
	SX_CS_LOW;
	spi_op.send(NRF_SX1278_SPI, REG_LR_FIFO|SX_MASK_RD);
	spi_op.recvmuti(NRF_SX1278_SPI, buf, size);
	SX_CS_HIGH;
	return VSFERR_NONE;
}

vsf_err_t sx1278_setchannel(uint32_t freq)
{
    freq = ( uint32_t )( ( double )freq / ( double )FREQ_STEP );
    sx1278_reg_wr( REG_LR_FRFMSB, ( uint8_t )( ( freq >> 16 ) & 0xFF ) );
    sx1278_reg_wr( REG_LR_FRFMID, ( uint8_t )( ( freq >> 8 ) & 0xFF ) );
    sx1278_reg_wr( REG_LR_FRFLSB, ( uint8_t )( freq & 0xFF ) );
	return VSFERR_NONE;
}

vsf_err_t sx1278_setopmode( uint8_t opMode )
{
	sx1278_reg_wr( REG_LR_OPMODE, ( sx1278_reg_rd( REG_LR_OPMODE ) & (~RFLR_OPMODE_MASK)) | opMode );
	return VSFERR_NONE;
}

#define SX1276_DEBUG

#ifdef SX1276_DEBUG
uint8_t debug[64];
#endif
void sx1278_onrxirq(void *param);



vsf_err_t sx1278_init(struct sx1278_param_t *param)
{
	uint8_t i,try = 3;
	memset(param, 0, sizeof(struct sx1278_param_t));
	
	//init control gpio
	NRF_GPIO->PIN_CNF[NRF_SX1278_CS] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
		
	NRF_GPIO->PIN_CNF[NRF_SX1278_RES] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
#ifdef NRF_SX1278_PWR
	NRF_GPIO->PIN_CNF[NRF_SX1278_PWR] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
#endif
	NRF_GPIO->PIN_CNF[NRF_SX1278_PAGAIN] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);

#ifdef NRF_SX1278_DEBUG_TX
NRF_GPIO->PIN_CNF[NRF_SX1278_DEBUG_TX] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
#endif

#ifdef NRF_SX1278_DEBUG_RX
NRF_GPIO->PIN_CNF[NRF_SX1278_DEBUG_RX] =
			  (GPIO_PIN_CNF_DIR_Output        	<< GPIO_PIN_CNF_DIR_Pos)
			| (GPIO_PIN_CNF_INPUT_Connect     	<< GPIO_PIN_CNF_INPUT_Pos)
			| (GPIO_PIN_CNF_PULL_Disabled     	<< GPIO_PIN_CNF_PULL_Pos)
			| (GPIO_PIN_CNF_DRIVE_S0S1        	<< GPIO_PIN_CNF_DRIVE_Pos)
			| (GPIO_PIN_CNF_SENSE_Disabled    	<< GPIO_PIN_CNF_SENSE_Pos);
#endif

	//reset it
	SX_PAGAIN_LOW;
	SX_RES_LOW;
	SX_CS_HIGH;


	spi_op.init(NRF_SX1278_SPI, NRF_SX1278_FREQ);
	
#ifdef NRF_SX1278_PWR	
	SX_PWR_HIGH;


	//wait power stable
	{
		TIMER delay;
		TIMER_SET(delay, 50);
		while(!(TIMER_CHECK(delay)));
	}
#endif	
	//wait power up
	
	//trans serval dat for delay
	for(i = 0 ; i < 128 ; i ++)
		spi_op.send(NRF_SX1278_SPI, 0xFF);
	
	
	SX_RES_HIGH;
	//trans serval dat for delay
	for(i = 0 ; i < 128 ; i ++)
		spi_op.send(NRF_SX1278_SPI, 0xFF);

setlora:
	sx1278_setopmode(RFLR_OPMODE_SLEEP);
	//trans serval dat for delay
	for(i = 0 ; i < 200 ; i ++)
		spi_op.send(NRF_SX1278_SPI, 0xFF);
	
	//config rf as lora
	sx1278_reg_wr( REG_LR_OPMODE, (sx1278_reg_rd(REG_LR_OPMODE)&(~RFLR_OPMODE_LONGRANGEMODE_MASK)) | RFLR_OPMODE_LONGRANGEMODE_ON );

	if((sx1278_reg_rd(REG_LR_OPMODE) & RFLR_OPMODE_LONGRANGEMODE_ON) == 0)
	{
		//fail to set lora retry
		try --;
		if(try)
			goto setlora;
		return VSFERR_FAIL;
	}
	
	//trans serval dat for delay
	for(i = 0 ; i < 64 ; i ++)
		spi_op.send(NRF_SX1278_SPI, 0xFF);
	
	//					0		1			2		3		4			5
	//set diomapping as rxdone 	rxtmrout 	fhss 	cad 	caddone 	modeready
    sx1278_reg_wr( REG_LR_DIOMAPPING1, DIOMAP1 );
    sx1278_reg_wr( REG_LR_DIOMAPPING2, 0x00 );
            


#ifdef SX1276_DEBUG
	//read all spi reg
	for(i = 0 ; i < 64 ; i ++)
		debug[i] = sx1278_reg_rd(i);
#endif	
	
	
	param->state = SX1278_STATE_UNCONFIG;
	
	
	//config exioe
	exioe_op.init(NRF_SX1278_DIO0_GPIOTE);
	exioe_op.config_onevt(NRF_SX1278_DIO0_GPIOTE ,sx1278_onrxirq, param);
	
	//config ppi trig cap
	NRF_PPI->CH[NRF_SX1278_PPI].EEP = (uint32_t)&NRF_SX1278_EVTCAP;
	NRF_PPI->CH[NRF_SX1278_PPI].TEP = (uint32_t)&NRF_SX1278_TASKCAP;
	
	NRF_PPI->CHENSET = 1 << NRF_SX1278_PPI; 
	
	
	return VSFERR_NONE;
}


vsf_err_t sx1278_fini(struct sx1278_param_t *param)
{
	param->state = SX1278_STATE_POWEROFF;
	
	return VSFERR_NONE;
}


vsf_err_t sx1278_config(struct sx1278_param_t *param, struct sx1276_config_t *config)
{
	if(param->state == SX1278_STATE_UNINIT)
		return VSFERR_FAIL;
	
	param->freq = config->freq;
	//setfreq first
	
	{
		uint32_t freq = ( uint32_t )( ( double )config->freq / ( double )FREQ_STEP );
		sx1278_reg_wr( REG_LR_FRFMSB,( uint8_t )( ( freq >> 16 ) & 0xFF ));
		sx1278_reg_wr( REG_LR_FRFMID,( uint8_t )( ( freq >> 8 ) & 0xFF ));
		sx1278_reg_wr( REG_LR_FRFLSB,( uint8_t )( ( freq ) & 0xFF ));
	}
	
	param->config = config;
	{
		uint8_t paConfig = 0;
		uint8_t paDac = 0;
		
		paConfig = sx1278_reg_rd( REG_LR_PACONFIG );
		paDac = sx1278_reg_rd( REG_LR_PADAC );

		if (config->freq > 860000000)
			paConfig = ( paConfig & RFLR_PACONFIG_PASELECT_MASK ) | RFLR_PACONFIG_PASELECT_RFO;
		else		
			paConfig = ( paConfig & RFLR_PACONFIG_PASELECT_MASK ) | RFLR_PACONFIG_PASELECT_PABOOST;
		
		paConfig = ( paConfig & RFLR_PACONFIG_MAX_POWER_MASK ) | 0x70;

		if( ( paConfig & RFLR_PACONFIG_PASELECT_PABOOST ) == RFLR_PACONFIG_PASELECT_PABOOST )
		{
			if( config->power > 17 )
			{
				paDac = ( paDac & RFLR_PADAC_20DBM_MASK ) | RFLR_PADAC_20DBM_ON;
			}
			else
			{
				paDac = ( paDac & RFLR_PADAC_20DBM_MASK ) | RFLR_PADAC_20DBM_OFF;
			}
			if( ( paDac & RFLR_PADAC_20DBM_ON ) == RFLR_PADAC_20DBM_ON )
			{
				if( config->power < 5 )
				{
					config->power = 5;
				}
				if( config->power > 20 )
				{
					config->power = 20;
				}
				paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( config->power - 5 ) & 0x0F );
			}
			else
			{
				if( config->power < 2 )
				{
					config->power = 2;
				}
				if( config->power > 17 )
				{
					config->power = 17;
				}
				paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( config->power - 2 ) & 0x0F );
			}
		}
		else
		{
			paDac = 0x84;
			if( config->power < -1 )
			{
				config->power = -1;
			}
			if( config->power > 14 )
			{
				config->power = 14;
			}
			paConfig = ( paConfig & RFLR_PACONFIG_OUTPUTPOWER_MASK ) | ( uint8_t )( ( uint16_t )( config->power + 1 ) & 0x0F );
		}
		
		sx1278_reg_wr( REG_LR_PACONFIG, paConfig );
		sx1278_reg_wr( REG_LR_PADAC, paDac );
	}
	
	sx1278_setopmode( RFLR_OPMODE_STANDBY );
	
	if( config->datarate > 12 )
    {
		config->datarate = 12;
    }
	else if( config->datarate < 6 )
    {
		config->datarate = 6;
	}
	
	if( config->freqhopon == true )
	{
		sx1278_reg_wr( REG_LR_PLLHOP, ( sx1278_reg_rd( REG_LR_PLLHOP ) & RFLR_PLLHOP_FASTHOP_MASK ) | RFLR_PLLHOP_FASTHOP_ON );
		sx1278_reg_wr( REG_LR_HOPPERIOD, config->hopperiod );
	}
	
	sx1278_reg_wr(REG_LR_MODEMCONFIG1, 
				(sx1278_reg_rd( REG_LR_MODEMCONFIG1 ) &
				RFLR_MODEMCONFIG1_BW_MASK &
				RFLR_MODEMCONFIG1_CODINGRATE_MASK &
				RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK )|
				(config->bandwidth)|(config->coderate << 1)|(config->fixlen));
                        
	sx1278_reg_wr( REG_LR_MODEMCONFIG2,
				(sx1278_reg_rd( REG_LR_MODEMCONFIG2 ) &
				RFLR_MODEMCONFIG2_SF_MASK &
				RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK &
				RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) |
				( config->datarate << 4 ) | ( config->crcon << 2 ) |
				( ( config->symbtimeout >> 8 ) & ~RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK ) );

	sx1278_reg_wr( REG_LR_SYMBTIMEOUTLSB, ( uint8_t )( config->symbtimeout & 0xFF ) );
    
	sx1278_reg_wr( REG_LR_MODEMCONFIG3, 
				( sx1278_reg_rd( REG_LR_MODEMCONFIG3 ) &
				RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK ) |
				( config->lowdatarateoptimize << 3 ) );

	        
	sx1278_reg_wr( REG_LR_PREAMBLEMSB, ( uint8_t )( ( config->preamblelen >> 8 ) & 0xFF ) );
	sx1278_reg_wr( REG_LR_PREAMBLELSB, ( uint8_t )( config->preamblelen & 0xFF ) );
	
	
	if( config->fixlen == 1 )
	{
		sx1278_reg_wr( REG_LR_PAYLOADLENGTH, config->payloadlen );
	}

	

	if( ( config->bandwidth == 9 ) && ( RF_MID_BAND_THRESH ) )
	{
		// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth 
		sx1278_reg_wr( REG_LR_TEST36, 0x02 );
		sx1278_reg_wr( REG_LR_TEST3A, 0x64 );
	}
	else if( config->bandwidth == 9 )
	{
		// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
		sx1278_reg_wr( REG_LR_TEST36, 0x02 );
		sx1278_reg_wr( REG_LR_TEST3A, 0x7F );
	}
	else
	{
		// ERRATA 2.1 - Sensitivity Optimization with a 500 kHz Bandwidth
		sx1278_reg_wr( REG_LR_TEST36, 0x03 );
	}
	
	if( config->datarate == 6 )
	{
		sx1278_reg_wr( 	REG_LR_DETECTOPTIMIZE, 
						(sx1278_reg_rd(REG_LR_DETECTOPTIMIZE)&RFLR_DETECTIONOPTIMIZE_MASK)
						|RFLR_DETECTIONOPTIMIZE_SF6 );
		sx1278_reg_wr( 	REG_LR_DETECTIONTHRESHOLD, 
						RFLR_DETECTIONTHRESH_SF6 );
	}
	else
	{
		sx1278_reg_wr( 	REG_LR_DETECTOPTIMIZE,
						( sx1278_reg_rd( REG_LR_DETECTOPTIMIZE ) &
						RFLR_DETECTIONOPTIMIZE_MASK ) |
						RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12 );
		sx1278_reg_wr( 	REG_LR_DETECTIONTHRESHOLD, 
						RFLR_DETECTIONTHRESH_SF7_TO_SF12 );
	}
	
	if( config->iqinverted == true )
	{
		sx1278_reg_wr( REG_LR_INVERTIQ, ( ( sx1278_reg_rd( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_ON ) );
		sx1278_reg_wr( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_ON );
	}
	else
	{
		sx1278_reg_wr( REG_LR_INVERTIQ, ( ( sx1278_reg_rd( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );
		sx1278_reg_wr( REG_LR_INVERTIQ2, RFLR_INVERTIQ2_OFF );
	}
	

	
	sx1278_reg_wr(REG_LR_FIFORXBASEADDR, FIFO_RX_BASE);
	sx1278_reg_wr(REG_LR_FIFOTXBASEADDR, FIFO_TX_BASE);
	
	sx1278_reg_wr( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
										//RFLR_IRQFLAGS_RXDONE |
										RFLR_IRQFLAGS_PAYLOADCRCERROR |
										RFLR_IRQFLAGS_VALIDHEADER |
										//RFLR_IRQFLAGS_TXDONE |
										RFLR_IRQFLAGS_CADDONE |
										RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
										RFLR_IRQFLAGS_CADDETECTED );
	
	//state inited
	param->state = SX1278_STATE_IDLE;
	
	return VSFERR_NONE;
}




uint32_t sx1278_getpkgtimeonair(struct sx1278_param_t *param, uint8_t pkglen)
{
	struct sx1276_config_t *config = param->config;
	double bw = 0.0;
	switch( config->bandwidth )
	{
	case 0: // 125 kHz
		bw = 125e3;
		break;
	case 1: // 250 kHz
		bw = 250e3;
		break;
	case 2: // 500 kHz
		bw = 500e3;
		break;
	}
	
	// Symbol rate : time for one symbol (secs)
	double rs = bw / ( 1 << config->datarate );
	double ts = 1 / rs;
	// time of preamble
	double tPreamble = ( config->preamblelen + 4.25 ) * ts;
	// Symbol length of payload and time
	double tmp = ceil( ( 8 * pkglen - 4 * config->datarate +
						 28 + 16 * config->crcon -
						 ( config->fixlen ? 20 : 0 ) ) /
						 ( double )( 4 * config->datarate -
						 ( ( config->lowdatarateoptimize > 0 ) ? 8 : 0 ) ) ) *
						 ( config->coderate + 4 );
	double nPayload = 8 + ( ( tmp > 0 ) ? tmp : 0 );
	double tPayload = nPayload * ts;
	// Time on air 
	double tOnAir = tPreamble + tPayload;
	// return us secs
	return floor( tOnAir * 1e6 + 0.999 );
}




vsf_err_t 	sx1278_config_onrecv(struct sx1278_param_t *param, void (*onrecv)(void *param, uint8_t *buf, uint8_t size, uint8_t rssi, uint32_t ts), void *onrecv_param)
{
	param->onrecv = onrecv;
	param->onrecv_param = onrecv_param;
	return VSFERR_NONE;
}

vsf_err_t sx1278_send(struct sx1278_param_t *param, uint8_t *buf, uint8_t size)
{
	struct sx1276_config_t *config = param->config;
	uint8_t end;
	
	sx1278_setopmode( RFLR_OPMODE_STANDBY );
	
	
	//clear irq flag
	if(config->freqhopon)
	{
		sx1278_reg_wr( REG_LR_HOPPERIOD, config->hopperiod );
		sx1278_reg_wr( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
										  RFLR_IRQFLAGS_RXDONE |
										  RFLR_IRQFLAGS_PAYLOADCRCERROR |
										  RFLR_IRQFLAGS_VALIDHEADER |
										  //RFLR_IRQFLAGS_TXDONE |
										  RFLR_IRQFLAGS_CADDONE |
										  //RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
										  RFLR_IRQFLAGS_CADDETECTED );
		
	}
	else
	{
		sx1278_reg_wr( REG_LR_HOPPERIOD, 0 );
		sx1278_reg_wr( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_RXTIMEOUT |
										  RFLR_IRQFLAGS_RXDONE |
										  RFLR_IRQFLAGS_PAYLOADCRCERROR |
										  RFLR_IRQFLAGS_VALIDHEADER |
										  //RFLR_IRQFLAGS_TXDONE |
										  RFLR_IRQFLAGS_CADDONE |
										  RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL |
										  RFLR_IRQFLAGS_CADDETECTED );
	}
	
	// Initializes the payload size
	sx1278_reg_wr( REG_LR_PAYLOADLENGTH, size);

	sx1278_fifo_wr(FIFO_TX_BASE, buf, size);
	
		
    sx1278_setopmode( RFLR_OPMODE_TRANSMITTER );
	

	//watfor trans end
	end = 1;
	while(end)
	{
		if((sx1278_reg_rd(REG_LR_IRQFLAGS)&RFLR_IRQFLAGS_TXDONE_MASK))
		{
			end = 0;
		}
	}
	sx1278_reg_wr( REG_LR_IRQFLAGSMASK, RFLR_IRQFLAGS_TXDONE);
	
	return VSFERR_NONE;
	
}

vsf_err_t sx1278_recv(struct sx1278_param_t *param, uint8_t *buf, uint8_t *size, uint8_t *rssi)
{
	uint8_t end;

	sx1278_setopmode( RFLR_OPMODE_STANDBY );
	
	sx1278_setopmode( RFLR_OPMODE_RECEIVER );
	
	//watfor trans end
	end = 1;
	while(end)
	{
		if((sx1278_reg_rd(REG_LR_IRQFLAGS)&RFLR_IRQFLAGS_RXDONE_MASK))
		{
			end = 0;
		}
	}
	
	//read dat out
	//get pkg size
	*size = sx1278_reg_rd(REG_LR_RXNBBYTES);
	
	//fifo read
	sx1278_fifo_rd(FIFO_RX_BASE, buf, *size);
	
	//read rssi
	*rssi = sx1278_reg_rd(REG_LR_PKTRSSIVALUE);
	
	//clear flag
	sx1278_reg_wr(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
	
	
	
	return VSFERR_NONE;
	
}



vsf_err_t sx1278_dosend(struct vsf_buffer_t *buf)
{
	uint16_t crc;
	sx1278_setopmode( RFLR_OPMODE_STANDBY );
	
	//start PA
	SX_PAGAIN_HIGH;
	
	//calc crc and fill in
	crc = crc_ccittfalse(buf->buffer, buf->size - 2);
	memcpy(buf->buffer + buf->size - 2, &crc, 2);

	sx1278_reg_wr( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
		
	// Initializes the payload size
	sx1278_reg_wr( REG_LR_PAYLOADLENGTH, buf->size);

	// Full buffer used for Tx            
	sx1278_reg_wr( REG_LR_FIFOTXBASEADDR, 0 );
	sx1278_reg_wr( REG_LR_FIFOADDRPTR, FIFO_TX_BASE );
	
	
	sx1278_fifo_wr(FIFO_TX_BASE, buf->buffer, buf->size);
	
    sx1278_setopmode( RFLR_OPMODE_TRANSMITTER );
	
	return VSFERR_NONE;
}


vsf_err_t sx1278_dorecv(struct vsf_buffer_t *buf)
{
	uint8_t pkgbaseaddr;
	if(buf->buffer == NULL)
		goto recverr;
	
	//incontinusmode noneed to changemode
	//sx1278_setopmode( RFLR_OPMODE_STANDBY );
	
	//read dat out
	//get pkg size
	buf->size = sx1278_reg_rd(REG_LR_RXNBBYTES);
	//get pkg start
	pkgbaseaddr = sx1278_reg_rd(REG_LR_FIFORXCURRENTADDR);
	
	//check is pkg size is valid
	if(buf->size > NRF_SX1278_MAXPKGSIZE)
		goto recverr;
	
	//fifo read
	sx1278_fifo_rd(pkgbaseaddr, buf->buffer, buf->size);
	
	//operation to small add delay between it
	sx1278_reg_wr(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
	//clear flag
	sx1278_reg_wr(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
	
	
	//check pkgcrc
	{
		uint16_t crcvalue;
		memcpy(&crcvalue, buf->buffer + buf->size - 2, 2);
		
		if(crc_ccittfalse(buf->buffer, buf->size - 2) != crcvalue)
			return VSFERR_FAIL;
	}
					
	//sx1278_setopmode( RFLR_OPMODE_RECEIVER );
	
	return VSFERR_NONE;

	
recverr:
	//clear flag
	sx1278_reg_wr(REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE);
	
	return VSFERR_FAIL;
}

uint8_t sx1278_getpkgrssi(struct sx1278_param_t *param)
{
	//read rssi
	return param->pkgrssi;
}



vsf_err_t sx1278_thread(struct sx1278_param_t *param, uint16_t t_us)
{
	uint32_t ts = t_us + GlobalTick * 1000;
	switch(param->state)
	{
		case SX1278_STATE_UNINIT:
		case SX1278_STATE_UNCONFIG:
			return VSFERR_NOT_READY;
		case SX1278_STATE_IDLE:
			//check for flags
			{
				if(sx1278_reg_rd(REG_LR_IRQFLAGS)&RFLR_IRQFLAGS_RXDONE)
				{
					uint8_t rssi;
					//NRF_GPIO->OUTSET = 1<<3;
				
					param->rx.buffer = param->recvbuf;
					param->rx.size = 0;
					if(sx1278_dorecv(&param->rx) == VSFERR_NONE)
					{
						rssi = sx1278_reg_rd(REG_LR_PKTRSSIVALUE);
						if(rssi > SX1276_RSSI_MAX)
							rssi = SX1276_RSSI_MAX;
						if(rssi < (SX1276_RSSI_MAX - 100))
							rssi = (SX1276_RSSI_MAX - 100);
					
						param->pkgrssi = rssi - (SX1276_RSSI_MAX - 100);
						//do recv cb
						if(param->onrecv != NULL)
							param->onrecv(param->onrecv_param, param->rx.buffer, param->rx.size, param->pkgrssi, ts);
						//NRF_GPIO->OUTCLR = 1<<3;
					
#ifdef NRF_SX1278_DEBUG_RX
					NRF_SX1278_DEBUG_RX_HIGH;
#endif	
					}
				}
			}
			
			//check is in recvmode
			//is in recv mode
			if((sx1278_reg_rd( REG_LR_OPMODE ) & RFLR_OPMODE_MASK) != RFLR_OPMODE_RECEIVER)
				sx1278_setopmode( RFLR_OPMODE_RECEIVER );
			
			break;
		case SX1278_STATE_TXREQ:
#ifdef NRF_SX1278_DEBUG_RX
			NRF_SX1278_DEBUG_RX_LOW;
#endif
			//NRF_GPIO->OUTSET = 1<<3;
#ifdef NRF_SX1278_DEBUG_TX
			NRF_SX1278_DEBUG_TX_HIGH;
#endif
			sx1278_dosend(&param->tx);
			param->state = SX1278_STATE_TXPROCESS;
			break;
		case SX1278_STATE_TXPROCESS:
			if((sx1278_reg_rd(REG_LR_IRQFLAGS)&RFLR_IRQFLAGS_TXDONE))
			{
				param->state = SX1278_STATE_IDLE;
				param->tx.size = 0;
				
				//clear flag
				sx1278_reg_wr( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE);
	
				SX_PAGAIN_LOW;
				sx1278_setopmode( RFLR_OPMODE_RECEIVER );
				//NRF_GPIO->OUTCLR = 1<<3;
#ifdef NRF_SX1278_DEBUG_TX
			NRF_SX1278_DEBUG_TX_LOW;
#endif
			}
			break;
		case SX1278_STATE_POWEROFF:
			{
				sx1278_reg_wr( REG_LR_DIOMAPPING1, 0x00 );
				sx1278_reg_wr( REG_LR_DIOMAPPING2, 0x00 );
				while ((sx1278_reg_rd(REG_LR_OPMODE) & RFLR_OPMODE_MASK) != RFLR_OPMODE_SLEEP)
					sx1278_setopmode(RFLR_OPMODE_SLEEP);

				//release port
#ifdef NRF_SX1278_PWR	
				SX_PWR_LOW;
#endif
				spi_op.fini(NRF_SX1278_SPI);

				//releaseport
				NRF_GPIO->DIRCLR = (1 << NRF_SX1278_RES)|(1 << NRF_SX1278_CS);
				SX_PAGAIN_LOW;
									//|(1 << NRF_SX1278_PAGAIN);
				
				param->state = SX1278_STATE_UNINIT;
			}
			break;
		case SX1278_STATE_CHANGEFREQ:
			sx1278_setopmode( RFLR_OPMODE_STANDBY );
			sx1278_setchannel( param->freq );
			sx1278_setopmode( RFLR_OPMODE_RECEIVER );
			param->state = SX1278_STATE_IDLE;
				
			break;
		default:
			return VSFERR_BUG;
	}
	return VSFERR_NONE; 
}

void sx1278_onrxirq(void *param)
{
	uint16_t val = NRF_SX1278_CAPTIME;
	sx1278_thread(param, val);
}

vsf_err_t sx1278_tx(struct sx1278_param_t *param, uint8_t *buf, uint8_t size)
{
	if(param->state != SX1278_STATE_IDLE)
		return VSFERR_NOT_READY;
	
	memcpy(param->sendbuf, buf, size);
	param->tx.buffer = param->sendbuf;
	param->tx.size = size;
	param->state = SX1278_STATE_TXREQ;
	return VSFERR_NONE;
}

vsf_err_t sx1278_setfreq(struct sx1278_param_t *param, uint32_t freq)
{
	if(param->state != SX1278_STATE_IDLE)
		return VSFERR_NOT_READY;
	param->freq = freq;
	
	param->state = SX1278_STATE_CHANGEFREQ;
	return VSFERR_NONE;			
}

const struct sx1278_op_t sx1278_op = 
{
	sx1278_init,
	sx1278_fini,
	sx1278_config,
	sx1278_config_onrecv,
	sx1278_setfreq,
	sx1278_thread,
	sx1278_tx
};


