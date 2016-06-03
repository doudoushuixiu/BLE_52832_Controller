#ifndef BSP_SX1278_H
#define BSP_SX1278_H
#include "app_type.h"
#include "component/buffer/buffer.h"

#define SX1276_RSSI_MAX	(-20 + 157)

struct sx1276_config_t
{
	uint32_t freq;
	uint32_t bandwidth;
	uint32_t datarate;
	uint16_t preamblelen;
	uint16_t symbtimeout;
	uint8_t coderate;
	int8_t 	power;
	uint8_t payloadlen;
	uint8_t hopperiod;
	uint8_t fixlen;
	uint8_t crcon;
	uint8_t iqinverted;
	uint8_t rxcontinuous;
	uint8_t freqhopon;
	uint8_t lowdatarateoptimize;
	
};

struct sx1278_param_t
{
	struct sx1276_config_t *config;
	uint32_t 				freq;
	struct vsf_buffer_t		tx;
	struct vsf_buffer_t		rx;
	void 					(*onrecv)(void *param, uint8_t *buf, uint8_t size, uint8_t rssi, uint32_t ts);
	void 					*onrecv_param;
	uint8_t					recvbuf[NRF_SX1278_MAXPKGSIZE];
	uint8_t					sendbuf[NRF_SX1278_MAXPKGSIZE];
	uint8_t 				pkgrssi;
	uint8_t 				state;
};

struct sx1278_op_t
{
	vsf_err_t 	(*init)(struct sx1278_param_t *param);
	vsf_err_t 	(*fini)(struct sx1278_param_t *param);
	vsf_err_t 	(*config)(struct sx1278_param_t *param, struct sx1276_config_t *config);
	vsf_err_t 	(*config_onrecv)(struct sx1278_param_t *param, void (*onrecv)(void *param, uint8_t *buf, uint8_t size, uint8_t rssi, uint32_t ts), void *onrecv_param);
	vsf_err_t 	(*setfreq)(struct sx1278_param_t *param, uint32_t freq);
	vsf_err_t 	(*thread)(struct sx1278_param_t *param, uint16_t t_us);
	vsf_err_t 	(*tx)(struct sx1278_param_t *param, uint8_t *buf, uint8_t size);
};

extern const struct sx1278_op_t sx1278_op;

#endif
