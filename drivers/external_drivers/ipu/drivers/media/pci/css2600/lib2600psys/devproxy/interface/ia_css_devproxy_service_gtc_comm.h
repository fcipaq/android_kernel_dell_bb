#ifndef IA_CSS_DEVPROXY_SERVICE_GTC_COMM_H_
#define IA_CSS_DEVPROXY_SERVICE_GTC_COMM_H_

#include "type_support.h"  /* uint8_t, uint32_t */
#include "ia_css_devproxy_ctrl.h"

/* GTC service state structure */
struct ia_css_devproxy_service_gtc_state_type {
	uint32_t initialized;
	uint32_t num_configs;
	uint32_t used_configs;
	ia_css_devproxy_ctrl_reference_t init_ref;
	uint32_t num_lines;
	uint32_t ack_addr;
	uint32_t ack_token;
};

void ia_css_devproxy_service_gtc_init_ref(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *init_ref);

void ia_css_devproxy_service_gtc_route_out(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t ack_addr,
	uint32_t ack_token);

void ia_css_devproxy_service_gtc_config(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	uint32_t num_lines);


#endif /*IA_CSS_DEVPROXY_SERVICE_GTC_COMM_H_*/
