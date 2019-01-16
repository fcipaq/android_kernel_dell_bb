#ifndef __IA_CSS_ISYSAPI_H__
#define __IA_CSS_ISYSAPI_H__

/**
 * errno.h specified error codes to be used
 * URL:http://man7.org/linux/man-pages/man3/errno.3.html>
 */


/* The following is needed for the function arguments */
#include "ia_css_isysapi_types.h"

/* To define the HANDLE */
#include "type_support.h"


/* This should contain the driver specified info for sys */
struct ia_css_driver_sys_config
{
	void * mmio_base_address;
	void * page_table_base_address; /* for all mmu's*/
	void * firmware_address;
	unsigned int num_send_queues;
	unsigned int num_recv_queues;
	unsigned int send_queue_size; /* max # tokens per queue */
	unsigned int recv_queue_size; /* max # tokens per queue */
};

 /**
 * struct ia_css_isys_device_cfg_data - ISYS device configuration data
 * @mipi: ISYS MIPI SRAM buffer partition
 * @pixel: ISYS PIXEL SRAM buffer partition
 */
struct ia_css_isys_device_cfg_data {
	struct ia_css_driver_sys_config driver_sys;
	struct ia_css_isys_buffer_partition mipi;
	struct ia_css_isys_buffer_partition pixel;
 };


/**
 * ia_css_isys_device_open() - configure ISYS device
 * @ context : device handle output parameter
 * @config: device configuration data struct ptr as input parameter,
 * ?read only? by css fw until function return
 * Ownership, ISYS will only access read my_device during fct call
 * Purpose: Partitions shared ISYS resources (mipi and pixel buffer)
 * Initialise Host/ISYS messaging queues
 * All streams must be stopped when calling ia_css_isys_device_open()
 *
 * Return:  int type error code (errno.h)
 */
extern int ia_css_isys_device_open(
	HANDLE * context,
	struct ia_css_isys_device_cfg_data *config
);

 /**
 * ia_css_isys_stream_open() - open and configure a virtual stream
 * @ stream_handle: stream handle
 * @ stream_cfg: stream configuration data struct pointer, which is
 * "read only" by ISYS until function return
 * ownership, ISYS will only read access stream_cfg during fct call
 * Pre-conditions:
 * Any Isys/Ssys interface changes must call ia_css_isys_stream_open()
 * Post-condition:
 * On successful call, ISYS hardware resource (IBFctrl, ISL, DMAs)
 * are acquired and ISYS server is able to handle stream specific commands
 * Return:  int type error code (errno.h)
 */
extern int ia_css_isys_stream_open(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_stream_cfg_data *stream_cfg
);

/**
 * ia_css_isys_stream_close() - close virtual stream
 * @ stream_handle: stream identifier
 * release ISYS resources by freeing up stream HW resources
 * ouput pin buffers ownership is returned to the driver
 * Return: int type error code (errno.h)
 */
extern int ia_css_isys_stream_close(
	HANDLE context,
	unsigned int stream_handle
);

/**
 * ia_css_isys_stream_start() - starts handling a mipi virtual stream
 * @ stream_handle: stream identifier
 * @next_frame:
 * if next_frame != NULL: apply next_frame
 * settings asynchronously and start stream
 * This mode ensures that the first frame is captured
 * and thus a minimal start up latency
 * (preconditions: sensor streaming must be switched off)
 *
 * if next_frame == NULL: sensor can be in a streaming state,
 * all capture indicates commands will be
 * processed synchronously (e.g. on mipi SOF events)
 *
 * To be called once ia_css_isys_stream_open() successly called
 * On success, the stream's HW resources are in active state
 *
 * Object ownership: During this function call,
 * next_frame struct must be read but not modified by the ISYS,
 * and in addition the driver is not allowed to modify it
 * on function exit next_frame ownership is returned to
 * the driver and is no longer accesses by iSYS
 * next_frame contains a collection of
 * ia_css_isys_output_pin * and ia_css_isys_input_pin *
 * which point to the frame's "ouput/input pin info & data buffers",
 *
 * Upon the ia_css_isys_stream_start() call,
 * ia_css_isys_output_pin* or ia_css_isys_input_pin*
 * will now be owned by the ISYS
 * these ptr will enable runtime/dynamic ISYS configuration and also
 * to store and write captured payload data
 * at the address specified in ia_css_isys_output_pin_payload
 * These ptrs should no longer be accessed by any other
 * code until (ia_css_isys_output_pin) gets handed
 * back to the driver  via the response mechansim
 * ia_css_isys_stream_handle_response()
 * the driver is responsible for providing valid
 * ia_css_isys_output_pin* or ia_css_isys_output_pin*
 * Pointers set to NULL will simply not be used by the ISYS
 *
 * Return: int type error code (errno.h)
 */
extern int ia_css_isys_stream_start(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame
);

/**
 * ia_css_isys_stream_stop() - Stops a mipi virtual stream
 * @ stream_handle: stream identifier
 * stop both accepting new commands and processing
 * submitted capture indication commands
 * Support for Secure Touch
 * Precondition: stream must be started
 * Return: int type error code (errno.h)
 */
extern int ia_css_isys_stream_stop(
	HANDLE context,
	unsigned int stream_handle
);

/**
 * ia_css_isys_stream_flush() - stops a mipi virtual stream but
 * completes processing cmd backlog
 * @ stream_handle: stream identifier
 * stop accepting commands, but process
 * the already submitted capture indicates
 * Precondition: stream must be started
 * Return: int type error code (errno.h)
 */
extern int ia_css_isys_stream_flush(
	HANDLE context,
	unsigned int stream_handle
);

/**
 * ia_css_isys_stream_capture_indication()
 * captures "next frame" on stream_handle
 * @ stream_handle: stream identifier
 * @ next_frame: frame pin payloads are provided atomically
 * purpose: stream capture new frame command, Successfull calls will
 * result in frame output pins being captured
 *
 * To be called once ia_css_isys_stream_start() is successly called
 * On success, the stream's HW resources are in active state
 *
 * Object ownership: During this function call,
 * next_frame struct must be read but not modified by the ISYS,
 * and in addition the driver is not allowed to modify it
 * on function exit next_frame ownership is returned to
 * the driver and is no longer accesses by iSYS
 * next_frame contains a collection of
 * ia_css_isys_output_pin * and ia_css_isys_input_pin *
 * which point to the frame's "ouput/input pin info & data buffers",
 *
 * Upon the ia_css_isys_stream_capture_indication() call,
 * ia_css_isys_output_pin* or ia_css_isys_input_pin*
 * will now be owned by the ISYS
 * these ptr will enable runtime/dynamic ISYS configuration and also
 * to store and write captured payload data
 * at the address specified in ia_css_isys_output_pin_payload
 * These ptrs should no longer be accessed by any other
 * code until (ia_css_isys_output_pin) gets handed
 * back to the driver  via the response mechansim
 * ia_css_isys_stream_handle_response()
 * the driver is responsible for providing valid
 * ia_css_isys_output_pin* or ia_css_isys_output_pin*
 * Pointers set to NULL will simply not be used by the ISYS
 *
 * Return: int type error code (errno.h)
 */

extern int ia_css_isys_stream_capture_indication(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame
);

/**
 * ia_css_isys_stream_handle_response() - handle ISYS responses
 * @received_response: provides response info from the
 * "next response element" from ISYS server
 * received_response will be written to during the fct call and
 * can be read by the drv once fct is returned
 *
 * purpose: Allows the client to handle received ISYS responses
 * Upon an IRQ event, the driver will call ia_css_isys_stream_handle_response()
 * until the queue is emptied
 * Responses returning IA_CSS_ISYS_RESP_TYPE_PIN_DATA_READY to the driver will
 * hand back ia_css_isys_output_pin ownership to the drv
 * ISYS FW will not write/read access ia_css_isys_output_pin
 * once it belongs to the driver
 * Pre-conditions: ISYS client must have sent a CMDs to ISYS srv
 * Return: int type error code (errno.h)
 */
extern int ia_css_isys_stream_handle_response(
	HANDLE context,
	struct ia_css_isys_resp_info *received_response
);
/**
 * ia_css_isys_device_close() - close ISYS device
 * @ context : device handle output parameter
 * Purpose: Free up context and config structures
 * All streams must be stopped when calling ia_css_isys_device_close()
 *
 * Return:  int type error code (errno.h)
 */
extern int ia_css_isys_device_close(
	HANDLE context,
	unsigned int nof_streams
);
#endif /*__IA_CSS_ISYSAPI_H__ */
