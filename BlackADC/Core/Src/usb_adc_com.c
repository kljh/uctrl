// #include "main.h"
// #include "usb_device.h"
#include "usbd_cdc_if.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

extern const uint32_t usb_adc_buf_len, usb_adc_buf_len_1, usb_adc_buf_len_2;
extern uint8_t  usb_adc_buf[];

extern uint32_t adc_samples_per_second;

uint8_t usb_adc_transmitting = 0;

uint8_t usb_status = USBD_OK;

uint8_t usb_msg[512];
uint16_t usb_msg_max_len = sizeof(usb_msg) - 1;

uint8_t usb_error[16];
uint16_t usb_error_max_len = sizeof(usb_msg) - 1;

uint16_t usb_adc_chunk = 0;
uint16_t usb_adc_chunks_per_buffer = 1;

uint8_t usb_adc_status()
{
	int adc_resolution = hadc1.Init.Resolution; // uint32_t
	if (hadc1.Init.Resolution == ADC_RESOLUTION_12B)
		adc_resolution = 12;
	else if (hadc1.Init.Resolution == ADC_RESOLUTION_10B)
		adc_resolution = 10;
	else if (hadc1.Init.Resolution == ADC_RESOLUTION_8B)
		adc_resolution = 8;
	else if (hadc1.Init.Resolution == ADC_RESOLUTION_6B)
		adc_resolution = 6;

	int adc_prescaler_div = 2;
	if (hadc1.Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV2)
		adc_prescaler_div = 2;
	else if (hadc1.Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV4)
		adc_prescaler_div = 4;
	else if (hadc1.Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV6)
		adc_prescaler_div = 6;
	else if (hadc1.Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV8)
		adc_prescaler_div = 8;

	int apb2_freq_Hz = 16000000;
	int adc_samples_per_second_theo = apb2_freq_Hz / adc_prescaler_div / (3+adc_resolution);

	// hadc1.Init.DataAlign

	int nbChars = snprintf(usb_msg, usb_msg_max_len, "\n"
			"USB_ADC_TRANSMIT\t%i\n"
			"USB_ADC_BUF_LEN\t%i\n"
			"USB_ERROR\t%s\n"
			"ADC_RESOLUTION\t%i\n"
			"ADC_SAMPLE_PER_SEC\t%i\n"
			"ADC_SAMPLE_PER_SEC_APB2_16MHz\t%i\n"
			,
			(int)usb_adc_transmitting,
			(int)usb_adc_buf_len,
			(char*)usb_error,
			(int)adc_resolution,
			(int)adc_samples_per_second,
			(int)adc_samples_per_second_theo
			);

	usb_status = CDC_Transmit_FS(usb_msg, nbChars);
	return usb_status;
}


uint8_t usb_adc_receive(uint8_t* buf, uint32_t *buf_len_ptr)
{
	uint32_t buf_len = *buf_len_ptr;

	if (buf_len > 1 && (buf[0]=='I' || buf[0]=='i'))
	{
		return usb_adc_status();
	}
	if (buf_len > 3 && (buf[0]=='S' || buf[0]=='s'))
	{
		if (buf[3]=='R' || buf[3]=='r')  // START
			usb_adc_transmitting = 2;
		else if (buf[4]=='P' || buf[3]=='p')  // STOP
			usb_adc_transmitting = 0;

		// restart from beginning of buffer in either case
		usb_adc_chunk = 0;
		snprintf(usb_error, usb_error_max_len, "OK");
	}
	return USBD_OK;
}

uint8_t usb_adc_transmit(uint8_t next_chunk)
{
	// When start new buffer (e.g. sending the first chnk of the buffer), next_chunk is 0.
	// When called to transfer subsequent chunks, next_chunk is 1.
	// We expect to be done transmitting all chunks before seeing, next_chunk = 0 again
	// (otherwise it means transmission over USB is too slow, either >12Mbps or limited by host)

	if (usb_cdc_initialised == 0)
	{
		return USBD_BUSY;
	}

    if (usb_adc_transmitting == 0)
    {
    	return USBD_OK;
    }

    if (usb_adc_transmitting > 2)
    {
    	return USBD_BUSY;
	}

	// check usb_adc_chunk counter
	if (next_chunk == 0) {
		if (usb_adc_chunk != 0)
		{
			// stop transmission
			usb_adc_transmitting = 0;

			// set error
			snprintf(usb_error, usb_error_max_len, "TOO FAST");

			return USBD_BUSY;
		} else {
			usb_adc_chunk = 0;
		}
	}

	// transmit chunk (if any left)
	if (usb_adc_chunk < usb_adc_chunks_per_buffer)
	{
		// transmit half buffer as a single chunk
		uint8_t* usb_buf = usb_adc_buf + ( usb_adc_transmitting == 1 ? 0 : usb_adc_buf_len_1 );
		uint16_t usb_buf_len = usb_adc_transmitting == 1 ? usb_adc_buf_len_1 : usb_adc_buf_len_2;
		usb_status = CDC_Transmit_FS(usb_buf, usb_buf_len);

		if (usb_status != USBD_OK)
		{

			// stop transmission
			usb_adc_transmitting = 0;

			// set error
			snprintf(usb_error, usb_error_max_len, "USB BUSY");

			return usb_status;
		}

		// increment chunk counter
		usb_adc_chunk++;
	} else {
		// transmission of all chunks complete
		usb_status = USBD_OK;
		usb_adc_chunk = 0;
	}

	return usb_status;
}
