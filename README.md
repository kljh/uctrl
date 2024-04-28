

STM32 microcontrollers feature fast ADC, sampling with 12 bit resolution at up to 2 Megasamples per second.

USB 2.0 Fullspeed specification is 12 megabits per second, or 1.5 MBytes per second.

STM32 microcontrollers should be able to sample an analogic and stream ait over USB at up to 1.5 Megasamples per second.
The project below was tested using internal STM32F401 oscilator as 725 kSample per second, or 300 kHz bandwidth.

Not quite an oscilloscope yet , but not bad for a £10 board.


# STM32 MCU

STM32CubeIDE

STM32F4 Discovery  STM32F407VG
https://www.st.com/en/evaluation-tools/stm32f4discovery.html

Blackpill MCU STM32F401CCU6  256kB flash / 64kB RAM / 84MHz
HSI	16MHz
LSI	32kHz
HSE	25MHz
LSE	32.768kHz
User LED Connected to	PC13 / Mode Sink

# USB setup

Pinout & Config
 - Connectivity
  - USB_OTG_FS
     Mode = Device Only
	 Active VBUS = Yes (VBUS Sensing enabled)
 - Middleware & Software
  - USB_DEVICE
    Class for FS IP = Communication Device Class (CDC)

Linux host
lsusb
cat /dev/ttyACM0


# ADC setup

Pinout & Config
 - Change PA0 to ADC1_IN5.
 - Under Analog / ADC1,
    change IN5 to IN5 Single-ended.
 - Under Analog / ADC1 / Configuration / Parameter Settings / ADC_Settings
    change 'Continuous Conversion Mode' to Enabled.
 - System Core / DMA / DMA1 (or DMA2 depending on MCU and ADC),
    Add a new request, and change it to ADC1. Change Mode to Circular,
 - Back to Analog / ADC1 / Configuration / Parameter Settings / ADC_Settings
    change DMA Continuous Requests to Enabled.

12 bits
130 buffer per 1000ms
130x4096samples = 532480 samples/s = 532 kS/s +/- 4 kS
16 MHz APB Clock x 1/2 prescaler / 15 step/sample = 533 kS/s

8 bits
178 buffer per 1000ms
178x4096samples = 729088 samples/s = 729 kS/s+/- 4 kS
16 MHz APB Clock x 1/2 prescaler / 11 = 727 kS/s


- Change PA10 (which is connected to header pin D2) to GPIO_Output.


# PWM Setup

Pinout & Config
 - Change PA5 to TIM2_CH1
 - Under TImers / TIM2 / Mode
    change Clock Source to 'Internal Clock'.
    change Channel 1 to 'PWM Generation Channel 1'.
 - Under TImers / TIM2 / Configuration / Parameter Settings
    change 'Prescaler (PSC - 16 bits value)' to '16-1', It will bring back APB2 from 16MHz to 1MHz TIM Clock.
    change 'Counter Period (AutoReload Register - 32 bits value )' to '100-1'. It will set the PWM to 100 periods of the TIM Clock => 10kHz.
    change 'Auto Reload-Preload' to 'Enabled'.
 - Start in main function, before the while loop, with
    TIM2->CCR1 = 30;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);



Refs:
 - https://www.digikey.co.uk/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c
 - https://michael.stapelberg.ch/posts/2021-04-27-linux-usb-virtual-serial-cdc-acm/
 - https://deepbluembedded.com/stm32-interrupts-tutorial-nvic-exti/
