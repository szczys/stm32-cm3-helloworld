#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/dma.h>
#include "debounce-stm32-cm3.h"
#include <stdio.h>

#define MY_USART  USART1
unsigned char message1[] = "HelloWorld";
unsigned char message2[] = "Button!";

uint8_t s1[20] = "Hello STM MEM2MEM\r\n";
uint8_t s2[20] = "Default Text";

volatile uint32_t ticks = 0;

static void initClock(void) {
  //Setup system clock for 64 MHz
  rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}

static void initIO(void) {
  // LED Setup
  //Enable peripheral clock
  rcc_periph_clock_enable(RCC_GPIOE);
  //Set discovery board LED pins as outputs
  gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, (GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13));

  // Button setup
	// Enable peripheral clock
	rcc_periph_clock_enable(RCC_GPIOA);
	// Set discovery board user button as input
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);
}

static void initSystick(void) { 
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8); // 64 MHz / 8 = 8 MHz
  systick_set_reload(7999);  // 8,000,000 / 8,000 = 1,000 (1ms interrupts)
  systick_interrupt_enable(); // Enable the systick interrupt
	systick_counter_enable(); // Start the systick
}

static void initUsart(void) {
	// Enable peripheral clock
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOA);   //Should already be enabled for button

	// Setup USART2 on PA2
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO2);

	// Handle the baud settings
  usart_set_mode(USART2, USART_MODE_TX);
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
	usart_enable(USART2);
}

static void initDMA(void) {
  //Enable DMA peripheral clock
  rcc_periph_clock_enable(RCC_DMA1);
  //Choose DMA channel
  //Set memory to memory
  dma_enable_mem2mem_mode(DMA1, DMA_CHANNEL1);
  //Setup the priority
  dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_VERY_HIGH);
  //Set the data size
  dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);
  dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);
  //Choose to increment after each write (both on source and destination)
  dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
  dma_enable_peripheral_increment_mode(DMA1, DMA_CHANNEL1);
  //Choose source (peripheral or memory)
  dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
  //Choose data transfer size
  dma_set_number_of_data(DMA1, DMA_CHANNEL1, 5);
  //Choose source address
  dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&s1);
  //Choose destination address
  dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&s2);
  //Enable transfer complete interrupt (are we going to use it in this demo?)
  //Enable the transfer
}

static uint32_t getTime(void) {
  return ticks;
}

static void sendMessage(uint8_t *s) {
  uint8_t i;
  while (*s != 0) { 
    usart_send_blocking(USART2, *s);
    s++;
  }
  usart_send_blocking(USART2, '\r');
  usart_send_blocking(USART2, '\n');
}

int main(void) {
	initClock();
	initIO();
  initSystick();
  initUsart();
  initDMA();

	/* Set two LEDs for wigwag effect when toggling. */
	gpio_set(GPIOE, GPIO8);

	//unsigned int i;
  uint32_t waitUntil = 1000;
	/* Blink the LEDs (PD8, PD9, PD10 and PD11) on the board. */
	while (1) {
    if (getTime() > waitUntil) {
      waitUntil = getTime()+1000;
		  /* Toggle LEDs. */
		  gpio_toggle(GPIOE, GPIO8 | GPIO9 | GPIO10 | GPIO11);
      // Send a message on USART2 (PA2 at 115200 8N1)
      sendMessage(s2);
    }
    // Toggle GPIO12 with each button down event
    if (get_key_press(KEY0)) {
      gpio_toggle(GPIOE, GPIO12);
      sendMessage(message2);
    }
    // Toggle GPIO13 with each held button event
    if (get_key_rpt(KEY0)) { 
      gpio_toggle(GPIOE, GPIO13);
      dma_enable_channel(DMA1, DMA_CHANNEL1);
      while (!(DMA_ISR(DMA1) & 0x0000001)) { ;; }
      dma_disable_channel(DMA1, DMA_CHANNEL1);
    }
	}

	return 0;
}

void sys_tick_handler(void)
{
	++ticks;
  if ((ticks%10) == 0) key_isr();
}
