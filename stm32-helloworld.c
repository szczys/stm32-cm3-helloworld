#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include "debounce-stm32-cm3.h"

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


static uint32_t getTime(void) {
  return ticks;
}

int main(void) {
	initClock();
	initIO();
  initSystick();

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
    }
    // Toggle GPIO12 with each button down event
    if (get_key_press(KEY0)) { gpio_toggle(GPIOE, GPIO12); }
    // Toggle GPIO13 with each held button event
    if (get_key_rpt(KEY0)) { gpio_toggle(GPIOE, GPIO13); }
	}

	return 0;
}

void sys_tick_handler(void)
{
	++ticks;
  if ((ticks%10) == 0) key_isr();
}
