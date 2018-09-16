#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/gpio.h>
#include "debounce-stm32-cm3.h"

volatile unsigned int key_press;
volatile unsigned int key_state;
volatile unsigned int key_rpt;

/*--------------------------------------------------------------------------
  FUNC: 9/15/18 - Used to read debounced button presses
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a button press
--------------------------------------------------------------------------*/
unsigned int get_key_press( unsigned int key_mask )
{
  cm_disable_interrupts();  // read and clear atomic !
  key_mask &= key_press;	  // read key(s)
  key_press ^= key_mask;    // clear key(s)
  cm_enable_interrupts();
  return key_mask;
}

/*--------------------------------------------------------------------------
  FUNC: 9/15/18 - Used to check for debounced buttons that are held down
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits is a button held long enough for
		its input to be repeated
--------------------------------------------------------------------------*/
unsigned int get_key_rpt( unsigned int key_mask ) 
{ 
  cm_disable_interrupts();  // read and clear atomic ! 
  key_mask &= key_rpt;      // read key(s) 
  key_rpt ^= key_mask;      // clear key(s) 
  cm_enable_interrupts();
  return key_mask; 
} 

/*--------------------------------------------------------------------------
  FUNC: 9/15/18 - Used to read debounced button released after a short press
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a quick press and release
--------------------------------------------------------------------------*/
unsigned int get_key_short( unsigned int key_mask ) 
{ 
  cm_disable_interrupts();  // read key state and key press atomic ! 
  return get_key_press( ~key_state & key_mask ); 
} 

/*--------------------------------------------------------------------------
  FUNC: 9/15/18 - Used to read debounced button held for REPEAT_START amount
	of time.
  PARAMS: A keymask corresponding to the pin for the button you with to poll
  RETURNS: A keymask where any high bits represent a long button press
--------------------------------------------------------------------------*/
unsigned int get_key_long( unsigned int key_mask ) 
{ 
  return get_key_press( get_key_rpt( key_mask )); 
}

/*--------------------------------------------------------------------------
  FUNC: 9/15/18 - Used to debounce buttons in an interrupt service routine
	that runs once every 10 milliseconds
  PARAMS: None
  RETURNS: None
--------------------------------------------------------------------------*/
void key_isr( void )
{ 
  static unsigned char ct0, ct1, rpt;  
  unsigned char i;
  i = key_state ^ gpio_port_read(KEY_PORT);    // key changed ? (natural state is high so no need for ~KEY_PIN)
  ct0 = ~( ct0 & i );          // reset or count ct0
  ct1 = ct0 ^ (ct1 & i);       // reset or count ct1
  i &= ct0 & ct1;              // count until roll over ?
  key_state ^= i;              // then toggle debounced state
  key_press |= key_state & i;  // 0->1: key press detect

  if( (key_state & REPEAT_MASK) == 0 )   // check repeat function 
     rpt = REPEAT_START;      // start delay 
  if( --rpt == 0 ){ 
    rpt = REPEAT_NEXT;         // repeat delay 
    key_rpt |= key_state & REPEAT_MASK; 
  }
}
