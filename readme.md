# Hello world on STM32 Discovery F3 using libopencm3

Trying out common tasks on an STM32F303 chip (STM32 F3 Discovery) using the libopencm3 peripheral library.
This library is included as a git submodule.

# Demonstrations:

## helloworld
 
* GPIO abstraction to address LEDs and read from user button
* Systick to generate 1ms interrupt
  * Non-blocking millisecond timer
  * Button debounce every 10ms
* UART2 messages 11520 8N1
  * "HelloWorld" printed at 1 Hz
  * "Button!" printed when user button is pressed


# Setup libopencm3 as a submodule:
git submodule add -b master https://github.com/libopencm3/libopencm3.git
git submodule init


