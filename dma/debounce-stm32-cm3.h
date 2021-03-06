#ifndef COMMANDS_H_
#define COMMANDS_H_

#define KEY0              GPIO0
#define KEY_PORT          GPIOA
#define KEY_CLOCK         RCC_GPIOA
//Add keys on the same port to keymask: (KEY0 | KEY1 | KEY2)
#define KEY_MASK          (KEY0)

/*********************** Debounce ****************************
* Info on debounce code:
* https://github.com/szczys/Button-Debounce/blob/master/debounce-test.c 
*/
//Debounce
#define REPEAT_MASK   (KEY0)   // repeat: key1, key2 
#define REPEAT_START  50       // after 500ms 
#define REPEAT_NEXT   70       // every 700ms
/************************************************************/

unsigned int get_key_press( unsigned int key_mask );
unsigned int get_key_rpt( unsigned int key_mask );
unsigned int get_key_short( unsigned int key_mask ); 
unsigned int get_key_long( unsigned int key_mask );
void key_isr( void );

#endif
