// Switch Process Rutine (Common)
// 2014.06.23 Eleken
// 2014.08.27 modified, 
// rev_c1815@yahoo.co.jp

#ifndef _YSWITCH_H_
#define _YSWITCH_H_

#define SWITCH_CALL_INTERVAL 100 // in ms
#define SWITCH_LONGPUSH_INTERVAL 500 // in ms
#define LONGPUSH_WEIGHT (SWITCH_LONGPUSH_INTERVAL/SWITCH_CALL_INTERVAL)

#define YSWITCH_ENABLE_RENC 0 // rotary encoder routine

#define YSWITCH_ENABLE_PUSHING 1 // enable pushing detection (requires 22 bytes)

void GetSwitchState(const uint8_t curPortState, uint8_t* pSwState);

#define SW_LONG 0x40
#define SW_PUSHING 0x80
#define SW_UNPUSHED 0

#define SW_SW1	1
#define SW_SW2	2
#define SW_SW3	3
#define SW_SW4	4
//#define SW_SW5	5


#define SW_PORT_MASK 0b00001111
#define SW1_PUSHED   0b00001110
#define SW2_PUSHED   0b00001101
#define SW3_PUSHED   0b00001011
#define SW4_PUSHED   0b00000111
//#define SW5_PUSHED   0b00001111


#if (YSWITCH_ENABLE_RENC)

#define RENC_NORM 0
#define RENC_FWD 1
#define RENC_REV 2
#define RENC_ERR 3

#define RENC_PORT_MASK	0b00000011
#define RENC_STATE_00	0b00000000
#define RENC_STATE_01	0b00000001
#define RENC_STATE_02	0b00000010
#define RENC_STATE_03	0b00000011

void GetREncState(const uint8_t curPortState, uint8_t* pEncState);

#endif

#endif
