// Switch Process Rutine
// 2014.06.23 Eleken
// 2014.08.27 modified, 
// rev_c1815@yahoo.co.jp

// No usage. 

#include <stdint.h>
#include "YSwitchProc.h"


// Call this function per 50-200 ms.
void GetSwitchState(const uint8_t curPortState, uint8_t* pSwState)
{
	static uint8_t sw_old;
	static uint8_t longpushCnt;
	static uint8_t s;
	
	
	{ // check switch state
		uint8_t sw = (curPortState & SW_PORT_MASK);
		
#if (YSWITCH_ENABLE_PUSHING)
		if((sw != SW_PORT_MASK) && 
			(longpushCnt > LONGPUSH_WEIGHT)){ // being long-pushed
			*pSwState = SW_PUSHING | s;
		}else if(sw_old != sw){ // switch state has changed
#else
		if(sw_old != sw){ // switch state has changed
#endif
			if(sw == SW1_PUSHED){//
				s = SW_SW1;
			}
#ifdef SW_SW2
			else if(sw == SW2_PUSHED){//
				s = SW_SW2;
			}
#ifdef SW_SW3
			else if(sw == SW3_PUSHED){//
				s = SW_SW3;
			}
#ifdef SW_SW4
			else if(sw == SW4_PUSHED){//
				s = SW_SW4;
			}
#ifdef SW_SW5
			else if(sw == SW5_PUSHED){//
				s = SW_SW5;
			}
#endif
#endif
#endif
#endif
			else if(sw == SW_PORT_MASK){// released
				if(longpushCnt == 0) {
					//sw_state = 0; // too short
				}else if(longpushCnt <= LONGPUSH_WEIGHT) {// < 500ms
					*pSwState = s; // short push
				}else {
					*pSwState = SW_LONG | s; // long push
				}
				s = SW_UNPUSHED;
			}else {
				s = SW_UNPUSHED;
			}
		}
		
		if(sw != SW_PORT_MASK){ // Pushed
			longpushCnt ++;
		}else{
			longpushCnt = 0;
		}
		sw_old = sw;
		
	}
	
	
	return ;
}


#if (YSWITCH_ENABLE_RENC)

const uint8_t encStateTable[] PROGMEM = 
{
	RENC_NORM, RENC_FWD, RENC_REV, RENC_ERR,  // state_old = 0
	RENC_REV, RENC_NORM, RENC_ERR, RENC_FWD,   // state_old = 1
	RENC_FWD, RENC_ERR, RENC_NORM, RENC_REV,   // state_old = 2
	RENC_ERR, RENC_REV, RENC_FWD, RENC_NORM,   // state_old = 3
};

// Get Rotary Encoder Operation
// Call this function per 2-8 ms.
void GetREncState(const uint8_t curPortState, uint8_t* pEncState)
{
	static uint8_t sw_old;
	static uint8_t state_old;
	
	
	{ // check switch state
		static uint8_t lastDirection;
		
		uint8_t sw = (curPortState & RENC_PORT_MASK);
		//if(sw_old != sw){ // switch state has changed
			uint8_t state;
#if (RENC_PORT_MASK == 0b00000011)
				state = sw;
#else
			if(sw == RENC_STATE_00){//
				state = 0;
			}
			else if(sw == RENC_STATE_01){//
				state = 1;
			}
			else if(sw == RENC_STATE_02){//
				state = 2;
			}
			else if(sw == RENC_STATE_03){//
				state = 3;
			}
#endif
			uint8_t idx = (state_old<<2 | state);
			
			*pEncState = pgm_read_byte(&encStateTable[idx]);
			
			// for error correction
			if(*pEncState == RENC_ERR){
				*pEncState = lastDirection;
			}
			if(*pEncState){
				lastDirection = *pEncState;
			}
			
			state_old = state;
		//}
		sw_old = sw;
	}
	
	
	return ;
}
#endif
