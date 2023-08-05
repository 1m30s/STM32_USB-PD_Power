// AQM1248 128x64 LCD Driver
// (c) 2022 Eleken

// AQM1248: Graphic lcd with 65x132 driver ST7565R

#include "main.h"

#include "AQM1248_Driver.h"

#include "numeric.h"
#include "alpha.h"

extern SPI_HandleTypeDef hspi2;

static uint8_t GetFont(char c, uint8_t i)
{
	return g_font0[5* (c - 0x20) + i];
}

static void AQM1248_Delay_2ms()
{
	uint32_t st = HAL_GetTick();
	while((HAL_GetTick() - st) < 2);
}
void AQM1248_WriteData(const uint8_t* buf, uint16_t length)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET); // RS=H (Data)

	HAL_SPI_Transmit(&hspi2, (uint8_t*)buf, length, 100);
//	HAL_SPI_Transmit_IT(&hspi2, (uint8_t*)buf, length);

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS=1
}
void AQM1248_WriteCommand(uint8_t cmd)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS=0
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET); // RS=L (CMD)

	HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
//	HAL_SPI_Transmit_IT(&hspi2, &cmd, 1);

	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS=1
}

void AQM1248_Init()
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET); // CS=1
	HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET); // RS=H (Data)
	
	AQM1248_WriteCommand(0xAE);
	AQM1248_WriteCommand(0xA0);
	AQM1248_WriteCommand(0xC8);
	AQM1248_WriteCommand(0xA3);
	
	AQM1248_WriteCommand(0x2C);
	AQM1248_Delay_2ms();
	AQM1248_WriteCommand(0x2E);
	AQM1248_Delay_2ms();
	AQM1248_WriteCommand(0x2F);
	
	AQM1248_WriteCommand(0x23); // Contrast
	AQM1248_WriteCommand(0x81);
	AQM1248_WriteCommand(0x1C);
	
	AQM1248_WriteCommand(0xA4);
	AQM1248_WriteCommand(0x40);
	AQM1248_WriteCommand(0xA6);
	AQM1248_WriteCommand(0xAF);
}


// Contrast: 0-63
void AQM1248_SetContrast(uint8_t newContrast)
{
	AQM1248_WriteCommand(0x23);
	AQM1248_WriteCommand(0x81);
	AQM1248_WriteCommand(newContrast & 0x3F);
}
// x: row(0-127)
// y: column(0-5)
void AQM1248_SetPos(uint8_t x, uint8_t y)
{
	AQM1248_WriteCommand(0x10 | x>>4); // Column address
	AQM1248_WriteCommand(0x00 | (x&0x0F));
	AQM1248_WriteCommand(0xB0 | y); // Page address
}
void AQM1248_ClearLine(uint8_t y)
{
	uint8_t x;
	uint8_t d = 0;
	AQM1248_SetPos(0, y);
	for(x=0; x<128; x++){
		AQM1248_WriteData(&d, 1);
	}
}
void AQM1248_Clear()
{
	uint8_t y;
	for(y=0; y<6; y++){
		AQM1248_ClearLine(y);
	}
}


// Draw bitmap from current pos
// bmp[0]: the most left row. b0: top - b7: bottom
// bmp[7]: the most right row.
void AQM1248_Draw8x8(const uint8_t* bmp)
{
	AQM1248_WriteData(bmp, 8);
}
void AQM1248_DrawText(const char* text, uint8_t x, uint8_t y)
{
	AQM1248_SetPos(x*5, y);
	uint16_t i;
	for(i=0; i<128; i++)
	{
		if(text[i] == '\0') break;
		if(x >= 128/5) {
			x = 0;
			y++;
			AQM1248_SetPos(x*5, y);
		}
		AQM1248_WriteData(&g_font0[5 * (text[i]-0x20)], 5);
		x ++;
	}
}


// Show 12x24 pixel numbers
void AQM1248_ShowNumber(uint16_t value, uint8_t digits, uint8_t x, uint8_t y, uint8_t dpPos)
{
	uint8_t i,j,idx;
	uint8_t d[4];
	
	
	// 4 digits
	d[0] = (value / 1000)%10;
	d[1] = (value / 100)%10;
	d[2] = (value / 10)%10;
	d[3] = (value)%10;
	
	uint8_t buf[12];
	for(j=0; j<3; j++)
	{
		AQM1248_SetPos(x,y+j);
		for(idx=4-digits; idx<4; idx++)
		{
			if(d[idx] == 0 && idx+1<dpPos)
			{
				for(i=0; i<12; i++)
				{
					buf[i] = 0;
				}
			}
			else
			{
				for(i=0; i<12; i++)
				{
					buf[i] = g_font1[120*j + d[idx]*12 + i];
				}
			}
			AQM1248_WriteData(buf , 12);
			if(j==2 && dpPos == (idx+1)){
				buf[0] = buf[1] = 0xC0;
				AQM1248_WriteData(buf , 2);
			}else{
				buf[0] = buf[1] = 0x0;
				AQM1248_WriteData(buf , 2);
			}
		}
	}
}

// x: row(0-127), y:column(0-7)
void AQM1248_ShowCharacterIcon(char* c, uint8_t length, uint8_t x, uint8_t y)
{
	uint8_t i,idx;
	AQM1248_SetPos(x,y);
	uint8_t d[2];
	d[0] = 0x7E;
	AQM1248_WriteData( d, 1 );
	for(idx=0; idx<length; idx++){
		for(i=0; i<5; i++){
			d[0] = ~GetFont(c[idx], i);
			AQM1248_WriteData( d, 1);
		}
	}
	d[0] = 0x7E;
	d[1] = 0x00;
	AQM1248_WriteData( d, 2 );
}

void AQM1248_ShowPDVersion(uint8_t ver)
{
	uint8_t i;
	AQM1248_SetPos(0, 0);
	// 5x4 = 20 bytes
	uint8_t picbuf[20];
	
	
	if(ver <= 2)
	{
		for(i=0; i<5; i++){
			picbuf[i] = GetFont('P', i);
		}
		for(i=0; i<5; i++){
			picbuf[5+i] = GetFont('D', i);
		}
		for(i=0; i<5; i++){
			picbuf[10+i] = GetFont('1'+ver, i);
		}
		picbuf[13] &= ~0x80;
		picbuf[14] |= 0x80;
		for(i=0; i<5; i++){
			picbuf[15+i] = GetFont('0', i);
		}
	}
	else // No icon
	{
		for(i=0; i<20; i++){
			picbuf[i] = 0;
		}
	}
	AQM1248_WriteData( picbuf, 20 );
}

// PD Power project specific
/*
struct sPDDisplayInfo
{
	uint16_t curVoltage; // mV
	uint16_t curCurrent; // mA
	//
	uint8_t pdVersion; // 0-2
	uint8_t pdStatus;  // b0: CC1, b1: CC2
	
	// Source Capability
	uint32_t obj[8];
};*/
extern ADC_HandleTypeDef hadc1;
extern uint8_t g_cs;
void AQM1248_UpdateDisplay(const struct sPDDisplayInfo* info)
{
//	AQM1248_Clear();
	AQM1248_ClearLine(0);
	AQM1248_ShowPDVersion(info->pdVersion);
	
	uint8_t xoffs = 0, yoffs = 1;
	/*
	AQM1248_ShowCharacterIcon("PPS", 3, xoffs+0, yoffs);
	AQM1248_ShowCharacterIcon("PD", 2, xoffs+20, yoffs);
	AQM1248_ShowCharacterIcon("1", 1, xoffs+30, yoffs);
	AQM1248_ShowCharacterIcon("2", 1, xoffs+30+8*1, yoffs);
	AQM1248_ShowCharacterIcon("3", 1, xoffs+30+8*2, yoffs);
	AQM1248_ShowCharacterIcon("4", 1, xoffs+30+8*3, yoffs);
	AQM1248_ShowCharacterIcon("5", 1, xoffs+30+8*4, yoffs);
	*/
	
	xoffs = 24;
	yoffs = 0;
	/*
	if(info->ccStatus & CC_STATUS_CC1_DETECT)
	{
		AQM1248_ShowCharacterIcon("CC1", 3, xoffs+0, yoffs);
	}
	if(info->ccStatus & CC_STATUS_CC2_DETECT)
	{
		AQM1248_ShowCharacterIcon("CC2", 3, xoffs+0, yoffs);
		
	}
	xoffs += 20;
	*/

	if(info->ccStatus & CC_STATUS_5V1A || 
		info->ccStatus & CC_STATUS_5V3A)
	{
		AQM1248_ShowCharacterIcon("5V", 2, xoffs, yoffs);
		xoffs += 13;
	}
	if(info->ccStatus & CC_STATUS_9V)
	{
		AQM1248_ShowCharacterIcon("9V", 2, xoffs, yoffs);
		xoffs += 13;
	}
	if(info->ccStatus & CC_STATUS_12V)
	{
		AQM1248_ShowCharacterIcon("12V", 3, xoffs, yoffs);
		xoffs += 18;
	}
	if(info->ccStatus & CC_STATUS_15V)
	{
		AQM1248_ShowCharacterIcon("15V", 3, xoffs, yoffs);
		xoffs += 18;
	}
	if(info->ccStatus & CC_STATUS_20V)
	{
		AQM1248_ShowCharacterIcon("20V", 3, xoffs, yoffs);
		xoffs += 18;
	}
	if(info->ccStatus & CC_STATUS_PPS)
	{
		AQM1248_ShowCharacterIcon("PPS", 3, xoffs, yoffs);
		xoffs += 18;
	}
	

	AQM1248_ClearLine(1);
	char buf[32];
	// Current output power
	uint16_t power = (uint32_t)info->curVoltage * info->curCurrent / 1000; // mW
	if(info->displayState == 1)
	{
		// Show current power
		sprintf(buf, "Pout=%0d.%02dW",
				power/1000, (power/10)%100
				);
		AQM1248_DrawText(buf, 0, 1);
	}
	else if(IS_PPS_MODE())
	{
		// Show PPS Voltage range
		sprintf(buf, "PPS: %0d.%02dV-%0d.%02dV",
				info->ppsRange[0]/1000, (info->ppsRange[0]/10)%100,
				info->ppsRange[1]/1000, (info->ppsRange[1]/10)%100);
		AQM1248_DrawText(buf, 0, 1);
	}
	else
	{
//		sprintf(buf, "ADC: %d,%d, cl:%d", g_adcData[0], g_adcData[3], HAL_ADCEx_Calibration_GetValue(&hadc1));
		sprintf(buf, "Fixed PDO / Pout=%0d.%02dW",
				power/1000, (power/10)%100
				);
		AQM1248_DrawText(buf, 0, 1);
	}

	AQM1248_ClearLine(2);
	// Show Request Voltage
	sprintf(buf, "Request: %0d.%02dV %0d.%01dA[%d/%d]",
			info->rqtVoltage/1000, (info->rqtVoltage/10)%100,
			info->rqtCurrent/1000, (info->rqtCurrent/100)%10,
			g_curRequestPDOIndex+1, g_receivedFirstAPDOIdx);
	AQM1248_DrawText(buf, 0, 2);

	AQM1248_ClearLine(3);
	AQM1248_ClearLine(4);
	AQM1248_ClearLine(5);

	AQM1248_ShowNumber(info->curVoltage/10, 4, 0, 3, 2); // 3-5 lines. pix 0-(4x14).
	AQM1248_DrawText("V", 4*14/5, 5);
//	if(info->displayState == 1){
//		AQM1248_ShowNumber(info->curCurrent,    3, 5*14, 3, 3); // 3-5 lines. pix 0-(4x14).
//		AQM1248_DrawText("W", 8*14/5, 5); // ww.w W
//	}else{
		AQM1248_ShowNumber(info->curCurrent/10,    3, 5*14, 3, 2); // 3-5 lines. pix 0-(4x14).
		AQM1248_DrawText("A", 8*14/5, 5); // x.xx A
//	}
}
// If you want to use display ram in MCU it requires (128x48/8) 768 bytes on SRAM.

