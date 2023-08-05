// AQM1248 128x64 LCD Driver
// (c) 2022 Eleken

#pragma once
#include <stdint.h>


// Detected Power Capability of Source
#define CC_STATUS_PPS 0x200 // Augmented PDO
#define CC_STATUS_VARIABLE 0x100
#define CC_STATUS_20V 0x80
#define CC_STATUS_15V 0x40
#define CC_STATUS_12V 0x20
#define CC_STATUS_9V 0x10
#define CC_STATUS_5V3A 0x08
#define CC_STATUS_5V1A 0x04
// CC Detect Status
#define CC_STATUS_CC2_DETECT 0x02
#define CC_STATUS_CC1_DETECT 0x01

// Declare (SPI Function)
void AQM1248_WriteData(const uint8_t* buf, uint16_t length);
void AQM1248_WriteCommand(uint8_t cmd);
//void AQM1248_Delay_2ms();

// Init
void AQM1248_Init();

// 0-63
void AQM1248_SetContrast(uint8_t newContrast);
// x: row(0-127)
// y: column(0-5)
void AQM1248_SetPos(uint8_t x, uint8_t y);

void AQM1248_Clear();
void AQM1248_ClearLine(uint8_t y);
void AQM1248_Draw8x8(const uint8_t* bmp);
void AQM1248_DrawText(const char* text, uint8_t x, uint8_t y);

void AQM1248_ShowNumber(uint16_t value, uint8_t digits, uint8_t x, uint8_t y, uint8_t dpPos);
void AQM1248_ShowCharacterIcon(char* c, uint8_t length, uint8_t x, uint8_t y);
void AQM1248_ShowPDVersion(uint8_t ver);

// PD Power project specific
struct sPDDisplayInfo
{
	uint16_t curVoltage; // mV
	uint16_t curCurrent; // mA

	uint16_t rqtVoltage; // mV
	uint16_t rqtCurrent; // mA
	uint16_t ppsRange[2]; // mV
	//
	uint8_t pdVersion; // 0-3
	uint8_t pdStatus;  // b0: CC1, b1: CC2
	// Display State
	uint8_t displayState;
	
	// CC Status (same to g_ccStatus)
	uint16_t ccStatus;
	
	// Source Capability
	USBPD_PDO_TypeDef obj[8];
};

void AQM1248_UpdateDisplay(const struct sPDDisplayInfo* info);
