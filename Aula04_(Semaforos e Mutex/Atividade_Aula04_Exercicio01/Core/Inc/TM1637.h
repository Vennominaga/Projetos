/*
 * TM1637.h
 *
 *  Created on: Jun 9, 2024
 *      Author: arunrawat
 *
 *
 *      Modificado: Vennominaga
 *      11 de Junho de 2025
 */

#ifndef INC_TM1637_H_
#define INC_TM1637_H_


// Comandos do TM1637
#define TM1637_CMD_SET_DATA_AUTOMATIC_ADDRESS		0x40
#define TM1637_CMD_SET_DATA_FIX_ADDRESS				0x44

#define TM1637_CMD_SET_ADDRESS      				0xC0
#define TM1637_CMD_DISPLAY_CONTROL 					0x80


//Posições
// Endereços fixos para cada posição do display
#define TM1637_ADDR_POS0 0xC0
#define TM1637_ADDR_POS1 0xC1
#define TM1637_ADDR_POS2 0xC2
#define TM1637_ADDR_POS3 0xC3

// Estrutura para mapeamento caractere -> segmentos
typedef struct
{
    char character;
    uint8_t segments;
} CharMap;


//Brightness
#define TM1637_BRIGHTNESS_MAX       0x0F
#define TM1637_BRIGHTNESS_DEFAULT   0x0A
#define TM1637_BRIGHTNESS_MIN       0x09

void TM1637_WriteData (uint8_t Addr, uint8_t *data, int size);
void TM1637_Init(void);
void TM1637_Clear(void);
void TM1637_SetBrightness(uint8_t brightness);
void TM1637_DisplaySegments(uint8_t position, uint8_t segments);
void TM1637_DisplayChar(uint8_t position,char character,uint8_t dot);
void TM1637_DisplayString(const char *str,uint8_t position);



#endif /* INC_TM1637_H_ */
