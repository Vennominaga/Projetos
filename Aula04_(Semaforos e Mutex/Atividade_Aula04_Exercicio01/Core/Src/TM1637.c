


#include "main.h"
#include "TM1637.h"


#define CLK_HIGH() HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
#define CLK_LOW()  HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
#define DATA_HIGH() HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_SET);
#define DATA_LOW()  HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, GPIO_PIN_RESET);

/*Variaveis Globais*/
uint8_t display_brightness;



// Tabela de mapeamento estático (dicionário)
static const CharMap char_map[] = {
    {'0', 0b00111111}, // 0
    {'1', 0b00000110}, // 1
    {'2', 0b01011011}, // 2
    {'3', 0b01001111}, // 3
    {'4', 0b01100110}, // 4
    {'5', 0b01101101}, // 5
    {'6', 0b01111101}, // 6
    {'7', 0b00000111}, // 7
    {'8', 0b01111111}, // 8
    {'9', 0b01101111}, // 9
    {'A', 0b01110111}, // A
    {'B', 0b01111100}, // b
    {'C', 0b00111001}, // C
    {'D', 0b01011110}, // d
    {'E', 0b01111001}, // E
    {'F', 0b01110001}, // F
    {'G', 0b00111101}, // G
    {'H', 0b01110110}, // H
    {'I', 0b00000110}, // I
    {'J', 0b00011110}, // J
    {'L', 0b00111000}, // L
    {'N', 0b01010100}, // n
    {'O', 0b00111111}, // O
    {'P', 0b01110011}, // P
    {'S', 0b01101101}, // S (igual a 5)
    {'U', 0b00111110}, // U
    {'Y', 0b01101110}, // y
    {' ', 0b00000000}, // Espaço
    {'-', 0b01000000}, // Traço
    {'.', 0b10000000}, // Ponto (somente para o segmento DP)
    {'_', 0b00001000}  // Underscore
};

#define CHAR_MAP_SIZE (sizeof(char_map)/sizeof(CharMap))


void Delay_us (int time)
{
	for (int i=0; i<time; i++)
	{
		for (int j=0; j<7; j++)
		{
			__asm__("nop");
		}
	}
}

void start (void)
{

	CLK_HIGH();
	DATA_HIGH();
	Delay_us (2);
	DATA_LOW();
}

void stop (void)
{
	CLK_LOW();
	Delay_us (2);
	DATA_LOW();
	Delay_us (2);
	CLK_HIGH();
	Delay_us (2);
	DATA_HIGH();
}

void waitforAck (void)
{
	CLK_LOW();
	Delay_us (5); // After the falling edge of the eighth clock delay 5us
	              // ACK signals the beginning of judgment
//	while (dio);  // Check the state of the Data pin
	CLK_HIGH();
	Delay_us (2);
	CLK_LOW();
}







void TM1637_SetBrightness(uint8_t brightness)
{
    display_brightness = brightness;

}

void writeByte (uint8_t byte)
{
	int i;
	for (i = 0; i<8; i++)
	{
		CLK_LOW();
		if (byte & 0x01) // low front
		{
			DATA_HIGH();
		}
		else
		{
			DATA_LOW();
		}
		Delay_us (3);
		byte = byte >> 1;
		CLK_HIGH();
		Delay_us (3);
	}
}



static uint8_t find_segments_for_char(char character)
{
	for (uint8_t i = 0;i<CHAR_MAP_SIZE; i++)
	{
		if(char_map[i].character == character)
			return char_map[i].segments;


	}
	return 0x00;

}




void TM1637_WriteData (uint8_t Addr, uint8_t *data, int size)
{
	start();
	writeByte(0x40);
	waitforAck();
	stop();

	start();
	writeByte(Addr);
	waitforAck();
	for (int i=0; i<size; i++)
	{
		writeByte(data[i]);
		waitforAck();
	}
	stop();

	start();
	writeByte(TM1637_CMD_DISPLAY_CONTROL | display_brightness );
	waitforAck();
	stop();
}



void TM1637_Init(void)
{
	TM1637_SetBrightness(TM1637_BRIGHTNESS_DEFAULT);
	TM1637_Clear();

}

void TM1637_Clear(void)
{
	uint8_t data_clear[4] = {0x00,0x00,0x00,0x00};
	start();
	writeByte(TM1637_CMD_DISPLAY_CONTROL | display_brightness );
	waitforAck();
	stop();

	TM1637_WriteData(TM1637_CMD_SET_ADDRESS,data_clear,4);

	stop();

}

void TM1637_DisplaySegments(uint8_t addr, uint8_t segments)
{
	start();
	writeByte(TM1637_CMD_SET_DATA_FIX_ADDRESS);
	waitforAck();
    stop();

    start();
    writeByte(addr);
    waitforAck();
    writeByte(segments);
    waitforAck();
    stop();

    start();
    writeByte(TM1637_CMD_DISPLAY_CONTROL | display_brightness );
    waitforAck();
    stop();

}

void TM1637_DisplayChar(uint8_t position, char character, uint8_t dot)
{
	if(position > TM1637_ADDR_POS3) return;

	uint8_t segment = find_segments_for_char(character);

	if(dot) segment |= 0b10000000;

	TM1637_DisplaySegments(position, segment);


}

void TM1637_DisplayString(const char *str,uint8_t position)
{
	uint8_t dot = 0;

	for(uint8_t i = position;i<=TM1637_ADDR_POS3;i++,str++)
	{
		if(*str == '.') dot = 1;
		TM1637_DisplayChar(i,*str, dot);
		dot = 0;

	}


}

