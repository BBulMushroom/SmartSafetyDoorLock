#define F_CPU 16000000UL
#define BAUD 9600

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <string.h>
#include <myatmega128.h>
#include <putil.h>
#include <mfrc522.h>
#include <spi.h>
#define DEBUGMODE 0	//0:normal, 1:debug
#define INITIAL_VALUE 0	//0: none, 1:store null data, 2:set initial values(rfid)
/////////////////////////////////////////////////////////////////////////////////////
// adc threshold
#define FIRE_THRESHOLD 250	//normal 192, active 500
#define TILT_THRESHOLD 700		//normal 992, active 574

/////////////////////////////////////////////////////////////////////////////////////
// set ports
#define OPEN_LED_PORT PORT(C, 0)	//door open display led
#define OPEN_BUTTON_PORT PORT(C, 1)	//door open button
#define PRESSURE_BUTTON_PORT PORT(C, 2)	//door open confirm button
#define SENIOR_SWITCH_PORT PORT(C, 3)	//for the elderly help switch

#define LIGHT1_PORT PORT(G, 0)	//lights
#define LIGHT2_PORT PORT(G, 1)
#define LIGHT3_PORT PORT(G, 2)
#define LIGHT_BUTTON_PORT PORT(G, 3)	//lights all off button

#define BUZZER_PORT PORT(B, 4)	//buzzer timer0 OC0
#define SERVO_PORT PORT(B, 5)	//servo motor timer1 OC1A
// timer2 : For 1 ms measurement

#define FIRE_PORT PORT(F, 0)	//fire detection sensor adc1
#define TILT_PORT PORT(F, 1)	//tilt sensor adc2
#define FIRE 0
#define TILT 1

// usart0 (Rx:PE0, Tx:PE1) : Bluetooth
// spi : RFID

#define AUTO_CLOSE_TIME 5000	//how long the door is closed until send a notification
#define DOOR_OPEN_TIME 3000	//open time when press the button
#define SHOCK_COUNT_TIME 5000	//shock count reset time
#define SENIOR_HELP_TIME 5000	//how long the door is closed until send a notification(for senior)
/////////////////////////////////////////////////////////////////////////////////////
// variables
unsigned char chRxTemp0;	//usart0 store buffer

volatile uint16_t shockCount = 0;	//shock count
volatile uint16_t tmpShockCount = 0;	//shock count for comparison

uint8_t currentRfidData[4];	//Current input rfid value
uint8_t lastRfidData[4];	//Previous rfid value to prevent duplicate output

// for timer2 count
volatile unsigned int autoCloseWaitTime = 0;
volatile unsigned int seniorHelpTime = 0;
volatile unsigned int lockedTime = 0;
volatile unsigned int shockCountTime = 0;
volatile unsigned int buttonRfidTime = 0;

struct RFID_info
{
	int RFID_index;	//index(0~4)
	char RFID_name[11];	//user name max 10 : 10byte
	uint8_t RFID_value[4];	//tag value 4byte
};
struct RFID_info RFID_data[5];	//store up to 5
char emptyName[11] = {0,0,0,0,0,0,0,0,0,0};
uint8_t emptyData[4] = {0,0,0,0};
uint8_t data1[4] = {0xc9, 0x19, 0x29, 0x8c};
uint8_t data2[4] = {0x4c, 0x95, 0x52, 0x18};
/////////////////////////////////////////////////////////////////////////////////////
// flags
bool flag_autoClose = true;	//auto close flag
bool flag_isBuzzerRunning = false;	//buzzer on/off flag
bool flag_isButtonPressed = false;	//open button flag
bool flag_isRFIDReceived = false;	//RFID receive flag
/////////////////////////////////////////////////////////////////////////////////////
//Function declaration
void urgentButton();
void checkADC(char* string, int num);
void init_Buzzer();
void buzzerOn();
void buzzerOff();
bool doorCheck();
void init_Servo();
void openDoor();
void closeDoor();
void init_RFID();
void byteToString(uint8_t data[4], char str[50]);
bool arrayEqual(uint8_t a[], uint8_t b[], uint8_t size);
void readRfid(uint8_t data[4]);
void storeRfid(uint8_t data[4]);
int searchEmptyMemory();
void writeRfid(int index);
int checkRfid(uint8_t data[4]);
void sendRfidInfo(int index);
void deleteRfid(int index);
void eeprom_update_data(int index, char name[11], uint8_t data[4]);
void eeprom_read_data(int index, char name[11], uint8_t data[4]);
bool eeprom_delete_data_index(int index);
/////////////////////////////////////////////////////////////////////////////////////
// USART
//
void urgentButton()	//112, 119, no button
{
	UCSR0B &= ~(1 << RXCIE0);	//unset receiving completion interrupt
	char tmpChar = USART0_Rx();
	switch (tmpChar)
	{
		case '1':	//112 button received
		case '2':	//119 button received
		flag_autoClose = false;	//the door won't close by itself
		buzzerOn();
		break;
		case '3':	//no button received
		flag_autoClose = true;
		buzzerOff();
		break;
		default:
		break;
	}
	UCSR0B |= (1 << RXCIE0);	//set receive complete interrupt
}

ISR(USART0_RX_vect)
{
	UCSR0B &= ~(1 << RXCIE0);	//unset receive complete interrupt
	chRxTemp0 = UDR0;
	char tmpName[11];
	uint8_t tmpData[4];
	switch(chRxTemp0)
	{
		case 'a':	//Send all stored data
		for(int i=0; i<5; i++)
		{
			eeprom_read_data(i, tmpName, tmpData);	//read data from eeprom
			sendRfidInfo(i);	//send
		}
		break;
		case '0':	//Send all data after deleting saved data
		case '1':
		case '2':
		case '3':
		case '4':
		eeprom_delete_data_index(chRxTemp0-48);	//Delete data by received index
		writeRfid(chRxTemp0-48);	//local variable update
		for(int i=0; i<5; i++)	//transmit all data to phone
		{
			eeprom_read_data(i, tmpName, tmpData);	//read data from eeprom
			sendRfidInfo(i);	//send
		}
		break;
		case 'x':	//lights on
		PORT_SET(LIGHT1_PORT);
		break;
		case 'y':
		PORT_SET(LIGHT2_PORT);
		break;
		case 'z':
		PORT_SET(LIGHT3_PORT);
		break;
		default:
		break;
	}
	UCSR0B |= (1 << RXCIE0);	//set receive complete interrupt
}
/////////////////////////////////////////////////////////////////////////////////////
// ADC
void checkADC(char* string, int num)	//print ADC value for debugging
{
	char str[5] = "";
	char tmp[50] = "";
	sprintf(tmp, "%s : %s", string, itoa(readADC(num),str,10));
	USART0_Tx_String(tmp);
	USART0_Tx('\r');
	USART0_Tx('\n');
}
////////////////////////////////////////////////////////////////////////////////////////
// Buzzer
void init_Buzzer()
{
	PORT_DIR_OUT(BUZZER_PORT);
	TCCR0 = 0b01011101; // WGM01:0 = 3, COM01:0 = 2, PRESCALE = 1024
	TCNT0 = 0x00;
	OCR0 = 0xf0;
}
void buzzerOn()
{
	TCCR0 = 0b01101101;	// buzzer on
	flag_isBuzzerRunning = true;
}
void buzzerOff()
{
	TCCR0 = 0b01011101;	// buzzer off
	flag_isBuzzerRunning = false;
}
/////////////////////////////////////////////////////////////////////////////////////
// Door
bool doorCheck()	//Check if the door is open. true if closed, false if open
{
	if(IS_PORT_SET(PRESSURE_BUTTON_PORT))	//door is open
	{
		return false;
	}
	else // 문이 닫혀 있음
	{
		return true;
	}
}
void init_Servo()	 //Servo motor initialization
{
	PORT_DIR_OUT(SERVO_PORT);
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13); //fast PWM, TOP: ICR1
	//TCCR1B |= (1<<CS11); //Prescaler 8, 2MHz
	ICR1 = 40000; //20msCycle
	TCCR1A |= (1<<COM1A1); //non-inverting mode
}
void openDoor()	//door lock unlocked
{
	TCCR1B |= (1<<CS11); //motor on
	OCR1A = 2000;
	flag_autoClose = true;	//set auto close
	autoCloseWaitTime = 0;	//Reset timer : measure door open time
	_delay_ms(300);
	TCCR1B &= ~(1<<CS11); //motor off
}
void closeDoor()	//door lock locked
{
	TCCR1B |= (1<<CS11); //motor on
	OCR1A = 4300;
	flag_autoClose = false;	//unset auto close
	_delay_ms(300);
	TCCR1B &= ~(1<<CS11); //motor off
}
/////////////////////////////////////////////////////////////////////////////////
// Timer2
ISR(TIMER2_OVF_vect)	//Run once every 16ms
{
	TCNT2 = 6;
	if(doorCheck())	//when the door is closed
	{
		lockedTime = 0;	//Reset door open measurement time
		///lock the door automatically
		if(flag_autoClose)	//When auto-lock is enabled
		{
			autoCloseWaitTime++;	//waiting
			if(autoCloseWaitTime >= DOOR_OPEN_TIME)	//Close the door after few seconds
			{
				closeDoor();
			}
		}
		else //When auto close is disabled (when the door is opened with a phone)
		{
			//It doesn't lock automatically and doesn't send messages to your phone
		}
		if(IS_PORT_CLR(SENIOR_SWITCH_PORT))	//When the switch for the elderly is pressed
		{
			seniorHelpTime++;
			if(seniorHelpTime >= SENIOR_HELP_TIME)	//If the door is not opened for a certain period of time
			{
				USART0_Tx_String("Call112");	//Call 112
				buzzerOn();	//buzzer on
				openDoor();	//open door
				flag_autoClose = false;	//unset auto close
				seniorHelpTime = 0;
			}
		}
		//In the test, the notification fires every 5 seconds, 
		//but in the real situation, the count time is several days, so the notification doesn't pop up rapidly
		else
		{
			seniorHelpTime = 0;
		}
	}
	else //when the door is opend
	{
		autoCloseWaitTime = 0;	//Reset auto close time
		seniorHelpTime = 0;
		///Send a notification to the phone if the door is still open
		if(flag_autoClose)	//When auto close is activated (normally when the door is opened)
		{
			lockedTime++;
			if(lockedTime >= AUTO_CLOSE_TIME)	//after few seconds
			{
				USART0_Tx_String("Please check the door");	//send message
				lockedTime = 0;	//reset locked time
			}
		}
		else //When auto close is disabled (when the door is opened with a phone)
		{
			//It doesn't lock automatically and doesn't send messages to your phone
		}
	}
	
	//Reset Shock Count Time
	if(shockCount != tmpShockCount)	//When the shock count is different from before
	{
		shockCountTime = 0;	//start counting again from the beginning
		tmpShockCount = shockCount;	//Save current shock count
	}
	if(shockCount)	//When shock count is not 0
	{
		shockCountTime++;
		if(shockCountTime >= SHOCK_COUNT_TIME)	//After few seconds, reset shock count
		{
			shockCount = 0;
			tmpShockCount = 0;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////
// RFID

void init_RFID()
{
	//initial value for debugging
	#if INITIAL_VALUE == 1
	eeprom_update_data(0, "PJB1", data1);
	eeprom_update_data(1, "PJB2", data2);
	eeprom_update_data(2, emptyName, emptyData);
	eeprom_update_data(3, emptyName, emptyData);
	eeprom_update_data(4, emptyName, emptyData);
	#else if INITIAL_VALUE == 2
	eeprom_update_data(0, emptyName, emptyData);
	eeprom_update_data(1, emptyName, emptyData);
	eeprom_update_data(2, emptyName, emptyData);
	eeprom_update_data(3, emptyName, emptyData);
	eeprom_update_data(4, emptyName, emptyData);
	#endif
	
	for(int i=0; i<5; i++)	//Save the eeprom value to local variable
	{
		writeRfid(i);
	}
}

void byteToString(uint8_t data[4], char str[50])	//Convert rfid data to string
{
	sprintf(str, "%x%x:%x%x:%x%x:%x%x"
	, data[0]/16, data[0]%16
	, data[1]/16, data[1]%16
	, data[2]/16, data[2]%16
	, data[3]/16, data[3]%16);
}
bool arrayEqual(uint8_t a[], uint8_t b[], uint8_t size)	//check if an array is equal
{
	for (int i=0; i<size; i++)
	{
		if(b[i] != a[i])
		return false;
	}
	return true;
}
void readRfid(uint8_t data[4])	//read RFID
{
	uint8_t byte;
	uint8_t str[16];
	byte = mfrc522_request(PICC_REQALL, str);
	byte = mfrc522_get_card_serial(str);

	if(byte == CARD_FOUND)
	{
		memcpy(currentRfidData, str, 4);	//save to current data
		if(arrayEqual(lastRfidData, currentRfidData, 4))	//If it is the same as the previous data (if you keep holding it)
		{
			flag_isRFIDReceived = false;	//it doesn't receive again
		}
		else
		{
			memcpy(data, str, 4);	//store in return value
			flag_isRFIDReceived = true;	//it received
		}
		memcpy(lastRfidData, currentRfidData, 4);	//move to previous value
		
		byte = mfrc522_request(PICC_REQALL, str);	//garbage value disposal
		byte = mfrc522_get_card_serial(str);
	}
	else
	{
		for(int i=0; i<4; i++)	//If there is no card, the previous value is reset.
		{
			lastRfidData[i] = 0;
		}
		flag_isRFIDReceived = false;
	}
}
void storeRfid(uint8_t data[4])	//Read and save RFID value
{
	int index = searchEmptyMemory();
	if(index < 5)	//Can store up to 5
	{
		USART0_Tx_String("Please enter the name\n");	//name request
		char tmpName[11];
		
		USART0_Rx_String(tmpName, 11);	//Get 10 digits (11 digits include null)
		eeprom_update_data(index, tmpName, data);	//store data in the eeprom
		writeRfid(index);	//also store in local variable
		for(int i=0; i<5; i++)	//Send all data including data changed to the phone
		{
			sendRfidInfo(i);
		}
		_delay_ms(200);
		USART0_Tx_String("\nregistration completed");
	}
	else //already 5 data stored, error
	{
		USART0_Tx_String("You can no longer register.\n");
	}
}
int searchEmptyMemory()	//find empty space
{
	for(int i=0; i<5; i++)
	{
		if (arrayEqual(RFID_data[i].RFID_value, emptyData, 4))	//If there is free space in memory
		{
			return i;	//return index
		}
	}
	return 5;	//Return 5 if not empty
}
void writeRfid(int index)	//Save the eeprom value to local variable
{
	RFID_data[index].RFID_index = index;
	eeprom_read_data(index, RFID_data[index].RFID_name, RFID_data[index].RFID_value);
}
int checkRfid(uint8_t data[4])	//check if it's in the data
{
	for(int i=0; i<5; i++)	//if exist,
	{
		if (arrayEqual(RFID_data[i].RFID_value, data, 4))	//return index after checking for empty space
		{
			return i;
		}
	}
	USART0_Tx_String("There are no matching cards.\n");	//else, return error
	return 5;
}
void sendRfidInfo(int index)	//Print the rfid value in local variable
{
	char tmpStr[100];
	sprintf(tmpStr, "%d,%s,%x%x%x%x%x%x%x%x,"
	, RFID_data[index].RFID_index, RFID_data[index].RFID_name
	, RFID_data[index].RFID_value[0]/16, RFID_data[index].RFID_value[0]%16
	, RFID_data[index].RFID_value[1]/16, RFID_data[index].RFID_value[1]%16
	, RFID_data[index].RFID_value[2]/16, RFID_data[index].RFID_value[2]%16
	, RFID_data[index].RFID_value[3]/16, RFID_data[index].RFID_value[3]%16);
	USART0_Tx_String(tmpStr);
}
void deleteRfid(int index)	//Delete rfid data in local variable
{
	RFID_data[index].RFID_index = 0;
	memset(RFID_data[index].RFID_name, 0, sizeof(RFID_data[index].RFID_name));
	memset(RFID_data[index].RFID_value, 0, sizeof(RFID_data[index].RFID_value));
}
//[0].. [9][10] [11][12][13][14] [15]..
//name1....'\0' data............ name2...
void eeprom_update_data(int index, char name[11], uint8_t data[4])	//store data in eeprom
{
	for(int i=0; i<11; i++)	//store name
	{
		if(name[i] == 0)	//until is not null
		{
			eeprom_update_byte(index*15 + i, 0);
			break;
		}
		eeprom_update_byte(index*15 + i, name[i]);
	}
	for(int i=0; i<4; i++)	//store data
	{
		eeprom_update_byte(index*15 + 11 + i, data[i]);
	}
}
void eeprom_read_data(int index, char name[11], uint8_t data[4])	//read data by index
{
	for(int i=0; i<11; i++)	//read name
	{
		if(eeprom_read_byte(index*15 + i) == 0)	//if null, end
		{
			name[i] = 0;
			break;
		}
		name[i] = eeprom_read_byte(index*15 + i);
	}
	for(int i=0; i<4; i++)	//read data
	{
		data[i] = eeprom_read_byte(index*15 + 11 + i);
	}
}
bool eeprom_delete_data_index(int index)	//delete data by index
{
	if(index < 5)
	{
		eeprom_update_data(index, emptyName, emptyData);
		return true;
	}
	else return false;
}
/////////////////////////////////////////////////////////////////////////////////

#if DEBUGMODE == 1
int main(void)
{
	init_Servo();
	init_Buzzer();
	init_Timer2();
	init_ADC();
	init_SPI();
	init_MFRC522();
	init_USART0();
	//init_RFID();

	while(1)
	{
		eeprom_update_data(0, "PJB1", data1);
		writeRfid(0);
		sendRfidInfo(0);
	}
}
#else
int main(void)
{
	// Initialize
	init_Servo();
	init_Buzzer();
	init_Timer2();
	init_ADC();
	init_SPI();
	init_MFRC522();
	init_USART0();
	_delay_ms(1000);
	
	PORT_DIR_IN(FIRE_PORT);	//fire detection sensor
	PORT_DIR_IN(TILT_PORT);	//shock detection sensor
	
	PORT_DIR_IN(PRESSURE_BUTTON_PORT);	//Door open confirmation button
	PORT_SET(PRESSURE_BUTTON_PORT);	//pull-up resistor setting, high: open, low: close
	
	PORT_DIR_IN(OPEN_BUTTON_PORT);	//door open button
	PORT_SET(OPEN_BUTTON_PORT);	//pull-up resistor setting, high: not pressed, low: pressed
	
	PORT_DIR_OUT(OPEN_LED_PORT);	//door open check led
	
	PORT_DIR_IN(SENIOR_SWITCH_PORT);	//help switch for the elderly
	PORT_SET(SENIOR_SWITCH_PORT);	//Pull-up resistor setting
	
	PORT_DIR_OUT(LIGHT1_PORT);	//lights
	PORT_DIR_OUT(LIGHT2_PORT);
	PORT_DIR_OUT(LIGHT3_PORT);
	PORT_DIR_IN(LIGHT_BUTTON_PORT);	//all light off button
	PORT_SET(LIGHT_BUTTON_PORT);	//Pull-up resistor setting
	
	_delay_ms(100);
	sei();
	init_RFID();
	_delay_ms(100);
	
	uint8_t rfidData[4];
	
	while(1)
	{
		if(IS_PORT_CLR(LIGHT_BUTTON_PORT))
		{
			PORT_CLR(LIGHT1_PORT);
			PORT_CLR(LIGHT2_PORT);
			PORT_CLR(LIGHT3_PORT);
		}
		if(flag_isBuzzerRunning)	//When the buzzer is ringing, button input has the highest priority
		{
			if(IS_PORT_CLR(OPEN_BUTTON_PORT))	//if Button pressd
			{
				buzzerOff();	//buzzer off
				flag_autoClose = true;
				_delay_ms(1000);
			}
			_delay_ms(1);
		}
		else if(doorCheck())	//run while the door is closed
		{
			readRfid(rfidData);	//read rfid
			if(flag_isRFIDReceived)	//If rfid is tagged
			{
				int tmpInt = checkRfid(rfidData);	//Check if the value is currently saved in data
				if(tmpInt < 5)	//if in data
				{
					USART0_Tx_String("Welcome,");
					USART0_Tx_String(RFID_data[tmpInt].RFID_name);
					USART0_Tx('\n');
					openDoor();
				}
				else //if not in data store value
				{
					storeRfid(rfidData);
				}
				flag_isRFIDReceived = false;	//receive flag reset
			}
			PORT_SET(OPEN_LED_PORT);	//led on
			if(!flag_autoClose && flag_isButtonPressed)	//When in automatic unlocking state (when the door is opened and closed by pressing a button)
			{
				flag_isButtonPressed = false;	//Reset button pressed flag
				_delay_ms(500);	//wait a while
				closeDoor();	//close the door
			}
			if(IS_PORT_CLR(OPEN_BUTTON_PORT))	//If the door is closed and the open button is pressed
			{
				openDoor();	//open the door
				flag_isButtonPressed = true;	//button pressed flag set
				flag_autoClose = true;
				_delay_ms(200);
			}
			if(readADC(TILT) < TILT_THRESHOLD)	//is shocked
			{
				shockCount++;
				if(shockCount > 2){
					USART0_Tx_String("112");	//Send 112 to phone
					urgentButton();	//Waiting for 1 byte data reception
					shockCount = 0;	//Reset Shock Count
				}
			}
			if(readADC(FIRE) > FIRE_THRESHOLD)	//is fire
			{
				USART0_Tx_String("119");	//Send 119 to phone
				openDoor();	//open the door
				urgentButton();	//Waiting for 1 byte data reception
			}
		}
		else //run while the door is open
		{
			PORT_CLR(OPEN_LED_PORT);	// led off
		}
	}
}
#endif