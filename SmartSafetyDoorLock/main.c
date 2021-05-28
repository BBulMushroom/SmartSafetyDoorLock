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
#define DEBUGMODE 0	// 0:normal, 1:debug
#define INITIAL_VALUE 0	// 0: none, 1:store null data, 2:set initial values(rfid)
/////////////////////////////////////////////////////////////////////////////////////
// adc threshold
#define FIRE_THRESHOLD 250	//normal 192, active 500
#define TILT_THRESHOLD 700		//normal 992, active 574

/////////////////////////////////////////////////////////////////////////////////////
// set ports
#define OPEN_LED_PORT PORT(C, 0)	//door open display led
#define OPEN_BUTTON_PORT PORT(C, 1)	// door open button
#define PRESSURE_BUTTON_PORT PORT(C, 2)	// door open confirm button
#define SENIOR_SWITCH_PORT PORT(C, 3)	// for the elderly help switch

#define LIGHT1_PORT PORT(G, 0)	// lights
#define LIGHT2_PORT PORT(G, 1)
#define LIGHT3_PORT PORT(G, 2)
#define LIGHT_BUTTON_PORT PORT(G, 3)	// lights all off button

#define BUZZER_PORT PORT(B, 4)	// buzzer timer0 OC0
#define SERVO_PORT PORT(B, 5)	// servo motor timer1 OC1A
// timer2 : 다용도 시간 측정용

#define FIRE_PORT PORT(F, 0)	// fire detection sensor adc1
#define TILT_PORT PORT(F, 1)	// tilt sensor adc2
#define FIRE 0
#define TILT 1

// usart0 (Rx:PE0, Tx:PE1) : Bluetooth
// spi : RFID

#define AUTO_CLOSE_TIME 5000	// doorlock 몇초동안 열려있을 시 경고메세지 보내는가에 대한 시간
#define DOOR_OPEN_TIME 3000	// 버튼 눌렀을 때 열려있을 시간
#define SHOCK_COUNT_TIME 5000	// shock count 초기화 시간
#define BUTTON_RFIDMODE_TIME 5000	//rfid읽기모드 진입 시간
#define SENIOR_HELP_TIME 5000	// 문이 열리지 않는 기간 카운트
/////////////////////////////////////////////////////////////////////////////////////
// variables
unsigned char chRxTemp0;	// usart0 store buffer

volatile uint16_t shockCount = 0;	// 충격 감지 카운트
volatile uint16_t tmpShockCount = 0;	// 충격 감지 카운트 비교용

uint8_t currentRfidData[4];	//현재 입력된 rfid값
uint8_t lastRfidData[4];	//중복출력 방지용 이전 rfid값
volatile uint16_t rfidFindCount = 0;	//중복출력 방지용 rfid 수신 카운트

// for timer2 count
volatile unsigned int autoCloseWaitTime = 0;
volatile unsigned int seniorHelpTime = 0;
volatile unsigned int lockedTime = 0;
volatile unsigned int shockCountTime = 0;
volatile unsigned int buttonRfidTime = 0;

struct RFID_info
{
	int RFID_index;	//인덱스(0~4)
	char RFID_name[11];	//사용자 이름 최대 10자 : 10바이트
	uint8_t RFID_value[4];	//태그값 4바이트
};
struct RFID_info RFID_data[5];	//5개까지 저장
char emptyName[11] = {0,0,0,0,0,0,0,0,0,0};
uint8_t emptyData[4] = {0,0,0,0};
uint8_t data1[4] = {0xc9, 0x19, 0x29, 0x8c};
uint8_t data2[4] = {0x4c, 0x95, 0x52, 0x18};
/////////////////////////////////////////////////////////////////////////////////////
// flags
bool flag_autoClose = true;	// 자동 잠김 플래그
bool flag_isBuzzerRunning = false;	// 부저 실행여부 플래그
bool flag_isButtonPressed = false;	// open button 눌림 플래그
bool flag_isRFIDReceived = false;	// RFID 수신 플래그
/////////////////////////////////////////////////////////////////////////////////////
// 함수 선언
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
void urgentButton()	//112, 119, no 버튼
{
	UCSR0B &= ~(1 << RXCIE0);	// 수신완료 인터럽트 해제
	char tmpChar = USART0_Rx();
	switch (tmpChar)
	{
		case '1':	// 112 버튼 입력
		case '2':	// 119 버튼 입력
		flag_autoClose = false;	// 문 저절로 안잠기게
		buzzerOn();
		break;
		case '3':	// No 버튼 입력
		flag_autoClose = true;
		buzzerOff();
		break;
		default:	// 예외처리
		break;
	}
	UCSR0B |= (1 << RXCIE0);	// 수신완료 인터럽트 설정
}

ISR(USART0_RX_vect)
{
	UCSR0B &= ~(1 << RXCIE0);	// 수신완료 인터럽트 해제
	chRxTemp0 = UDR0;
	char tmpName[11];
	uint8_t tmpData[4];
	switch(chRxTemp0)
	{
		case 'a':	// 저장된 데이터 모두 전송
		for(int i=0; i<5; i++)
		{
			eeprom_read_data(i, tmpName, tmpData);	// 데이터 읽기
			sendRfidInfo(i);
		}
		break;
		case '0':	// 저장된 데이터 삭제 후 데이터 모두 전송
		case '1':
		case '2':
		case '3':
		case '4':
		eeprom_delete_data_index(chRxTemp0-48);	// 받은 데이터를 인덱스로 해서 삭제
		writeRfid(chRxTemp0-48);	// 변수 업데이트
		for(int i=0; i<5; i++)	// 데이터 전송
		{
			eeprom_read_data(i, tmpName, tmpData);	// 데이터 읽기
			sendRfidInfo(i);
		}
		break;
		case 'x':
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
	UCSR0B |= (1 << RXCIE0);	// 수신완료 인터럽트 설정
}
/////////////////////////////////////////////////////////////////////////////////////
// ADC
void checkADC(char* string, int num)	// 디버깅용 adc값 출력 함수
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
	TCCR0 = 0b01101101;	// buzzer 켜기
	flag_isBuzzerRunning = true;
}
void buzzerOff()
{
	TCCR0 = 0b01011101;	// buzzer 끄기
	flag_isBuzzerRunning = false;
}
/////////////////////////////////////////////////////////////////////////////////////
// Door
bool doorCheck()	// 문이 열려있는지 check. 닫혀있으면 true, 열려있으면 false
{
	if(IS_PORT_SET(PRESSURE_BUTTON_PORT))	// 문이 열려 있음
	{
		return false;
	}
	else // 문이 닫혀 있음
	{
		return true;
	}
}
void init_Servo()	 //서보 모터 초기화
{
	PORT_DIR_OUT(SERVO_PORT);
	TCCR1A |= (1<<WGM11);
	TCCR1B |= (1<<WGM12) | (1<<WGM13); //고속 PWM 모드, TOP : ICR1
	//TCCR1B |= (1<<CS11); //분주율 8, 2MHz
	ICR1 = 40000; //20ms주기
	TCCR1A |= (1<<COM1A1); //비반전 모드
}
void openDoor()	//door lock unlocked
{
	TCCR1B |= (1<<CS11); //켜기
	OCR1A = 2000;
	flag_autoClose = true;	// 자동 닫힘 설정
	autoCloseWaitTime = 0;	// 문 열림 시간 측정 타이머 초기화
	_delay_ms(300);
	TCCR1B &= ~(1<<CS11); //끄기
}
void closeDoor()	//door lock locked
{
	TCCR1B |= (1<<CS11); //켜기
	OCR1A = 4300;
	flag_autoClose = false;	// 자동 닫힘 해제
	_delay_ms(300);
	TCCR1B &= ~(1<<CS11); //끄기
}
/////////////////////////////////////////////////////////////////////////////////
// Timer2
ISR(TIMER2_OVF_vect)	//16ms마다 한번 실행
{
	TCNT2 = 6;
	if(doorCheck())	// 문이 닫혀있을 때
	{
		lockedTime = 0;	// 문열림 측정 시간 초기화
		/// 문 자동으로 잠그기
		if(flag_autoClose)	// 자동잠금 활성화 상태일 때
		{
			autoCloseWaitTime++;	// 기다리기
			if(autoCloseWaitTime >= DOOR_OPEN_TIME)	// 일정 시간 지나면 문 닫기
			{
				closeDoor();
			}
		}
		else // 자동잠금 비활성화 상태일 때(문이 폰으로 열렸을 때)
		{
			// 자동으로 잠그지도 않고 폰으로 메시지 보내지도 않음
		}
		if(IS_PORT_CLR(SENIOR_SWITCH_PORT))	// 노인용 스위치가 눌려있을 때
		{
			seniorHelpTime++;
			if(seniorHelpTime >= SENIOR_HELP_TIME)	// 일정 시간동안 문이 열리지 않으면
			{
				USART0_Tx_String("Call112");	// 바로 112에 연락
				buzzerOn();
				openDoor();	// 문열기
				flag_autoClose = false;	// 자동잠금 해제
				seniorHelpTime = 0;
			}
		}
		// 테스트에서는 5초마다 알림이 가지만 실제 상황은 카운트 시간이 몇일 단위이므로 알림이 계속 뜨지 않음
		else
		{
			seniorHelpTime = 0;
		}
	}
	else // 문이 열려있을 때
	{
		autoCloseWaitTime = 0;	// 잠금 시간 초기화
		seniorHelpTime = 0;
		/// 문 계속 열려있으면 폰으로 알림 보내기
		if(flag_autoClose)	// 자동잠금 활성화 상태일 때(일반적으로 문을 연 상황)
		{
			lockedTime++;
			if(lockedTime >= AUTO_CLOSE_TIME)	// 일정 시간 지나면
			{
				USART0_Tx_String("Please check the door");	// 메시지 전송
				lockedTime = 0;	// 제한시간 초기화
			}
		}
		else // 자동잠금 비활성화 상태일 때(문이 폰으로 열렸을 때)
		{
			// 자동으로 잠그지도 않고 폰으로 메시지 보내지도 않음
		}
	}
	
	// 충격 카운트 시간 초기화
	if(shockCount != tmpShockCount)	// shock count가 전과 다를 때
	{
		shockCountTime = 0;	// 처음부터 다시 카운트 시작
		tmpShockCount = shockCount;	// 현재 shock count 저장
	}
	if(shockCount)	// shock count가 0이 아닐 때
	{
		shockCountTime++;
		if(shockCountTime >= SHOCK_COUNT_TIME)	// 일정 시간 지나면 shock count 초기화
		{
			shockCount = 0;
			tmpShockCount = 0;
		}
	}
}
/////////////////////////////////////////////////////////////////////////////////
// RFID

// 블루투스 연결되면 atmega에 있는 데이터 폰으로 전송 (init_RFID에서)
// 항상 rfid 읽기
// atmega에서 데이터 저장 : 폰으로 name string 요청 -> atmega에서 name 받아 저장 -> 폰으로 모든 데이터 다시 전송
// 폰에서 삭제 누르면 요청 보냄


//폰                     atmega
// (폰에 데이터 저장)
// 데이터 요청
//						요청 수신(인터럽트)
//						데이터 송신
// 수신 후 저장
/////////////////////////////////////////////////////////////////////////////////
// (저장된 데이터 삭제)
// 데이터 삭제 요청
//                      요청 수신(인터럽트)
//                      데이터 삭제
//                      (폰에 데이터 저장)실행
//


void init_RFID()
{
	//초기값 지정
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
	
	for(int i=0; i<5; i++)	//eeprom값을 변수에 저장
	{
		writeRfid(i);
	}
}

void byteToString(uint8_t data[4], char str[50])	//rfid 데이터를 string으로 변환
{
	sprintf(str, "%x%x:%x%x:%x%x:%x%x"
	, data[0]/16, data[0]%16
	, data[1]/16, data[1]%16
	, data[2]/16, data[2]%16
	, data[3]/16, data[3]%16);
}
bool arrayEqual(uint8_t a[], uint8_t b[], uint8_t size)	//배열 같은지 확인하는 함수
{
	for (int i=0; i<size; i++)
	{
		if(b[i] != a[i])
		return false;
	}
	return true;
}
void readRfid(uint8_t data[4])	//RFID값 읽기
{
	uint8_t byte;
	uint8_t str[16];
	byte = mfrc522_request(PICC_REQALL, str);
	byte = mfrc522_get_card_serial(str);

	if(byte == CARD_FOUND)
	{
		memcpy(currentRfidData, str, 4);	//현재 데이터에 저장
		if(arrayEqual(lastRfidData, currentRfidData, 4))	//전의 데이터와 같으면(계속 대고 있으면)
		{
			flag_isRFIDReceived = false;
		}
		else
		{
			memcpy(data, str, 4);	//반환값에 저장
			flag_isRFIDReceived = true;
		}
		memcpy(lastRfidData, currentRfidData, 4);	//이전값으로 옮기기
		
		byte = mfrc522_request(PICC_REQALL, str);	// 쓰레기값 처리
		byte = mfrc522_get_card_serial(str);
	}
	else
	{
		for(int i=0; i<4; i++)	//카드가 없으면 이전값 초기화
		{
			lastRfidData[i] = 0;
		}
		flag_isRFIDReceived = false;
	}
}
void storeRfid(uint8_t data[4])	//RFID값 읽어서 저장
{
	int index = searchEmptyMemory();
	if(index < 5)	//5개까지 저장 가능
	{
		USART0_Tx_String("Please enter the name\n");	//이름 입력받음
		char tmpName[11];
		
		USART0_Rx_String(tmpName, 11);	//10자리 받기(null까지 11자리)
		eeprom_update_data(index, tmpName, data);	//정보 저장
		writeRfid(index);	// 변수에도 저장
		for(int i=0; i<5; i++)	// 폰으로 바뀐 데이터 포함 모두 전송
		{
			sendRfidInfo(i);
		}
		_delay_ms(200);
		USART0_Tx_String("\nregistration completed");
	}
	else
	{
		USART0_Tx_String("You can no longer register.\n");
	}
}
int searchEmptyMemory()	// 빈 공간 찾기
{
	for(int i=0; i<5; i++)
	{
		if (arrayEqual(RFID_data[i].RFID_value, emptyData, 4))	//메모리가 비어있으면
		{
			return i;
		}
	}
	return 5;	//비어있지 않으면 5 반환
}
void writeRfid(int index)	// eeprom값을 내부 변수에 저장
{
	RFID_data[index].RFID_index = index;
	eeprom_read_data(index, RFID_data[index].RFID_name, RFID_data[index].RFID_value);
}
int checkRfid(uint8_t data[4])	// 저장값에 있는지 확인
{
	for(int i=0; i<5; i++)	//입력값과 같은 기존 저장값이 있을 때
	{
		if (arrayEqual(RFID_data[i].RFID_value, data, 4))
		{
			return i;
		}
	}
	USART0_Tx_String("There are no matching cards.\n");
	return 5;
}
void sendRfidInfo(int index)	//저장된 rfid값 출력하기
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
void deleteRfid(int index)	//RFID 저장값 삭제
{
	RFID_data[index].RFID_index = 0;
	memset(RFID_data[index].RFID_name, 0, sizeof(RFID_data[index].RFID_name));
	memset(RFID_data[index].RFID_value, 0, sizeof(RFID_data[index].RFID_value));
}
//[0].. [9][10] [11][12][13][14] [15]..
//name1....'\0' data............ name2...
void eeprom_update_data(int index, char name[11], uint8_t data[4])	// 값 받아 데이터 저장
{
	for(int i=0; i<11; i++)	// name 저장
	{
		if(name[i] == 0)	// null문자가 아닐때까지 저장
		{
			eeprom_update_byte(index*15 + i, 0);
			break;
		}
		eeprom_update_byte(index*15 + i, name[i]);
	}
	for(int i=0; i<4; i++)	// data 저장
	{
		eeprom_update_byte(index*15 + 11 + i, data[i]);
	}
}
void eeprom_read_data(int index, char name[11], uint8_t data[4])	// index로 데이터 읽기
{
	for(int i=0; i<11; i++)	// name 읽기
	{
		if(eeprom_read_byte(index*15 + i) == 0)	// null이면 읽기 종료
		{
			name[i] = 0;
			break;
		}
		name[i] = eeprom_read_byte(index*15 + i);
	}
	for(int i=0; i<4; i++)	// data 읽기
	{
		data[i] = eeprom_read_byte(index*15 + 11 + i);
	}
}
bool eeprom_delete_data_index(int index)	// index로 데이터 지우기
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
	
	PORT_DIR_IN(FIRE_PORT);	// 온도센서
	PORT_DIR_IN(TILT_PORT);	// 기울기센서
	
	PORT_DIR_IN(PRESSURE_BUTTON_PORT);	// 문열림 확인 버튼
	PORT_SET(PRESSURE_BUTTON_PORT);	// 문열림 확인 버튼 풀업저항, high:열림, low:닫힘
	
	PORT_DIR_IN(OPEN_BUTTON_PORT);	// 문열림 버튼
	PORT_SET(OPEN_BUTTON_PORT);	// 풀업저항 설정, high:안눌림, low:눌림
	
	PORT_DIR_OUT(OPEN_LED_PORT);	// 문열림 확인 led
	
	PORT_DIR_IN(SENIOR_SWITCH_PORT);	// 노인용 도움 스위치
	PORT_SET(SENIOR_SWITCH_PORT);	// 풀업저항 설정
	
	PORT_DIR_OUT(LIGHT1_PORT);	// 전구
	PORT_DIR_OUT(LIGHT2_PORT);
	PORT_DIR_OUT(LIGHT3_PORT);
	PORT_DIR_IN(LIGHT_BUTTON_PORT);	// 전구 모두 끄기 버튼
	PORT_SET(LIGHT_BUTTON_PORT);	// 풀업저항
	
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
		if(flag_isBuzzerRunning)	// 부저가 울리고 있을 때는 버튼입력 최우선
		{
			if(IS_PORT_CLR(OPEN_BUTTON_PORT))	// 버튼 눌리면
			{
				buzzerOff();	// 부저 끄기
				flag_autoClose = true;
				_delay_ms(1000);
			}
			_delay_ms(1);
		}
		else if(doorCheck())	//  문이 닫혀 있는 동안 실행
		{
			readRfid(rfidData);	//rfid 읽기
			if(flag_isRFIDReceived)	//수신되었는지 확인
			{
				int tmpInt = checkRfid(rfidData);
				if(tmpInt < 5)	//현재 저장되어있는 값인지 확인
				{
					USART0_Tx_String("Welcome,");
					USART0_Tx_String(RFID_data[tmpInt].RFID_name);
					USART0_Tx('\n');
					openDoor();
				}
				else //저장 시도
				{
					storeRfid(rfidData);
				}
				flag_isRFIDReceived = false;
			}
			PORT_SET(OPEN_LED_PORT);	// led on
			if(!flag_autoClose && flag_isButtonPressed)	// 자동 잠금 해제 상태일 때(버튼 누르고 문을 열었다가 닫았을 때)
			{
				flag_isButtonPressed = false;	// 버튼 눌림 플래그 초기화
				_delay_ms(500);	// 잠시 기다렸다가
				closeDoor();	// 문닫기
			}
			if(IS_PORT_CLR(OPEN_BUTTON_PORT))	// 문이 닫혀있는데 open 버튼이 눌렸으면
			{
				openDoor();	// 문열기
				flag_isButtonPressed = true;	// 버튼 눌림 플래그 set
				flag_autoClose = true;
				_delay_ms(200);
			}
			if(readADC(TILT) < TILT_THRESHOLD)	// 충격 발생
			{
				shockCount++;
				if(shockCount > 2){
					USART0_Tx_String("112");	// 블루투스 112 전송
					urgentButton();	// 1byte 데이터 수신 대기
					shockCount = 0;	// 충격 카운트 초기화
				}
			}
			if(readADC(FIRE) > FIRE_THRESHOLD)	// 화재 발생
			{
				USART0_Tx_String("119");	// 블루투스 119 전송
				openDoor();	// 문열기
				urgentButton();	// 1byte 데이터 수신 대기
			}
		}
		else // 문이 열려 있는 동안 실행
		{
			PORT_CLR(OPEN_LED_PORT);	// led off
		}
	}
}
#endif