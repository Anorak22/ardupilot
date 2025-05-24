#include "AP_MyModule/AP_MyModule.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

uint8_t sign;
uint8_t textarray[] = {0xc0, 0xdf, 0xe0, 0xff, 0x20, 0x20, 0x41, 0x5A, 0x61, 0x7a};
char testString[] = {"AZaz  АЯая"};

extern const AP_HAL::HAL& hal;

MyModule *MyModule::_instance = nullptr;

MyModule::MyModule() :
    _last_run_ms(0),
    _initialized(false) {}

void MyModule::init()
{
    if (_initialized) {
        return;
    }

    setup_uart(hal.serial(4), "SERIAL4");

    _initialized = true;
    hal.console->printf("MyModule initialized\n");
}

uint32_t del = AP_HAL::millis();

void MyModule::update()
{
    if (!(_enabled && _initialized)) {
        return;
    }

    AP_HAL::UARTDriver *uart = hal.serial(4);
    AP_AHRS &ahrs = AP::ahrs();

    sendTextline(uart, textarray, 1);

    sendTextline(uart, textarray, 2);

    //Питч и рол
    float pitch, roll;
    AP::vehicle()->get_osd_roll_pitch_rad(pitch, roll);
    pitch = -degrees(pitch) + 180;
    roll = -degrees(roll) + 180;

    //Высота
    float alt;
    ahrs.get_relative_position_D_home(alt);  // Получаем высоту относительно домашней позиции (отрицательное значение)
    alt = -alt;

    //Статус режима полета
    uint8_t flight_mode = AP::vehicle()->get_mode();

    //Напряжение
    float V = AP::battery().voltage(0); // 0 - первая батарея

    sendTelemetry(uart, pitch, roll, alt, 100, 100, flight_mode, 10, V, 25, 1500);
}

void MyModule::setup_uart(AP_HAL::UARTDriver *uart, const char *name)
{
    if (uart == nullptr) {
        return;
    }

    uart->begin(100000);
}

uint8_t checksum(uint8_t *buf, uint8_t bufsize )
{
    uint8_t cksum = 0;

    for (uint8_t i = 2; i < BUFSIZE - 1; i++) {
        cksum += buf[i];
    }
    return cksum;
}

uint16_t sendTextline(AP_HAL::UARTDriver *uart, uint8_t *text, uint8_t line)
{
  uint8_t outputBuf [BUFSIZE];
  outputBuf[0] = 0x55;    // Шапка1
	outputBuf[1] = 0x55;    // Шапка2
  outputBuf[2] = 0x27;    // 39 символов
	outputBuf[3] = line;    // Номер команды, должен быть 1 или 2
 
  for (uint8_t i = 0; i < MAXTEXTLENGTH; i++) {
    outputBuf[i+4] = text[i];
  }
  // Контролька
  outputBuf[42] = checksum(outputBuf, outputBuf[3]);

  uart->write(outputBuf, outputBuf[2]+4);
  uart->flush();
  memset(outputBuf, 0, sizeof(outputBuf));

  return 0;
}

// send telemetry 
// serial, pitch, roll, alt, X, Y, (uint8_t) status, I, V, (uint8_t) db, thrust
uint16_t sendTelemetry(AP_HAL::UARTDriver *uart,
                      uint16_t pitch, uint16_t roll, uint16_t alt, 
                      uint16_t X, uint16_t Y, uint8_t status, 
                      uint16_t I, uint16_t V, uint8_t db, uint16_t thrust)
{
  uint16_t BB;
  uint8_t outputBuf [BUFSIZE];

  outputBuf[0] = 0x55;    // Шапка1
	outputBuf[1] = 0x55;    // Шапка2
  outputBuf[2] = 0X21;    // Количество байт (можно всегда 30)
	outputBuf[3] = 0X00;    // Номер команды, должен быть 0
		
  //Питч и рол	
  BB = (uint16_t) trunc((pitch)*10);	
	uint8_t hb=BB>>8;						
	uint8_t lbP=(BB<<8)>>8;					
	hb=hb<<4;						
	BB=trunc((roll)*10);		
	hb=hb+(BB>>8);					
	uint8_t lbR=(BB<<8)>>8;	 				
	outputBuf[4]=hb;                        
	outputBuf[5]=lbP; //питч
	outputBuf[6]=lbR; //рол
		    
	outputBuf[7]=0x00;
	outputBuf[8]=0x00;
	outputBuf[9]=0x00;
			
  //Высота
	BB = (uint16_t)trunc(32767 + alt * 100); 
	outputBuf[10] = BB>>8;                
	outputBuf[11] = BB;
			
	outputBuf[12] = 0x00;
	outputBuf[13] = 0x00;	

  // Х в метрах
  BB = X;
	outputBuf[14] = 0x00;                 
	outputBuf[15] = 0x00;
			
  // Y в метрах
	BB = Y;
	outputBuf[16] = 0x00;
	outputBuf[17] = 0x00;
			
  // Статус режима полёта 0=ACRO 1=STAB 2=ANGL    
	outputBuf[18] = status;                
			
  // LQ
	BB = I;                            
	outputBuf[19] = 0x00;
	outputBuf[20] = 0x00;
			
  // Напряжение
	BB = V;
	outputBuf[21] = BB>>8;
	outputBuf[22] = BB;
			
	outputBuf[23]=0x00;
	outputBuf[24]=0x00;
	outputBuf[25]=0x00;
	outputBuf[26]=0x00;
			
	outputBuf[27]=0x00;
	outputBuf[28]=0x00;
	outputBuf[29]=0x00;
	outputBuf[30]=0x00;

  //децибелы
  outputBuf[31] = db;
  //Тяга
	outputBuf[32] = (uint8_t)truncf((thrust-1000)/10);        

  // Контролька
  outputBuf[33] = checksum(outputBuf, outputBuf[3]);

  //отправка в хардсериал 
  uart->write(outputBuf, outputBuf[2]+3);
  uart->flush();

  memset(outputBuf, 0, sizeof(outputBuf));
  return 0;
}