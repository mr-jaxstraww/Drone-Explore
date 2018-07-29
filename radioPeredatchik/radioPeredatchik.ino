#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // библиотека  радиомодуля 

#define freq 25 // - частота обновлений

//LEFT JOYSTICK
#define leftJoyX A7
#define leftJoyY A6
#define leftPoint 2

//RIGHT JOYSTICK
#define rightJoyX A1
#define rightJoyY A0
#define rightPoint 7

RF24 radio(9, 10); // - создать модуль на пинах 9 и 10
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // - номера труб

unsigned long microsPerReading, microsPrevious; // - переменные для таймера


void setup()
{
  /*настройка радио модуля nrf24l01*/
  radio.begin(); //активировать модуль
  radio.setAutoAck(1);
  radio.setRetries(0, 15);
  radio.enableAckPayload();
  radio.setPayloadSize(32); //размер пакета в байтах

  radio.openWritingPipe(address[0]);
  radio.setChannel(0x60);

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);

  radio.powerUp();
  radio.stopListening();
  /*----*/
  /*читаем значения с левого джойстика*/
  pinMode(leftJoyX, INPUT);
  pinMode(leftJoyY, INPUT);
  pinMode(leftPoint, INPUT);
  /*читаем значения с правого джойстика*/
  pinMode(rightJoyX, INPUT);
  pinMode(rightJoyY, INPUT);
  pinMode(rightPoint, INPUT);

  microsPerReading = 1000000 / freq; // - инициализуем "таймер" для темпа расчетов
  microsPrevious = micros();	// - получаем начальное время (время запуска)
}


void loop()
{
	//[0] - leftJoy X(высота); [1] - leftJoy Y; [2] - leftJoy Point(кнопка); 
	//[3] rightJoy X(тангаж(+45-0;0--45)); [4] rightJoy Y(крен(+45-0;0--45)); [5] rightJoyPoint(кнопка);
	unsigned long microsNow; // - переменная для хранения нынешнего времени
	microsNow = micros();
	if (microsNow - microsPrevious >= microsPerReading) {
		int pitch = getPitch();
		int roll = getRoll();
		int allJoyValued[] = { map(analogRead(leftJoyX),0,1023,800,2200), map(analogRead(leftJoyY),0,1023,800,2200), analogRead(leftPoint),pitch, roll, analogRead(rightPoint) };
		radio.write(&allJoyValued, sizeof(allJoyValued)); // - отправляем значение по каналу радиомодуля
		// increment previous time, so we keep proper pace
		microsPrevious = microsPrevious + microsPerReading;
	}
  
}

//расчет угла наклона тангажа
int getPitch() {
	int pitchS = analogRead(rightJoyX);
	if (pitchS <= 634) {
		pitchS = map(pitchS, 0, 634, -45, 0);
	}
	else if (pitchS >= 634) {
		pitchS = map(pitchS, 634, 1023, 0, 45);
	}
	return pitchS;
}
//расчет угла наклона крена
int getRoll() {
	int rollS = analogRead(rightJoyY);
	if(rollS <= 604) {
		rollS = map(rollS, 0, 604, -45, 0);
	}
	else if (rollS >= 604) {
		rollS = map(rollS, 604, 1023, 0, 45);
	}
	return rollS;
}


