#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // библиотека  радиомодуля 

//LEFT JOYSTICK
#define leftJoyX A7
#define leftJoyY A6
#define leftPoint 2

//RIGHT JOYSTICK
#define rightJoyX A1
#define rightJoyY A0
#define rightPoint 7

RF24 radio(9, 10); // создать модуль на пинах 9 и 10
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //номера труб



void setup()
{
  Serial.begin(9600);
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
  /*читае значения с правого джойстика*/
  pinMode(rightJoyX, INPUT);
  pinMode(rightJoyY, INPUT);
  pinMode(rightPoint, INPUT);

}


void loop()
{
  int allJoyValued[] = {analogRead(leftJoyX), analogRead(leftJoyY), analogRead(leftPoint), analogRead(rightJoyX), analogRead(rightJoyY), analogRead(rightPoint)};
  //Serial.print("LeftJoyY"); Serial.println(allJoyValued[1]);
  radio.write(&allJoyValued,sizeof(allJoyValued));
  delay(10);
}



