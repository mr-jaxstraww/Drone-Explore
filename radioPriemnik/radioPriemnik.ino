#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // библиотека  радиомодуля 
#include <Servo.h>        // библотека сервопривода
#include <pisk.h>         // библиотека баззера

#define bazzer 6        // баззер

RF24 radio(9,10); // создать модуль на пинах 9 и 10 
byte address[][6] = {"1Node","2Node","3Node","4Node","5Node","6Node"}; //номера труб

Servo motor1,motor2,motor3,motor4; // 1 мотор 
void setup()
{
  pinMode(bazzer,OUTPUT); 
  Serial.begin(9600);
  /*--------настройка nrf модулей-----------*/
  radio.begin(); //активировать модуль
  radio.setAutoAck(1);
  radio.setRetries(0,15);
  radio.enableAckPayload();
  radio.setPayloadSize(32); //размер пакета в байтах 

  radio.openReadingPipe(1,address[0]);
  radio.setChannel(0x60);

  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);

  radio.powerUp();
  radio.startListening();
  /*---------------------------------------*/
  motor1.attach(2); // подключаем мотор (D2 пин)
  motor2.attach(3);
  motor3.attach(4);
  motor4.attach(5);

  motor1.writeMicroseconds(2200); // }
  delay(2000);					  //  каллибровка мотора 1
  motor1.writeMicroseconds(800);  // }

  motor2.writeMicroseconds(2200); // }
  delay(2000);            //  каллибровка мотора 2
  motor2.writeMicroseconds(800);  // }

  motor3.writeMicroseconds(2200); // }
  delay(2000);            //  каллибровка мотора 3
  motor3.writeMicroseconds(800);  // }

  motor4.writeMicroseconds(2200); // }
  delay(2000);            //  каллибровка мотора 4
  motor4.writeMicroseconds(800);  // }

  delay(5000);
  /*---------------------------------------*/
}


void loop()   
{
  int potData[6];
  byte pipeNo;
  while(radio.available(&pipeNo)){
    radio.read(&potData,sizeof(potData));         //[0] - leftJoy X;[1] - leftJoy Y; [2] - leftJoy Point; [3] rightJoy X; [4] rightJoy Y; [5] rightJoyPoint
      
	   int valMotor1 = map(potData[0],0,1023,800,2200); // получили значения с джойстика (0-1023) --> преобразовали в значения для коптера (800-2300)
      motor1.writeMicroseconds(valMotor1);//запускаем мотор 1 на нужных оборотах 
      motor2.writeMicroseconds(valMotor1);//запускаем мотор 2 на нужных оборотах 
      motor3.writeMicroseconds(valMotor1);//запускаем мотор 3 на нужных оборотах 
      motor4.writeMicroseconds(valMotor1);//запускаем мотор 4 на нужных оборотах 
      //Serial.println(valMotor1);
  }
}
