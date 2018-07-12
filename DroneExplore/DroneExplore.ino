#include <MadgwickAHRS.h> // - библиотека фильтра(урезанная для MPU6050,без магнитометра
#include <MPU6050.h> // - библиотека для работы с MPU6050
#include <pisk.h> // - библиотека для работы с пьезо Динамиком
#include <SPI.h> // - библиотека для работы с шиной SPI
#include "nRF24L01.h" // - библиотека радиомодуля
#include "RF24.h" // - библиотека  радиомодуля 
#include <Servo.h> // - библотека сервопривода

#define freq 25 // - частота обновлений

MPU6050 accelgyro; // - обьявляем датчик 
Madgwick filter; // - обьявляем фильтр Себ.Маджвика

RF24 radio(9, 10); // - создать модуль на пинах 9 и 10 
byte address[][6] = { "1Node","2Node","3Node","4Node","5Node","6Node" }; // - номера труб

Servo motor1, motor2, motor3, motor4; // - обьявляем моторы с ПИД регуляторами

unsigned long microsPerReading, microsPrevious; // - переменные для таймера
bool piskBool = 0; // - включатель для пьезо Динамика

void setup() {
	Serial.begin(9600);

	/*--------настройка nrf модулей-----------*/
	radio.begin(); //активировать модуль
	radio.setAutoAck(1);
	radio.setRetries(0, 15);
	radio.enableAckPayload();
	radio.setPayloadSize(32); //размер пакета в байтах 

	radio.openReadingPipe(1, address[0]);
	radio.setChannel(0x60);

	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);

	radio.powerUp();
	radio.startListening();
	/*-----------------------------------------*/
	motor1.attach(2); // инициализуруем мотор (D2 пин)
	motor2.attach(3); // инициализуруем мотор (D3 пин)
	motor3.attach(4); // инициализуруем мотор(D4 пин)
	motor4.attach(5); // инициализуруем мотор(D5 пин)
	/*-----------------------------------------*/
	motor1.writeMicroseconds(2200); // }
	delay(2000);					// - каллибровка мотора 1
	motor1.writeMicroseconds(800);  // }

	motor2.writeMicroseconds(2200); // }
	delay(2000);					// - каллибровка мотора 2
	motor2.writeMicroseconds(800);  // }

	motor3.writeMicroseconds(2200); // }
	delay(2000);					// - каллибровка мотора 3
	motor3.writeMicroseconds(800);  // }

	motor4.writeMicroseconds(2200); // }
	delay(2000);					// - каллибровка мотора 4
	motor4.writeMicroseconds(800);  // }

	delay(5000);

	pinMode(6, OUTPUT); // - пин динамика на "ВЫХОД"
	pisk(6, 3, 300, 100); // - звук начала каллибровки
	piskBool = !piskBool; // - инвертируем значение включателя пьезо Динамика (0->1)

	accelgyro.initialize(); // - инициализируем датчик 
	filter.begin(freq); // - инициализируем фильтр 


	microsPerReading = 1000000 / freq; // - инициализуем "таймер" для темпа расчетов
	microsPrevious = micros();	// - получаем начальное время (время запуска)
}

void loop() {
	/*обьявляем переменные для хранения значений,полученных с датчика*/
	int ax_raw, ay_raw, az_raw; 
	int gx_raw, gy_raw, gz_raw;
	/*обьявляем переменные для хранения переведенных в систему СИ значений (для фитрации) */
	float ax, ay, az;
	float gx, gy, gz;
	/*обьявляем переменные для храненеия итоговых углов*/
	float rollX, pitchY;

	unsigned long microsNow; // - переменная для хранения нынешнего времени

	/*обновляем значение времени*/
	microsNow = micros();
	/*проверяем таймер и запускаем его*/
	if (microsNow - microsPrevious >= microsPerReading) {

		/*считываем данные с датчика*/
		accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

		/*конвертируем из необработанных данных в G и в градусы/секунды*/
		ax = convertRawAcceleration(ax_raw);
		ay = convertRawAcceleration(ay_raw);
		az = convertRawAcceleration(az_raw);
		gx = convertRawGyro(gx_raw);
		gy = convertRawGyro(gy_raw);
		gz = convertRawGyro(gz_raw);

		/*обновляем фильтр, который вычисляет кватернионы*/
		filter.updateIMU(gx, gy, gz, ax, ay, az);

		rollX = filter.getRoll(); // - получаем угол вращения по оси X,т.к mpu6050 перевернут на 180 градусов,нужно произвести калибровку
		pitchY = filter.getPitch(); // - получаем угол вращения по оси Y

		// - каллибровка  
		if (piskBool = 1 &&-182.0f <= rollX && rollX <= -178.0f && -3.0f <= pitchY && pitchY <= 3.0f) {
			pisk(6, 3, 300, 100); // - звук конца каллибровки
			piskBool = !piskBool; // - инвертируем значение включателя пьезо Динамика (1->0)
		}
		else
		{
			Serial.print("X: "); Serial.print(rollX); Serial.print("Y: "); Serial.println(pitchY); 
		}
		// increment previous time, so we keep proper pace
		microsPrevious = microsPrevious + microsPerReading;
	}
}

float convertRawAcceleration(int a_raw) {
	/* поскольку мы используем диапазон 2G
	   -2g отображает исходное значение -32768
	   +2g отображает исходное значение +32768 
	*/

	float a = (a_raw * 2.0f) / 32768.0f;
	return a;
}

float convertRawGyro(int g_raw) {
	/* поскольку мы используем диапазон 250 градусов / секунд
	  -250 - необработанное значение -32768
	  +250 - необработанное значение +32768 
	*/

	float g = (g_raw * 250.0) / 32768.0;
	return g;
}


