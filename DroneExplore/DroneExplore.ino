#include <MadgwickAHRS.h> // - ���������� �������(��������� ��� MPU6050,��� ������������
#include <MPU6050.h> // - ���������� ��� ������ � MPU6050
#include <pisk.h> // - ���������� ��� ������ � ����� ���������
#include <SPI.h> // - ���������� ��� ������ � ����� SPI
#include "nRF24L01.h" // - ���������� �����������
#include "RF24.h" // - ����������  ����������� 
#include <Servo.h> // - ��������� ������������

#define freq 25 // - ������� ����������

MPU6050 accelgyro; // - ��������� ������ 
Madgwick filter; // - ��������� ������ ���.��������

RF24 radio(9, 10); // - ������� ������ �� ����� 9 � 10 
byte address[][6] = { "1Node","2Node","3Node","4Node","5Node","6Node" }; // - ������ ����

Servo motor1, motor2, motor3, motor4; // - ��������� ������ � ��� ������������

unsigned long microsPerReading, microsPrevious; // - ���������� ��� �������
bool piskBool = 0; // - ���������� ��� ����� ��������

void setup() {
	Serial.begin(9600);

	/*--------��������� nrf �������-----------*/
	radio.begin(); //������������ ������
	radio.setAutoAck(1);
	radio.setRetries(0, 15);
	radio.enableAckPayload();
	radio.setPayloadSize(32); //������ ������ � ������ 

	radio.openReadingPipe(1, address[0]);
	radio.setChannel(0x60);

	radio.setPALevel(RF24_PA_MAX);
	radio.setDataRate(RF24_250KBPS);

	radio.powerUp();
	radio.startListening();
	/*-----------------------------------------*/
	motor1.attach(2); // �������������� ����� (D2 ���)
	motor2.attach(3); // �������������� ����� (D3 ���)
	motor3.attach(4); // �������������� �����(D4 ���)
	motor4.attach(5); // �������������� �����(D5 ���)
	/*-----------------------------------------*/
	motor1.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 1
	motor1.writeMicroseconds(800);  // }

	motor2.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 2
	motor2.writeMicroseconds(800);  // }

	motor3.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 3
	motor3.writeMicroseconds(800);  // }

	motor4.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 4
	motor4.writeMicroseconds(800);  // }

	delay(5000);

	pinMode(6, OUTPUT); // - ��� �������� �� "�����"
	pisk(6, 3, 300, 100); // - ���� ������ �����������
	piskBool = !piskBool; // - ����������� �������� ���������� ����� �������� (0->1)

	accelgyro.initialize(); // - �������������� ������ 
	filter.begin(freq); // - �������������� ������ 


	microsPerReading = 1000000 / freq; // - ������������ "������" ��� ����� ��������
	microsPrevious = micros();	// - �������� ��������� ����� (����� �������)
}

void loop() {
	/*��������� ���������� ��� �������� ��������,���������� � �������*/
	int ax_raw, ay_raw, az_raw; 
	int gx_raw, gy_raw, gz_raw;
	/*��������� ���������� ��� �������� ������������ � ������� �� �������� (��� ��������) */
	float ax, ay, az;
	float gx, gy, gz;
	/*��������� ���������� ��� ��������� �������� �����*/
	float rollX, pitchY;

	unsigned long microsNow; // - ���������� ��� �������� ��������� �������

	/*��������� �������� �������*/
	microsNow = micros();
	/*��������� ������ � ��������� ���*/
	if (microsNow - microsPrevious >= microsPerReading) {

		/*��������� ������ � �������*/
		accelgyro.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

		/*������������ �� �������������� ������ � G � � �������/�������*/
		ax = convertRawAcceleration(ax_raw);
		ay = convertRawAcceleration(ay_raw);
		az = convertRawAcceleration(az_raw);
		gx = convertRawGyro(gx_raw);
		gy = convertRawGyro(gy_raw);
		gz = convertRawGyro(gz_raw);

		/*��������� ������, ������� ��������� �����������*/
		filter.updateIMU(gx, gy, gz, ax, ay, az);

		rollX = filter.getRoll(); // - �������� ���� �������� �� ��� X,�.� mpu6050 ���������� �� 180 ��������,����� ���������� ����������
		pitchY = filter.getPitch(); // - �������� ���� �������� �� ��� Y

		// - �����������  
		if (piskBool = 1 &&-182.0f <= rollX && rollX <= -178.0f && -3.0f <= pitchY && pitchY <= 3.0f) {
			pisk(6, 3, 300, 100); // - ���� ����� �����������
			piskBool = !piskBool; // - ����������� �������� ���������� ����� �������� (1->0)
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
	/* ��������� �� ���������� �������� 2G
	   -2g ���������� �������� �������� -32768
	   +2g ���������� �������� �������� +32768 
	*/

	float a = (a_raw * 2.0f) / 32768.0f;
	return a;
}

float convertRawGyro(int g_raw) {
	/* ��������� �� ���������� �������� 250 �������� / ������
	  -250 - �������������� �������� -32768
	  +250 - �������������� �������� +32768 
	*/

	float g = (g_raw * 250.0) / 32768.0;
	return g;
}


