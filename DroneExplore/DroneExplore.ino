#include <MadgwickAHRS.h> // - ���������� �������(��������� ��� MPU6050,��� ������������
#include <MPU6050.h> // - ���������� ��� ������ � MPU6050
#include <pisk.h> // - ���������� ��� ������ � ����� ���������
#include <SPI.h> // - ���������� ��� ������ � ����� SPI
#include "nRF24L01.h" // - ���������� �����������
#include "RF24.h" // - ����������  ����������� 
#include <Servo.h> // - ��������� ������������
/*------------------------------------------------------*/
#define freq 25 // - ������� ����������
/*------------------------------------------------------*/
MPU6050 accelgyro; // - ��������� ������ 
Madgwick filter; // - ��������� ������ ���.��������
/*------------------------------------------------------*/
RF24 radio(9, 10); // - ������� ������ �� ����� 9 � 10 
byte address[][6] = { "1Node","2Node","3Node","4Node","5Node","6Node" }; // - ������ ����
int heightNRF, pitchNRF, rollNRF; // - ������,������,����,���������� � �����������
/*------------------------------------------------------*/
Servo motor1, motor2, motor3, motor4; // - ��������� ������ � ��� ������������
Servo thrust[4] = { motor1, motor2, motor3, motor4 }; // ���� ���� �������
Servo pitchTOP[2] = { motor1, motor2 };//D2,D3
Servo pitchBOTTOM[2] = { motor4, motor3 };//D5,D4
Servo rollTOP[2] = { motor1, motor4 };//D2,D5
Servo rollBOTTOM[2] = { motor2, motor3 };//D3,D4
/*------------------------------------------------------*/

unsigned long microsPerReading, microsPrevious; // - ���������� ��� �������
bool piskBool = 0; // - ���������� ��� ����� ��������
bool isCallibrated = 0; // - ���� �����������
 
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
	motor1.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 1
	motor1.writeMicroseconds(800);  // }

	delay(500);

	motor2.attach(3); // �������������� ����� (D3 ���)
	motor2.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 2
	motor2.writeMicroseconds(800);  // }

	delay(500);

	motor3.attach(4); // �������������� �����(D4 ���)
	motor3.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 3
	motor3.writeMicroseconds(800);  // }

	delay(500);

	motor4.attach(5); // �������������� �����(D5 ���)
	motor4.writeMicroseconds(2200); // }
	delay(2000);					// - ����������� ������ 4
	motor4.writeMicroseconds(800);  // }
	
	delay(2500);
	
	/*----------------------------------*/
	pinMode(6, OUTPUT); // - ��� �������� �� "�����"
	piskSOS(6); // - ���� ������ �����������
	piskBool = !piskBool; // - ����������� �������� ���������� ����� �������� (0->1)
	/*----------------------------------*/
	accelgyro.initialize(); // - �������������� ������ 
	filter.begin(freq); // - �������������� ������ 


	microsPerReading = 1000000 / freq; // - ������������ "������" ��� ����� ��������
	microsPrevious = micros();	// - �������� ��������� ����� (����� �������)
}

void loop() {
	/*��������� ���������� ��� �������� ������� ������ � �����������*/
	int potData[6];
	byte pipeNo;
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
		/*�������� �������� �� �����������*/
		while (radio.available(&pipeNo)) {
			//[0] - leftJoy X(������); [1] - leftJoy Y; [2] - leftJoy Point(������); 
			//[3] rightJoy X(������(+45-0;0--45)); [4] rightJoy Y(����(+45-0;0--45)); [5] rightJoyPoint(������);
			radio.read(&potData, sizeof(potData));
			heightNRF = potData[0]; // - ������ � ������������(�����������)
			pitchNRF = potData[3]; // - ������ � ������������(�����������)
			rollNRF = potData[4]; // - ���� � ������������(�����������)
		}
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

		rollX = filter.getRoll(); // - �������� ���� �����(�������� �� ��� X),�.� mpu6050 ���������� �� 180 ��������,����� ���������� ����������
		pitchY = filter.getPitch(); // - �������� ���� �������(�������� �� ��� Y)

		// - �����������  
		if (piskBool == 1  && checkHorizontal(pitchY, rollX)) {
			piskBool = 0; // - ����������� �������� ���������� ����� �������� (1->0)
			pisk(6, 3, 300, 100); // - ���� ����� �����������
			isCallibrated = 1; // - ������� ����������� ���������
		}
		else if (isCallibrated == 1)
		{	
			getThrust(heightNRF);
		}
		// increment previous time, so we keep proper pace
		microsPrevious = microsPrevious + microsPerReading;
	}
}
//������������ ������ ���� ��� ����� 90 ����.
void getThrust(int power_raw) {
	thrust[0].writeMicroseconds(power_raw);
	thrust[1].writeMicroseconds(power_raw);
	thrust[2].writeMicroseconds(power_raw);
	thrust[3].writeMicroseconds(power_raw);
}
void getThrust(int power_raw, int angle_pitch, int angle_roll, int real_pitch, int real_roll) {
	int realPitch, realRoll;
	realPitch = real_roll;
	realRoll = real_pitch;
	
}
/*�������� �� �������������� ������������*/
bool checkHorizontal(float pitchY, float rollX)  {
	if (-185.0f <= rollX && rollX <= -175.0f && -5.0f <= pitchY && pitchY <= 5.0f) {
		return true;
	}
	else return false;
}
/*����������� � G ������ � �������������*/
float convertRawAcceleration(int a_raw) {
	/* ��������� �� ���������� �������� 2G
	   -2g ���������� �������� �������� -32768
	   +2g ���������� �������� �������� +32768 
	*/

	float a = (a_raw * 2.0f) / 32768.0f;
	return a;
}
/*����������� � ����/��� ������ � ���������*/
float convertRawGyro(int g_raw) {
	/* ��������� �� ���������� �������� 250 �������� / ������
	  -250 - �������������� �������� -32768
	  +250 - �������������� �������� +32768 
	*/

	float g = (g_raw * 250.0) / 32768.0;
	return g;
}


