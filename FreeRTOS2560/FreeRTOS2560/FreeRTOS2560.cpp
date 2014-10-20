/*
 * FreeRTOS2560.c
 *
 * Created: 10/12/2014 6:07:26 PM
 *  Author: Yumeng
 */ 

#include <avr/io.h>
#include "FreeRTOS.h"
#include <Arduino.h>
#include <task.h>
#include <queue.h>
#include <timers.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define trigPin A1
#define echoPin A2
#define GREEN_PIN 35
#define RED_PIN 34

#define trigPinTwo A3
#define echoPinTwo A4

#define trigPinThree A5
#define echoPinThree A6
//---------------------------------------------------------Task2-------------------------------------------------


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#define BUZZER 29

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
HMC5883L mag;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
int ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
int walkingFlag = 0;
int averageReading[1000];
long readings50[50];
long readings50x[50];
long readings50y[50];
long minReading = 100000;
long maxReading = 0;
long average;
long averagex;
long averagey;
long average4[4];
long average4x[4];
long average4y[4];
long threshold = 0;
long sample_new;
long sample_old;
long sample_result;
long sample_oldx;
long sample_oldy;
long sample_newx;
long sample_newy;
long sample_resultx;
long sample_resulty;
int count=0;
int countA = 0;
int stepTaken = 0;
int walkMode = 0;
int sendFlag = 0;

#define OUTPUT_READABLE_ACCELGYRO


char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character
int READY;
int walked;
int ACK;
int ACK_DATA;
int NACK;
int sendIndex= 0;
int stepped = 0;
int DATA;
int DATA_ID;
long DATA_DATA;
char DATA_DATA_STRING;
int DATA_REMAINDER;
char charID[2];
char charData[4];
char charRemainder[4];
int bearingBuffer[10];
//Bearing
float heading;
int bearings;




void setup() {
	Serial.begin(9600);
	Serial1.begin(9600);
	
	//---------------------------------task2 setup  untrosound-----------------------------
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(RED_PIN, OUTPUT);
	
	pinMode(trigPinTwo, OUTPUT);
	pinMode(echoPinTwo, INPUT);

	pinMode(trigPinThree, OUTPUT);
	pinMode(echoPinThree, INPUT);
	//---------------------------------task1 setup  mpu-------------------------------
	
	pinMode(BUZZER, OUTPUT);
	
	 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    //Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    accelgyro.setI2CBypassEnabled(true);
    mag.initialize();
    Serial.println("Testing device connections...");
    Serial.println(mag.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	
}
void readHMC(){
	 // read raw heading measurements from device
	 mag.getHeading(&mx, &my, &mz);
	 // To calculate heading in degrees. 0 degree indicates North
	 heading = atan2(my, mx);
	 if(heading < 0)
	 heading += 2 * M_PI;
	 bearings = (int)((heading * 180/M_PI)+94)%360;
	 Serial.print("bearings:\t");
	 Serial.println(bearings);
}

char Comp(char* This) {
	while (Serial1.available() > 0) // Don't read unless
	// there you know there is data
	{
		if(index < 19) // One less than the size of the array
		{
			inChar = Serial1.read(); // Read a character
			inData[index] = inChar; // Store it
			index++; // Increment where to write next
			inData[index] = '\0'; // Null terminate the string
		}
	}

	if (strcmp(inData,This)  == 0) {
		for (int i=0;i<19;i++) {
			inData[i]=0;
		}
		index=0;
		return(0);
	}
	else {
		return(1);
	}
}
void sendBearings(){
	DATA = bearingBuffer[sendIndex];
	//debugging
	Serial.print("FUCK");
	Serial.println(DATA);
	Serial.println(stepTaken);
	DATA_REMAINDER = DATA % 360;
	charID[0] = (unsigned char)(DATA_ID + 48);
	charID[1] = '\n';
	Serial1.write(charID[0]);
	Serial1.write(charID[1]);
	Serial.println(charID[0]);
	Serial.println(charID[1]);
	charData[0] = (unsigned char)(DATA/100 + 48);
	charData[1] = (unsigned char)((DATA/10)%10 + 48);
	charData[2] = (unsigned char)(DATA%10 + 48);
	charData[3] = '\n';
	Serial1.write(charData[0]);
	Serial1.write(charData[1]);
	Serial1.write(charData[2]);
	Serial1.write(charData[3]);
	Serial.println(charData[0]);
	Serial.println(charData[1]);
	Serial.println(charData[2]);
	Serial.println(charData[3]);
	
	charRemainder[0] = (unsigned char)(DATA_REMAINDER/100 + 48);
	charRemainder[1] = (unsigned char)((DATA_REMAINDER/10)%10 + 48);
	charRemainder[2] = (unsigned char)(DATA_REMAINDER%10 + 48);
	charRemainder[3] = '\n';
	
	Serial1.write(charRemainder[0]);
	Serial1.write(charRemainder[1]);
	Serial1.write(charRemainder[2]);
	Serial1.write(charRemainder[3]);
	Serial.println(charRemainder[0]);
	Serial.println(charRemainder[1]);
	Serial.println(charRemainder[2]);
	Serial.println(charRemainder[3]);
	DATA_ID = 0;
}
void walking()
{
	while(count < 50)
	{
		for(countA = 0; countA < 4; countA++){
			accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			average4[countA] = az;
			average4x[countA] = ax;
			average4y[countA] = ay;
		}
		average = (average4[0] + average4[1] + average4[2] + average4[3])/4;
		readings50[count] = average;
		averagex = (average4x[0] + average4x[1] + average4x[2] + average4x[3])/4;
		readings50x[count] = averagex;
		averagey = (average4y[0] + average4y[1] + average4y[2] + average4y[3])/4;
		readings50y[count] = averagey;
		if(count == 0){
			sample_new = readings50[0];
			sample_newy = readings50y[0];
			sample_newx = readings50x[0];
		}
		
		sample_old = sample_new;
		sample_new = average;
		sample_result = sample_old - sample_new;
		sample_oldx = sample_newx;
		sample_newx = averagex;
		sample_resultx = sample_oldx - sample_newx;
		sample_oldy = sample_newy;
		sample_newy = averagey;
		sample_resulty = sample_oldy - sample_newy;
		/* Serial.print("ax"); Serial.print("\t");  Serial.print("ay");  Serial.print("\t"); Serial.println("az");
		  Serial.print(averagex); Serial.print("\t");  Serial.print(averagey);  Serial.print("\t"); Serial.println(average);*/
		if(sample_result>3000 && (threshold-sample_new > 2000)){ //threshold to be determined
			if(sample_result>sample_resultx && sample_result>sample_resulty){
				if(stepTaken == 10 ) // reset the steptaken to 0 if he hit the maximum buffer
				stepTaken = 0;
				digitalWrite(BUZZER, HIGH); //when I take a step
				delay(20);
				digitalWrite(BUZZER, LOW);
				walked = 1;
				DATA_ID = 1;
				readHMC();   //take the bearings
				bearingBuffer[stepTaken] = bearings;   //store bearings of the step into the bearingbuffer
				if(sendFlag){
					sendBearings();
				}
				sendFlag = 0;
				stepTaken++;     //increment the steptaken index
			}
			Serial.print("Step"); Serial.print("\t");
			Serial.println(stepTaken);
			delay(400);
		}
		if(minReading >readings50[count]){
			minReading = readings50[count];
		}
		if(readings50[count] > maxReading){
			maxReading = readings50[count];
		}
		count++;
		//Serial.print("count: ");
		//Serial.println(count);
	}
	threshold = (maxReading + minReading)/ 2;
	//reinitialisation for comparing
	maxReading = 0;
	minReading =1000000;
	count = 0;
	delay(50);
}

void task1 (void *p)
{
	while(1){
		Serial.println("mpu test");
		 READY = Comp("ready"); // when rpi  send me ready, i send him back ready
		 if(READY == 0){
			 Serial1.write("ready");
		 }
		 ACK = Comp("ACK"); //when rpi send me ack, i start walking for the first time
		 if(ACK == 0){
			 Serial.println("ACK");
			 digitalWrite(BUZZER, HIGH); //Beep so i know i can start walking
			 delay(100);
			 digitalWrite(BUZZER, LOW);
			 walkMode = 1;
			 sendFlag = 1;
		 }
		 ACK_DATA = Comp("ACK!"); // when rpi recieved the data i send
		 if(ACK_DATA == 0){
			 sendFlag = 1;
			 sendIndex++;
			 if(sendIndex == 10){
				 sendIndex = 0;
			 }
			 ACK_DATA = 1;
		 }
		 NACK = Comp("NACK");
		 if(NACK == 0){
			 Serial.println("NACK");
			 sendFlag = 1;
			 NACK = 1;
		 }
		 if(walkMode == 1){
			 walking(); //begin walking
		 }
		 vTaskDelay(200);
	}
}

long ratio = 1.2*4;

void task2(void *p) {
	while(1){
		vTaskDelay(400);
		long t = millis();
		long duration, distance;
		digitalWrite(trigPin, LOW);  // Added this line
		delayMicroseconds(2); // Added this line
		digitalWrite(trigPin, HIGH);
		delayMicroseconds(10); // Added this line
		digitalWrite(trigPin, LOW);
		duration = pulseIn(echoPin, HIGH, 1000000);
		distance = (duration/2)*ratio / 29.1;

		if(distance > 0 && distance <100){
			digitalWrite(RED_PIN, HIGH);
			digitalWrite(GREEN_PIN,LOW);
		}
		if(distance >= 100){
			digitalWrite(RED_PIN,LOW);
			digitalWrite(GREEN_PIN, HIGH);
		}
/*

		long t2 = millis();

		Serial.print("The time needed for ultrosound is");
		Serial.println(t2-t);*/
		delay(400);
		
		Serial.print("The distance to obstacles in front is: --------------first");
		Serial.print(distance);
		Serial.println("cm");
		
	}
}


void task3(void *p) {
	while(1){
		vTaskDelay(300);
		long duration, distance;
		digitalWrite(trigPinTwo, LOW);  // Added this line
		delayMicroseconds(2); // Added this line
		digitalWrite(trigPinTwo, HIGH);
		delayMicroseconds(10); // Added this line
		digitalWrite(trigPinTwo, LOW);
		duration = pulseIn(echoPinTwo, HIGH, 1000000);
		distance = (duration/2)*ratio / 29.1;

		if(distance > 0 && distance <100){
			digitalWrite(RED_PIN, HIGH);
			digitalWrite(GREEN_PIN,LOW);
		}
		if(distance >= 100){
			digitalWrite(RED_PIN,LOW);
			digitalWrite(GREEN_PIN, HIGH);
		}
		delay(1000);
		Serial.print("The distance to obstacles in front is:------------second ");
		Serial.print(distance);
		Serial.println(" cm");
	}
}

void task4(void *p) {
	while(1){
		vTaskDelay(500);
		long duration, distance;
		digitalWrite(trigPinThree, LOW);  // Added this line
		delayMicroseconds(2); // Added this line
		digitalWrite(trigPinThree, HIGH);
		delayMicroseconds(10); // Added this line
		digitalWrite(trigPinThree, LOW);
		duration = pulseIn(echoPinThree, HIGH, 1000000);
		distance = (duration/2)*ratio / 29.1;

		if(distance > 0 && distance <100){
			digitalWrite(RED_PIN, HIGH);
			digitalWrite(GREEN_PIN,LOW);
		}
		if(distance >= 100){
			digitalWrite(RED_PIN,LOW);
			digitalWrite(GREEN_PIN, HIGH);
		}
		delay(1000);
		Serial.print("The distance to obstacles in front is:------------third");
		Serial.print(distance);
		Serial.println(" cm");
	}
}

#define STACK_DEPTH 256
//512+256 = 768


void vApplicationIdleHook()
{
	
}

int main(void)
{
	init();
	setup();


	pinMode(11,OUTPUT);
	TaskHandle_t t1, t2, t3, t4;

	//Create tasks
	xTaskCreate(task1, "Task 1", STACK_DEPTH, NULL, 6, &t1);
	xTaskCreate(task2, "Task 2", STACK_DEPTH, NULL, 5, &t2);
	xTaskCreate(task3, "Task 3", STACK_DEPTH, NULL, 5, &t3);
	//xTaskCreate(task4, "Task 4", STACK_DEPTH, NULL, 5, &t4);
	
	vTaskStartScheduler();

}

