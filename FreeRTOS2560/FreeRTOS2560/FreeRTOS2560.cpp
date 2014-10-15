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


//---------------------------------------------------------Task2-------------------------------------------------


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
HMC5883L mag;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO


#define LED_PIN 13
#define LED_PIN2 14
bool blinkState = false;


void setup() {
	//---------------------------------task3 setup  untrosound-----------------------------
	pinMode(trigPin, OUTPUT);
	pinMode(echoPin, INPUT);
	pinMode(GREEN_PIN, OUTPUT);
	pinMode(RED_PIN, OUTPUT);

	//---------------------------------task2 setup  mpu-------------------------------
	
	// configure Arduino LED for
	pinMode(LED_PIN, OUTPUT);
	pinMode(LED_PIN2, OUTPUT);
	
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

	// display tab-separated gyro x/y/z values
	Serial.print("mag:\t");
	Serial.print(mx); Serial.print("\t");
	Serial.print(my); Serial.print("\t");
	Serial.print(mz); Serial.print("\t");
	
	// To calculate heading in degrees. 0 degree indicates North
	float heading = atan2(my, mx);
	if((int)heading < 0){
		heading += 2 * M_PI;
	}
	Serial.print("heading:\t");
	Serial.println(heading * 180/M_PI);

	// blink LED to indicate activity
	blinkState = !blinkState;
	digitalWrite(LED_PIN2, blinkState);
}


void task2 (void *p)
{
	while (1)
	{
	//readHMC();
	// read raw accel/gyro measurements from device
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	delay(1000);
	// these methods (and a few others) are also available
	//accelgyro.getAcceleration(&ax, &ay, &az);
	//accelgyro.getRotation(&gx, &gy, &gz);

	//#ifdef OUTPUT_READABLE_ACCELGYRO
	// display tab-separated accel/gyro x/y/z values
	Serial.print("a/g:\t");
	Serial.print(ax); Serial.print("\t");
	Serial.print(ay); Serial.print("\t");
	Serial.print(az); Serial.print("\t");
	Serial.print(gx); Serial.print("\t");
	Serial.print(gy); Serial.print("\t");
	Serial.println(gz);
//	#endif

	//#ifdef OUTPUT_BINARY_ACCELGYRO
	//Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
	//Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
	//Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
	//Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
	//Serial..write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
	//#ewrite((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
	//Serialndif
//
	//// blink LED to indicate activity
	//blinkState = !blinkState;
	//digitalWrite(LED_PIN, blinkState);
	//vTaskDelay(2000);
	}
}

void task3(void *p) {
	while(1){
		long duration, distance;
		digitalWrite(trigPin, LOW);  // Added this line
		delayMicroseconds(2); // Added this line
		digitalWrite(trigPin, HIGH);
		delayMicroseconds(10); // Added this line
		digitalWrite(trigPin, LOW);
		duration = pulseIn(echoPin, HIGH, 1000000);
		distance = (duration/2) / 29.1;

		if(distance > 0 && distance <100){
			digitalWrite(RED_PIN, HIGH);
			digitalWrite(GREEN_PIN,LOW);
			vTaskDelay(500);
		}
		if(distance >= 100){
			digitalWrite(RED_PIN,LOW);
			digitalWrite(GREEN_PIN, HIGH);
			vTaskDelay(500);
		}
		/*
		if ( distance >= 400)
		distance = 399;
		if ( distance <= 0)
		distance = 0;
		if (distance <= 9)  //when wall is too far away, the distance value will fluctuate.
		distance = 399;*/
		Serial.println("The distance to obstacles in front is: ");
		Serial.println(distance);
		Serial.println("cm");
		delay(1000);
	}
}

void task1(void *p)
{
	while (1)
	{
	
	digitalWrite(11, HIGH);
	vTaskDelay(500);
	digitalWrite(11,LOW);
	vTaskDelay(500);
	}
}


#define STACK_DEPTH 502


void vApplicationIdleHook()
{
	
}

int main(void)
{
	init();
	setup();
	Serial.begin(9600);


	pinMode(11,OUTPUT);
	//TaskHandle_t t1, t2, t3;
	TaskHandle_t t2,t3;
	//TaskHandle_t t1,t3;
	//TaskHandle_t t3;
	//TaskHandle_t t1;
	//Create tasks
	//xTaskCreate(task1, "Task 1", STACK_DEPTH, NULL, 4, &t1);
	xTaskCreate(task2, "Task 2", STACK_DEPTH, NULL, 5, &t2);
	xTaskCreate(task3, "Task 3", STACK_DEPTH, NULL, 3, &t3);

	vTaskStartScheduler();

}

