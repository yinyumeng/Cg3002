#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#define BUZZER 29

/* Ultrasounds pin*/
#define trigPin1 A0
#define echoPin1 A1
#define trigPin2 A2
#define echoPin2 A3
#define trigPin3 A4
#define echoPin3 A5
#define trigPin4 A6
#define echoPin4 A7

/*Ultrasound variables*/
int duration1, distance1;
int duration2, distance2;
int duration3, distance3;
int duration4, distance4;

MPU6050 accelgyro;
HMC5883L mag;
/*Accelerometer Variable*/
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


#define OUTPUT_READABLE_ACCELGYRO

/*UART variables*/
int walkMode = 0;  //flag to initiate walking
int sendFlag = 0;  //flag to initiate sending
char inData[20]; // Allocate some space for the string
char inChar=-1; // Where to store the character read
byte index = 0; // Index into array; where to store the character
int READY;  
int ACK;
int ACK_DATA;
int NACK;
int sendIndex= 0;    //to keep track on which step being send
int DATA;
int DATA_ID;
int DATA_REMAINDER;
char charID[2];
char charData[4];
char charRemainder[4];
int bearingBuffer[20];
int bufferEmpty; //1 is empty, 0 is not empty
//Bearing
float heading;
int bearings;
int bearingArr[5];

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)

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
    pinMode(BUZZER, OUTPUT);
    /* Ultrasound pinmodes */
    pinMode(trigPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin2, INPUT);
    pinMode(trigPin3, OUTPUT);
    pinMode(echoPin3, INPUT);
    pinMode(trigPin4, OUTPUT);
    pinMode(echoPin4, INPUT);
}
void ultrasound1()
{
  digitalWrite(trigPin1, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = (duration1/2) / 29.1;
  if ( distance1 >= 400)
    distance1 = 399;
  if ( distance1 <= 0)
    distance1 = 0;
  if (distance1 <= 9)  //when wall is too far away, the distance value will fluctate.
    distance1 = 399;
  Serial.println("The distance1 to obstacles in front is: ");
  Serial.print(distance1);
  Serial.println(" cm");
}
void ultrasound2()
{
  digitalWrite(trigPin2, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = (duration2/2) / 29.1;
  if ( distance2 >= 400)
    distance2 = 399;
  if ( distance1 <= 0)
    distance2 = 0;
  if (distance2 <= 9)  //when wall is too far away, the distance value will fluctate.
    distance2 = 399;
  Serial.println("The distance2 to obstacles in front is: ");
  Serial.print(distance2);
  Serial.println(" cm");
}
void ultrasound3()
{
  digitalWrite(trigPin3, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin3, LOW);
  duration3 = pulseIn(echoPin3, HIGH);
  distance3 = (duration3/2) / 29.1;
  if ( distance3 >= 400)
    distance3 = 399;
  if ( distance1 <= 0)
    distance3 = 0;
  if (distance3 <= 9)  //when wall is too far away, the distance value will fluctate.
    distance3 = 399;
  Serial.println("The distance3 to obstacles in front is: ");
  Serial.print(distance3);
  Serial.println(" cm");
}
void ultrasound4()
{
  digitalWrite(trigPin4, LOW); 
  delayMicroseconds(2); 
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin4, LOW);
  duration4 = pulseIn(echoPin4, HIGH);
  distance4 = (duration4/2) / 29.1;
  if ( distance4 >= 400)
    distance4 = 399;
  if ( distance1 <= 0)
    distance4 = 0;
  if (distance4 <= 9)  //when wall is too far away, the distance value will fluctate.
    distance4 = 399;
  Serial.println("The distance4 to obstacles in front is: ");
  Serial.print(distance4);
  Serial.println(" cm");
}
int comp(const void *elem1, const void *elem2)
{
  int f = *((int*)elem1);
  int s = *((int*)elem2);
  if (f > s) return  1;
  if (f < s) return -1;
  return 0;
}
int median(int pastzvals[])
{
  qsort(pastzvals, sizeof(pastzvals)/sizeof(*pastzvals), sizeof(*pastzvals), comp);
  return pastzvals[2];
}
void readHMC(){
    // read raw heading measurements from device
    for(int i = 0; i<5 ; i++){
    mag.getHeading(&mx, &my, &mz);
// To calculate heading in degrees. 0 degree indicates North
    heading = atan2(my, mx);
    if(heading < 0)
      heading += 2 * M_PI;
    bearingArr[i] = (int)((heading * 180/M_PI)+94)%360;
    delay(30); //to ensure the readings dont cock up
    }
    bearings = median(bearingArr);
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
    DATA_ID = 1;
    //debugging
    Serial.print("DATA sending");
    Serial.println(DATA);
    DATA_REMAINDER = DATA % 360;
    charID[0] = (unsigned char)(DATA_ID + 48);
    charID[1] = '\n';
    Serial1.write(charID[0]);
    Serial1.write(charID[1]);
    //Serial.println(charID[0]);
    //Serial.println(charID[1]);
    charData[0] = (unsigned char)(DATA/100 + 48);
    charData[1] = (unsigned char)((DATA/10)%10 + 48);
    charData[2] = (unsigned char)(DATA%10 + 48);
    charData[3] = '\n';
    Serial1.write(charData[0]);
    Serial1.write(charData[1]);
    Serial1.write(charData[2]);
    Serial1.write(charData[3]);  
//    Serial.println(charData[0]);
//    Serial.println(charData[1]);
//    Serial.println(charData[2]);
//    Serial.println(charData[3]);      
    
    charRemainder[0] = (unsigned char)(DATA_REMAINDER/100 + 48);
    charRemainder[1] = (unsigned char)((DATA_REMAINDER/10)%10 + 48);
    charRemainder[2] = (unsigned char)(DATA_REMAINDER%10 + 48);
    charRemainder[3] = '\n';
    
    Serial1.write(charRemainder[0]);
    Serial1.write(charRemainder[1]);
    Serial1.write(charRemainder[2]);
    Serial1.write(charRemainder[3]);
//    Serial.println(charRemainder[0]);
//    Serial.println(charRemainder[1]);
//    Serial.println(charRemainder[2]);
//    Serial.println(charRemainder[3]);
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
     // Serial.print("ax"); Serial.print("\t");  Serial.print("ay");  Serial.print("\t"); Serial.println("az");
    //  Serial.print(averagex); Serial.print("\t");  Serial.print(averagey);  Serial.print("\t"); Serial.println(average);
      if(sample_result>700 && (threshold-sample_new > 3000)){ //threshold to be determined
        if(((sample_result-sample_resultx)>500) && ((sample_result-sample_resulty)> 500)){
          if(stepTaken == 20 ) // reset the steptaken to 0 if he hit the maximum buffer
           stepTaken = 0;    
          digitalWrite(BUZZER, HIGH); //when I take a step
          delay(20);
          digitalWrite(BUZZER, LOW);          
          DATA_ID = 1;
          readHMC();   //take the bearings
          bearingBuffer[stepTaken] = bearings;   //store bearings of the step into the bearingbuffer
          if(sendFlag){
            sendBearings();
            sendFlag = 0;
          }        
          stepTaken++;     //increment the steptaken index     
        }
         Serial.print("Step"); Serial.print("\t");
         Serial.println(stepTaken);
         delay(600);
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
void loop()
{
//  ultrasound1();
//  ultrasound2();
//  ultrasound3();
//  ultrasound4();
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
    walkMode = 1; //ready to walk
    sendFlag = 1;  //ready to send
  }
  if(walkMode == 1){
    walking(); //begin walking
  }
  ACK_DATA = Comp("ACK!"); // when rpi recieved the data i send and he request me to send me any data that have not been sent
  if(ACK_DATA == 0){
    //sendFlag = 1;     
    Serial.println("NICHOLAS ANT BRAIN");
    if((stepTaken - sendIndex)!= 1){   // if some data has not been send... it will send the data. cause im using stepTaken and sendIndex to keep track which bearing buffer slot am i sending.
      sendIndex++;                     // so if the stepTaken is 3, it means that 0,1,2 of the buffer is filled. and if sendIndex is 0. the arduino have to send 1 and 2.  
      if(sendIndex == 20){             // so... after the sendIndex hit 2 which means the arduino has sent 0,1,2 and the stepTaken is still 3. it means that buffer[3] is still empty.
        sendIndex = 0;                 // hence dont send. therefore i used ((stepTaken - sendIndex)!=1) as the condition.
      } 
      sendBearings();
    }
    ACK_DATA = 1;
  }
  NACK = Comp("NACK");
  if(NACK == 0){
    Serial.println("NACK");
    sendBearings();
    NACK = 1;
  }
}
