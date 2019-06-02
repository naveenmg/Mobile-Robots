#include <AFMotor.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define trigPinr A0
#define echoPinr A1
#define trigPinl A2
#define echoPinl A3
#define trigPinf A4
#define echoPinf A5
#define MAX_DISTANCE 800

RF24 radio(42, 43);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
const byte rxAddr[6] = "00001";

char charValx[8]; 
char charValy[8];
double floatValx;
double floatValy;

typedef struct{
  
  float floatValx;
  float floatValy;
}
data1;

data1 mag1;

 AF_DCMotor motor1(1); // create motor #1 using M1 output on Motor Drive Shield, set to 1kHz PWM frequency
AF_DCMotor motor2(4); // create motor #2, using M2 output, set to 1kHz PWM frequency
String motorSet = "";
int speedSet = 0;

void setup()
{
Serial.begin (115200);
pinMode(50, OUTPUT);


digitalWrite(50, LOW);
pinMode(32, OUTPUT);
digitalWrite(32, LOW);

        pinMode(trigPinf, OUTPUT); // saída de sinal do arduino do trigger_front
        pinMode(echoPinf, INPUT);// entrada de sinal do arduino do echo_front
        
        pinMode(trigPinl, OUTPUT);// saída de sinal do arduino do trigger_front
        pinMode(echoPinl, INPUT);// entrada de sinal do arduino do echo_front        
        
        pinMode(trigPinr, OUTPUT);// saída de sinal do arduino do trigger_front
        pinMode(echoPinr, INPUT);// entrada de sinal do arduino do echo_front
       
}
void loop()
{ 

   comp();
  Radio();

  
  long time_front, time_left, time_right, rightit, leftit, front; 
  digitalWrite(trigPinf, LOW);  // é declarada as respectivas entradas e saídas de sinal do
     delayMicroseconds(2);               // sensor ultrassônico e armazenada pela variável do sensor
     digitalWrite(trigPinf, HIGH); // que converte a velocidade do som que é de 340 m/s ou  
     delayMicroseconds(5);               // 29 microsegundos por centímetro, como o sinal vai e volta
     digitalWrite(trigPinf, LOW);  // esse tempo é a metade sendo sensor= tempo/29/2 ;
     time_front = pulseIn(echoPinf, HIGH); // assim segue também nos outros dois sensores .
     front = time_front/29/2;
     digitalWrite(trigPinl, LOW);
     delayMicroseconds(2);
     digitalWrite(trigPinl, HIGH);
     delayMicroseconds(5);
     digitalWrite(trigPinl, LOW); 
   time_left = pulseIn(echoPinl, HIGH);
     leftit = time_left/29/2; 
      digitalWrite(trigPinr, LOW);
     delayMicroseconds(2);
     digitalWrite(trigPinr, HIGH);
     delayMicroseconds(5);
     digitalWrite(trigPinr, LOW); 
     time_right = pulseIn(echoPinr, HIGH);
     rightit =time_right/29/2; 
 if(rightit<30)
 {
     if(front >10) // If if there is free way to front it follows this logic below
         {  
           // Use the four if's down if within that are to control the robot rightigibilidade ,
           // To keep it following the righteita wall straight
           if(rightit >10 && rightit< 20) //If the distance to the wall righteita is between 9 and 12 cm , the robot
                                         // Keep straight;
                                          { moveStop();           
        moveForward();                                                    
       delay(100);
       }
             if(rightit>=20)
             {moveStop();
              moveForwardr();
      delay(100);
             }
             if(rightit<=10)
             {moveStop();
              moveForwardl();
           delay(100);
             }
            }  
         }
         else{moveStop(); turnRight();}
         if(leftit <=20 && rightit>20 && front <=10){
         moveStop();turnRight();
         }
         
         if(leftit >20 && rightit>20 && front <=10){
         moveStop();turnRight();
         }
         
         if(rightit<=20 && leftit>20 && front <=10) {
        moveStop();turnLeft();
         }
         
         
         if(rightit<=20 && leftit<=20 && front<=10) {
        moveStop(); moveBackward();
         }
 
}
//-------------------------------------------------------------------------------------------------------------------------------------
void moveStop() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);}  // stop the motors.
//-------------------------------------------------------------------------------------------------------------------------------------
void moveForward() {
    motorSet = "FORWARD";
    motor1.run(FORWARD);      // turn it on going forward
    motor2.run(FORWARD);      // turn it on going forward
    motor1.setSpeed(100);
    motor2.setSpeed(100);
}
void moveForwardl() {
    motorSet = "FORWARD";
    motor1.run(FORWARD);      // turn it on going forward
    motor2.run(FORWARD);      // turn it on going forward
    motor1.setSpeed(103);
    motor2.setSpeed(100);
}
void moveForwardr() {
    motorSet = "FORWARD";
    motor1.run(FORWARD);      // turn it on going forward
    motor2.run(FORWARD);      // turn it on going forward
    motor1.setSpeed(100);
    motor2.setSpeed(103);
}
//---------------------------------------------------------------------------

void moveBackward() {
    motorSet = "BACKWARD";
    motor1.run(FORWARD);      // turn motor 1 forward
  motor2.run(BACKWARD);     // turn motor 2 backward
  delay(600); // run motors this way for 400ms
    motor1.setSpeed(100);
    motor2.setSpeed(100);
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnRight() {
  motorSet = "RIGHT";
  motor1.run(FORWARD);      // turn motor 1 forward
  motor2.run(BACKWARD);     // turn motor 2 backward
   motor1.setSpeed(100);
    motor2.setSpeed(100);
  delay(300); // run motors this way for 400ms
  motorSet = "FORWARD";
  motor1.run(FORWARD);      // set both motors back to forward
  motor2.run(FORWARD);  
      
}  
//-------------------------------------------------------------------------------------------------------------------------------------
void turnLeft() {
  motorSet = "LEFT";
  motor1.run(BACKWARD);     // turn motor 1 backward
  motor2.run(FORWARD); // turn motor 2 forward
   motor1.setSpeed(100);
    motor2.setSpeed(100);
  delay(300); // run motors this way for 400ms
  motorSet = "FORWARD";
  motor1.run(FORWARD);      // turn it on going forward
  motor2.run(FORWARD);      // turn it on going forward
    
}  

 void Radio()
  {
    pinMode(50, OUTPUT);
    digitalWrite(50, HIGH);
    delay(100);
    
    radio.begin();
    
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);
    radio.setRetries(15, 15);
    radio.openWritingPipe(rxAddr[0]);
    radio.openReadingPipe(1, rxAddr[1]);
    radio.stopListening();
    radio.powerUp();
    radio.write(&mag1, sizeof(mag1));
    
    Serial.println();
    Serial.print(" FLOAT VALUE OF X  :     ");
    Serial.println(floatValx);
    Serial.println();
    Serial.print(" FLOAT VALUE OF Y  :     ");
    Serial.println(floatValy);
    Serial.println();
    delay(100);
    pinMode(50, OUTPUT);
    digitalWrite(50, LOW);

 }

 void comp()
 {
    pinMode(32, OUTPUT);
    digitalWrite(32, HIGH);
   delay(100);
 if(!mag.begin())
  {
        Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
        while(1);
  }
  

  
  sensors_event_t event; 
  mag.getEvent(&event);
  mag1.floatValx=event.magnetic.x;
  mag1.floatValy=event.magnetic.y;
  Serial.println(); 
  Serial.print("x   :  ");
  Serial.print(floatValx);  
  Serial.println();
  Serial.print("y   :  ");
  Serial.print(floatValy);
  Serial.println();
  Serial.println();
   
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);
 
  delay(200);
 }

