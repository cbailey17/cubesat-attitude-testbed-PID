#include <Wire.h>
#include <Adafruit_MotorShield.h>
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
Adafruit_DCMotor *M2 = AFMS.getMotor(2);
Adafruit_DCMotor *M3 = AFMS.getMotor(3);
Adafruit_DCMotor *M4 = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);
#define ENCODER_A 3 
#define ENCODER_B 4

long int rightCount = 0;
double angle=0;
unsigned long time;
//unsigned long time0;
//unsigned long time1;
//unsigned long time2;
int V1=150; int V2; int V3; int V4; double long i=0;
String inString;
int A[11];
int m1=250;
int m2=250;
int m3=250;
int m4=250;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial3.begin(115200);
  Serial3.flush();
  Serial3.setTimeout(25);
  AFMS.begin();  // create with the default frequency 1.6KHz ,AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  delay(2000);
  // Set the speed to start, from -255 to 255
  M1->setSpeed(0); M2->setSpeed(0); M3->setSpeed(0); M4->setSpeed(0);
  M1->run(FORWARD); M2->run(FORWARD); M3->run(FORWARD); M4->run(FORWARD);
 
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);

  // initialize hardware interrupts
//  attachInterrupt(0, leftEncoderEvent, CHANGE);
  attachInterrupt(1, rightEncoderEvent, CHANGE);
  M1->run(RELEASE);
}

void loop() { 
  Parser();
 Serial.println("---------------------");
  Serial.println("Motor torques");
  Serial.print(m1);
  Serial.print(m2);
//  Serial.print(m3);
//  Serial.println(m4);
  V1=m1-250;// the control input to motor 1
  V2=m2-250;// the control input to motor 2
  V3=m3-250;// the control input to motor 3
  V4=m4-250;// the control input to motor 4
//  Serial.println("---------------------");
//  Serial.println("Velocities");
//  Serial.print(V1);
//  Serial.print(V2);
//  Serial.print(V3);
//  Serial.println(V4);
  
  MotorsInputV(V1,V2,V3,V4);
  delay(1000);
}

int MotorsInputV(int v1,int v2,int v3,int v4){
  if (abs(v1) == v1) {
    M1->run(FORWARD);
    M1->setSpeed(v1);
    } else {
    M1->run(BACKWARD);
    M1->setSpeed(abs(v1));
    }
      if (abs(v2) == v2) {
  M2->run(FORWARD);
  M2->setSpeed(v2);
    } else {
  M2->run(BACKWARD);
  M2->setSpeed(abs(v2));
    }
      if (abs(v3) == v3) {
  M3->run(FORWARD);
  M3->setSpeed(v3);
    } else {
  M3->run(BACKWARD);
  M3->setSpeed(abs(v3));
    }
      if (abs(v4) == v4) {
  M4->run(FORWARD);
  M4->setSpeed(v4);
    } else {
  M4->run(BACKWARD);
  M4->setSpeed(abs(v4));
    }
}

void rightEncoderEvent() {
  if (digitalRead(ENCODER_A) == HIGH) {
    if (digitalRead(ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
  //angle=rightCount/105.6*2*22/7;
}

static inline int8_t sgn(int val) {
 if (val < 0) return -1;
 if (val==0) return 0;
 return 1;
}

void Parser(){
  if (Serial3.available()) {
    inString = Serial3.readString();
    Serial.println("-------------------");
    Serial.println(inString);
   if(inString[0]=='#'){
    A[0]=inString[3]-'0';
    A[1]=(inString[2]-'0')*10;
    A[2]=(inString[1]-'0')*100;
    m1=A[0]+A[1]+A[2];

    A[3]=inString[7]-'0';
    A[4]=(inString[6]-'0')*10;
    A[5]=(inString[5]-'0')*100;
    m2=A[3]+A[4]+A[5];
        
    A[6]=inString[11]-'0';
    A[7]=(inString[10]-'0')*10;
    A[8]=(inString[9]-'0')*100;
    m3=A[6]+A[7]+A[8];

    A[9]=inString[15]-'0';
    A[10]=(inString[14]-'0')*10;
    A[11]=(inString[13]-'0')*100;
    m4=A[9]+A[10]+A[11];
    Serial3.flush();
    delay(100);
   }
  }

}
