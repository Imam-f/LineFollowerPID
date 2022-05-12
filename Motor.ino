
#include <SoftwareSerial.h>
#include <QMC5883LCompass.h>          // Wire.h Included
QMC5883LCompass compass;

// Motor Driver
#define ENA 5          // deklarasi pin ENA
#define IN1 4          // deklarasi pin IN1
#define IN2 7          // deklarasi pin IN2
#define IN3 2          // deklarasi pin IN3
#define IN4 3          // deklarasi pin IN4
#define ENB 6          // deklarasi pin ENB

// Shift Register for 5 IR Sensor
#define _Q7 8          // deklarasi pin Not Serial Input
#define  CP 9          // deklarasi pin Clock Pulse
#define _PL 10         // deklarasi pin Not Load / Shift
#define _CE 11         // deklarasi pin Not Clock Enable

// Ultrasonic Sensor 
#define ECHO 12        // deklarasi pin Echo
#define TRIG 13        // deklarasi pin Trigger

// I2C Connection and Other IR Sensor 
#define ir_pin_FL  A0  // deklarasi pin Front Left
#define ir_pin_BL  A1  // deklarasi pin Back Left
#define ir_pin_FR  A2  // deklarasi pin Front Right
#define ir_pin_BR  A3  // deklarasi pin Back Right
#define SDA        A4  // deklarasi pin SDA
#define SCL        A5  // deklarasi pin SCL
#define ir_pin_L   A6  // deklarasi pin Left
#define ir_pin_R   A7  // deklarasi pin Right

//---------------------------------

// Initial Values and Other Constants
#define init_v_L  160         // rightMotor mid. = 135 - 40(diff.) = 95 
#define init_v_R  160         // 255(max.) - 120(min.) / 2 = 135 + 15(offs.)
#define line_th   500         // Batas Analog mendeteksi line
#define line_th_D 300         // Batas Analog antara sensor gelap - sensor terang
#define Goal      25          // State Goal 
#define obs_dist  10          // Distance to Obstacle (cm)
#define motor_dir 0           // Default Direction of Motor

#define addr_MPU  0x0D        // Address IMU
#define addr_ESP  0x12        // Address ESP Cam

void setup() {
  Serial.begin(9600);               // Open serial communications and wait for port to open:9600
  while (!Serial) {
    ;                               // wait for serial port to connect. Needed for Native USB only
  }
  Serial.println("Ready");
         
  Serial.println("CLEARDATA");      // Data Logging
  Serial.println("LABEL,Date,Time,Error");
  Serial.println("RESETTIMER");

  Wire.begin();                     // Create Wire Object
  
  // Konfigurasi pin-pin sebagai Output  
  pinMode(ir_pin_L, INPUT);         // Line Sensor Depan        
  pinMode(ir_pin_R, INPUT);
  
  pinMode(ir_pin_FL, INPUT);        // Line Sensor Kiri
  pinMode(ir_pin_BL, INPUT);
  
  pinMode(ir_pin_FR, INPUT);        // Line Sensor Kanan
  pinMode(ir_pin_BR, INPUT);

  pinMode(TRIG, OUTPUT);            // Ultrasonic
  pinMode(ECHO, INPUT);

  pinMode(_Q7, INPUT);              // Shift Register
  pinMode(CP, OUTPUT);
  pinMode(_PL, OUTPUT);
  pinMode(_CE, OUTPUT);

  for (int i = 2; i < 8; i++) {     // Motor Driver
    pinMode(i, OUTPUT);
  }
  
  compass.init();                   // IMU Sensor 
}

// Global Variable
//uint8_t state;
int ir_val_L, ir_val_R;             // Sensors Reading
int ir_val_LL[2], ir_val_RR[2];
int prev_b;
byte ret = B00000;     

float error, errorLast, erroInte;   // PID Parameter
int err, ctrl_sig;
float Kp = 20;
float Ki = 10;
float Kd = 0;
float errorDiff;
float output;

int motor_v_L;                      // Motor Speed
int motor_v_R;

int state, action;                  // Current State and Action

unsigned long oldTime;              // Time measure

void loop() {
  moveCar();
  isNode();
//  Serial.print("DATA, DATE, TIMER, ");
//  Serial.print(error);
//  Serial.println(" ,");

//  Default Condition Robot Always Moving
  readPYNQ(state,action);
  if (state != Goal) {
    if (action == 0) {
      moveCar();
      Serial.println("\nmoveCar\n");
    }
    else if (action == 1) {
      rotate(0);
      moveCar();
      Serial.println("\nKanan\n");
    }
    else if (action == 2) {
      Serial.println("\nBelakang\n");
      rotate(0);
      rotate(0);
      moveCar();
    }
    else {
      Serial.println("\nKiri\n");
      rotate(1);
      moveCar();
    }
  } else {
    Serial.println("GOAL REACHED!");
  }
  Serial.println(state);
  delay(50);
}

void moveCar() {
  //readSensor(ir_pin_L, ir_pin_R, &ir_val_L, &ir_val_R);  // 2 IR front sensor
  //-----------------------------------------------------------------------          
  readShiftReg(&ret);               // 5 IR front sensor
  
  //Serial.println(ret,BIN);
  //delay(1000);
  //-----------------------------------------------------------------------
  switch (ret) {
    case B00000:
      err = 2* err;
      break;

    case B00010:
    case B00110:
      err = 1;
      break;

    case B00001:
    case B00011:
    case B00111:
      err = 2;
      break;

    case B00100:
    case B01110:
      err = 0;
      break;

    case B01000:
    case B01100:
      err = -1;
      break;

    case B10000:
    case B11000:
    case B11100:
      err = -2;
      break;

    case B01111:                     // Robot Meets Node
    case B11110:  
    case B11111:
      //isNode(state);
      break;
      
    default:
      err = err;
      break;
  }
  //-----------------------------------------------------------------------
  
  ctrl_sig = calculatePID(err);      // Run Motor
  motor_v_L = init_v_L - ctrl_sig;
  motor_v_R = init_v_R + ctrl_sig;
  motorWrite(motor_dir, motor_dir, motor_v_L, motor_v_R);
  delay(5);
}

int calculatePID(int err) {
  unsigned long dt = calculateDeltaTime();
  error = error * 0.7 + err * 0.3;   // Low Pass filter to smoothens Digital Reading
  //error = err;
  errorDiff = error - errorLast;
  //Serial.println(errorDiff);
  erroInte = constrain(erroInte + error, -100, 100);
  //Serial.println(erroInte);
  
  Serial.println("-------------------");

  output = Kp * error + Ki * erroInte * dt + Kd * errorDiff / dt;
  Serial.println(output);

  errorLast = error;
  return output;
}

unsigned long calculateDeltaTime(){
  unsigned long currentTime = millis();
  unsigned long deltaTime = currentTime - oldTime;
  oldTime = currentTime;
  return deltaTime;
}

void readSensor(char pin1, char pin2, int*pin1Val, int*pin2Val) {
  *pin1Val = analogRead(pin1);
  *pin2Val = analogRead(pin2);
}

void readShiftReg(byte *ret){
  // Write pulse to _PL pin
  digitalWrite(_PL, LOW);
  delayMicroseconds(5);
  digitalWrite(_PL, HIGH);
  delayMicroseconds(5);
  
  // Get data from 74HC165
  digitalWrite(CP, HIGH);
  digitalWrite(_CE, LOW);
  byte  incoming = shiftIn(_Q7, CP, MSBFIRST);
  byte  mask = B00011111;
  digitalWrite(_CE, HIGH);
  *ret = (incoming & mask);
}

void motorWrite(bool en_forwardLeft, bool en_forwardRight, int motor_v_L, int motor_v_R) {
  digitalWrite(IN1, en_forwardLeft);
  digitalWrite(IN2, !en_forwardLeft);
  analogWrite(ENA, constrain(motor_v_L, 95, 250));     // Won't exceed 250 in speed

  digitalWrite(IN4, en_forwardRight);
  digitalWrite(IN3, !en_forwardRight);
  analogWrite(ENB, constrain(motor_v_R, 120, 250));
}


void rotate(bool CW) {
  while (!isRotated(CW)) {          
    motorWrite(CW, !CW, 250, 250);
    delay(5);
  }
}

bool isRotated (bool CW){                
  int a, b, b_trgt;
  if (!CW) b_trgt = (prev_b + 4)%16;    // Clockwise
  else b_trgt = (prev_b - 4)%16;        // Counter Clockwise
  
  if (b_trgt < 0) b_trgt += 16;         // Avoid Negative Bearing
  compass.read();
  a = compass.getAzimuth();
  b = compass.getBearing(a);            // Direction Compass is Facing, 0-15
  
  if (b == b_trgt){prev_b = b; ;return 1;
  } else return 0;
}

//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------
//------------------BATAS PROGRAM YANG SUDAH SELESAI, DIBAWAH INI BELUM---------------------
//------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------

int isNode() {
  bool isNode;
  // Check If It Meets Node
  readSensor(ir_pin_FL, ir_pin_FR, &ir_val_LL[0], &ir_val_RR[0]);
  isNode = ir_val_LL[0] + ir_val_RR[0] > 1000;
  Serial.println(ir_val_LL[0]);
  Serial.println(ir_val_RR[0]);

  Serial.println(isNode);
  if (isNode) {
    motorWrite(0, 0, 0, 0);
    int adjustMode = 0;             // 0: Too Front/Behind, 1: Too Right/Left
    while (!readQR()){
      adjustCar(adjustMode);
      adjustMode = !adjustMode;
    }
  }
  return 1;
}

void adjustCar(int adjustMode) {
  int angle, sensorSummed, ctrl_sig, motorDirection;
  int del, motor_dir_t;

  Serial.println("\n adjustCar \n");
  if (!adjustMode){                     // 0: Too Front/Behind For reading QR
    
    readSensor(ir_pin_FL, ir_pin_BL, &ir_val_LL[0], &ir_val_LL[1]);        
    if(ir_val_LL[0] + ir_val_LL[1] > line_th){
        if(ir_val_LL[0] > line_th){      // Left Too Behind
          motor_dir_t = 1;
        }
        else if(ir_val_LL[1] > line_th){ // Left Too Front
          motor_dir_t = 0;
        }
        motorWrite(!motor_dir_t, motor_dir_t, init_v_L, 0);
    }
    
   readSensor(ir_pin_FR, ir_pin_BR, &ir_val_RR[0], &ir_val_RR[1]);
   if(ir_val_RR[0] + ir_val_RR[1] > line_th){
        if(ir_val_RR[0] > line_th){      // Right Too Behind
          motor_dir_t = 1;
        }
        else if(ir_val_LL[1] > line_th){ // Right Too Front
          motor_dir_t = 0;
        }
        motorWrite(motor_dir_t, !motor_dir_t, 0, init_v_R);
    }
    delay(100);
  } 
  
   else {                         // 1: Too Right/Left, Aligning with Main Line
    byte ret = B00000;            // Front IR sensor 
    readShiftReg(&ret);

    if(ret == B00100){
      Serial.println("Aligned w/ Main Line");
    }else{
      switch (ret) { 
    
        case B00001:
        case B00011:
        case B00111:
          del = 200;
          motor_dir_t = 1;            // Kanan Maju
          break;
             
        case B00010:
        case B00110:
          del = 100;
          motor_dir_t = 1;            // Kanan Maju
          break;
    
        case B01000:
        case B01100:
          del = 100;
          motor_dir_t = 0;            // Kiri Maju
          break;
    
        case B10000:
        case B11000:
        case B11100:
          del = 200;
          motor_dir_t = 0;            // Kiri Maju
          break;
          
        default:
          del = 0;
          break;
      }
      motorWrite(motor_dir_t, !motor_dir_t, init_v_L, init_v_R);
      delay(del);
    }
  }
  motorWrite(0, 0, 0, 0);            // Stop Adjusting
}

bool isObstacle(){
  long duration; int distance;
  digitalWrite(TRIG, LOW);           // Clears the trigPin condition
  delayMicroseconds(2);
  
  digitalWrite(TRIG, HIGH);          // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  duration = pulseIn(ECHO, HIGH);    // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;   // Speed of sound wave divided by 2 (go and back)
  
  Serial.print("Distance: ");        // Displays the distance on the Serial Monitor
  Serial.print(distance);
  Serial.println(" cm");

  if (distance < obs_dist) return HIGH; else return LOW;
}

int readQR() {
    int doneRead, reading, times_reading;
        Wire.beginTransmission(addr_ESP);
        Wire.write(byte(0x00));                  // Start with register[3].
        Wire.endTransmission();

        Wire.requestFrom(addr_ESP, 2);
        while (!doneRead && times_reading < 4) { // Limit QR Reading only 4 times
            delay(50);
            if(Wire.available() >= 2) {
                doneRead = Wire.read();          // register[0]
                reading = Wire.read();           // register[1]
            }
        }
    state = reading;
    return doneRead;
}

int readPYNQ(int state, int action){
    int a = 0;
    int s = 0;
    int overflow = 0;
    char userInput[50];
    
    if (Serial.available() > 0) {
        
        String command;
        overflow = 1;

        for (int i = 0; i < 50; i++) {
            while(Serial.available() < 1) {}
            userInput[i] = Serial.read();
            if(userInput[i] == '\n' || userInput[i] == '\0') {
                overflow = 0;
                break;
            }
        }

        if(overflow == 1) {
            return;
        }
        command = String(userInput);

        if (command.startsWith("get")) {
            command.remove(0, 4);
            if(command.startsWith("internal")) {
                command.remove(0, 9);
                Serial.println("internal working");
            } else if(command.startsWith("state")) {
                command.remove(0, 6);
                Serial.println(s);
            }
        } else if (command.startsWith("set")) {
            command.remove(0, 4);
            if(command.startsWith("action")) {
                command.remove(0, 7);
                a = (int)command[0] + 1 - (int)'1';

                //////////////
                int nextstate, isValid;
                if (a == 0)
                    nextstate = s - 5;
                else if (a == 1)
                    nextstate = s + 1;
                else if (a == 2)
                    nextstate = s + 5;
                else
                    nextstate = s - 1;

                if (nextstate < 0)
                    isValid = 0;
                else if (nextstate > 24)
                    isValid = 0;
                else if ((s % 5 == 4) && (a == 1))
                    isValid = 0;
                else if ((s % 5 == 0) && (a == 3))
                    isValid = 0;
                else
                    isValid = 1;

                if (isValid && s != 24) {
                    s = nextstate;
                }
                /////////////
                Serial.print("set a");
                Serial.println(a);
            }
            if(command.startsWith("state")) {
                command.remove(0, 6);
                s = (int)command[0] + 1 - (int)'1';
                Serial.print("set s");
                Serial.println(s);
            }
        } else if (command.startsWith("g_status")) {
            Serial.println("[success] ");
        } else {
            Serial.flush();
        }
        delay(500);
    }
    state = s;
    action = a;
}

void printSensor(int ir_val_L, int ir_val_R, int ir_val_LL0, 
                 int ir_val_LL1, int ir_val_RR0, int ir_val_RR1){
    Serial.print("Left Front = ");
    Serial.println(ir_val_L);
    Serial.print("Right Front = ");
    Serial.println(ir_val_R);
    Serial.print("Front Left = ");
    Serial.println(ir_val_LL0);
    Serial.print("Back Left = ");
    Serial.println(ir_val_LL1);
    Serial.print("Front Right = ");
    Serial.println(ir_val_RR0);
    Serial.print("Back Right = ");
    Serial.println(ir_val_RR1);
    Serial.println();
    Serial.println();
}
