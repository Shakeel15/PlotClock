// Plot Thermometer & Clock
//A different approach to Plotclock v1.3
//@author: Shakeel

// units: mm; microseconds; radians
// origin: bottom left of drawing surface
// all libraries see http://playground.arduino.cc
// V1.1 added support for Motion (PIR) sensor or tactile switch to avoid ploting if not required
// Bug Fix in calibration
#include <EEPROMEx.h>
#include <Streaming.h>
#include <Time.h> 
#include <Servo.h>
#include <OneWire.h> // for temp measurement

boolean CALIBRATION;   // enable calibration mode
int address_CALIBRATION;

#define REALTIMECLOCK    // enable real time clock
#define SERVOPINLIFT  2
#define SERVOPINLEFT  3
#define SERVOPINRIGHT 4

#define Temperaturepin 5
OneWire  ds(Temperaturepin);
boolean showTemp;
int address_showTemp;

#define auxPin 7
boolean auxActive = false;
boolean auxHaveBeenActive = false;

#define switchPin 6
#define ledPin 13
boolean switchFunctionPlotOn;
int address_switchFunctionPlotOn;
boolean lastSwitchStatus;
boolean flipPlot;
int address_flipPlot;

// When in calibration mode, adjust the following factor until the servos move exactly 90 degrees
int SERVOFAKTORLEFT = 650;
int SERVOFAKTORRIGHT = 650;

int address_SERVOFAKTORLEFT;
int address_SERVOFAKTORRIGHT;

// Zero-position of left and right servo
// When in calibration mode, adjust the NULL-values so that the servo arms are at all times parallel
// either to the X or Y axis
int SERVOLEFTNULL = 2250;
int SERVORIGHTNULL = 920;

int address_SERVOLEFTNULL;
int address_SERVORIGHTNULL;

// lift positions of lifting servo
int LIFT0  = 1800; //1080 // on drawing surface
int LIFT1 = 1645; // 925  // between numbers
int LIFT2 = 1445; //725  // going towards sweeper

int address_LIFT0;
int address_LIFT1;
int address_LIFT2;

// speed of liftimg arm, higher is slower
#define LIFTSPEED 1500

// length of arms
#define L1 35
#define L2 55.1
#define L3 13.2

// origin points of left and right servo 
#define O1X 22
#define O1Y -25
#define O2X 47
#define O2Y -25


#ifdef REALTIMECLOCK
// for instructions on how to hook up a real time clock,
// see here -> http://www.pjrc.com/teensy/td_libs_DS1307RTC.html
// DS1307RTC works with the DS1307, DS1337 and DS3231 real time clock chips.
// Please run the SetTime example to initialize the time on new RTC chips and begin running.

#include <Wire.h>
#include <DS1307RTC.h> // see http://playground.arduino.cc/Code/time    
#endif

int servoLift = 1500;

Servo servo1;  // 
Servo servo2;  // 
Servo servo3;  // 

volatile double lastX = 75;
volatile double lastY = 47.5;

int last_min = 0;

// Setup procedure
int a;
int cal_step = 10;
// calibration modes
int cal_mode = 99;
#define right_servo_factor 1
#define left_servo_factor 2
#define right_servo_0_position 3
#define left_servo_0_position 4
#define lift0_position 5
#define lift1_position 6
#define lift2_position 7

void setup() 
{ 
  EEPROM.setMemPool(100, EEPROMSizeUno);

  address_SERVOFAKTORLEFT = EEPROM.getAddress(sizeof(int));
  address_SERVOFAKTORRIGHT = EEPROM.getAddress(sizeof(int));
  address_SERVOLEFTNULL = EEPROM.getAddress(sizeof(int));
  address_SERVORIGHTNULL = EEPROM.getAddress(sizeof(int));
  address_LIFT0 = EEPROM.getAddress(sizeof(int));
  address_LIFT1 = EEPROM.getAddress(sizeof(int));
  address_LIFT2 = EEPROM.getAddress(sizeof(int));
  address_CALIBRATION = EEPROM.getAddress(sizeof(byte));
  address_showTemp = EEPROM.getAddress(sizeof(byte));
  address_switchFunctionPlotOn = EEPROM.getAddress(sizeof(byte));
  address_flipPlot = EEPROM.getAddress(sizeof(byte));

  SERVOFAKTORLEFT = EEPROM.readInt(address_SERVOFAKTORLEFT);
  SERVOFAKTORRIGHT = EEPROM.readInt(address_SERVOFAKTORRIGHT);
  SERVOLEFTNULL = EEPROM.readInt(address_SERVOLEFTNULL);
  SERVORIGHTNULL = EEPROM.readInt(address_SERVORIGHTNULL);
  LIFT0  = EEPROM.readInt(address_LIFT0);
  LIFT1 = EEPROM.readInt(address_LIFT1);
  LIFT2 = EEPROM.readInt(address_LIFT2);
  CALIBRATION = EEPROM.read(address_CALIBRATION);
  showTemp = EEPROM.read(address_showTemp);
  switchFunctionPlotOn = EEPROM.read(address_switchFunctionPlotOn);
  flipPlot = EEPROM.read(address_flipPlot);

  pinMode(switchPin,INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(auxPin,INPUT_PULLUP);
  Serial.begin(9600);

  printStatus();

#ifdef REALTIMECLOCK
  //  Serial.begin(9600);
  //while (!Serial) { ; } // wait for serial port to connect. Needed for Leonardo only

  // Set current time only the first to values, hh,mm are needed  
  tmElements_t tm;
  if (RTC.read(tm)) 
  {
    setTime(tm.Hour,tm.Minute,tm.Second,tm.Day,tm.Month,tm.Year);
    Serial.println("DS1307 time is set OK.");
  } 
  else 
  {
    if (RTC.chipPresent())
    {
      Serial.println("DS1307 is stopped.  Please run the SetTime example to initialize the time and begin running.");
    } 
    else 
    {
      Serial.println("DS1307 read error!  Please check the circuitry.");
    } 
    // Set current time only the first to values, hh,mm are needed
    setTime(19,38,0,0,0,0);
  }
#else  
  // Set current time only the first to values, hh,mm are needed
  setTime(19,38,0,0,0,0);
#endif

  drawTo(75.2, 47);
  lift(0);
  servo1.attach(SERVOPINLIFT);  //  lifting servo
  servo2.attach(SERVOPINLEFT);  //  left servo
  servo3.attach(SERVOPINRIGHT);  //  right servo
  delay(1000);

  lastSwitchStatus = digitalRead(switchPin);
  if (!switchFunctionPlotOn) showTemp = lastSwitchStatus;

  servo1.detach();
  servo2.detach();
  servo3.detach(); 

} 

void loop() { 

  if (lastSwitchStatus != digitalRead(switchPin)) {
    lastSwitchStatus = !lastSwitchStatus;
    if (!switchFunctionPlotOn) showTemp = lastSwitchStatus;
    printStatus();
  }
  if (digitalRead(auxPin)) auxActive = true;
  if (Serial.available()) 
    a = Serial.read();
  else
    a = 0;
  if (char(a) == 'x') {
    CALIBRATION = !CALIBRATION;
    EEPROM.write(address_CALIBRATION,CALIBRATION);
    printStatus();
    if (!CALIBRATION) {
      servo1.detach();
      servo2.detach();
      servo3.detach(); 
    }
    cal_mode = 99;
  }
  if (CALIBRATION) {
    if (cal_mode == 99) {
      calibration_instructions();  
      cal_mode = 0;
    }
    setup_procedure(a, cal_step, cal_mode);
  }
  else {
      if (!(switchFunctionPlotOn && !digitalRead(switchPin))) {
        if (last_min != minute()) {
          if (auxActive)  {
            if (!showTemp) {
              if (!flipPlot) writeTime(5,19,28,34,48,25,0.9);
              else writeTime(62,48,38,33,19,43,-0.9);
            }
            else {
              if (!flipPlot) writeTemperature(5,19,28, 34,25,0.9);
              else writeTemperature(62,48,38,33,43,-0.9);
            }
            auxActive = false;
            auxHaveBeenActive = true;
          } // auxActive
          else {
            if (auxHaveBeenActive && !auxActive)  {  
              if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
              if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
              if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);
              number(3, 3, 111, 1);
              lift(1);
              servo1.detach();
              servo2.detach();
              servo3.detach(); 
              auxHaveBeenActive = false;
            } //auxHaveBeenActive && !auxActive
          } // else auxActive
        }// if new minute
      }
  } //else 
} // main

void writeTemperature(int apo, int bpo, int cpo, int dpo, int offspo, float scale)  {
  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
  if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

  float tempFloat = temperature();
  tempFloat = tempFloat * 10;
  int temp = tempFloat;
  int temp2 = temp / 100;
  int temp1 = (temp - temp2 * 100) / 10;
  int temp0 = temp - temp2 * 100 - temp1 * 10;

  lift(0);
  number(3, 3, 111, 1);
  number(apo,offspo,temp2,scale);
  number(bpo,offspo,temp1,scale);
  number(cpo,offspo, 12,scale);  
  number(dpo,offspo,temp0,scale);
  lift(2);
  drawTo(74.2, 47.5);
  lift(1);
  servo1.detach();
  servo2.detach();
  servo3.detach(); 

  last_min = minute();

} //writeTemperature

void writeTime(int apo, int bpo, int cpo, int dpo, int epo, int offspo, float scale) {
  int i = 0;

  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
  if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

  lift(0);

  hour();
  while ((i+1)*10 <= hour())
  {
    i++;
  }
  number(3, 3, 111, 1);
  number(apo,offspo,i,scale);
  number(bpo,offspo,(hour()-i*10),scale);

  number(cpo,offspo, 11,scale);  

  i=0;
  while ((i+1)*10 <= minute())
  {
    i++;
  }
  number(dpo,offspo,i,scale);
  number(epo,offspo, (minute()-i*10),scale);

  lift(2);
  drawTo(74.2, 47.5);
  lift(1);
  last_min = minute();

  servo1.detach();
  servo2.detach();
  servo3.detach(); 
}  // writeTime

// Writing numeral with bx by being the bottom left originpoint. Scale 1 equals a 20 mm high font.
// The structure follows this principle: move to first startpoint of the numeral, lift down, draw numeral, lift up
void number(float bx, float by, int num, float scale) {

  switch (num) {

  case 0:
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(0);
    bogenGZS(bx + 7 * scale, by + 10 * scale, 10 * scale, -0.8, 6.7, 0.5);
    lift(1);
    break;
  case 1:

    drawTo(bx + 3 * scale, by + 15 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(1);
    break;
  case 2:
    drawTo(bx + 2 * scale, by + 12 * scale);
    lift(0);
    bogenUZS(bx + 8 * scale, by + 14 * scale, 6 * scale, 3, -0.8, 1);
    drawTo(bx + 1 * scale, by + 0 * scale);
    drawTo(bx + 12 * scale, by + 0 * scale);
    lift(1);
    break;
  case 3:
    drawTo(bx + 2 * scale, by + 17 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 3, -2, 1);
    bogenUZS(bx + 5 * scale, by + 5 * scale, 5 * scale, 1.57, -3, 1);
    lift(1);
    break;
  case 4:
    drawTo(bx + 10 * scale, by + 0 * scale);
    lift(0);
    drawTo(bx + 10 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 6 * scale);
    drawTo(bx + 12 * scale, by + 6 * scale);
    lift(1);
    break;
  case 5:
    drawTo(bx + 2 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 6 * scale, 6 * scale, -2.5, 2, 1);
    drawTo(bx + 5 * scale, by + 20 * scale);
    drawTo(bx + 12 * scale, by + 20 * scale);
    lift(1);
    break;
  case 6:
    drawTo(bx + 2 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 6 * scale, 6 * scale, 2, -4.4, 1);
    drawTo(bx + 11 * scale, by + 20 * scale);
    lift(1);
    break;
  case 7:
    drawTo(bx + 2 * scale, by + 20 * scale);
    lift(0);
    drawTo(bx + 12 * scale, by + 20 * scale);
    drawTo(bx + 2 * scale, by + 0);
    lift(1);
    break;
  case 8:
    drawTo(bx + 5 * scale, by + 10 * scale);
    lift(0);
    bogenUZS(bx + 5 * scale, by + 15 * scale, 5 * scale, 4.7, -1.6, 1);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 5 * scale, -4.7, 2, 1);
    lift(1);
    break;

  case 9:
    drawTo(bx + 9 * scale, by + 11 * scale);
    lift(0);
    bogenUZS(bx + 7 * scale, by + 15 * scale, 5 * scale, 4, -0.5, 1);
    drawTo(bx + 5 * scale, by + 0);
    lift(1);
    break;

  case 111:

    lift(0);
    drawTo(70, 46);
    drawTo(65, 43);

    drawTo(65, 49);
    drawTo(5, 49);
    drawTo(5, 45);
    drawTo(65, 45);
    drawTo(65, 40);

    drawTo(5, 40);
    drawTo(5, 35);
    drawTo(65, 35);
    drawTo(65, 30);

    drawTo(5, 30);
    drawTo(5, 25);
    drawTo(65, 25);
    drawTo(65, 20);

    drawTo(5, 20);
    drawTo(60, 44);

    drawTo(75.2, 47);
    lift(2);

    break;

  case 11:
    drawTo(bx + 5 * scale, by + 15 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 15 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    break;

  case 12:
    drawTo(bx + 5 * scale, by + 5 * scale);
    lift(0);
    bogenGZS(bx + 5 * scale, by + 5 * scale, 0.1 * scale, 1, -1, 1);
    lift(1);
    break;

  }
}



void lift(char lift) {
  switch (lift) {
    // room to optimize  !

  case 0: //850

      if (servoLift >= LIFT0) {
      while (servoLift >= LIFT0) 
      {
        servoLift--;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT0) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }

    }

    break;

  case 1: //150

    if (servoLift >= LIFT1) {
      while (servoLift >= LIFT1) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);

      }
    } 
    else {
      while (servoLift <= LIFT1) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }

    }

    break;

  case 2:

    if (servoLift >= LIFT2) {
      while (servoLift >= LIFT2) {
        servoLift--;
        servo1.writeMicroseconds(servoLift);
        delayMicroseconds(LIFTSPEED);
      }
    } 
    else {
      while (servoLift <= LIFT2) {
        servoLift++;
        servo1.writeMicroseconds(servoLift);				
        delayMicroseconds(LIFTSPEED);
      }
    }
    break;
  }
}


void bogenUZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = -0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) > ende);

}

void bogenGZS(float bx, float by, float radius, int start, int ende, float sqee) {
  float inkr = 0.05;
  float count = 0;

  do {
    drawTo(sqee * radius * cos(start + count) + bx,
    radius * sin(start + count) + by);
    count += inkr;
  } 
  while ((start + count) <= ende);
}


void drawTo(double pX, double pY) {
  double dx, dy, c;
  int i;

  // dx dy of new point
  dx = pX - lastX;
  dy = pY - lastY;
  //path lenght in mm, times 4 equals 4 steps per mm
  c = floor(4 * sqrt(dx * dx + dy * dy));

  if (c < 1) c = 1;

  for (i = 0; i <= c; i++) {
    // draw line point by point
    set_XY(lastX + (i * dx / c), lastY + (i * dy / c));

  }

  lastX = pX;
  lastY = pY;
}

double return_angle(double a, double b, double c) {
  // cosine rule for angle between c and a
  return acos((a * a + c * c - b * b) / (2 * a * c));
}

void set_XY(double Tx, double Ty) 
{
  delay(1);
  double dx, dy, c, a1, a2, Hx, Hy;

  // calculate triangle between pen, servoLeft and arm joint
  // cartesian dx/dy
  dx = Tx - O1X;
  dy = Ty - O1Y;

  // polar lemgth (c) and angle (a1)
  c = sqrt(dx * dx + dy * dy); // 
  a1 = atan2(dy, dx); //
  a2 = return_angle(L1, L2, c);

  servo2.writeMicroseconds(floor(((a2 + a1 - M_PI) * SERVOFAKTORLEFT) + SERVOLEFTNULL));

  // calculate joinr arm point for triangle of the right servo arm
  a2 = return_angle(L2, L1, c);
  Hx = Tx + L3 * cos((a1 - a2 + 0.621) + M_PI); //36,5°
  Hy = Ty + L3 * sin((a1 - a2 + 0.621) + M_PI);

  // calculate triangle between pen joint, servoRight and arm joint
  dx = Hx - O2X;
  dy = Hy - O2Y;

  c = sqrt(dx * dx + dy * dy);
  a1 = atan2(dy, dx);
  a2 = return_angle(L1, (L2 - L3), c);

  servo3.writeMicroseconds(floor(((a1 - a2) * SERVOFAKTORRIGHT) + SERVORIGHTNULL));

}

float temperature() {
  byte present = 0;
  byte data[12];
  byte addr[8];
  float celsius;
  int int_cel;

  ds.reset_search();
  ds.search(addr);
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1);
  delay(1000);
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);        
  for (byte i = 0; i < 9; i++) {          
    data[i] = ds.read();
  }
  unsigned int raw = (data[1] << 8) | data[0];
  byte cfg = (data[4] & 0x60);
  if (cfg == 0x00) raw = raw << 3;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw << 2; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw << 1; // 11 bit res, 375 ms
  celsius = (float)raw / 1.60;
  int_cel = celsius;
  celsius = int_cel;
  celsius = celsius/10;
  return celsius;
}

void setup_procedure(int a,  int &cal_step, int &cal_mode)  {

  if (a==0) return;

  if (!servo1.attached()) servo1.attach(SERVOPINLIFT);
  if (!servo2.attached()) servo2.attach(SERVOPINLEFT);
  if (!servo3.attached()) servo3.attach(SERVOPINRIGHT);

  // Setup mode and adjust
  switch (char(a))  {            
  case '0':
    calibration_instructions();  
    break;
  case '+':
    value_adjust(cal_mode, cal_step);
    break;
  case '-':
    value_adjust(cal_mode, -1 * cal_step);
    break;
  case '1':
    cal_mode = left_servo_0_position;
    servo2.writeMicroseconds(SERVOLEFTNULL);
    break;
  case '2':
    cal_mode = right_servo_0_position;
    servo3.writeMicroseconds(SERVORIGHTNULL);
    break;
  case '3':
    cal_mode = right_servo_factor;
    drawTo(-3, 29.2);
    break;
  case '4':
    cal_mode = left_servo_factor;
    drawTo(74.1, 28);
    break;
  case 'D':
    if (cal_step == 10) cal_step = 1; 
    else cal_step = 10;
    Serial.print("Calibrationstep = ");
    Serial.println(cal_step);
    break;
  case '8':
    lift(2);
    drawTo(75.2, 47);
    break;
  case '9':
    drawTo(30, 30);
    break;
  case '5':
    cal_mode = lift0_position;
    lift(0);
    break;
  case '6':
    cal_mode = lift1_position;
    lift(1);
    break;
  case '7':
    cal_mode = lift2_position;
    lift(2);
    break;
  case 'A':
    SERVOFAKTORLEFT = 650;
    SERVOFAKTORRIGHT = 650;
    SERVOLEFTNULL = 2250;
    SERVORIGHTNULL = 920;
    LIFT0  = 1800;
    LIFT1 = 1645;
    LIFT2 = 1445;
    break;
  case 'B':
    EEPROM.writeInt(address_SERVOFAKTORLEFT, SERVOFAKTORLEFT);
    EEPROM.writeInt(address_SERVOFAKTORRIGHT, SERVOFAKTORRIGHT);
    EEPROM.writeInt(address_SERVOLEFTNULL, SERVOLEFTNULL);
    EEPROM.writeInt(address_SERVORIGHTNULL, SERVORIGHTNULL);
    EEPROM.writeInt(address_LIFT0, LIFT0);
    EEPROM.writeInt(address_LIFT1, LIFT1);
    EEPROM.writeInt(address_LIFT2, LIFT2);
    break;
  case 'C':
    SERVOFAKTORLEFT = EEPROM.readInt(address_SERVOFAKTORLEFT);
    SERVOFAKTORRIGHT = EEPROM.readInt(address_SERVOFAKTORRIGHT);
    SERVOLEFTNULL = EEPROM.readInt(address_SERVOLEFTNULL);
    SERVORIGHTNULL = EEPROM.readInt(address_SERVORIGHTNULL);
    LIFT0  = EEPROM.readInt(address_LIFT0);
    LIFT1 = EEPROM.readInt(address_LIFT1);
    LIFT2 = EEPROM.readInt(address_LIFT2);
    break;
  case 'w':
    flipPlot = !flipPlot;
    EEPROM.write(address_flipPlot,flipPlot);
    break;
  case 'y':
    showTemp = !showTemp;
    EEPROM.write(address_showTemp,showTemp); 
    printStatus();
    calibration_instructions();      
    break;
  case 'z':
    switchFunctionPlotOn = !switchFunctionPlotOn;
    EEPROM.write(address_switchFunctionPlotOn,switchFunctionPlotOn);
    printStatus();
    calibration_instructions();      
    break;
  }
}

void value_adjust(int cal_mode, int cal_step) {

  switch (cal_mode)  {
  case right_servo_0_position:
    SERVORIGHTNULL = SERVORIGHTNULL + cal_step;
    servo3.writeMicroseconds(SERVORIGHTNULL);
    break;
  case left_servo_0_position:  
    SERVOLEFTNULL = SERVOLEFTNULL + cal_step;
    servo2.writeMicroseconds(SERVOLEFTNULL);
    break;
  case right_servo_factor:
    SERVOFAKTORRIGHT = SERVOFAKTORRIGHT + cal_step;
    drawTo(-3, 29.2);
    break;
  case left_servo_factor:
    SERVOFAKTORLEFT = SERVOFAKTORLEFT + cal_step;
    drawTo(74.1, 28);
    break;
  case lift0_position:
    LIFT0 = LIFT0 + cal_step;
    lift(0);
    break;
  case lift1_position:
    LIFT1 = LIFT1 + cal_step;
    lift(1);
    break;
  case lift2_position:
    LIFT2 = LIFT2 + cal_step;
    lift(2);
    break;
  }
}

void printStatus()  {
  Serial << endl;
  Serial << "x - Change Setup mode  --- Setup mode = ";
  if (CALIBRATION) Serial << "On"; 
  else Serial << "Off";
  Serial << endl;
  if (switchFunctionPlotOn) {
    Serial << "Switch enable/disable plot function" << endl;
    Serial << "Plot function " << digitalRead(switchPin) << endl;
    if (showTemp) Serial << "Shows Temperature" << endl;
    else Serial << "Shows Time" << endl;
  }
  else {
    Serial << "Switch toggle Temperature/Time" << endl;
    if (digitalRead(switchPin)) Serial << "Shows Temperature" << endl;
    else Serial << "Shows Time" << endl;
  }
  Serial << "Flip plot " << flipPlot << endl;
}

void calibration_instructions(){
  Serial << endl;
  Serial << F("0 - Menue") << endl;
  Serial << F("+ - increase") << endl;
  Serial << F("- - decrease") << endl;
  Serial << F("1 - SERVOLEFTNULL position; adjust First Pos 3!") << endl;
  Serial << F("2 - SERVORIGHTNULL position; adjust First pos 4!") << endl; 
  Serial << F("3 - left position; adjust SERVOFAKTORRIGHT") << endl;
  Serial << F("4 - right position; adjust SERVOFAKTORLEFT") << endl;
  Serial << F("5 - Lift 0; adjust Lift 0") << endl;
  ;
  Serial << F("6 - Lift 1; adjust Lift 1") << endl;
  ;
  Serial << F("7 - Lift 2; adjust Lift 2") << endl;
  ;
  Serial << F("8 - Upper origin") << endl;
  Serial << F("9 - Mid position") << endl;
  Serial << F("A - Load default calibration values") << endl;
  Serial << F("B - Store curent values") << endl; 
  Serial << F("C - Load stored values") << endl; 
  Serial << F("D - Toggle 10 / 1 step") << endl;
  Serial << F("w - flip plot") << endl;
  if (!switchFunctionPlotOn)   {
    Serial << F("z - Switch enable/disable plot function") << endl;
  }  
  else {
    Serial << F("z - Switch toggle between Time and Temperature") << endl;
    if (showTemp) Serial << F("y - Plot time") << endl;
    else Serial << F("y - Plot temperature") << endl;
  } 
  Serial << F("x - leave/enter Setup mode") << endl;
}


