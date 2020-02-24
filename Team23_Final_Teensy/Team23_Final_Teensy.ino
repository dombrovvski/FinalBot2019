/* 
weaponServo controls the continuous rotation end of the weapon arm
armServo refers to the servo at the base of the arm that extends once 
 
 *   BOARD                    SERVO_PIN_A   SERVO_PIN_B   SERVO_PIN_C
//   Teensy 2.0                    14            15             4
 *  
 *  ABOUT
 *  -- Uses hardware ISR to increment counter for button presses
 *  -- ISR is called when a digital pin changes from low to high
 *  -- armServo is a servo object that controls initial arm extension
 *  -- weaponServo is servo obj that controls the speed+direction of continuous servos
 *  -- on startup, matrix displays team name
 *  -- default matrix is fronwing face, but when attacking matrix makes smiling face as an intimidation tactic

 *  Nas
 */

#include <Wire.h>
#include <WireIMXRT.h>
#include <WireKinetis.h>
#include <Adafruit_LEDBackpack.h>
#include <PWMServo.h>

//for i2C, D0 is SCL, D1 is SDA

Adafruit_8x8matrix matrix = Adafruit_8x8matrix();

//4, 14, 15 are compatible with PWMServo library
//weapon is SERVO_PIN_A = 14 (B5)
//arm is SERVO_PIN_B = 15 (B6)

PWMServo weaponServo; //stop pos 1500 micros, range 700 - 2300, CW 1500-700, CCW 1500-2300
PWMServo armServo; //servo for controlling arm extension, 771-2740(2193)
int triggerPin = PIN_D2;//interrupt triggering pin -- waits for HIGH from ESP


volatile int pressed, started = 0;

static const uint8_t PROGMEM
  smile_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10100101,
    B10011001,
    B01000010,
    B00111100 },
  frown_bmp[] =
  { B00111100,
    B01000010,
    B10100101,
    B10000001,
    B10011001,
    B10100101,
    B01000010,
    B00111100 };
    
void matrixText() {
  matrix.setTextSize(1);
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextColor(LED_ON);
  for (int8_t x=0; x>=-36; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Team 23");
    matrix.writeDisplay();
  }
}
//void matrixTextAttack() {
//  matrix.setTextSize(1);
//  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
//  matrix.setTextColor(LED_ON);
//  for (int8_t x=0; x>=-36; x--) {
//    matrix.clear();
//    matrix.setCursor(x,0);
//    matrix.print("DIE DIE DIE");
//    matrix.writeDisplay();
//  }
//}

void attackTrigger() {
  //on button push
  pressed = 1; //toggle pressed state to control weapon
  if (started == 0) {
    started = 1;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  matrix.begin(0x70);  // pass in the address

  attachInterrupt(digitalPinToInterrupt(triggerPin), attackTrigger, RISING); //triggers on low to high

  weaponServo.attach(SERVO_PIN_B, 771, 2740); //pin, min, max pulse width in microsecs
  armServo.attach(SERVO_PIN_A, 700, 2300); //unclear on limits, but apparently 0deg = 1ms, 180deg = 2ms
  pinMode(PIN_B5, OUTPUT);
  pinMode(PIN_D0, OUTPUT);
  pinMode(PIN_D1, OUTPUT);
  pinMode(triggerPin, INPUT);
}

void loop() {
  weaponControl();
  matrixControl();
}

void weaponControl () {
  
    extendArm();
   
  if (pressed == 1) {
    weaponServo.write(120); //max speed one direction
    delay(5);
//    pressed = 0;
  } else if (pressed == 0) {
      weaponServo.write(50); //slow af in other 
      delay(5);
  }
}

void extendArm() {
  if (started == 1) { //for first press, extend arm
   armServo.write(180); //extend arm
   delay(5);
    started = 2; //make sure never extend again
  } else if (pressed == 1) {
    armServo.write(120);
    delay(5);
  } else if (pressed == 0) {
    armServo.write(180);
    delay(5);
  }

}

void matrixControl () {
  if (started == 1) {
    matrix.clear();
    matrixText(); //set to default automatically
  } else if (pressed == 2) {
    matrix.clear();
    matrix.drawBitmap(0, 0, smile_bmp, 8, 8, LED_ON);
    matrix.writeDisplay();
    pressed = 0;
    delay(100);
  } else if (pressed == 1) {
    matrix.clear();
    matrix.drawBitmap(0, 0, frown_bmp, 8, 8, LED_ON);
    matrix.writeDisplay();
  }
}
