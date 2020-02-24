/* MEAM510 Final Robot Code
 * Tophat + Vive + Car + Teensy + Auto
 * Anastasia, Jiacan, Geli, Juncheng
 * ~~~~~ Team23 FTW~~~~~
 * 
 *    TABLE OF CONTENTS
 * 
 *    I2C -> line 26
 *    LED -> line 210
 *    Motor/Servo -> line 369
 *    Vive -> line 600 
 *    Auto  -> line 745 - line 1013
 *    Setup -> line 1031
 *    Loop -> 1077
 * 
 */

// ========================= includes =============================
// add any libraries here
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

// =================================================================
// ========================I2C start =============================
// =================================================================


#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)33            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)25            // gpio number for I2C master data
#define I2C_MASTER_NUM I2C_NUMBER(1)                // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 40000                    // I2C master clock frequency (Hz)
#define I2C_MASTER_TX_BUF_DISABLE 0                 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                 // I2C master doesn't need buffer

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS     // ESP32 slave address, you can set any 7bit value
#define WRITE_BIT I2C_MASTER_WRITE                  // I2C master write
#define READ_BIT I2C_MASTER_READ                    // I2C master read
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                           // I2C master will not check ack from slave
#define ACK_VAL I2C_MASTER_ACK                      // I2C ack value
#define NACK_VAL I2C_MASTER_NACK                    // I2C nack value

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) 
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) 
    {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) 
    {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
    int ret;

    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
        Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
        Serial.printf(" MASTER READ FROM SLAVE ******\n");
        disp_buf(data_rd, DATA_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
            esp_err_to_name(ret));
    }
}

static void i2c_write_test()
{ 
    int ret;
                                                                             
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information being send over I2C
//        Serial.printf(" MASTER WRITE TO SLAVE\n");
//        disp_buf(data_wr, W_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
    }
}
// =================================================================
// ========================== I2C end ==============================
// =================================================================


// =================================================================
// ====================== Interrupt start ==========================
// =================================================================
// Timer + Interrupt for reading I2C
hw_timer_t* timer = NULL;                               // initialize a timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   // needed to sync between main loop and ISR when modifying shared variable
volatile bool readI2C = 0;                              // should we read data from I2C?

void IRAM_ATTR readI2COnTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    readI2C = 1;                        // need to read I2C next loop
    portEXIT_CRITICAL_ISR(&timerMux);
}
// =================================================================
// ======================= Interrupt end ===========================
// =================================================================


// =================================================================
// ========================= LED start =============================
// =================================================================
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define ROBOTNUM    3               // robot number on meta team (1-4)
#define TEAMCOLOR   RED            // color for the robot team, either RED or BLUE
// ==========================

#define NEO_LED_PIN 12              // pin attached to LED ring
#define LED_TYPE    WS2812          // APA102
#define COLOR_ORDER GRB             // changes the order so we can use standard RGB for the values
#define NUM_LEDS    24              // number of LEDs in the ring
CRGB leds[NUM_LEDS];                // set value of LED, each LED is 24 bits

#define BRIGHTNESS          60      // lower the brightness a bit

// core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) 
    {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) 
    {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void)
{
    // tell FastLED about the LED strip configuration
    FastLED.addLeds<LED_TYPE,NEO_LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    int core = xPortGetCoreID();


    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);
}

void ShowRobotNum(void)
{
    int robotLeds[] = {0,6,12,18};      // location of the LEDs used to display the robot number

    // change the LEDs based on the robot number
    leds[robotLeds[0]] = TEAMCOLOR;     // The first LED is always displayed with the robot number

    switch (ROBOTNUM)
    {
        case 1:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = 0;
            break;
        case 2:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = 0;
            break;
        case 3:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
        case 4:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
    }
}

void ShowHealth(int health)
{
  int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23}; // the location of the 24 LEDs used for health

  float ratio=1.3333;//equals to 20/15
  for(int i = 0; i < (health*ratio); i++){
    leds[healthLeds[i]] = HEALTHCOLOR;
  }
  for(int i = health*ratio+1; i < 20;i++){
    leds[healthLeds[i]] = 0;
  }
}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}


void ShowRespawnTimer(int respawnTime)
{
  for(int i = 0; i < (respawnTime*1.6); i++){
    leds[i] = RED;
  }
  for(int i = respawnTime*1.6+1; i < 20;i++){
    leds[i] = 0;
  }
  
}
// =================================================================
// ========================== LED end ==============================
// =================================================================


// =================================================================
// ====================== Motor/Servo start ========================
// =================================================================
/*


*/
#include <WiFi.h>
#include <WiFiUDP.h>
#define LEDC_CHANNEL1 0 //forward pin a
#define LEDC_CHANNEL2 1 //forward pin b
#define LEDC_CHANNELs 4 //steering servo control

#define LEDC_RESOLUTION_BITS 10 //10 bit res bay-BEE
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1)
#define LEDC_TIMER_BIT 10 //10 bit resolution for ledctimer
#define LEDC_FREQ_HZ 1200 //for motor -- 5000Hz -- changed to 200
#define LEDC_BASE_FREQ 50 //for servo -- 50Hz
//
const char* ssid = "TwasbBrilligAndTheSlithyToves"; //jabberwocky
  
// variables for UDP
WiFiUDP udp;
bool dataReceived = false;
unsigned int UDPlocalPort = 2200;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2100;  // UDP port number on the target ESP
const int packetSize = 100;          // define packetSize (length of message)
byte receiveBuffer[packetSize];        // create the receiveBuffer array
int yUDP, xUDP;
int zIO, pressed; //stop button

//modify start
int PWM_PINA = 23;
int PWM_PINB = 22;

int FWD_PINA = 14; //forward motorA
int REV_PINA = 32; //backward A

int FWD_PINB = 26; // forward motorB
int REV_PINB = 27; //backward B
  
int servoPin = 19; //servo control pin

int teensyPin = 21; //for digital comm with teensy


int Min = 40; // (40/1024) * 20ms
int Max = 80; // (80/1024) * 20ms 
int Mid = 60;
//modify end
 
  //STA MODE
  
  // IP Addresses
IPAddress IPlocal(192,168,1,161);         // initialize local IP address 
IPAddress IPtarget(192,168,1,114);      // initialize target IP address
  
volatile int ISRcnt; //poor ISRcnt has terribly volatile mood swings :(
//honestly, its relationship with ISR is so toxic, there's no stability
  
hw_timer_t * timer2 = NULL; //point to timer
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED; //handles syncing shared var btwn loop() & ISR

void IRAM_ATTR onTimer() { //place in IRAM 
  portENTER_CRITICAL_ISR(&timerMux2); //enter critical section  ** sweating intensifies **
  ISRcnt++; //counter that will be checked before calling UDPreceive()
  portEXIT_CRITICAL_ISR(&timerMux2);
}

// can set syntax to be like analogWrite() with input[ 0 : valueMax ] 
//aka a Canal street version of analogWrite() --> suuuure, that Prada bag is totally real for $50 on a sidewalk       
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {            
  uint32_t duty =  LEDC_RESOLUTION * min(value, valueMax) / valueMax;   
  ledcWrite(channel, duty);  // write duty to LEDC
}



void motor_setup() {

   //STA WIFI SETUP
   
  WiFi.begin(ssid);           // connect to network (ssid)
  WiFi.setSleep(false); //fix delay?

  IPAddress gateway(192,168,1,1);         // init gateway IP
  IPAddress subnet(255,255,255,0);        // init subnet mask
  WiFi.config(IPlocal, gateway, subnet);     // set IP address of ESP

  udp.begin(UDPlocalPort);    // configure a port for UDP comms

    // hold the code here and wait until the WiFi is connected
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected!"); 

  //add start
  pinMode(PWM_PINA, OUTPUT);
  pinMode(PWM_PINB, OUTPUT);
  //add end
  
  pinMode(REV_PINA, OUTPUT); //reverse pin a
  pinMode(REV_PINB, OUTPUT);  //reverse pin a
  pinMode(FWD_PINA, OUTPUT); //fwd pin a
  pinMode(FWD_PINB, OUTPUT); //fwd pin b
  pinMode(servoPin, OUTPUT); //servo pwm control pin
  pinMode(teensyPin, OUTPUT); //teensy comm pin


  
  ledcSetup(LEDC_CHANNEL1, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcSetup(LEDC_CHANNEL2, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS); //channel0, 60Hz, 13bits
  ledcAttachPin(PWM_PINA, LEDC_CHANNEL1);// forward A
  ledcAttachPin(PWM_PINB, LEDC_CHANNEL2);// forward B

  // Setup timer and attach timer to a pin
  ledcSetup(LEDC_CHANNELs, LEDC_BASE_FREQ, LEDC_TIMER_BIT); 
  ledcAttachPin(servoPin, LEDC_CHANNELs);
}

int UDPreceive () { //the UDP reception desk is pretty flaky, but speedy, solid 2.5 star hotel
  int cb = udp.parsePacket(); // read data (check to see the number of bytes of the packet)
  
  if (cb)
  {
    dataReceived = true; //data flag
    udp.read(receiveBuffer, packetSize); // y'all know what's up
    xUDP = receiveBuffer[0]; //save in global val
    yUDP = receiveBuffer[1];    //save in global val
    pressed = receiveBuffer[2]; //z press = attack trigger
  }
}


void HeyBuddySlowItDown() {
  digitalWrite(FWD_PINA,0);
  digitalWrite(FWD_PINB,0);
  digitalWrite(REV_PINA,0);
  digitalWrite(REV_PINB,0);
}

void servoControl (int val) { //val is xUDP
  if (val > 103) {
    val = map(val, 103, 255, Mid, Max); //map to max and min duty vals
    ledcWrite(LEDC_CHANNELs, val); //use existing write function

  } else if (val < 90) {
    val = map(val, 1, 90, Min, Mid); //map to max and min duty vals -- check if inverted
    ledcWrite(LEDC_CHANNELs, val);  //use existing write function

  } else {
    ledcWrite(LEDC_CHANNELs, Mid); //midpoint of min&max
  }
}


void setMotor(int val)
{
  if (val > 107) {  //120 is upper bound of center value for joystick
    val = map(val, 107, 255, 1, 255); //map half range of joystick to full PWM range
    
    digitalWrite(FWD_PINB, HIGH);
    digitalWrite(REV_PINB, LOW);  //set reverse pins low
    digitalWrite(FWD_PINA, HIGH);
    digitalWrite(REV_PINA, LOW);
    ledcAnalogWrite(LEDC_CHANNEL1, val);  //write PWM to fwd pins 
    ledcAnalogWrite(LEDC_CHANNEL2, val); 
    
  } else if (val < 93) { //115 is around lower bound of center value
    val = map(val, 1, 93, 255, 1); //half to full range
    
    digitalWrite(REV_PINA, HIGH);
    digitalWrite(FWD_PINA, LOW); //set forward pins low
    digitalWrite(REV_PINB, HIGH);
    digitalWrite(FWD_PINB, LOW);
    ledcAnalogWrite(LEDC_CHANNEL1, val);  //write PWM to rev pins
    ledcAnalogWrite(LEDC_CHANNEL2, val);
    
  } 
} 

int motorLoop(int health, int stat) {
  if (ISRcnt > 0) { //if interrupt has been called
    portENTER_CRITICAL(&timerMux2); //handles syncing shared var
    ISRcnt--; //bring the count back down (kinda a bully, poor count's self esteem is in the pits)
    portEXIT_CRITICAL(&timerMux2);     //exit crit period
    UDPreceive(); //check controller output
  }
  if ((health == 0) || (stat == 1)) {
    HeyBuddySlowItDown();
    servoControl(Mid);
  } else if (dataReceived) { //if boolean was set in UDPreceive()
    servoControl(xUDP); //steer first
    setMotor(yUDP); //then power motor
    if (pressed > 1) { //teensy weapon press check
      digitalWrite(teensyPin, HIGH);
    } else { 
      digitalWrite(teensyPin, LOW);
    }
    
    dataReceived = false; //reset flag (let your freak flag fly)
  }
}

// =================================================================
// ======================= Motor/Servo end =========================
// =================================================================

/*
   ==================================================================
   =========================== VIVE START =============================
   ==================================================================

    Dual sensor Vive Localization using hardware interrupts on ESP32

    BPV22NF photodiode w/ TLV2462IP op amp (no additional filtering etc)

*/
int yF2, xF3;

int driveSpeed = 90;
int currentFunc = 13;
bool red = 0;
bool blue = 0;

int frontVive = 35;
int rearVive = 34;
int fSyncCnt, rSyncCnt = 0;
int fDone = 1;
int rDone = 0;
volatile unsigned long fPulseWidth, fPulseFall, xF, yF, fDistance;  //holds the length of the input pulse.  volatile because it is updated by the ISR
volatile unsigned long rPulseWidth, rPulseFall, xR, yR, rDistance;  //holds the length of the input pulse.  volatile because it is updated by the ISR
volatile unsigned long fPulseRise, rPulseRise = 0;  //time the pulse started.  Used in calculation of the pulse length
volatile boolean newRPulse, newFPulse = false;  //flag to indicate that a new pulse has been detected
bool autoTime = 0;


portMUX_TYPE mux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux2 = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR frontPW() //isr for getting pulse width and distance
{
  portENTER_CRITICAL(&mux1);
  if (digitalRead(frontVive) == HIGH)  //if the pin is HIGH
  {
    fPulseRise = micros();  //save the current time in microseconds
    newFPulse = true;
    fDistance = fPulseRise - fPulseFall; //get the distance between end of last pulse and start of new pulse
  } else if (newFPulse == true)  //otherwise if we are on a new pulse
  {
    fPulseFall = micros(); //save time of fall
    fPulseWidth = fPulseFall - fPulseRise;  //calculate the pulse width
    newFPulse = false;  //set flag to indicate that we are done with new pulse
  } else if ( autoTime = 0) { //do nothing -- keep ISR short if not in automode
    
  }
  portEXIT_CRITICAL(&mux1);
}

void IRAM_ATTR rearPW() //isr for getting pulse width and distance
{
  portENTER_CRITICAL(&mux2);
  if (digitalRead(rearVive) == HIGH)  //if the pin is HIGH
  {
    rPulseRise = micros();  //save the current time in microseconds
    newRPulse = true;
    rDistance = rPulseRise - rPulseFall; //get the distance between end of last pulse and start of new pulse
  } else if (newRPulse == true)  //otherwise if we are on a new pulse
  {
    rPulseFall = micros(); //save time of fall
    rPulseWidth = rPulseFall - rPulseRise;  //calculate the pulse width
    newRPulse = false;  //set flag to indicate that we are done with new pulse
  } else if ( autoTime = 0) { //do nothing -- keep ISR short if not in automode
    
  }
  portEXIT_CRITICAL(&mux2);
}

void frontLoop () {
  if (fPulseWidth != -1) //restricts bc loop is faster than ISR is called
  {
//    Serial.print("fSyncCnt: ");
//    Serial.println(fSyncCnt);
    if (fPulseWidth > 60) // threshold for sync pulses, subject to change based on nature's whimsy
    {
      fSyncCnt++; //increment count for sync pulses
      fPulseWidth = -1; //reset 
      if (fSyncCnt > 3) //make sure synccnt is set up to go into next sweep loop
      {
        fSyncCnt = 3;
      }
    }
    else
    {
      fPulseWidth = -1; 
      if (fSyncCnt == 3) //if we have counted 3 syncs
      {
        fSyncCnt = 0; //reset
        xF = fDistance; //use distance calc from ISR to get how far we are on x axis
//        Serial.print("xF: ");
//        Serial.println(xF);
      }
      else if (fSyncCnt == 1) //after sync pulse between x&y
      {
        fSyncCnt = 0; //reset counter
        yF = fDistance; //get distance along y axis
//        Serial.print("yF: ");
//        Serial.println(yF);
      }
    }
  }
  fDone = 1;
}

void rearLoop() {
  if (rPulseWidth != -1) //restricts bc loop is faster than ISR is called
  {
//    Serial.print("rSyncCnt: ");
//    Serial.println(rSyncCnt);
    if (rPulseWidth > 60) // threshold for sync pulses, subject to change based on nature's whimsy
    {
      rSyncCnt++; //increment count for sync pulses
      rPulseWidth = -1; //reset 
      if (rSyncCnt > 3) //make sure synccnt is set up to go into next sweep loop
      {
        rSyncCnt = 3;
      }
    }
    else
    {
      rPulseWidth = -1; 
      if (rSyncCnt == 3) //if we have counted 3 syncs
      {
        rSyncCnt = 0; //reset
        xR = rDistance; //use distance calc from ISR to get how far we are on x axis
//        Serial.print("xR: ");
//        Serial.println(xR);
      }
      else if (rSyncCnt == 1) //after sync pulse between x&y
      {
        rSyncCnt = 0; //reset counter
        yR = rDistance; //get distance along y axis
//        Serial.print("yR: ");
//        Serial.println(yR);
      }
    }
  }
  rDone = 1;
}

void vive () {
 if (fDone) {
    fDone = 0;
    rearLoop();
    if(rDone) {
      rDone = 0;
      frontLoop();
    }
  }
}

void viveSetup() { //setup after because otherwise throws errors?
  pinMode (frontVive, INPUT);
  pinMode (rearVive, INPUT);
//call interrupt function on change in pin value
  attachInterrupt(digitalPinToInterrupt(frontVive), frontPW, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(rearVive), rearPW, CHANGE); 
}


// =====================================================================
// ============================== AUTO LOOP ============================
// =====================================================================
//declaring variables here so easier to reference on readthrough
//#define RED 1
int turningX = 4380;
int goal[2] = {4470, 3700};

void autoLoop()
{
  unsigned long xF0;
  unsigned long yF0;
  unsigned long xR0;
  unsigned long yR0;


  xF0 = xF;
  yF0 = yF;
  xR0 = xR;
  yR0 = yR;
    
  while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
  {
    vive(); //get more coordinates
  }
  if (yF > 4500)
  {
    if (xR < goal[0]) //
    {
    while(!(abs((long)yF - (long)yR)<30))
    {
      xF0 = xF;
      yF0 = yF;
      xR0 = xR;
      yR0 = yR;
    
      while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
      {
        vive();
      }

      servoControl(240);
      digitalWrite(FWD_PINA,HIGH);
      digitalWrite(FWD_PINB,LOW);
      digitalWrite(REV_PINA,LOW);
      digitalWrite(REV_PINB,HIGH);
      ledcAnalogWrite(LEDC_CHANNEL1, 250);
      ledcAnalogWrite(LEDC_CHANNEL2, 250);
      delay(50);

    }

//    Serial.println("Facing right!");
    HeyBuddySlowItDown();
    delay(1000);

    while (!(xR > turningX && xF < turningX))
    {
      xF0 = xF;
      yF0 = yF;
      xR0 = xR;
      yR0 = yR;
      while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
      {
          vive();
      }
    
      servoControl(95);
      digitalWrite(FWD_PINA,HIGH);
      digitalWrite(FWD_PINB,HIGH);
      digitalWrite(REV_PINA,LOW);
      digitalWrite(REV_PINB,LOW);
      ledcAnalogWrite(LEDC_CHANNEL1, 160);
      ledcAnalogWrite(LEDC_CHANNEL2, 160);
      delay(10);
    }
//    Serial.println("It's good here!");
    
    HeyBuddySlowItDown();
    delay(2000);

    xF0 = xF;
    yF0 = yF;
    xR0 = xR;
    yR0 = yR;
    
    while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
    {
      vive();
    }
  
    while(!(abs((long)xF - (long)xR)<50))
    {
//      Serial.print("abs(xF - xR): ");
//      Serial.println(abs((long)xF - (long)xR));

      xF0 = xF;
      yF0 = yF;
      xR0 = xR;
      yR0 = yR;
    
      while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
      {
          vive();
      }
      
      servoControl(10);
      digitalWrite(FWD_PINA,LOW);
      digitalWrite(FWD_PINB,HIGH);
      digitalWrite(REV_PINA,HIGH);
      digitalWrite(REV_PINB,LOW);
      ledcAnalogWrite(LEDC_CHANNEL1, 250);
      ledcAnalogWrite(LEDC_CHANNEL2, 250);
      delay(50);
    }
    
//    Serial.println("Facing straight!");
    }
    else
    {
      while(!(abs((long)yF - (long)yR)<30))
      {
        xF0 = xF;
        yF0 = yF;
        xR0 = xR;
        yR0 = yR;
    
        while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
        {
          vive();
        }

        servoControl(10);
        digitalWrite(FWD_PINA,LOW);
        digitalWrite(FWD_PINB,HIGH);
        digitalWrite(REV_PINA,HIGH);
        digitalWrite(REV_PINB,LOW);
        ledcAnalogWrite(LEDC_CHANNEL1, 250);
        ledcAnalogWrite(LEDC_CHANNEL2, 250);
        delay(50);

      }

//    Serial.println("Facing left!");
      HeyBuddySlowItDown();
      delay(1000);

      while (!(xR < turningX && xF > turningX))
      {
        xF0 = xF;
        yF0 = yF;
        xR0 = xR;
        yR0 = yR;
        while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
        {
          vive();
        }
    
        servoControl(95);
        digitalWrite(FWD_PINA,HIGH);
        digitalWrite(FWD_PINB,HIGH);
        digitalWrite(REV_PINA,LOW);
        digitalWrite(REV_PINB,LOW);
        ledcAnalogWrite(LEDC_CHANNEL1, 160);
        ledcAnalogWrite(LEDC_CHANNEL2, 160);
        delay(10);
      }
//    Serial.println("It's good here!");
    
      HeyBuddySlowItDown();
      delay(2000);

      xF0 = xF;
      yF0 = yF;
      xR0 = xR;
      yR0 = yR;
    
      while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
      {
        vive();
      }
  
      while(!(abs((long)xF - (long)xR)<60))
      {
//      Serial.print("abs(xF - xR): ");
//      Serial.println(abs((long)xF - (long)xR));

        xF0 = xF;
        yF0 = yF;
        xR0 = xR;
        yR0 = yR;
    
        while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
        {
          vive();
        }
      
        servoControl(240);
        digitalWrite(FWD_PINA,HIGH);
        digitalWrite(FWD_PINB,LOW);
        digitalWrite(REV_PINA,LOW);
        digitalWrite(REV_PINB,HIGH);
        ledcAnalogWrite(LEDC_CHANNEL1, 250);
        ledcAnalogWrite(LEDC_CHANNEL2, 250);
        delay(50);
      }
    
//    Serial.println("Facing straight!");
    
    }
    HeyBuddySlowItDown();
    delay(1000);


    while (!(yR < goal[1] && yF > goal[1]))
    {
      xF0 = xF;
      yF0 = yF;
      xR0 = xR;
      yR0 = yR;
      while(xF == xF0 || yF == yF0 || xR == xR0 || yR == yR0)
      {
          vive();
      }

      if ((xF + xR)/2 < goal[0] - 15)
      {
        servoControl(180);
        digitalWrite(FWD_PINA,HIGH);
        digitalWrite(FWD_PINB,HIGH);
        digitalWrite(REV_PINA,LOW);
        digitalWrite(REV_PINB,LOW);
        ledcAnalogWrite(LEDC_CHANNEL1, 220);
        ledcAnalogWrite(LEDC_CHANNEL2, 150);
        delay(100);
        HeyBuddySlowItDown();
        delay(50);
      }
      else if ((xF + xR)/2 > goal[0] + 15)
      {
        servoControl(20);
        digitalWrite(FWD_PINA,HIGH);
        digitalWrite(FWD_PINB,HIGH);
        digitalWrite(REV_PINA,LOW);
        digitalWrite(REV_PINB,LOW);
        ledcAnalogWrite(LEDC_CHANNEL1, 150);
        ledcAnalogWrite(LEDC_CHANNEL2, 220);
        delay(100);
        HeyBuddySlowItDown();
        delay(50);
        
      }
      else
      {
        servoControl(97);
        digitalWrite(FWD_PINA,HIGH);
        digitalWrite(FWD_PINB,HIGH);
        digitalWrite(REV_PINA,LOW);
        digitalWrite(REV_PINB,LOW);
        ledcAnalogWrite(LEDC_CHANNEL1, 160);
        ledcAnalogWrite(LEDC_CHANNEL2, 160);
        delay(100);
        HeyBuddySlowItDown();
        delay(50);
      }
    }

//    Serial.println("It's better here!");
     HeyBuddySlowItDown();
     delay(7000);
  }
    
}

/*
   ==================================================================
   =========================== AUTO END =============================
   ==================================================================
*/ 
// =====================================================================
// ============================= SETUP =================================
// =====================================================================
void setup()
{
    Serial.begin(115200);
    
    // ========================= I2C start =============================
    ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
    // ========================== I2C end ==============================

    // ===================== Interrupts start ==========================
    // default clock speed is 240MHz
    // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
    // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
    timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
    timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
    timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
    timerAlarmEnable(timer);                                // start the timer

    //UDPReceive timer
    timer2 = timerBegin(1, 240, true); //start timer1, prescaler 240 (means microsecs), count up (turn up woot woot)
    timerAttachInterrupt(timer2, &onTimer, true); //attach timer to onTimer() which executes on interrupt
    timerAlarmWrite(timer2, 10000, true); //trigger interrupt at set intervals (reset@10000 counts)
    timerAlarmEnable(timer2); //duh

// ====================== Interrupts end ===========================

// ========================= LED start =============================
    SetupFastLED(); 
// ========================== LED end ==============================
    
//===========================VIVE start ============================
    viveSetup();
//===========================VIVE end ==============================

// ======================== MOTOR start ============================
    motor_setup();
// ========================= MOTOR end =============================
      
}
// =====================================================================
// ========================== END OF SETUP =============================
// =====================================================================


// =====================================================================
// ============================== LOOP =================================
// =====================================================================
void loop()
{
    // ========================= I2C start =============================
    // static variables
    // static variables only get initialized once, when the loop function is first called
    // values of static variables persist through function calls
    static bool gameStatus = 0;     // game on: 1, game off: 0
    static bool reset = 0;          // 1 for resetting
    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
    static bool syncStatus = 0;     // 1 for sync

    static int health;              // robot's health
    static int respawnTimer;        // amount of time remaining on respawn

    int stat = 0; //

    if (readI2C)
    { 
        readI2C = 0;                // set readI2C to be false
        i2c_write_test();           // need this write for some reason in order to get read to work?
        i2c_read_test();            // read information from slave (top hat)  

        // read information
        gameStatus  = 1 & (data_rd[0] >> 0);
        reset       = 1 & (data_rd[0] >> 1);
        autoMode    = 1 & (data_rd[0] >> 2);
        syncStatus  = 1 & (data_rd[0] >> 3);
    
        if (data_rd[1] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            health = data_rd[1];
        }

        if (data_rd[2] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            respawnTimer = data_rd[2];
        }
    }
    // ========================== I2C end ==============================

    // ========================= LED start =============================
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    
    if (health == 0)
    {
        clearLEDs();
        ShowRespawnTimer(respawnTimer);
    }

    FastLEDshowESP32();
    // ========================== LED end ==============================


    // ======================== MOTOR start ============================
    if (gameStatus == 1 ) {
      stat = 1;
      motorLoop(health, stat);
    }
    else if (autoMode == 1) {
        stat = 1;
        autoTime = 1;
        autoLoop();
    } else if (stat == 0){
      motorLoop(health, stat);
    }
}

// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
