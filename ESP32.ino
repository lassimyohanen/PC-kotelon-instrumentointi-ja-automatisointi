#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "ulptool.h"

#include <SPI.h>
#include <MFRC522.h>
#include <stdio.h>
#include <FastLED.h>
#include <Stepper.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define BLYNK_PRINT Serial

//Hall sensor data is being read from GPIO34

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");
 
//#define RST_PIN 9 // Configurable, see typical pin layout above
//#define SS_PIN 10 // Configurable, see typical pin layout above
const int RST_PIN = 22; // Reset pin
const int SS_PIN = 21; // Slave select pin

const int stepsPerRevolution = 32;  // change this to fit the number of steps per revolution
const int GearedStepsPerRev = 32*64;  // change this to fit the number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 26, 25, 33, 32);

char auth[] = "<INSERT API KEY>";
// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "<WIFI SSID>";
char pass[] = "<WIFI PASSWORD>";

#define NUM_LEDS 60
#define DATA_PIN 27
CRGB leds[NUM_LEDS];

//variables
boolean correct_card = false;
int n=0;
int i=0;
int k=0;

int R = 0;
int G = 0;
int B = 0;

void setup() {
  init_ulp_program();
  Serial.begin(115200); // Initialize serial communications with the PC
  while (!Serial); // Do nothing if no serial port is opened (added for Arduinos based on ATMEGA32U4)
  SPI.begin(); // Init SPI bus
  pinMode(12, INPUT_PULLDOWN);
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  Blynk.begin(auth, ssid, pass);
}

static void start_ulp_program()
{
  /* Start the program */
  esp_err_t err = ulp_run((&ulp_entry - RTC_SLOW_MEM) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);
}

static void init_ulp_program()
{
  esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                  (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
  ESP_ERROR_CHECK(err);

  /* Configure ADC channel */
  /* Note: when changing channel here, also change 'adc_channel' constant
     in adc.S */
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_ulp_enable();
  ulp_low_threshold = 2.3 * (4095 / 3.3);  //2 volt

  /* Set ULP wake up period to 100ms */
  ulp_set_wakeup_period(0, 100 * 1000);

  /* Disable pullup on GPIO15, in case it is connected to ground to suppress
     boot messages.
  */
  rtc_gpio_pullup_dis(GPIO_NUM_15);
  rtc_gpio_hold_en(GPIO_NUM_15);
}

boolean BLYNK_ready(){
  
}
BLYNK_WRITE(V2)
{
  R = param.asInt();
 
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] =  CRGB(R,G,B); 
  }
  FastLED.show();
}

BLYNK_WRITE(V3)
{
int G = param.asInt();
 
for(int i=0;i<NUM_LEDS;i++){
    leds[i] =  CRGB(R,G,B);
  }
  
FastLED.show();
}

BLYNK_WRITE(V4)
{
  B = param.asInt();
  for(int i=0;i<NUM_LEDS;i++){
    leds[i] =  CRGB(R,G,B);
    
}
FastLED.show();
}

boolean RFID_read(){
// Look for new cards
printf("Looking for new cards..\n");
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
mfrc522.PCD_Init(); // Init MFRC522
if ( ! mfrc522.PICC_IsNewCardPresent()) {
  printf("No card found..\n");
  //delay(2000);
  return false;
}
 
// Select one of the cards
if ( ! mfrc522.PICC_ReadCardSerial()) {
  printf("Selecting a card to read..\n");
  boolean tag_read = true;
  return false;
}
if (mfrc522.uid.uidByte[0] == 0x09 && 
    mfrc522.uid.uidByte[1] == 0x18 &&
    mfrc522.uid.uidByte[2] == 0xa7 &&
    mfrc522.uid.uidByte[3] == 0x63){
    correct_card = true;
    printf("Correct card read, access granted..\n");
    return true;
    }
}


boolean power_state(){
  if (digitalRead(12) == HIGH){
    printf("PSU is connected..\n");
    return true;
  }
  else{
    printf("PSU is NOT connected..\n");
    return false;
  }
}


void LED_alarm(){
  for(n=0; n<3; n++){
    for(i=0; i<NUM_LEDS; i++)
      leds[i] = CRGB::Red; // CRGB::{255,255,255,255} // color [4] = {r, g, b, brightness}
    FastLED.show();
    delay(250);
    for(k=0; k<NUM_LEDS; k++)
      leds[k] = CRGB::Black;
    FastLED.show();
    delay(250);
  }
  return;
}

void shut_down(){
  printf("Shutdown initiated.. Remove PSU cord!\n");
  while(digitalRead(12) == HIGH){
    printf("WARNING! PSU still connected!\n");
    LED_alarm();
    delay(1000);
  }
  //summeri();
  printf("Hatch closing initiated..\n");
  delay(5000);
  stepmotor_close();
  //ESP_sleep();
  return;
}

void startup(){
  printf("Hatch opening initiated..\n");
  stepmotor_open();
  while(digitalRead(12) == LOW){
    printf("Hatch opened, waiting for PSU cord..\n");
    delay(1000);
  //battery_charge();
  }
  LED_alarm();
  return;
}

void stepmotor_close(){
  myStepper.setSpeed(150);
  // step one revolution  in one direction:
  printf("Closing the hatch..\n");
  myStepper.step(GearedStepsPerRev);
}

void stepmotor_open(){
  myStepper.setSpeed(150);
  // step one revolution in the other direction:
  printf("Opening the hatch..\n");
  myStepper.step(-GearedStepsPerRev);
}
 
void loop() {
  printf("Loop starting..\n");
  while(RFID_read() == 0 && digitalRead(12) == HIGH){
    RFID_read();
    Blynk.run();
  }
  while(RFID_read() == 0 && digitalRead(12) == LOW){
    RFID_read();
  }
  printf("Right card found! Continuing..\n");
  if(correct_card == true && power_state()){
    printf("Correct card found and power is on, lets shutdown!\n");
    shut_down();
    delay(100);
    start_ulp_program();
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    esp_deep_sleep_start();
  }
  else if(correct_card == true && power_state() == false){
    printf("Correct card found and power is NOT on, lets startup!!\n");
    startup();
  }
  else{
    printf("Didn't find the right card. Lets try again..\n");
  }
}
