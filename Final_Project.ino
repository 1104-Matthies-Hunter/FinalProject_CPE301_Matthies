/***********************************
 * Name: Hunter Matthies
 * Assignment: Final Project
 * Date: 11/29/2023
 ***********************************/

#include <Arduino.h>
#include <Stepper.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <uRTCLib.h>

#define DHT11_PIN 7
#define SIGNAL_PIN 5  //analog pin 5
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

volatile unsigned char *portH = 0x102;  //for dc motor
volatile unsigned char *portDDRH = 0x101;
volatile unsigned char *portA = 0x22;  //for water sensor
volatile unsigned char *portDDRA = 0x21;
volatile unsigned char *portC = 0x28;  //for LEDs
volatile unsigned char *portDDRC = 0x27;
volatile unsigned char *portG = 0x34;  //for buttons
volatile unsigned char *portDDRG = 0x33;

volatile unsigned char *myTCCR1A = 0x80;
volatile unsigned char *myTCCR1B = 0x81;
volatile unsigned char *myTCCR1C = 0x82;
volatile unsigned char *myTIMSK1 = 0x6F;
volatile unsigned char *myTIFR1 = 0x36;
volatile unsigned int *myTCNT1 = 0x84;

uRTCLib rtc(0x68);
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };


const int stepsPerRevolution = 2038;
Stepper myStepper = Stepper(stepsPerRevolution, 46, 50, 48, 52);

dht DHT;
const int RS = 11, EN = 12, D4 = 2, D5 = 3, D6 = 4, D7 = 5;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
int value = 0;
int minute = 0;
const byte offPin = 18;
const byte resetPin = 19;
volatile byte state1 = LOW;
volatile byte state2 = LOW;
volatile byte state3 = LOW;
volatile byte state4 = LOW;
volatile byte state5 = LOW;

void setup() {
  U0init(9600);
  adc_init();
  lcd.begin(16, 2);
  URTCLIB_WIRE.begin();
  *portDDRH |= 0b01100000;
  //rtc.set(0,14,19,6,15,12,23);
  *portDDRA |= 0b00000010;
  *portDDRG &= 0b00000000;
  *portA &= 0b00000000;
  *portDDRC |= 0b10101010;
  *portC &= 0b00000000;
  attachInterrupt(digitalPinToInterrupt(offPin), off, RISING);
  attachInterrupt(digitalPinToInterrupt(resetPin), reset, RISING);
}

void loop() {
  rtc.refresh();
  if (state2 == 0){
    if (state1 == 0) {
      state5 = 1;
      //Stepper Motor
      myStepper.setSpeed(10);
      myStepper.step(-stepsPerRevolution);
      

      //LCD and DHT11 Sensor
      if(minute != rtc.minute()){
        int chk = DHT.read11(DHT11_PIN);
        lcd.setCursor(0, 0);
        lcd.print("Temp: ");
        lcd.print(DHT.temperature);
        lcd.print((char)223);
        lcd.print("C");
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print(DHT.humidity);
        lcd.print("%");
      }

      //Water Sensor
      value = 0;
      *portA |= 0b00000010;
      value = adc_read(SIGNAL_PIN);
      *portA &= 0b00000000;
      if (value < 100){
        state2 = !state2;
      }

      if (DHT.temperature > 20){
        //DC Motor On
        *portC &= 0b00000000;
        *portC |= 0b00000001;
        *portH |= 0b00100000;
        if(state4 == 1){
          printvalue(rtc.year());
          U0putchar('/');
          printvalue(rtc.month());
          U0putchar('/');
          printvalue(rtc.day());
          U0putchar(' ');
          printvalue(rtc.hour());
          U0putchar(':');
          printvalue(rtc.minute());
          U0putchar(':');
          printvalue(rtc.second());
          U0putchar('\n');
        }
        state3 = 1;
        state4 = 0;
      }
      else{
        *portC &= 0b00000000;
        *portC |= 0b00001000;
        *portH &= 0b00000000;
        if(state3 == 1){
          printvalue(rtc.year());
          U0putchar('/');
          printvalue(rtc.month());
          U0putchar('/');
          printvalue(rtc.day());
          U0putchar(' ');
          printvalue(rtc.hour());
          U0putchar(':');
          printvalue(rtc.minute());
          U0putchar(':');
          printvalue(rtc.second());
          U0putchar('\n');
        }
        state4 = 1;
        state3 = 0;
      }
    } 
    else {
      *portH &= 0b00000000;
      *portC &= 0b00000000;
      *portC |= 0b00100000;
      lcd.clear();
    }
  }
  else {
    *portH &= 0b00000000;
    *portC &= 0b00000000;
    *portC |= 0b00000010;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.write("ERROR");
    if(state5 == 1){
      printvalue(rtc.year());
      U0putchar('/');
      printvalue(rtc.month());
      U0putchar('/');
      printvalue(rtc.day());
      U0putchar(' ');
      printvalue(rtc.hour());
      U0putchar(':');
      printvalue(rtc.minute());
      U0putchar(':');
      printvalue(rtc.second());
      U0putchar('\n');
    }
    state5 = 0;
  }
  minute = rtc.minute();
}

void off() {
  state1 = !state1;
  printvalue(rtc.year());
  U0putchar('/');
  printvalue(rtc.month());
  U0putchar('/');
  printvalue(rtc.day());
  U0putchar(' ');
  printvalue(rtc.hour());
  U0putchar(':');
  printvalue(rtc.minute());
  U0putchar(':');
  printvalue(rtc.second());
  U0putchar('\n');
  if (state1 == 0){
    int chk = DHT.read11(DHT11_PIN);
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(DHT.temperature);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(DHT.humidity);
    lcd.print("%");
  }
}

void reset() {
  *portA |= 0b00000010;
  value = adc_read(SIGNAL_PIN);
  *portA &= 0b00000000;
  if(value > 100){
    printvalue(rtc.year());
    U0putchar('/');
    printvalue(rtc.month());
    U0putchar('/');
    printvalue(rtc.day());
    U0putchar(' ');
    printvalue(rtc.hour());
    U0putchar(':');
    printvalue(rtc.minute());
    U0putchar(':');
    printvalue(rtc.second());
    U0putchar('\n');
    state2 = 0;
    lcd.clear();
    int chk = DHT.read11(DHT11_PIN);
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(DHT.temperature);
    lcd.print((char)223);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(DHT.humidity);
    lcd.print("%");
  }
}

void adc_init() {
  // setup the A register
  *my_ADCSRA |= 0b10000000;  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111;  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111;  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000;  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111;  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000;  // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX &= 0b01111111;  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX |= 0b01000000;  // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX &= 0b11011111;  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= 0b11100000;  // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num) {
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if (adc_channel_num > 7) {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

void printvalue(unsigned int analogValue) {
  unsigned char flag = 0;
  if (analogValue >= 1000) {
    U0putchar(analogValue / 1000 + '0');
    flag = 1;
    analogValue = analogValue % 1000;
  }
  if (analogValue >= 100 || flag == 1) {
    U0putchar(analogValue / 100 + '0');
    flag = 1;
    analogValue = analogValue % 100;
  }
  if (analogValue >= 10 || flag == 1) {
    U0putchar(analogValue / 10 + '0');
    flag = 1;
    analogValue = analogValue % 10;
  }
  U0putchar(analogValue + '0');
}

void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

unsigned char U0kbhit() {
  return (RDA & *myUCSR0A);
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putchar(unsigned char U0pdata) {
  while (!(TBE & *myUCSR0A))
    ;
  *myUDR0 = U0pdata;
}