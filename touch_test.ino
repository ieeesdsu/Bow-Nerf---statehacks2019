#include <boot.h>
#include <builtins.h>
#include <common.h>
#include <cpufunc.h>
#include <util/crc16.h>
#include <util/delay.h>
#include <eeprom.h>
#include <fuse.h>
#include <avr/interrupt.h>
#include <io.h>
#include <lock.h>
#include <util/parity.h>
#include <pgmspace.h>
#include <portpins.h>
#include <power.h>
#include <sfr_defs.h>
#include <signature.h>
#include <sleep.h>
#include <version.h>
#include <wdt.h>
#include <xmega.h>

#define _NOP() __asm__ __volatile__("nop")
#define _NOP() do { __asm__ volatile ("nop"); } while (0)


int ledPin = 13; // pin for the LED
unsigned long DaysValues[72];
unsigned long touchCount = 0;
int dataPointCounter = 0;
//
//Starting here Jared wrote all this
//

unsigned long periodCounter = 0;
 ISR(__vectorTIM1_COMPA_vect)
 {
  //increment period counter
  periodCounter++;

  Serial.println("period Incremented");
 }
 
void InitializeISR()
{
  //enable global interrupts
  sei();
  //enable Timer1 EventA match Interrupt
  TIMSK1 |= (1 << ICIE1);
}

void InitializeTimer1()
{
  //set fast ModePWM 
  TCCR1A |= ((1 << WGM11) | (1 << WGM10));
  TCCR1B |= ((1 << WGM13) | (1 << WGM12));
  
  //set nonInverting Mode
  TCCR1A |= ((1 << COM1A1) | (1 << COM1A0));
  
  //set prescaler to 1000
  TCCR1B |= ((1 << CS12) | (1 << CS10));
  TCCR1B &= ~(1 << CS11);
  
  //Set OCR1A
  OCR1A = 62499;
}

//
//End of where Jared wrote code
//


void setup() {
  InitializeTimer1(); //Jared Added this
  InitializeISR();    //Jared Added this
  pinMode(ledPin, OUTPUT);  
  pinMode(ctsPin, INPUT);
  int inputPin = 4; // choose the input pin (for PIR sensor) 
  int pirState = LOW; // we start, assuming no motion detected 
  int val = 0; // variable for reading the pin status 
  pinMode(inputPin, INPUT); // declare infrared sensor as input
  Serial.begin(9600);
}

void loop() {

  // touch sensor variable
  int ctsValue = digitalRead(ctsPin);

  /*
  // smoke sensor variables
  float sensor_volt;
  float RS_air; //  Get the value of RS via in a clear air
  float R0 = 60;  // Get the value of R0 via in H2
  float sensorValue;

  // touch sensor code implementation
  if (ctsValue == HIGH){
    digitalWrite(ledPin, HIGH);
    Serial.println("bee detected");
    
  }
  else{
    digitalWrite(ledPin,LOW);
    Serial.println("no activity");
  } 
  delay(500);

  sensorValue = analogRead(A0);
  sensor_volt = sensorValue/1024*5.0;

  Serial.print("sensor_volt = ");
  Serial.print(sensor_volt);
  Serial.println("V");
  delay(1000);

  // Smoke sensor code implementation
  /*--- Get a average data by testing 100 times ---*/
  /*for(int x = 0 ; x < 100 ; x++)
  {
      sensorValue = sensorValue + analogRead(A0);
  }
  sensorValue = sensorValue/100.0;
  /*-----------------------------------------------*/

  /*sensor_volt = sensorValue/1024*5.0;
  RS_air = (5.0-sensor_volt)/sensor_volt; // omit *RL
  R0 = RS_air/9.8; // The ratio of RS/R0 is 9.8 in a clear air from Graph (Found using WebPlotDigitizer)

  Serial.print("sensor_volt = ");
  Serial.print(sensor_volt);
  Serial.println("V");

  Serial.print("R0 = ");
  Serial.println(R0);
  delay(1000);
  //end sensor code
  */
  

  //
  // infrared code implementation
  //

  val = digitalRead(inputPin); // read input value
  if (val == HIGH) // check if the input is HIGH
  {
    digitalWrite(ledPin, HIGH);  // turn LED ON 
    if (pirState == LOW)  // we have just turned on
    { 
      Serial.println("Bee detected!"); // We only want to print on the output change, not state
      pirState = HIGH;
    }
  }
  else
  {
    digitalWrite(ledPin, LOW); // turn LED OFF 
    if (pirState == HIGH) // we have just turned off
    {
      Serial.println("No bee detected."); // We only want to print on the output change, not state 
      pirState = LOW; 
    } 
   }

  //
  // end infrared code implementation
  //

  if(periodCounter == 300)
  {
    DaysValues[dataPointCounter] = touchCount;
    ++dataPointCounter;
    touchCount = 0;
    if(dataPointCounter == 72)
    {
      //Day is over, store the Days data
      
      //reset data collectionpoints for the next day
      for(dataPointCounter = 0; dataPointCounter < 73; dataPointCounter++)
            DaysValues[dataPointCounter] = 0;
      dataPointCounter = 0;
      periodCounter = 0;
      }
    }
  
}
