
/*
* Getting Started example sketch for nRF24L01+ radios
* This is a very basic example of how to send data from one node to another
* Updated: Dec 2014 by TMRh20
*/

#include <SPI.h>
#include "RF24.h"

RF24 radio(7,8);


void setup() {

  delay(5000);
  
  radio.begin();

}

void loop() {

} // Loop

