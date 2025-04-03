//Arduino Barcode and QR code scanner http://www.circuitschools.com

#include <SoftwareSerial.h>
SoftwareSerial mySerial(6, 7); // TX, RX
 
void setup()
{
  Serial.begin(9600);  
  mySerial.begin(9600); // set the data rate for the SoftwareSerial port
  Serial.println("Barcode scanner Arduino");
}
 
void loop()
{
  if (mySerial.available()) // Check if there is Incoming Data in the Serial Buffer.
  {
    while (mySerial.available()) // Keep reading Byte by Byte from the Buffer till the Buffer is empty
    {
      char input = mySerial.read(); // Read 1 Byte of data and store it in a character variable
      Serial.print(input); // Print the Byte
      delay(5); // A small delay
    }
    Serial.println();
  }
}