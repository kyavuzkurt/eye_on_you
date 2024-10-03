#include <Servo.h>

Servo servox;
Servo servoy;  // Create a servo object

void setup() { // setup method
  Serial.begin(9600);      // Initialize serial communication at 9600 baud
  servox.attach(3);       // Attach the servo to pin 3
  servoy.attach(5);
  Serial.println("Servo Control Ready. Enter position (0-180):");
}

void loop() { 
  if (Serial.available() > 0) {                  
    String input = Serial.readStringUntil('\n');   
    input.trim();                                 
    if (input.length() > 0) {                     
      char axis = input.charAt(0);              
      int pos = input.substring(1).toInt();       
      if (pos >= 0 && pos <= 180) {           
        if (axis == 'x') {
          servox.write(pos);                     
          Serial.print("Moved x to position: ");
        } else if (axis == 'y') {
          servoy.write(pos);                      
          Serial.print("Moved y to position: ");
        } else {
          Serial.println("Error: Invalid axis. Use 'x' or 'y'.");
          return;
        }
        Serial.println(pos);
      } else {
        Serial.println("Error: Position must be between 0 and 180.");
      }
    }
  }
}