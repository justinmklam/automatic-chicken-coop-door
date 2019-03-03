/*
* Motor control setup for pololu jrk21v3 with Arduino UNO R3, verified using linear actualtor LACT2P
*
* Pololu jrk config utility in Serial mode using UART detect baud rate interface. 
* starting with the default configuration settings for LACT2P linear actuators provided on the pololu website
*
* pin 8 connected to jrk pin Rx
* jrk grnd connected to arduino ground
*/


#include <SoftwareSerial.h>
SoftwareSerial mySerial(7,8); // RX, TX, plug your control line into pin 8 and connect it to the RX pin on the JRK21v3

int myTarget = 0; // target position, 0-4095 is the range of the JRK21V3 controller. 

//stuff used for input from pc
char buffer[5] ;
int pointer = 0;
byte inByte = 0;



// announcer for PC Serial output
void announcePos(int (position)) {
  Serial.print("positiion set to ");
  Serial.println(position);
  Serial.flush();
} 




//sets the new target for the JRK21V3 controller, this uses pololu high resulution protocal
void Move(int x) {
  word target = x;  //only pass this ints, i tried doing math in this and the remainder error screwed something up
  mySerial.write(0xAA); //tells the controller we're starting to send it commands
  mySerial.write(0xB);   //This is the pololu device # you're connected too that is found in the config utility(converted to hex). I'm using #11 in this example
  mySerial.write(0x40 + (target & 0x1F)); //first half of the target, see the pololu jrk manual for more specifics
  mySerial.write((target >> 5) & 0x7F);   //second half of the target, " " " 
}  

void MotorOff() {
  Serial.println("Turning off");
  mySerial.write(0xAA);
  mySerial.write(0xB);
  mySerial.write(0x7F);
}




void setup()
{
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Initialized");
  Serial.flush();// Give reader a chance to see the output.

  int myTarget = 0; //the health level at any point in time
  Serial.println("Enter '#' and 4 digit position level (#0000-#4095)");

}

    

void loop()
{

  if (Serial.available() >0) {
   // read the incoming byte:
   
   inByte = Serial.read();
   delay(10);
   
   // If the marker's found, next 4 characters are the position
   if (inByte == '#') {
   

     while (pointer < 4) { // accumulate 4 chars
        buffer[pointer] = Serial.read(); // store in the buffer
        pointer++; // move the pointer forward by 1
      }
      Serial.flush();
      //translating into an int
      myTarget=(buffer[0]-48)*1000+(buffer[1]-48)*100+(buffer[2]-48)*10+(buffer[3]-48);
      pointer =0;  
    
     //makes sure the target is within the bounds
     if (myTarget < 0){
        myTarget = 0;
        }
     else if (myTarget > 4095){
        myTarget=4095;
        }
        
     Move(myTarget);  
     announcePos(myTarget); 
   }
   else if (inByte == '!'){
    MotorOff();
   }

  } 
}
