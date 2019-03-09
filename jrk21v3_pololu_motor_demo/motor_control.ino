/*
* Motor control commands for Pololu jrk23v3
*/

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
