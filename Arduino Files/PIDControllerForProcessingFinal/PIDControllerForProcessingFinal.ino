/*
// This code is used to run a joystick in one axis. It used a brushed dc motor shield, and a magnetic hall effect sensor to
// determine location and is ased on a design from Vanderbilt University.


This code communicates with a processing sketch called HapticPaddlePIDFrontEnd_v3.
It allows the user to see the input and output of the PID controller. It was 
developed for use in DMC classes at Tennessee Tech University

Inputs are taken from 3 switches to turn on or off the P,I,and D parameters.
This is done so that students can explore the different kinds of controllers.
P = D10
I = D9
D = D8

This code uses the Arduino PID Library as a controller

Black Motor Lead -> B3
Red Motor Lead   -> B4
Sensor pin       -> A3



// Pulling the stick to the right is positive.

//Last edited 1/29/2015
//Matthew Powelson - Tennessee Tech University
*/

#include <PID_v1.h>



// Pins for gain switches
const int Ppin = 10;
const int Ipin = 9;
const int Dpin = 8;

// Default gain values  <<<<<-------------------------------------
float Kp = 0.3;
float Ki = 0.01;
float Kd = 0.01;


int motorDirectionPin = 13; // Pin on shield for motor direction (motor a)
int motorSpeedPin = 11; // Pin on shield for motor PWM (speed) (motor a)
int sensorPin = 3; // pin that magnetic encoder is on


int gap = 50;
double center = 0;
double offset = 0;
int location;
int previouslocation;
double absoluteLocation;

int overdrive = 0;
int sensorRange = 1028;           //This number mulitplied by overdrive and added to the current sensor reading should give abs location. This number can vary with setup.
int safetyCutoff = 1500;
double motorSpeed = 0;


//Variables may need to be doubles. Check if errors
PID myPID(&absoluteLocation, &motorSpeed, &center, 2, 5, 1, DIRECT);
long clock = 0;

unsigned long serialTime;
//--------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);                  //Slowing this down greatly decreases performance of the paddle
  Serial.println("Location Setup");
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
  pinMode(2, OUTPUT);
//  center = analogRead(sensorPin);
  offset = analogRead(sensorPin);
  location = analogRead(sensorPin);
  previouslocation = analogRead(sensorPin);

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255 , 255);
  myPID.SetSampleTime(5);             // milliseconds
  myPID.SetTunings( Kp, Ki, Kd );
  //  myPID.SetTunings( 1.0 , 0 , 0.0 );



//Enable pins for the switch inputs
  pinMode(Ppin, INPUT);
  pinMode(Ipin, INPUT);
  pinMode(Dpin, INPUT);

  digitalWrite(Ppin, HIGH);   //enable internal pullups
  digitalWrite(Ipin, HIGH);   //enable internal pullups
  digitalWrite(Dpin, HIGH);   //enable internal pullups


}
//----------------------------------------------------------------------------------------------------------
void loop()
{
  GetLocation();

  //-------------------------------------------------------------------------
  // At this point we have the variable absoluteLocation and motorSpeed.
  // Put code to relate the two here ie, f(absoluteLocation) = motorSpeed
  // motorSpeed should range from -255 to 255
  // A positive motorSpeed will try to increase absoluteLocation.
  // A negative motorSpeed will try to decrease absoluteLocation.

  //Modify code below








  //Turn on or off gains based on switch
  float p = Kp;
  float i = Ki;
  float d = Kd;

  if (digitalRead(Ppin) == HIGH)     p = 0;
  if (digitalRead(Ipin) == HIGH)     i = 0;
  if (digitalRead(Dpin) == HIGH)     d = 0;
  myPID.SetTunings( p, i, d );
  
  //Display current Gains
//  Serial.print("  Kp = ");
//  Serial.print(myPID.GetKp());
//  Serial.print(" Ki = ");
//  Serial.print(myPID.GetKi());
//  Serial.print(" Kd = ");
//  Serial.print(myPID.GetKd());

  // Compute PID
  myPID.Compute();


  // Create Dead Zone in around center to account for sensor noise
  if (absoluteLocation < (center + gap) && absoluteLocation > (center - gap)) // center of range of movement, no motor speed
  {
    motorSpeed = 0;
  }











  //Modify code above

  //----------------------------------------------------------
  // Display time since last cycle
//  Serial.print("   Cycle Time: ");
//  Serial.print(millis() - clock);
  clock = millis();

  // Drive the motors according to variable motorSpeed
  WriteMotors();

  //------------------------------------------------------------
  //send-receive with processing if it's time
  if (millis() > serialTime)
  {
    SerialReceive();
    SerialSend();
    serialTime += 10;    //Basically every time
  }







  // Safety cutoff - Shuts off motors if sensor is outside safe range.
  //---------------------------------------------------------

  if (absoluteLocation > center + safetyCutoff || absoluteLocation < center - safetyCutoff )
  {
    digitalWrite(motorDirectionPin, LOW);
    analogWrite(motorSpeedPin,  0);
    while (1) Serial.println("Safety Cutoff Triggered. Infinite loop initiated.");
  }


}  //loop

//----------------------------------------------------------------------------------------------------------
void GetLocation()
{

//  Serial.print("center: ");
//  Serial.print(center);
//  Serial.print("   ");
  //  Serial.print(analogRead(sensorPin));
  previouslocation = location;
  location = analogRead(sensorPin);

  if ((previouslocation - location) > 800)
  {
    overdrive++;
  }

  if ((location - previouslocation) > 800)
  {
    overdrive--;
  }
//  Serial.print("   Overdrive: ");
//  Serial.print(overdrive);
//  Serial.print("    Absolute Location: ");
  absoluteLocation = location + overdrive * sensorRange - offset;
//  Serial.print(absoluteLocation);

}

//---------------------------------------------------------------------------------------------------------
//motorSpeed should be a value from -255 - 255
void WriteMotors()
{
  //Write to the motors
  //------------------------------------------------------------
//  Serial.print("     Motor Speed: ");
  Serial.print(motorSpeed);
  if (motorSpeed > 0)
  {
    digitalWrite(motorDirectionPin, LOW);
    analogWrite(motorSpeedPin,  abs(motorSpeed));
//    Serial.println("     Motors LOW");
  }
  if (motorSpeed < 0)
  {
    digitalWrite(motorDirectionPin, HIGH);
    analogWrite(motorSpeedPin,  abs(motorSpeed));
//    Serial.println("     Motors HIGH");
  }
  if (motorSpeed == 0)
  {
    digitalWrite(motorDirectionPin, HIGH);
    analogWrite(motorSpeedPin,  0);
//    Serial.println("     Motors Off");
  }
}







//Below is code for interface with processing
//=============================================================================================================

/********************************************
 * Serial Communication functions / helpers
 ********************************************/


union {                // This Data structure lets
  byte asBytes[24];    // us take the byte array
  float asFloat[6];    // sent from processing and
}                      // easily convert it to a
foo;                   // float array



// getting float values from processing into the arduino
// was no small task.  the way this program does it is
// as follows:
//  * a float takes up 4 bytes.  in processing, convert
//    the array of floats we want to send, into an array
//    of bytes.
//  * send the bytes to the arduino
//  * use a data structure known as a union to convert
//    the array of bytes back into an array of floats

//  the bytes coming from the arduino follow the following
//  format:
//  0: 0=Manual, 1=Auto, else = ? error ?
//  1: 0=Direct, 1=Reverse, else = ? error ?
//  2-5: float center
//  6-9: float absoluteLocation
//  10-13: float motorSpeed
//  14-17: float P_Param
//  18-21: float I_Param
//  22-245: float D_Param
void SerialReceive()
{

  // read the bytes sent from Processing
  int index = 0;
  byte Auto_Man = -1;
  byte Direct_Reverse = -1;
  while (Serial.available() && index < 26)
  {
    if (index == 0) Auto_Man = Serial.read();
    else if (index == 1) Direct_Reverse = Serial.read();
    else foo.asBytes[index - 2] = Serial.read();
    index++;
  }

  // if the information we got was in the correct format,
  // read it into the system
  if (index == 26  && (Auto_Man == 0 || Auto_Man == 1) && (Direct_Reverse == 0 || Direct_Reverse == 1))
  {
    center = double(foo.asFloat[0]);
    //absoluteLocation=double(foo.asFloat[1]);       // * the user has the ability to send the
    //   value of "absoluteLocation"  in most cases (as
    //   in this one) this is not needed.
    if (Auto_Man == 0)                    // * only change the motorSpeed if we are in
    { //   manual mode.  otherwise we'll get an
      motorSpeed = double(foo.asFloat[2]);    //   motorSpeed blip, then the controller will
    }                                     //   overwrite.

    double p, i, d;                       // * read in and set the controller tunings
    Kp = double(foo.asFloat[3]);           //
    Ki = double(foo.asFloat[4]);           //
    Kd = double(foo.asFloat[5]);           //
    //myPID.SetTunings(p, i, d);            //

    if (Auto_Man == 0) myPID.SetMode(MANUAL); // * set the controller mode
    else myPID.SetMode(AUTOMATIC);             //

    if (Direct_Reverse == 0) myPID.SetControllerDirection(DIRECT); // * set the controller Direction
    else myPID.SetControllerDirection(REVERSE);          //
  }
  Serial.flush();                         // * clear any random data from the serial buffer
}

// unlike our tiny microprocessor, the processing ap
// has no problem converting strings into floats, so
// we can just send strings.  much easier than getting
// floats from processing to here no?
void SerialSend()
{
  Serial.print("PID ");
  Serial.print(center);
  Serial.print(" ");
  Serial.print(absoluteLocation);
  Serial.print(" ");
  Serial.print(motorSpeed);
  Serial.print(" ");
  Serial.print(myPID.GetKp());
  Serial.print(" ");
  Serial.print(myPID.GetKi());
  Serial.print(" ");
  Serial.print(myPID.GetKd());
  Serial.print(" ");
  if (myPID.GetMode() == AUTOMATIC) Serial.print("Automatic");
  else Serial.print("Manual");
  Serial.print(" ");
  if (myPID.GetDirection() == DIRECT) Serial.println("Direct");
  else Serial.println("Reverse");
}


