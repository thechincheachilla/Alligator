
#include "Pitches.h"
#include <Servo.h>;

// Counters
unsigned int msecCount = 0;  // mSec counter
unsigned int secCount  = 0;  // second counter

// Motors and Servos
Servo hipServo;      // turns alligator (180 degrees ish)
Servo wheelMotorF;   // front 
Servo wheelMotorB;   // back
Servo headServo;     // opens mouth
Servo tailServo;     // wags tail



//------------------
// Motion States
//------------------
// changeMotionState
// Controls motion state changes
// Each motion command will run until the timer reaches the target value.  
// This flag is set when that value is reached (timeout condition).
// The flag remains set until the command processor begins processing the
// next motion command - at which point is is reset (false) until the next
// timeout event.
//
// NOTE:  Currently this means we have only one valve controlling all motion.
//        therefore motions must be processed linearly and not in parallel.
//        This can be changed.  But the change requires a separate valve.
//        (think this through).
//        It might be best handled by making the command processor an object
//        When a parallel command sequence is desired, then a new command
//        processor is spawned to service the new command sequence. 
//
boolean changeMotionState = false;

// Motion States
#define BEGIN           0
#define STOPPED         1
#define MOVE_LEFT       2
#define MOVE_RIGHT      3
#define MOVE_UP         4
#define MOVE_DOWN       5
#define MOVE_FWD        6
#define MOVE_BACK       7
#define OPEN_MOUTH      8
#define CLOSE_MOUTH     9
#define WAG_TAIL_LEFT   10
#define WAG_TAIL_RIGHT  11
#define RIGHT_BACK      12
#define LEFT_BACK       13
#define LEFT_FORWARD    14
#define RIGHT_FORWARD   15
#define END_CMD         255

unsigned int     motionState       = STOPPED;
unsigned int     motionTarget      = 0;
int              servoPos          = 90;    // centerd hip

 
//-----------------------------
// Alligator Mode 
//-----------------------------
// Each alligator mode owns an array of commands that are executed
// when in that mode.

// Alligator Modes
#define A_STOPPED      0
#define A_FORWARD      1
#define A_AVOIDANCE    2

int alligatorMode  = A_FORWARD;

int aAvoidanceIndex = 0;         // index for sequence of avoidance cmds
int aForwardIndex   = 0;
int aStoppedIndex   = 0;

int aAvoidanceCmd[] = {BEGIN, STOPPED, OPEN_MOUTH, CLOSE_MOUTH, WAG_TAIL_LEFT, WAG_TAIL_RIGHT, 
                      WAG_TAIL_LEFT, WAG_TAIL_RIGHT, RIGHT_BACK, LEFT_FORWARD, 
                      LEFT_FORWARD, RIGHT_FORWARD, END_CMD};

int aForwardCmd[]   = {BEGIN, MOVE_FWD, END_CMD};

int aStoppedCmd[]   = {BEGIN, STOPPED, END_CMD};


// timeout values (seconds) for events
#define STOP_AVOID_TIME      2000
#define OPEN_MOUTH_TIME      1500
#define CLOSE_MOUTH_TIME     250
#define WAG_TAIL_TIME        750              //done
#define LEFT_FORWARD_TIME    2000
#define RIGHT_BACK_TIME      3000
#define RIGHT_FORWARD_TIME   2000


// range_finder pins
const int echoPin   = 2;     // ultrasonic sensor
const int trigPin   = 4;    // ultrasonic sensor

const float cmConversion = 0.017;
const int   maxDistance  = 200;
const int   minDistance  = 30;   //Distance to nearest object

// Sound 
const int tonePin   = 3;  // for sound
const unsigned int numNotes = 12;
const int          noteDuration = 500;
const float        noteRange = maxDistance / numNotes;

// nose LED Ports
const int noseLedR  = 13;
const int noseLedL  = 5;

//----------------
// setup
//----------------
void setup() {
  
  setupTimer();
  
  pinMode (noseLedR, OUTPUT);
  pinMode (noseLedL, OUTPUT);
  // setup ports
  
  Serial.begin(9600);
  

  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);
  wheelMotorF.attach(8);
  wheelMotorB.attach(9);
  hipServo.attach(7);
  headServo.attach(11);
  tailServo.attach(12);

  // setup for run conditions
  wheelMotorF.write(90);  // stop
  wheelMotorB.write(90);  // stop
  hipServo.write(91);     // center
 
  motionState = STOPPED;			
  alligatorMode = A_STOPPED;
  aStoppedIndex = 0;
  setTimer(2);      // set timer for 2 seconds
  //changeMotionState = true; 
//--------------
// Loop
//----------------
void loop() {
  
  
  int status = STOPPED;
  
  // Read the Distance Finder (Ultrasonic)
  int distance = takeSounding_cm();  
  printStatus(distance);
      
  // Check a timeout
  if (motionTarget !=0 && msecCount > motionTarget) {
    //
    // A timeout occurred. 
    // Set state to true - which means that a state change must
    // occur in this loop pass and the motionTarget must be reloaded.
    //
    changeMotionState = true;
    motionTarget = 0;  // The cmd processor will set the timer again.
  }

  // Check the operating mode.
  switch (alligatorMode) {
    case A_FORWARD:
    
      digitalWrite(noseLedR, LOW);
      digitalWrite(noseLedL, LOW);
      
      // A_FORWARD is the default mode.  The alligator will continue moving
      // forward until an obstacle is found.  Then it will go into a series of
      // timed motions.
			
      // if we see an obstacle then avoid
      if (distance <= minDistance) {
        alligatorMode = A_AVOIDANCE;
        aForwardIndex = 0;
        aAvoidanceIndex = 0;  // start at the beginning of the cmd index.
        changeMotionState = true;
      }
      else {
	      status = ProcessCommand(aForwardCmd, &aForwardIndex, sizeof(aForwardCmd));
        // We should never get to the END_CMD in this path because
        // there is no timer set for Forward motion.  We rely on 
        // the distance finder to tell us when to enter avoidance mode.
        if (status == END_CMD) {
          Serial.println("Found the Forward END_CMD - we should never see this message!");
          aForwardIndex = 0;
          alligatorMode = A_AVOIDANCE;
        }
			}
      break;
      
    case A_AVOIDANCE:
      digitalWrite(noseLedR, HIGH);
      digitalWrite(noseLedL, HIGH);
      status = ProcessCommand(aAvoidanceCmd, &aAvoidanceIndex, sizeof(aAvoidanceCmd));
      if (status == END_CMD) {
        aAvoidanceIndex = 0;
        alligatorMode = A_FORWARD;
        changeMotionState = true;
      }
      break;
      
    case A_STOPPED:
      digitalWrite(noseLedR, LOW);
      digitalWrite(noseLedL, LOW);
      status = ProcessCommand(aStoppedCmd, &aStoppedIndex,sizeof(aStoppedCmd));
      if (status == END_CMD) {
        aStoppedIndex = 0;
        alligatorMode = A_FORWARD;
        changeMotionState = true;     // force it to change motion state
      }
      break;
			
    default:
      Serial.println("ILLEGAL MODE");
      break;
  }
}


/*
  Function:  ProcessCommand
  Description:
    Processes arrays filled with commands.  The alligatorMode determines
    which array is used.  The various modes are:  
      A_STOPPED, A_FORWARD, A_AVOIDANCE
    The commands are processed only when changeMotionState is true.  This gets
    set to true only when:
      - The last movement command has reached it's time limit
      - In forward mode, an obstacle has been found
      - The alligatorMode changed.
  Parameters:
    cmdArray - The command array (it could be any of the three)
    pIndex - A pointer to the current index intot the command array
    aSize - The size of the command array.
  Returns:
    ID of the command it is currently working on.
*/
int ProcessCommand(int cmdArray[], int *pIndex, int aSize) {
  // In avoidance mode we go through a sequence of states. 
  // We only need to change our state if the changeMotionState flag is set.
  // When the state is to be changed, we read the next command from 
  // the command array.

  if (changeMotionState) {
    changeMotionState = false;
    Serial.print("*** Processing Command ***  cmd = ");
    
    
    wheelMotorF.write(90);
    wheelMotorB.write(90);
    headServo.write(90);
    //tailServo.write(120);

    // read the next command from the array, print it out
    // and operate on it.
    (*pIndex)++;   
    if(*pIndex > aSize) {
      Serial.print("SOMETHING TERRIBLE HAPPENED!  INDEX ");
      Serial.print(*pIndex);
      Serial.print(" arraySize = ");
      Serial.print(sizeof(cmdArray));
      Serial.println(" OUT OF BOUNDS!");
      (*pIndex) = 0;
      return END_CMD;
    }
    int cmd = cmdArray[*pIndex];
    Serial.println(cmd);
    
    switch(cmd) {
      case STOPPED:
        servoPos = 90;          // center
		    wheelMotorF.write(90);
				wheelMotorB.write(90);
				motionState = STOPPED;
				setTimer(STOP_AVOID_TIME);
				break;
        
			case OPEN_MOUTH:
                                servoPos = 90;          // center
				headServo.write(30);
				motionState = OPEN_MOUTH;
				setTimer(OPEN_MOUTH_TIME);
				break;
        
			case CLOSE_MOUTH:
                                servoPos = 90;          // center
				headServo.write(150);
				motionState = CLOSE_MOUTH;
				setTimer(CLOSE_MOUTH_TIME);
				break;
        
			case WAG_TAIL_LEFT:
                                servoPos = 90;          // center
				tailServo.write(140);
				motionState = WAG_TAIL_LEFT;
				setTimer(WAG_TAIL_TIME);
				break;
        
			case WAG_TAIL_RIGHT:
        servoPos = 90;          // center
				tailServo.write(60);
				motionState = WAG_TAIL_RIGHT;
				setTimer(WAG_TAIL_TIME);
				break;
        
			case LEFT_FORWARD:
				//Turn 30 degrees left
				wheelMotorF.write(0);
				wheelMotorB.write(180);
				servoPos = 150;             // 30 degrees left of center
				tailServo.write(90);
				motionState = LEFT_FORWARD;
				setTimer(LEFT_FORWARD_TIME); 
				break;
				
			case RIGHT_FORWARD:
				//Turn 30 degrees right
				wheelMotorF.write(0);
				wheelMotorB.write(180);
        tailServo.write(90);
        servoPos = 30;          // 30 degrees right of center
				
				motionState = RIGHT_FORWARD;
				setTimer(RIGHT_FORWARD_TIME); 
				break;
				
			case RIGHT_BACK:
				//Turn 30 degrees right    
				wheelMotorF.write(180);
				wheelMotorB.write(0);
				servoPos = 25;
				motionState = RIGHT_BACK;
				setTimer(RIGHT_BACK_TIME); 
                                tailServo.write(90);
				break;
				
      case MOVE_FWD:				
        wheelMotorF.write(0);
      	wheelMotorB.write(180);
      	motionState   = MOVE_FWD;
        tailServo.write(90);
        servoPos = 90;          // center
  			motionTarget  = 0;      // turn off motion timer
        // Notice that we don't set a timer for this one.
        // That is because we want to continue moving forward 
        // until we see an obstacle.
				break;
				
			case END_CMD:
        servoPos = 90;          // center
				break;
        
			default:
				// BUGBUG:  Do something, print something
        Serial.println("ILLEGAL CMD");
				break;
		}
    hipServo.write(servoPos + 1);
	} // endif changeMotionState
  return cmdArray[*pIndex];
}


/*
  Function:  setTimer
  Description:
    Sets a timeout value of the number of seconds to wait until the timeout.
    Adds the numSec that are passed in to the second counter to set
    a new motionTarget time.  A timeout event occurs when the sec
    counter reaches the motionTarget value.
  Parameters:
    numSec - The number of seconds before the timeout.
*/
void setTimer(int numSec) {
  motionTarget = msecCount + numSec;
  if (motionTarget == 0) {
    motionTarget++;
  }
}

//------------------------------------
// Ultrasonic Distance Finder
//------------------------------------
// returns distance in centimeters
//
int takeSounding_cm()
{
  static int lastDist=-1;
  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  digitalWrite (trigPin, HIGH);
  delayMicroseconds (10);
  digitalWrite (trigPin, LOW);
  delayMicroseconds (2);
  int duration = pulseIn (echoPin, HIGH);
  int distance = duration * cmConversion;
  
  if (distance < 0){
    distance = 40; 
  }
  if (lastDist < 0) {
    if (distance != 0) {
      lastDist = distance;
    }
  }

  if (abs(lastDist - distance) < 8) {
    distance = lastDist;
  }
  
  return distance;
}

//-----------------------------
// TIMER STUFF - IMPORTANT
//-----------------------------
void setupTimer() {
  
  noInterrupts();  // disable all interrupts
  
  // Set timer0 interrupt at 2kHz
  TCCR0A = 0;
  TCCR0B = 0;
  TCNT0  = 0;  // Timeer Counter 0 - init to 0
  
  // Output Compare Register 0
  // set compare match register for 1khz increments
  // (1msec cycle)
  OCR0A = 249;  // 16,000,000/(1000*64) - 1 (must be < 256)
  
  // turn on CTC mode (Clear Timer on Compare)
  TCCR0A |= (1 << WGM01); // Waveform Generator Match

  // Set CS01 and CS00 bits for 64 prescaler
  TCCR0B |= (1 << CS01) | (1 << CS00);  
  
  // Timer Interrupt Mask Register  
  // Output Compare Interrupt Enable - enable it
  TIMSK0 |= (1 << OCIE0A);
  
  interrupts();  // enable interrupts
}

//
// Timer 0 Interrupt Service Routine
// Timer 0 is setup to get us here every msec
//
ISR(TIMER0_COMPA_vect) {
  // We should get here every 1 msec.
  msecCount++;
  if (msecCount % 1000 == 0) {
    // we should get here every second (1000 msec)
    secCount++;

    // *** The LED is only for debugging the timing ***
    // Flip the led with each pass
    //digitalWrite(ledPin, 1 - digitalRead(ledPin));
    //digitalWrite(ledPin, digitalRead(ledPin) ^ 1);
  }
}


/*
  Function:  printStatus
  Description:
    Prints out many of the important variables for debuggin purposes.
  Parameters:
    distance - The current distance in cm
*/
void printStatus(int distance) {
  Serial.print("Distance: ");
  Serial.print(distance);
  
  Serial.print(" MotionTarget: ");
  Serial.print(motionTarget);
  
  Serial.print(" msecCount: ");
  Serial.print(msecCount);
  
  Serial.print(" motionState: ");
  switch (motionState) {
    case STOPPED:
      Serial.print("STOPPED");
      break;
    case MOVE_FWD:
      Serial.print("MOVE_FWD");
      break;
    case LEFT_BACK:
      Serial.print("LEFT_BACK");
      break;
    case LEFT_FORWARD:
      Serial.print("LEFT_FORWARD");
      break;
    case RIGHT_BACK:
      Serial.print("RIGHT_BACK");
      break;
    case RIGHT_FORWARD:
      Serial.print("RIGHT_FORWARD");
      break;
    case WAG_TAIL_RIGHT:
      Serial.print("WAG_TAIL_RIGHT");
      break;
    case WAG_TAIL_LEFT:
      Serial.print("WAG_TAIL_LEFT");
      break;
    break;
  }
  
  
  Serial.print(" alligatorMode: ");
  switch (alligatorMode) {
    case A_STOPPED:
      Serial.print("A_STOPPED");
      break;
    case A_FORWARD:
      Serial.print("A_FORWARD");
      break;
    case A_AVOIDANCE:
      Serial.print("A_AVOIDANCE");
      break;
    break;
  }

  Serial.print(" (");
  Serial.print(aStoppedIndex);
  Serial.print(",");
  Serial.print(aForwardIndex);
  Serial.print(",");
  Serial.print(aAvoidanceIndex);
  Serial.print(" )");
  
  Serial.println();
}
