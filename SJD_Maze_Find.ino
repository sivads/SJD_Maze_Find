/****************************************************************************************
 Programming by S. Davis and J. Davis
 (c) SJ Davis Robotics
 Date: 06/2014
                        
This program aims to demonstrate the maze navigation and line following functionality of 
the ArcBotics Sparki robot platform using the Infrared Reflectance Sensors and the 
Ultrasonic Range Finder.

Sparki's goal is to navigate the maze and find the object a the END of the maze.  He will 
pick up the object and return it to the START of the maze.

1.  He uses all five of his Infrared Reflectance Sensors to follow the maze center black
line to ensure that he is always traveling in the center between the maze walls.
2.  He uses his Ultrasonic Range Finder to detect the maze walls and make navigation 
decisions.
3.  He uses his Infrared Reflectance Sensors to detect the START and END of the maze.
4.  He uses his front Gripper to pick up the object at the END of the maze and return it
to the START of the maze.
5.  He uses his buzzer (beeper) to indicate turns, walls, and dead ends.
6.  He uses his buzzer (beeper) to play a melody when he completes the maze.
7.  He uses his RGB LED to indicate on (green) and off (red) centerline conditions.
8.  He uses his RGB LED to indicate START and END marker detection (blue). 

Documentation, including the Maze Specifications, Program Flow, and Special Maneuvering 
algorithms is included here as PDF files.
               
This application was created as part of Science Fair project for my son's elementary
school.  The theme of the project was Robot Sensors.  We worked very hard and had lots of 
fun learning about Sparki and developing the code together!  If you like what we did or 
find this code useful for your own project, please consider donating to our education 
fund so that can continue to do projects like this and pursue higher educational goals 

E-mail/PayPal Donations: donations@sjdavisrobotics.com

BitCoin Donations: 1Hp3htmCAyiYNg52XXhoVtjB7VdkqaQDMq

**Thank you!**

Steve (dad) and Jonathan (son) Davis

*****************************************************************************************

This Software is provided under the MIT Open Source License Agreement:
 ___           _      ___     _         _   _       
|SJ \ __ ___ _(c)___ | _ \___| |__  ___| |_(_)__ ___
| |) / _` \ V / (_-< |   / _ \ '_ \/ _ \  _| / _(_-<
|___/\__,_|\_/|_/__/ |_|_\___/_.__/\___/\__|_\__/__/
(c) SJ Davis Robotics                                                

Copyright (c) 2014, SJ Davis Robotics (Steven E. Davis and Jonathan Davis)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE. 

			#     #                         #######                  ### 
			#     #   ##   #    # ######    #       #    # #    #    ### 
			#     #  #  #  #    # #         #       #    # ##   #    ### 
			####### #    # #    # #####     #####   #    # # #  #     #  
			#     # ###### #    # #         #       #    # #  # #        
			#     # #    #  #  #  #         #       #    # #   ##    ### 
			#     # #    #   ##   ######    #        ####  #    #    ### 
			
                                        
                                                                    Width Set to 90 ---->
****************************************************************************************/
                                                               
                                            
#include <Sparki.h> 	// include the sparki library
#include "pitches.h" 	// include a list of pitches

// Program Mode Constants
const boolean MAZE_MODE = true;

// Seek Distance Constants

const int SEEK_DISTANCE_FORWARD 	= 14; // forward distance to seek objects in cm
const int SEEK_DISTANCE_RIGHT 		= 10; // right distance to seek objects in cm
const int SEEK_DISTANCE_LEFT 		= 10; // left distance to seek objects in cm

// This value specifies the number of times the forward range function,
// _SparkiPositionFromForwardObject() will hunt for the specified forward range.
const int FORWARD_RANGE_HUNT_THRESHOLD = 15;

// Coin flip constants
const int HEADS = 1;
const int TAILS = 0;

// -----------------------------------------------------------------------
// IR Sensors (line detection) Raw Date

// White-Black Line Detection Threshold
// Typically, white paper is ~1000, a black line is below 400, and off the edge is below 200.
// This value may need adjusting, depending on outside factors, like how much sunlight
// (infrared light) a room has, how dark/white something is, how far away the sensor is 
// from surface, etc.
const int IR_THRESHOLD = 600;		

// IR Sensor (line detection) Data Variables
int edgeLeft = 0;		// left edge IR sensor data
int lineLeft = 0;		// left IR sensor data
int lineCenter = 0;		// center IR sensor data
int lineRight = 0;		// right IR sensor data
int edgeRight = 0;		// right edge IR sensor data

// IR Sensor Data (converted to boolean values)
boolean bEdgeLeft = false;
boolean bLineLeft = false;
boolean bLineCenter = false;
boolean bLineRight = false;
boolean bEdgeRight = false;

// Used to select a random direction
int nDirection = 0;

// -1 = Left, 1 = Right, 0 = Forward
int nLastDirection = 0;
int nLostCount = 0;
boolean bHaveObject = false;
boolean bTerminate = false;
boolean bStartFinishMarker = false;

// -----------------------------------------------------------------------
// Remote Control (IR Receiver) Code

// Remote Control Code
int nRemoteCode = 0;

// =========================================================================
// INIT
// =========================================================================
void setup() 
{
	_LEDOff();

	// initialize to look forward
	sparki.servo(SERVO_CENTER);

	// open the gripper
	_openGripper();

	// Randomize
	randomSeed(sparki.edgeLeft());

	// Make some noise to indicate initialization complete
	_SparkiPlay_StartUpSound();

} // END INIT


// =========================================================================
// MAIN LOOP 
// =========================================================================
void loop() {
  
_START_:

	// ----------------------------------------------------------------------
	// Read the IR Remote Control Code
	nRemoteCode = sparki.readIR();

	// ----------------------------------------------------------------------
	// Read IR Sensor Data
	_ReadIRSensorData();
	
	// ----------------------------------------------------------------------
	// Display IR Sensor Data on LCD Screen
	_UpdateLCDInformation();
	
	// ----------------------------------------------------------------------
	// Sparki Moves
  	
  	// ---------------------------------------------------
  	// SPARKI -- ARE YOU ON THE BLACK PERPENDICULAR LINE?
  	// ---------------------------------------------------
  	// Black Perpendicular Line Detection:
  	// The black perpendicular line across Sparki's path indicates 
  	// the START of the Maze or the FINISH of the maze, depending on
  	// the START or FINISH markers just beyond the black perpendicular line.
  	if (bEdgeLeft && bLineLeft && bLineCenter && bLineRight && bEdgeRight) {

  		// -------------------------------------------
  		// SPARKI SENSES THE BLACK PERPENDICULAR LINE
  		// -------------------------------------------

  		// START or FINISH indicator BLUE
  		_LEDBlue();

  		// Halt Sparki
  		sparki.moveStop();

  		// Make some noise
  		_SparkiBeep(2);

  		// --------------------------------------------------------------
  		// SPARKI -- FIND OUT IF YOU ARE AT THE START OR THE FINISH LINE
  		// --------------------------------------------------------------

  		// Find the START or FINISH (Left or Right Edge Sensor only)
		// The START or FINISH markers are beyond the black solid perpendicular line
  		bStartFinishMarker = false;

  		do {
  			// Move Sparki ahead slowly
  			sparki.moveForward(1);
  			
  			// Read the IR Sensor data (trying to find the START or FINISH marker)
  			_ReadIRSensorData();
  			_UpdateLCDInformation();

  			// If the left edge sensor is white and the right edge sensor is black (START) OR
  			// the left edge sensor is black and the right edge sensor is white (FINISH),
  			// then, Sparki will drop the object at the START line (if he has it) OR he will 
  			// pick up the object at the FINISH line (if he does not have it).
  			
  			if ( (!bEdgeLeft && bEdgeRight) ||
  				 (bEdgeLeft && !bEdgeRight) ) {
 				
  				bStartFinishMarker = true;
  				
  				// Left Edge is White, Right Edge is Black = START
  				if (!bEdgeLeft && bEdgeRight) {
  					
					// --------------------------------
					// SPARKI IS AT THE START MARKER!!
					// --------------------------------
  					
  					// If Sparki has the object, he will drop it at the START line.
  					if (bHaveObject) {
						
						// --------------------------
						// SPARKI -- DROP THE OBJECT
						// --------------------------
						
						// Drop the object
						_openGripper();
						
						// Play Victory Song
						_SparkiPlay_ShaveAndAHaircut();
						
						// Back away from object so Sparki doesn't hit it when he turns around.
						sparki.moveBackward(6);		
						
						// Sparki now has the object, so set the bHaveObject flag
						bHaveObject = false;
					}
  				}
  				
  				// Left Edge is Black, Right Edge is White = FINISH
  				if (bEdgeLeft && !bEdgeRight) {
  					
					// --------------------------------
					// SPARKI IS AT THE FINISH MARKER!!
					// --------------------------------
  					
  					// If Sparki does not have the object, he will pick it up at the FINISH line.
  					if (!bHaveObject) {
						
						// -------------------------------------------
						// SPARKI -- GET THE OBJECT WITH YOUR GRIPPER
						// -------------------------------------------

						// Get object (it should be right in front of him!)
						_closeGripper();
						
						bHaveObject = true;
  					}
  				}
  				
  				// ---------------------
  				// SPARKI -- TURN AROUND
  				// ---------------------
  				
  				// Turn Sparki Around:
  				// The START and FINISH lines are the end points of the maze, 
  				// so Sparki must turn around!
  				_SparkiTurnAround();
  			}
  		
  		} while (!bStartFinishMarker);
 		
  		// Solid Black Line Handler complete, so go back to the start of the main loop.
  		goto _START_;
  	
  	}
	
	// ---------------------------------------
	// SPARKI -- DETECT AND STAY ON THE LINE!
	// ---------------------------------------
	
	// Check if Sparki is not on the line (no sensors are detecting the line)
	// Sparki may be completely lost if he continues to be off the line.

	if (!bLineLeft && !bLineCenter && !bLineRight) {

		// -------------------------------_
		// SPARKI -- YOU ARE OFF THE LINE!
		// --------------------------------

		// Sparki is not on the line (no sensors are detecting the line)
		
		// Off line indicator RED
		_LEDRed();

		// Count how many times Sparki is off the line
		nLostCount = nLostCount + 1;

		// If Sparki continues to be lost, then stop him and halt the program.
		// This should not happen if Sparki stays in the maze.

		if ( nLostCount > 30 ) {
			
			// ---------------------------------------------
			// SPARKI -- IF YOU ARE COMPLETELY LOST -- HALT
			// ---------------------------------------------
			sparki.beep();
			sparki.moveStop();
			bTerminate = true;

			goto _THE_END_;
		}
		else {
			
			// -------------------------------
			// SPARKI -- GET BACK ON THE LINE!
			// -------------------------------
			
			// Turn 5-deg in a random direction to hunt for the line.
			// This could be improve with a smarter approach where the angle could be
			// adjusted depending on his previous turn.  However, this seems to work just fine.

			if (_flipCoin() == HEADS) {
				sparki.moveRight(5);
			}
			else {
				sparki.moveLeft(5);
			}
			
			// Move backward one click, but every 5 times, move forward 2 clicks.
			// This keeps Sparki from wandering too far backwards when he is lost.

			if ( nLostCount % 5 == 0) {
				sparki.moveForward(2);
			}
			else {

				sparki.moveBackward(1);
			}
		}
	}
	else {
		
		// --------------------------------
		// SPARKI -- YOUR ARE ON THE LINE?
		// --------------------------------
		
		// Sparki is on the line here 
		// (at least one of the sensors is detecting the line)

		_LEDGreen();
		nLostCount = 0;

		if ( bLineLeft && bLineRight ) {

			// In rare cases, both the left and right sensors will pick up the line.
			// If Sparki moves forward 2 clicks (charges ahead), he will have an opportunity 
			// to to see if what he detects is more meaningful.
			sparki.moveForward(2);

		}
		else {
			// Left rail detected 
			// Get Sparki back on the center line by moving him slightly left.
			if ( bLineLeft ) {  
				sparki.moveLeft(5);
				nLastDirection = -1;
			}

			// Right rail detected 
			// Get Sparki back on the center line by moving him slightly right..
			if ( bLineRight ) {  
				sparki.moveRight(5); 
				nLastDirection = 1;
			}

			// If the center line sensor is the only one reading a line...
			if ( bLineCenter && !bLineRight && !bLineLeft )	{
				
				// MAZE NAVIGATION:
				// Maze mode uses Sparki's ultrasonic range sensors to make turn decisions.
				if (MAZE_MODE == true) {
					
					// --------------------------------------------
					// SPARKI -- CHECK YOUR RIGHT -- IS IT CLEAR?
					// --------------------------------------------
					if (_RightIsObstacle()) {

						// --------------------------------------------
						// SPARKI -- CHECK YOUR FRONT -- IS IT CLEAR?
						// --------------------------------------------
						if (_FrontIsClear()) {
							
							// ----------------------------------------------
							// SPARKI -- YOUR FRONT IS CLEAR -- MOVE FORWARD
							// ----------------------------------------------
							
							// Front is clear --> Move Forward Five Paces
							sparki.moveForward(5);
						
						}
						else {
							
							// -----------------------------------
							// SPARKI -- YOUR FRONT IS NOT CLEAR!
							// -----------------------------------
							
							_SparkiPlay_FrontSound();

							// Position Sparki at specified distance from the detected forward wall.
							if (_SparkiPositionFromForwardObject(5)) {
							
								// ------------------------------
								// SPARKI -- IS YOUR LEFT CLEAR?
								// ------------------------------
								
								if ( _LeftIsObstacle() ) {

									// -------------------------------------------
									// SPARKI -- YOU ARE BOXED IN -- TURN AROUND!
									// -------------------------------------------
									
									// Boxed in -- Turn Around.
									_SparkiPlay_TurnAroundSound();
									_SparkiTurnAround();

								}
								else {

									// -------------------------------------------
									// SPARKI -- YOUR LEFT IS CLEAR -- TURN LEFT!
									// -------------------------------------------
									
									// Turn Left
									_SparkiPlay_LeftSound();
									_SparkiTurnLeft();
									sparki.moveForward(10);

								}
							}
							else {
								// Sparki cannot range the forward object, so go back the
								// way he came.

								// Turn around
								_SparkiPlay_TurnAroundSound();
								_SparkiTurnAround();

							}
						}
					}
					else {

						// --------------------------------------------
						// SPARKI -- YOUR RIGHT IS CLEAR -- TURN RIGHT
						// --------------------------------------------
						
						// Turn Sparki right 
     					_SparkiPlay_RightSound();

						// This tries to align Sparki to the right turn line BEFORE turning.
						// Sparki needs to be near the center of the maze corridor before making
						// the turn.
						boolean bLineFound = false;

						do {
							_ReadIRSensorData();

							_UpdateLCDInformation();

							if ( !bEdgeRight ) {
								sparki.moveForward(1);	
							}
							else {
								bLineFound = true;
							}
					
						} while (!bLineFound);

						// Now, line up for the turn.
						sparki.moveForward(5);

						// Do the turn
						_SparkiTurnRight();

						// Move forward
						sparki.moveForward(10);
					}
				} 
				else { 
					
					// If not in Maze Mode, just move forward (no turns)
					sparki.moveForward(5);
					
				} // Maze Mode
			}  
		}
	}

_THE_END_:
	if (bTerminate) {
		return;
	}

	//delay(100); // wait 0.1 seconds

}  // END MAIN LOOP

// =========================================================================
// Helper Functions
// =========================================================================

//--------------------------------------------------------------------
// Gripper Control Functions

// Open the Gripper
void _openGripper() {
	sparki.gripperOpen();
	delay(4000);
	sparki.gripperStop();
}

// Close the Gripper
void _closeGripper() {
	sparki.gripperClose();
	delay(4000);
	sparki.gripperStop();
}

//--------------------------------------------------------------------
// LED Control Functions
// Turn LED solid Green

void _LEDGreen() {
	sparki.RGB(0, 255, 0); 	// Make the LED maximum Green
} // END _LEDGreen

// Turn LED solid Red
void _LEDRed() {
	sparki.RGB(255, 0, 0); 	// Make the LED maximum Green
} // END _LEDRed

// Turn LED solid Blue
void _LEDBlue() {
	sparki.RGB(0, 0, 255); 	// Make the LED maximum Blue
} // END _LEDBlue


// Turn LED OFF
void _LEDOff() {
	sparki.RGB(RGB_OFF);
} // END _LEDOff

// Turn LED solid white (all colors on)
void _LEDWhite() {
	sparki.RGB(255, 255, 255);
} // END _LEDWhite

//--------------------------------------------------------------------
// Utility Functions

// Coin Flip:
// Returns Heads or Tails according to defined constants.
int _flipCoin() {
	int nCoin = random(0,2);
	if (nCoin == 0) {
		return TAILS;
	}
	else {
		return HEADS;
	}
} // END _flipCoin

//--------------------------------------------------------------------
// Sparki Maze Movement Functions

// Turn Sparki Around
// This function turns Sparki around so that he can find the guide line.
// Turn him a little past 180-deg, but also turn him slightly, then back up 
// a few paces so that he won't run into the maze wall.
void _SparkiTurnAround() {
	sparki.moveRight(45);
	sparki.moveBackward(2);
	sparki.moveRight(150);
} // END _SparkiTurnAround

// Turn Sparki Right
// This function turns Sparki right so that he can find the guide line.
// Turn him a little past 90-deg.
void _SparkiTurnRight() {
	sparki.moveRight(95);
} // END _SparkiTurnRight

// Turn Sparki Left
// This function turns Sparki left so that he can find the guide line.
// Turn him a little past 90-deg.
void _SparkiTurnLeft() {
	sparki.moveLeft(95);
} // END _SparkiTurnLeft

//--------------------------------------------------------------------
// IR Sensor Functions

// Reads all five IR Sensors
// All data read will be in the global variables so that all other functions
// can use the data (see variable declarations above).
void _ReadIRSensorData() {

	edgeLeft = sparki.edgeLeft();		// get left edge IR sensor data
	lineLeft = sparki.lineLeft();		// get left IR sensor data
	lineCenter = sparki.lineCenter();	// get center IR sensor data
	lineRight = sparki.lineRight();		// get right IR sensor data
	edgeRight = sparki.edgeRight();		// get right edge IR sensor data

	// Turn the sensor data into boolean values based on threshold number
	bEdgeLeft = false;
	if (edgeLeft < IR_THRESHOLD) {bEdgeLeft = true;}

	bLineLeft = false;
	if (lineLeft < IR_THRESHOLD) {bLineLeft = true;}

	bLineCenter = false;
	if (lineCenter < IR_THRESHOLD) {bLineCenter = true;}
	
	bLineRight = false;
	if (lineRight < IR_THRESHOLD) {bLineRight = true;}

	bEdgeRight = false;
	if (edgeRight < IR_THRESHOLD) {bEdgeRight = true;}

} // END _ReadIRSensorData

//--------------------------------------------------------------------
// LCD Information Display 

// LCD Update
void _UpdateLCDInformation() {
	
	sparki.clearLCD();                       // clear the screen
	sparki.println("SJ Davis Robotics");
	
	sparki.println("** IR Sensor Data **");
	
	// Left Edge IR Sensor Data
	sparki.print(" << EDGE L = ");
	sparki.print(edgeLeft);
	sparki.print(" ");
	sparki.println(bEdgeLeft);
	// Left Line IR Sensor Data
	sparki.print(" <  LINE L = ");
	sparki.print(lineLeft);
	sparki.print(" ");
	sparki.println(bLineLeft);
	// Center Line IR Sensor Data
	sparki.print(" |  LINE C = ");
	sparki.print(lineCenter);
	sparki.print(" ");
	sparki.println(bLineCenter);
	// Right Line IR Sensor Data
	sparki.print(" >  LINE R = ");
	sparki.print(lineRight);
	sparki.print(" ");
	sparki.println(bLineRight);
	// Right Edge IR Sensor Data
	sparki.print(" >> EDGE R = ");
	sparki.print(edgeRight);
	sparki.print(" ");
	sparki.println(bEdgeRight);
	
	// Remote control code received.
	// sparki.print("~ REMOTE = ");
	// sparki.println(nRemoteCode);

	sparki.updateLCD(); 

} // END _UpdateLCDInformation

//--------------------------------------------------------------------
// Sparki's Sounds

const int MORSE_DOT = 16;
const int MORSE_DASH = 6;

void _SparkiBeep(int n) {
	if ( n > 0 ) {
		do {
			sparki.beep();
			delay(500);
			n = n - 1;
		} while(n > 0);
	}
} // END _SparkiBeep

void _SparkiPlay_StartUpSound() {

	int melody[] = { NOTE_C5, NOTE_A5};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 4 };
	_SparkiPlayMelody(melody, noteDurations, 2);

}

void _SparkiPlay_ShaveAndAHaircut() {
	// notes in the melody:
	int melody[] = { NOTE_C4, NOTE_G3,NOTE_G3, NOTE_A3, NOTE_G3,0, NOTE_B3, NOTE_C4 };
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, 8, 8, 4, 4, 4, 4, 4 };
	_SparkiPlayMelody(melody, noteDurations, 8);
}

void _SparkiPlay_FrontSound() {
	// notes in the melody:
	// Morse Code C for Center:  -.-.
	int melody[] = { NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { MORSE_DASH, MORSE_DOT, MORSE_DASH, MORSE_DOT};
	_SparkiPlayMelody(melody, noteDurations, 4);
}

void _SparkiPlay_RightSound() {
	// notes in the melody:
	// Morse Code R for Right:  .-.
	int melody[] = { 0, NOTE_C5, NOTE_C5, NOTE_C5};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, MORSE_DOT, MORSE_DASH, MORSE_DOT };
	_SparkiPlayMelody(melody, noteDurations, 4);
}

void _SparkiPlay_LeftSound() {
	// notes in the melody:
	// Morse Code L for Left:  .-..
	int melody[] = { 0, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 4, MORSE_DOT, MORSE_DASH, MORSE_DOT, MORSE_DOT};
	_SparkiPlayMelody(melody, noteDurations, 5);
}

void _SparkiPlay_TurnAroundSound() {
	// notes in the melody:
	int melody[] = { NOTE_G5, NOTE_C4};
	// note durations: 4 = quarter note, 8 = eighth note, etc.:
	int noteDurations[] = { 8, 4 };
	_SparkiPlayMelody(melody, noteDurations, 2);
}

// Melody Player:
// Plays the specified melody (notes and durations)
void _SparkiPlayMelody(int melody[], int durations[], int nSize) {

	// NOTE:
	// Rats.  C arrays don't store their own sizes anywhere, so sizeof() only 
	// works the way you expect if the size is known at compile time. 
	// malloc() is treated by the compiler as any other function, so sizeof() 
	// can't tell that melody is an array, let alone how big it is. 
	// If you need to know the size of the array, you need to explicitly pass 
	// it to your function, either as a separate argument, or by using a struct 
	// containing a pointer to your array and its size.
	for (int thisNote = 0; thisNote < nSize; thisNote++) {

		// calculate the note duration as 1 second divided by note type.
		//e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
		int noteDuration = 1000/durations[thisNote];

		sparki.beep(melody[thisNote],noteDuration);

		// to distinguish the notes, set a minimum time between them.
		// the note's duration + 30% seems to work well:
		int pauseBetweenNotes = noteDuration * 1.30;

		delay(pauseBetweenNotes);

		// stop the tone playing:
		sparki.noBeep();
	}

} // END _SparkiPlayMelody

//--------------------------------------------------------------------
// Sparki's Eyes

// Is there an obstacle to the right of Sparki?
boolean _RightIsObstacle() {
	sparki.servo(85);
	delay(200);
	int cm = sparki.ping();
	delay(200);
	sparki.servo(SERVO_CENTER);
	if (cm < SEEK_DISTANCE_RIGHT) {
		return true;
	}
	else {
		return false;
	}
} // END _RightIsObstacle

// Is there an obstacle to the left of Sparki?
boolean _LeftIsObstacle() {
	sparki.servo(-85);
	delay(200);
	int cm = sparki.ping();
	delay(200);
	sparki.servo(SERVO_CENTER);
	if (cm < SEEK_DISTANCE_LEFT) {
		return true;
	}
	else {
		return false;
	}
} // END _RightIsObstacle

// Is Sparky's path in fromt of him clear?
boolean _FrontIsClear() {
	sparki.servo(SERVO_CENTER);
	delay(200);
	int cm = sparki.ping();
	if (cm < SEEK_DISTANCE_FORWARD) {
		return false;
	}
	else {
		return true;
	}
} // END _FrontIsClear

// Move Sparki to a specified distance from a forward object.
boolean _SparkiPositionFromForwardObject(int distance) {
	boolean bBoom = false;
	int nForwardMoves = 0;
	int nBackwardMoves = 0;

	sparki.servo(SERVO_CENTER);
	delay(200);

	do {
		int cm = sparki.ping();
		// If the forward object (a wall) is at the specified range
		// then exit the loop.
		if ( (cm < distance + 1) && (cm > distance - 1) ) {
			bBoom = true;
		}
		else {
			// Error condition:  hunting too much (threshold?)
			// This is supposed to handle the condition where something other 
			// than a valid wall is detected (someone's hand?) and Sparki 
			// cannot determine it's position (false ping or obstacle removed).
			if ( nForwardMoves + nBackwardMoves > FORWARD_RANGE_HUNT_THRESHOLD) {
				// Can't find the forward object in specified range.
				// Range not found, return fails.
				return false;
			}

			if ( cm < distance ) {
				// Move Sparki backward one click
				sparki.moveBackward(1);
				// Count the times Sparki moves backward
				nBackwardMoves++;
			}
			else {
				// Move Sparki forward one click
				sparki.moveForward(1);
				// Count the times Sparki moves forward.
				nForwardMoves++;
			}
		}
	} while (!bBoom);

	// Range found, return true.
	return true;

} // END _SparkiPositionFromForwardObject





