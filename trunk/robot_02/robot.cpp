#include "WProgram.h"
#include "Wire.h"
#include "Servo.h"
#include "math.h"
//#include "WiFly.h"


// Ultrasonic sensors pins
#define USENSOR1 44
#define USENSOR2 45

// Servo pins
const int USERVO = 3;


// Motors pins
#define MOTOR_FL_POWER 7
#define MOTOR_RL_POWER 6
#define MOTOR_FR_POWER 5
#define MOTOR_RR_POWER 4

#define MOTOR_FL_FORWARD LOW
#define MOTOR_RL_FORWARD HIGH
#define MOTOR_FR_FORWARD LOW
#define MOTOR_RR_FORWARD HIGH

#define MOTOR_FL_BACKWARD HIGH
#define MOTOR_RL_BACKWARD LOW
#define MOTOR_FR_BACKWARD HIGH
#define MOTOR_RR_BACKWARD LOW

#define MOTOR_FL_DIRECTION 22
#define MOTOR_RL_DIRECTION 23
#define MOTOR_FR_DIRECTION 24
#define MOTOR_RR_DIRECTION 25

// Compass pins
const int HMC6352Address = 0x42;
// Shift the device's documented slave address (0x42) 1 bit right
// This compensates for how the TWI library only wants the
// 7 most significant bits (with the high bit padded with 0)
const int HMC6352SlaveAddress = HMC6352Address >> 1; // This results in 0x21 as the address to pass to TWI






/*
 * Method declarations.
 */
void updateRobotOrientation();
void updateRanges();
void updateSingleRange(int heading, int range);
int roundToFactor(int value, int roundFactor);
void robotForward(int time, int speed);
void robotBackward(int time, int speed);
void robotTurn();
void robotStop();
void robotMove();
void choosePath(bool forceNew);
void preventColisions();
int getDistanceForHeading(int heading);
int getCurrentRoundedHeading();
void takeHttpRequest();
void robotBackupAndChooseAnotherPath();





const bool ENABLE_SERIAL_DEBUG = false;

/**
 * The robot scans around for obstacles. This value tells the
 * minimum distance to keep from objects in front of him
 * while he travels around.
 *
 * Anything this close will make the robot refuse to go there.
 */
const int MIN_DISTANCE_BUFFER = 12;

/**
 * This has the same role as MIN_DISTANCE_BUFFER, but for diagonals.
 */
const int MIN_DISTANCE_BUFFER_DIAG = 10;

/*
 * A multiplication applied to MIN_DISTANCE_BUFFER_*
 * when choosing new paths. This allows the robot to pick better
 * paths and prevent it from choosing a path which might be onstructed.
 * A value of 1.3 to 1.5 is ok.
 */
const double MIN_DISTANCE_FACTOR = 1.2;

/**
 * Speed, between 1 and 254, at which to move forward.
 */
const int CRUISE_SPEED = 150;

/**
 * This is the speed at which the robot performs course correction
 * tasks, like correcting its heading or simply turning. It must be a value
 * between 1 and 254.
 */
const int COURSE_CORRECTION_SPEED = 100;

/**
 * Time spent trying to correct the course of the robot.
 * after the given time interval, in miliseconds, the robot
 * will stop trying to correct the course and back up a little,
 * then choose some other direction.
 */
const int COURSE_CORRECTION_TIME = 1000000;

/**
 * This is the number of ultranosic scan samples to use when averaging the
 * distance measurements. In order to have better accuracy, the robot always
 * does X scans in each direction and averages the results, thus reducing
 * the noise.
 */
const int USENS_NBSAMPLES = 2;

/**
 * This is the number of compass scan samples to use when averaging the
 * heading measurements. In order to have better accuracy, the robot always
 * does X scans, thus reducing the noise.
 */
const int COMPASS_NBSAMPLES = 2;



/**********************************************************************************/



/**
 * Enum of the possible states of the motors.
 */
enum MOTOR_STATE {
	MOTOR_STOPPED,
	MOTOR_FORWARD,
	MOTOR_BACKWARD,
	MOTOR_LEFT,
	MOTOR_RIGHT
};

/**
 * Current motor state.
 */
MOTOR_STATE currentMotorState = MOTOR_STOPPED;

/**
 * Current polar heading where the robot is trying to go.
 */
int currentDirection = 0;

/**
 * Current polar orientation of the robot.
 */
int currentOrientation = 0;

unsigned long courseCorrectionTimestamp = millis();

/**
 * Sensors servo object
 */
Servo uSensorServo;

/**
 * Current position (in relative degrees) of the servo motor.
 */
int uServoPosition = 0;

/**
 * Structure describing the distance available in all directions, relative to
 * the robot.
 */
struct RANGES {
	int range0;
	int range45;
	int range90;
	int range135;
	int range180;
	int range225;
	int range270;
	int range315;
};

/**
 * Current distances all around the robot.
 */
struct RANGES currentRanges;


//Server server(80);


void setup() {

	// Setup the motor pins.
	pinMode(MOTOR_FL_DIRECTION, OUTPUT);
	pinMode(MOTOR_RL_DIRECTION, OUTPUT);
	pinMode(MOTOR_FR_DIRECTION, OUTPUT);
	pinMode(MOTOR_RR_DIRECTION, OUTPUT);
	pinMode(MOTOR_FL_POWER, OUTPUT);
	pinMode(MOTOR_RL_POWER, OUTPUT);
	pinMode(MOTOR_FR_POWER, OUTPUT);
	pinMode(MOTOR_RR_POWER, OUTPUT);

	uSensorServo.attach(USERVO);
	pinMode(USENSOR1, INPUT);
	pinMode(USENSOR2, INPUT);

	if (ENABLE_SERIAL_DEBUG) {
		Serial.begin(9600);
	}

	// Make sure the robot is stopped.
	robotStop();

	// Start the Wire lib
	Wire.begin();

	// Update the orientation
	updateRobotOrientation();

	// Position the servo to his start position
	uSensorServo.write(0);
	uServoPosition = 0;
	delay(500);

	// Init HTTP comms
//	SpiSerial.begin();
//	delay(5000);
//	server.begin();

	for (int i = 0; i < 4; i++) {
		updateRanges();
	}

	choosePath(false);
}

void loop() {

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("*****************************************************************************");
	}

    // Update the polar heading
	updateRobotOrientation();

	// Update the ranges
	if (currentMotorState == MOTOR_FORWARD) {
		updateRanges();
	}

	// take any HTTP requests.
	//takeHttpRequest();

	// go.
	robotMove();

	if (ENABLE_SERIAL_DEBUG) {
		// throttle a bit.
		delay(1000);
	}
}

/**
 * Updates the current heading of the robot in degrees relative
 * to Earth's north.
 */
void updateRobotOrientation() {

	unsigned long orientationsSum = 0;

	for (int i = 0; i < COMPASS_NBSAMPLES; i++) {

		byte headingData[2];

		// Send a "A" command to the HMC6352
		// This requests the current heading data
		Wire.beginTransmission(HMC6352SlaveAddress);
		Wire.send('A'); // The "Get Data" command
		Wire.endTransmission();

		delay(10); // The HMC6352 needs at least a 70us (microsecond) delay
		// after this command.  Using 10ms just makes it safe
		// Read the 2 heading bytes, MSB first
		// The resulting 16bit word is the compass heading in 10th's of a degree
		// For example: a heading of 1345 would be 134.5 degrees

		Wire.requestFrom(HMC6352SlaveAddress, 2); // Request the 2 byte heading (MSB comes first)

		int i = 0;
		while (Wire.available() && i < 2) {
			headingData[i] = Wire.receive();
			i++;
		}

		orientationsSum += (headingData[0] * 256 + headingData[1]) / 10; // Put the MSB and LSB together

		delay(50);
	}

	currentOrientation = orientationsSum / COMPASS_NBSAMPLES;

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ updateRobotOrientation");
		Serial.print("Current heading: ");
		Serial.print(currentOrientation);
		Serial.println(" degrees");
	}
}




/**
 * Updates the map of ranges by turning the sensor and measuring
 * the ranges, then updating the map itself by accounting
 * for the polar heading of the robot.
 */
void updateRanges() {

	// If the robot is going forward, we scan:
	// front, front left and front right.
	int targetServoOffset;
	int currentRoundedHeading = getCurrentRoundedHeading();
	if (currentMotorState == MOTOR_FORWARD) {
		// Position the servo
		if (uServoPosition > 90) {
			// put in front
			targetServoOffset = -(uServoPosition - 90);
		} else if (uServoPosition == 90) {
			int leftDiag = currentRoundedHeading - 45;
			if (leftDiag < 0) {
				leftDiag = 360 - leftDiag;
			}
			int rightDiag = currentRoundedHeading + 45;
			if (rightDiag >= 360) {
				rightDiag = rightDiag - 360;
			}
			int leftDist = getDistanceForHeading(leftDiag);
			int rightDist = getDistanceForHeading(rightDiag);

			// A third of the time, look the other way anyways.
			int iSecret = rand() % 10;
			if (iSecret <=2) {
				if (leftDist >= rightDist) {
					targetServoOffset = 45;
				} else {
					targetServoOffset = -45;
				}
			} else {
				if (leftDist <= rightDist) {
					targetServoOffset = 45;
				} else {
					targetServoOffset = -45;
				}
			}
		} else {
			// put in front
			targetServoOffset = 45;
		}
	} else {
		// Position the servo
		if (uServoPosition < 135) {
			targetServoOffset = 45;
		} else {
			targetServoOffset = -135;
		}
	}
	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ updateRanges");
		Serial.print("Current servo position is:");
		Serial.println(uServoPosition);
		Serial.print("Ofsetting by:");
		Serial.println(targetServoOffset);
	}

	if (targetServoOffset > 0) {
		for(int pos = uServoPosition; pos < uServoPosition + targetServoOffset; pos += 5)
		{
			uSensorServo.write(pos);
			delay(30);
		}
		uServoPosition += targetServoOffset;
		delay(100);
	} else {
		for(int pos = uServoPosition; pos > uServoPosition + targetServoOffset; pos -= 5)
		{
			uSensorServo.write(pos);
			delay(30);
		}
		uServoPosition += targetServoOffset;
		delay(100);
	}

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ updateRanges");
		Serial.print("Servo position now is:");
		Serial.println(uServoPosition);
	}


	// Determine where each sensor is pointing relative
	// to the bot.
	int sensor1Heading = 0;
	int sensor2Heading = 0;
	switch (uServoPosition) {
		case 0:
			sensor1Heading = 270;
			sensor2Heading = 90;
			break;
		case 45:
			sensor1Heading = 225;
			sensor2Heading = 45;
			break;
		case 90:
			sensor1Heading = 180;
			sensor2Heading = 0;
			break;
		case 135:
			sensor1Heading = 135;
			sensor2Heading = 315;
			break;
		default:
			if (ENABLE_SERIAL_DEBUG) {
				Serial.println("************************ updateRanges");
				Serial.print("ERROR - uServoPosition in invalid state:");
				Serial.println(uServoPosition);
			}
			return;
	}

	if (ENABLE_SERIAL_DEBUG) {
		int polarHeading1;
		int polarHeading2;
		if (currentOrientation + sensor1Heading >= 360) {
			polarHeading1 = currentOrientation + sensor1Heading - 360;
		} else {
			polarHeading1 = currentOrientation + sensor1Heading;
		}
		if (currentOrientation + sensor2Heading >= 360) {
			polarHeading2 = currentOrientation + sensor2Heading - 360;
		} else {
			polarHeading2 = currentOrientation + sensor2Heading;
		}
		// Round it to the 45 degrees exactitude
		Serial.println("************************ updateRanges");
		Serial.print("Sensor 1 heading: ");
		Serial.println(roundToFactor(polarHeading1, 45));
		Serial.print("Sensor 2 heading: ");
		Serial.println(roundToFactor(polarHeading2, 45));
	}

	/*
	 * Sensor 1. Not needed to update when going
	 * forward.
	 */
	long pingSum = 0;
	int avgPing = 0;
	if (currentMotorState != MOTOR_FORWARD) {
		// read
	//	for (int i = 0; i < USENS_NBSAMPLES; i++) {
	//		pingSum += analogRead(USENSOR1);
	//		delay(1);
	//	}
	//	int avgPing = pingSum / USENS_NBSAMPLES;
		for (int i = 0; i < USENS_NBSAMPLES; i++) {
			pingSum += pulseIn(USENSOR1, HIGH);
		}
		avgPing = pingSum / USENS_NBSAMPLES / 147;


		// write
		updateSingleRange(sensor1Heading, avgPing);
	}


	/*
	 * Sensor 2
	 */
	pingSum = 0;
//	for (int i = 0; i < USENS_NBSAMPLES; i++) {
//		pingSum += analogRead(USENSOR2);
//		delay(1);
//	}
//	avgPing = pingSum / USENS_NBSAMPLES;
	for (int i = 0; i < USENS_NBSAMPLES; i++) {
		pingSum += pulseIn(USENSOR2, HIGH);
	}
	avgPing = pingSum / USENS_NBSAMPLES / 147;

	// write
	updateSingleRange(sensor2Heading, avgPing);


	/*
	 * Debug
	 */
	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("***************************** updateRanges");

		Serial.print("\t\t");
		Serial.println(currentRanges.range0);

		Serial.print("\t");
		Serial.print(currentRanges.range315);
		Serial.print("\t\t");
		Serial.println(currentRanges.range45);

		Serial.print(currentRanges.range270);
		Serial.print("\t\t\t\t");
		Serial.println(currentRanges.range90);

		Serial.print("\t");
		Serial.print(currentRanges.range225);
		Serial.print("\t\t");
		Serial.println(currentRanges.range135);

		Serial.print("\t\t");
		Serial.println(currentRanges.range180);
	}
}


/**
 * Updates a range in the map
 *
 * heading -> The sensor direction, relative to the robot;
 * range -> The range in inches
 */
void updateSingleRange(int heading, int range) {

	// Find the correct heading value by considering the compass value
	// Position 0 is equal to the left of the robot.
	int polarHeading;
	if (currentOrientation + heading >= 360) {
		polarHeading = currentOrientation + heading - 360;
	} else {
		polarHeading = currentOrientation + heading;
	}

	// Round it to the 45 degrees exactitude
	int roundedHeading = roundToFactor(polarHeading, 45);

	switch (roundedHeading) {
		case 0:
		case 360:
			currentRanges.range0 = range;
			break;
		case 45:
			currentRanges.range45 = range;
			break;
		case 90:
			currentRanges.range90 = range;
			break;
		case 135:
			currentRanges.range135 = range;
			break;
		case 180:
			currentRanges.range180 = range;
			break;
		case 225:
			currentRanges.range225 = range;
			break;
		case 270:
			currentRanges.range270 = range;
			break;
		case 315:
			currentRanges.range315 = range;
			break;
		default:
			if (ENABLE_SERIAL_DEBUG) {
				Serial.println("************************ updateSingleRange");
				Serial.print("ERROR - Heading value was resolved to ");
				Serial.println(roundedHeading);
			}
	}
}


int roundToFactor(int value, int roundFactor) {

	// nb of whole times the factor appears in the value
	int nbValues = floor(value / roundFactor);

	// diff between the low end and the value
	int bottomDiff = value - (nbValues * roundFactor);

	// diff between the high end and the value
	int topDiff = ((nbValues + 1) * roundFactor) - value;

	if (topDiff <= bottomDiff) {
		return ((nbValues + 1) * roundFactor);
	} else {
		return (nbValues * roundFactor);
	}
}

/**
 * Makes the robot go forward.
 * time: 0 or nb of milis to go forward.
 * speed: 1-255 speed.
 */
void robotForward(int time, int speed) {

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ robotForward");
		Serial.println("Going forward.");
	}

	if (currentMotorState != MOTOR_FORWARD) {
		digitalWrite(MOTOR_FL_DIRECTION, MOTOR_FL_FORWARD);
		digitalWrite(MOTOR_RL_DIRECTION, MOTOR_RL_FORWARD);
		digitalWrite(MOTOR_FR_DIRECTION, MOTOR_FR_FORWARD);
		digitalWrite(MOTOR_RR_DIRECTION, MOTOR_RR_FORWARD);
		analogWrite(MOTOR_FL_POWER, speed);
		analogWrite(MOTOR_RL_POWER, speed);
		analogWrite(MOTOR_FR_POWER, speed);
		analogWrite(MOTOR_RR_POWER, speed);
		currentMotorState = MOTOR_FORWARD;
	}

	if (time > 0) {
		delay(time);
		analogWrite(MOTOR_FL_POWER, 0);
		analogWrite(MOTOR_RL_POWER, 0);
		analogWrite(MOTOR_FR_POWER, 0);
		analogWrite(MOTOR_RR_POWER, 0);
		currentMotorState = MOTOR_STOPPED;
	}
}



/**
 * Makes the robot go reverse.
 * time: 0 or nb of milis to go forward.
 * speed: 1-255 speed.
 */
void robotBackward(int time, int speed) {

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ robotBackward");
		Serial.println("Going backwards.");
	}

	if (currentMotorState != MOTOR_BACKWARD) {
		digitalWrite(MOTOR_FL_DIRECTION, MOTOR_FL_BACKWARD);
		digitalWrite(MOTOR_RL_DIRECTION, MOTOR_RL_BACKWARD);
		digitalWrite(MOTOR_FR_DIRECTION, MOTOR_FR_BACKWARD);
		digitalWrite(MOTOR_RR_DIRECTION, MOTOR_RR_BACKWARD);
		analogWrite(MOTOR_FL_POWER, speed);
		analogWrite(MOTOR_RL_POWER, speed);
		analogWrite(MOTOR_FR_POWER, speed);
		analogWrite(MOTOR_RR_POWER, speed);
		currentMotorState = MOTOR_BACKWARD;
	}

	if (time > 0) {
		delay(time);
		analogWrite(MOTOR_FL_POWER, 0);
		analogWrite(MOTOR_RL_POWER, 0);
		analogWrite(MOTOR_FR_POWER, 0);
		analogWrite(MOTOR_RR_POWER, 0);
		currentMotorState = MOTOR_STOPPED;
	}
}


/**
 * Makes the robot stop.
 */
void robotStop() {
	analogWrite(MOTOR_FL_POWER, 0);
	analogWrite(MOTOR_RL_POWER, 0);
	analogWrite(MOTOR_FR_POWER, 0);
	analogWrite(MOTOR_RR_POWER, 0);
	currentMotorState = MOTOR_STOPPED;
}




/**
 * Makes the robot turn to a specified heading.
 * heading: 0-359 polar heading to stop at.
 * speed: 1-255 speed.
 */
void robotTurn() {

	int roundedCurrentHeading = getCurrentRoundedHeading();

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("********************************** robotTurn");
		Serial.print("Rounded Current Heading : ");
		Serial.println(roundedCurrentHeading);
		Serial.print("Rounded Target Heading : ");
		Serial.println(currentDirection);
	}

	// Check if the heading is already attained.
	// Check if we are in the right direction.
	unsigned long newTimestamp = millis();
	if (newTimestamp < courseCorrectionTimestamp) {
		// value has overflown.
		courseCorrectionTimestamp = newTimestamp;
		// we can leave early.
		return;
	}

	if (roundedCurrentHeading == currentDirection) {
		courseCorrectionTimestamp = newTimestamp;
		if (currentMotorState != MOTOR_FORWARD) {
			robotStop();
		}
		return;
	}

	if (newTimestamp - courseCorrectionTimestamp >= COURSE_CORRECTION_TIME
		&& (currentMotorState == MOTOR_LEFT || currentMotorState == MOTOR_RIGHT))
	{
		// Abort. We're stuck.
		if (ENABLE_SERIAL_DEBUG) {
			Serial.println("************************ robotTurn");
			Serial.print("Aborting course correction after trying for ");
			Serial.print(COURSE_CORRECTION_TIME);
			Serial.println(" millis.");
		}
		robotBackupAndChooseAnotherPath();
		courseCorrectionTimestamp = newTimestamp;
	}


	// Start turning

	// Figure out the shortest turn. Left or right.
	// 0-> left, 1->right
	int direction;
	if (roundedCurrentHeading < currentDirection) {
		int diff = currentDirection - roundedCurrentHeading;
		if (diff > 180) {
			direction = 0;
		} else {
			direction = 1;
		}
	} else {
		int diff = roundedCurrentHeading - currentDirection;
		if (diff > 180) {
			direction = 1;
		} else {
			direction = 0;
		}
	}

	if (direction == 0) {
		if (currentMotorState != MOTOR_LEFT) {
			if (ENABLE_SERIAL_DEBUG) {
				Serial.println("************************ robotTurn");
				Serial.print("Turning left to heading:");
				Serial.println(currentDirection);
				Serial.print("Current heading:");
				Serial.println(roundedCurrentHeading);
			}
			digitalWrite(MOTOR_FL_DIRECTION, MOTOR_FL_BACKWARD);
			digitalWrite(MOTOR_RL_DIRECTION, MOTOR_RL_BACKWARD);
			digitalWrite(MOTOR_FR_DIRECTION, MOTOR_FR_FORWARD);
			digitalWrite(MOTOR_RR_DIRECTION, MOTOR_RR_FORWARD);
			analogWrite(MOTOR_FL_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_RL_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_FR_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_RR_POWER, COURSE_CORRECTION_SPEED);
			currentMotorState = MOTOR_LEFT;
			delay(500);
		}
	} else {
		if (currentMotorState != MOTOR_RIGHT) {
			if (ENABLE_SERIAL_DEBUG) {
				Serial.println("************************ robotTurn");
				Serial.print("Turning right to heading:");
				Serial.println(currentDirection);
				Serial.print("Current heading:");
				Serial.println(roundedCurrentHeading);
			}
			digitalWrite(MOTOR_FL_DIRECTION, MOTOR_FL_FORWARD);
			digitalWrite(MOTOR_RL_DIRECTION, MOTOR_RL_FORWARD);
			digitalWrite(MOTOR_FR_DIRECTION, MOTOR_FR_BACKWARD);
			digitalWrite(MOTOR_RR_DIRECTION, MOTOR_RR_BACKWARD);
			analogWrite(MOTOR_FL_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_RL_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_FR_POWER, COURSE_CORRECTION_SPEED);
			analogWrite(MOTOR_RR_POWER, COURSE_CORRECTION_SPEED);
			currentMotorState = MOTOR_RIGHT;
			delay(500);
		}
	}
}



int getCurrentRoundedHeading() {
	int roundedCurrentHeading =
			roundToFactor(currentOrientation, 45);
	if (roundedCurrentHeading == 360) {
		roundedCurrentHeading = 0;
	}
	return roundedCurrentHeading;
}



/**
 * This is the main logic of the robot's movements.
 */
void robotMove() {

	// prevent collisions.
	preventColisions();

	robotTurn();

	// Nothing fancy to do. Just go forward
	// if we're stopped.
	if (currentMotorState == MOTOR_STOPPED) {

		choosePath(false);

		robotTurn();

		if (ENABLE_SERIAL_DEBUG) {
			Serial.println("************************ moveRobot");
			Serial.println("All good. Going forward.");
		}

		robotForward(-1, CRUISE_SPEED);
	}
}


/**
 * Chooses a new direction to go to.
 * forceNew: Forces the choice to be slightly different
 * from the last direction, because we suspect something is in our way.
 */
void choosePath(bool forceNew) {

	// The best choice is either full ahead, diag left or diag right.
	int bestChoice = -1;
	int bestChoiceValue = 999;

	for (int i = 0; i < 360; i += 45) {
		if (forceNew && currentDirection == i) {
			continue;
		}

		int turnDiff = currentDirection - i;
		if (turnDiff < 0) {
			turnDiff = i - currentDirection;
		}

		int curLeft = i - 45;
		if (curLeft < 0) {
			curLeft += 360;
		}
		int curRight = i + 45;
		if (curRight >= 360) {
			curRight -= 360;
		}

		int curDist = getDistanceForHeading(i);
		int curDistLeft = getDistanceForHeading(curLeft);
		int curDistRight = getDistanceForHeading(curRight);

		if (curDist >= (MIN_DISTANCE_BUFFER * MIN_DISTANCE_FACTOR)
			&& curDistLeft >= (MIN_DISTANCE_BUFFER_DIAG * MIN_DISTANCE_FACTOR)
			&& curDistRight >= (MIN_DISTANCE_BUFFER_DIAG * MIN_DISTANCE_FACTOR))
		{
			if (turnDiff < bestChoiceValue) {
				bestChoiceValue = turnDiff;
				bestChoice = i;
			}
		} else {
			if (ENABLE_SERIAL_DEBUG) {
				Serial.println("************************ choosePath");
				Serial.print("Rejecting option ");
				Serial.println(i);
				Serial.print("Distance ahead: ");
				Serial.println(curDist);
				Serial.print("Distance left: ");
				Serial.println(curDistLeft);
				Serial.print("Distance left: ");
				Serial.println(curDistRight);
				Serial.print("Turn difference: ");
				Serial.println(turnDiff);
				Serial.print("Best heading: ");
				Serial.println(bestChoice);
				Serial.print("Best heading turn difference: ");
				Serial.println(bestChoiceValue);
			}
		}
	}

	if (ENABLE_SERIAL_DEBUG) {
		Serial.println("************************ choosePath");
		Serial.println("Path chosen.");
		Serial.print("Best heading: ");
		Serial.println(bestChoice);
		Serial.print("Best heading turn difference: ");
		Serial.println(bestChoiceValue);
	}

	if (bestChoice >= 0) {
		currentDirection = bestChoice;
	} else {
		// This is a problem. It can happen when all directions are
		// perceived as having an obstacle. Just move backward.... whatever.
		currentDirection += 180;
		if (currentDirection >= 360) {
			currentDirection -= 360;
		}
	}
}






/**
 * Checks if something is blocking the robot from going where
 * it intends to.
 * Returns true if a problem was detected.
 */
void preventColisions() {

	if (currentMotorState != MOTOR_FORWARD) {
		return;
	}

	// Check straight ahead
	if (getDistanceForHeading(currentDirection) <= MIN_DISTANCE_BUFFER) {
		if (ENABLE_SERIAL_DEBUG) {
			Serial.println("************************ movepreventColisions");
			Serial.println("Changing course. Collision detected ahead.");
		}

		robotBackupAndChooseAnotherPath();

		// We can leave early.
		return;
	}

	// Check diagonals forward
	int leftDiag = currentDirection - 45;
	if (leftDiag < 0) {
		leftDiag = 360 - leftDiag;
	}
	int rightDiag = currentDirection + 45;
	if (rightDiag >= 360) {
		rightDiag = rightDiag - 360;
	}

	if (getDistanceForHeading(leftDiag) <= MIN_DISTANCE_BUFFER_DIAG
			|| getDistanceForHeading(rightDiag) <= MIN_DISTANCE_BUFFER_DIAG)
	{
		if (ENABLE_SERIAL_DEBUG) {
			Serial.println("************************ movepreventColisions");
			Serial.println("Changing course. Collision detected ahead.");
		}
		robotBackward(500, COURSE_CORRECTION_SPEED);
		choosePath(true);
		robotTurn();
	}
}

void robotBackupAndChooseAnotherPath() {
	// stop now!
	robotStop();

	// back up a little
	robotBackward(500, COURSE_CORRECTION_SPEED);

	// Scan the area.
	for (int i = 0; i < 4; i++) {
		updateRanges();
	}

	// Choose another path.
	choosePath(true);
}











/**
 * Returns the range in a given heading.
 */
int getDistanceForHeading(int heading) {
	switch (heading) {
		case 0:
		case 360:
			return currentRanges.range0;
			break;
		case 45:
			return currentRanges.range45;
			break;
		case 90:
			return currentRanges.range90;
			break;
		case 135:
			return currentRanges.range135;
			break;
		case 180:
			return currentRanges.range180;
			break;
		case 225:
			return currentRanges.range225;
			break;
		case 270:
			return currentRanges.range270;
			break;
		case 315:
			return currentRanges.range315;
			break;
	}
	return 999;
}




void takeHttpRequest() {
//	Client client = server.available();
//	  if (client) {
//		if (ENABLE_SERIAL_DEBUG) {
//			Serial.println("Web request present.");
//			delay(2000);
//		}
//
//		char buffer[2048];
//
//		// an http request ends with a blank line
//		boolean currentLineIsBlank = true;
//		int i = 0;
//		while (client.connected() && i < 2048) {
//		  if (client.available()) {
//
//			buffer[i] = client.read();
//
//			// if you've gotten to the end of the line (received a newline
//			// character) and the line is blank, the http request has ended,
//			// so you can send a reply
//			if (buffer[i] == '\n' && currentLineIsBlank) {
//			  // send a standard http response header
//			  client.println("HTTP/1.1 200 OK");
//			  client.println("Content-Type: text/html");
//			  client.println();
//
//			  client.print("<html><head><title>Robot 02 Control Panel</title></head><body>");
//
//			  client.print("<h1>Robot status</h1>");
//			  client.print("<p>Current Motor Status : ");
//			  switch (currentMotorState) {
//			  case MOTOR_FORWARD:
//				  client.print("FORWARD</p>");
//				  break;
//			  case MOTOR_BACKWARD:
//				  client.print("BACKWARDS</p>");
//				  break;
//			  case MOTOR_STOPPED:
//				  client.print("STOPPED</p>");
//				  break;
//			  case MOTOR_LEFT:
//				  client.print("TURNING LEFT</p>");
//				  break;
//			  case MOTOR_RIGHT:
//				  client.print("TURNING RIGHT</p>");
//				  break;
//			  }
//			  client.print("<p>Current Compass Position : ");
//			  client.print(currentOrientation);
//			  client.print("</p><p>Current Direction : ");
//			  client.print(currentDirection);
//			  client.print("</p><p>Environment distances :</br><table border=\"3\"><row><column>");
//			  client.print(getDistanceForHeading(315));
//			  client.print("</column><column>");
//			  client.print(getDistanceForHeading(0));
//			  client.print("</column><column>");
//			  client.print(getDistanceForHeading(45));
//			  client.print("</column></row><row><column>");
//			  client.print(getDistanceForHeading(270));
//			  client.print("</column><column>&nbsp;</column><column>");
//			  client.print(getDistanceForHeading(90));
//			  client.print("</column></row><row><column>");
//			  client.print(getDistanceForHeading(225));
//			  			  client.print("</column><column>");
//			  client.print(getDistanceForHeading(180));
//			  			  client.print("</column><column>");
//			  client.print(getDistanceForHeading(135));
//			  client.print("</column></row></table></p></body></html>");
//			  break;
//			}
//			if (buffer[i] == '\n') {
//			  // you're starting a new line
//			  currentLineIsBlank = true;
//			}
//			else if (buffer[i] != '\r') {
//			  // you've gotten a character on the current line
//			  currentLineIsBlank = false;
//			}
//			i++;
//		  }
//		}
//		// give the web browser time to receive the data
//		delay(500);
//		// close the connection:
//		client.stop();
//	  }
}




int main(void) {
	init();
	setup();
	for (;;)
		loop();
	return 0;
}

extern "C" void __cxa_pure_virtual() {
	cli();
	for (;;)
		;
}

