#ifndef _DUAL_AXIS_TRACKER
#define _DUAL_AXIS_TRACKER

#include <Arduino.h>


#define IDX_ENA_PIN 0
#define IDX_EXTEND_PIN 1
#define IDX_RETRACT_PIN 2

#define PITCH_ENA_PIN 7
#define PITCH_EXTEND_PIN 6
#define PITCH_RETRACT_PIN 5

#define ROLL_ENA_PIN 13
#define ROLL_EXTEND_PIN 12
#define ROLL_RETRACT_PIN 11

#define CMD_TEST_PITCH 	"testp"
#define CMD_TEST_ROLL	"testr"
#define CMD_READA		"reada"
#define CMD_PITCH   	"pitch"
#define CMD_ROLL		"roll"
#define CMD_MOVE_TO		"moveto"
#define CMD_UNKNOWN		"wtf"

#define CMD_TEST_PITCH_ID 	1
#define CMD_TEST_ROLL_ID	2
#define CMD_READA_ID		3
#define CMD_PITCH_ID   		4
#define CMD_ROLL_ID			5
#define CMD_MOVE_TO_ID		6
#define CMD_UNKNOWN_ID		7


#define REQUIRED_ANGLE_PRECISION 2.0
#define MIN_ACTUATOR_BURST_MILLIS 250

// 45 degrees in 10 seconds
#define ANGULAR_VELOCITY_PITCH 4.5

// 80 degrees in 10 seconds
#define ANGULAR_VELOCITY_ROLL 8.0


enum ActuatorMoveDirection { MoveNone, MoveExtend, MoveRetract };

char buffer[100];

class Actuator {
	private:		
		uint8_t pins[3];		
		
	public:										
		Actuator(int ena, int extend, int retract) {									

			pins[IDX_ENA_PIN] = ena;
			pins[IDX_EXTEND_PIN] = extend;
			pins[IDX_RETRACT_PIN] = retract;			
			 
			for(int i=0; i<3; i++) {
				pinMode(pins[i], OUTPUT);			// initialize digital pins as outputs
				digitalWrite(pins[i], LOW);			// write LOW to every pin
			}		  			
		}					

		int getEnablePin() { return pins[IDX_ENA_PIN]; }
		int getExtendPin() { return pins[IDX_EXTEND_PIN]; }
		int getRetractPin() { return pins[IDX_RETRACT_PIN]; }
		
		void extend(int millis) {				
			digitalWrite(pins[IDX_EXTEND_PIN], HIGH);
			digitalWrite(pins[IDX_RETRACT_PIN], LOW);			
  			digitalWrite(pins[IDX_ENA_PIN], HIGH);
			
			delay(millis);
			stop();
		}

		void retract(int millis) {
			digitalWrite(pins[IDX_EXTEND_PIN], LOW);
			digitalWrite(pins[IDX_RETRACT_PIN], HIGH);			
  			digitalWrite(pins[IDX_ENA_PIN], HIGH);

			delay(millis);
			stop();
		}

		void stop() {
			for(int i=0; i<3; i++) {			
				digitalWrite(pins[i], LOW);
			}
		}
};


class AbstractCommand {
	protected:
		int commandId;

	public:	 
		virtual void execute() = 0;
		int getCommandId() { return commandId; }
};

class AbstractAccelerometer {
	public:
		virtual void getXYZ(float *x, float *y, float *z) = 0;

		void getPitchAndRoll(float *pitch, float *roll) {
			float ax, ay, az;
			
			getXYZ(&ax, &ay, &az);					// obtain angles

			*pitch = atan2(-ax, az) * 180 / M_PI; 	// rotation on Y axis
    		*roll = atan2(-ay, az) * 180 / M_PI;  	// rotation on X axis 
		}		
};

class TestAccelerometer : public AbstractAccelerometer {
	public:
		float ax, ay, az;

		TestAccelerometer(float x, float y, float z) {
			ax = x;
			ay = y;
			az = z;
		} 

		void getXYZ(float *x, float *y, float *z) {
			*x = ax;
			*y = ay;
			*z = az;
		}
};


class TestActuatorCommand : public AbstractCommand {
	protected:		
		Actuator *pactuator;		

	public:
		TestActuatorCommand(Actuator *p, int id) {			
			pactuator = p;
			commandId = id;
		}

		void execute() override {
			pactuator->extend(2000);
			pactuator->retract(2000);
		}				
};


class ActuatorCommand : public AbstractCommand {
	private:		
		Actuator * actuator;
		AbstractAccelerometer * accel;		
		String message;
		float goal;
		float angularVelocity;			// degree / second


	public:
		ActuatorCommand(Actuator *pact, AbstractAccelerometer * paccel, float av, int id) {			
			actuator = pact;
			accel = paccel;
			angularVelocity = av;
			commandId = id;
			message = "boo!";
		}

		float getGoalAngle() { return goal; }
		void setGoalAngle(float angle) { goal = angle; }
		float getAngularVelocity() { return angularVelocity; }
		String getMessage() { return message; }		

		float absDiff(float angle) {
			float diff = angle - goal;
			return diff >= 0 ? diff : -1 * diff;
		}

		int estimateTime2TravelFrom(float angle) {
			float angle2Travel = angle - goal;			    // degrees
			float seconds = angle2Travel / angularVelocity;	// time in seconds to rotate $angle2Travel degrees
			if(seconds < 0)
				seconds *= -1;
			return (int)(1000 * seconds);			    	// return time in milliseconds					
		}


		void execute() override {
			float pitch, roll, angle;							

			for(int i = 0; i < 4; i++) {					// move + 3 attempts to adjust position
				accel->getPitchAndRoll(&pitch, &roll);							
				
				if(commandId == CMD_PITCH_ID) {			
					angle = pitch;					
				} 
				else if(commandId == CMD_ROLL_ID) {				
					angle = roll;					
				} else {					
					Serial.print("Error: unknown command: "); Serial.print(commandId);
					Serial.println("");
					return;
				}
				
				float diff = absDiff(angle);										

				if(diff < REQUIRED_ANGLE_PRECISION) {																	
					Serial.print("angles diff is too small: "); Serial.print(diff);					
					Serial.print(" angle: "); Serial.print(angle);					
					Serial.print(" goal:"); Serial.print(goal);					
					Serial.println("");
					return;
				}				
							
				int millis = estimateTime2TravelFrom(angle);				

				if(millis < MIN_ACTUATOR_BURST_MILLIS) {
					Serial.print("estimated time to move is too small: "); Serial.print(millis); Serial.print(" millis");					
					Serial.println("");					
					return;			
				}									
				
				if( angle > goal ) {		// actuator extending					
					Serial.print(" extend "); Serial.print(millis);
					Serial.println("");
					actuator->extend(millis);
				} else  {					// actuator retracting
					Serial.print(" retract "); Serial.print(millis);
					Serial.println("");
					actuator->retract(millis);
				} 
				
			} // end for			
		}		
};


class ReadAcceleratorCommand : public AbstractCommand {
	private:
		AbstractAccelerometer *paccel;
		float x, y, z, pitch, roll;

	public:
		ReadAcceleratorCommand(AbstractAccelerometer *p) {
			paccel = p;
			commandId = CMD_READA_ID;
		}

		void execute() override {						
			paccel->getXYZ(&x, &y, &z);
			paccel->getPitchAndRoll(&pitch, &roll);	

			Serial.print("x: "); Serial.print(x);
			Serial.print(", y: "); Serial.print(y);
			Serial.print(", z: "); Serial.print(z);
			Serial.print(", pitch: "); Serial.print(pitch);
			Serial.print(", roll: ");Serial.print(roll);
        	Serial.println("");					
		}		

		float getX() { return x; }
		float getY() { return y; }
		float getZ() { return z; }
		float getPitch() { return pitch; }
		float getRoll() { return roll; }
		
		const char * getPrintableResult() {			
			String s_x = String(x, 2);
			String s_y = String(y, 2);
			String s_z = String(z, 2);
			String s_pitch = String(pitch, 2);
			String s_roll = String(roll, 2);
			
			memset(buffer, 0, sizeof(buffer));			

			sprintf(buffer, "x: %s, y: %s, z: %s, pitch: %s, roll: %s", 
				s_x.c_str(), 
				s_y.c_str(), 
				s_y.c_str(), 
				s_pitch.c_str(), 
				s_roll.c_str());			
						
			return buffer;			
		}
};


class MovetoCommand : public AbstractCommand {
	private:
		ActuatorCommand * pitchCommand;
		ActuatorCommand * rollCommand;

		float pitchAngle, rollAngle;

	public:
		MovetoCommand(ActuatorCommand * pitch, ActuatorCommand * roll) {
			commandId = CMD_MOVE_TO_ID;

			pitchCommand = pitch;
			rollCommand = roll;
		}

		void setPitchAngle(float angle) { pitchAngle = angle; }
		void setRollAngle(float angle) { rollAngle = angle; }

		void execute() override {			
			if( pitchAngle != 0.0 || rollAngle != 0.0 ) {
				pitchCommand->setGoalAngle(pitchAngle);						
				rollCommand->setGoalAngle(rollAngle);

				pitchCommand->execute();
				rollCommand->execute();			
			} else {
				Serial.println("Error: one or both angles are 0!");
			}			
		}
};

class UnknownCommand : public AbstractCommand {
	public:
		UnknownCommand() {
			commandId = CMD_UNKNOWN_ID;
		}

		void execute() override {						
			
		}		
};


class TrackerController {
	private:
		AbstractAccelerometer * accel;
		Actuator * pitchActuator;
		Actuator * rollActuator;

		TestActuatorCommand * pitchTestCommand;
		TestActuatorCommand * rollTestCommand;
		ActuatorCommand * pitchCommand;
		ActuatorCommand * rollCommand;
		ReadAcceleratorCommand * readAcceleratorCommand;	
		MovetoCommand * movetoCommand;	
		UnknownCommand * unknownCommand;

	public:
		TrackerController(AbstractAccelerometer * accelPtr, Actuator * pitchActuatorPtr, Actuator * rollActuatorPtr) {
			accel = accelPtr;
			pitchActuator = pitchActuatorPtr;
			rollActuator = rollActuatorPtr;

			pitchTestCommand = new TestActuatorCommand(pitchActuator, CMD_TEST_PITCH_ID);
			rollTestCommand = new TestActuatorCommand(rollActuator, CMD_TEST_ROLL_ID);
			pitchCommand = new ActuatorCommand(pitchActuator, accel, ANGULAR_VELOCITY_PITCH, CMD_PITCH_ID);			
			rollCommand = new ActuatorCommand(rollActuator, accel, ANGULAR_VELOCITY_ROLL, CMD_ROLL_ID);			
			readAcceleratorCommand = new ReadAcceleratorCommand(accel);
			movetoCommand = new MovetoCommand(pitchCommand, rollCommand);
			unknownCommand = new UnknownCommand();
		}

		~TrackerController() {
			delete pitchTestCommand;
			delete rollTestCommand;
			delete pitchCommand;
			delete rollCommand;
			delete readAcceleratorCommand;	
			delete movetoCommand;
			delete unknownCommand;
		}

		const AbstractCommand * parseCommandString(String str) {
			AbstractCommand * command = unknownCommand;

			if(str.endsWith(CMD_TEST_PITCH)) {				
				command = pitchTestCommand;
			} else if(str.endsWith(CMD_TEST_ROLL)) {				
				command = rollTestCommand;
			} else if (str.endsWith(CMD_READA)) {				
				command = readAcceleratorCommand;
			} else if(str.startsWith(CMD_PITCH)) {
				float pitchAngle = str.substring(5).toFloat();
				pitchCommand->setGoalAngle(pitchAngle);
				command = pitchCommand;
			} else if(str.startsWith(CMD_ROLL)) {
				float rollAngle = str.substring(4).toFloat();
				rollCommand->setGoalAngle(rollAngle);
				command = rollCommand;
			} else if(str.startsWith(CMD_MOVE_TO)) {
				// moveto 0.0:0.0
				// moveto $pitch:$roll
				int idxCommas = str.indexOf(':');				

				if(idxCommas > 0) {
					String pitchStr = str.substring(6, idxCommas);
					String rollStr = str.substring(idxCommas + 1);

					movetoCommand->setPitchAngle(pitchStr.toFloat());
					movetoCommand->setRollAngle(rollStr.toFloat());
					command = movetoCommand;					 	
				} 											
			} 					

			return command;
		}

};


#endif