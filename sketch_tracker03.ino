#include "tracker.h"

#ifdef TEST_TRACKER
#include <AUnit.h>

test(Actuator) {
  Actuator a = Actuator(7, 6, 5);

  assertEqual(7, a.getEnablePin());
  assertEqual(6, a.getExtendPin());
  assertEqual(5, a.getRetractPin());

  TestActuatorCommand tac = TestActuatorCommand(&a, CMD_TEST_PITCH);
  assertEqual(CMD_TEST_PITCH, tac.getCommandId());
  //tac.execute();
}

test(ActuatorCommand) {
  Actuator a = Actuator(7, 6, 5);
  TestAccelerometer accel = TestAccelerometer(0, 0, 256);  
  ActuatorCommand pitch = ActuatorCommand(&a, &accel, ANGULAR_VELOCITY_PITCH, CMD_PITCH);

  assertEqual(CMD_PITCH, pitch.getCommandId());

  pitch.setGoalAngle(0.0);  
  assertEqual(0.0, pitch.absDiff(0.0));
  assertEqual(1.0, pitch.absDiff(1.0));
  assertEqual(1.0, pitch.absDiff(-1.0));

  pitch.setGoalAngle(45);
  int milli = pitch.estimateTime2TravelFrom(0);
  assertEqual(10000, milli);      // full range 45% - in 10 seconds

  pitch.setGoalAngle((float)45/2);
  assertEqual(22.5, pitch.getGoalAngle());

  milli = pitch.estimateTime2TravelFrom(0);
  assertEqual(milli, 5000);       // half of the full range
  //pitch.execute();  
  //String s1 = pitch.getMessage();
  //Serial.println(s1);
  //assertEqual(s1, "pitch diff is too small");
  

  //pitch.setGoalAngle(5.0);  
  //pitch.execute();
  //assertEqual(pitch.getMessage(), "estimated time 2 travel eq 0");
  //String s2 = pitch.getMessage();
  //Serial.println(s2);
}

test(ReadAcceleratorCommand) {
  TestAccelerometer accel = TestAccelerometer(0, 0, 0);  

  float x, y, z;

  accel.getXYZ(&x, &y, &z);

  assertEqual(0.0, x);
  assertEqual(0.0, y);
  assertEqual(0.0, z);

  float pitch, roll;

  accel.getPitchAndRoll(&pitch, &roll);

  assertEqual(0.0, pitch);
  assertEqual(0.0, roll);
}

test(TrackerController) {  
  Actuator pitchA = Actuator(7, 6, 5);
  Actuator rollA = Actuator(7, 6, 5);
  TestAccelerometer accel = TestAccelerometer(0, 0, 10);
  
  TrackerController ctrl = TrackerController(&accel, &pitchA, &rollA);
  AbstractCommand * cmd = NULL;

  cmd = ctrl.parseCommandString("foo");
  assertEqual(CMD_UNKNOWN, cmd->getCommandId());

  //cmd = ctrl.parseCommandString(CMD_TEST_PITCH);
  //assertEqual(CMD_TEST_PITCH, cmd->getCommandId());

  //cmd = ctrl.parseCommandString(CMD_TEST_ROLL);
  //assertEqual(CMD_TEST_ROLL, cmd->getCommandId());

  cmd = ctrl.parseCommandString(CMD_READA);
  assertEqual(CMD_READA, cmd->getCommandId());
  ReadAcceleratorCommand * readAcceleratorCommandPtr = (ReadAcceleratorCommand *)cmd;
  Serial.println(readAcceleratorCommandPtr->getPrintableResult());
}

void setup() {
  Serial.begin(9600);  
  Serial.println(F("Testing..."));

  //Serial.readStringUntil(char terminator)
  //aunit::TestRunner::exclude("Actuator");
  //aunit::TestRunner::exclude("ReadAcceleratorCommand");
  aunit::TestRunner::include("ActuatorCommand");
}

void loop() {    
  aunit::TestRunner::run();  
}

#else

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>


Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified();

class Accelerometer : public AbstractAccelerometer {
  private:
    Adafruit_ADXL345_Unified * _adxl;

	public:
    Accelerometer(Adafruit_ADXL345_Unified *padxl) {
      _adxl = padxl;
    }

		void getXYZ(float *x, float *y, float *z) override {
      sensors_event_t event;   
      _adxl->getEvent(&event);

      *x = event.acceleration.x;
      *y = event.acceleration.y;
      *z = event.acceleration.z;  
    }		
};

Accelerometer * accel = NULL;
Actuator * pitchA = NULL;
Actuator * rollA = NULL;
TrackerController * ctrl = NULL;

void setup() {
  Serial.begin(9600);  

  if(!adxl.begin()) {
    Serial.println("No ADXL345 sensor detected.");
    while(1);
  } else {
    Serial.println(F("Accelerometer started"));
  }  

  accel = new Accelerometer(&adxl);
  pitchA = new Actuator(PITCH_ENA_PIN, PITCH_EXTEND_PIN, PITCH_RETRACT_PIN);
  rollA = new Actuator(ROLL_ENA_PIN, ROLL_EXTEND_PIN, ROLL_RETRACT_PIN);
  ctrl = new TrackerController(accel, pitchA, rollA);
}


void loop() {    
  AbstractCommand * pcommand = NULL;    

  if(Serial.available() > 0) {
    String line = Serial.readString();
    line.trim();    
        
    Serial.print("got command: ");
    Serial.println(line);     

    pcommand = ctrl->parseCommandString(line);    
    int c_id = pcommand->getCommandId();    

    switch( c_id ) {
      case CMD_TEST_PITCH_ID:
      case CMD_TEST_ROLL_ID:        
        Serial.println("will test a");
        pcommand->execute();
        break;

      case CMD_READA_ID:        
        Serial.println("will reada");        
        pcommand->execute();                
        break;

      case CMD_PITCH_ID:      
        Serial.println("will pitch");        
        pcommand->execute();
        break;

      case CMD_ROLL_ID:
        Serial.println("will roll");
        pcommand->execute();
        break;

      case CMD_MOVE_TO_ID:
        Serial.println("will move to");
        pcommand->execute();
        break;
        
      default:
        Serial.println("wtf?");
        break;
    }        
  }  
}
    
#endif




















