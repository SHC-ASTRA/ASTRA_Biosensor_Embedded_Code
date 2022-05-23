#include <Arduino.h>
#include <AccelStepper.h>
#include <PWMServo.h>
#include "SCMD.h"
#include "SCMD_config.h"
#include <Wire.h>

const unsigned int stepPin = 34;
const unsigned int dirPin = 33;

const unsigned int pump1Pwm = 15;
const unsigned int pump2Pwm = 8;
const unsigned int pump3Pwm = 24;
const unsigned int pumpDir = 25;
const unsigned int fan1Pin = 39;
const unsigned int fan2Pin = 37;
const unsigned int fan3Pin = 35;
const unsigned int actuatorPwm = 7;
const unsigned int actuatorDir = 6;

const unsigned int servo1Pin = 0;
const unsigned int servo2Pin = 1;
const unsigned int servo3Pin = 2;
const unsigned int servo4Pin = 3;
const unsigned int servo5Pin = 4;

int servo1RestPos = 90;
int servo2RestPos = 90;
int servo3RestPos = 90;
int servo4RestPos = 90;
int servo5RestPos = 90;

float pumpMaxRuntime = 2.0;
float fanMaxRuntime = 20.0;
float actuatorMaxRuntime = 11;

int microstepping = 32;
int stepsPerRevolution = 200;
int numCuvettes = 10;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

PWMServo servo1;
PWMServo servo2;
PWMServo servo3;
PWMServo servo4;
PWMServo servo5;

void parseCommand(String command);
void printHelp();
void runEquipment(float value, float maximum, unsigned int pin, String name);
void runEquipmentWithDir(float value, float maximum, unsigned int pin, unsigned int dirPin, bool dir, String name, String trueAction, String falseAction);
void runServo(int value, int min, int max, PWMServo& servo, String name);

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(10000);

  while (!Serial)
  {
    /* code */
  }

  Serial.println(" <<<< Bootup begin! >>>> ");

  Serial.print("Configuring GPIO pins...");

  pinMode(pump1Pwm, OUTPUT);
  pinMode(pump2Pwm, OUTPUT);
  pinMode(pump3Pwm, OUTPUT);
  pinMode(pumpDir, OUTPUT);
  pinMode(fan1Pin, OUTPUT);
  pinMode(fan2Pin, OUTPUT);
  pinMode(fan3Pin, OUTPUT);
  pinMode(actuatorPwm, OUTPUT);
  pinMode(actuatorDir, OUTPUT);
  Serial.println("Done.");

  Serial.print("Configuring stepper motor...");
  stepper.setMaxSpeed(20 * microstepping);
  stepper.setAcceleration(5 * microstepping);
  Serial.println("Done.");

  Serial.print("Configuring servo motors...");
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo3.attach(servo3Pin);
  servo4.attach(servo4Pin);
  servo5.attach(servo5Pin);
  servo1.write(servo1RestPos);
  servo2.write(servo2RestPos);
  servo3.write(servo3RestPos);
  servo4.write(servo4RestPos);
  servo5.write(servo5RestPos);
  Serial.println("Done.");

  Serial.println("Ready!");
  Serial.println("");

  printHelp();
}

void loop()
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
  }
}

void parseCommand(String command)
{
  if (command.length() > 0)
  {
    if (command.indexOf((char)8) > 0)
    {
      Serial.println("Backspace detected in command sequence, discarding input!");
      return;
    }
    String exec = command.substring(0, min(2, command.length() - 1));
    String valStr = command.substring(2);

    if (exec.startsWith("h"))
    {
      printHelp();
    }
    else if (exec.equals("p1"))
    {
      float value = valStr.toFloat();

      bool dir = value > 0;
      value = abs(value);

      runEquipmentWithDir(value, pumpMaxRuntime, pump1Pwm, pumpDir, dir, "Pump 1", "Dispensing", "Suctioning");

      Serial.println("Done.");
    }
    else if (exec.equals("p2"))
    {
      float value = valStr.toFloat();

      bool dir = value > 0;
      value = abs(value);

      runEquipmentWithDir(value, pumpMaxRuntime, pump2Pwm, pumpDir, dir, "Pump 2", "Dispensing", "Suctioning");

      Serial.println("Done.");
    }
    else if (exec.equals("p3"))
    {
      float value = valStr.toFloat();

      bool dir = value > 0;
      value = abs(value);

      runEquipmentWithDir(value, pumpMaxRuntime, pump3Pwm, pumpDir, dir, "Pump 3", "Dispensing", "Suctioning");

      Serial.println("Done.");
    }
    else if (exec.equals("pp"))
    {
      bool dir = false;

      Serial.print("Suctioning ");
      Serial.print("all pumps");
      Serial.print(" for ");
      Serial.print("30");
      Serial.print(" seconds...");

      digitalWrite(pumpDir, dir);
      digitalWrite(pump1Pwm, HIGH);
      digitalWrite(pump2Pwm, HIGH);
      digitalWrite(pump3Pwm, HIGH);
      delay((uint32_t)(30 * 1000));
      digitalWrite(pump1Pwm, LOW);
      digitalWrite(pump2Pwm, LOW);
      digitalWrite(pump3Pwm, LOW);

      Serial.println("Done.");
    }
    else if (exec.equals("f1"))
    {
      float value = valStr.toFloat();

      runEquipment(value, fanMaxRuntime, fan1Pin, "Fan 1");

      Serial.println("Done.");
    }
    else if (exec.equals("f2"))
    {
      float value = valStr.toFloat();

      runEquipment(value, fanMaxRuntime, fan2Pin, "Fan 2");

      Serial.println("Done.");
    }
    else if (exec.equals("f3"))
    {
      float value = valStr.toFloat();

      runEquipment(value, fanMaxRuntime, fan3Pin, "Fan 3");

      Serial.println("Done.");
    }
    else if (exec.equals("at"))
    {
      float value = valStr.toFloat();

      bool dir = value > 0;
      value = abs(value);

      runEquipmentWithDir(value, actuatorMaxRuntime, actuatorPwm, actuatorDir, !dir, "Actuator", "Retracting", "Extending");

      Serial.println("Done.");
    }
    else if (exec.equals("s1"))
    {
      int value = valStr.toInt();

      runServo(value, 0, 180, servo1, "cup 1 servo");

      Serial.println("Done.");
    }
    else if (exec.equals("s2"))
    {
      int value = valStr.toInt();

      runServo(value, 0, 180, servo2, "cup 2 servo");

      Serial.println("Done.");
    }
    else if (exec.equals("s3"))
    {
      int value = valStr.toInt();

      runServo(value, 0, 180, servo3, "cup 3 servo");

      Serial.println("Done.");
    }
    else if (exec.equals("s4"))
    {
      int value = valStr.toInt();

      runServo(value, 0, 180, servo4, "capping servo");

      Serial.println("Done.");
    }
    else if (exec.equals("s5"))
    {
      int value = valStr.toInt();

      runServo(value, 0, 180, servo5, "microscope servo");

      Serial.println("Done.");
    }
    else if (exec.equals("ca"))
    {
      float value = valStr.toFloat();
      long pos = value * ((float)(microstepping * stepsPerRevolution)) / numCuvettes;

      float nowpos = -(float)stepper.currentPosition() * (float)numCuvettes / ((float)microstepping * (float)stepsPerRevolution);

      Serial.print("Moving carousel from ");
      Serial.print(nowpos);
      Serial.print(" cuvette spaces to ");
      Serial.print(value);
      Serial.print(" cuvette spaces...");

      stepper.moveTo(-pos);

      while (stepper.isRunning())
      {
        stepper.run();
      }

      Serial.println("Done.");
    }
    else if (exec.equals("cr"))
    {
      float value = valStr.toFloat();
      long pos = value * ((float)(microstepping * stepsPerRevolution)) / numCuvettes;

      float nowpos = -(float)stepper.currentPosition() * (float)numCuvettes / ((float)microstepping * (float)stepsPerRevolution);

      Serial.print("Moving carousel from ");
      Serial.print(nowpos);
      Serial.print(" cuvette spaces to ");
      Serial.print(nowpos + value);
      Serial.print(" cuvette spaces...");

      stepper.move(-pos);

      while (stepper.isRunning())
      {
        stepper.run();
      }

      Serial.println("Done.");
    }
    else
    {
      Serial.print("Error, unrecognized command \"");
      Serial.print(exec);
      Serial.println("\". Use the \"h\" command to view help.");
    }
  }
}

void runServo(int value, int min, int max, PWMServo& servo, String name)
{
  if (value < min)
  {
    Serial.print("Error, ");
    Serial.print(value);
    Serial.print(" degrees is below min of ");
    Serial.print(min);
    Serial.println(" degrees!");
    return;
  }
  else if (value > max)
  {
    Serial.print("Error, ");
    Serial.print(value);
    Serial.print(" degrees is above max of ");
    Serial.print(max);
    Serial.println(" degrees!");
    return;
  }

  uint8_t servoPos = servo.read();

  Serial.print("Moving ");
  Serial.print(name);
  Serial.print(" from ");
  Serial.print(servoPos);
  Serial.print(" degrees to ");
  Serial.print(value);
  Serial.print(" degrees...");

  for (uint8_t newServoPos = servoPos; newServoPos != value; newServoPos += value > servoPos ? 1 : -1)
  {
    servo.write(newServoPos);
    delay(10);
  }
}

void runEquipment(float value, float maximum, unsigned int pin, String name)
{
  if (value > maximum)
  {
    Serial.print("Error, ");
    Serial.print(value);
    Serial.print(" seconds is above max of ");
    Serial.print(maximum);
    Serial.println(" seconds!");
    return;
  }

  Serial.print("Running ");
  Serial.print(name);
  Serial.print(" for ");
  Serial.print(value);
  Serial.print(" seconds...");

  digitalWrite(pin, HIGH);
  delay((uint32_t)(value * 1000));
  digitalWrite(pin, LOW);
}

void runEquipmentWithDir(float value, float maximum, unsigned int pin, unsigned int dirPin, bool dir, String name, String trueAction, String falseAction)
{
  if (value > maximum)
  {
    Serial.print("Error, ");
    Serial.print(value);
    Serial.print(" seconds is above max of ");
    Serial.print(maximum);
    Serial.println(" seconds!");
    return;
  }

  Serial.print(dir ? trueAction : falseAction);
  Serial.print(" ");
  Serial.print(name);
  Serial.print(" for ");
  Serial.print(value);
  Serial.print(" seconds...");

  digitalWrite(dirPin, dir);
  delay(1);
  digitalWrite(pin, HIGH);
  delay((uint32_t)(value * 1000));
  digitalWrite(pin, LOW);
}

void printHelp()
{
  Serial.println(" -------------- ");
  Serial.println("Commands");
  Serial.println("h - view this help message");
  Serial.println("p1 X - Run pump 1 for X seconds. Max is 2. (decimals allowed)");
  Serial.println("       Positve seconds dispense. Negative seconds suction.");
  Serial.println("p2 X - Run pump 2 for X seconds. Max is 2. (decimals allowed)");
  Serial.println("       Positve seconds dispense. Negative seconds suction.");
  Serial.println("p3 X - Run pump 3 for X seconds. Max is 2. (decimals allowed)");
  Serial.println("       Positve seconds dispense. Negative seconds suction.");
  Serial.println("pp   - Run all pumps in reverse for 30 seconds.");
  Serial.println("f1 X - Run Fan 1 for X seconds. Max is 20. (decimals allowed)");
  Serial.println("f2 X - Run Fan 2 for X seconds. Max is 20. (decimals allowed)");
  Serial.println("f3 X - Run Fan 3 for X seconds. Max is 20. (decimals allowed)");
  Serial.println("s1 X - Move measuring cup servo 1. Closed is 0, open is 180. (no decimals)");
  Serial.println("s2 X - Move measuring cup servo 2. Closed is 0, open is 180. (no decimals)");
  Serial.println("s3 X - Move measuring cup servo 3. Closed is 0, open is 180. (no decimals)");
  Serial.println("s4 X - Move capping mechanism servo. Closed is ###, open is ###, stowed is ###. (no decimals)");
  Serial.println("s5 X - Move microscope servo. Deployed is ###, stowed is ###. (no decimals)");
  // Serial.println("lt X - Set the LED brightness. Off is 0, full is 255. (no decimals)");
  Serial.println("at X - Move actuator for X seconds.  Max is 11. (decimals allowed)");
  Serial.println("       Positve seconds extend. Negative seconds retract.");
  Serial.println("ca X - Move the carousel to an abolute position. (decimals allowed)");
  Serial.println("       1.0 moves exactly one cuvette spacing.");
  Serial.println("       Reference position is carousel zero position.");
  Serial.println("cr X - Move the carousel to a relative position. (decimals allowed)");
  Serial.println("       1.0 moves exactly one cuvette spacing.");
  Serial.println("       0.3 moves a cuvette from pump 1 to pump 2");
  Serial.println("       5.1 moves a cuvette from pump 1 to sample acquisition");
  Serial.println("       Reference position is carousel position when command is run.");
  // Serial.println("zc   - zero the carousel (moves until limit switch is triggered)");
  Serial.println(" -------------- ");
}