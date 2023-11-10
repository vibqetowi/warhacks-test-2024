#include <Arduino.h>
// // #include <NewPing.h>

// this code follows an object-oriented paradigm where the robot
// is stored in the code as a class containing attributes
// (pin numbers and speed) and methods to turn left for instance

// define the robot class
class Robot {
  public:
    // Define the pin numbers for the IR sensors
    const int irAnalogSensorLeft;
    const int irAnalogSensorRight;

    // Define the pin numbers for the ultrasonic sensor
    // const int 

    const int speed; // Define the motor speed

    // Define the motor pin mappings
    // for motor, pins with "~" must be used
    const int motorLeftPinForward;
    const int motorLeftPinBack;
    const int motorRightPinForward;
    const int motorRightPinBack;

    // you may find the logic varies per challenge, therefore 
    // this variable will tell the robot which challenge you are running
    const int challengeNumber;

    // Constructor
    // here you enter the pin names for each of the functions, order matters!
    Robot(int irLeft, int irRight, int motorLeftforward, int motorLeftback, int motorRightforward, int motorRightback, int speed, int challenge) :
        irAnalogSensorLeft(irLeft),
        irAnalogSensorRight(irRight),
        motorLeftPinForward(motorLeftforward),
        motorLeftPinBack(motorLeftback),
        motorRightPinForward(motorRightforward),
        motorRightPinBack(motorRightback),
        speed(speed),
        challengeNumber(challenge) {}

    void forward() {
      analogWrite(motorLeftPinForward, speed);
      analogWrite(motorLeftPinBack, 0);
      analogWrite(motorRightPinForward, speed);
      analogWrite(motorRightPinBack, 0);
      Serial.print("forward\n");
    }

    void reverse() {
      analogWrite(motorLeftPinForward, 0);
      analogWrite(motorLeftPinBack, speed);
      analogWrite(motorRightPinForward, 0);
      analogWrite(motorRightPinBack, speed);
      Serial.print("backwards\n");
    }

    void turn_right() {
      analogWrite(motorLeftPinForward, speed);
      analogWrite(motorLeftPinBack, 0);
      analogWrite(motorRightPinForward, 0);
      analogWrite(motorRightPinBack, 0);
      Serial.print("right\n");
    }

    void turn_left() {
      analogWrite(motorLeftPinForward, 0);
      analogWrite(motorLeftPinBack, 0);
      analogWrite(motorRightPinForward, speed);
      analogWrite(motorRightPinBack, 0);
      Serial.print("left\n");
    }

    void stop() {
      analogWrite(motorLeftPinForward, 0);
      analogWrite(motorLeftPinBack, 0);
      analogWrite(motorRightPinForward, 0);
      analogWrite(motorRightPinBack, 0);
      Serial.print("stop\n");
    }

    // this function returns the values read by the IR sensor and may be useful to you
    int getIntensity(int irAnalogPin) {
      int intensity = analogRead(irAnalogPin);
      Serial.print(intensity);
      return intensity;
    }
};

// what we did here is create a robot object that will be used throughout the program
// you can experiment with speed, it just changes the amount of current used
Robot robot(A0, A1, 5, 6, 9, 10, 1000, 1);  

void setup() {
    Serial.begin(9600);

    // Set the IR sensor pins as input
    pinMode(robot.irAnalogSensorLeft, INPUT);
    pinMode(robot.irAnalogSensorRight, INPUT);

    // Set the motor control pins as output
    pinMode(robot.motorLeftPinForward, OUTPUT);
    pinMode(robot.motorLeftPinBack, OUTPUT);
    pinMode(robot.motorRightPinForward, OUTPUT);
    pinMode(robot.motorRightPinBack, OUTPUT);
}

const int IRtreshold = 200;

void loop() {
    // these debugging statements may be useful to you
    // view this output in serial monitor (control + shift + M)
    Serial.print("IR left: ");
    int IRleft = robot.getIntensity(robot.irAnalogSensorLeft);
    Serial.print(" IR right: ");
    int IRright = robot.getIntensity(robot.irAnalogSensorRight);
    Serial.print("\n");

    // as you can see I accessed the challengeNumber property of the robot to find
    // which logic to run, you may find that the logic is the same for both challenges
    // // but hint: it is not 
    if (robot.challengeNumber == 1) {
        // // basic control flow to steer device (logic left as an exercise to teams)
        if (IRleft < IRtreshold && IRright < IRtreshold) {
            robot.forward();
        }
        else if (IRleft > IRtreshold && IRright < IRtreshold) {
            robot.turn_left();
        }
        else if (IRleft < IRtreshold && IRright > IRtreshold) {
            robot.turn_right();
        }
        // else if (condition3) {
        //   robot.reverse();
        // }
        else {
            robot.stop();
            delay(4000);
            robot.forward();
            delay(500);
        }
    }
    // elif (robot.challengeNumber == 2) {
    //     if (condition4) {
    //       robot.forward();
    //     }
    //     elif (condition5) {
    //       robot.turn_left();
    //     }
    //     elif (condition6) {
    //       robot.turn_right();
    //     }
    //     elif (condition7) {
    //       robot.reverse();
    //     }
    //     elif (condition8) {
    //       robot.stop();
    //     }
    // }
}
