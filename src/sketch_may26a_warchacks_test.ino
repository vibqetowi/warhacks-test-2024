#include <Arduino.h>
// // #include <NewPing.h>


void setup() {
  Serial.begin(9600);
  // Initialize the built-in LED pin as an output
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Turn the built-in LED on
  digitalWrite(LED_BUILTIN, HIGH);
  delay(200); // Wait for 1 second (1000 milliseconds)

  // Turn the built-in LED off
  digitalWrite(LED_BUILTIN, LOW);
  delay(200); // Wait for 1 second (1000 milliseconds)
}


// // this code follows an object-oriented paradigm where the robot
// // is stored in the code as a class containing attributes
// // (pin numbers and speed) and methods to turn left for instance


// // define the robot class
// class Robot {

// public:
//   // Define the pin numbers for the IR sensors
//   const int  irAnalogSensorLeft;
//   const int irAnalogSensorRight;

//   // Define the pin numbers for the ultrasonic sensor
//   // const int 
  
//   const int speed; // Define the motor speed
  
//   // Define the motor pin mappings
//   const int motorLeftPinForward;
//   const int motorLeftPinBack;
//   const int motorRightPinForward;
//   const int motorRightPinBack;

//   // you may find the logic varies per challenge, therefore 
//   // this variable will tell the robot which challenge you are running
//   const int challengeNumber;

//   // Constructor
//   // here you enter the pin names for each of the functions, order matters!
//   Robot(int irLeft, int irRight, int mLf, int mLb, int mRf, int mRb, int speed, int challenge):
//         irAnalogSensorLeft(irLeft),
//         irAnalogSensorRight(irRight),
//         motorLeftPinForward(mLf),
//         motorLeftPinBack(mLb),
//         motorRightPinForward(mRf),
//         motorRightPinBack(mRb),
//         speed(speed),
//         challengeNumber(challenge)
//   {}


//   void forward() {
//     analogWrite(motorLeftPinForward, speed);
//     analogWrite(motorLeftPinBack, 0);
//     analogWrite(motorRightPinForward, 0);
//     analogWrite(motorRightPinBack, speed);

//     // Serial.print("forward\n");
//   }

//   void reverse() {
//     analogWrite(motorLeftPinForward, 0);
//     analogWrite(motorLeftPinBack, speed);
//     analogWrite(motorRightPinForward, speed);
//     analogWrite(motorRightPinBack, 0);
//     // Serial.print("backwards\n");
//   }

//   void turn_right() {
//     analogWrite(motorLeftPinForward, speed);
//     analogWrite(motorLeftPinBack, 0);
//     analogWrite(motorRightPinForward, 0);
//     analogWrite(motorRightPinBack, 0);
//     // Serial.print("right\n");
//   }

//   void turn_left() {
//     analogWrite(motorLeftPinForward, 0);
//     analogWrite(motorLeftPinBack, 0);
//     analogWrite(motorRightPinForward, 0);
//     analogWrite(motorRightPinBack, speed);
//     // Serial.print("left\n");

//   }

//   void stop() {
//     analogWrite(motorLeftPinForward, 0);
//     analogWrite(motorLeftPinBack, 0);
//     analogWrite(motorRightPinForward, 0);
//     analogWrite(motorRightPinBack, 0);
//     // Serial.print("stop\n");
//   }

// // this function returns the values read by the IR sensor and may be useful to you
//   int getIntensity(int irAnalogPin) {
//     int intensity = analogRead(irAnalogPin);
//     Serial.print(intensity);
//     return intensity;
//   }

// // this function returns the cm distance of an obstacle and returns the value as 
// // a number, you may find this useful
// //   bool obstacleDistance() {
// //   int distance = sonar.ping_cm();
// //   // Serial.println(distance);
// //   return (distance);
// // }
// };


// // what we did here is create a robot object that will be used throughout the program
// Robot robot(A0, A1, 10, 11, A4, A5, 100, 1);  

// // here we create a an object that will interact with the sonar sensor
// // the first two values are the pin numbers and the third is the max detection distance in cm 
// // the documentation for the sonar is here: https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
// // NewPing sonar(12,13,50);

// void setup() {
//   Serial.begin(9600);

//   // Set the IR sensor pins as input
//   pinMode(robot.irAnalogSensorLeft, INPUT);
//   pinMode(robot.irAnalogSensorRight, INPUT);

//   // Set the motor control pins as output
//   pinMode(robot.motorLeftPinForward, OUTPUT);
//   pinMode(robot.motorLeftPinBack, OUTPUT);
//   pinMode(robot.motorRightPinForward, OUTPUT);
//   pinMode(robot.motorRightPinBack, OUTPUT);
// }

// void loop() {
//   // these debugging statements may be useful to you
//   // view this output in serial monitor (control + shift + M)
//   Serial.print("IR left: ");
//   int intensity1 = robot.getIntensity(robot.irAnalogSensorLeft);
//   Serial.print(" IR right: ");
//   int intensity2 = robot.getIntensity(robot.irAnalogSensorRight);
//   Serial.print("\n");

// // as you can see I accessed the challengeNumber property of the robot to find
// // which logic to run, you may find that the logic is the same for both challenges
// // // but hint: it is not 
// // if (robot.challengeNumber == 1){
// //     // basic control flow to steer device (logic left as an exercise to teams)
// //     // if (condition0){
// //     //   robot.forward();
// //     // }
// //     // elif (condition1){
// //     //   robot.turn_left();
// //     // }
// //     // elif (condition2){
// //     //   robot.turn_right();
// //     // }
// //     // elif (condition3){
// //     //   robot.reverse();
// //     // }
// //     // else {
// //     //   robot.stop();
// //     // }
// //     continue;
// //   } elif (robot.challengeNumber == 2){
// //         if (condition4){
// //         robot.forward();
// //       }
// //       elif (condition5){
// //         robot.turn_left();
// //       }
// //       elif (condition6){
// //         robot.turn_right();
// //       }
// //       elif (condition7){
// //         robot.reverse();
// //       }
// //       elif (condition8) {
// //         robot.stop();
// //       }
// //   }
// }



