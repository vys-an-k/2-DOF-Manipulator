#include <Servo.h>
#include <math.h>

// Servo objects
Servo servo1;        // Base joint
Servo servo2;        // Elbow joint
Servo servoGripper;  // Gripper

// Link lengths in cm
float L1 = 15.0;
float L2 = 11.0;

// Target coordinates
float x = 2.0;
float y = 3.0;

void setup() {
  servo1.attach(3);         // Base joint pin
  servo2.attach(6);         // Elbow joint pin
  servoGripper.attach(9);   // Gripper servo pin

  // Open gripper at start
  servoGripper.write(120);   // Adjust based on your servo's "open" angle
}

void loop() {
  float theta1, theta2;

  if (computeIK(x, y, theta1, theta2)) {
    int angle1 = degrees(theta1);
    int angle2 = degrees(theta2);

    servo1.write(constrain(angle1, 0, 95));
    servo2.write(constrain(angle2, 0, 180));
  }

  // Example: close and open the gripper every 2 seconds
  servoGripper.write(30);   // Open
  delay(2000);
  servoGripper.write(90);   // Close
  delay(2000);
}

bool computeIK(float x, float y, float &theta1, float &theta2) {
  float distance = sqrt(x * x + y * y);
  if (distance > (L1 + L2)) return false; // Unreachable

  float cos_theta2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (cos_theta2 < -1 || cos_theta2 > 1) return false;

  float sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);
  theta2 = atan2(sin_theta2, cos_theta2);

  float k1 = L1 + L2 * cos_theta2;
  float k2 = L2 * sin_theta2;
  theta1 = atan2(y, x) - atan2(k2, k1);

  return true;
}
