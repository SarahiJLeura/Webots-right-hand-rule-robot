#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <cmath>

#define TIME_STEP 64

#define MAX_SPEED 6.28
#define NUM_SENSORS 8
#define NUM_MOTORS 2
// Tolerance for stopping the robot at the target coordinates
#define TOLERANCE 0.03

using namespace webots;

// This structure holds the relevant sensor readings
struct SensorReadings {
  bool right_wall;     // Sensor ps2: detects wall on the right
  bool front_wall;     // Sensor ps0: detects wall in front
  bool sensor_corner;  // Sensor ps1: used to avoid collisions at corners
  bool right_corner;   // True if both right and front sensors detect walls
  bool no_wall;        // True if there's no wall in front or right
};

void initializeMotors(Robot *r, Motor *m[]);
void initializeSensors(Robot *r, DistanceSensor *ps[]);
void initializeGPS(Robot *robot, GPS *&gps);
SensorReadings readSensors(DistanceSensor *ps[]);
void decideMovement(const SensorReadings &sen, double &ls, double &rs);
bool isGoalReached(double posX, double posY);

int main(int argc, char **argv) {
  // Declare the robot; use arrays to minimize variable count
  Robot *robot = new Robot();
  DistanceSensor *prox_sensors[NUM_SENSORS];
  Motor *motors[NUM_MOTORS];
  GPS *gps;
  
  initializeMotors(robot, motors);
  initializeSensors(robot, prox_sensors);
  initializeGPS(robot, gps);
  
  // Initialize motor speeds
  double left_speed = MAX_SPEED;
  double right_speed = MAX_SPEED;

  // Flag to check if robot just started moving
  bool init = true;
  
  while (robot->step(TIME_STEP) != -1) {
    // Read sensors and decide movement accordingly
    SensorReadings sensors = readSensors(prox_sensors);
    // Get robot's position using GPS
    const double *position = gps->getValues();
    
    // Stop if goal is reached
    if (isGoalReached(position[0], position[1])) {
      std::cout << "GOAL REACHED: STOP" << std::endl;
      left_speed = 0;
      right_speed = 0;
    } else {
      if (sensors.no_wall && init) {
        std::cout << "init" << std::endl;
        // At the beginning, go straight until hitting a wall
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        init = false;
        decideMovement(sensors, left_speed, right_speed);
      }

      // Adjust if getting too close to a wall
      if (sensors.sensor_corner) {
        left_speed = MAX_SPEED / 8;
        right_speed = MAX_SPEED;
      }
    }

    // Set motor velocities
    motors[0]->setVelocity(left_speed);
    motors[1]->setVelocity(right_speed);
  }

  delete robot;
  return 0;
}

// Initialize the proximity sensors
void initializeSensors(Robot *robot, DistanceSensor *prox_sensors[]) {
  for (int i = 0; i < NUM_SENSORS; i++) { 
    std::string ps_name = "ps" + std::to_string(i);
    prox_sensors[i] = robot->getDistanceSensor(ps_name);
    prox_sensors[i]->enable(TIME_STEP);
  }
}

// Initialize the motors
void initializeMotors(Robot *robot, Motor *motors[]) {
  const char *motNames[NUM_MOTORS] = {"left wheel motor", "right wheel motor"};
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i] = robot->getMotor(motNames[i]);
    motors[i]->setPosition(INFINITY);
    motors[i]->setVelocity(0.0);
  }
}

// Initialize the GPS
void initializeGPS(Robot *robot, GPS *&gps) {
  gps = robot->getGPS("gps");
  gps->enable(TIME_STEP);
}

// Read and interpret sensor values
SensorReadings readSensors(DistanceSensor *prox_sensors[]) {
  SensorReadings readings;
  readings.right_wall = prox_sensors[2]->getValue() > 80;
  readings.front_wall = prox_sensors[0]->getValue() > 80;
  readings.sensor_corner = prox_sensors[1]->getValue() > 80;
  readings.right_corner = readings.right_wall && readings.front_wall;
  readings.no_wall = !readings.right_wall && !readings.front_wall;
  return readings;
}

// Decide movement based on sensor readings
void decideMovement(const SensorReadings &sensors, double &left_speed, double &right_speed) {
  if (sensors.right_corner) {
    // In a corner: turn left
    std::cout << "RIGHT CORNER: LEFT" << std::endl;
    left_speed = MAX_SPEED / 8;
    right_speed = MAX_SPEED;
  } else if (sensors.front_wall) {
    // Wall in front: turn left
    std::cout << "FRONT WALL: LEFT" << std::endl;
    left_speed = -MAX_SPEED;
    right_speed = MAX_SPEED;
  } else if (sensors.right_wall) {
    // Wall on the right: go straight
    std::cout << "GO STRAIGHT" << std::endl;
    left_speed = MAX_SPEED;
    right_speed = MAX_SPEED;
  } else {
    // No wall on the right: turn right
    std::cout << "NO WALLS: RIGHT" << std::endl;
    left_speed = MAX_SPEED;
    right_speed = MAX_SPEED / 8;
  }
}

// Check if robot is near the target position
bool isGoalReached(double posX, double posY) {
  const double target[2] = {1.86, -0.26};
  return fabs(posX - target[0]) < TOLERANCE &&
         fabs(posY - target[1]) < TOLERANCE;
}