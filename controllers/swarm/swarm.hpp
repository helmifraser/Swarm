// webots
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/GPS.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

// standard
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <vector>

// own
#include "configLoader.hpp"

using namespace webots;

#define TIME_STEP 32
#define PS_THRESHOLD 150
#define SEPARATION_THRESHOLD 80
#define WHEEL_SPEED 1000
#define ARRAY_SIZE 10
#define SECTOR_ANGLE 45
#define ALIGN_ERROR 15
#define ALIGN_THRESHOLD 2
#define RANGE 0.35
#define ROULETTE 7

struct ReceiverData {
  std::array<int, 6> data{};
  bool received;
  std::array<std::string, ARRAY_SIZE> orientationString;
  std::array<double, ARRAY_SIZE> orientationDouble;
  double surrounding[2][2];
};


class Swarm : public DifferentialWheels{
private:
  Camera *camera;
  DistanceSensor *distanceSensors[8];
  LED *leds[8];
  DifferentialWheels *diffWheels;
  Emitter *emitter;
  Receiver *receiver;
  Keyboard *keyboard;
  Compass *compass;
  GPS *gps;

  ReceiverData myData;

  std::string robot_name;
  const char *data;

  std::array<int, 3> speed;
  std::array<double, 3> currentOrientation;
  double signalStrength[100];
  double emitterDirection[100][3];
  int multiplier;
  int robots;
  int choose;
  int targetHeading;

  int matrix[100][100];

  double ps_values[8];
  bool left_obstacle, right_obstacle, front_obstacle, back_obstacle;
  bool aligned;

public:
  Swarm();
  void run();
  void randomAlign();
  int randomVal(int range);
  void objectDetectionMode();
  void teleop();
  void flock();
  void computeDirections(std::array<double, 100> &allDirections);
  void computeCluster(std::array<double, 100> &allDirections, std::array<int, 4> &output);
  int chooseSector(std::array<int, 4> &output);
  bool checkSector(int sector);
  double randomSectorAngle(int sector);
  void adjust(double angle);
  void sendCurrentSpeed();
  void sendCurrentOrientation();
  void getReceiverData();
  void processReceiverData(std::array<std::string, ARRAY_SIZE> data);
  void sendPacket(std::string message);
  void distanceCheck();
  void objectDetection(double speedAdjust);
  void separation();
  void align(int heading);
  int getCurrentHeading();
  int orientationToHeading(int current);
  bool checkAlignment();
  void saveCompassValues();
  std::array<double, 3> getGPSValue();
  double getCurrentOrientation();
  double computeVectorMagnitude(std::array<double, 3> components);
  double computeVectorAngle(std::array<double, 3> components);
  int computeAverageHeading();
  int calculateMode(int array[], int size);
  void keyboardControl(int keyPress);
  int readKey();
  void move(int left, int right);
  void setLEDs(int value);
  double checkDistanceSensor(int n);
  double roundNum(double x);
  std::string printName();

};
