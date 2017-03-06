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

using namespace webots;

#define TIME_STEP 4
#define PS_THRESHOLD 150
#define SEPARATION_THRESHOLD 80
#define WHEEL_SPEED 1000
#define ARRAY_SIZE 10
#define SECTOR_ANGLE 45
#define ALIGN_ERROR 15
#define ALIGN_THRESHOLD 2
#define RANGE 0.35
#define ROULETTE 7
