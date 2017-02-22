// webots
#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/DifferentialWheels.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Keyboard.hpp>
#include <webots/LED.hpp>
#include <webots/LightSensor.hpp>
#include <webots/Receiver.hpp>
#include <webots/Robot.hpp>

// standard
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <vector>
#include <algorithm>
#include <iterator>

using namespace webots;
using namespace std::chrono_literals;

typedef std::chrono::high_resolution_clock Clock;

#define TIME_STEP 64
#define PS_THRESHOLD 200
#define SEPARATION_THRESHOLD 80
#define WHEEL_SPEED 800
#define ARRAY_SIZE 10
#define SECTOR_ANGLE 45
#define ALIGN_ERROR 25
#define ALIGN_THRESHOLD 2
