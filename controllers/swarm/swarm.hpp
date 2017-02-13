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
#include <iostream>
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <vector>

using namespace webots;

#define TIME_STEP 64
#define PS_THRESHOLD 250.0
#define SEPARATION_THRESHOLD 80
#define WHEEL_SPEED 500
