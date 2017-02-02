// webots
#include <webots/Camera.hpp>
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

using namespace webots;

#define TIME_STEP 64

template <class T, std::size_t I, std::size_t... J> struct MultiDimArray {
  using Nested = typename MultiDimArray<T, J...>::type;
  // typedef typename MultiDimArray<T, J...>::type Nested;
  using type = std::array<Nested, I>;
  // typedef std::array<Nested, I> type;
};

template <class T, std::size_t I> struct MultiDimArray<T, I> {
  using type = std::array<T, I>;
  // typedef std::array<T, I> type;
};

struct ReceiverData {
  std::array<int, 6> data{};
  bool received;
  // MultiDimArray<int, 8, 6>::type data;
};

class MyRobot : public DifferentialWheels {

private:
  Camera *camera;
  DistanceSensor *distanceSensors[8];
  LED *leds[8];
  DifferentialWheels *diffWheels;
  LightSensor *ambientLight[8];
  Emitter *irEm[9];
  Receiver *irRec[9];
  Keyboard *keyboard;

  ReceiverData myData;

  std::string robot_name;
  const char *data;

  std::array<int, 3> speed;
  std::array<double, 3> emitterDirection;
  double signalStrength;
  int multiplier;
  double highReading;
  double lowReading;

public:
  MyRobot() {

    // camera
    camera = getCamera("camera");

    std::string ls = "ls";
    std::string ps = "ps";
    std::string em = "em";
    std::string led = "led";
    std::string rc = "rc";

    for (unsigned int i = 0; i < 8; i++) {
      ambientLight[i] = getLightSensor(ls.replace(2, 1, std::to_string(i)));
      ambientLight[i]->enable(TIME_STEP);
      distanceSensors[i] =
          getDistanceSensor(ps.replace(2, 1, std::to_string(i)));
      distanceSensors[i]->enable(TIME_STEP);
      irEm[i] = getEmitter(em.replace(2, 1, std::to_string(i)));
      irRec[i] = getReceiver(rc.replace(2, 1, std::to_string(i)));
      irRec[i]->enable(TIME_STEP);
      leds[i] = getLED(led.replace(3, 1, std::to_string(i)));
    }

    irEm[8] = getEmitter("em8");
    irRec[8] = getReceiver("rc8");
    irRec[8]->enable(TIME_STEP);

    keyboard = getKeyboard();
    keyboard->enable(TIME_STEP);

    robot_name = getName();
    data = "";

    speed = {100, 50, 0};
    multiplier = 1;

    myData.received = false;
    myData.data[6] = 0;

    highReading = 0;
    lowReading = 1000000;
  }

  virtual ~MyRobot() {}

  void run() {
    while (step(TIME_STEP) != -1) {
      std::cout << "1 for keyboard control, 2 for object avoidance"
                << std::endl;
      int decide = readKey();
      switch (decide) {
      case 49:
        std::cout << "Keyboard control mode" << std::endl;
        teleop();
        break;
      case 50:
        std::cout << "Object avoidance mode" << std::endl;
        objectDetectionMode();
      }
    }
  }

  void objectDetectionMode() {
    setLEDs(1);
    while (step(TIME_STEP != -1)) {
      objectDetection();
    }
  }

  void teleop() {
    while (step(TIME_STEP) != -1) {
      if (robot_name.compare("e2") == 0) {
        setLEDs(1);
        // -----get data-----
        int keyPress = readKey();

        // -----process data-----

        // -----send actuator commands-----
        keyboardControl(keyPress);
        std::string message = std::to_string((int)diffWheels->getLeftSpeed()) +
                              " " +
                              std::to_string((int)diffWheels->getRightSpeed());
        // for (unsigned int i = 0; i < 8; i++) {
        //   sendPacket(i, message);
        // }
        sendPacket(8, message);
      }
      if (robot_name.compare("e2") != 0) {
        while (step(TIME_STEP) != -1) {

          setLEDs(0);

          // -----get data-----
          std::string temp = getReceiverData(8);
          std::cout << "[" << robot_name << "] "
                    << "sig str " << signalStrength << std::endl;
          std::cout << "[" << robot_name << "] "
                    << "dir " << emitterDirection[0] << " "
                    << emitterDirection[1] << " " << emitterDirection[2]
                    << std::endl;

          // -----process data-----
          processReceiverData(temp);

          // -----send actuator commands-----
          if (myData.received == true) {
            std::cout << "[" << robot_name << "] "
                      << "Received signal " << myData.data[1] << " "
                      << myData.data[2] << std::endl;
            move(myData.data[1], myData.data[2]);
          }

          else {
            std::cout << "[" << robot_name << "] "
                      << "Signal lost, roaming" << std::endl;
            setLEDs(1);
            objectDetection();
          }
        }
      }
    }
  }
  std::string getReceiverData(int i) {
    Receiver *copy = (Receiver *)malloc(sizeof(Receiver));
    while (irRec[i]->getQueueLength() > 0) {
      data = (char *)irRec[i]->getData();
      signalStrength = (double)irRec[i]->getSignalStrength();
      for (size_t n = 0; n < 3; n++) {
        emitterDirection[n] = irRec[i]->getEmitterDirection()[n];
      }
      memcpy(copy, data, sizeof(Receiver));
      irRec[i]->nextPacket();
    }
    return (char *)copy;
  }

  void processReceiverData(std::string data) {
    myData.received = false;
    for (unsigned int i = 0; i < 6; i++) {
      myData.data[i] = 0;
    }
    try {
      std::regex re("[*0-9*]+|[-][*0-9*]+");
      std::sregex_iterator next(data.begin(), data.end(), re);
      std::sregex_iterator end;
      int count = 0;
      while (next != end) {
        std::smatch match = *next;
        std::string match1 = match.str();
        myData.data[count] = atoi(match1.c_str());
        next++;
        count++;
        if (next == end) {
          myData.received = true;
        }
      }

      // myData.received = false;
    } catch (std::regex_error &e) {
    }
  }

  void sendPacket(int emitterNumber, std::string message) {
    std::string temp = "[" + robot_name + "] ";
    temp.append(message);
    const char *packet = temp.c_str();
    irEm[emitterNumber]->send(packet, (strlen(packet) + 1));
  }

  void objectDetection() {
    double ps_values[8];
    for (int i = 0; i < 8; i++)
      ps_values[i] = checkDistanceSensor(i);

    // detect obsctacles
    bool left_obstacle =
        ps_values[0] > 100.0 || ps_values[1] > 100.0 || ps_values[2] > 100.0;
    bool right_obstacle =
        ps_values[5] > 100.0 || ps_values[6] > 100.0 || ps_values[7] > 100.0;

    // init speeds
    double left_speed = 500;
    double right_speed = 500;

    // modify speeds according to obstacles
    if (left_obstacle & !right_obstacle) {
      // turn left
      left_speed -= 500;
      right_speed += 500;
      std::cout << "[" << robot_name << "] "
                << "Turning left" << std::endl;
    } else if (right_obstacle & !left_obstacle) {
      // turn right
      left_speed += 500;
      right_speed -= 500;
      std::cout << "[" << robot_name << "] "
                << "Turning right" << std::endl;
    } else if (right_obstacle & left_obstacle) {
      left_speed = -500;
      right_speed = -500;
      std::cout << "[" << robot_name << "] "
                << "Backwards" << std::endl;
    }

    move((int)left_speed, (int)right_speed);
  }

  void keyboardControl(int keyPress) {

    switch (keyPress) {
    // W
    case 87:
      move(multiplier * speed[0], multiplier * speed[0]);
      break;

    // S
    case 83:
      move(-multiplier * speed[0], -multiplier * speed[0]);
      break;

    // A
    case 65:
      move(-multiplier * speed[0], multiplier * speed[0]);
      break;

    // D
    case 68:
      move(multiplier * speed[0], -multiplier * speed[0]);
      break;

    // Q
    case 81:
      move(multiplier * speed[1], multiplier * speed[0]);
      break;

    // E
    case 69:
      move(multiplier * speed[0], multiplier * speed[1]);
      break;

    // UP ARROW
    case 315:
      if (multiplier < 10) {
        multiplier++;
      }
      break;

    // DOWN ARROW
    case 317:
      if (multiplier > 1) {
        multiplier--;
      }
      break;

    default:
      move(multiplier * speed[2], multiplier * speed[2]);
      break;
    }

    std::cout << "Going " << multiplier << " times base speed" << std::endl;
  }

  int readKey() { return keyboard->getKey(); }

  void move(int left, int right) { setSpeed(left, right); }

  void setLEDs(int value) {
    for (int i = 0; i < 8; i++)
      leds[i]->set(value);
  }

  double checkLightSensor(int n) { return ambientLight[n]->getValue(); }

  double checkDistanceSensor(int n) { return distanceSensors[n]->getValue(); }

  void rangeTest(int n) {
    double current = checkDistanceSensor(n);

    if (current > highReading) {
      highReading = current;
    }

    if (current < lowReading) {
      lowReading = current;
    }
  }

  int isInteger(auto x) {
    if (x == floor(x)) {
      return 1;
    } else {
      return 0;
    }
  }
};

int main(int argc, char *argv[]) {

  MyRobot *robot = new MyRobot();
  robot->run();
  delete robot;
  return 0;
}
