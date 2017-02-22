#include "swarm.hpp"

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
  // MultiDimArray<int, 8, 6>::type data2;
  std::array<std::string, ARRAY_SIZE> orientationString;
  std::array<double, ARRAY_SIZE> orientationDouble;
  double surrounding[2][2];
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
  Compass *compass;

  ReceiverData myData;

  std::string robot_name;
  const char *data;

  std::array<int, 3> speed;
  // std::array<double, 3> emitterDirection;
  std::array<double, 3> currentOrientation;
  double signalStrength[100];
  double emitterDirection[100][3];
  int multiplier;
  int robots;
  int choose;
  int targetHeading;

  int matrix[100][100];

  double ps_values[8];
  bool left_obstacle, right_obstacle, front_obstacle;
  bool aligned;

public:
  MyRobot() {

    // camera
    camera = getCamera("camera");
    compass = getCompass("compass");
    compass->enable(TIME_STEP);

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
    irEm[8]->setRange(0.35);
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

    robots = 0;
    srand(time(0));
    choose = rand() % 3;
    aligned = false;

    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 100; j++) {
        matrix[i][j] = 0;
      }
    }

    for (int i = 0; i < 100; i++) {
      for (int j = 0; j < 100; j++) {
        srand(time(NULL) * i + j);
        matrix[i][j] = rand() % 15872567351;
      }
    }

  }

  virtual ~MyRobot() {}

  void run() {
    while (step(TIME_STEP) != -1) {
      std::cout << "1 for keyboard control, 2 for object avoidance, 3 for other, 4 for randomVal"
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
        break;
      case 51:
        std::cout << "Other" << std::endl;
        follow();
        break;
      case 52:
        randomAlign();
        break;
      }
    }
  }

  void randomAlign(){
    int random = randomVal();
    while (step(TIME_STEP != -1)) {
      saveCompassValues();
      int test2 = getCurrentHeading();
      std::cout << printName() << "random " << random << std::endl;
      if (test2 != random) {
        align(random);
        std::cout << printName() << "Moving, current " << getCurrentHeading() << std::endl;
      } else{
        move(0,0);
        std::cout << printName() << "Stop" << std::endl;
        break;
      }
    }
  }

  int randomVal(){
    int robNum = atoi(robot_name.substr(1,strlen(robot_name.c_str())-1).c_str());
    return matrix[robNum][rand() % 100] % 8;
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
        setLEDs(0);
        // -----get data-----
        int keyPress = readKey();

        saveCompassValues();

        // -----process data-----

        // -----send actuator commands-----
        keyboardControl(keyPress);

        // sendCurrentOrientation();
      }
      if (robot_name.compare("e2") != 0) {
        follow();
        // while (step(TIME_STEP) != -1) {
        //
        //   setLEDs(0);
        //
        //   // -----get data-----
        //   getReceiverData(8);
        //   distanceCheck();
        //   saveCompassValues();
        //
        //   // -----process data-----
        //   // processReceiverData(temp);
        //   // std::cout << printName() << "Received: " << temp << std::endl;
        //   // std::cout << printName()
        //   //           << "Angle: " << computeVectorAngle(emitterDirection)
        //   //           << std::endl;
        //
        //   // -----send actuator commands-----
        //   sendCurrentOrientation();
        //   separation();
        //   // if (myData.received) {
        //   //   align();
        //   // } else if (!myData.received) {
        //   //   setLEDs(1);
        //   //   objectDetection();
        //   // }
        // }
        // }
        // if (1) {
        //   setLEDs(1);
        //   // -----get data-----
        //   getReceiverData(8);
        //   saveCompassValues();
        //
        //   // -----process data-----
        //   processReceiverData(myData.orientationString);
        //   int targetHeading = computeAverageHeading();
        //   std::cout << printName()
        //             << "Average heading: " << computeAverageHeading()
        //             << std::endl;
        //
        //   // -----send actuator commands-----
        //   std::cout << printName() << "Aligned:" << aligned << std::endl;
        //   if (targetHeading != getCurrentHeading()) {
        //     align(targetHeading);
        //   } else {
        //     // objectDetection();
        //     move(0, 0);
        //   }
        // while (targetHeading != getCurrentHeading()) {
        //   align(targetHeading);
        //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // }
        sendCurrentOrientation();
      }
    }
  }

  void follow() {
    bool together = false;
    randomAlign();

    while (step(TIME_STEP != -1)) {
      getReceiverData(8);
      saveCompassValues();
      distanceCheck();

      processReceiverData(myData.orientationString);

      int index = std::distance(
          signalStrength,
          std::max_element(signalStrength,
                           signalStrength +
                               sizeof(signalStrength) / sizeof(double)));
      std::array<double, 3> direction = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        direction[k] = emitterDirection[index][k];
      }
      double nearestNeighbour = computeVectorAngle(direction);

      // together = false;
      if ((robots >= ALIGN_THRESHOLD) && (!left_obstacle) &&
          (!right_obstacle) && together == false &&
          signalStrength[index] < 10) {
            setLEDs(0);

        adjust(nearestNeighbour);
        std::cout << printName() << "Sig str " << signalStrength[index]
                  << std::endl;
      } else if ((robots >= ALIGN_THRESHOLD) && (left_obstacle) &&
                 (right_obstacle) && together == false) {
                   setLEDs(0);

        separation();
      } else if ((robots < ALIGN_THRESHOLD) && together == false) {
        setLEDs(0);

        std::cout << printName() << "Robots" << robots << std::endl;
        std::cout << printName() << "Searching, sig str " << signalStrength[index] << std::endl;
        objectDetection();

        // adjust(nearestNeighbour);
      } else if ((robots >= ALIGN_THRESHOLD) && together == false &&
                 signalStrength[index] > 10) {
                   setLEDs(0);

        std::cout << printName() << " Done, sig str " << signalStrength[index]
                  << std::endl;

        // irEm[8]->setRange(2);
        move(0, 0);
        together = true;
      }

      std::cout << printName() << "Range " << irEm[8]->getRange() << std::endl;

      if (together == true) {
        setLEDs(1);
        irEm[8]->setRange(2);
        int heading = getCurrentHeading();
        int targetHeading = 2;
        // std::cout << printName() << "targetHeading " << targetHeading
        //           << std::endl;
        if (heading == targetHeading & checkAlignment() == true) {
          heading = computeAverageHeading();
        } else {
          heading = targetHeading;
        }
        align(heading);
        if (robots < ALIGN_THRESHOLD) {
          together = false;
          setLEDs(0);
        }
      }

      sendCurrentOrientation();
    }
  }

  void adjust(double angle) {
    double left_speed = WHEEL_SPEED;
    double right_speed = WHEEL_SPEED;

    if (((angle > (360 - ALIGN_ERROR)) & (angle < 360)) |
        ((angle > 0) & (angle < ALIGN_ERROR))) {
      right_speed = left_speed = WHEEL_SPEED;
    } else if ((angle > 270) & (angle < (360 - ALIGN_ERROR))) {
      left_speed =
          (1 + (ALIGN_ERROR + angle - 360) / (90 - ALIGN_ERROR)) * WHEEL_SPEED;
      // right_speed = ( ( (360 - ALIGN_ERROR) - angle)/85) )*WHEEL_SPEED;
      right_speed = WHEEL_SPEED;
    } else if ((angle >= 180) & (angle < 270)) {
      left_speed = ((angle - 270) / 90) * WHEEL_SPEED;
      right_speed = WHEEL_SPEED;
    } else if ((angle >= 90) & (angle < 180)) {
      right_speed = ((90 - angle) / (90)) * WHEEL_SPEED;
      left_speed = WHEEL_SPEED;
    } else if ((angle >= ALIGN_ERROR) & (angle < 90)) {
      right_speed =
          (1 - (angle - ALIGN_ERROR) / (90 - ALIGN_ERROR)) * WHEEL_SPEED;
      left_speed = WHEEL_SPEED;
    }

    // if ( (angle > (360 - ALIGN_ERROR) & angle < 360) | (angle > 0 & angle <
    // ALIGN_ERROR) ) {
    //   std::cout << printName() << "Robot ahead" << std::endl;
    //   right_speed = left_speed = WHEEL_SPEED;
    // } else if ((angle > 270) & (angle < (360 - ALIGN_ERROR))) {
    //   std::cout << printName() << "1st quadrant" << std::endl;
    //   left_speed -= WHEEL_SPEED;
    //   right_speed += WHEEL_SPEED;
    // } else if ((angle >= 180) & (angle < 270)) {
    //   std::cout << printName() << "2nd quadrant" << std::endl;
    //   left_speed -= WHEEL_SPEED;
    //   right_speed += WHEEL_SPEED;
    // } else if ((angle >= 90) & (angle < 180)) {
    //   std::cout << printName() << "3rd quadrant" << std::endl;
    //   left_speed += WHEEL_SPEED;
    //   right_speed -= WHEEL_SPEED;
    // } else if ( (angle >= ALIGN_ERROR) & (angle < 90)) {
    //   std::cout << printName() << "4th quadrant" << std::endl;
    //   left_speed += WHEEL_SPEED;
    //   right_speed -= WHEEL_SPEED;
    // } else {
    //   right_speed = left_speed = WHEEL_SPEED;
    // }

    if (left_speed > 1000) {
      left_speed = 1000;
    }

    if (right_speed > 1000) {
      right_speed = 1000;
    }

    move((int)left_speed, (int)right_speed);
  }

  void sendCurrentSpeed() {
    std::string message = std::to_string((int)diffWheels->getLeftSpeed()) +
                          " " +
                          std::to_string((int)diffWheels->getRightSpeed());
    sendPacket(8, message);
  }

  void sendCurrentOrientation() {
    std::string message = std::to_string(getCurrentOrientation());
    sendPacket(8, message);
  }

  void getReceiverData(int i) {
    Receiver *copy = (Receiver *)malloc(sizeof(Receiver));
    myData.orientationString = {};
    robots = (irRec[i]->getQueueLength() + 1) / 2;
    for (int i = 0; i < 100; i++) {
      signalStrength[i] = 0;
    }
    for (int k = 0; k < irRec[i]->getQueueLength(); k++) {
      data = (char *)irRec[i]->getData();
      signalStrength[k] = (double)irRec[i]->getSignalStrength();
      for (int n = 0; n < 3; n++) {
        emitterDirection[k][n] = irRec[i]->getEmitterDirection()[n];
      }
      memcpy(copy, data, sizeof(Receiver));
      myData.orientationString[k] = (char *)copy;
      // std::cout << "OS: " << k << " " << myData.orientationString[k] <<
      // std::endl;
      irRec[i]->nextPacket();
    }
  }

  void processReceiverData(std::array<std::string, ARRAY_SIZE> data) {
    myData.received = false;
    for (unsigned int i = 0; i < ARRAY_SIZE; i++) {
      myData.orientationDouble[i] = 0;
    }
    try {
      // std::regex re("[*0-9*]+|[-][*0-9*]+|[*0-9*.*0-9*]+|[-][*0-9*.*0-9*]");
      std::regex re("[*0-9*.*0-9*]+");
      for (int i = 0; i < robots; i++) {
        std::sregex_iterator next(data[i].begin(), data[i].end(), re);
        std::sregex_iterator end;
        while (next != end) {
          std::smatch match = *next;
          std::string match1 = match.str();
          myData.orientationDouble[i] = atoi(match1.c_str());
          if (myData.orientationDouble[i] >= 352) {
            myData.orientationDouble[i] = 0;
          }
          next++;
          if (next == end) {
            myData.received = true;
          }
        }
      }

      // myData.received = false;
    } catch (std::regex_error &e) {
    }
  }

  void sendPacket(int emitterNumber, std::string message) {
    std::string temp = printName();
    temp.append(message);
    const char *packet = temp.c_str();
    irEm[emitterNumber]->send(packet, (strlen(packet) + 1));
  }

  void distanceCheck() {
    for (int i = 0; i < 8; i++) {
      ps_values[i] = checkDistanceSensor(i);
    }

    // detect obsctacles
    left_obstacle = (ps_values[0] > PS_THRESHOLD) ||
                    (ps_values[1] > PS_THRESHOLD) ||
                    (ps_values[2] > PS_THRESHOLD);
    right_obstacle = (ps_values[5] > PS_THRESHOLD) ||
                     (ps_values[6] > PS_THRESHOLD) ||
                     (ps_values[7] > PS_THRESHOLD);
    front_obstacle =
        (ps_values[0] > PS_THRESHOLD) & (ps_values[7] > PS_THRESHOLD);
  }

  void objectDetection() {
    // setLEDs(1);
    distanceCheck();

    // init speeds
    double left_speed = WHEEL_SPEED;
    double right_speed = WHEEL_SPEED;

    // modify speeds according to obstacles
    if (left_obstacle & !right_obstacle) {
      // turn left
      left_speed -= WHEEL_SPEED;
      right_speed += WHEEL_SPEED;
      // std::cout << printName() << "Turning left" << std::endl;
    } else if (right_obstacle & !left_obstacle) {
      // turn right
      left_speed += WHEEL_SPEED;
      right_speed -= WHEEL_SPEED;
      // std::cout << printName() << "Turning right" << std::endl;
    } else if (right_obstacle & left_obstacle) {
      left_speed = -WHEEL_SPEED;
      right_speed = -WHEEL_SPEED;
      // std::cout << printName() << "Backwards" << std::endl;
    }

    if (left_speed > 1000) {
      left_speed = 1000;
    }

    if (right_speed > 1000) {
      right_speed = 1000;
    }
    move((int)left_speed, (int)right_speed);
  }

  void separation() {
    distanceCheck();

    // init speeds
    double left_speed = WHEEL_SPEED;
    double right_speed = WHEEL_SPEED;

    // modify speeds according to obstacles
    if (left_obstacle | right_obstacle) {
      // turn left
      left_speed = -left_speed;
      right_speed = -right_speed;
    } else {
      left_speed = right_speed = 0;
    }

    move((int)left_speed, (int)right_speed);
  }

  void align(int heading) {

    double target = 0;
    switch (heading) {
    case 0:
      target = 359;
      break;
    case 1:
      target = 45;
      break;
    case 2:
      target = 90;
      break;
    case 3:
      target = 135;
      break;
    case 4:
      target = 180;
      break;
    case 5:
      target = 225;
      break;
    case 6:
      target = 270;
      break;
    case 7:
      target = 315;
      break;
    }
    // std::cout << "Target: " << target << std::endl;
    int current = getCurrentOrientation();
    int max = target + ALIGN_ERROR;
    int min = target - ALIGN_ERROR;
    if (min <= 0) {
      min = 360 - abs(min);
    }

    if ((current >= min) & (current <= max)) {
      move(0, 0);
      // std::cout << "Stop" << std::endl;
      aligned = true;
    } else if (current >= max) {
      move(-WHEEL_SPEED, WHEEL_SPEED);
      // std::cout << "Move" << std::endl;
      aligned = false;
    } else if (current <= min) {
      move(WHEEL_SPEED, -WHEEL_SPEED);
      // std::cout << "Move" << std::endl;
      aligned = false;
    }
  }

  int getCurrentHeading() {
    int heading = 0;
    int current = getCurrentOrientation();
    if ((current >= (360 - SECTOR_ANGLE / 2)) |
        (current < (SECTOR_ANGLE / 2))) {
      heading = 0;
    } else if ((current >= SECTOR_ANGLE / 2) &
               (current < (1.5 * SECTOR_ANGLE))) {
      heading = 1;
    } else if ((current >= (1.5 * SECTOR_ANGLE)) &
               (current < (2.5 * SECTOR_ANGLE))) {
      heading = 2;
    } else if ((current >= (2.5 * SECTOR_ANGLE)) &
               (current < (3.5 * SECTOR_ANGLE))) {
      heading = 3;
    } else if ((current >= (3.5 * SECTOR_ANGLE)) &
               (current < (4.5 * SECTOR_ANGLE))) {
      heading = 4;
    } else if ((current >= (4.5 * SECTOR_ANGLE)) &
               (current < (5.5 * SECTOR_ANGLE))) {
      heading = 5;
    } else if ((current >= (5.5 * SECTOR_ANGLE)) &
               (current < (6.5 * SECTOR_ANGLE))) {
      heading = 6;
    } else if ((current >= (6.5 * SECTOR_ANGLE)) &
               (current < (7.5 * SECTOR_ANGLE))) {
      heading = 7;
    }
    return heading;
  }

  bool checkAlignment() {
    bool alignment = false;
    if (getCurrentHeading() == computeAverageHeading()) {
      alignment = true;
    }
    return alignment;
  }

  void saveCompassValues() {
    const double *compassVal = compass->getValues();
    for (int i = 0; i < 3; i++) {
      currentOrientation[i] = compassVal[i];
    }
  }

  double getCurrentOrientation() {
    double angle = 360 - computeVectorAngle(currentOrientation);

    return roundNum(angle);
  }

  double computeVectorMagnitude(std::array<double, 3> components) {
    double resultant = sqrt(pow(components[0], 2) + pow(components[2], 2));
    return roundNum(resultant);
  }

  double computeVectorAngle(std::array<double, 3> components) {
    double x = components[0];
    double z = components[2];
    double angle = 0;
    double pi = 3.141592;

    // Front of robot taken as 0 degrees i.e -z

    if ((x > 0) && (z < 0)) {
      z = -z;
      angle = 90 - atan((z / x)) * 180 / pi;
    } else if ((x > 0) && (z > 0)) {
      angle = 90 + atan((z / x)) * 180 / pi;
    } else if ((x < 0) && (z > 0)) {
      x = -x;
      angle = 180 + (90 - atan((z / x)) * 180 / pi);
    }
    if ((x < 0) && (z < 0)) {
      x = -x;
      z = -z;
      angle = 270 + atan((z / x)) * 180 / pi;
    }

    return roundNum(angle);
  }

  int computeAverageHeading() {
    // std::cout << "Robots " << robots << std::endl;
    int headings[robots];

    std::array<double, ARRAY_SIZE> copy = myData.orientationDouble;

    // if (robots > 0) {
    //   for (int i = 0; i < robots; i++) {
    //     if (copy[i] >= 337.5 | copy[i] < 22.5) {
    //       headings[i] = 0;
    //     } else if (copy[i] >= 22.5 & copy[i] < 67.5) {
    //       headings[i] = 1;
    //     } else if (copy[i] >= 67.5 & copy[i] < 112.5) {
    //       headings[i] = 2;
    //     } else if (copy[i] >= 112.5 & copy[i] < 157.5) {
    //       headings[i] = 3;
    //     } else if (copy[i] >= 157.5 & copy[i] < 202.5) {
    //       headings[i] = 4;
    //     } else if (copy[i] >= 202.5 & copy[i] < 247.5) {
    //       headings[i] = 5;
    //     } else if (copy[i] >= 247.5 & copy[i] < 292.5) {
    //       headings[i] = 6;
    //     } else if (copy[i] >= 292.5 & copy[i] < 337.5) {
    //       headings[i] = 7;
    //     }
    //   }
    // }

    if (robots > 0) {
      for (int i = 0; i < robots; i++) {
        if ((copy[i] >= (360 - SECTOR_ANGLE / 2)) |
            (copy[i] < (SECTOR_ANGLE / 2))) {
          headings[i] = 0;
        } else if ((copy[i] >= SECTOR_ANGLE / 2) &
                   (copy[i] < (1.5 * SECTOR_ANGLE))) {
          headings[i] = 1;
        } else if ((copy[i] >= (1.5 * SECTOR_ANGLE)) &
                   (copy[i] < (2.5 * SECTOR_ANGLE))) {
          headings[i] = 2;
        } else if ((copy[i] >= (2.5 * SECTOR_ANGLE)) &
                   (copy[i] < (3.5 * SECTOR_ANGLE))) {
          headings[i] = 3;
        } else if ((copy[i] >= (3.5 * SECTOR_ANGLE)) &
                   (copy[i] < (4.5 * SECTOR_ANGLE))) {
          headings[i] = 4;
        } else if ((copy[i] >= (4.5 * SECTOR_ANGLE)) &
                   (copy[i] < (5.5 * SECTOR_ANGLE))) {
          headings[i] = 5;
        } else if ((copy[i] >= (5.5 * SECTOR_ANGLE)) &
                   (copy[i] < (6.5 * SECTOR_ANGLE))) {
          headings[i] = 6;
        } else if ((copy[i] >= (6.5 * SECTOR_ANGLE)) &
                   (copy[i] < (7.5 * SECTOR_ANGLE))) {
          headings[i] = 7;
        }
      }
    }

    int m, temp, n;
    for (m = 0; m < robots; m++) {
      for (n = 0; n < robots - m; n++) {
        if (headings[n] > headings[n + 1]) {
          temp = headings[n];
          headings[n] = headings[n + 1];
          headings[n + 1] = temp;
        }
      }
    }

    // for (int i = 0; i < robots; i++) {
    //   std::cout << printName() << " " << headings[i] << std::endl;
    // }

    int target = calculateMode(headings, robots);

    // std::cout << printName() << "mode " << target << std::endl;

    return target;
  }

  int calculateMode(int array[], int size) {
    int counter = 1;
    int max = 0;
    int mode = array[0];
    int oldMode = mode;
    int oldMode2 = mode;

    for (int i = 0; i < size - 1; i++) {
      if (array[i] == array[i + 1]) {
        counter++;
        if (counter >= max) {
          if (oldMode2 == oldMode) {
            max = counter;
            oldMode = mode;
            mode = array[i];
          } else if ((array[i] != mode) & (array[i] != oldMode)) {
            oldMode2 = array[i];
          }
        }
      } else {
        counter = 1; // reset counter.
      }
    }

    if ((mode != oldMode) & (mode != oldMode2) & (oldMode != oldMode2)) {
      switch (choose) {
      case 0:
        mode = mode;
        break;

      case 1:
        mode = oldMode;
        break;

      case 2:
        mode = oldMode2;
        break;
      }
    }

    // std::cout << "oldMode: " << oldMode << std::endl;
    // std::cout << "oldMode2: " << oldMode2 << std::endl;
    // std::cout << "mode: " << mode << std::endl;

    return mode;
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

  int isInteger(auto x) {
    if (x == floor(x)) {
      return 1;
    } else {
      return 0;
    }
  }

  double roundNum(double x) { return floor(x * 100 + 0.5) / 100; }

  std::string printName() {
    std::string name = "[" + robot_name + "] ";
    return name;
  }
};

int main(int argc, char *argv[]) {

  MyRobot *robot = new MyRobot();
  robot->run();
  delete robot;
  return 0;
}
