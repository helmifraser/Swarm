#include "swarm.hpp"

Swarm::Swarm() {

  compass = getCompass("compass");
  compass->enable(TIME_STEP);

  gps = getGPS("gps");
  gps->enable(TIME_STEP);

  std::string ps = "ps";
  std::string led = "led";

  for (unsigned int i = 0; i < 8; i++) {
    distanceSensors[i] = getDistanceSensor(ps.replace(2, 1, std::to_string(i)));
    distanceSensors[i]->enable(TIME_STEP);
    leds[i] = getLED(led.replace(3, 1, std::to_string(i)));
  }

  emitter = getEmitter("emitter");
  emitter->setRange(RANGE);
  receiver = getReceiver("receiver");
  receiver->enable(TIME_STEP);

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

/* this function waits for a keyboard input to specify which mode the simulation
 *    should run in
 */

void Swarm::run() {
  std::cout << "1 for keyboard control, 2 for object avoidance, 3 for "
               "flocking"
            << std::endl;
  std::cout << "Press 5 to stop the simulation. Not doing this will corrupt "
               "the data output."
            << std::endl;
  while (step(TIME_STEP) != -1) {
    int decide = readKey();
    switch (decide) {
    case 49:
      std::cout << "Keyboard control mode (epuck2)" << std::endl;
      std::cout << "WASDQE keys for control" << std::endl;
      teleop();
      break;
    case 50:
      std::cout << "Object avoidance mode" << std::endl;
      objectDetectionMode();
      break;
    case 51:
      std::cout << "Flock" << std::endl;
      flock();
      break;
    }
  }
}

/*
  Flocking algorithm.
  This function provides the swarming behaviour.
  A detailed explanation of which can be found in the Implementation section.
 */

void Swarm::flock() {

  // creates a robot specific .csv file for outputting data to
  std::string filename = printName() + "_data.csv";
  std::ofstream data(filename, std::ios::out);
  if (data.is_open()) {
    data << printName() << "\n";
    data << "x-value,z-value,robots,orientation\n";
  }

  // spins each robot to a random angle
  randomAlign();

  int quit = 0;
  int step_count = 0;

  // this loop runs infinitely until Webots throws an error or the escape key is
  // pressed
  while (step(TIME_STEP != -1) && quit != 1) {

    getReceiverData();
    processReceiverData(myData.orientationString);

    saveCompassValues();
    distanceCheck();

    int index = std::distance(
        signalStrength,
        std::max_element(signalStrength,
                         signalStrength +
                             sizeof(signalStrength) / sizeof(double)));

    std::array<double, 3> direction = {0, 0, 0};
    if (robots > ALIGN_THRESHOLD) {
      for (int k = 0; k < 3; k++) {
        direction[k] = emitterDirection[index + 1][k];
      }
    } else if (robots == 1) {
      for (int k = 0; k < 3; k++) {
        direction[k] = emitterDirection[index][k];
      }
    }

    double nearestNeighbour = computeVectorAngle(direction);

    /* simple finite state machine arbitration

       If the conditions are right to swarm, robot adjusts its heading.
       Else, move away from any obstacles.

     */

    if ((robots >= ALIGN_THRESHOLD) && (!left_obstacle) && (!right_obstacle) &&
        signalStrength[index] < 10) {
      setLEDs(0);
      adjust(nearestNeighbour);

    } else {
      setLEDs(1);
      objectDetection(1.0);
    }

    double robotsDouble = robots;

    /* Dynamically adaptable range.
       This adds a "gravitational" aspect to a large group, encouraging
       robots to aggregate.
     */

    if (robots >= 1) {
      emitter->setRange(RANGE * (robotsDouble + 1));
    }

    sendCurrentOrientation();
    step_count++;

    int quitKey = readKey();

    switch (quitKey) {
    case 53:
      quit = 1;
      break;

    default:
      break;
    }

    std::array<double, 3> position = getGPSValue();

    if (data.is_open()) {
      data << position[0] << ",";
      data << position[2] << ",";
      data << robots << ",";
      data << getCurrentOrientation() << ",";
      data << " \n";
    }
  }
  move(0, 0);
  data.close();
}

void Swarm::randomAlign() {
  int random = randomVal(8);
  while (step(TIME_STEP != -1)) {
    saveCompassValues();
    int test2 = getCurrentHeading();
    if (test2 != random) {
      align(random);
    } else {
      move(0, 0);
      break;
    }
  }
}

int Swarm::randomVal(int range) {
  int robNum =
      atoi(robot_name.substr(1, strlen(robot_name.c_str()) - 1).c_str());
  return matrix[robNum][rand() % 100] % range;
}

void Swarm::objectDetectionMode() {
  setLEDs(1);
  while (step(TIME_STEP != -1)) {
    objectDetection(1.0);
  }
}

void Swarm::teleop() {
  std::string test;
  std::string filename = printName() + "_data_" + test + ".csv";
  std::ofstream data(filename, std::ios::out);

  while (step(TIME_STEP) != -1) {
    if (robot_name.compare("e2") == 0) {
      setLEDs(0);
      // -----get data-----
      int keyPress = readKey();

      saveCompassValues();

      // -----process data-----

      // -----send actuator commands-----
      keyboardControl(keyPress);
      std::array<double, 3> position = getGPSValue();
      std::cout << printName();
      std::cout << "GPS ";
      for (int i = 0; i < 3; i++) {
        std::cout << position[i] << " ";
      }
      std::cout << std::endl;
    }
    if (robot_name.compare("e2") != 0) {
      flock();
      sendCurrentOrientation();
    }
  }
}

void Swarm::computeDirections(std::array<double, 100> &allDirections) {
  std::array<double, 3> vector;
  for (int i = 0; i < robots; i++) {
    for (int j = 0; j < 3; j++) {
      vector[j] = emitterDirection[i][j];
    }

    allDirections[i] = computeVectorAngle(vector);
  }
}

void Swarm::computeCluster(std::array<double, 100> &allDirections,
                           std::array<int, 4> &output) {

  int countArray[4] = {0, 0, 0, 0};
  // Sorts in ascending order
  std::sort(std::begin(allDirections), std::end(allDirections));

  for (int i = 100 - robots; i < 100; i++) {
    if (allDirections[i] >= 0 && allDirections[i] < 90) {
      countArray[0]++;
    } else if (allDirections[i] >= 90 && allDirections[i] < 180) {
      countArray[1]++;
    } else if (allDirections[i] >= 180 && allDirections[i] < 270) {
      countArray[2]++;
    } else if (allDirections[i] >= 270 && allDirections[i] <= 360) {
      countArray[3]++;
    }
  }

  for (int i = 0; i < 4; i++) {
    output[i] = countArray[i];
  }
}

int Swarm::chooseSector(std::array<int, 4> &output) {
  // roulette wheel selection using a fitness function
  int sum = 0;
  std::array<int, 4> outputCopy = output;

  for (int i = 0; i < 4; i++) {
    outputCopy[i] = outputCopy[i] * ROULETTE;
    if (outputCopy[i] == 0) {
      outputCopy[i] = 1;
    }
    sum += outputCopy[i];
  }

  int value = randomVal(sum);
  int i = 0;

  while (value >= 0) {
    value -= outputCopy[i];
    i++;
  }
  i--;

  return i;
}

bool Swarm::checkSector(int sector) {
  int myHeading = getCurrentHeading();
  int mySector = 0;

  if ((myHeading >= 0) & (myHeading < 2)) {
    mySector = 1;
  }

  if ((myHeading >= 2) & (myHeading < 4)) {
    mySector = 2;
  }

  if ((myHeading >= 4) & (myHeading < 6)) {
    mySector = 3;
  }

  if ((myHeading >= 6) & (myHeading <= 7)) {
    mySector = 4;
  }

  if (mySector == sector) {
    return true;
  } else {
    return false;
  }
}

double Swarm::randomSectorAngle(int sector) {
  double angle = randomVal(90);

  switch (sector) {
  case 0:
    angle += 0;
    break;
  case 1:
    angle += 90;
    break;
  case 2:
    angle += 180;
    break;
  case 3:
    angle += 270;
    break;
  }

  return angle;
}

void Swarm::adjust(double angle) {
  double left_speed = WHEEL_SPEED;
  double right_speed = WHEEL_SPEED;

  if (((angle > (360 - ALIGN_ERROR)) & (angle < 360)) |
      ((angle > 0) & (angle < ALIGN_ERROR))) {
    right_speed = left_speed = WHEEL_SPEED;
  } else if ((angle > 270) & (angle < (360 - ALIGN_ERROR))) {
    left_speed =
        (1 + (ALIGN_ERROR + angle - 360) / (90 - ALIGN_ERROR)) * WHEEL_SPEED;
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

  if (left_speed > 1000) {
    left_speed = 1000;
  }

  if (right_speed > 1000) {
    right_speed = 1000;
  }

  move((int)left_speed, (int)right_speed);
}

void Swarm::sendCurrentSpeed() {
  std::string message = std::to_string((int)diffWheels->getLeftSpeed()) + " " +
                        std::to_string((int)diffWheels->getRightSpeed());
  sendPacket(message);
}

void Swarm::sendCurrentOrientation() {
  std::string message = std::to_string(getCurrentOrientation());
  sendPacket(message);
}

void Swarm::getReceiverData() {
  Receiver *copy = (Receiver *)malloc(sizeof(Receiver));
  myData.orientationString = {};
  robots = roundNum((receiver->getQueueLength() + 1) / 2);
  for (int i = 0; i < 100; i++) {
    signalStrength[i] = 0;
  }
  for (int k = 0; k < receiver->getQueueLength(); k++) {
    data = (char *)receiver->getData();
    signalStrength[k] = (double)receiver->getSignalStrength();
    for (int n = 0; n < 3; n++) {
      emitterDirection[k][n] = receiver->getEmitterDirection()[n];
    }
    memcpy(copy, data, sizeof(Receiver));
    myData.orientationString[k] = (char *)copy;
    receiver->nextPacket();
  }
}

void Swarm::processReceiverData(std::array<std::string, ARRAY_SIZE> data) {
  myData.received = false;
  for (unsigned int i = 0; i < ARRAY_SIZE; i++) {
    myData.orientationDouble[i] = 0;
  }
  try {
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
  } catch (std::regex_error &e) {
  }
}

void Swarm::sendPacket(std::string message) {
  std::string temp = printName();
  temp.append(message);
  const char *packet = temp.c_str();
  emitter->send(packet, (strlen(packet) + 1));
}

void Swarm::distanceCheck() {
  for (int i = 0; i < 8; i++) {
    ps_values[i] = checkDistanceSensor(i);
  }

  // detect obsctacles
  right_obstacle = (ps_values[0] > PS_THRESHOLD) ||
                   (ps_values[1] > PS_THRESHOLD) ||
                   (ps_values[2] > PS_THRESHOLD);
  left_obstacle = (ps_values[5] > PS_THRESHOLD) ||
                  (ps_values[6] > PS_THRESHOLD) ||
                  (ps_values[7] > PS_THRESHOLD);
  front_obstacle =
      (ps_values[0] > PS_THRESHOLD) & (ps_values[7] > PS_THRESHOLD);
  back_obstacle = (ps_values[3] > PS_THRESHOLD) & (ps_values[4] > PS_THRESHOLD);
}

void Swarm::objectDetection(double speedAdjust) {
  distanceCheck();

  double left_speed = speedAdjust * WHEEL_SPEED;
  double right_speed = speedAdjust * WHEEL_SPEED;

  if (!left_obstacle & right_obstacle) {
    left_speed -= speedAdjust * WHEEL_SPEED;
    right_speed += speedAdjust * WHEEL_SPEED;
  } else if (!right_obstacle & left_obstacle) {
    left_speed += speedAdjust * WHEEL_SPEED;
    right_speed -= speedAdjust * WHEEL_SPEED;
  } else if (right_obstacle & left_obstacle) {
    left_speed = -speedAdjust * WHEEL_SPEED;
    right_speed = -speedAdjust * WHEEL_SPEED;
  }

  if (left_speed > 1000) {
    left_speed = 1000;
  }

  if (right_speed > 1000) {
    right_speed = 1000;
  }
  move((int)left_speed, (int)right_speed);
}

void Swarm::separation() {
  distanceCheck();

  // init speeds
  double left_speed = WHEEL_SPEED;
  double right_speed = WHEEL_SPEED;

  // modify speeds according to obstacles
  if (left_obstacle | !right_obstacle && !back_obstacle) {
    left_speed = left_speed;
    right_speed = -right_speed;
  } else if (!left_obstacle | right_obstacle && !back_obstacle) {
    left_speed = -left_speed;
    right_speed = right_speed;
  } else if (back_obstacle) {
    left_speed = right_speed = WHEEL_SPEED;
  } else {
    left_speed = right_speed = 0;
  }

  move((int)left_speed, (int)right_speed);
}

void Swarm::align(int heading) {

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
  int current = getCurrentOrientation();
  int max = target + ALIGN_ERROR;
  int min = target - ALIGN_ERROR;
  if (min <= 0) {
    min = 360 - abs(min);
  }

  if ((current >= min) & (current <= max)) {
    move(0, 0);
    aligned = true;
  } else if (current >= max) {
    move(-WHEEL_SPEED, WHEEL_SPEED);
    aligned = false;
  } else if (current <= min) {
    move(WHEEL_SPEED, -WHEEL_SPEED);
    aligned = false;
  }
}

int Swarm::getCurrentHeading() {
  int heading = 0;
  int current = getCurrentOrientation();
  if ((current >= (360 - SECTOR_ANGLE / 2)) | (current < (SECTOR_ANGLE / 2))) {
    heading = 0;
  } else if ((current >= SECTOR_ANGLE / 2) & (current < (1.5 * SECTOR_ANGLE))) {
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

int Swarm::orientationToHeading(int current) {
  int heading = 0;
  if ((current >= (360 - SECTOR_ANGLE / 2)) | (current < (SECTOR_ANGLE / 2))) {
    heading = 0;
  } else if ((current >= SECTOR_ANGLE / 2) & (current < (1.5 * SECTOR_ANGLE))) {
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

bool Swarm::checkAlignment() {
  bool alignment = false;
  if (getCurrentHeading() == computeAverageHeading()) {
    alignment = true;
  }
  return alignment;
}

void Swarm::saveCompassValues() {
  const double *compassVal = compass->getValues();
  for (int i = 0; i < 3; i++) {
    currentOrientation[i] = compassVal[i];
  }
}

std::array<double, 3> Swarm::getGPSValue() {
  const double *dataGPS = gps->getValues();
  std::array<double, 3> GPSout;
  for (int i = 0; i < 3; i++) {
    GPSout[i] = dataGPS[i];
  }

  return GPSout;
}

double Swarm::getCurrentOrientation() {
  double angle = 360 - computeVectorAngle(currentOrientation);

  return roundNum(angle);
}

double Swarm::computeVectorMagnitude(std::array<double, 3> components) {
  double resultant = sqrt(pow(components[0], 2) + pow(components[2], 2));
  return roundNum(resultant);
}

double Swarm::computeVectorAngle(std::array<double, 3> components) {
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

int Swarm::computeAverageHeading() {
  int headings[robots];

  std::array<double, ARRAY_SIZE> copy = myData.orientationDouble;

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

  int target = calculateMode(headings, robots);

  return target;
}

int Swarm::calculateMode(int array[], int size) {
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

  return mode;
}

void Swarm::keyboardControl(int keyPress) {

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

int Swarm::readKey() { return keyboard->getKey(); }

void Swarm::move(int left, int right) { setSpeed(left, right); }

void Swarm::setLEDs(int value) {
  for (int i = 0; i < 8; i++)
    leds[i]->set(value);
}

double Swarm::checkDistanceSensor(int n) {
  return distanceSensors[n]->getValue();
}

double Swarm::roundNum(double x) { return floor(x * 100 + 0.5) / 100; }

std::string Swarm::printName() {
  std::string name = "[" + robot_name + "] ";
  return name;
}

int main(int argc, char *argv[]) {

  Swarm *robot = new Swarm();
  robot->run();
  delete robot;
  return 0;
}
