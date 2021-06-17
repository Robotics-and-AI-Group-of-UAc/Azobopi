#include "main.h"

// Initialize motors library
ESP32MotorControl MotorControl = ESP32MotorControl();

// Initialize NeoPixel Led if defined
#ifdef NEOPIXEL
Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIXEL, NEO_GRB + NEO_KHZ800);
#endif // ifdef NEOPIXEL

// Timer & Mutex for encoders and PID counters
hw_timer_t  *timer      = NULL;
portMUX_TYPE timerMux   = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE counterMux = portMUX_INITIALIZER_UNLOCKED;

// Initialize PID control for each motor
PID pidleft(&Setpoint, &enc_readL, &val_outputL, kp, ki, kd);
PID pidright(&Setpoint, &enc_readR, &val_outputR, kp, ki, kd);

void Encoders_Interrupt(void)
{
  byte next_state, table_input;

  // Encoder 1
  next_state     = digitalRead(ENC1_A) << 1;
  next_state    |= digitalRead(ENC1_B);
  table_input    = (encoder1_state << 2) | next_state;
  encoder1_pos  -= encoder_table[table_input];
  encoder1_state = next_state;

  // Encoder 2
  next_state     = digitalRead(ENC2_A) << 1;
  next_state    |= digitalRead(ENC2_B);
  table_input    = (encoder2_state << 2) | next_state;
  encoder2_pos  += encoder_table[table_input];
  encoder2_state = next_state;

  counterPID += 1;
}

void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  Encoders_Interrupt();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void startTimer()
{
  if (timer == NULL) {
    timer = timerBegin(2, 4000, true);
    timerAttachInterrupt(timer, &onTimer, true);
  }
  timerAlarmWrite(timer, 1, true);
  timerAlarmEnable(timer);
}

void stopTimer()
{
  if (timer != NULL) {
    timerAlarmDisable(timer);
    timerDetachInterrupt(timer);
    timerEnd(timer);
    timer        = NULL;
    encoder1_pos = 0;
    encoder2_pos = 0;
  }
}

void setLed(int r, int g, int b)
{
    #ifdef NEOPIXEL
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
    #else // ifdef NEOPIXEL

  if (r > 0) r = 1;

  if (g > 0) g = 1;

  if (b > 0) b = 1;
  digitalWrite(RED,   r);
  digitalWrite(GREEN, g);
  digitalWrite(BLUE,  b);
    #endif // ifdef NEOPIXEL
}

void read_cmd_buttons()
{
  button_red.loop();

  if (button_red.isPressed()) {
    mov                      = TURN_LEFT;
    recorded_button[nr_comm] = mov;
    nr_comm++;
  }
  button_blue.loop();

  if (button_blue.isPressed()) {
    mov = FORWARD;

    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
  button_green.loop();

  if (button_green.isPressed()) {
    mov = TURN_RIGHT;

    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
  button_yellow_bottom.loop();

  if (button_yellow_bottom.isPressed()) {
    mov = BACKWARD;

    if (mov != 0) {
      recorded_button[nr_comm] = mov;
      nr_comm++;
    }
    mov = 0;
  }
} // read_cmd_buttons

void odometry()
{}

void init(void)
{
  setLed(0, 0, 255); // set blue

  if (y_count > 3) {
    button_yellow_left.resetCount();
  } // reset

  if (y_count == 1) {
    // Initialize state
    nr_comm = 0;                                         // start the command reading
    memset(recorded_button, 0, sizeof(recorded_button)); // initialize to zero the commands vector
    machine_state = READ_COMM_ST;
  }

  if (y_count == 0) {
    on_execute_test_st = 0;
    on_execute_comm_st = 0;
  }
}

void readComm(void)
{
  setLed(255, 0, 0);               // red

  if (nr_comm < MAX_NR_COMMANDS) { // it only keeps the first max_nr_commands...
    read_cmd_buttons();
  }

  // -- wait for the y_count = 2 and nr_commands != 0
  if (y_count == 2 and nr_comm != 0) {
    machine_state = START_EXEC_ST;
  }
}

void startExec(void)
{
  setLed(0, 255, 0); // green
  button_blue.loop();

  if (y_count > 2) {
    if (on_execute_comm_st == 1) {
      button_yellow_left.resetCount();
    }
    machine_state = INIT_ST;
  }

  if (button_blue.isPressed()) {
    comm_index         = nr_comm;
    on_execute_comm_st = 1; // executed at least once ...
    machine_state      = EXEC_ST;
  }
}

void exec(void)
{
  comm_index--;

  if (comm_index >= 0) { // avoid getting nonsense data
    int action = recorded_button[(nr_comm - 1) - comm_index];

    if (action == FORWARD) {
      machine_state = FORWARD_ST;
    } else if (action == BACKWARD) {
      machine_state = BACK_ST;
    } else if (action == TURN_LEFT) {
      machine_state = TURN_LEFT_ST;
    } else if (action == TURN_RIGHT) {
      machine_state = TURN_RIGHT_ST;
    }
  }

  if (comm_index < 0) {             // no more commands
    button_blue.loop();

    if (button_blue.isReleased()) { // wait till button releases state
      machine_state = START_EXEC_ST;
    }
  }
}

void stopAll(void)
{
  MotorControl.motorsStop();

  if (millis() >= time_now + STOP_DELAY) {
    machine_state = stop_next_state;
  }
}

void turnRight(void)
{
  /*float distance = a / (360 * CURVE_CIRCUMFERENCE);
   * float nRevol = distance / WHEEL_CIRCUMFERENCE;
   * float encTarget = nRevol * ROTATION_TICKS;*/

  if ((abs(encoder1_pos) < SETPOINT_TURN) &&
      (abs(encoder2_pos < SETPOINT_TURN)))
  {
    startTimer();

    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorForward(0, vel);
    MotorControl.motorReverse(1, ver);

    if (counterPID > 50) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void turnLeft(void)
{
  /*float distance = a / (360 * CURVE_CIRCUMFERENCE);
   * float nRevol = distance / WHEEL_CIRCUMFERENCE;
   * float encTarget = nRevol * ROTATION_TICKS;*/
  if ((abs(encoder1_pos) < SETPOINT_TURN) &&
      (abs(encoder2_pos < SETPOINT_TURN)))
  {
    startTimer();
    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorReverse(0, vel);
    MotorControl.motorForward(1, ver);

    if (counterPID > 50) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void forward(void)
{
  if ((abs(encoder1_pos) < SETPOINT_RUN) &&
      (abs(encoder2_pos) < SETPOINT_RUN)) {
    startTimer();

    int vel = kspeed * (speedL + val_outputL);
    int ver = kspeed * (speedR + val_outputR);
    MotorControl.motorForward(0, vel);
    MotorControl.motorForward(1, ver);

    if (counterPID > freq) {
      portENTER_CRITICAL_ISR(&counterMux);
      counterPID = 0;
      portEXIT_CRITICAL_ISR(&counterMux);
      enc_readL = encoder1_pos;
      enc_readR = encoder2_pos;
      pidleft.Compute();
      pidright.Compute();

      /*DEBUG_PRINT(enc_readL);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(val_outputL);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(vel);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(enc_readR);
       * DEBUG_PRINT(",");
       * DEBUG_PRINT(val_outputR);
       * DEBUG_PRINT(",");
       * DEBUG_PRINTLN(ver);*/
    }
  } else {
    stopTimer();
    time_now = millis();

    stop_next_state = EXEC_ST;
    machine_state   = STOP_ST;
  }
}

void fsm(void)
{
  button_yellow_left.loop(); // loop() for left yellow
  y_count = button_yellow_left.getCount();

  switch (machine_state) {
  case INIT_ST:
    init();
    break;

  case READ_COMM_ST:
    readComm();
    break;

  case START_EXEC_ST:
    startExec();
    break;

  case EXEC_ST:
    exec();
    break;

  case FORWARD_ST:
    forward();
    break;

  case TURN_RIGHT_ST:
    turnRight();
    break;

  case TURN_LEFT_ST:
    turnLeft();
    break;

  case STOP_ST:
    stopAll();
    break;

  case VOID_ST:
    stopAll();
    break;
  }
} // fsm

void setup()
{
  // Encoders Pins
  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);

  // set debounce time to 50 milliseconds
  button_yellow_left.setDebounceTime(50);
  button_yellow_left.setCountMode(COUNT_FALLING);
  button_red.setDebounceTime(50);
  button_blue.setDebounceTime(50);
  button_green.setDebounceTime(50);
  button_yellow_bottom.setDebounceTime(50);

  // Motor Pins
  MotorControl.attachMotors(25, 26, 32, 33); // ROBOT José trocar 25 por 27

  // LEDs
  #ifdef NEOPIXEL
  pixels.begin(); // INITIALIZE NeoPixel strip object
  pixels.clear(); // Set all pixel colors to 'off'
  #else // ifdef NEOPIXEL
  pinMode(RED,   OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE,  OUTPUT);
  #endif // ifdef NEOPIXEL

  Serial.begin(115200);
  machine_state = INIT_ST;
}

void loop()
{
  fsm();
}
