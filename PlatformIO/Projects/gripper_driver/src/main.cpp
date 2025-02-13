#include <Arduino.h>
#include <Servo.h> //this enables the library for the servo motor

#define IN0 2 /* D2  */
#define IN1 3 /* D3  */
#define IN2 4 /* D4  */
#define IN3 7 /* D7  */

#define OUT0 8  /* D8  */
#define OUT1 9  /* D9  */
#define OUT2 10 /* D10 */
#define OUT3 14 /* A0 */

#define ADC0 20 /* A6 */
#define ADC1 17 /* A3 */
#define ADC2 16 /* A2 */
#define ADC3 21 /* A7 */

#define PWM0 5 /* D5 */
#define PWM1 6 /* D6 */

#define ENABLE0 15 /* A1 */
#define ENABLE1 19 /* A5 */

#undef F
#define F(x) x

/*Membrane sens trigger voltage from V div*/
#define PRESSURE_TH_V 3
/*10-bit, V threshold, (2^10/5V)*th_voltage */
#define PRESSURE_TH_ADC (int)(204.8 * (float)PRESSURE_TH_V)
/* Sensor is connected to A2*/
#define PRESSURE_ADC ADC2
/*Depends on gripper design, can offset '0' position by compensating for it*/
#define SERVO_OFFSET 0
#define SERVO_START_POS 0

/*Degrees per mS, datasheep max: 0.17 s/60º (4.8 V) ~~ 2.5mS max*/
#define SERVO_OPEN_MS 260

/* Moves 90 +- 60 */
#define SERVO_STOP_VAL 90
#define SERVO_CLOSE_VAL 80
#define SERVO_SQUEEZE_VAL 87
#define SERVO_OPEN_VAL 100


// #define SERIAL_CONTROL

typedef enum servo_driver_states
{
  initial,
  open,
  closing,
  holding,
  opening
};

servo_driver_states gripper_state = initial;

/*movement range goes from 0 - 120 degs*/
// uint8_t servo_position = 0;
// uint8_t servo_setpoint = 0;
// uint32_t last_increment_time = 0;

static const uint8_t g_Major = 0;
static const uint8_t g_Minor = 3;
static const char g_info[] = F("info");
static const char g_goto[] = F("goto");
static const char g_low[] = F("low");
static const char g_high[] = F("high");
static const char g_cursor[] = F(">>");

static const uint32_t flashInterval = 500; /* milliseconds */

struct pin_definition
{
  const uint8_t pin;
  const char *name;
};

struct analog_pin_definition
{
  const uint8_t pin;
  const char *name;
  uint16_t period;
  unsigned long lastPublish;
};

static Servo g_servoMotor0;
static Servo g_servoMotor1;

struct pin_definition input_pins[] = {{IN0, F("IN0")}, {IN1, F("IN1")}, {IN2, F("IN2")}, {IN3, F("IN3")}};
struct pin_definition output_pins[] = {{OUT0, F("OUT0")}, {OUT1, F("OUT1")}, {OUT2, F("OUT2")}, {OUT3, F("OUT3")}, {ENABLE0, F("EN0")}, {ENABLE1, F("EN1")}};
struct analog_pin_definition analog_pins[] = {{ADC0, F("ADC0"), 0, 0}, {ADC1, F("ADC1"), 0, 0}, {ADC2, F("ADC2"), 0, 0}};

void Help()
{
  Serial.print(F("Levelshifter board "));
  Serial.print(g_Major);
  Serial.print(F("."));
  Serial.print(g_Minor);
  Serial.println(F("---------------------------------------"));
  Serial.println(F("'?' this screen"));
  Serial.println(F("'info' display the current settings of in and out"));
}

void Info()
{

  // First time initialization, we report the current state of the output pins
  for (uint8_t index = 0; index < sizeof(output_pins) / sizeof(struct pin_definition); index++)
  {
    Serial.print(F("OUTPUT: "));
    Serial.print(output_pins[index].name);
    Serial.print(F("="));
    Serial.println(digitalRead(output_pins[index].pin) ? g_high : g_low);
  }

  // First time initialization, we report and record the current state for the input pins
  for (uint8_t index = 0; index < sizeof(input_pins) / sizeof(struct pin_definition); index++)
  {
    Serial.print("INPUT:  ");
    Serial.print(input_pins[index].name);
    Serial.print('=');

    Serial.println(digitalRead(input_pins[index].pin) == 0 ? g_low : g_high);
  }
}

void Goto(Servo *servo, int16_t position)
{
  // Serial.print("Moving to: ");
  // Serial.println(position);
  servo->write(position);
}

void UnderPressure(const uint8_t pin)
{
  uint16_t pressure = analogRead(pin);
  Serial.println(pressure);
}

void UserEvaluation(const uint8_t length, const char buffer[])
{

  if ((length == 1) && (buffer[0] == '?'))
  {
    Help();
  }
  else if (strncmp(g_info, buffer, (sizeof(g_info) / sizeof(char) - 1)) == 0)
  {
    Info();
  }
  else if (strncmp(g_goto, buffer, (sizeof(g_goto) / sizeof(char) - 1)) == 0)
  {
    if ((buffer[sizeof(g_goto) / sizeof(char)] == '=') && (length >= (sizeof(g_goto) / sizeof(char) + 2)))
    {
      int degrees = atoi(&(buffer[sizeof(g_goto) / sizeof(char) + 1]));
      if (buffer[sizeof(g_goto) / sizeof(char) - 1] == '0')
      {
        Goto(&g_servoMotor0, degrees);
      }
      else
      {
        Goto(&g_servoMotor1, degrees);
      }
    }
  }
  else
  {
    bool handled = false;

    for (uint8_t index = 0; ((handled == false) && (index < sizeof(output_pins) / sizeof(struct pin_definition))); index++)
    {
      const uint8_t keyLength = strlen(output_pins[index].name);
      if (strncmp(output_pins[index].name, buffer, keyLength) == 0)
      {
        handled = true;
        if ((buffer[keyLength] != '=') || (length != (keyLength + 2)))
        {
          Serial.println(F("Syntax should be '<pin name>=0' or '<pin name>=1'"));
        }
        else
        {
          const char value = toupper(buffer[keyLength + 1]);
          if ((value == 'L') || (value == '0'))
          {
            digitalWrite(output_pins[index].pin, 0);
          }
          else if ((value == 'H') || (value == '1'))
          {
            digitalWrite(output_pins[index].pin, 1);
          }
          else
          {
            Serial.println(F("Unrecognized value, must be '0', '1', 'l', 'h', 'L' or 'H'"));
          }
        }
      }
    }

    for (uint8_t index = 0; ((handled == false) && (index < sizeof(analog_pins) / sizeof(struct analog_pin_definition))); index++)
    {
      const uint8_t keyLength = strlen(analog_pins[index].name);
      if (strncmp(analog_pins[index].name, buffer, keyLength) == 0)
      {
        handled = true;
        if ((buffer[keyLength] != '=') || (length != (keyLength + 2)))
        {
          Serial.println(F("Syntax should be '<pin name>=<duration in 100ms>'"));
        }
        else
        {
          const uint16_t value = atoi(&(buffer[keyLength + 1]));
          analog_pins[index].period = value * 100;
          analog_pins[index].lastPublish = 0;
          if (value == 0)
          {
            Serial.print(F("Disabled the monitoring of: "));
            Serial.println(analog_pins[index].name);
          }
          else
          {
            Serial.print(F("Enabled monitoring of ["));
            Serial.print(analog_pins[index].name);
            Serial.print(F("] to a value of: "));
            Serial.print(analog_pins[index].period);
            Serial.println(F(" ms"));
          }
        }
      }
    }

    if (handled == false)
    {
      Serial.print(F("Unrecognized verb: "));
      Serial.println(buffer);
    }
  }
  Serial.print(F(">>"));
}

void InputEvaluation(uint32_t &state)
{
  // First time initialization, we report and record the current state
  for (uint8_t index = 0; index < sizeof(input_pins) / sizeof(struct pin_definition); index++)
  {
    bool value = !(digitalRead(input_pins[index].pin) == 0);

    // Check if we have a different state than we recorded..
    if (value ^ ((state & (1 << index)) != 0))
    {
      // Yes, so time to notify
      Serial.println();
      Serial.print(F("INPUT:  "));
      Serial.print(input_pins[index].name);
      Serial.print(F("="));
      Serial.println(value ? g_high : g_low);
      Serial.print(F(">>"));

      // and record!!
      state ^= (1 << index);
    }
  }
}

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
static unsigned long g_previousFlashMillis = 0;

// This is the buffer to remember what the user is entering through the
// serial console
static char g_buffer[16];
static uint8_t g_length;

static uint32_t g_lastState = 0;

/*returns true/false depending if object is gripped.*/
bool object_gripped()
{
  if (analogRead(PRESSURE_ADC) <= PRESSURE_TH_ADC)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void init_motor_endstop()
{
  static uint8_t ministate = 0;
  static uint32_t time_capt = 0;

  switch (ministate)
  {
  case 0:

    Goto(&g_servoMotor0, SERVO_CLOSE_VAL);

    if (object_gripped())
    {
      Serial.print("detected :");
      Serial.println(object_gripped ? "touch" : "notouch");

      ministate = 1;
      time_capt = millis();
      return;
    }

    break;

  case 1:
    Goto(&g_servoMotor0, SERVO_OPEN_VAL);

    if ((millis() - time_capt) >= (SERVO_OPEN_MS))
    {
      ministate = 2;
      return;
    }

    break;

  case 2:

    Goto(&g_servoMotor0, SERVO_STOP_VAL);
    gripper_state = open;
    Serial.println("Done");
    break;

  default:
    break;
  }
}

void motor_system_state_machine()
{

  static uint32_t process_start_time = 0;
  static uint32_t grip_open_delta_time = 0;

  switch (gripper_state)
  {
  case initial:
    init_motor_endstop();
    break;

  case open:

    Goto(&g_servoMotor0, SERVO_STOP_VAL);

    if (digitalRead(IN0))
    {
      Serial.println("STMCN: Close init");
      process_start_time = millis();
      gripper_state = closing;
      return;
    }
    break;

  case closing:

    Goto(&g_servoMotor0, SERVO_CLOSE_VAL);

    if (object_gripped())
    {

      /*Check if we practically close without grabbing something. */
      if ((millis() - process_start_time) >= (SERVO_OPEN_MS - 2))
      {
        Serial.println("STMCN: OBJ NOT grabbed.");
        grip_open_delta_time = millis() - process_start_time;
        /*Clip it in case touch isn't detectec*/
        if (grip_open_delta_time > SERVO_OPEN_MS)
        {
          grip_open_delta_time = SERVO_OPEN_MS;
        }

        Serial.println("reached endstop");
        gripper_state = holding;
        digitalWrite(OUT1, HIGH);
        return;
      }
      else /* if it was under gripped before timeout*/
      {
        /* Records how long to open for*/
        Serial.println("STMCN: OBJ grabbed.");
        grip_open_delta_time = millis() - process_start_time;
        /*Clip it in case touch isn't detectec*/
        if (grip_open_delta_time > SERVO_OPEN_MS)
        {
          grip_open_delta_time = SERVO_OPEN_MS;
        }
        gripper_state = holding;
        digitalWrite(OUT0, HIGH);
        return;
      }
    }

    break;

  case holding:

    Goto(&g_servoMotor0, SERVO_SQUEEZE_VAL);
    /*if grip signal is not there anymore*/
    if (!digitalRead(IN0))
    {
      Serial.println("STMCN: Grabber released.");
      process_start_time = millis();
      gripper_state = opening;
      return;
    }

    break;

  case opening:

    Goto(&g_servoMotor0, SERVO_OPEN_VAL);
    digitalWrite(OUT0, LOW);
    digitalWrite(OUT1, LOW);

    if ((millis() - process_start_time) >= grip_open_delta_time)
    {
      gripper_state = open;
      return;
    }

    break;

  default:
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.flush();

  // Initialize digital pin LED_BUILTIN as an output, just for DEBUG purpose.
  pinMode(LED_BUILTIN, OUTPUT);

  // First time initialization, we report the current state of the output pins
  for (uint8_t index = 0; index < sizeof(output_pins) / sizeof(struct pin_definition); index++)
  {
    pinMode(output_pins[index].pin, OUTPUT);
  }

  // First time initialization, we report and record the current state for the input pins
  for (uint8_t index = 0; index < sizeof(input_pins) / sizeof(struct pin_definition); index++)
  {
    pinMode(input_pins[index].pin, INPUT);
    if (digitalRead(input_pins[index].pin) != 0)
    {
      g_lastState |= (1 << index);
    }
  }

  Help();
  Info();
  Serial.print(g_cursor);

  g_servoMotor0.attach(PWM0);
  g_servoMotor1.attach(PWM1);
}

void loop()
{

  unsigned long currentMillis = millis();

  if (static_cast<uint32_t>(currentMillis - g_previousFlashMillis) >= flashInterval)
  {
    g_previousFlashMillis = currentMillis;
    // Serial.println(gripper_state);
  }

  if (object_gripped())
  {
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

#ifdef SERIAL_CONTROL
  /* Handle serial buffer and feed into UserEval*/
  while (Serial.available() > 0)
  {
    char character = Serial.read();
    if (::isprint(character))
    {
      if (g_length < sizeof(g_buffer))
      {
        g_buffer[g_length++] = character;
        Serial.print(character);
      }
      else
      {
        Serial.print(F("*"));
      }
    }
    else
    {
      g_buffer[g_length] = '\0';
      Serial.println();
      UserEvaluation(g_length, g_buffer);
      g_length = 0;
      Serial.print(g_cursor);
    }
  }
#else
  motor_system_state_machine();
#endif

#ifdef VERBOSE_SERIAL

  /* print changes */
  InputEvaluation(g_lastState);

  for (uint8_t index = 0; (index < sizeof(analog_pins) / sizeof(struct analog_pin_definition)); index++)
  {
    if ((analog_pins[index].period != 0) && (analog_pins[index].lastPublish < currentMillis))
    {
      analog_pins[index].lastPublish = currentMillis + analog_pins[index].period;
      UnderPressure(analog_pins[index].pin);
    }
  }
#endif
}
