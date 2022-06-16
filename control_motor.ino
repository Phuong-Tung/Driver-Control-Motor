
int char_temp = 0;    // for incoming serial data
int8_t vel_idx = 0;
float twist_v;
float twist_w;
char* c_token;
static int8_t is_processing = 0;
static int8_t buf_temp[20];

volatile int16_t pulse_max_right = 0;
volatile int16_t pulse_max_left = 0;
volatile int16_t pulse_counter_right = 0;
volatile int16_t pulse_counter_left = 0;
volatile int8_t pulse_on_right = 0;
volatile int8_t pulse_on_left = 0;

#define sgn(x) ((x) < 0 ? -1 : 1)

#define ENA_PIN_LEFT    2
#define DIR_PIN_LEFT    3
#define PUL_PIN_LEFT    4
#define ENA_PIN_RIGHT   5
#define DIR_PIN_RIGHT   6
#define PUL_PIN_RIGHT   7

#define LOCK_TIMER_TICK      1   // 2 * 0.5 us
#define TIMER_TICK      (100 * LOCK_TIMER_TICK)   // us

#define PPR             800
#define RADIUS          0.04
#define LENGTH          0.4
#define PI              3.14

#define V_MAX           0.75    // m/s
#define V_MIN           0.01    // m/s

void setup() {
  Serial.begin(115200);    // opens serial port, sets data rate to 9600 bps
  set_timer();
  pinMode(ENA_PIN_LEFT, OUTPUT);
  pinMode(DIR_PIN_LEFT, OUTPUT);
  pinMode(PUL_PIN_LEFT, OUTPUT);
  pinMode(ENA_PIN_RIGHT, OUTPUT);
  pinMode(DIR_PIN_RIGHT, OUTPUT);
  pinMode(PUL_PIN_RIGHT, OUTPUT);
  pinMode(PUL_PIN_RIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
//
  enable_left_motor(false);
  enable_right_motor(false);
//  pulse_max_left = 12;
//  digitalWrite(ENA_PIN_LEFT, LOW);
//  digitalWrite(PUL_PIN_LEFT, HIGH);
//  digitalWrite(DIR_PIN_LEFT, HIGH);
}

void loop() {
  // send data only when you receive data:
  if (Serial.available() > 0) {
    bool rs = get_vel();
    if (rs) {
      control_motor(twist_v, twist_w);
    }
  }
}

void control_motor(float v, float w) {
  float vl = (2 * v + w * LENGTH) / 2;
  float vr = (2 * v - w * LENGTH) / 2;

  if(abs(vl) > V_MAX) {
    vl = sgn(vl) * V_MAX;
  }
  if(abs(vr) > V_MAX) {
    vr = sgn(vr) * V_MAX;
  }

  if(abs(vl) < V_MIN) {
     enable_left_motor(false);
  }
  else {
    enable_left_motor(true);
  }
  if(abs(vr) < V_MIN) {
     enable_right_motor(false);
  }
  else {
    enable_right_motor(true);
  }
  if((abs(vl) < V_MIN) && (abs(vr) < V_MIN)) {
    return ;
  }

  Serial.println("---------------------------");
  Serial.println(vl);
  Serial.println(vr);
  
  // Time of a half of pulse
  float Thl = (PI * RADIUS * 1000000) / (vl * PPR);     // in us
  float Thr = (PI * RADIUS * 1000000) / (vr * PPR);     // in us
  control_step(Thl / TIMER_TICK, Thr / TIMER_TICK);

  Serial.println(Thl);
  Serial.println(Thr);
  Serial.println(pulse_max_left);
  Serial.println(pulse_max_right);
}

void enable_left_motor(bool ena) {
  if(ena) {
    digitalWrite(ENA_PIN_LEFT, LOW);
  }
  else {
    digitalWrite(ENA_PIN_LEFT, HIGH);
  }
}

void enable_right_motor(bool ena) {
  if(ena) {
    digitalWrite(ENA_PIN_RIGHT, LOW);
  }
  else {
    digitalWrite(ENA_PIN_RIGHT, HIGH);
  }
}

void dir_left_motor(bool dir) {
  if(dir) {
    digitalWrite(DIR_PIN_LEFT, LOW);
  }
  else {
    digitalWrite(DIR_PIN_LEFT, HIGH);
  }
}

void dir_right_motor(bool dir) {
  if(dir) {
    digitalWrite(DIR_PIN_RIGHT, HIGH);
  }
  else {
    digitalWrite(DIR_PIN_RIGHT, LOW);
  }
}

void control_step(float tl, float tr) {
  if(tl > 0) {
    dir_left_motor(false);
    pulse_max_left = (int)tl;
  }
  else {
    dir_left_motor(true);
    pulse_max_left = (int)(-tl);
  }
  if(tr > 0) {
    dir_right_motor(false);
    pulse_max_right = (int)tr;
  }
  else {
    dir_right_motor(true);
    pulse_max_right = (int)(-tr);
  }
}

void set_timer() {
  cli();              // tắt ngắt toàn cục

  /* Reset Timer/Counter1 */
  TCCR1A = 0;
  TCCR1B = 0;
  TIMSK1 = 0;

  /* Setup Timer/Counter1 */
  // prescale = 8 and CTC mode 4
  TCCR1B |= (1 << WGM12) | (1 << CS11);
  // initialize OCR1A
  OCR1A = TIMER_TICK;
  TIMSK1 = (1 << OCIE1A);     // Output Compare Interrupt Enable Timer/Counter1 channel A
  sei();                      // cho phép ngắt toàn cục
}

ISR (TIMER1_COMPA_vect)
{
  if (pulse_counter_left > pulse_max_left) {
    pulse_counter_left = 0;
    if (pulse_on_left) {
      pulse_on_left = 0;
    }
    else {
      pulse_on_left = 1;
    }
  }
  if (pulse_counter_right > pulse_max_right) {
    pulse_counter_right = 0;
    if (pulse_on_right) {
      pulse_on_right = 0;
    }
    else {
      pulse_on_right = 1;
    }
  }
  if (pulse_on_left) {
    digitalWrite(PUL_PIN_LEFT, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(PUL_PIN_LEFT, LOW);
    digitalWrite(LED_BUILTIN, LOW);
  }
  if (pulse_on_right) {
    digitalWrite(PUL_PIN_RIGHT, HIGH);
  }
  else {
    digitalWrite(PUL_PIN_RIGHT, LOW);
  }
  pulse_counter_left++;
  pulse_counter_right++;
}

bool get_vel() {
  char_temp = Serial.read();

  if (char_temp == ']')
  {
    is_processing = 0;
    //
    c_token = strtok((char*)buf_temp, ",");
    if (c_token != NULL)
    {
      twist_v = atof(c_token);
      c_token = strtok(NULL, ",");
      if (c_token != NULL)
      {
        twist_w = atof(c_token);
      }
    }

    // TODO: Cal encoder

    return 1;
  }
  if (is_processing)
  {
    buf_temp[vel_idx++] = char_temp;
  }
  if (char_temp == '[')
  {
    vel_idx = 0;
    is_processing = 1;
  }
  return 0;
}
