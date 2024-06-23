#include <Arduino.h>

const int PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 500;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 8; // We'll use same resolution as Uno (8 bits, 0-255) but ESP32 can go up to 16 bits

const int Micro_Pin = 18; // limit switch pin
int state = 0; // limit switch state


// The max duty cycle value based on PWM resolution (will be 255 if resolution is 8 bits)
const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);
const int DELAY_MS = 100;  // delay between fade increments

const int ENA = 14; // motor enable
const int IN1 = 27; // input 1
const int IN2 = 26; // input2

int active = IN1; // pwm pin
int steady = IN2; // 0 pin

const uint8_t  pinA = 12;
const uint8_t  pinB = 13;

int counter = 0; // encoder counter

int max_height = 35; // maximum height

int preset = true; // beginning calibration
int height = 0;


// pid variables defined
float arr[10] = {0};
float target = 17; // setpoint
//float target = 3.5; // cmps target
float err = 0;
double pid = 100;

float sig = 0; // sum of errors

double kp = 25.5; // proportional
double kd = 15.9; // derivative
double ki = 10; // integral


void IRAM_ATTR A()
{
  if (digitalRead(pinB) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void IRAM_ATTR B()
{
  if (digitalRead(pinA) == HIGH)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

// void myDelay(unsigned long duration)
// {
//   // delay function to use instead of delay()
//   unsigned long start = millis();
//   while (millis() - start <= duration);
// }


void setup() {

  Serial.begin(115200);

  // limitSwitch.setDebounceTime(50); // set debounce time to 50 milliseconds

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);


  // encoder pins
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  // attach interrupt to encoder pins 
  attachInterrupt(digitalPinToInterrupt(pinA), A, RISING);
  attachInterrupt(digitalPinToInterrupt(pinB), B, RISING);

  Serial.println("--------------- setup done -----------------");


}


void loop() {


  state = digitalRead(Micro_Pin); // low if pushed 
  delay(50);
  
  Serial.print("state : ");
  Serial.println(state);



  // calibaration
  if (preset){
    // motor direction
    digitalWrite(ENA, HIGH);

    analogWrite(steady, 0);
 
    while(state != 0){ // don't update pid before calibration
      state = digitalRead(Micro_Pin); // low if pushed 

      analogWrite(active, 80); // slowly go down to touch the limit switch

      delay(DELAY_MS);

  Serial.print("counter : ");
  Serial.println(counter);


    }
    steady = IN1;
    active = IN2;

    digitalWrite(ENA, LOW);
    // analogWrite(steady, 0);
    // analogWrite(active, 5);

    delay(1000);
    digitalWrite(ENA, HIGH);

    preset = false; // end of calibration
    counter = 0; // set encoder data to 0
  }


  height = -1 * counter / 200;

  Serial.print("height : ");
  Serial.println(height); // encoder data
  // read encoder position 
 
  // start the control loop

  if (height == max_height){
    active = IN1;
    steady = IN2;
  }
  else{
    active = IN2;
    steady = IN1;
  }

  analogWrite(steady, 0);

  analogWrite(active , pid);

  Serial.print("pid : ");
  Serial.println(pid);

  err = target - height; // pid error
  
  arr[0] = err;
  sig = 0;
  for (int i = 0; i < 10; i++){
    sig += arr[i]; // intergal term
  }

  pid = kp*err + kd*(arr[0] - arr[1]) + ki*sig;

  if (pid > 255) pid = 255;

}