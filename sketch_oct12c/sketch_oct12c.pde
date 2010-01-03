// Horscht 5
// Horscht 1
#include <Wire.h>
#include <string.h>
#undef int
#include <stdio.h>

// BEGIN VARIABLES
uint8_t outbuf[6];		// array to store raw arduino output
int ledPin = 13;
int MOTOR_1_SPEED_PIN = 6;
int MOTOR_2_SPEED_PIN = 5;
int MOTOR_1_DIRECTION_PIN_A = 7;
int MOTOR_1_DIRECTION_PIN_B = 8;
int MOTOR_2_DIRECTION_PIN_A = 2;
int MOTOR_2_DIRECTION_PIN_B = 4;

//read nunchuk data
int z_button = 0;              // button z state
int c_button = 0;              // button c state
int joy_x_axis;
int joy_y_axis;
int accel_x_axis;
int accel_y_axis;
int accel_z_axis;

// calibrated nunchuk data
int joy_x_norm;
int joy_y_norm;

//constants for calibration
int joy_x_neutral = 127;
int joy_x_min = 0;
int joy_x_max = 254;
int joy_y_neutral = 127;
int joy_y_min = 0;
int joy_y_max = 254;
int deadzone = 3;       // deadzone of the joystick. the zone of +-<deadzone> around joy_x(y)_neutral will be set zero
int deadzone_x_min = 124;
int deadzone_x_max = 130;
int deadzone_y_min = 124;
int deadzone_y_max = 130;

// END VARIABLES

void setup ()
{
  Serial.begin (19200);
  Wire.begin();    // join i2c bus with address 0x52
  nunchuck_init(); // send the initilization handshake
  pinMode(MOTOR_1_SPEED_PIN, OUTPUT);    // motor 1 enable (pwm)
  pinMode(MOTOR_2_SPEED_PIN, OUTPUT);    // motor 2 enable (pwm)
  pinMode(MOTOR_1_DIRECTION_PIN_A, OUTPUT);    // motor 1 rot_a
  pinMode(MOTOR_1_DIRECTION_PIN_B, OUTPUT);    // motor 1 rot_b
  pinMode(MOTOR_2_DIRECTION_PIN_A, OUTPUT);    // motor 2 rot_a
  pinMode(MOTOR_2_DIRECTION_PIN_B, OUTPUT);    // motor 2 rot_b

  p("Finished setup\n");
}

void p (char s[])
{
  Serial.print(s);
}

void nunchuck_init ()
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52
  Wire.send (0x40);		// sends memory address
  Wire.send (0x00);		// sends sent a zero. 
  Serial.print(Wire.endTransmission ());	// stop transmitting
}

void send_zero ()
{
  Wire.beginTransmission (0x52);	// transmit to device 0x52
  Wire.send (0x00);		// sends one byte
  Wire.endTransmission ();	// stop transmitting
}

// ###################### LOOP
void loop ()
{
  // get new nunchuk data
  get_nunchuk_data();
  // enter calibration when c-button pressed
  if (c_button == 1)  calibrate ();
  // motor control
  motor_control();
  // minimum interval
  delay (100);
}

// request data from the nunchuk decode them and print them to serial and in outbuf[]
void get_nunchuk_data ()
{
  int cnt = 0;
  Wire.requestFrom (0x52, 6); // request data from nunchuck
  while (Wire.available() && cnt < 6) // read 6 bytes
    {
      outbuf[cnt] = nunchuk_decode_byte (Wire.receive()); // receive byte as an integer
      cnt++;
    }
  decode_outbuf();
  normalize();
  send_zero (); // send the request for next bytes
  print ();
}

void normalize() {
  // normalize data with calibration data
  // if value is in deadzone the set to zero
  if ((joy_y_axis >= (deadzone_y_min)) && (joy_y_axis <= (deadzone_y_max)))
  {
    joy_y_norm = 0;
    //digitalWrite(7, HIGH);
    //digitalWrite(8, HIGH);
  }
  
  // if value is below deadzone, remap interval (joy_y_min - start of deadzone) to (-255 - 0)
  if (joy_y_axis < (deadzone_y_min))
  {
    joy_y_norm = map(joy_y_axis, joy_y_min, (deadzone_y_min), -255, 0);
  }
  
  // if value is above deadzone, remap interval (end of deadzone - joy_y_max) to (0 - 255) and set rotation direction forward
  if (joy_y_axis > (deadzone_y_max))
  {
    joy_y_norm = map(joy_y_axis, (deadzone_y_max), joy_y_max, 0, 255);
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
  if (joy_y_axis > joy_y_max) joy_y_norm = 255;
  if (joy_y_axis < joy_y_min) joy_y_norm = -255;
  
  // TODO: x-axis normalization = steering
}

void motor_control() {
  if(joy_y_norm > 0) {
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW); 
  }
  if(joy_y_norm < 0) {
    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);
  }
  analogWrite(MOTOR_1_SPEED_PIN, joy_y_norm);
}

// calibration procedure
// release joystick(!!!) then press and hold c-button. while holding, move joystick to all four maxima (+-x and +-y)
// led on "ledPin" will be on during calibration
void calibrate ()
{
  digitalWrite (ledPin, HIGH);	// sets the LED on
  Serial.print("beginning calibration");
  Serial.print ("\r\n");
  joy_x_neutral = outbuf[0];
  joy_x_min = 120;
  joy_x_max = 120;
  joy_y_neutral = outbuf[1];
  joy_y_min = 120;
  joy_y_max = 120;
  while(c_button == 1)
  {    
    if (outbuf[0] < joy_x_min)    joy_x_min = outbuf[0];
    if (outbuf[0] > joy_x_max)    joy_x_max = outbuf[0];
    if (outbuf[1] < joy_y_min)    joy_y_min = outbuf[1];
    if (outbuf[1] > joy_y_max)    joy_y_max = outbuf[1];
    get_nunchuk_data();
    delay(100);
  }
  deadzone_x_min = joy_x_neutral - deadzone;
  deadzone_x_max = joy_x_neutral + deadzone;
  deadzone_y_min = joy_y_neutral - deadzone;
  deadzone_y_max = joy_y_neutral + deadzone;
  
  digitalWrite (ledPin, LOW);	// sets the LED off
  Serial.print("end calibtration");
  Serial.print ("\r\n");
}

// this is just a debugging function which prints stuff to serial port
void print ()
{
  Serial.print (joy_y_axis, DEC);
  Serial.print ("\t");
  
  Serial.print (joy_y_norm, DEC);
  Serial.print ("\t");
  
  Serial.print (joy_y_min);
  Serial.print ("\t");
  
  Serial.print (joy_y_neutral);
  Serial.print ("\t");
  
  Serial.print (joy_y_max);
  Serial.print ("\t");

  //Serial.print (joy_y_axis, DEC);
  //Serial.print ("\t");

  //Serial.print (accel_x_axis, DEC);
  //Serial.print ("\t");

  //Serial.print (accel_y_axis, DEC);
  //Serial.print ("\t");

  //Serial.print (accel_z_axis, DEC);
  //Serial.print ("\t");

  //Serial.print (z_button, DEC);
  //Serial.print ("\t");

  //Serial.print (c_button, DEC);
  //Serial.print ("\t");

  Serial.print ("\r\n");
}

// Encode data to format that most wiimote drivers except
// only needed if you use one of the regular wiimote drivers
char nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}

// recalculation:
// accel data is 10 bits long so we read 8 bits, then we have to add on the last 2 bits.
// => multiply them by 2 * 2
// byte outbuf[5] contains bits for z and c buttons
// it also contains the least significant bits for the accelerometer data so we have to check each bit of byte outbuf[5]
void decode_outbuf() {
  joy_x_axis = outbuf[0];  
  joy_y_axis = outbuf[1];
  accel_x_axis = outbuf[2] * 2 * 2;
  accel_y_axis = outbuf[3] * 2 * 2;
  accel_z_axis = outbuf[4] * 2 * 2;
  z_button = 1;
  c_button = 1;
  if ((outbuf[5] >> 0) & 1)  z_button = 0;
  if ((outbuf[5] >> 1) & 1)  c_button = 0;
  if ((outbuf[5] >> 2) & 1)  accel_x_axis += 2;
  if ((outbuf[5] >> 3) & 1)  accel_x_axis += 1;
  if ((outbuf[5] >> 4) & 1)  accel_y_axis += 2;
  if ((outbuf[5] >> 5) & 1)  accel_y_axis += 1;
  if ((outbuf[5] >> 6) & 1)  accel_z_axis += 2;
  if ((outbuf[5] >> 7) & 1)  accel_z_axis += 1;
}

