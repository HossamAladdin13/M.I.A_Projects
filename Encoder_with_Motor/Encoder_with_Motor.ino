// Encoder working with motor
// Encoder calculates motor speed

#define encoder_output_per_rotation 2500 // change it to the real value
#define chanelA_pin PB13 // the encoder output connectted to this pin for chanel A
#define chanelB_pin PB12 // chanel B pin
#define pwm PA7
#define dir PA6

volatile long encoder_output_value = 0;
int RPM = 0;
// counters for the time to calculate the motor speed
long pre_millis = 0 ;
long cur_millis = 0 ;

void setup() {
  Serial3.begin(115200);
  pinMode(chanelA_pin , INPUT_PULLUP);
  pinMode(chanelB_pin , INPUT_PULLUP);
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
  attachInterrupt(chanelA_pin, interruptA_fun, CHANGE);
  attachInterrupt(chanelB_pin, interruptB_fun, CHANGE);
  digitalWrite(dir, HIGH);

  pre_millis = millis() ;
}

void loop() {
  analogWrite(pwm, 255);
  cur_millis = millis ();
  if (cur_millis - pre_millis > 1000) { // if one second passed
    RPM = (float)((encoder_output_value * 60) / encoder_output_per_rotation);
    Serial3.print("motor speed = ");
    Serial3.println(RPM);
    encoder_output_value = 0 ;
    pre_millis = cur_millis ;
  }
  
}
void interruptA_fun() {
  if (digitalRead(chanelB_pin) != digitalRead(chanelA_pin) ) {
    encoder_output_value ++ ;
  } else {
    encoder_output_value -- ;
  }
}

void interruptB_fun () {
  if (digitalRead(chanelB_pin) == digitalRead(chanelA_pin) ) {
    encoder_output_value ++ ;
  } else {
    encoder_output_value -- ;
  }
}

/*
  clockwise
  when A on Rising state B is High
     ____      ____      ____      ____
  ____|    |____|    |____|    |____|    |  (((A)))
   ____      ____      ____      ____
  __|    |____|    |____|    |____|    |    (((B)))
  counterclockwise
  when A on Rising state B is LOw
   ____      ____      ____      ____
  __|    |____|    |____|    |____|    |    (((A)))
     ____      ____      ____      ____
  ____|    |____|    |____|    |____|    |  (((B)))
*/
