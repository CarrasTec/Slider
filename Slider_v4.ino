#define BAUD_RATE 115200
#define RX_BUFFER_SIZE 128
#define FAIL(status) return(status);
#define STATUS_OK 0
#define STATUS_EXPECTED_COMMAND_LETTER 1
#define STATUS_BAD_NUMBER_FORMAT 2
#define MAX_INT_DIGITS 8

char receivedChars[RX_BUFFER_SIZE];
int status_engine=0;
int velocidad = 0;

int Slide_Steps = 0;
int Pan_Steps = 0;
int Slide_Steps_T = 0;
int Pan_Steps_T = 0;

int Slide_Dist = 0;
int Pan_Angle = 0;

#define Slide_Step 2  //Digital 2
#define Slide_Dir 3    //Digital 3
#define Pan_Step 4   //Digital 4
#define Pan_Dir 5     //Digital 5
#define SPEED_STEP 1
#define JoyX1 A0
#define JoyY2 A1
#define JoyP1 6 //Digital 6
#define JoyP2 7 //Digital 6

int JoyXValue;
int JoyYValue;

int stepdelay_min=600;
int stepdelay_max=2000;
int8_t motorAfw=1;
int8_t motorAbk=-1;
int8_t motorBfw=1;
int8_t motorBbk=-1;

void stepper_init()
{
  // Configure step and direction interface pins
  pinMode(Slide_Step, OUTPUT);
  pinMode(Slide_Dir, OUTPUT);
  pinMode(Pan_Step, OUTPUT);
  pinMode(Pan_Dir, OUTPUT);
  digitalWrite(Slide_Dir, LOW);
  digitalWrite(Pan_Dir, LOW);
}

bool serial_read(char *line) {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  const byte numChars = RX_BUFFER_SIZE;
  bool nuevo_dato = false;
  line[ndx] = '\0';
  while (Serial.available() > 0 && nuevo_dato == false) {
    rc = Serial.read();
    if (rc != endMarker) {
      line[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      line[ndx] = 0; // terminate the string
      line[ndx+1] = 0; // terminate the string
      ndx = 0;
      nuevo_dato = true;
    }
  }
  return nuevo_dato;
}

uint8_t gc_execute_line(char *line){
  char letter;
  float value;
  int int_value = 0;
  uint8_t char_counter = 0;
  while (line[char_counter] != 0) { // Loop until no more g-code words in line.
    letter = line[char_counter];
    if((letter < 'A') || (letter > 'Z')) { FAIL(STATUS_EXPECTED_COMMAND_LETTER); } // [Expected word letter]
    char_counter++;
    if (!read_float(line, &char_counter, &value)) { FAIL(STATUS_BAD_NUMBER_FORMAT); } // [Expected word value]
    int_value = trunc(value);
    switch(letter) {
      case 'P': Pan_Steps_T = int_value; break;
      case 'S': Slide_Steps_T = int_value; break;
    }
  }
  return(STATUS_OK);
}

uint8_t read_float(char *line, uint8_t *char_counter, float *float_ptr)                  
{
  char *ptr = line + *char_counter;
  unsigned char c;
  // Grab first character and increment pointer. No spaces assumed in line.
  c = *ptr++;
  // Capture initial positive/minus character
  bool isnegative = false;
  if (c == '-') {
    isnegative = true;
    c = *ptr++;
  } else if (c == '+') {
    c = *ptr++;
  }
  // Extract number into fast integer. Track decimal in terms of exponent value.
  uint32_t intval = 0;
  int8_t exp = 0;
  uint8_t ndigit = 0;
  bool isdecimal = false;
  while(1) {
    c -= '0';
    if (c <= 9) {
      ndigit++;
      if (ndigit <= MAX_INT_DIGITS) {
        if (isdecimal) { exp--; }
        intval = (((intval << 2) + intval) << 1) + c; // intval*10 + c
      } else {
        if (!(isdecimal)) { exp++; }  // Drop overflow digits
      }
    } else if (c == (('.'-'0') & 0xff)  &&  !(isdecimal)) {
      isdecimal = true;
    } else {
      break;
    }
    c = *ptr++;
  }
  // Return if no digits have been read.
  if (!ndigit) { return(false); };
  // Convert integer into floating point.
  float fval;
  fval = (float)intval;
  // Apply decimal. Should perform no more than two floating point multiplications for the
  // expected range of E0 to E-4.
  if (fval != 0) {
    while (exp <= -2) {
      fval *= 0.01; 
      exp += 2;
    }
    if (exp < 0) { 
      fval *= 0.1; 
    } else if (exp > 0) {
      do {
        fval *= 10.0;
      } while (--exp > 0);
    } 
  }
  // Assign floating point value with correct sign.    
  if (isnegative) {
    *float_ptr = -fval;
  } else {
    *float_ptr = fval;
  }
  *char_counter = ptr - line;// - 1; // Set char_counter to next statement
  return(true);
}

void stepperMoveSlide(int dir){
  if(dir>0){
     digitalWrite(Slide_Dir, LOW);
  }else{
     digitalWrite(Slide_Dir, HIGH);
  }
  digitalWrite(Slide_Step, HIGH);
  digitalWrite(Slide_Step, LOW);
}

void stepperMovePan(int dir){
  if(dir>0){
     digitalWrite(Pan_Dir, LOW);
  }else{
     digitalWrite(Pan_Dir, HIGH);
  }
  digitalWrite(Pan_Step, HIGH);
  digitalWrite(Pan_Step, LOW);
}


void distToSteps(){
  Slide_Steps = Slide_Dist*100;
}

static void initISR()
{   
  //ISRCount = 0;  // clear the value of the ISR counter;
  //cli();//stop interrupts
  TIMSK2 = 0;  // disable interrupts 
  /* setup for timer 2 */
  //set timer2 interrupt at 8kHz
  TCCR2A = 0;// set entire TCCR2A register to 0
  TCCR2B = 0;// same for TCCR2B
  TCNT2  = 0;//initialize counter value to 0
  // set compare match register for 8khz increments
  OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR2A |= (1 << WGM21);
  // Set CS21 bit for 8 prescaler
  TCCR2B |= (1 << CS21);   
  // enable timer compare interrupt
  TIMSK2 |= (1 << OCIE2A);
  //sei();//allow interrupts
} 

long contador=0;
long contador1=0;
long mDelay=stepdelay_max;
long temp_delay;
int speedDiff;
int dA,dB,maxD;
float stepA,stepB,cntA=0,cntB=0;
long pasos;
int d;
bool finish_move=true;
bool automatico = false;

void calcmoves(){
  contador=0;
    pasos=0;
    mDelay=stepdelay_max;
    speedDiff = -SPEED_STEP;
    dA = Slide_Steps_T - Slide_Steps;
    dB = Pan_Steps_T - Pan_Steps;
    maxD = max(abs(dA),abs(dB));
    stepA = (float)abs(dA)/(float)maxD;
    stepB = (float)abs(dB)/(float)maxD;
}
ISR (TIMER2_COMPA_vect)
{ 
  //Serial.println("interrupt");
  //TIMSK2 = 0;
  //TCNT2 = 0;    // reset the clock counter register
  contador1 += 125;
  contador += 125;
  if ((contador >= mDelay) && ((Slide_Steps!=Slide_Steps_T)||(Pan_Steps!=Pan_Steps_T))){
   
    pasos++;
    contador=0;
    // move A
    if(Slide_Steps!=Slide_Steps_T){
      cntA+=stepA;
      if(cntA>=1){
        d = dA>0?motorAfw:motorAbk;
        Slide_Steps+=(dA>0?1:-1);
        stepperMoveSlide(d);
        cntA-=1;
      }
    }
    // move B
    if(Pan_Steps!=Pan_Steps_T){
      cntB+=stepB;
      if(cntB>=1){
        d = dB>0?motorBfw:motorBbk;
        Pan_Steps+=(dB>0?1:-1);
        stepperMovePan(d);
        cntB-=1;
      }
    }
   mDelay=constrain(mDelay+speedDiff,stepdelay_min,stepdelay_max);
   if((maxD-pasos)<((stepdelay_max-stepdelay_min)/SPEED_STEP)){
      speedDiff = SPEED_STEP;
    }
  }
  if (contador1 > 100000){
    contador1 = 0;
    JoyXValue = analogRead(JoyX1);
    JoyYValue = analogRead(JoyY2);
    if (JoyYValue > 500 && JoyYValue < 550){
      JoyYValue = 500;
    }
    if (JoyXValue > 500 && JoyXValue < 550){
      JoyXValue = 500;
    }
    JoyYValue -= 500;
    JoyXValue -= 500;
    if (JoyXValue != 0){
      JoyXValue = map(JoyXValue,-512,512,-500,500);
      Slide_Steps_T += JoyXValue;
      calcmoves();
      mDelay = constrain(1250 - abs(JoyXValue)*2,300,1500);
    }else if(!automatico){
      Slide_Steps_T = Slide_Steps;
    }
    
    if (JoyYValue != 0){
      JoyYValue = map(JoyYValue,-512,512,-100,100);
      Pan_Steps_T += JoyYValue;
      calcmoves();
      mDelay = constrain(1250 - abs(JoyYValue)*2,300,1500);
    }
    else if(!automatico){
      Pan_Steps_T = Pan_Steps;
    }
  }
}

//float EMA_a = 0.6;      //initialization of EMA alpha
//int EMA_S = 0;          //initialization of EMA S

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Slider is ready");
  stepper_init();
  pinMode(JoyP1, INPUT_PULLUP);
  pinMode(JoyP2, INPUT_PULLUP);
  pinMode(13, OUTPUT); //apagar el led
  digitalWrite(13, LOW);
  initISR();
}

bool press1 = false;
bool press2 = false;

int grabar = 0;
int reproducir = 0;
int Slide_Steps_T_ini;
int Pan_Steps_T_ini;
int Slide_Steps_T_fin;
int Pan_Steps_T_fin;

void loop() {
  //JoyXValue = (0.6*analogRead(JoyX1)) + ((1-0.6)*JoyXValue)-528; 
  //JoyYValue = (0.2*analogRead(JoyY2)) + ((1-0.2)*JoyYValue);
  //print the values with to plot or view
  //Serial.println(digitalRead(JoyP2));
  if (!digitalRead(JoyP1) && !press1){
    press1 = true;
    if (grabar == 0){
      grabar++;
      Slide_Steps_T_ini = Slide_Steps;
      Pan_Steps_T_ini = Pan_Steps;
      Serial.print("Grabar inicio Slide= ");Serial.print(Slide_Steps);Serial.print(" Pan=");Serial.println(Pan_Steps);
    }
    else if (grabar == 1){
      grabar = 0;
      Slide_Steps_T_fin = Slide_Steps;
      Pan_Steps_T_fin = Pan_Steps;
      Serial.print("Grabar Fin Slide= ");Serial.print(Slide_Steps);Serial.print(" Pan=");Serial.println(Pan_Steps);
    }
  } else if (digitalRead(JoyP1) && press1){
    press1 = false;
    delay(200);
  }

  if (!digitalRead(JoyP2) && !press2){
    press2 = true;
    if (reproducir == 0){
      digitalWrite(13, HIGH);
      reproducir++;
      Slide_Steps_T = Slide_Steps_T_ini;
      Pan_Steps_T = Pan_Steps_T_ini;
      calcmoves();
      Serial.print("Reproduce inicio Slide= ");Serial.print(Slide_Steps_T);Serial.print(" Pan=");Serial.println(Pan_Steps_T);
      automatico = true;
    }
    else if (reproducir == 1){
      reproducir = 2;
      Slide_Steps_T = Slide_Steps_T_fin;
      Pan_Steps_T = Pan_Steps_T_fin;
      calcmoves();
      Serial.print("Reproduce fin Slide= ");Serial.print(Slide_Steps_T);Serial.print(" Pan=");Serial.println(Pan_Steps_T);
      automatico = true;
    }
    else if (reproducir == 2){
      reproducir = 0;
      automatico = false;
      digitalWrite(13, LOW);
    }
  } else if (digitalRead(JoyP2) && press2){
    press2 = false;
    delay(200);
  }
  
  if (serial_read(receivedChars)) {
    Serial.print("line received: ");
    Serial.println(receivedChars);
    status_engine=gc_execute_line(receivedChars);
    if (status_engine == STATUS_OK) {
      Serial.print("Slide= ");Serial.print(Slide_Steps);Serial.print(" ->");Serial.println(Slide_Steps_T);
      Serial.print("Pan= ");Serial.print(Pan_Steps);Serial.print(" -> ");Serial.println(Pan_Steps_T);
      calcmoves();
      //doMove();
    }
    else if (status_engine == STATUS_EXPECTED_COMMAND_LETTER) {
      Serial.println("Error: EXPECTED_COMMAND_LETTER");
    }
    else if (status_engine == STATUS_BAD_NUMBER_FORMAT) {
      Serial.println("Error: BAD_NUMBER_FORMAT");
    }
  }
}
