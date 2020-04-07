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

#define Slide_Step 2  //Digital2
#define Slide_Dir 3    //Digital3
#define Pan_Step 4   //Digital5
#define Pan_Dir 5     //Digital6
#define SPEED_STEP 1


void stepper_init()
{
  // Configure step and direction interface pins
  //Slider_Step = Digital2
  //Slider_Dir = Digital3
  //Pan_Step = Digital5
  //Pan_Dir = Digital6
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


long stepAuxDelay=0;
int stepdelay_min=600;
int stepdelay_max=2000;
int8_t motorAfw=1;
int8_t motorAbk=-1;
int8_t motorBfw=1;
int8_t motorBbk=-1;
long last_time;

void doMove()
{
  long mDelay=stepdelay_max;
  long temp_delay;
  int speedDiff = -SPEED_STEP;
  int dA,dB,maxD;
  float stepA,stepB,cntA=0,cntB=0;
  int d;
  dA = Slide_Steps_T - Slide_Steps;
  dB = Pan_Steps_T - Pan_Steps;
  maxD = max(abs(dA),abs(dB));
  stepA = (float)abs(dA)/(float)maxD;
  stepB = (float)abs(dB)/(float)maxD;
  for(int i=0;(Slide_Steps!=Slide_Steps_T)||(Pan_Steps!=Pan_Steps_T);i++){
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
        Serial.println("P_Step");
      }
    }
    mDelay=constrain(mDelay+speedDiff,stepdelay_min,stepdelay_max);
    temp_delay = mDelay + stepAuxDelay;
    if(millis() - last_time > 400)
    {
      last_time = millis();
      //if(true == process_serial()) //falta definir
      //{
      //  return;
      //}
    }

    if(temp_delay > stepdelay_max)
    {
      temp_delay = stepAuxDelay;
      delay(temp_delay/1000);
      delayMicroseconds(temp_delay%1000);
    }
    else
    {
      delayMicroseconds(temp_delay);
    }
    if((maxD-i)<((stepdelay_max-stepdelay_min)/SPEED_STEP)){
      speedDiff=SPEED_STEP;
    }
  }
  //Serial.printf("finally %d A:%d B;%d\n",maxD,posA,posB);
  Slide_Steps = Slide_Steps_T;
  Pan_Steps = Pan_Steps_T;
}

void distToSteps(){
  Slide_Steps = Slide_Dist*100;
}


void setup() {
  Serial.begin(BAUD_RATE);
  Serial.println("Slider is ready");
  stepper_init();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (serial_read(receivedChars)) {
    Serial.print("line received: ");
    Serial.println(receivedChars);
    status_engine=gc_execute_line(receivedChars);
    if (status_engine == STATUS_OK) {
      Serial.print("Slide= ");Serial.print(Slide_Steps);Serial.print(" ->");Serial.println(Slide_Steps_T);
      Serial.print("Pan= ");Serial.print(Pan_Steps);Serial.print(" -> ");Serial.println(Pan_Steps_T);
      doMove();
    }
    else if (status_engine == STATUS_EXPECTED_COMMAND_LETTER) {
      Serial.println("Error: EXPECTED_COMMAND_LETTER");
    }
    else if (status_engine == STATUS_BAD_NUMBER_FORMAT) {
      Serial.println("Error: BAD_NUMBER_FORMAT");
    }
  }
}
