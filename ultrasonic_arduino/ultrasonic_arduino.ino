// Include the Wire library for I2C
#include <Wire.h>  // look on the web for an improved wire library which improves reliability by performing re-init on lockups 

/* Pinout definitions -> adjust settings here
 ********************************************/
#define A_ECHO A1
#define A_TRIGGER A0
#define B_ECHO A3
#define B_TRIGGER A2
#define C_ECHO 2
#define C_TRIGGER 3
#define D_ECHO 4
#define D_TRIGGER 5

/* Program defitions
 *******************/
#define NUM_SENSORS 4               // # of sensors
#define PAUSE_MEAS 60               // measurement cycle [ms]
#define MIN_DEVIATION 0.02          // * 100%: values less than this are ignored
#define MAX_DEVIATION 0.05          // * 100%: values greater than this are filtered 
#define FILTER_ARRAY_SHIFT 2        // edit here to adjust FILTER_ARRAY_SIZE
#define FILTER_ARRAY_SIZE (1<<FILTER_ARRAY_SHIFT) // make size to the power of 2 for better performance

#define CM_PER_SECOND (0.034 / 2)  // v=s/t <-> t=s/v
#define T_ECHO_MIN (2 / CM_PER_SECOND)
#define T_ECHO_MAX (380 / CM_PER_SECOND)
#define T_ECHO_OUT_OF_RANGE (400 / CM_PER_SECOND)
unsigned long measure_distance(uint8_t triggerPin, uint8_t echoPin){
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  
  // measure duration of pulse from ECHO pin
  // pulseIn: The length of the pulse (in microseconds) or 0 if no pulse started before the timeout.
  unsigned long duration = pulseIn(echoPin, HIGH, T_ECHO_MAX);
  if(duration < T_ECHO_MIN) // includes duration == 0
    return T_ECHO_OUT_OF_RANGE;
  return duration;
}

float filter_median(unsigned short _i, unsigned long t){
  /*
	*  _i	index to array, which sensor
	*  t	time measured by sensor 
  */
  static unsigned long filter_array[NUM_SENSORS << FILTER_ARRAY_SHIFT] = { 0 };
  static unsigned short i[NUM_SENSORS] = { 0 };             // index for each sensor
  static unsigned long filter_array[num_sensors << filter_array_shift] = { 0 };
  static unsigned short i[num_sensors] = { 0 };             // index for each sensor
  
  unsigned long *arr = filter_array + (_i << FILTER_ARRAY_SHIFT);
  unsigned short *index = i + _i;
  unsigned long old_t = arr[*index];
  float d = abs( ((float)t) / ((float)old_t) - 1);
  *index = (*index + 1) % FILTER_ARRAY_SIZE;
  if(d < MIN_DEVIATION){ //ignore
    arr[*index] = old_t;
    t = old_t;
  }
  else if(d > MAX_DEVIATION){ //return average of last values
    arr[*index] = t;
    unsigned long mean = 0;
    for(int i = 0; i < FILTER_ARRAY_SIZE; i++)
      mean += arr[i];
    t = mean >> FILTER_ARRAY_SHIFT;
  } 
  else { // return this value
	arr[*index] = t;
  }
  return (float(t)) * CM_PER_SECOND;
}

/* alternative method, not that good
float filter_moving_average (unsigned short _i, unsigned long t){
  static unsigned long filter_array[NUM_SENSORS << FILTER_ARRAY_SHIFT] = { 0 };
  static unsigned short i[NUM_SENSORS] = { 0 };             // index for each sensor
  static unsigned long averages[NUM_SENSORS] = { 0 };       // average for each sensor

  unsigned long *arr = filter_array + (_i << FILTER_ARRAY_SHIFT);
  unsigned short *index = i + _i;
  unsigned long *mean = averages + _i;
  
  *mean = *mean - arr[*index] + t; // replace oldest entry by newest
  arr[*index] = t;
  *index = (*index + 1) % FILTER_ARRAY_SIZE;

  return ((float)((*mean) >> FILTER_ARRAY_SHIFT)) * CM_PER_SECOND;
}*/

/* alternative method, not that good
float filter_median2(unsigned short _i, unsigned long t){
  static unsigned long filter_array[NUM_SENSORS << FILTER_ARRAY_SHIFT] = { 0 };
  static unsigned short i[NUM_SENSORS] = { 0 };             // index for each sensor
  
  unsigned long *arr = filter_array + (_i << FILTER_ARRAY_SHIFT);
  unsigned short *index = i + _i;
  unsigned long old_t = arr[*index];
  float d = abs( ((float)t) / ((float)old_t) - 1);
  *index = (*index + 1) % FILTER_ARRAY_SIZE;
  if(d < MIN_DEVIATION){ //ignore
    arr[*index] = old_t;
    t = old_t;
  }
  else if(d > MAX_DEVIATION){ //return average of last values
    unsigned long mean = 0;
    for(int i = 0; i < FILTER_ARRAY_SIZE; i++)
      mean += arr[i];
    mean = mean - old_t + t;
    t = mean >> FILTER_ARRAY_SHIFT;
    arr[*index] = t;
  } 
  else { // return this value
	arr[*index] = t;
  }
  return (float(t)) * CM_PER_SECOND;
}*/

/* I2C Definitions
 *****************/
 enum { 
  I2C_MSG_ARGS_MAX = 32,
  I2C_RESP_LEN_MAX = 32
};

#define I2C_ADDR                 8             // This is slave #8
#define TWI_FREQ_SETTING         400000L       // 400KHz for I2C
#define CPU_FREQ                 F_CPU //16000000L     // 16MHz

int argsCnt = 0;                        // how many arguments were passed with given command

byte i2cArgs[I2C_MSG_ARGS_MAX];         // array to store args received from master
int i2cArgsLen = 0;                     // how many args passed by master to given command

uint8_t i2cResponseBuffer[I2C_RESP_LEN_MAX];  // array to store response
uint8_t i2cResponseRaw[I2C_RESP_LEN_MAX];  // array to store raw binary response
uint8_t *i2cResponse = NULL;
short i2cResponseLen = 0;                 // response length

void setup() {
	// Serial.begin(9600);
	// Serial.println("Good to go!");
	// Setup pin 13 as output and turn LED off
	// pinMode(LED_BUILTIN, OUTPUT);
	// digitalWrite(LED_BUILTINdPin, LOW);
	
	// setup Ultasonic Sensor
	pinMode(A_TRIGGER, OUTPUT);
	digitalWrite(A_TRIGGER, LOW);
	pinMode(A_ECHO, INPUT);
	digitalWrite(A_ECHO, LOW);       // turn off pullup resistors.. just in case
	pinMode(B_TRIGGER, OUTPUT);
	digitalWrite(B_TRIGGER, LOW);
	pinMode(B_ECHO, INPUT);
	digitalWrite(B_ECHO, LOW);
	pinMode(C_TRIGGER, OUTPUT);
	digitalWrite(C_TRIGGER, LOW);
	pinMode(C_ECHO, INPUT);
	digitalWrite(C_ECHO, LOW);
	pinMode(D_TRIGGER, OUTPUT);
	digitalWrite(D_TRIGGER, LOW);
	pinMode(D_ECHO, INPUT);
	digitalWrite(D_ECHO, LOW);
	// init the array
	unsigned long a = measure_distance(A_TRIGGER, A_ECHO),
	b = measure_distance(B_TRIGGER, B_ECHO),
	c = measure_distance(C_TRIGGER, C_ECHO),
	d = measure_distance(D_TRIGGER, D_ECHO);
	for(unsigned i = 0; i < filter_array_size; i++){
		filter_median(1, a);
		filter_median(2, b);
		filter_median(3, c);
		filter_median(4, d);
	}
	// >> starting i2c
	TWBR = ((CPU_FREQ / TWI_FREQ_SETTING) - 16) / 2;
	Wire.begin(I2C_ADDR);                        // join i2c bus 
	Wire.onRequest(requestEvent);                // register event
	Wire.onReceive(receiveEvent);    
	// << starting i2c
}

void loop()
{
  unsigned long t;
  float d;
  t = measure_distance(A_TRIGGER, A_ECHO);
  d = filter_median(0, t);
  *(i2cResponseRaw) = t;
  *(float*)(i2cResponseBuffer) = d;
  delay(PAUSE_MEAS - (t >> 10));

  t = measure_distance(B_TRIGGER, B_ECHO);
  d = filter_median(1, t);
  *(i2cResponseRaw + 4) = t;
  *(float*)(i2cResponseBuffer + 4) = d;
  delay(PAUSE_MEAS - (t >> 10));

  t = measure_distance(C_TRIGGER, C_ECHO);
  d = filter_median(2, t);  
  *(i2cResponseRaw + 8) = t;
  *(float*)(i2cResponseBuffer + 8) = d;
  delay(PAUSE_MEAS - (t >> 10));

  t = measure_distance(D_TRIGGER, D_ECHO); 
  d = filter_median(3, t);
  *(i2cResponseRaw + 12) = t;
  *(float*)(i2cResponseBuffer + 12) = d;
  delay(PAUSE_MEAS - (t >> 10));
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent(){ Wire.write(i2cResponse, i2cResponseLen); }

// function that executes when master sends data (begin-end transmission)
// this function is registered as an event, see setup()
void receiveEvent(int howMany){
	//digitalWrite(13,HIGH);
	int cmdRcvd = -1;
	int argIndex = -1;
	argsCnt = 0;

	if (Wire.available()){
		cmdRcvd = Wire.read();                 // receive first byte - command assumed
		while(Wire.available()){               // receive rest of tramsmission from master assuming arguments to the command
			if (argIndex < I2C_MSG_ARGS_MAX){
				argIndex++;
				i2cArgs[argIndex] = Wire.read();
			} else {
			; // implement logging error: "too many arguments"
			}
			argsCnt = argIndex+1;  
		}
	} else {
		// implement logging error: "empty request"
		return;
	}
	//  Serial.print("Received arg + data: ");
	//  Serial.print(cmdRcvd);
	//  Serial.print(" | ");
	//  Serial.println(i2cArgs[1]<<8 + i2cArgs[0]);
	switch(cmdRcvd){
		case 'a': // All data
			i2cResponseLen = 4 * sizeof(float);
			i2cResponse = i2cResponseBuffer;
			break;
		case 'r': // Raw data 
			i2cResponseLen = 4 * sizeof(long unsigned);
			i2cResponse = (uint8_t*) i2cResponseRaw;
		break;
		case 1:
		case 2:
		case 3:
		case 4:
			i2cResponseLen = sizeof(float);
			cmdRcvd = (cmdRcvd - 1) * sizeof(float);
			i2cResponse = i2cResponseBuffer + cmdRcvd;
			break;
		case 't':
			temp = *(float*)i2cArgs;
		//	Serial.print("received temp arg: ");
		//	Serial.println(*(float*)i2cArgs);
		default:
			i2cResponseLen = 0;
			break;
	}
} 
