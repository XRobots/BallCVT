#include <Servo.h>

Servo servo1;   // throttle
Servo servo2;   // clutched CVT
Servo servo3;   // clutched CVT

// Radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN
const byte addresses[][6] = {"00001", "00002"};

//**************remote control****************
struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
    int16_t menuDown;  
    int16_t Select;    
    int16_t menuUp;  
    int16_t toggleBottom;  
    int16_t toggleTop; 
    int16_t toggle1;
    int16_t toggle2;
    int16_t mode;  
    int16_t RLR;
    int16_t RFB;
    int16_t RT;
    int16_t LLR;
    int16_t LFB;
    int16_t LT;
};

RECEIVE_DATA_STRUCTURE mydata_remote;

int RLR = 0;
int RFB = 0;
int RT = 0;
int LLR = 0;
int LFB = 0;
int LT = 0;

unsigned long currentMillis;
long previousMillis = 0;    // set up timers
long interval = 10;        // time constant for timer

int throttle;           // accumulator for the throttle
int throttleOffset;     // throttle with offset for mid position of actual throttle

void setup() {

   
    // initialize serial communication
    Serial.begin(115200);
    
    radio.begin();
    radio.openWritingPipe(addresses[0]); // 00002
    radio.openReadingPipe(1, addresses[1]); // 00001
    radio.setPALevel(RF24_PA_MIN);

    radio.startListening();

    servo2.attach(22);    // clutch CVT
    servo3.attach(24);    // clutch CVT
    
    servo1.attach(26);    // throttle

    servo1.write(80);
    servo2.write(90);
    servo3.write(90);

    delay(1000);

    
}   // end of setup

// ********************* MAIN LOOP *******************************

void loop() {  
      
        currentMillis = millis();
        if (currentMillis - previousMillis >= 10) {  // start timed event
          
            previousMillis = currentMillis;


            // check for radio data
            if (radio.available()) {
                    radio.read(&mydata_remote, sizeof(RECEIVE_DATA_STRUCTURE));   
            }  

            // threshold remote data
            // some are reversed based on stick wiring in remote
            RFB = (thresholdStick(mydata_remote.RFB))*-1;   
            RLR = thresholdStick(mydata_remote.RLR);
            RT = thresholdStick(mydata_remote.RT);   
            LFB = (thresholdStick(mydata_remote.LFB))*-1;   
            LLR = thresholdStick(mydata_remote.LLR);
            LT = thresholdStick(mydata_remote.LT);            

            RT = map(RT,0,512,0,2);

            // make the throttle respond to accoumulated value of each stick press

            throttle = throttle + RT;
            throttleOffset = throttle + 90;
            throttleOffset = constrain(throttleOffset,10,170);

            Serial.println(throttleOffset);
            servo1.write(throttleOffset);

            RFB = map(RFB,-512,512,10,170);
            LFB = map(LFB,-512,512,10,170);

            // write to the two servos operating the clutched CVTs

            servo3.write(RFB);
            servo2.write(LFB);             
            

      
        }     // end of timed loop         
   
}       // end  of main loop
