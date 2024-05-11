#include <MeMCore.h>

// Define time delay before the next RGB colour turns ON to allow LDR to stabilize
#define RGBWait 200 // in milliseconds 

// Define time delay before taking another LDR reading
#define LDRWait 10 // in milliseconds 

// time duration for turning
#define TURNING_TIME_MS 400 // 330 milliseconds

// time duration for moving one grid
#define ONE_GRID_TIME_MS 850 // 2000milliseconds

#define DETECTOR A0 // IR detector pin at A0
#define LDR A1   // LDR sensor pin at A1

// Define ultrasonic sensor
#define TIMEOUT 2000 // max time to wait (in ms); 
#define SPEED_OF_SOUND 340 // speed of sound in m/s
#define ULTRASONIC 12 // 12 for port 1

// Define line sensor
MeLineFollower lineFinder(PORT_2); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
MeBuzzer buzzer; // play sound when robot reaches end

#define S1 D9
#define S2 D10

int status = 0; // global status; 0 = do nothing, 1 = mBot runs

void on_LED (int c);
int getAvgReading(int times);

uint8_t moveSpeed = 200;

float colourArray[] = {0,0,0};
float whiteArray[] = {520, 264, 258};
float blackArray[] = {397, 549, 538};
float greyDiff[] = {123, 290, 327};
float redArray[] = {468, 81, 66}; 
float orangeArray[] = {495, 133, 68}; 
float blueArray[] = {290, 209, 238};
float greenArray[] = {230,168,120}; 
float purpleArray[] = {298, 133, 185}; 

char colourStr[3][5] = {"R = ", "G = ", "B = "};
float ambient;

// motor movements - function prototypes
void go_straight();
void turn_left();
void turn_right();
void u_turn();
void two_consec_turns_right();
void two_consec_turns_left();
void stop_moving();
void waypoint_challenge(float *colourArray);
void play_victory_sound();

void setBalance(){
    // set white balance
    Serial.println("Put White Sample For Calibration ...");
    
    //delay for five seconds for getting sample ready
    delay(5000);  

    // scan the white sample.
    // go through one colour at a time, set the maximum reading for each colour (red, green and blue) to the white array
    for (int i = 0; i <= 2; i++) {
        Serial.print(colourStr[i]);
        on_LED(i);
        delay(RGBWait);
        
        // scan 5 times and return the average
        whiteArray[i] = getAvgReading(5); 
        delay(RGBWait);

        // show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
        Serial.println(int(whiteArray[i])); 
    }

    // done scanning white, time for the black sample.
    // set black balance
    Serial.println("Put Black Sample For Calibration ...");
    //delay for five seconds for getting sample ready 
    delay(5000);  

    // go through one colour at a time, set the minimum reading for red, green and blue to the black array
    for (int i = 0;i<=2;i++) {
        Serial.print(colourStr[i]);
        on_LED(i);
        delay(RGBWait);
        blackArray[i] = getAvgReading(5);
        delay(RGBWait);

        // show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
        Serial.println(int(blackArray[i])); 

        // the difference between the maximum and the minimum gives the range
        greyDiff[i] = whiteArray[i] - blackArray[i];
    }

    // delay another 5 seconds for getting ready colour objects
    Serial.println("Colour Sensor Is Ready.");
    delay(5000);
}


void setup(){
    //setup the output pins
    pinMode(DETECTOR, INPUT);
    pinMode(LDR, INPUT);
    pinMode(A2, OUTPUT);
    pinMode(A3, OUTPUT);
}

void loop(){
    int count = 0;
    ambient = ((float)analogRead(DETECTOR)/1023.0) * 5.0;
    delay(10);

    digitalWrite(A2, LOW);
    digitalWrite(A3, LOW);
    delay(10);
    
    float value = ((float)analogRead(DETECTOR)/1023.0) * 5.0;
    delay(10);
    
    digitalWrite(A2, HIGH);
    digitalWrite(A3, HIGH);
    delay(2);

    if (value <= ambient - 1 ) { // if IR sensor is too near to wall
        leftMotor.run(0);
        rightMotor.run(150); // turn left for 0.1s 
        delay(50);
        leftMotor.run(-moveSpeed); // resume normal speed
        rightMotor.run(moveSpeed);
    } 

    // set as output
    pinMode(ULTRASONIC, OUTPUT); 
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2); 
    digitalWrite(ULTRASONIC, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC, LOW);
    delayMicroseconds(2);

    // set as input
    pinMode(ULTRASONIC, INPUT); 

    // time taken in microseconds for pulse to reach wall and return
    long duration = pulseIn(ULTRASONIC, HIGH, TIMEOUT); 
    if (duration > 0) {
        // distance in cm the sensor is from the wall
        float cms = SPEED_OF_SOUND * 100.0 * duration / 1000000 / 2; 

        if (cms < 7) {
            leftMotor.run(-150);
            rightMotor.run(0);
            delay(50);
            leftMotor.run(-moveSpeed);
            rightMotor.run(moveSpeed);
        }
     }

    // check if line follower detects a black line
    int sensorState = lineFinder.readSensors(); // read the line sensor's state
    
    if (sensorState == S1_IN_S2_IN) { // robot is on the black line
        leftMotor.stop(); // Left wheel stops
        rightMotor.stop(); // Right wheel stops
   
        // check colour of paper
        for (int c = 0; c <= 2; c++) { 
        on_LED(c);
        delay(RGBWait);
        colourArray[c] = getAvgReading(5);
        colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
        delay(RGBWait);
    } 

    check_colour(colourArray); // check what to do at the waypoint
    }
  
    // mbot move straight 
    leftMotor.run(-moveSpeed);
    rightMotor.run(moveSpeed);
}

void on_LED (int c){
    if (c == 0){ 
        // turning on red LED
        digitalWrite(A2, HIGH);
        digitalWrite(A3, HIGH);
    } else if (c == 1) { 
        // turning on green LED
        digitalWrite(A2, HIGH);
        digitalWrite(A3, LOW);
    } else { 
        // turning on blue LED
        digitalWrite(A2, LOW);
        digitalWrite(A3, HIGH);
    }
}

void check_colour(float *colourArray){
    if (colourArray[0] >= 220 && colourArray[1] >= 160 && colourArray[2] >= 200) { 
        // here, the G and B value matches white

        // play completion music
        play_victory_sound(); 
        // delay for approximately 16.5 minutes to prevent mBot from moving after finishing the challenge
        delay(1000000); 
    } else if (colourArray[0] >= 350) { 
        // here, the R value matches red and orange
        if (colourArray[1] >= 0) { 
            //here, the G value matches orange
            u_turn(); 
        } else {
            turn_left();
        }
    } else if (colourArray[0] >= 150) { 
        // here, the R value matches blue and purple
        if (colourArray[1] >= 50) { 
            // here, the G value matches blue
            two_consec_turns_right();
        } else {
            // should be purple detected if this branch is reached
            two_consec_turns_left(); 
        }
    } else {
        // only green remaining
        turn_right(); 
    }
}

void play_victory_sound() {
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(988, 393);
    buzzer.tone(988, 393);
    buzzer.tone(880, 789);
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(880, 393);
    buzzer.tone(880, 393);
    buzzer.tone(784, 393);
    buzzer.tone(740, 131);
    buzzer.tone(659, 262);
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(784, 526);
    buzzer.tone(880, 262);
    buzzer.tone(740, 393);
    buzzer.tone(659, 131);
    buzzer.tone(587, 262);
    buzzer.tone(587, 262);
    buzzer.tone(587, 262);

    buzzer.tone(880, 526);
    buzzer.tone(784, 1052);
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(988, 393);
    buzzer.tone(988, 393);
    buzzer.tone(880, 789);
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(1175, 526);
    buzzer.tone(740, 262);
    buzzer.tone(783, 393);
    buzzer.tone(740, 131);
    buzzer.tone(659, 262);
    buzzer.tone(587, 131);
    buzzer.tone(659, 131);
    buzzer.tone(783, 131);
    buzzer.tone(659, 131);

    buzzer.tone(783, 526);
    buzzer.tone(880, 263);
    buzzer.tone(740, 393);
    buzzer.tone(659, 131);
    buzzer.tone(587, 526);
    buzzer.tone(587, 263);

    buzzer.tone(880, 526);
    buzzer.tone(783, 1052);

    buzzer.noTone();
}

int getAvgReading(int times){      
    // find the average reading for the requested number of times of scanning LDR
    int reading;
    int total = 0;

    // take the reading as many times as requested and add them up
    for (int i = 0; i < times; i++) {
        reading = analogRead(LDR);
        total = reading + total;
        delay(LDRWait);
    }

    // calculate the average and return it
    return total/times;
}

void go_straight(){
    leftMotor.run(-moveSpeed);
    rightMotor.run(moveSpeed);
}

void stop_moving(){
    leftMotor.run(0);
    rightMotor.run(0);
}

void turn_left(){
    leftMotor.run(moveSpeed);
    rightMotor.run(moveSpeed);

    // Keep turning left for this time duration
    delay(TURNING_TIME_MS); 

    stop_moving();
    delay(100);
}

void turn_right(){
    leftMotor.run(-moveSpeed);
    rightMotor.run(-moveSpeed);

    // Keep turning right for this time duration
    delay(TURNING_TIME_MS); 
    
    stop_moving();
    delay(100);
}

void u_turn(){
    turn_right();
    delay(1000);

    turn_right();
    delay(1000);
    
    go_straight();
}

void two_consec_turns_right(){
    turn_right();

    go_straight();
    delay(ONE_GRID_TIME_MS);
    
    turn_right();
    
    stop_moving();
}

void two_consec_turns_left(){
    leftMotor.run(moveSpeed);
    rightMotor.run(moveSpeed);

    // Keep turning left for this time duration
    delay(400); 

    stop_moving();
    delay(100);
    
    go_straight();
    delay(ONE_GRID_TIME_MS);
    
    leftMotor.run(moveSpeed);
    rightMotor.run(moveSpeed);
    
    // Keep turning left for this time duration
    delay(420); 
    
    stop_moving();
    go_straight();
}


 


