#include <NewPing.h>
2#include <AFMotor.h>
3#include <Servo.h> 
4 
5// Ultranic Pin Configuration
6   
7#define TRIG_PIN A0
8#define ECHO_PIN A1
9 
10#define MAX_DISTANCE 400
11#define   MAX_SPEED 255
12#define MAX_SPEED_OFFSET -8
13 
14#define COLL_DIST 20
15#define   TURN_DIST COLL_DIST+10
16#define ACT_TIME 250
17 int calibrationTime = 30;        
18
19//the   time when the sensor outputs a low impulse
20long unsigned int lowIn;         
21
22//the   amount of milliseconds the sensor has to be low 
23//before we assume all motion   has stopped
24long unsigned int pause = 5000;  
25
26boolean lockLow = true;
27boolean   takeLowTime;  
28
29int pirPin = A3;    //the digital pin connected to the PIR   sensor's output
30int ledPin = A2;
31NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE);   
32 
33AF_DCMotor motorR(1, MOTOR12_1KHZ); // Set motor #1, 1kHz PWM
34AF_DCMotor   motorL(4, MOTOR12_1KHZ); // Set motor #2, 1kHz PWM
35 
36Servo myservo;  // Set   servo object to control a servo 
37String motorSet = "";
38 
39int curDist   = 0, pos, speedSet = 0;
40//int pos;
41//int speedSet = 0;
42 
43void setup()   {
44  Serial.begin(9600);
45  pinMode(pirPin, INPUT);
46  pinMode(ledPin, OUTPUT);
47   digitalWrite(pirPin, LOW);
48
49  //give the sensor some time to calibrate
50   Serial.print("calibrating sensor ");
51    for(int i = 0; i < calibrationTime;   i++){
52      Serial.print(".");
53      delay(1000);
54      }
55    Serial.println("   done");
56    Serial.println("SENSOR ACTIVE");
57    delay(50);
58  myservo.attach(9);   // Set to attach the servo on pin 9 
59  myservo.write(90);  // Write 90 to face   servo forward
60  delay(2000);
61 
62  motorSet = "FORWARD";
63  moveForward();
64   
65}
66 
67void loop() {
68  
69  checkPath();
70  if(digitalRead(pirPin)   == HIGH){
71       digitalWrite(ledPin, HIGH);   //the led visualizes the sensors   output pin state
72       if(lockLow){  
73         //makes sure we wait for a   transition to LOW before any further output is made:
74         lockLow = false;             
75         Serial.println("---");
76         Serial.print("motion   detected at ");
77         Serial.print(millis()/1000);
78         Serial.println("   sec"); 
79         delay(50);
80         }         
81         takeLowTime =   true;
82       }
83
84     if(digitalRead(pirPin) == LOW){       
85       digitalWrite(ledPin,   LOW);  //the led visualizes the sensors output pin state
86
87       if(takeLowTime){
88         lowIn = millis();          //save the time of the transition from high to   LOW
89        takeLowTime = false;       //make sure this is only done at the start   of a LOW phase
90        }
91       //if the sensor is low for more than the given   pause, 
92       //we assume that no more motion is going to happen
93       if(!lockLow   && millis() - lowIn > pause){  
94           //makes sure this block of code is   only executed again after 
95           //a new motion sequence has been detected
96            lockLow = true;                        
97           Serial.print("motion   ended at ");      //output
98           Serial.print((millis() - pause)/1000);
99            Serial.println(" sec");
100           delay(50);
101           }
102        }
103  
104}
105 
106void checkPath() {
107  
108  int curLeft = 0; int curRight   = 0; int curFront = 0;
109  curDist = 0;
110  
111  checkForward();
112  myservo.write(135);
113   delay(100);
114  for (pos = 135; pos >= 45; pos -= 45) {
115    myservo.write(pos);
116     delay(170);
117    curDist = readPing();
118    
119    if (curDist < COLL_DIST)   { checkCourse(); break; }
120    if (curDist < TURN_DIST) { changePath(); } 
121   
122  }    
123}  
124 
125int readPing() {
126  int cm = 0;
127  while (cm <   2) {int uS = sonar.ping(); cm = uS/US_ROUNDTRIP_CM;}
128  return cm;
129}
130 
131void   checkForward() { 
132  if (motorSet=="FORWARD") { motorR.run(FORWARD); motorL.run(FORWARD);   } 
133}    
134void changePath() {
135 
136  if (pos < 90) { veerLeft(); } 
137   if (pos > 90) { veerRight(); }
138  
139}
140 
141void veerRight() {
142  motorR.run(BACKWARD);   motorL.run(FORWARD); 
143  delay(ACT_TIME); 
144  motorR.run(FORWARD);   motorL.run(FORWARD);
145   motorSet = "FORWARD";
146}
147 
148void veerLeft() {
149  motorL.run(BACKWARD);   motorR.run(FORWARD); 
150  delay(ACT_TIME); 
151  motorL.run(FORWARD);   motorR.run(FORWARD);
152   motorSet = "FORWARD";
153}
154 
155void checkCourse() {
156  moveBackward();
157   delay(ACT_TIME);
158  moveStop();
159  setCourse();
160}
161 
162void setCourse()   {
163  if (pos < 90) { turnRight(); } 
164  if (pos > 90) { turnLeft(); }
165}
166   
167void moveBackward() {
168  motorSet = "BACKWARD";
169  
170  motorR.run(BACKWARD);   // Turn right motor backward    
171  motorL.run(BACKWARD);  // Turn left motor   backward
172  
173  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2)
174  {
175     motorL.setSpeed(speedSet);
176    motorR.setSpeed(speedSet+MAX_SPEED_OFFSET);
177     delay(5);
178  }
179}  
180 
181void moveForward() {
182  motorSet = "FORWARD";
183   checkForward();
184  for (speedSet = 0; speedSet < MAX_SPEED; speedSet +=2)  {
185     motorL.setSpeed(speedSet);
186    motorR.setSpeed(speedSet+MAX_SPEED_OFFSET);
187     delay(4);
188  }
189}
190 
191void moveStop() { motorR.run(RELEASE); motorL.run(RELEASE);   }
192 
193void turnRight() {
194  motorSet = "RIGHT";
195  motorR.run(FORWARD);       // Turn right motor forward
196  motorL.run(BACKWARD);     // Turn left motor   backward
197  delay(ACT_TIME);
198  motorSet = "FORWARD";
199  checkForward();
200}   
201 
202void turnLeft() {
203  motorSet = "LEFT";
204  motorR.run(BACKWARD);      // Turn right motor backward
205  motorL.run(FORWARD);      // Turn left motor   forward
206  delay(ACT_TIME);
207  motorSet = "FORWARD";
208  checkForward();
209}   
