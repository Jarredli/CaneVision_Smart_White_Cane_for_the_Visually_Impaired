/*
 * Project: CaneVision Smart Navigation Module (V3.0)
 * Jingkun Li (Jarred) | Advisor: Carlotta Berry, Ph.D. | Pioneer Academics
 * Features: Attachable module for the traditional white cane. The whole system involves multi-direction detection system and alerting system.
 * Moving Average Filter; Non-blocking millis() Architecture; Environment Adaptive Learning
 */

// Define hardware connections for sensors and actuators
const int TRIG_PIN = 9;       
const int ECHO_PIN = 10;      
const int IR_PIN = A0;        
const int LIGHT_PIN = A1;     
const int MOTOR_PIN = 3;      
const int BUZZER_PIN = 5;     
const int LED_PIN = 6;        

// Define operational thresholds and zones for spatial awareness
const int WARNING_ZONE = 150; 
const int DANGER_ZONE = 50;   
const int DARK_THRES = 300;   

// Variables for IR sensor dynamic baseline and adaptive learning
int irBaseline = 0;           
int dropThreshold = 0;        
const int CALIBRATION_SAMPLES = 50; 
const int DROP_OFFSET = 150;  

// Variables for ultrasonic moving average filter
const int NUM_READINGS = 5;   
int readings[NUM_READINGS];   
int readIndex = 0;            
int total = 0;                
int averageDistance = 0;      

// Variables for non-blocking timing architecture
unsigned long previousMillis = 0;  
const long loopInterval = 50;      

// Variables for dynamic auditory feedback (Non-blocking buzzer)
unsigned long previousBuzzerMillis = 0; 
int buzzerState = HIGH; 

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  // Initialize outputs safely (HIGH means OFF for this active buzzer)
  analogWrite(MOTOR_PIN, 0);          
  digitalWrite(BUZZER_PIN, HIGH);     
  digitalWrite(LED_PIN, LOW);         

  // --- Ultrasonic Sensor Initialization Fix (Prevent Cold Start Bug) ---
  // Get an initial real distance reading to populate the filter array
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long initDuration = pulseIn(ECHO_PIN, HIGH);
  int initDistance = initDuration * 0.034 / 2;

  // Fill the array with the real distance to avoid false low averages
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = initDistance;
    total += initDistance;
  }
  
  Serial.println("Calibrating IR Baseline... Keep the cane steady on flat ground.");
  digitalWrite(LED_PIN, HIGH); 
  
  long tempSum = 0;
  for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
    tempSum += analogRead(IR_PIN);
    delay(50); 
  }
  
  irBaseline = tempSum / CALIBRATION_SAMPLES; 
  dropThreshold = irBaseline + DROP_OFFSET;
  
  digitalWrite(LED_PIN, LOW); 
  Serial.print("Calibration Complete! Drop Threshold set to: ");
  Serial.println(dropThreshold);
  delay(1000); 
}

void loop() {
  unsigned long currentMillis = millis();

  // Execute main logic every 50ms
  if (currentMillis - previousMillis >= loopInterval) {
    previousMillis = currentMillis; 

    // Ambient light evaluation
    int lightLevel = analogRead(LIGHT_PIN);
    digitalWrite(LED_PIN, (lightLevel > DARK_THRES) ? HIGH : LOW);

    // Ultrasonic ranging via pulse generation
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    int currentDistance = duration * 0.034 / 2;

    // Moving Average Filter execution
    total = total - readings[readIndex];       
    readings[readIndex] = currentDistance;     
    total = total + readings[readIndex];       
    readIndex = (readIndex + 1) % NUM_READINGS;                 
    averageDistance = total / NUM_READINGS; 

    // IR surface adaptation and hazard detection
    int irValue = analogRead(IR_PIN);
    bool dropHazard = (irValue > dropThreshold); 
    if (!dropHazard) {
      irBaseline = (irBaseline * 99 + irValue) / 100; 
      dropThreshold = irBaseline + DROP_OFFSET; 
    }

    // Proportional Feedback Logic (Motor Control)
    if (dropHazard || (averageDistance > 0 && averageDistance <= DANGER_ZONE)) {
      analogWrite(MOTOR_PIN, 255);      
    } else if (averageDistance > DANGER_ZONE && averageDistance <= WARNING_ZONE) {
      int dynamicVibration = map(averageDistance, WARNING_ZONE, DANGER_ZONE, 80, 255);
      analogWrite(MOTOR_PIN, dynamicVibration); 
    } else {
      analogWrite(MOTOR_PIN, 0);        
    }

    // Serial Debugging Output
    Serial.print("Dist: "); Serial.print(averageDistance); Serial.print("cm | ");
    Serial.print("IR: "); Serial.print(irValue); Serial.print(" | ");
    Serial.print("PWM: "); 
    if (dropHazard || (averageDistance > 0 && averageDistance <= DANGER_ZONE)) {
      Serial.println(255);
    } else if (averageDistance > DANGER_ZONE && averageDistance <= WARNING_ZONE) {
      Serial.println(map(averageDistance, WARNING_ZONE, DANGER_ZONE, 80, 255));
    } else {
      Serial.println(0);
    }
  }

  // Independent Non-blocking Auditory Feedback Architecture
  // This runs continuously outside the 50ms loop to ensure precise acoustic timing
  int irValueAsync = analogRead(IR_PIN);
  bool dropHazardAsync = (irValueAsync > dropThreshold); 

  if (dropHazardAsync || (averageDistance > 0 && averageDistance <= DANGER_ZONE)) {
    // Critical Danger: Solid continuous beep
    digitalWrite(BUZZER_PIN, LOW); 
  } else if (averageDistance > DANGER_ZONE && averageDistance <= WARNING_ZONE) {
    // Warning Zone: Dynamic beep intervals based on proximity
    // Maps distance (50-150cm) to beep interval (100ms to 800ms)
    int beepInterval = map(averageDistance, DANGER_ZONE, WARNING_ZONE, 100, 800);
    
    if (currentMillis - previousBuzzerMillis >= beepInterval) {
      previousBuzzerMillis = currentMillis;
      // Toggle the buzzer state to create a pulsing beep
      buzzerState = (buzzerState == HIGH) ? LOW : HIGH; 
      digitalWrite(BUZZER_PIN, buzzerState);
    }
  } else {
    // Safe Zone: Keep buzzer completely OFF
    digitalWrite(BUZZER_PIN, HIGH); 
    buzzerState = HIGH; 
  }
}
