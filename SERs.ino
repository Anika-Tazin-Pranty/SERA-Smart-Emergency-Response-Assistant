// Multi-Hazard Safety Monitoring System
// This system monitors for fire, gas leaks, earthquakes, and high temperature
// and takes appropriate safety actions based on detected hazards

#include <DHT.h>                    // Temperature and humidity sensor library
#include <Wire.h>                   // I2C communication library for LCD
#include <LiquidCrystal_I2C.h>      // LCD display library
#include <Servo.h>                  // Servo motor control library

// ========== PIN DEFINITIONS ==========
// Sensor pins
#define DHTPIN 2                    // DHT11 temperature/humidity sensor data pin
#define DHTTYPE DHT11               // Specify DHT sensor type
#define SW420_PIN 3                 // SW-420 vibration sensor pin (earthquake detection)
const int MQ2_PIN = A1;             // MQ-2 gas sensor analog pin (gas leak detection)
#define FLAME_PIN 4                 // Flame sensor digital output pin (fire detection)

// Output device pins
#define GREEN_LED 7                 // Green LED - normal operation indicator
#define RED_LED 8                   // Red LED - fire/critical alert indicator
#define YELLOW_LED 11               // Yellow LED - earthquake alert indicator
#define BLUE_LED 12                 // Blue LED - gas leak alert indicator
#define BUZZER_PIN 6                // Buzzer for audio alerts
#define SERVO_GAS_VALVE_PIN 9       // Servo motor for gas valve control
#define SERVO_DOOR_PIN 10           // Servo motor for door control
#define RELAY_FAN_PIN 5             // Relay to control ventilation fan
#define ACK_BUTTON_PIN 13           // Acknowledgment button to reset system

// ========== HARDWARE INITIALIZATION ==========
DHT dht(DHTPIN, DHTTYPE);                    // Initialize DHT sensor object
Servo gasValveServo;                         // Servo object for gas valve control
Servo doorServo;                             // Servo object for door control
LiquidCrystal_I2C lcd(0x27, 16, 2);         // LCD object (I2C address 0x27, 16x2 display)

// ========== GLOBAL VARIABLES ==========
// Sensor reading variables
int temperature, humidity;                   // Temperature (°C) and humidity (%) readings
int gasLevel = 0;                           // Gas concentration level (0-1023)
bool flameDetected, vibrationDetected;      // Boolean flags for flame and vibration detection
bool overrideActive = false;                // Flag to indicate if system is overridden by user

// Button debouncing variables (prevents false button presses)
unsigned long lastButtonPress = 0;          // Timestamp of last button press
bool lastButtonState = HIGH;                // Previous state of button (HIGH = not pressed)
bool buttonPressed = false;                 // Flag for button press detection

// Alert timing variables
unsigned long alertStartTime = 0;           // When alert mode started
bool inAlertMode = false;                   // Flag indicating if system is currently alerting

// ========== SETUP FUNCTION ==========
// Runs once when Arduino starts - initializes all components
void setup() {
  Serial.begin(115200);                     // Initialize serial communication for debugging
  dht.begin();                              // Initialize temperature/humidity sensor
  
  // Attach servo motors to their control pins
  gasValveServo.attach(SERVO_GAS_VALVE_PIN);
  doorServo.attach(SERVO_DOOR_PIN);

  // Set servos to initial positions (0 degrees)
  gasValveServo.write(0);                   // Gas valve open position
  doorServo.write(0);                       // Door closed position

  // Configure digital pins as inputs or outputs
  pinMode(SW420_PIN, INPUT);                // Vibration sensor input
  pinMode(FLAME_PIN, INPUT);                // Flame sensor digital input
  pinMode(MQ2_PIN, INPUT);                  // Gas sensor analog input
  pinMode(GREEN_LED, OUTPUT);               // Status LED outputs
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);              // Buzzer output
  pinMode(RELAY_FAN_PIN, OUTPUT);           // Fan relay output
  pinMode(ACK_BUTTON_PIN, INPUT_PULLUP);    // Button input with internal pullup resistor

  // Initialize LCD display
  lcd.init();                               // Initialize LCD
  lcd.backlight();                          // Turn on LCD backlight
}

// ========== MAIN LOOP ==========
// Continuously runs to monitor sensors and respond to hazards
void loop() {
  // Priority 1: Always check if user pressed acknowledgment button first
  checkAcknowledgementButton();
  
  // Priority 2: If system is overridden, show normal status and skip hazard detection
  if (overrideActive) {
    handleNormalSituation();
    return;                                 // Exit loop early, skip sensor monitoring
  }

  // ========== SENSOR READING SECTION ==========
  // Read all sensors to determine current conditions
  temperature = dht.readTemperature();              // Get temperature in Celsius
  humidity = dht.readHumidity();                    // Get relative humidity percentage
  gasLevel = analogRead(MQ2_PIN);                   // Get gas concentration (0-1023 range)
  flameDetected = digitalRead(FLAME_PIN);           // Get flame sensor state (LOW = flame detected)
  vibrationDetected = digitalRead(SW420_PIN);       // Get vibration sensor state (HIGH = vibration)

  // Check if temperature/humidity sensor is working properly
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;                                         // Skip this loop if sensor error
  }

  // ========== SENSOR DATA LOGGING ==========
  // Print current sensor readings to serial monitor for debugging
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println("%");
  Serial.print("Gas Level: ");
  Serial.println(gasLevel);
  Serial.print("Vibration: ");
  Serial.println(vibrationDetected ? "Detected" : "Not Detected");
  Serial.print("Flame: ");
  Serial.println(flameDetected ? "Not Detected" : "Detected");    // Note: LOW = flame detected

  // ========== HAZARD DETECTION AND RESPONSE LOGIC ==========
  // Check if any hazardous conditions are present
  // Thresholds: Temperature > 35°C, Gas Level > 500, Vibration = HIGH, Flame = LOW
  if (temperature >= 32 || gasLevel > 400 || vibrationDetected || !flameDetected) {
    
    // ========== MULTIPLE HAZARD SCENARIO HANDLING ==========
    // Each condition checks specific combinations of hazards and responds appropriately
    
    // Scenario 1: Earthquake + Fire (vibration + flame, normal temp/gas)
    if (temperature <= 32 && gasLevel <= 400 && vibrationDetected && !flameDetected) {
      handleEarthquakeGasLeak();            // Handle as earthquake with fire risk
      waitForAcknowledgment(5000);          // Wait 5 seconds for user acknowledgment
    }
    // Scenario 2: Earthquake only (vibration only, no other hazards)
    else if (temperature <= 32 && gasLevel <= 400 && vibrationDetected && flameDetected) {
      handleEarthquake();                   // Handle earthquake scenario
      waitForAcknowledgment(5000);
    }
    // Scenario 3: Gas leak + Fire (gas + flame, no earthquake/high temp)
    else if (temperature <= 32 && gasLevel > 400 && !vibrationDetected && !flameDetected) {
      handleGasLeakDoor();                  // Handle gas leak with flame present
      waitForAcknowledgment(5000);
    }
    // Scenario 4: Gas leak only (high gas, no other hazards)
    else if (temperature <= 32 && gasLevel > 400 && !vibrationDetected && flameDetected) {
      handleGasLeak();                      // Handle gas leak scenario
      waitForAcknowledgment(5000);
    }
    // Scenario 5: Gas leak + Earthquake + Fire (gas + vibration + flame)
    else if (temperature <= 32 && gasLevel > 400 && vibrationDetected && !flameDetected) {
      handleEarthquakeGasLeak();            // Handle combined earthquake and gas leak
      waitForAcknowledgment(5000);
    }
    // Scenario 6: Gas leak + Earthquake (gas + vibration, no flame)
    else if (temperature <= 32 && gasLevel > 400 && vibrationDetected && flameDetected) {
      handleEarthquakeGasLeak();            // Handle combined hazards
      waitForAcknowledgment(5000);
    }
    // Scenario 7: Fire + High temperature (high temp + flame, no gas/earthquake)
    else if (temperature >= 32 && gasLevel <= 400 && !vibrationDetected && !flameDetected) {
      handleFire();                         // Handle fire scenario
      waitForAcknowledgment(5000);
    }
    // Scenario 8: High temperature only (high temp, no other hazards)
    else if (temperature > 32 && gasLevel <= 400 && !vibrationDetected && flameDetected) {
      handleVentilation();                  // Handle high temperature with ventilation
      waitForAcknowledgment(5000);
    }
    // Scenario 9: Fire + Earthquake + High temperature
    else if (temperature > 32 && gasLevel <= 400 && vibrationDetected && !flameDetected) {
      handleFireEarthquake();               // Handle fire during earthquake
      waitForAcknowledgment(5000);
    }
    // Scenario 10: High temperature + Earthquake
    else if (temperature > 32 && gasLevel <= 400 && vibrationDetected && flameDetected) {
      handleVentilationEarthquake();        // Handle high temp during earthquake
      waitForAcknowledgment(5000);
    }
    // Scenario 11: Fire + Gas leak + High temperature
    else if (temperature > 32 && gasLevel > 400 && !vibrationDetected && !flameDetected) {
      handleFireGasLeak();                  // Handle fire with gas leak
      waitForAcknowledgment(5000);
    }
    // Scenario 12: Gas leak + High temperature
    else if (temperature > 32 && gasLevel > 400 && !vibrationDetected && flameDetected) {
      handleGasLeak();                      // Handle gas leak scenario
      waitForAcknowledgment(5000);
    }
    // Scenario 13: All hazards present (worst case scenario)
    else if (temperature > 32 && gasLevel > 400 && vibrationDetected && !flameDetected) {
      handleAll();                          // Handle all hazards simultaneously
      waitForAcknowledgment(5000);
    }
    // Scenario 14: High temperature + Gas leak + Earthquake
    else if (temperature > 32 && gasLevel > 400 && vibrationDetected && flameDetected) {
      handleEarthquakeGasLeak();            // Handle combined earthquake and gas leak
      waitForAcknowledgment(5000);
    }
    // Default case: Handle as normal if no specific scenario matches
    else {
      handleNormalSituation();
    }
    
    waitForAcknowledgment(5000);            // Additional safety wait as in original code
  }
  else {
    // ========== NO HAZARD DETECTED ==========
    handleNormalSituation();               // All conditions normal, show normal status
  }

  // ========== END OF LOOP CLEANUP ==========
  // Turn off all alarms at end of each loop cycle
  digitalWrite(BUZZER_PIN, LOW);            // Silence buzzer
  digitalWrite(RED_LED, LOW);               // Turn off red LED
  digitalWrite(YELLOW_LED, LOW);            // Turn off yellow LED
  digitalWrite(BLUE_LED, LOW);              // Turn off blue LED
  delay(500);                               // Half-second delay before next loop
}

// ========== ACKNOWLEDGMENT SYSTEM FUNCTIONS ==========

// Non-blocking wait function that continues checking for acknowledgment button
// while waiting for specified time period
void waitForAcknowledgment(unsigned long waitTime) {
  unsigned long startTime = millis();       // Record when wait period started
  inAlertMode = true;                       // Set flag that system is in alert mode
  
  // Continue waiting until time expires OR user presses acknowledgment button
  while (millis() - startTime < waitTime && !overrideActive) {
    checkAcknowledgementButton();           // Keep checking button during wait
    if (overrideActive) {                   // If button was pressed
      inAlertMode = false;                  // Exit alert mode
      return;                               // Exit immediately, don't continue waiting
    }
    delay(50);                              // Small delay to prevent system overload
  }
  
  inAlertMode = false;                      // Clear alert mode flag when done waiting
}

// Button handling function with debouncing to prevent false triggers
void checkAcknowledgementButton() {
  bool currentButtonState = digitalRead(ACK_BUTTON_PIN);    // Read current button state
  unsigned long currentTime = millis();                     // Get current timestamp
  
  // Detect button press: transition from HIGH to LOW (due to pullup resistor)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    // Debounce check: ignore if button pressed too recently (prevents bounce)
    if (currentTime - lastButtonPress > 200) {             // 200ms debounce period
      lastButtonPress = currentTime;                        // Update last press time
      buttonPressed = true;                                 // Set button press flag
      
      // If system is currently alerting, activate override mode
      if (inAlertMode) {
        overrideActive = true;                              // Enable system override
        resetSystem();                                      // Reset system to safe state
        Serial.println("System reset due to button press.");
      }
    }
  }
  
  lastButtonState = currentButtonState;                     // Update button state for next check
}

// ========== HAZARD RESPONSE FUNCTIONS ==========

// Normal operation - no hazards detected
void handleNormalSituation() {
  // Update LCD display to show normal status
  lcd.setCursor(0, 0);                      // Move cursor to first line
  lcd.print("System Normal   ");           // Display status (extra spaces clear old text)
  lcd.setCursor(0, 1);                      // Move cursor to second line
  lcd.print("Temp: ");                      // Show current temperature and humidity
  lcd.print(temperature);
  lcd.print(" Hum: ");
  lcd.print(humidity);
  lcd.print("  ");                          // Clear any remaining characters
  
  // Set LED indicators for normal operation
  digitalWrite(GREEN_LED, HIGH);            // Green LED on = normal operation
  digitalWrite(RED_LED, LOW);               // All alert LEDs off
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);            // Buzzer off
  digitalWrite(RELAY_FAN_PIN, LOW);         // Ventilation fan off
  gasValveServo.write(0);                   // Gas valve open (normal position)
  doorServo.write(0);                       // Door closed (normal position)
}

// Earthquake detected (vibration sensor triggered)
void handleEarthquake() {
  // Display earthquake alert on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Earthquake!");                 // Alert message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");           // Instructions for user
  
  // Set earthquake alert indicators
  digitalWrite(YELLOW_LED, HIGH);           // Yellow LED indicates earthquake
  digitalWrite(GREEN_LED, LOW);             // Turn off normal operation LED
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, LOW);         // Keep fan off during earthquake
  gasValveServo.write(0);                   // Keep gas valve open
  doorServo.write(90);                      // Open door for evacuation route
}

// Gas leak detected (MQ-2 sensor reading above threshold)
void handleGasLeak() {
  // Display gas leak alert on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gas Leak!");                   // Alert message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");           // User instructions
  
  // Set gas leak alert indicators
  digitalWrite(BLUE_LED, HIGH);             // Blue LED indicates gas leak
  digitalWrite(GREEN_LED, LOW);             // Turn off all other LEDs
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Turn on ventilation fan to clear gas
  gasValveServo.write(180);                  // Close gas valve to stop leak source
  doorServo.write(0);                       // Keep door closed to contain gas
}

// Gas leak with flame present (extremely dangerous - explosive risk)
void handleGasLeakDoor() {
  // Display critical gas + flame alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gas Leak + Flame!");          // Critical alert message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set critical alert indicators (gas + fire)
  digitalWrite(BLUE_LED, HIGH);             // Blue for gas leak
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, HIGH);              // Red for fire/explosion risk
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Ventilation on to clear gas
  gasValveServo.write(90);                  // Close gas valve immediately
  doorServo.write(90);                      // Open door for evacuation
}

// Fire detected (high temperature + flame sensor triggered)
void handleFire() {
  // Display fire alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire Detected!");             // Fire alert message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set fire alert indicators
  digitalWrite(RED_LED, HIGH);              // Red LED indicates fire
  digitalWrite(GREEN_LED, LOW);             // Turn off other LEDs
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound fire alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Turn on fan to remove smoke and heat
  gasValveServo.write(90);                  // Close gas valve (fire safety)
  doorServo.write(90);                      // Open door for evacuation
}

// High temperature but no flame (overheating situation)
void handleVentilation() {
  // Display high temperature alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("High Temp!");                 // Temperature alert
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set ventilation mode (no emergency LEDs, just cooling)
  digitalWrite(BLUE_LED, LOW);              // All alert LEDs off
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Turn on fan for cooling
  gasValveServo.write(0);                   // Keep gas valve open (no immediate danger)
  doorServo.write(0);                       // Keep door closed (just ventilation needed)
}

// Combined earthquake and gas leak (double hazard)
void handleEarthquakeGasLeak() {
  // Display combined hazard alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("EQ + Gas Leak!");             // Combined hazard message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set indicators for both hazards
  digitalWrite(YELLOW_LED, HIGH);           // Yellow for earthquake
  digitalWrite(BLUE_LED, HIGH);             // Blue for gas leak
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Ventilation on for gas clearance
  gasValveServo.write(90);                  // Close gas valve for safety
  doorServo.write(90);                      // Open door for evacuation
}

// Fire during earthquake (complex emergency scenario)
void handleFireEarthquake() {
  // Display fire + earthquake alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire + EQ!");                 // Combined emergency message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set indicators for both hazards
  digitalWrite(RED_LED, HIGH);              // Red for fire
  digitalWrite(YELLOW_LED, HIGH);           // Yellow for earthquake
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, LOW);         // Fan OFF to avoid spreading fire
  gasValveServo.write(90);                  // Close gas valve (fire safety)
  doorServo.write(90);                      // Open door for evacuation
}

// High temperature during earthquake
void handleVentilationEarthquake() {
  // Display temperature + earthquake alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp + EQ!");                 // Combined condition message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set indicators for both conditions
  digitalWrite(YELLOW_LED, HIGH);           // Yellow for earthquake
  digitalWrite(BLUE_LED, LOW);              // Blue off (no gas leak)
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Ventilation on for cooling
  gasValveServo.write(0);                   // Keep gas valve open
  doorServo.write(90);                      // Open door for evacuation (earthquake)
}

// Fire with gas leak present (extremely dangerous)
void handleFireGasLeak() {
  // Display critical fire + gas alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fire + Gas Leak!");           // Critical emergency message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Set indicators for both critical hazards
  digitalWrite(RED_LED, HIGH);              // Red for fire
  digitalWrite(BLUE_LED, HIGH);             // Blue for gas leak
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BUZZER_PIN, HIGH);           // Sound critical alarm
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Ventilation on (risky but necessary)
  gasValveServo.write(90);                  // Close gas valve immediately
  doorServo.write(90);                      // Open door for immediate evacuation
}

// Multiple hazards detected simultaneously (worst case scenario)
void handleAll() {
  // Display multiple hazards alert
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Multiple Hazards!");          // Maximum alert message
  lcd.setCursor(0, 1);
  lcd.print("Press ACK to stop");
  
  // Activate all hazard indicators
  digitalWrite(RED_LED, HIGH);              // Red for fire
  digitalWrite(YELLOW_LED, HIGH);           // Yellow for earthquake
  digitalWrite(BLUE_LED, HIGH);             // Blue for gas leak
  digitalWrite(GREEN_LED, LOW);             // Green off (not normal)
  digitalWrite(BUZZER_PIN, HIGH);           // Sound maximum alert
  digitalWrite(RELAY_FAN_PIN, HIGH);        // Ventilation on for gas/smoke clearance
  gasValveServo.write(90);                  // Close gas valve for safety
  doorServo.write(90);                      // Open door for immediate evacuation
}

// Reset system to safe state when acknowledgment button is pressed
void resetSystem() {
  // Display reset confirmation message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Reset");               // Reset confirmation message
  lcd.setCursor(0, 1);
  lcd.print("Button Pressed!");            // Show button was pressed
  
  // Set all outputs to safe/normal state
  digitalWrite(GREEN_LED, HIGH);            // Green LED on (system reset)
  digitalWrite(RED_LED, LOW);               // All alert LEDs off
  digitalWrite(YELLOW_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER_PIN, LOW);            // Turn off buzzer immediately
  digitalWrite(RELAY_FAN_PIN, LOW);         // Turn off fan
  gasValveServo.write(0);                   // Open gas valve (normal position)
  doorServo.write(0);                       // Close door (normal position)
  
  delay(2000);                              // Show reset message for 2 seconds
  overrideActive = false;                   // Clear override flag after showing message
}