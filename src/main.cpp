#include <Arduino.h>
#include <ESP32Servo.h>
#include <ESC.h>  // https://esp32.com/viewtopic.php?t=20450
#include <HX711.h>  // https://randomnerdtutorials.com/esp32-load-cell-hx711/

// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2

// SERIAL
    // Buffer for the serial bytes
    const byte bufferSize = 64;
    char message[bufferSize];
    byte bufferIndex = 0;

    // Start and End marker: every line received starts and ends with these
    const char startMarker = '<';
    const char endMarker = '>'; 
    boolean receiveInProgress;

    // Boolean to indicate if there is data
    boolean newData = false;

    // Parser for the message
    char *command;
    int n_samples;
    float step;
    float max_pwm;
    void parseMessage(char* message);
    void receiveWithStartEndMarkers();
    void interpretMessage();
    
// DATA COLLECTION
    // Data structure for the set of data points
    struct datum {
        float pwm;
        float rpm;
        float voltage;
        float current;
        float force;
    };
    void getDatumPoint(float pwmValue, int n_samples, struct datum *z);
    void collectData(int &n_samples, float &step);

// IR SENSOR (used as encoder): HW201 MH-B
    #define IR 13
    #define MAX_RPM 30000         // max measurable RPM
    const unsigned long MIN_INTERRUPT_DELAY = 1000000 / (MAX_RPM / 60); // Minimum time delay between interrupts to be valid
    //volatile long counter = 0;    // Count the sensor's pulses
    volatile unsigned long interruptT=0;
    volatile unsigned long interruptOldT = 0;
    volatile unsigned long interruptDeltaT;
    volatile float rpm;
    //void IRAM_ATTR addOnePulse();   // When an interrupt is detected, increase the counter
    void IRAM_ATTR computeRPM();    // Returns the measured rpm

// LOADCELL with HX711 amplificator
    #define AMP_DT 22
    #define AMP_SCK 23
    HX711 loadcell;
    double force;
    #define LOADCELL_GAIN 399243    // Caibration value: output with 1kg load

// ESC controller for the BLDC motor
    #define ESC_PIN 26
    Servo motor;
    int speed;      // 0 <= speed <= 180
    void setPWM(float);

// Emergency button
    #define BUTTON_PIN 15
    void IRAM_ATTR stop();

// ================================== SETUP & LOOP ================================================
void setup() {
    Serial.begin(115200);

    // Interrupt routines
    attachInterrupt(IR, computeRPM, RISING);
    attachInterrupt(BUTTON_PIN, stop, RISING);
    
    // Loadcell configuration
    loadcell.begin(AMP_DT, AMP_SCK);
    loadcell.set_scale(LOADCELL_GAIN);
    loadcell.tare();

    // ESC configuration
    motor.attach(ESC_PIN, 1000, 2000);
    setPWM(0);

    Serial.println("Arduino is ready");
}

void loop() {
    receiveWithStartEndMarkers();
    interpretMessage();
}

// ================================= FUNCTION DEFINITIONS ==================================
void parseMessage(char* message) {
    int i;

    char *n_samples_string, *step_string, *max_pwm_string;

    // Parse the command
    command = message;                  // Set the command to start where message start
    for(i=0; message[i]!=' '; i++);     // Read all char until I find a space
    message[i++] = 0;                   // When I find a space, terminate the string

    // Parse the number of samples
    n_samples_string = message + i;
    for( ; message[i]!=' '; i++);       
    message[i++] = 0;

    // Parse the step
    step_string = message + i;
    for( ; message[i]!=' '; i++);       
    message[i++] = 0;

    // Parse the max_pwm value
    max_pwm_string = message + i;

    //Serial.printf("[\'%s\',\'%s\',\'%s\']", command, n_samples_string, step_string);

    // Convert the strings to integers
    n_samples = atoi(n_samples_string);
    step = atof(step_string);
    max_pwm = atof(max_pwm_string);
}

// Read the message
void receiveWithStartEndMarkers() {
    // char I'm about to read from the serial buffer (the bouard buffer, not the one I created!)
    char rc;

    // I read charachters while the sender sends them and I don't receive the endMarker
    while( Serial.available() > 0 && newData == false ) {

        // Read a char from the board serial buffer
        rc = Serial.read();

        // Check if a new message has started
        if (rc == startMarker) {
            // Now I know I'm receiving a message
            receiveInProgress = true;
        }
        // If I'm here it means I'm receiving
        else if (receiveInProgress) {   // redundant?

            // Check if the received char is not the endMarker
            if (rc != endMarker) {
                message[bufferIndex] = rc;
                bufferIndex++;
                // Overflow overwriting
                if (bufferIndex >= bufferSize) bufferIndex = bufferSize-1;
            }
            // If it is, I close the received string and reset the buffer
            else {
                message[bufferIndex] = '\0';
                receiveInProgress = false;
                bufferIndex = 0;
                newData = true;
            }
        }
    }
}

void getDatumPoint(float pwmValue, int n_samples, struct datum *z) {
    // Read data and store it in z

    // Set the pwm value to test
    setPWM(pwmValue);

    // Skip transient period
    delay(200);

    // Compute the mean of the required amount of samples
    float sum_rpm = 0;
    float sum_force = 0;
    for( int i=0; i<n_samples; i++ ) {
        // rpm are update automatically with interrupts
        sum_rpm += rpm;
        // The force must be obtained by polling the sensor
        sum_force += loadcell.get_units(1) * 9.81;
        delay(30);
    }
    float mean_rpm = sum_rpm / n_samples;
    float mean_force = sum_force / n_samples;

    z->pwm = pwmValue;
    z->rpm = mean_rpm ;
    z->voltage = 0;
    z->current = 0;
    z->force = mean_force;
}

void collectData(int &n_samples, float &step) {
    Serial.println("PWM,RPM,Voltage,Current,Force");
    // Create the matrix which will contain the data points
    // NO! It provokes a stack overflow because too much memory
    int S_size = (int)(abs(max_pwm/step)) + 1;
    // struct datum S[S_size];

    loadcell.tare();
    setPWM(0);
    rpm = 0;

    // Detects step to decide if going 0->180 or 180->0
    float pwm_0 = step>0 ? 0 : max_pwm;
    float pwm_end = step>0 ? max_pwm : 0;

    // Cycle to get the data points for each pwm value
    float pwm=pwm_0;
    for ( int i=0; i<S_size; pwm += step, i++) {
        // Create the datum 
        struct datum z;

        // Pass by reference
        getDatumPoint(pwm, n_samples, &z);

        // Send the matrix to the PC via serial
        Serial.printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", z.pwm, z.rpm, z.voltage, z.current, z.force);
    }

    // Send something to say it finished
    Serial.println("Finished");
    setPWM(0);
}

void interpretMessage() {
    if (newData) {
        //Serial.printf("Message received: %s. Response: ", message);
        newData = false;

        // Parse the message
        parseMessage(message);
        //Serial.printf("Command: %s, N: %d, step: %.2f\r\n", command, n_samples, step);
        if (!strcmp(command, "M")) {
            collectData(n_samples, step);
        }
        else {
            Serial.print("Not implemented");
        }

        Serial.println();
    }
}

void IRAM_ATTR computeRPM() {
    // Update the timers
    interruptT = micros();
    interruptDeltaT = interruptT - interruptOldT;

    // Check if the minimum delay has passed. If not the interrupt is no considered valid 
    if (interruptDeltaT < MIN_INTERRUPT_DELAY) return;

    // Compute the rpm
    // NOTE: we are receiving an interrupt every time each of the two blades passes
    rpm = 30000000.0 / interruptDeltaT;

    // Store the time instant
    interruptOldT = interruptT;
}

void IRAM_ATTR stop() {
    setPWM(0);
    Serial.println("STOP");
    ESP.restart();
}

void setPWM(float pwm) {
    // NOTE: the write() function converts pwm to int
    motor.write(pwm);
}