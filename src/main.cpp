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
    void parseMessage(char* message);
    void receiveWithStartEndMarkers();
    void interpretMessage();
    
// DATA COLLECTION: RAMP
    // Data structure for the set of data points
    struct datum {
        float pwm;
        float rpm;
        float thrust;
    };
    int n_samples;
    float step;
    float min_pwm;
    float max_pwm;
    void getDatumPoint(float pwmValue, int n_samples, struct datum *z);
    void collectDataRamp(int &n_samples, float &step);

// DATA COLLECTION: STEP
    const int step_n_samples = 1000;
    volatile unsigned long t_samples[step_n_samples];
    volatile float rpm_samples[step_n_samples];
    volatile int sample_index = 0;
    float step_value;
    volatile unsigned long max_time;
    volatile unsigned long t, t_start;
    void collectDataStep(float &step_value, volatile unsigned long &max_time);

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
    void IRAM_ATTR computeRPM_storeInVariable();    // For the ramp measurement
    void IRAM_ATTR computeRPM_storeInMatrix();      // For the step measurement

// LOADCELL with HX711 amplificator
    #define AMP_DT 22
    #define AMP_SCK 23
    HX711 loadcell;
    double thrust;
    #define LOADCELL_GAIN 399243    // Caibration value: output with 1kg load

// ESC controller for the BLDC motor
    #define ESC_PIN 26
    ESC motor(ESC_PIN);
    int speed;
    void setPWM(float);

// Emergency button
    #define BUTTON_PIN 15
    void IRAM_ATTR stop();

// ================================== SETUP & LOOP ================================================
void setup() {
    Serial.begin(9600);
    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);

    // Interrupt routines
    attachInterrupt(IR, computeRPM_storeInVariable, RISING);
    attachInterrupt(BUTTON_PIN, stop, RISING);
    
    // Loadcell configuration
    loadcell.begin(AMP_DT, AMP_SCK);
    loadcell.set_scale(LOADCELL_GAIN);
    loadcell.tare();

    // ESC configuration
    //motor.attach(ESC_PIN, 1000, 2000);
    motor.arm();
    setPWM(0);

    Serial.println("\r\nArduino is ready");
}

void loop() {
    receiveWithStartEndMarkers();
    interpretMessage();
}

// ================================= FUNCTION DEFINITIONS ==================================
void parseMessage(char* message) {
    int i;

    // Parse the command
    command = message;                  // Set the command to start where message start
    for(i=0; message[i]!=' '; i++);     // Read all char until I find a space
    message[i++] = 0;                   // When I find a space, terminate the string

    if (!strcmp(command, "R")) {
        char *n_samples_string, *step_string, *min_pwm_string, *max_pwm_string;

        // Parse the number of samples
        n_samples_string = message + i;
        for( ; message[i]!=' '; i++);       
        message[i++] = 0;

        // Parse the step
        step_string = message + i;
        for( ; message[i]!=' '; i++);       
        message[i++] = 0;

        // Parse the min_pwm value
        min_pwm_string = message + i;
        for( ; message[i]!=' '; i++);       
        message[i++] = 0;

        // Parse the max_pwm value
        max_pwm_string = message + i;

        // Convert the strings to numbers
        n_samples = atoi(n_samples_string);
        step = atof(step_string);
        min_pwm = atof(min_pwm_string);
        max_pwm = atof(max_pwm_string);
    }
    else if (!strcmp(command, "S")) {
        char *step_value_string, *n_samples_string, *max_time_string;

        // Parse the value of the step input
        step_value_string = message + i;
        for( ; message[i]!=' '; i++);       
        message[i++] = 0;

        // Parse the max_time
        max_time_string = message + i;

        // Convert string to numbers
        step_value = atof(step_value_string);
        max_time = atoi(max_time_string);
    }

    
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

    // Blink
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    // Skip transient period
    delay(100);

    // Compute the mean of the required amount of samples
    float sum_rpm = 0;
    float sum_thrust = 0;
    for( int i=0; i<n_samples; i++ ) {
        // rpm are update automatically with interrupts
        sum_rpm += rpm;
        // The force must be obtained by polling the sensor
        sum_thrust += loadcell.get_units(1);     // Expressed in Newtons
        delay(50);
    }
    float mean_rpm = sum_rpm / n_samples;
    float mean_thrust = sum_thrust / n_samples;

    z->pwm = pwmValue;
    z->rpm = mean_rpm ;
    z->thrust = mean_thrust;
}

void collectDataRamp(int &n_samples, float &step) {
    Serial.println("PWM,RPM,Thrust");

    // Set the correct interrupt function
    detachInterrupt(IR);
    attachInterrupt(IR, computeRPM_storeInVariable, RISING);

    // Create the matrix which will contain the data points
    // NO! It provokes a stack overflow because too much memory
    int S_size = (int)(abs((max_pwm-min_pwm)/step)) + 1;

    loadcell.tare();
    setPWM(0);
    rpm = 0;

    // Detects step to decide if going 0->180 or 180->0
    float pwm_0 = step>0 ? min_pwm : max_pwm;
    float pwm_end = step>0 ? max_pwm : min_pwm;

    // Cycle to get the data points for each pwm value
    float pwm=pwm_0;
    for ( int i=0; i<S_size; pwm += step, i++) {
        // Create the datum 
        struct datum z;

        // Pass by reference
        getDatumPoint(pwm, n_samples, &z);

        // Send the matrix to the PC via serial
        Serial.printf("%.2f,%.3f,%.3f\r\n", z.pwm, z.rpm, z.thrust);
    }

    // Send something to say it finished
    Serial.println("Finished");
    setPWM(0);
}

void collectDataStep(float &step_value, volatile unsigned long &max_time) {
    digitalWrite(BUILTIN_LED, HIGH);
    //Serial.println("Time,RPM");

    // Here we use a different approach
    // Every time the IR sensor detects an impulse, we store the rpm value in an array
    // In the array we also find the offset
    // We do not measure the thrust, because it is too slow

    // Matrix of data
    sample_index = 0;

    // Initialize the propeller
    setPWM(1200);
    rpm = 0;
    delay(100);

    // Set the correct interrupt function
    detachInterrupt(IR);
    attachInterrupt(IR, computeRPM_storeInMatrix, RISING);

    // Define the initial delay
    int init_delay = 1000;

    // Set the starting time

    // Wait a small delay to actually see the starting point in the graph
    delay(init_delay);

    digitalWrite(BUILTIN_LED, HIGH);

    // Send the step signal
    t_start = micros();
    setPWM(step_value);

    // Wait until max_time has passed
    delay(max_time);
    detachInterrupt(IR);
    setPWM(0);
    digitalWrite(BUILTIN_LED, LOW);

    // Process the data
    unsigned long t = 0;
    float rpm;
    for (int i=0; i<sample_index; i++) {
        t += t_samples[i];
        rpm = 60000000.0 / t_samples[i];

        t_samples[i] = t;
        rpm_samples[i] = rpm;
    }

    // Now I should have all the data I need in the matrix
    Serial.println("T,RPM");
    Serial.printf("0,0\r\n");
    Serial.printf("%.3f,0\r\n", (float)init_delay);
    for (int i=0; i<sample_index; i++) {
        unsigned long t = t_samples[i]/1000;
        Serial.printf("%lu, %.3f\r\n", t+init_delay, rpm_samples[i]);
    }
    Serial.println("Finished");
}

void interpretMessage() {
    if (newData) {
        Serial.printf("Message received: %s. Response: ", message);
        newData = false;

        // Parse the message
        parseMessage(message);

        if (!strcmp(command, "R")) {
            collectDataRamp(n_samples, step);
        }
        else if (!strcmp(command, "S")) {
            Serial.printf("Command: %s, Step: %d, max_time: %d\r\n", command, step_value, max_time);
            collectDataStep(step_value, max_time);
        }
        else {
            Serial.print("Not implemented");
        }

        Serial.println();
    }
}

void IRAM_ATTR computeRPM_storeInVariable() {
    // Update the timers
    interruptT = micros();
    interruptDeltaT = interruptT - interruptOldT;

    // Check if the minimum delay has passed. If not the interrupt is no considered valid 
    if (interruptDeltaT < MIN_INTERRUPT_DELAY) return;

    // Compute the rpm
    // NOTE: we are receiving an interrupt every time each of the two blades passes
    rpm = 60000000.0 / interruptDeltaT;

    // Store the time instant
    interruptOldT = interruptT;
}

void IRAM_ATTR computeRPM_storeInMatrix() {
    // Update the timers
    interruptT = micros();
    interruptDeltaT = interruptT - interruptOldT;

    // Check if the minimum delay has passed. If not the interrupt is no considered valid 
    if (interruptDeltaT < MIN_INTERRUPT_DELAY) return;

    // Skip rpm computation
    // rpm = 60000000.0 / interruptDeltaT;

    // Time delay from the starting time
    // t = interruptT - t_start;

    // Store the values in the matrix
    /*if (sample_index < step_n_samples) {
        t_samples[sample_index] = t;
        rpm_samples[sample_index] = rpm;
        sample_index++;
    }*/
    if (interruptOldT != 0) {
        t_samples[sample_index] = interruptDeltaT;
        sample_index++;
    }

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
    motor.speed(pwm);
}