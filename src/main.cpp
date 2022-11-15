#include <Arduino.h>

// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2

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
char *command, *n_samples_string, *step_string;
int n_samples, step;
void parseMessage(char* message) {
    int i;

    // Parse the command
    command = message;                  // Set the command to start where message start
    for(i=0; message[i]!=' '; i++);     // Read all char until I find a space
    command[i++] = 0;                   // When I find a space, terminate the string

    // Parse the number of samples
    n_samples_string = message + i;
    for( ; message[i]!=' '; i++);       
    n_samples_string[i++] = 0;

    // Parse the step
    step_string = message + i;
    for( ; message[i]!=' '; i++);
    step_string[i++] = 0;

    // Convert the strings to integers
    n_samples = atoi(n_samples_string);
    step = atoi(step_string);
}

// Data structure for the set of data points
struct datum {
    int pwm;
    float rpm;
    float force;
};

void collectData(int n_samples, int step) {
    
}


// Boiate
int i = 0;
void interpretN() {
    Serial.printf("%d", i);
    i++;
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

// Print the message
void interpretMessage() {
    if (newData) {
        Serial.printf("Message received: %s. Response: ", message);
        newData = false;

        // Parse the message
        parseMessage(message);
        Serial.printf("Command: %s, N: %d, step: %d", command, n_samples, step);

        if (!strcmp(command, "N")) {
            interpretN();
        }
        else if (!strcmp(command, "M")) {
            collectData(&n_samples, &step);
        }
        else {
            Serial.print("Not implemented");
        }

        Serial.println();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Arduino is ready");
}

void loop() {
    receiveWithStartEndMarkers();
    interpretMessage();
}