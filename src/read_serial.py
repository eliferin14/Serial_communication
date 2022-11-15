import serial
import csv

# Function that reads a line and returns the polished string    
    # ser.readline() returns bytes. To obtain something readable I decode with .decode('ascii') and .strip() to remove '\r\n'
def my_readline(ser):
    line = ser.readline().decode('ascii').strip()
    return line

# Function to write a message to the board, following the convention <message>
def my_write(ser, message):
    ser.write(("<"+message+">").encode())

# Serial parameters
port = '/dev/ttyUSB0'
baud = 115200
timeout = 1

# Open serial port
ser = serial.Serial(port, baud, timeout=timeout)
ser.flush()

# NOTES
    # Serial.println() terminates the string with \r\n
    # ser.readline() expects an EOL = \r\n
    # ser.readline() returns bytes. To obtain something readable I decode with .decode('ascii') and .strip() to remove '\r\n'

# Command to collect data: 
# EXAMPLE: M 10 2
command = 'M'   # Measure
n_samples = 1000  # How many samples do I want to average for each input value
step = -0.1      # Step for the input signal
message = " ".join([command, str(n_samples), str(step)])

# TODO: Given n_samples, step and a filename def a function to send the command and store
# so we can use it in a cycle and get multiple instances of datasets easily

# Save data to a file in csv format
# https://makersportal.com/blog/2018/2/25/python-datalogger-reading-the-serial-output-from-arduino-to-analyze-data-using-pyserial
file = open("Dataset.csv", 'a')
file.truncate(0)
csv_writer = csv.writer(file, delimiter=',', escapechar=' ', quoting=csv.QUOTE_NONE)

# Send the command to get the measurements
my_write(ser, message)
# Listen to the response
while 1:
    # Read a line = a row of the matrix S = a sample
    line = my_readline(ser)

    # Check if the line is empty => skip
    if line == '':
        continue

    # If there is something, I check if it is the termination message
    if line == "Finished":
        file.close()
        break

    # If I'm here it is a proper sample
    print(line)
    csv_writer.writerow([line])
    

ser.close() 