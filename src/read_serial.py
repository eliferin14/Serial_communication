import serial

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

# NOTES
    # Serial.println() terminates the string with \r\n
    # ser.readline() expects an EOL = \r\n
    # ser.readline() returns bytes. To obtain something readable I decode with .decode('ascii') and .strip() to remove '\r\n'

# Command to collect data: 
# EXAMPLE: M 10 2
command = 'M'   # Measure
n_samples = 100  # How many samples do I want to average for each input value
step = 1        # Step for the input signal
message = " ".join([command, str(n_samples), str(step)])

my_write(ser, message)
while 1:
    print(my_readline(ser))



while 1:
    command = input("Enter command: \n \t[N] to get an increasing natural number\nCommand: ")
    ser.write(("<"+command+">").encode())
    print(my_readline(ser))

ser.close() 