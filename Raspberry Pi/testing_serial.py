import serial
import sys

# Define the serial connection
ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)

def send_to_teensy(data):
    # Append a newline character at the end of the data
    ser.write((data + '\n').encode())

# Read input from command line arguments
if len(sys.argv) != 2:
    print("Usage: python raspi_serial.py <data_to_send>")
    sys.exit(1)

data_to_send = sys.argv[1]

try:
    # Send data to Teensy with a newline character
    send_to_teensy(data_to_send)
    print(f"Sent to Teensy: {data_to_send}")
except Exception as e:
    print(f"Error: {e}")

# Close the serial connection
ser.close()
