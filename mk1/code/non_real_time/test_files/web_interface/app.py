from flask import Flask, request, render_template
import serial

app = Flask(__name__)
ser = serial.Serial('/dev/ttyS0', 115200)  # Serial connection to Teensy
last_received_data = "No data available"

@app.route('/')
def index():
    return render_template('index.html', last_received_data=last_received_data)

@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.form['command']
    ser.write((command + '\n').encode())  # Send command to Teensy
    return 'Command sent successfully'

@app.route('/data')
def get_data():
    global last_received_data
    if ser.in_waiting:
        last_received_data = ser.readline().decode().strip()
        return last_received_data
    else:
        return last_received_data

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)