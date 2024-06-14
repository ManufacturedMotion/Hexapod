from flask import Flask, render_template, request, jsonify
from multiprocessing import Queue
from threading import Thread

app = Flask(__name__)
command_queue = Queue()
serial_data_queue = Queue()  # Queue for storing serial data

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/send_command', methods=['POST'])
def send_command():
    command = request.form['command']
    print(f"Received command: {command}")
    command_queue.put(command)  # Put the received command into the queue
    return 'Command received'

@app.route('/data')
def get_serial_data():
    if not serial_data_queue.empty():
        data = serial_data_queue.get()
        return jsonify({'data': data})
    else:
        return jsonify({'data': None})

def run_server():
    app.run(host='0.0.0.0', port=5000)

def start_server():
    server_thread = Thread(target=run_server)
    server_thread.start()