#   M I N E   D E T E C T I O N   D R O N E   S E R V E R



# IMPORTS
import logging
import requests
import threading
import time
from flask import Flask, jsonify, request
from flask_restful import Api, Resource
from flask_cors import CORS



# defining the server
app = Flask(__name__)
api = Api(app)
CORS(app, origins=['http://localhost:3000', 'http://172.20.10.7:3000']) # allowed URLs
# server address
# http://172.20.10.6:5000



# icon
@app.route('/favicon.ico')
def favicon():
    return app.send_static_file('favicon.ico')



# lock
print_lock = threading.Lock()



# GLOBAL VARIABLES
flight_time = 0
drone_started = False
rotors_states = {}
drone_height = 0
mine_detected = False
last_coordinates = {}
battery_level = 100



# FUNCTIONS
# Flight Time
def send_flight_time():
    global flight_time
    interval = 1

    url = 'http://127.0.0.1:5000/send-flight-time'
    
    while True:
        try:
            headers = {'Content-Type': 'application/json'}
            response = requests.get(url)

            if response.status_code == 200:
                data = response.json()
                isRunning = data.get('isRunning', False)
            else:
                isRunning = False

            if isRunning:
                flight_time += 1

            response = requests.post(url, json={'isRunning': isRunning, 'flight_time': flight_time}, headers=headers)

            with print_lock:
                if response.status_code == 200:
                    print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + 'Flight Time' + '\033[0m')
                    print('\033[38;2;{r};{g};{b}m'.format(r=192, g=255, b=192) + ' Value = ' + '\033[0m' + str(flight_time))
                else:
                    print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Flight Time' + '\033[0m')
                    print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            with print_lock:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + 'Flight Time' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))
        
        time.sleep(interval)
# Rotors' States
def send_rotors_states():
    global rotors_states
    interval = 1

    url = 'http://127.0.0.1:5000/send-rotors-states'

    while True:
        try:
            response = requests.get(url)
            if response.status_code == 200:
                rotors_states = response.json().get('rotors_states', {})

                headers = {'Content-Type': 'application/json'}
                response = requests.post(url, json=rotors_states, headers=headers)

                with print_lock:
                    if response.status_code == 200:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + "Rotors' States" + '\033[0m')
                    else:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + "Rotors' States" + '\033[0m')
                        print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
            else:
                with print_lock:
                    print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + "Rotors' States" + '\033[0m')
                    print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            with print_lock:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + "Rotors' States" + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))

        time.sleep(interval)
# Drone Height
def send_drone_height():
    global drone_height
    interval = 1

    url = 'http://127.0.0.1:5000/send-drone-height'

    while True:
        try:
            response = requests.get(url)

            if response.status_code == 200:
                drone_height = response.get('drone_height', 0)

                headers = {'Content-Type': 'application/json'}
                response = requests.post(url, json={'drone_height': drone_height}, headers=headers)

                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + 'Drone Height' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=192, g=255, b=192) + ' Value = ' + '\033[0m' + str(drone_height))
            else:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Drone Height' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + 'Drone Height' + '\033[0m')
            print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))

        time.sleep(interval)
# Coordinates
def send_coordinates():
    global last_coordinates
    interval = 1

    url = 'http://127.0.0.1:5000/send-coordinates'

    while True:
        try:
            response = requests.get(url)
            if response.status_code == 200:
                last_coordinates = response.json().get('coordinates', {}) 

                headers = {'Content-Type': 'application/json'}
                response = requests.post(url, json=last_coordinates, headers=headers)

                with print_lock:
                    if response.status_code == 200:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + 'Coordinates' + '\033[0m')
                    else:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Coordinates' + '\033[0m')
                        print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
            else:
                with print_lock:
                    print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Coordinates' + '\033[0m')
                    print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            with print_lock:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + 'Coordinates' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))

        time.sleep(interval)
# Mine Detection
def send_mine_detection():
    global mine_detected
    interval = 1

    url = 'http://127.0.0.1:5000/send-mine-detection'

    while True:
        try:
            response = requests.get(url)
            if response.status_code == 200:
                mine_detected = response.json().get('mine_detected', False)

                headers = {'Content-Type': 'application/json'}
                response = requests.post(url, json={'mine_detected': mine_detected}, headers=headers)

                with print_lock:
                    if response.status_code == 200:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + 'Mine Detection' + '\033[0m')
                        print('\033[38;2;{r};{g};{b}m'.format(r=192, g=255, b=192) + ' Value = ' + '\033[0m' + str(mine_detected))
                    else:
                        print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Mine Detection' + '\033[0m')
                        print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
            else:
                with print_lock:
                    print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Mine Detection' + '\033[0m')
                    print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            with print_lock:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + 'Mine Detection' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))

        mine_detected = not mine_detected

        time.sleep(interval)
# Battery Level
def send_battery_level():
    global battery_level
    interval = 10

    url = 'http://127.0.0.1:5000/send-battery-level'

    while True:
        try:
            response = requests.get(url)

            if response.status_code == 200:
                battery_level = response.get('battery_level', 100)

                headers = {'Content-Type': 'application/json'}
                response = requests.post(url, json={'battery_level': battery_level}, headers=headers)

                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'SUCCESS = ' + '\033[38;2;{r};{g};{b}m'.format(r=128, g=255, b=128) + 'Battery Level' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=192, g=255, b=192) + ' Value = ' + '\033[0m' + str(battery_level))
            else:
                print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=0) + 'FAILURE = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=128, b=128) + 'Battery Level' + '\033[0m')
                print('\033[38;2;{r};{g};{b}m'.format(r=255, g=192, b=192) + ' Status Code = ' + '\033[0m' + str(response.status_code))
        except Exception as e:
            print('\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=0) + 'ERROR = ' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=128) + 'Battery Level' + '\033[0m')
            print('\033[38;2;{r};{g};{b}m'.format(r=255, g=255, b=192) + ' Exception = ' + '\033[0m' + str(e))

        time.sleep(interval)



# RESOURCES
# Hello
class Hello(Resource):
    def get(self):
        return jsonify({'message': 'Server is running...'})
# Flight Time
class FlightTime(Resource):
    def get(self):
        global flight_time

        return jsonify({'flight_time': flight_time})
    def post(self):
        data = request.json
        flight_time = data.get('flight_time')

        return jsonify({'flight_time': flight_time})
# Rotors' States
class RotorsStates(Resource):
    def get(self):
        global rotors_states

        return jsonify({'rotors_states': rotors_states})
    def post(self):
        data = request.json
        rotor_nw_rpm = data.get('rotor_nw_rpm')
        rotor_nw_angle = data.get('rotor_nw_angle')
        rotor_ne_rpm = data.get('rotor_ne_rpm')
        rotor_ne_angle = data.get('rotor_ne_angle')
        rotor_sw_rpm = data.get('rotor_sw_rpm')
        rotor_sw_angle = data.get('rotor_sw_angle')
        rotor_se_rpm = data.get('rotor_se_rpm')
        rotor_se_angle = data.get('rotor_se_angle')
        
        return jsonify({
            'rotors_states': {
                'rotor_nw': {
                    'rotor_nw_rpm': rotor_nw_rpm,
                    'rotor_nw_angle': rotor_nw_angle
                },
                'rotor_ne': {
                    'rotor_ne_rpm': rotor_ne_rpm,
                    'rotor_ne_angle': rotor_ne_angle
                },
                'rotor_sw': {
                    'rotor_sw_rpm': rotor_sw_rpm,
                    'rotor_sw_angle': rotor_sw_angle
                },
                'rotor_se': {
                    'rotor_se_rpm': rotor_se_rpm,
                    'rotor_se_angle': rotor_se_angle
                }
            }
        })
# Drone Height
class DroneHeight(Resource):
    def get(self):
        global drone_height

        return jsonify({'drone_height': drone_height})
    def post(self):
        data = request.json
        drone_height = data.get('drone_height')

        return jsonify({'drone_height': drone_height})
# Coordinates
class Coordinates(Resource):
    def get(self):
        global last_coordinates

        return jsonify({'last_coordinates': last_coordinates})
    def post(self):
        data = request.json
        latitude = data.get('latitude')
        longitude = data.get('longitude')

        return jsonify({
            'coordinates': {
                'latitude': latitude,
                'longitude': longitude
            }
        })
# Mine Detection
class MineDetection(Resource):
    def get(self):
        global mine_detected

        return jsonify({'mine_detected': mine_detected})
    def post(self):
        data = request.json
        mine_detected = data.get('mine_detected')

        return jsonify({'mine_detected': mine_detected})
# Battery Level
class BatteryLevel(Resource):
    def get(self):
        global battery_level

        return jsonify({'battery_level': battery_level})
    def post(self):
        data = request.json
        battery_level = data.get('battery_level')

        return jsonify({'battery_level': battery_level})

# adding the resources
api.add_resource(Hello, '/', methods=['GET'])
api.add_resource(FlightTime, '/send-flight-time', methods=['GET', 'POST'])
api.add_resource(RotorsStates, '/send-rotors-states', methods=['GET', 'POST'])
api.add_resource(DroneHeight, '/send-drone-height', methods=['GET', 'POST'])
api.add_resource(Coordinates, '/send-coordinates', methods=['GET', 'POST'])
api.add_resource(MineDetection, '/send-mine-detection', methods=['GET', 'POST'])
api.add_resource(BatteryLevel, '/send-battery-level', methods=['GET', 'POST'])



# MAIN
if __name__ == '__main__':
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.CRITICAL)

    print('\n\n\n' + '\033[38;2;{r};{g};{b}m'.format(r=255, g=0, b=255) + 'MINE DETECTION DRONE SERVER' + '\033[0m')
    print('\033[38;2;{r};{g};{b}m'.format(r=0, g=255, b=0) + 'Server is running...' + '\033[0m')

    t1 = threading.Thread(target=send_flight_time)
    t1.start()
    t2 = threading.Thread(target=send_rotors_states)
    t2.start()
    t3 = threading.Thread(target=send_drone_height)
    t3.start()
    t4 = threading.Thread(target=send_coordinates)
    t4.start()
    t5 = threading.Thread(target=send_mine_detection)
    t5.start()
    t6 = threading.Thread(target=send_battery_level)
    t6.start()

    app.run(debug=False)


