from flask import Flask, render_template, jsonify
from flask_socketio import SocketIO
import smbus
import time
import math
import adafruit_dht as dht
from PMS7003 import PMS7003
import serial
import board
import threading
import RPi.GPIO as GPIO

# 부저 핀 번호
buzzer = 18

# MPU6050 레지스터 주소
MPU6050_ADDR = 0x69
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H = 0x43

bus = smbus.SMBus(1)  # Raspberry Pi의 I2C 버스

# MPU6050 초기화
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

def read_raw_data(addr):
    # 두 개의 레지스터에서 데이터 읽기
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:  # 음수 처리
        value = value - 65536
    return value

# 초기 변수 설정
prev_time = time.time()
angle_x = angle_y = angle_z = 0

# PMS7003 세팅
dust = PMS7003()

# Baud Rate
Speed = 9600

# UART / USB Serial
USB0 = '/dev/ttyUSB0'
UART = '/dev/ttyAMA0'

# USB PORT
SERIAL_PORT = USB0

pms7003_data = {
    "pm10": 0,
    "pm25": 0,
    "pm100": 0,
}

# Flask 웹 서버 설정
app = Flask(__name__)
socketio = SocketIO(app)

DHT22_data = {
    "temp": 0.0,
    "humi": 0.0,
}

mpu6050_data = {
    "x": 0.00,
    "y": 0.00,
    "z": 0.00,
}

# 각 면의 기능 설명
face_template = {
    1: "Clock",  # 시계
    2: "Particle",  # 미세먼지
    3: "Timer",  # 타이머
    4: "TempHumi",  # 온습도
}

face_data = {
    1: "시계 기능: 현재 시간을 보여줍니다.",
    2: pms7003_data,
    3: "타이머 기능: 설정된 타이머를 보여줍니다.",
    4: DHT22_data,
}

dht_device = dht.DHT22(board.D4)

def determine_face(accel_x, accel_y):
    angle_x = math.atan2(accel_y, math.sqrt(accel_x ** 2)) * 180 / math.pi
    angle_y = math.atan2(accel_x, math.sqrt(accel_y ** 2)) * 180 / math.pi

    if -45 <= angle_x <= 45 and -45 <= angle_y <= 45:
        return 1
    elif angle_x > 45:
        return 2
    elif angle_x < -45:
        return 3
    elif angle_y > 45:
        return 4
    elif angle_y < -45:
        return 5
    else:
        return 6
    # 각도 계산 필요

def get_dustchk():
    #serial setting
    ser = serial.Serial(SERIAL_PORT, Speed, timeout = 1)

    buffer = ser.read(1024)
    if(dust.protocol_chk(buffer)):
        data = dust.unpack_data(buffer)
        return data[dust.DUST_PM2_5_ATM], data[dust.DUST_PM10_0_ATM]
    else:
        print ("data read Err")

    ser.close()

def read_sensor_data():
    temperature = 0
    humidity = 0
    try:
        while True:
            # 센서 데이터를 읽는 부분
            try:
                temperature = dht_device.temperature
                humidity = dht_device.humidity
            except RuntimeError as error:
                print("Sensor error")
            except Exception as error:
                dht_device.exit()
                raise error
            DHT22_data['temp'] = temperature
            DHT22_data['humi'] = humidity
    
            pms7003_data['pm25'], pms7003_data['pm100'] = get_dustchk()
            
            print(f"데이터 : 온도 {DHT22_data['temp']} / 습도 {DHT22_data['humi']}")
            print(f"데이터 : 25 {pms7003_data['pm25']} / 100 {pms7003_data['pm100']}")
            
            # 2초마다 센서 데이터 갱신
            time.sleep(2)
    finally:
        dht_device.exit()


def read_mpu_data():
    new_value = 1
    global prev_time, angle_x, angle_y, angle_z
    try:
        while True:
            global current_eye
            
            # 가속도계 데이터 읽기
            acc_x = read_raw_data(ACCEL_XOUT_H)
            acc_y = read_raw_data(ACCEL_XOUT_H + 2)
            acc_z = read_raw_data(ACCEL_XOUT_H + 4)

            # 자이로스코프 데이터 읽기
            gyro_x = read_raw_data(GYRO_XOUT_H)
            gyro_y = read_raw_data(GYRO_XOUT_H + 2)
            gyro_z = read_raw_data(GYRO_XOUT_H + 4)

            # 가속도계 각도 계산
            acc_angle_x = math.atan2(acc_y, acc_z) * 180 / math.pi
            acc_angle_y = math.atan2(-acc_x, math.sqrt(acc_y**2 + acc_z**2)) * 180 / math.pi
            acc_angle_z = 0  # 가속도계로 Z축 각도는 직접 계산 불가 (대부분 자이로 사용)

            # 현재 시간 계산
            curr_time = time.time()
            elapsed_time = curr_time - prev_time

            # 자이로스코프 각도 변화 계산
            gyro_angle_x = gyro_x / 131.0 * elapsed_time  # 131: 감도 스케일 팩터
            gyro_angle_y = gyro_y / 131.0 * elapsed_time
            gyro_angle_z = gyro_z / 131.0 * elapsed_time

            # 컴플리멘터리 필터
            angle_x = 0.98 * (angle_x + gyro_angle_x) + 0.02 * acc_angle_x
            angle_y = 0.98 * (angle_y + gyro_angle_y) + 0.02 * acc_angle_y
            angle_z = angle_z + gyro_angle_z  # Z축은 자이로스코프만 사용

            prev_time = curr_time

            # 출력 부분을 데이터로 저장하기
            mpu6050_data['x'] = angle_x
            mpu6050_data['y'] = angle_y
            mpu6050_data['z'] = angle_z
            
            global current_eye
            
            if mpu6050_data['x'] > 160.0 or mpu6050_data['x'] < -160.0:
                new_value = 3
            elif mpu6050_data['x'] > -10.0 and mpu6050_data['x'] < 10.0:
                new_value = 1
            elif mpu6050_data['x'] > -90.0 and mpu6050_data['x'] < -70.0:
                new_value = 4
            elif mpu6050_data['x'] > 70.0 and mpu6050_data['x'] < 90.0:
                new_value = 2
                
            if new_value != current_eye:
                current_eye = new_value
                print(f"센서 값 변경: {current_eye}")
                socketio.emit('reload_page')  # 클라이언트로 새로 고침 이벤트를 보냄

            time.sleep(0.01)
    except:
        print('에러')

sensor_thread_started = False

# 센서 스레드를 시작하는 함수
def start_sensor_thread():
    global sensor_thread_started
    if not sensor_thread_started:  # 스레드가 시작되지 않은 경우에만 실행
        sensor_thread = threading.Thread(target=read_sensor_data)
        sensor_thread.daemon = True  # Flask 서버가 종료되면 스레드도 종료
        sensor_thread.start()
        sensor_thread_started = True  # 스레드가 시작되었음을 표시
        
mpu_thread_started = False

# 센서 스레드를 시작하는 함수
def start_mpu_thread():
    global mpu_thread_started
    if not mpu_thread_started:  # 스레드가 시작되지 않은 경우에만 실행
        mpu_thread = threading.Thread(target=read_mpu_data)
        mpu_thread.daemon = True  # Flask 서버가 종료되면 스레드도 종료
        mpu_thread.start()
        mpu_thread_started = True  # 스레드가 시작되었음을 표시


@app.route('/')
def display_face():
    data = face_data[current_eye]  # face_data[face]
    return render_template(face_template[current_eye] + '.html', data=data)


@socketio.on('finishTimer')
def finish_timer():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(buzzer, GPIO.OUT)
    GPIO.setwarnings(False)
    
    pwm = GPIO.PWM(buzzer, 1024)
    pwm.start(100.0)
    time.sleep(1.5)
    pwm.stop()
    time.sleep(0.75)
    pwm.start(100.0)
    time.sleep(1.5)
    pwm.stop()
    
    GPIO.cleanup()


@socketio.on('connect')
def handle_connect():
    print('클라이언트 연결됨')


if __name__ == '__main__':
    # 센서 스레드를 시작
    start_sensor_thread()
    start_mpu_thread()
    
    # Flask 서버 실행
    socketio.run(app, debug=False, host='0.0.0.0', port=5000, allow_unsafe_werkzeug=True)
