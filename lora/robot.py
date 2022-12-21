import sys
from re import A
import time

import busio
from digitalio import DigitalInOut, Direction, Pull
import board
from board import *

import adafruit_ssd1306
import adafruit_rfm69
import adafruit_dht
from enum import Enum

import RPi.GPIO as GPIO
import threading


MOTOR_ENCODER_PIN = 4
SENSOR_PIN = D19
BAUD_RATE = 2000000
TX_POWER = 13
RADIO_FREQUENCY = 915.0

MOTOR_FWD_PIN = 20
MOTOR_BWD_PIN = 21
ROTATION_ENCODINGS = 121

SERVO_PIN = 13
LEFT_ANGLE = 0
CENTER_ANGLE = 90
RIGHT_ANGLE = 15

class GearState(Enum):
    IDLE = "IDLE"
    FWD = "FWD"
    BWD = "BWD"

class TurnState(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    CENTER = "CENTER"

class Robot:
    def __init__(self, robot=True):
        self.gear = GearState.IDLE
        self.turn = TurnState.CENTER
        self.num_packets = 0
        self.temperature = 0.0
        self.humidity = 0.0
        self.last_rssi = None

        # Enable sensor if robot == TRUE
        self.robot = robot
        self.sensor = adafruit_dht.DHT22(SENSOR_PIN) if robot else None
        self.discover_mode = False # robot drops any state change packets when in discover mode

        # Button A
        self.btnA = DigitalInOut(board.D5)
        self.btnA.direction = Direction.INPUT
        self.btnA.pull = Pull.UP

        # Button B
        self.btnB = DigitalInOut(board.D6)
        self.btnB.direction = Direction.INPUT
        self.btnB.pull = Pull.UP
        
        # Button C
        self.btnC = DigitalInOut(board.D12)
        self.btnC.direction = Direction.INPUT
        self.btnC.pull = Pull.UP

        # Motor control
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(MOTOR_FWD_PIN, GPIO.OUT)
        GPIO.setup(MOTOR_BWD_PIN, GPIO.OUT)
        self.m_f_pwm = GPIO.PWM(MOTOR_FWD_PIN, 40)
        self.m_b_pwm = GPIO.PWM(MOTOR_BWD_PIN, 40)
        self.m_f_pwm.start(0)
        self.m_b_pwm.start(0)

        GPIO.setup(MOTOR_ENCODER_PIN, GPIO.IN)
        self.encoder_state = GPIO.input(MOTOR_ENCODER_PIN)
        self.rotation_count = 0
        self.stateCount = 0
        self.stateDeadline = None

        # Create the I2C interface.
        i2c = busio.I2C(board.SCL, board.SDA)

        # 128x32 OLED Display
        reset_pin = DigitalInOut(board.D4)
        self.display = adafruit_ssd1306.SSD1306_I2C(128, 32, i2c, reset=reset_pin)

        self.display.fill(0)
        self.display.show()
        self.refresh_display()


        # Radio
        CS = DigitalInOut(board.CE1)
        RESET = DigitalInOut(board.D25)
        spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
        self.radio = adafruit_rfm69.RFM69(spi, CS, RESET, RADIO_FREQUENCY, baudrate=BAUD_RATE)
        self.radio.tx_power = TX_POWER
        self.radio.encryption_key = b'\x01\x02\x03\x04\x05\x06\x07\x08\x01\x02\x03\x04\x05\x06\x07\x08'

        # Servo
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        self.servo = GPIO.PWM(SERVO_PIN, 50)
        self.servo.start(0)
        self.set_servo(CENTER_ANGLE)

    def refresh_display(self):
        self.display.fill(0)
        self.display.text(
                f"G: {self.gear.value} T: {self.turn.value}" +
                f"\nPKTS_RCVD: {self.num_packets}" +
                f"\nTEM: {self.temperature} HUM: {self.humidity}" + 
                f"\nRSSI: {self.last_rssi}",
                0, 0, 1
            )
        self.display.show()

    def read_sensor(self):
        if not self.sensor:
            return
        try:
            self.temperature, self.humidity = self.sensor.temperature, self.sensor.humidity
        except RuntimeError or OverflowError as e:
            pass

    def read_motor_encoder(self):
        new_encoder_state = GPIO.input(MOTOR_ENCODER_PIN)
        if new_encoder_state != self.encoder_state:
            self.encoder_state = new_encoder_state
            self.stateCount += 1

    def ping(self):
        if self.robot:
            return
        data = bytes("PING", "utf-8")
        self.send_radio(data)

    def await_ping(self, curr_pings):
        return self.num_packets > curr_pings

    def read_radio(self):
        # Recieve the latest Packet, If there is one.
        packet = self.radio.receive()
        if packet is None:
            return None

        # Is the packet garbled?
        try:
            packet_text = str(packet, "utf-8")
            self.num_packets += 1
            self.last_rssi = self.radio.last_rssi
        except UnicodeDecodeError as e:
            print(e)
            return

        # Interpret the Command
        # State change packets for robot
        if self.robot:
            if self.discover_mode:
                pass
            elif packet_text == "GEAR":
                self.change_gear()
            elif packet_text == "TURN":
                self.change_turn()
            elif packet_text == "DISCOVER":
                t = threading.Thread(name="discover", target=self.discover)
                t.start()
            # robot ACKs packet
            s = f"{self.gear.value} {self.turn.value} {self.temperature} {self.humidity}"
            data = bytes(s, "utf-8")
            self.send_radio(data)
            self.refresh_display()

        # ACK packets for controller, updates controller with robot state
        else:
            states = packet_text.split(" ")
            if states[0] == "IDLE":
                self.gear = GearState.IDLE
            if states[0] == "FWD":
                self.gear = GearState.FWD
            if states[0] == "BWD":
                self.gear = GearState.BWD
            if states[1] == "LEFT":
                self.turn = TurnState.LEFT
            if states[1] == "CENTER":
                self.turn = TurnState.CENTER
            if states[1] == "RIGHT":
                self.turn = TurnState.RIGHT
            self.temperature, self.humidity = states[2], states[3]
            self.refresh_display()
        return packet

    def send_radio(self, data):
        self.radio.send(data)

    # State changes
    def change_gear(self):
        if self.gear == GearState.IDLE:
            self.motor_fwd()
        elif self.gear == GearState.FWD:
            self.motor_bwd()
        elif self.gear == GearState.BWD:
            self.motor_idle()

    def change_turn(self):
        if self.turn == TurnState.RIGHT:
            self.turn = TurnState.CENTER
            self.set_servo(CENTER_ANGLE)
        elif self.turn == TurnState.CENTER:
            self.turn = TurnState.RIGHT
            self.set_servo(RIGHT_ANGLE)

    # Movement
    def motor_idle(self):
        self.gear = GearState.IDLE
        self.m_f_pwm.stop()
        self.m_b_pwm.stop()

    def motor_fwd(self, duty=75):
        self.gear = GearState.FWD
        self.m_f_pwm.start(duty)
        self.m_b_pwm.stop()

    def motor_bwd(self, duty=75):
        self.gear = GearState.BWD
        self.m_f_pwm.stop()
        self.m_b_pwm.start(duty)

    def motor_encoder_move(self, rotations=1.5, duty=75):
        self.stateDeadline = self.stateCount + rotations * ROTATION_ENCODINGS
        while self.stateDeadline and self.stateCount <= self.stateDeadline:
            print("moving")
            self.motor_fwd(duty)
        self.gear = GearState.IDLE
        self.motor_idle()
        self.stateDeadline = None

    def set_servo(self, angle):
        GPIO.output(SERVO_PIN, False)
        self.servo.ChangeDutyCycle(0)
        time.sleep(.1)
        duty = angle / 18 + 2
        GPIO.output(SERVO_PIN, True)
        self.servo.ChangeDutyCycle(duty)
        #time.sleep(1)
        #GPIO.output(SERVO_PIN, False)
        #self.servo.ChangeDutyCycle(0)

    def discover(self):
        self.discover_mode = True
        self.turn = TurnState.RIGHT
        self.set_servo(RIGHT_ANGLE)
        self.refresh_display()
        
        max_seen = -1000
        max_step = 0
        rssi_vals = []
        for step in range(8):
            print("step: " + str(step))
            curr_pings = self.num_packets
            print(curr_pings)
            while not self.await_ping(curr_pings):
                print(self.num_packets)
                time.sleep(1.0)
            rssi_vals.append(self.radio.last_rssi)
            if (self.radio.last_rssi > max_seen):
                max_step = step
            time.sleep(.6)
            self.motor_encoder_move(rotations=2.25, duty=25)
        time.sleep(1)
        self.motor_encoder_move(rotations=2.25 * max_step, duty=25)
        self.discover_mode = False

    # Buttons
    def buttonA(self):
        if self.robot:
            self.change_gear()
            return
        data = bytes("GEAR", "utf-8")
        self.send_radio(data)
    def buttonB(self):
        if self.robot:
            self.change_turn()
            return
        data = bytes("TURN", "utf-8")
        self.send_radio(data)
    def buttonC(self):
        if self.robot:
            t = threading.Thread(name="discover", target=self.discover)
            t.start()
            return
        data = bytes("DISCOVER", "utf-8")
        self.send_radio(data)

r = Robot(robot=(len(sys.argv) < 2))

def read_lora():
    while True:
        if not r.btnA.value:
            r.buttonA()
        if not r.btnB.value:
            r.buttonB()
        if not r.btnC.value:
            r.buttonC()
        r.read_radio()
        time.sleep(0.1)

def read_sensor():
    while True:
        r.read_sensor()
        time.sleep(0.5)

def read_motor():
    while True:
        r.read_motor_encoder()

def ping():
    while True:
        r.ping()
        time.sleep(1.0)

def main():
    controller_tasks = [read_lora, ping]
    robot_tasks = [read_lora, read_motor, read_sensor]
    tasks = robot_tasks if r.robot else controller_tasks
    for task in tasks:
        t = threading.Thread(name=task.__name__, target=task)
        t.start()
main()
