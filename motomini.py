import socket
from PyQt5.QtCore import QByteArray
from typedef import *

class Motomini:
    def __init__(self) -> None:
        self.sever_socket = socket.socket()
        # self.sever_socket = QUdpSocket()
        self.connect_status = False
        self.servo_status: bool = False
        self.ip: str = ""
        # self.ip = QHostAddress()
        self.port: int = 0

        self.rx_buffer: QByteArray = QByteArray()
        self.rx_buffer_pulse: QByteArray = QByteArray()
        self.rx_buffer_cartesian: QByteArray = QByteArray()
        self.rx_buffer_byte: QByteArray = QByteArray()

    def connectMotomini(self, ip: str, port: int):
        self.sever_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.ip = ip
        self.port = port
        self.sever_socket.connect((self.ip, self.port))
        # self.sever_socket.bind(self.ip,self.port)
        self.connect_status = True

    def disconnectMotomini(self):
        self.sever_socket.close()
        self.ip = ""
        self.port = 0
        self.connect_status = False

    def checkConnectStatus(self):
        return self.connect_status

    def checkServoStatus(self):
        return self.servo_status

    def sendData(self, buffer: QByteArray):
        self.sever_socket.sendto(buffer, (self.ip, self.port))
        # self.sever_socket.writeDatagram(buffer,self.ip,self.port)

    def receiveData(self):
        self.sever_socket.settimeout(500)
        try:
            self.rx_buffer = self.sever_socket.recv(520)
            if self.rx_buffer[11] == 2:
                self.rx_buffer_cartesian = self.rx_buffer
            elif self.rx_buffer[11] == 3:
                self.rx_buffer_pulse = self.rx_buffer
            elif self.rx_buffer[11] == 14:
                self.rx_buffer_byte = self.rx_buffer
        except socket.timeout:
            pass

    def onServo(self):
        header = txHeader
        header.id = receiveType.ON_SERVO.value
        header.command_no = 0x83
        header.instance = 2
        header.attribute = 1
        header.service = 0x10
        header.data_size = 4

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(QByteArray(struct.pack("<I", 1)))
        self.sendData(buffer=buffer)
        self.receiveData()
        if (self.rx_buffer[11] == 0) & (int.from_bytes(self.rx_buffer[26:28],"big") == 0):
            self.servo_status = True
            return 0
        else:
            return 1

    def offServo(self):
        header = txHeader
        header.id = receiveType.OFF_SERVO.value
        header.command_no = 0x83
        header.instance = 2
        header.attribute = 1
        header.service = 0x10
        header.data_size = 4

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(QByteArray(struct.pack("<I", 2)))
        self.sendData(buffer=buffer)
        self.receiveData()
        if (self.rx_buffer[11] == 1) & (int.from_bytes(self.rx_buffer[26:28],"big") == 0):
            self.servo_status = False
            return 0
        else:
            return 1

    def homeServo(self):
        data = txPulse
        data.r1 = 0
        data.r2 = 0
        data.r3 = 0
        data.r4 = 0
        data.r5 = 0
        data.r6 = 0
        data.speed = 10 * 100

        header = txHeader
        header.id = receiveType.HOME_POS.value
        header.command_no = 0x8B
        header.instance = 1
        header.attribute = 1
        header.service = 0x02
        header.data_size = data.size

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(data.returnByteArray())
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def getCartasianPos(self):
        header = txHeader
        header.id = receiveType.GET_POSITION.value
        header.command_no = 0x75
        header.instance = 0x65
        header.attribute = 0
        header.service = 0x01
        header.data_size = 0

        buffer = QByteArray()
        buffer = header.returnByteArray()
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def moveCartasianPos(self, speed: int32, pos: list):
        data = txPosition
        data.x = pos[0]
        data.y = pos[1]
        data.z = pos[2]
        data.Roll = pos[3]
        data.Pitch = pos[4]
        data.Yaw = pos[5]
        data.speed = speed

        header = txHeader
        header.id = receiveType.WRITE_POSITION.value
        header.command_no = 0x8A
        header.instance = 0x01
        header.attribute = 0x01
        header.service = 0x02
        header.data_size = data.size

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(data.returnByteArray())
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def getPulsePos(self):
        header = txHeader
        header.id = receiveType.GET_PULSE.value
        header.command_no = 0x75
        header.instance = 0x01
        header.attribute = 0
        header.service = 0x01
        header.data_size = 0

        buffer = QByteArray()
        buffer = header.returnByteArray()
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def movePulsePos(self, speed: int32, pos: list):
        data = txPulse
        data.r1 = pos[0]
        data.r2 = pos[1]
        data.r3 = pos[2]
        data.r4 = pos[3]
        data.r5 = pos[4]
        data.r6 = pos[5]
        data.speed = speed

        header = txHeader
        header.id = receiveType.WRITE_PUSLE.value
        header.command_no = 0x8B
        header.instance = 0x01
        header.attribute = 0x01
        header.service = 0x02
        header.data_size = data.size

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(data.returnByteArray())
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def getVariablePos(self, index: uint16):
        header = txHeader
        header.id = receiveType.GET_VARPOS.value
        header.command_no = 0x7F
        header.instance = index
        header.attribute = 0x00
        header.service = 0x0E
        header.data_size = 0

        buffer = QByteArray()
        buffer = header.returnByteArray()
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def writeVariablePos(self, index: uint16, pos: list):
        data = txVariablePosition
        data.data_type = 0x11
        data.first_axis_position = pos[0]
        data.second_axis_position = pos[1]
        data.third_axis_position = pos[2]
        data.fourth_axis_position = pos[3]
        data.fifth_axis_position = pos[4]
        data.sixth_axis_position = pos[5]

        header = txHeader
        header.id = receiveType.WRITE_VARPOS.value
        header.command_no = 0x7F
        header.instance = index
        header.attribute = 0x00
        header.service = 0x02
        header.data_size = data.size

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(data.returnByteArray())
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def getByte(self, index: uint16):
        header = txHeader
        header.id = receiveType.GET_BYTE.value
        header.command_no = 0x7A
        header.instance = index
        header.attribute = 0x01
        header.service = 0x0E
        header.data_size = 0

        buffer = QByteArray()
        buffer = header.returnByteArray()
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def writeByte(self, index: uint16, var: uint8):
        header = txHeader
        header.id = receiveType.WRITE_BYTE.value
        header.command_no = 0x7A
        header.instance = index
        header.attribute = 0x01
        header.service = 0x10
        header.data_size = 1

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(QByteArray(struct.pack("B", var)))
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def selectJob(self, job_name:str, line_no:uint32):
        header = txHeader
        header.id = receiveType.SELECT_JOB.value
        header.command_no = 0x87
        header.instance = 0x01
        header.attribute = 0x00
        header.service = 0x02
        header.data_size = 36

        data = convertNameJob
        data.job_name = job_name
        job_encoded = data.returnByteArray()
        
        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(job_encoded)
        buffer.append(QByteArray(struct.pack("I", line_no)))
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def startJob(self):
        header = txHeader
        header.id = receiveType.START_JOB.value
        header.command_no = 0x86
        header.instance = 0x01
        header.attribute = 0x01
        header.service = 0x10
        header.data_size = 4

        buffer = QByteArray()
        buffer = header.returnByteArray()
        buffer.append(QByteArray(struct.pack("I", 1)))
        self.sendData(buffer=buffer)
        self.receiveData()
        return 0

    def convertPos(self):
        if self.rx_buffer_cartesian != b"":
            for i in range(6):
                constVariable.CartesianPos[i] = int.from_bytes(self.rx_buffer_cartesian[52 + i*4:56 + i*4],"little",signed=True)
        if self.rx_buffer_pulse != b"":
            for i in range(6):
                constVariable.PulsePos[i] = int.from_bytes(self.rx_buffer_pulse[52 + i*4:56 + i*4],"little",signed=True)
        if self.rx_buffer_byte != b"":
            constVariable.B022 = self.rx_buffer_byte[32]
            