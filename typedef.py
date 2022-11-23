from enum import Enum
from PyQt5.QtCore import QByteArray
import struct

from numpy import int32, uint, uint16, uint32, uint8, size


class constVariable:
    pulse_per_degree_S = 34816 / 30
    pulse_per_degree_L = 102400 / 90
    pulse_per_degree_U = 51200 / 90
    pulse_per_degree_RBT = 10204 / 30
    error_trigger = 5
    CartesianPos  = [0,0,0,0,0,0]
    PulsePos      = [0,0,0,0,0,0]
    B022          = 0

class receiveType(Enum):
    ON_SERVO = 0x00
    OFF_SERVO = 0x01
    GET_POSITION = 0x02
    GET_PULSE = 0x03
    WRITE_POSITION = 0x04
    WRITE_PUSLE = 0x05
    SELECT_JOB = 0x06
    START_JOB = 0x07
    HOME_POS = 0x08
    SAVE_FILE = 0x09
    LOAD_FILE = 0x0A
    DELETE_FILE = 0x0B
    GET_VARPOS = 0x0C
    WRITE_VARPOS = 0x0D
    GET_BYTE = 0x0E
    WRITE_BYTE = 0x0F


class Joint_position(Enum):
    NA = -1
    S_joint: float
    L_joint: float
    U_joint: float
    R_joint: float
    B_joint: float
    T_joint: float


class tableColumn(Enum):
    NA = -1
    STT = 0
    S_col = 1
    L_col = 2
    U_col = 3
    R_col = 4
    B_col = 5
    T_col = 6
class tableColumn_2(Enum):
    NA = -1
    STT = 0
    X_col = 1
    Y_col = 2
    Z_col = 3
    Roll_col = 4
    Pitch_col = 5
    Yaw_col = 6


class txHeader:
    header_size: uint16 = 32
    data_size: uint16 = 0
    reserve1:uint8 = 3
    processing_division: uint8 = 1
    ack: uint8 = 0
    id: uint8 = 0
    command_no: uint16 = 0
    instance: uint16 = 0
    attribute: uint8 = 0
    service: uint8 = 0
    padding: uint16 = 0

    def returnByteArray():
        buffer = QByteArray()
        buffer.append(
            QByteArray(
                struct.pack(
                    "<4s2H4BI8s2H2BH",
                    b"YERC",
                    txHeader.header_size,
                    txHeader.data_size,
                    txHeader.reserve1,
                    txHeader.processing_division,
                    txHeader.ack,
                    txHeader.id,
                    0,
                    b"99999999",
                    txHeader.command_no,
                    txHeader.instance,
                    txHeader.attribute,
                    txHeader.service,
                    txHeader.padding,
                )
            )
        )
        return buffer


class txPosition:
    control_group_robot: uint32 = 1
    control_group_station: uint32 = 0
    classification_in_speed: uint32 = 0
    speed: uint32 = 0
    coordinate: uint32 = 0x10
    x: int32 = 0
    y: int32 = 0
    z: int32 = 0
    Roll: int32 = 0
    Pitch: int32 = 0
    Yaw: int32 = 0
    reservation1: uint32 = 0
    reservation2: uint32 = 0
    type: uint32 = 0
    expanded_type: uint32 = 0
    tool_no: uint32 = 0
    user_coordinate_no: uint32 = 0
    base_1_position: uint32 = 0
    base_2_position: uint32 = 0
    base_3_position: uint32 = 0
    station_1_position: uint32 = 0
    station_2_position: uint32 = 0
    station_3_position: uint32 = 0
    station_4_position: uint32 = 0
    station_5_position: uint32 = 0
    station_6_position: uint32 = 0
    size = 104

    def returnByteArray():
        buffer = QByteArray()
        buffer.append(
            QByteArray(
                struct.pack(
                    "<5I6i15I",
                    txPosition.control_group_robot,
                    txPosition.control_group_station,
                    txPosition.classification_in_speed,
                    txPosition.speed,
                    txPosition.coordinate,
                    txPosition.x,
                    txPosition.y,
                    txPosition.z,
                    txPosition.Roll,
                    txPosition.Pitch,
                    txPosition.Yaw,
                    txPosition.reservation1,
                    txPosition.reservation2,
                    txPosition.type,
                    txPosition.expanded_type,
                    txPosition.tool_no,
                    txPosition.user_coordinate_no,
                    txPosition.base_1_position,
                    txPosition.base_2_position,
                    txPosition.base_3_position,
                    txPosition.station_1_position,
                    txPosition.station_2_position,
                    txPosition.station_3_position,
                    txPosition.station_4_position,
                    txPosition.station_5_position,
                    txPosition.station_6_position,
                )
            )
        )
        return buffer


class txPulse:
    control_group_robot: uint32 = 1
    control_group_station: uint32 = 0
    classification_in_speed: uint32 = 0
    speed: uint32 = 0
    r1: int32 = 0
    r2: int32 = 0
    r3: int32 = 0
    r4: int32 = 0
    r5: int32 = 0
    r6: int32 = 0
    r7: int32 = 0
    r8: int32 = 0
    tool_no: uint32 = 0
    base_1_position: uint32 = 0
    base_2_position: uint32 = 0
    base_3_position: uint32 = 0
    station_1_position: uint32 = 0
    station_2_position: uint32 = 0
    station_3_position: uint32 = 0
    station_4_position: uint32 = 0
    station_5_position: uint32 = 0
    station_6_position: uint32 = 0
    size = 88

    def returnByteArray():
        buffer = QByteArray()
        buffer.append(
            QByteArray(
                struct.pack(
                    "<4I8i10I",
                    txPulse.control_group_robot,
                    txPulse.control_group_station,
                    txPulse.classification_in_speed,
                    txPulse.speed,
                    txPulse.r1,
                    txPulse.r2,
                    txPulse.r3,
                    txPulse.r4,
                    txPulse.r5,
                    txPulse.r6,
                    txPulse.r7,
                    txPulse.r8,
                    txPulse.tool_no,
                    txPulse.base_1_position,
                    txPulse.base_2_position,
                    txPulse.base_3_position,
                    txPulse.station_1_position,
                    txPulse.station_2_position,
                    txPulse.station_3_position,
                    txPulse.station_4_position,
                    txPulse.station_5_position,
                    txPulse.station_6_position,
                )
            )
        )
        return buffer


class txVariablePosition:
    data_type: uint32 = 0
    figure: uint32 = 0
    tool_no: uint32 = 0
    user_coordinate_no: uint32 = 0
    extended_figure: uint32 = 0
    first_axis_position: int32 = 0
    second_axis_position: int32 = 0
    third_axis_position: int32 = 0
    fourth_axis_position: int32 = 0
    fifth_axis_position: int32 = 0
    sixth_axis_position: int32 = 0
    seventh_axis_position: int32 = 0
    eighth_axis_position: int32 = 0
    size = 52

    def returnByteArray():
        buffer = QByteArray()
        buffer.append(
            QByteArray(
                struct.pack(
                    "<5I8i",
                    txVariablePosition.data_type,
                    txVariablePosition.figure,
                    txVariablePosition.tool_no,
                    txVariablePosition.user_coordinate_no,
                    txVariablePosition.extended_figure,
                    txVariablePosition.first_axis_position,
                    txVariablePosition.second_axis_position,
                    txVariablePosition.third_axis_position,
                    txVariablePosition.fourth_axis_position,
                    txVariablePosition.fifth_axis_position,
                    txVariablePosition.sixth_axis_position,
                    txVariablePosition.seventh_axis_position,
                    txVariablePosition.eighth_axis_position,
                )
            )
        )
        return buffer


class convertNameJob:
    job_name:str = ""
    def returnByteArray():
        list_name= []
        list_name[:0] = convertNameJob.job_name
        buffer = QByteArray()
        for i in range(size(list_name)):
            buffer.append(QByteArray(struct.pack("c",bytes(list_name[i], 'UTF-8'))))
        for i in range(size(list_name),32):
            buffer.append(QByteArray(struct.pack("B",0)))
        return buffer
# Use to check current flag to pick object
class flag:
    flag_kokomi   = 0       # Flag_kokomi 
    flag_cungdinh = 0       # Flag_cungdinh
    flag_haohao   = 0       # Flag_haohao
    flag_omachi   = 0       # Flag_omachi
    flag_bistro   = 0       # Flag_bistro
    name          = ''      # Object name
    flag_setName  = []      # Show object name
    
class init_pos:    
    P101 = [-125*1000, -220*1000, -120*1000, -180*10000, 0, 0]
    P102 = [-40 *1000, -220*1000, -120*1000, -180*10000, 0, 0]
    P103 = [ 40 *1000, -220*1000, -120*1000, -180*10000, 0, 0]
    P104 = [-10 *1000, -315*1000, -120*1000, -180*10000, 0, 0]
    P105 = [-90 *1000, -315*1000, -120*1000, -180*10000, 0, 0]
    P110 = [180 *1000, -130*1000,  10 *1000, -180*10000, 0, 0]
    P121 = [250 *1000, -195*1000, -120*1000, -180*10000, 0, 0]
    
class conveyor:
    speed = 0

class Byte:
    B022 = 0
class CountObject:
    cung_dinh = 0
    hao_hao   = 0
    kokomi    = 0
    bistro    = 0
    omachi    = 0
    queue     = []
    
class Queue:
    def __init__(self):
        self.queue = []
        
    def enqueue(self,item):
        self.queue.append(item)
    
    def dequeue(self):
        if len(self.queue) < 1:
            return None
        return self.queue.pop(0)
    
    def display(self):
        print(self.queue)
     
    def size(self):
        return len(self.queue)
        
def dequeue(queue):
    if len(queue) < 1:
        return None
    return queue.pop(0)
