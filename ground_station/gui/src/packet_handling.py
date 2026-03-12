from ctypes import *
from threading import Lock
import time
from collections import namedtuple

#changed to ctypes as its far more readable. and less confusing to implment

class VehicleState(c_uint8):
    VEHICLE_INIT    = 0x00
    VEHICLE_ARMED   = 0x01
    VEHICLE_FLIGHT  = 0x02
    VEHICLE_LANDING = 0x03
    VEHICLE_ERROR   = 0xFF

class TelemetryHeader(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("sync_word", c_uint32),
        ("version", c_uint8),
        ("msg_type", c_uint8),
        ("sequence", c_uint16),
        ("payload_length", c_uint16),
    ]


class AttitudeData(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("roll", c_float),
        ("pitch", c_float),
        ("yaw", c_float),
    ]


class PositionData(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("x", c_float),
        ("y", c_float),
        ("z", c_float),
    ]


class VelocityData(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("vx", c_float),
        ("vy", c_float),
        ("vz", c_float),
    ]


class StatusData(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("state", VehicleState),
        ("uptime", c_uint32),
        ("battery_percent", c_uint8),
    ]


class ErrorData(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [


        ("code", c_uint8),
        ("message", c_char * 32),
    ]


class RotorState(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("angular_velocity", c_float),
    ]


class TelemetryPayload(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("attitude", AttitudeData),
        ("position", PositionData),
        ("velocity", VelocityData),
        ("status", StatusData),
        ("error", ErrorData),
        ("crc32", c_uint32),
    ]


class TelemetryPacket(LittleEndianStructure):
    _pack_ = 1
    _fields_ = [
        ("header", TelemetryHeader),
        ("payload", TelemetryPayload),
    ]

data_with_time = namedtuple('data_with_time', ['data', 'time'])

class telemetry_latest_packet:

    def __init__(self):
        self.packet_lock = Lock()
        self.latest_parsed_data = None
        self.last_time_stamp = 0
    

    def insert(self,data):
        try:
            self.packet_lock.acquire()
            print(f"{len(data)} expected length = {sizeof(TelemetryPacket)}")
            if len(data) >= sizeof(TelemetryPacket):
                self.latest_parsed_data = TelemetryPacket.from_buffer_copy(data)

        except:
            pass
        finally:
            self.packet_lock.release()


    def get(self):
        with self.packet_lock:
            if self.latest_parsed_data is not None:
                self.last_time_stamp = time.time()
                packet = data_with_time(self.latest_parsed_data, self.last_time_stamp)
                self.latest_parsed_data = None
                return packet
            else:
                packet = data_with_time(None, self.last_time_stamp)
                return packet
