import struct
from enum import IntEnum
import queue

class msg_type(IntEnum):
    MSG_TELEMETRY_PACKET    = 0x00
    MSG_ATTITUDE            = 0x01
    MSG_POSITION            = 0x02
    MSG_VELOCITY            = 0x03
    MSG_STATUS              = 0x04
    MSG_ERROR               = 0x05
    MSG_ROTOR               = 0x06




class msg_parse:

    def parse_attitude(self, data):
        roll, pitch, yaw = struct.unpack("<fff", data)
        return {
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw
        }

    def parse_position(self, data):
        lat, lon, alt = struct.unpack("<fff", data)
        return {
            "x": lat,
            "y": lon,
            "z": alt
        }

    def parse_velocity(self, data):
        vx, vy, vz = struct.unpack("<fff", data)
        return {
            "vx": vx, "vy": vy, "vz": vz
        }

    def parse_status(self, data):
        
        return {

        }

    def parse_error(self, data):
        code, msg_raw = struct.unpack("<B32s", data)
        message = msg_raw.split(b'\x00', 1)[0].decode('utf-8')
        return {
            "code": code,
            "message": message
        }

    def parse_rotor(self, data):
        (ang_vel,) = struct.unpack("<f", data)
        return {
            "angular_velocity": ang_vel
        }

    def telemetry_packet(self, data):

        Size_ATT   = 12
        Size_POS   = 12
        Size_VEL   = 12
        Size_STAT  = 6 

        offset_ATT   = 0
        offset_POS   = offset_ATT + Size_ATT
        offset_VEL   = offset_POS + Size_POS
        offset_STAT  = offset_VEL + Size_VEL
        offset_ROTOR = offset_STAT + Size_STAT

        attitude_raw = data[offset_ATT : offset_POS]
        position_raw = data[offset_POS : offset_VEL]
        velocity_raw = data[offset_VEL : offset_STAT]
        status_raw   = data[offset_STAT: offset_ROTOR]

        return {
            **self.parse_attitude(attitude_raw),
            **self.parse_position(position_raw),
            **self.parse_velocity(velocity_raw),
            **self.parse_status(status_raw)
        }



def parse_header(data):
    type = struct.unpack('<B' , data[:1])[0]
    return type


def parse_handler(data):
    type = parse_header(data)
    size_of_header = 1
    payload = data[size_of_header:]

    parser = msg_parse()
    
    handlers = {
            msg_type.MSG_ATTITUDE:          parser.parse_attitude,
            msg_type.MSG_POSITION:          parser.parse_position,
            msg_type.MSG_VELOCITY:          parser.parse_velocity,
            msg_type.MSG_STATUS:            parser.parse_status,
            msg_type.MSG_ERROR:             parser.parse_error,
            msg_type.MSG_ROTOR:             parser.parse_rotor,
            msg_type.MSG_TELEMETRY_PACKET:  parser.telemetry_packet
        }

    
    return handlers[type](payload)
    



class telemetry_queue:

    def __init__(self):
        self.que = queue.Queue()
    

    def insert(self,data):
        parsed_data = parse_handler(data)
        self.que.put(parsed_data)


    def get(self):
        merged_dict = {}
        while not self.que.empty():
            data = self.que.get()
            print("DEBUG: got from queue:", data, type(data))
            merged_dict.update(data)

        return merged_dict