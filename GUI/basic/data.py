import ctypes

class SensorData(ctypes.Structure):
    # C++의 __attribute__((packed))와 동일한 효과를 냅니다.
    # 바이트 패딩 없이 1바이트 단위로 정렬합니다.
    _pack_ = 1
    
    _fields_ = [
        ("timestamp",     ctypes.c_uint32), # 4 bytes
        ("sensor_update", ctypes.c_uint8),  # 1 byte
        
        ("ax",            ctypes.c_float),  # 4 bytes
        ("ay",            ctypes.c_float),  # 4 bytes
        ("az",            ctypes.c_float),  # 4 bytes
        
        ("gx",            ctypes.c_float),  # 4 bytes
        ("gy",            ctypes.c_float),  # 4 bytes
        ("gz",            ctypes.c_float),  # 4 bytes
        
        ("w",             ctypes.c_float),  # 4 bytes (Quaternion)
        ("x",             ctypes.c_float),  # 4 bytes
        ("y",             ctypes.c_float),  # 4 bytes
        ("z",             ctypes.c_float),  # 4 bytes
        
        ("raw_p",         ctypes.c_float),  # 4 bytes
        ("filt_p",        ctypes.c_float),  # 4 bytes
        
        ("filt_alt",      ctypes.c_float),  # 4 bytes
        ("alt_baro",      ctypes.c_float),  # 4 bytes
        ("alt_imu",       ctypes.c_float),  # 4 bytes
        
        ("filt_velocity", ctypes.c_float),  # 4 bytes
        ("vel_z_imu",     ctypes.c_float),  # 4 bytes
        ("vel_z_baro",    ctypes.c_float),  # 4 bytes
        
        ("flight_state",  ctypes.c_uint8),  # 1 byte
        ("ej1_state",     ctypes.c_uint8),  # 1 byte
        ("ej2_state",     ctypes.c_uint8),  # 1 byte
        ("sep_state",     ctypes.c_uint8),  # 1 byte
    ]

if __name__ == "__main__":
    pass