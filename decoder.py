import struct
import pandas as pd
from enum import Enum


class Sensor(Enum):
    HIGH_G_ACCELEROMETER = 0x00
    IMU = 0x01
    MAGNETOMETER = 0x02
    BAROMETER = 0x03
    GPS_MODULE = 0x04
    PROCESSED_DATA = 0x05
    OTHER_SENSOR = 0x06

class SensorData(Enum): # Intended to be generic so "ACCELEROMETER_X" could be high-g or low-g
    # Raw movement
    ACCELEROMETER_X = 0x00
    ACCELEROMETER_Y = 0x01
    ACCELEROMETER_Z = 0x02
    MAGNETOMETER_X = 0x03
    MAGNETOMETER_Y = 0x04
    MAGNETOMETER_Z = 0x05
    GYROSCOPE_X = 0x06
    GYROSCOPE_Y = 0x07
    GYROSCOPE_Z = 0x08

    # Air
    PRESSURE = 0x10
    TEMPERATURE = 0x11

    # Time
    HOUR = 0x20
    MINUTE = 0x21
    SECOND = 0x22
    MILLISECOND = 0x23

    # GPS
    GPS_FIX = 0x30
    GPS_QUALITY = 0x31
    SATELLITES = 0x32

    # Rocket state
    ROCKET_STATE = 0x40
    BATTERY_VOLTAGE = 0x41
    SENSORS_DETECTED = 0x42
    SENSOR_DATA_AVAILABLE = 0x43

    # Angle
    HEADING = 0x50
    ANGLE = 0x50 # Alias for heading
    QUATERNION_1 = 0x51
    QUATERNION_I = 0x52
    QUATERNION_J = 0x53
    QUATERNION_K = 0x54

    # Location and ground-frame data
    LATITUDE = 0x60
    LONGITUDE = 0x61
    SPEED = 0x62
    ACCELERATION_X = 0x63
    ACCELERATION_Y = 0x64
    ACCELERATION_Z = 0x65
    VELOCITY_X = 0x66
    VELOCITY_Y = 0x67
    VELOCITY_Z = 0x68
    POSITION_X = 0x69
    POSITION_Y = 0x6A
    POSITION_Z = 0x6B
    ALTITUDE = 0x6B # Altitude is Z position

def calculateChecksum(p):
    ptr = bytearray(p)
    sum = 0
    # Iterate over size of struct minus the checksum byte itself
    for i in range(len(ptr) - 1):
        sum ^= ptr[i]
    return sum

# Define the format string based on your C++ struct
# < = little endian
# I = uint32 (4 bytes)
# I = uint32 (4 bytes)
# B = uint8 (1 byte)
# B = uint8 (1 byte)
# f = float (4 bytes)
# B = uint8 (1 byte - checksum)
STRUCT_FMT = '<IIBBfB'
STRUCT_SIZE = struct.calcsize(STRUCT_FMT)

data_list = []

with open('/run/media/bensimmons/8214-BC9F/log06.bin', 'rb') as f:
    endian = "little"
    version = f.read(1)
    endianRaw = f.read(1)
    if endianRaw == 0x0.to_bytes(1, 'little'):
        endian = "little"
        STRUCT_FMT = "<" + STRUCT_FMT[1:]
    else:
        endian = "big"
        STRUCT_FMT = ">" + STRUCT_FMT[1:]
    if version != 0x1.to_bytes(1, endian):
        print("Unsupported version")
        exit(1)
    size = f.read(2)
    if int.from_bytes(size, endian) != STRUCT_SIZE:
        print("Unexpected struct size")
        exit(1)

    while True:
        bytes_read = f.read(STRUCT_SIZE)
        if len(bytes_read) < STRUCT_SIZE:
            break # End of file

        # Unpack binary data to tuple
        unpacked = struct.unpack(STRUCT_FMT, bytes_read)

        # Verify checksum
        micros, millis, sensor_id, data_id, value, checksum = unpacked
        calculated_checksum = calculateChecksum(bytes_read)
        if calculated_checksum != checksum:
            print(f"Checksum mismatch: calculated {calculated_checksum}, expected {checksum}")
            continue # Skip this record

        data_list.append(unpacked)

columns = ['micros', 'millis', 'sensor_id', 'data_id', 'value', 'checksum']
df = pd.DataFrame(data_list, columns=columns)

# map sensor_id and data_id to their names
df['sensor_name'] = df['sensor_id'].map(lambda x: Sensor(x).name)
df['data_name'] = df['data_id'].map(lambda x: SensorData(x).name)

# filter to only GPS data
# df = df[df['sensor_name'] == 'GPS_MODULE']
# df = df[df['data_name'] == 'SPEED']
# df = df[df['sensor_name'] == 'HIGH_G_ACCELEROMETER']
# df = df[df['data_name'] == 'ACCELEROMETER_X']

# add millis delta column (time between readings)
df['millis_delta'] = df['millis'].diff().fillna(0)
df['micros_delta'] = df['micros'].diff().fillna(0)
# print average millis delta
# avg_millis_delta = df['millis_delta'].mean()
# print(f"Average millis delta: {avg_millis_delta}")

print(df.head(20))
df.to_csv("flight_data.csv", index=False) # Save as CSV if needed
