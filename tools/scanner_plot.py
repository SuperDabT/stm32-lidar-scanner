import serial

ser=  serial.Serial('COM3',115200,timeout=1)

while True: 
    line= ser.readline().decode('utf-8',errors='ignore').strip()
    if not line:
        continue
    parts = line.split(',')
    if len(parts) != 5:
        continue
    pan, tilt, dist, temp, valid = (int(p) for p in parts)
    print(pan, tilt, dist, valid)