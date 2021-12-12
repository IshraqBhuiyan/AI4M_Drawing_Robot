import serial
import time
ser = serial.Serial('/dev/cu.wchusbserial1410')
time.sleep(2)
print(ser.name)
ser.readline()
ser.readline()

minval=40
maxval = 1024

def write_read(x):
  ser.write(bytes(x))
  time.sleep(0.05)
  data = ser.readline()
  return data

for iter in range(100):
  data=write_read(b'read\n')
  desY=60.0 * ((float(int(data)-minval)/float(maxval-minval))-0.5)
  print(data, desY)

  data = write_read(b'goto 0, '+str(desY).encode('ascii')+b',-45\n')
  time.sleep(0.02)

ser.close()
