import serial
import sys

def readFromAVR(serialPort):
    while(1):
        line = serialPort.readline()
        print(line)    

if __name__ == "__main__":
    
    PORT = '/dev/ttyUSB0'
    BAUDRATE =  9600
    TIMEOUT = None
    SCREEN_WIDTH = 80

    ## Take command-line arguments to override defaults above
    if len(sys.argv) == 3:
        port = sys.argv[1]
        baudrate = int(sys.argv[2])
    else:
	    print("Optional arguments port, baudrate set to defaults.")
    
    port, baudrate = (PORT, BAUDRATE)
    serialPort = serial.Serial(port, baudrate, timeout=TIMEOUT)
    serialPort.flush()
    readFromAVR(serialPort)