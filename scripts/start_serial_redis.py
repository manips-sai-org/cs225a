import serial
import sys
import redis
BAUD = 115200

if len(sys.argv) == 1:
	print("ERROR: Please enter a name")
	sys.exit()

name = sys.argv[1]
names = {"chase": "/dev/cu.usbserial-0001"}
if not name in names:
	print("ERROR: Please enter one of the following names", names.keys())
port = names[name]
def main():
	r = redis.Redis(
    host='localhost',
    port=6379,
    decode_responses=True)
	
	ser = serial.Serial(port, 115200)
	while True:	
		RAW = ser.readline().decode("utf-8")
		text = RAW.replace("\r\n", "")
		value = int(text)
		print(text=="23")
		r.publish("button", str(text=="23"))
		#text = RAW.replace("")
		#print(ser.readline())


if __name__ == "__main__":
	main()
