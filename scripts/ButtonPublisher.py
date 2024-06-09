import serial
import redis

# Setup serial port
serial_port = serial.Serial(
    port="COM6",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

# Setup Redis connection
redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)
redis_key = "sai2::sim::panda_bat::dynamic_object::ball_button"

# Initialize the key to false
redis_client.set(redis_key, "false")
last_state = "false"

try:
    while True:
        try:
            # Read a line from the serial port
            line = serial_port.readline().decode().strip()
            
            # Check if the line matches "23" and update only if state has changed
            current_state = "true" if line == "23" else "false"
            if current_state != last_state:
                redis_client.set(redis_key, current_state)
                print(f"Button press detected: {current_state}")
                last_state = current_state

        except Exception as e:
            print(f"ERROR: {str(e)}")

except KeyboardInterrupt:
    print("Terminated by user")

finally:
    serial_port.close()
    print("Serial port closed")