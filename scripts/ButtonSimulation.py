import redis
import time

# Setup Redis connection
r = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Simulate button press
while True:
    # Set to 'true' to simulate the button press
    r.set("sai2::sim::panda_bat::dynamic_object::ball_button", "true")
    print("Simulated button press - set to 'true'")
    time.sleep(3)  # Wait for two seconds

    # Set back to 'false'
    r.set("sai2::sim::panda_bat::dynamic_object::ball_button", "false")
    print("Simulated button state - set to 'false'")
    time.sleep(3)  # Wait for two seconds