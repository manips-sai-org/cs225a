import keyboard
import redis

r = redis.StrictRedis()

while True:
    key_pressed = keyboard.read_key()
    if key_pressed == "w":  # forward
        r.set("sai2::up", 1)
        print("Pressed W")
    elif key_pressed == "s":  # backward
        r.set("sai2::down", 1)
        print("Pressed S")
    elif key_pressed == "a":  # left
        r.set("sai2::left", 1)
        print("Pressed A")
    elif key_pressed == "d":  # right
        r.set("sai2::right", 1)        
        print("Pressed D")
    elif key_pressed == "q":  # ccw
        r.set("sai2::forward", 1)
        print("Pressed Q")
    elif key_pressed == "e":  # cw
        r.set("sai2::backward", 1)
        print("Pressed E")
    # elif key_pressed == "c":  # up
    #     r.set("sai2::sim::power::up", 1)
    #     print("Pressed C")
    # elif key_pressed == "v":  # down
    #     r.set("sai2::sim::power::down", 1)
    #     print("Pressed V")