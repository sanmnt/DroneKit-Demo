import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil


# - Importing Tkinter: sudo apt-get install python-tk
from tkinter import *


# -- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')

# -- Setup the commanded flying speed
gnd_speed = 5  # [m/s]


# -- Define arm and takeoff
def arm_and_takeoff(altitude):
    while not vehicle.is_armable:
        print("waiting to be armable")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed: time.sleep(1)

    print("Taking Off")
    vehicle.simple_takeoff(altitude)

    while True:
        v_alt = vehicle.location.global_relative_frame.alt
        print(">> Altitude = %.1f m" % v_alt)
        if v_alt >= altitude - 1.0:
            print("Target altitude reached")
            break
        time.sleep(1)


# -- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

    Bitmask to indicate which dimensions should be ignored by the vehicle
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that
    none of the setpoint dimensions should be ignored). Mapping:
    bit 1: x,  bit 2: y,  bit 3: z,
    bit 4: vx, bit 5: vy, bit 6: vz,
    bit 7: ax, bit 8: ay, bit 9:


    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # -- BITMASK -> Consider only the velocities
        0, 0, 0,  # -- POSITION
        vx, vy, vz,  # -- VELOCITY
        0, 0, 0,  # -- ACCELERATIONS
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()


a = []


# -- Key event function

#def moveLeft():
    # Call work function
    



def printInput():
    global inp, inp2, inp3
    inp = inputtxt.get(1.0, "end-1c")
    inp2 = inputtxt2.get(1.0, "end-1c")
    inp3 = inputtxt3.get(1.0, "end-1c")
    a.append(inp2)
    plotPoints(inp, inp2, inp3)
    print("Vehicle's Latitude              =  ", vehicle.location.global_relative_frame.lat)
    print("Vehicle's Longitude             =  ", vehicle.location.global_relative_frame.lon)
    print("Vehicle's Altitude (in meters)  =  ", vehicle.location.global_relative_frame.alt)
    


def plotPoints(inp, inp2, inp3):
    location = LocationGlobalRelative(float(inp), float(inp2), float(inp3))
    vehicle.simple_goto(location)
    if inp2 == vehicle.location.global_relative_frame.lon:
        a.pop(0)



# if event.char == event.keysym: #-- standard keys

# else: #-- non standard keys

# ---- MAIN FUNCTION
# - Takeoff
arm_and_takeoff(10)

# Printing Vehicle's Latitude
print("Vehicle's Latitude              =  ", vehicle.location.global_relative_frame.lat)

# Printing Vehicle's Longitude
print("Vehicle's Longitude             =  ", vehicle.location.global_relative_frame.lon)

# Printing Vehicle's Altitude
print("Vehicle's Altitude (in meters)  =  ", vehicle.location.global_relative_frame.alt)

# - Read the keyboard with tkinter
root = Tk()
root.geometry('400x400')
inputtxt = Text(root, height=2, width=10)
inputtxt2 = Text(root, height=2, width=10)
inputtxt3 = Text(root, height=2, width=10)
inputtxt.pack()
inputtxt2.pack()
inputtxt3.pack()
printButton = Button(root, text="set coordinates", command=printInput)
printButton.pack()

#Button(root,text="Move Left",command = moveLeft).pack()
root.mainloop()
