
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil



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




def plotPoints(inp, inp2, inp3):
    location = LocationGlobalRelative(inp, inp2, inp3)
    vehicle.simple_goto(location)



arm_and_takeoff(10.0)

A = []

with open("input.txt") as fileIn:
    for line in fileIn:
        n1, n2, n3, n4 = (float(s) for s in line.split())

        A.append([n1, n2, n3, n4])
        print(vehicle.location.global_relative_frame.lat)
        if A[0][0] >= vehicle.location.global_relative_frame.lat-.00005 and A[0][0] <= vehicle.location.global_relative_frame.lat+.00005 and A[0][1]>= vehicle.location.global_relative_frame.lon-.00005 and A[0][1]<= vehicle.location.global_relative_frame.lon+.00005:
            print('Destination Reached')
            A.pop(0)
        else:
            A.sort(key=lambda x: x[3])
        plotPoints(A[0][0],A[0][1], A[0][2])
        time.sleep(8)

while len(A)!=0:
    plotPoints(A[0][0], A[0][1], A[0][2])
    if A[0][0] >= vehicle.location.global_relative_frame.lat-.00005 and A[0][0] <= vehicle.location.global_relative_frame.lat+.00005 and A[0][1]>= vehicle.location.global_relative_frame.lon-.00005 and A[0][1]<= vehicle.location.global_relative_frame.lon+.00005:
        print('Destination Reached')
        A.pop(0)

    time.sleep(3)


