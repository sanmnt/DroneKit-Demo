from dronekit import connect, VehicleMode, LocationGlobalRelative
import time

#-- Connect to the vehicle
import argparse
parser = argparse.ArgumentParser(description='command')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect


print("Connection to the vehicle on %s"%connection_string)
vehicle = connect(connection_string, wait_ready=True)

#-- Define the function for takoff
def arm_and_takeoff(tgt_altitude):
    print("Arming motors")

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(tgt_altitude)

    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)

    #-- wait to reach the target altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt

        if altitude >= tgt_altitude -1:
            print("Altitude reached")
            break

        time.sleep(1)


#-----Main program -----
arm_and_takeoff(10)

#---set the default speed
vehicle.airspeed = 7

#--go to wpl
print("go to wpl")
wpl = LocationGlobalRelative(34.05771167944215, -117.82201154434912, 10)

vehicle.simple_goto(wpl)

#-- here you can do all your magic--
time.sleep(30)

#-- Coming
print("Coming back")
vehicle.mode = vehicleMode("RTL")

time.sleep(20)

#-- close connection
vehicle.close()
