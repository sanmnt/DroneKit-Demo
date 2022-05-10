import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk
from tkinter import *


#-- Connect to the vehicle
print('Connecting...')
vehicle = connect('udp:127.0.0.1:14551')

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]

#-- Define arm and takeoff
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
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
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
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
#-- Key event function
def key(event):
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 5:
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("RTL")
            
    else: #-- non standard keys
        if event.keysym == 1:
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 2:
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 3:
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 4:
            set_velocity_body(vehicle, 0, gnd_speed, 0)
    
    

variable = 1
def direction1():
	global variable
	variable = 1
def direction2():
	global variable
	variable = 2
def direction3():
	global variable
	variable = 3
def direction4():
	global variable
	variable = 4
def direction5():
	global variable
	variable = 5

def run(direction):
	while True:
		if(direction == 1):
			key(1)
		elif(direction == 2):
			key(2)
		elif(direction == 3):
			key(3)
		elif(direction == 4):
			key(4)
		elif(direction == 5):
			key(5)
			break

arm_and_takeoff(10)
root = tk.Tk()
root.geometry("400x400")
Button(root, text="Forword", command=direction1).pack()
Button(root, text="Backword", command=direction2).pack()
Button(root, text="Left", command=direction3).pack()
Button(root, text="Right", command=direction4).pack()
Button(root, text="RTL", command=direction5).pack()
#---- MAIN FUNCTION
#- Takeoff
#def main():
	
 	
	#- Read the keyboard with tkinter
	
	#print(">> Control the drone with the arrow keys. Press r for RTL mode")
	
	#root.bind_all('<Key>', key)
print(variable)
root.bind(variable, run)
root.mainloop()
