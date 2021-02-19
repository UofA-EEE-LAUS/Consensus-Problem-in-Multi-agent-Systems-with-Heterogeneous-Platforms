# start_up

import logging
import math
import sys
import time
from threading import Timer



import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander




logging.basicConfig(level=logging.INFO)
URI = 'radio://0/80/2M/E7E7E7E7E7'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)



#main
if __name__ == '__main__':
     # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        with PositionHlCommander(scf) as pc:
        # We take off when the commander is created
            print('Taking off!')
            time.sleep(3)
            while 1: 
                x = float(input(" enter x position: "))
                y = float(input(" enter y position: "))
                z = float(input(" enter z position: "))
                
                pc.go_to(x,y,z)
                coordinate = [x,y,z]
                print(" move to ",coordinate)
                current_position=pc.get_position()
                print("current position is :",current_position)
                time.sleep(3)
                c = int(input("would you like to continue? (enter 0/1)"))
                if c == 1:
                    print("Please enter location:")
                else:
                    print("Perpare to land!")
                    break
    
            print('Landing!')
