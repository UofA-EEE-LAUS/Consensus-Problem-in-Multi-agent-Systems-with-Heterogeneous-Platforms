In "Lib Python Change" file, all codes are based on Crazyflie Python Lib Examples. 

------------------------------------

"easy.py" 
This file is going to ask a coordinate from user through terminal, 
and print out the current position after arrived. Then, user can choose 
to land or continue fly to next destination.

"multiranger_push.py" 
This file is from updated Crazyflie Python Lib examples. 
It's using multiranger deck to check surrounding obsticle distance 
in 5 directions front, back, left, right and upside of drone.

"simple.py"
This file is depended on Crazyflie Python Lib examples "sequence.py".
It will follow the desired sequence to fly to the destination. 
In the main function, "duration" means the time of flight to control
drone's flying speed and record the flying distance.

"cflib"
In this file, it has the change of Python high-level command. Please
open the file for more information.