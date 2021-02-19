The Rover Code Test .ino file can be tested directly on the rover, just change the desired x, y and angle. Just connect the Arduino MKR1000 board with USB to a computer and upload the code. Disconnect the USB and take it to the desired workspace. Turn on the switches on the PCB and the rover will move to the desired location with rotational angle. 

The Rover_collision_avoidance .ino file is about collision avoidance function. The rover will stop when the sensor detects the obstacle. However, the rover will receive all the sensor reading that can not identify the sensor’s direction, so the sensor’s ID should be identified for future modify. 

For the rover to run on wifi, firstly connect Arduino MKR1000 via USB and once the code has been uploaded, turn on the serial monitor within the Arduino IDE and scroll all the way to the top. There will be a IP address that must be noted. Disconnect the USB and open the Matlab code wifitest.m. Make sure the IP address mentioned in the Matlab code matches with the IP address noted prior. Run the code and wait about 10s for the rover to connect. Now give inputs as [x, y, angle] as string vector in the Matlab command line for rover actuation.