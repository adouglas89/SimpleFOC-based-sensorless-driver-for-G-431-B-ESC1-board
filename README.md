# SimpleFOC-based-sensorless-driver-for-G-431-B-ESC1-board
You need to install the simplefoc library before you can use this system, and also the board files or whatever for the B-G431-B-ESC1 board, I think it's called STMduino or something, the whole package, both of these are available through the arduino package manager interface.
Then figure out how to link to the device, then you can upload it. 
You need to send it "O1" over the serial port to get it to start going right now, it starts in a disabled state for safety, in case it rebooted.  I recommend a TTL-USB converter and the arduino serial monitor.
You can read and write various things to the device over serial. You must pad the commands with the right number of characters or it will sit there waiting till the right number of chars has been recieved. Probably it would be better to make it wait for a hard return or something.

This is a repository to produce a silent sensorless motor driver for a gimbal motor using the simpleFOC libraries on a B-G431-b-ESC1 board.  It will accept speed commands by PWM, in the standard way for a reversible fan (50% duty is standing still, 100% is full forward 0% is full backwards).  It is reversible direction and will have rapid acceleration capabilities.
You may or may not argue if it is Field Oriented Control or not.  In reality, most drivers which claim to be are not really FOC anway, and the definition is rather unclear.
This is for a fan, it will not likely work well in other contexts.
The plan is that it will work by using a drive system that controls and ramps frequency and voltage directly according to a set of curves that have been established during a calibration proceedure.  Then, on top of that, a state observer operates to determine the angle between the rotor and the magnetic field produced by the current flowing through the coils (which could be used to infer the rotor-stator angle).  Then, the voltage is regulated by a PID controller to eliminate the excess which leads to overheating, and regulate the angle to near optimal (usually, optimal plus a small safety margin, because if it gets too close it tends to stall).  It will have stall detection and overcurrent protection as well.
I am using the JD power 3151C motor, which is a low cogging motor (ring ceramic magnet, not plate magnets, so SM type magnets, not PM type, however it still gives a sine wave output when freewheeling for some reason).
This is for the TW4 energy recovery ventilator fan.
There will be another board at some point that will run a very similar program, in production.
Note that these exact boards tend to get hot, that's just a quirk of the hardware.
