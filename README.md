# Final_Tetrafloat_system

USER GUIDE

Step 1: Download Arduino IDE and Processing app Version 2.2.1 (NOT NEWEST VERSION)
        https://www.arduino.cc/en/Main/Software
        https://processing.org/download/?processing
        
Step 2: Copy libraries from the 'Required libraries' folder to the Arduino libraries directory

Step 3: Open latest script MPU-9150 > MPU-9150_PID_Deadband > MPU-9150_PID_Deadband.ino

Step 4: Select the board and port from the tools menu in the Arduino IDE

Step 5: Set the user definable variables to desired values

Step 6: Plug Arduino into USB port and press upload

Step 7: To see values in the serial monitor: 
                Open the serial monitor in the Arduino IDE and set baud rate to 38400.
                Values are as follows: time (milliseconds), Corrected yaw angle (degrees), PID output, Clockwise winch status, Anticlockwise winch status
        
Step 8: To see the values on a realtime graph:
                Open the processing script Graph > Graph.pde.
                Ensure that the Ardunio is plugged in and has the correct code flashed.
                Press run (Angle is shown in black, PID output in red), to close graph and save values to csv file press any button.
                If you get the error 'Error opening serial port' change the number in square brackets.
                in the line of code (usually between 0 and 3): serial = new Serial(this, Serial.list()[1], 38400);

NOTE: If magnetometer needs calibrating refer to this guide to determine the 3 magnetometer offsets  http://www.instructables.com/id/Simple-Manual-Magnetometer-Calibration/?ALLSTEPS

ALSO NOTE: To change the digital outputs change the code between lines 129 and 147
