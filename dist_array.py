# Code to display an array of distance vs degrees
# Change the value of port accordingly

import PyLidar3
import time # Time module
port = "/dev/ttyACM0" #linux
Obj = PyLidar3.YdLidarG4(port)  

if(Obj.Connect()):
    print(Obj.GetDeviceInfo())
    print("b")
    gen = Obj.StartScanning()
    t = time.time() # start time 
    while (time.time() - t) < 30: #scan for 30 seconds
        print(next(gen))
        print("c")
        time.sleep(0.5)
    Obj.StopScanning()
    Obj.Disconnect()
else:
    print("Error connecting to device")
