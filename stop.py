import os
import sys
# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__),'../git')) 
try:
    import dartv2_drivers_v3.drivers_v3 as drv
except:
    print ('could not import dartv2_drivers_v3.drivers_v3, try adding .. to sys.path')
    sys.path.append(os.path.join(os.path.dirname(__file__),'..'))
import dartv2_drivers_v3.drivers_v3 as drv
import time

if __name__ == "__main__":
    mybot = drv.DartV2DriverV3()
 
    # set speed to 0
    spdl = 0
    spdr = 0
    mybot.powerboard.set_speed (spdl, spdr)
    time.sleep (0.05)

    # check battery level
    print ("Battery Voltage : %.2f V"%(mybot.encoders.battery_voltage()))
   
    mybot.end() # clean end of the robot mission

