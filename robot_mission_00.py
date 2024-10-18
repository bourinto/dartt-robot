import os
import sys
# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__),'../git')) 
try:
    import dartv2_drivers_v3.drivers_v3 as drv
except:
    print ('could not import dartv2_drivers_v3.drivers_v3, try adding .. to sys.path')
    sys.path.append(os.path.join(os.path.dirname(__file__),'..'))
import dartv2_control as control
import time

if __name__ == "__main__":
    mybot = drv.DartV2DriverV3()
    spd = 60
    duration = 2.0

    # method 1 : test directly the bot
    encleft0, encright0 = mybot.powerboard.get_front_encoders()
    time0= time.time()
    mybot.powerboard.set_speed (spd,spd)
    time.sleep(duration)
    mybot.powerboard.set_speed (0,0)
    time1= time.time()
    encleft1, encright1 = mybot.powerboard.get_front_encoders()

    print ("delta time:",time1-time0)
    print ("delta encoders left:",encleft1-encleft0)
    print ("delta encoders right:",encright1-encright0)

    # wait for the robot to stop
    time.sleep(duration)
    
    # meth2 : test the bot using functions from the control module
    mycontrol = control.DartV2Control(mybot) # create the control object
    mycontrol.get_front_encoders_now_and_last (init=True) # get the front encoders fisrt time
    time0= time.time()
    mybot.powerboard.set_speed (spd,spd)
    time.sleep(duration)
    mybot.powerboard.set_speed (0,0)
    time1= time.time()
    # get the front encoders now and last values
    encleft0, encright0, time0, encleft1, encright1, time1 = mycontrol.get_front_encoders_now_and_last ()
    print ("delta time:",time1-time0)
    print ("delta encoders left:",encleft1-encleft0)
    print ("delta encoders right:",encright1-encright0)
    
    mybot.end() # clean end of the robot mission

