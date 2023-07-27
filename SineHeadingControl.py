from pymavlink import mavutil
import sys
import signal
from pid import PID
import numpy as np
import math

def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:a
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values
    )


def set_rotation_power(mav, power=0):
    """Set rotation power
    Args:
        power (int, optional): Power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range.")
        power = np.clip(power, -100, 100)

    power = int(round(power,0))# round b/c int casting always rounds down, decreasing accuracy
    print (f"power: {power}")
    set_rc_channel_pwm(mav, 4, 1500 + power * 5)
    #set_rc_channel_pwm(mav, 4, 1550)


def main():
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    # catch CTRL+C
    def signal_handler(sig, frame):
        print("CTRL+C pressed. Disarming")
        mav.arducopter_disarm()
        mav.motors_disarmed_wait()
        print("Disarmed")
        sys.exit(0)

    # catch CTRL+C
    signal.signal(signal.SIGINT, signal_handler)

    # wait for the heartbeat message to find the system ID
    mav.wait_heartbeat()


    # ask user for depth
    desired_heading_deg = float(input("Enter target heading: "))
    
    # arm the vehicle
    print("Arming")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    set_rotation_power(mav, 0)

    print("Armed")

    # set mode to MANUAL
    print("Setting mode to MANUAL")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        19,  # Manual mode
    )
    print("Mode set to MANUAL")

    # TODO: convert heading to radians
    desired_heading = math.radians(desired_heading_deg) #% math.pi*2

    pid = PID(100, 25, -10, 4)
    #print("Heading: ", np.rad2deg(yaw))
#print("prevYaw: ", np.rad2deg(previousYaw))
    while True:
        # get yaw from the vehicle
        msg = mav.recv_match(type="ATTITUDE", blocking=True)
        yaw = msg.yaw
        yaw_rate = msg.yawspeed
        
        if yaw < 0: # convert the 0 to 180 to -180 to -0 mapping into a  0 to 360 mapping
            yaw = yaw + math.pi*2


        print("Heading: ", np.rad2deg(yaw))
        
        
        # calculate error
        error = desired_heading - yaw
        print("Error pre processing: ", np.rad2deg(error))
        #these if statements determine the real way that the robo should go, regardless of heading
        # ... aka getting the true distance from the target '
        # if error > math.pi: 
        #     error = (math.pi * 2 - error) * -1# trying to get there from the far side, so swap to the close size
        # if error < -math.pi:
        #     error = math.pi* 2 + error# same thing but for other side
        #error = error % math.pi*2
        if error > math.pi: 
            error = (math.pi - (error % math.pi) ) * -1# trying to get there from the far side, so swap to the close size
        if error < -math.pi:
            error = error % math.pi 
            #error = math.pi* 2 + error# same thing but for other side
        print("Error post processing: ", np.rad2deg(error))

        output = pid.update(error, error_derivative=yaw_rate)
        print("Output: ", output)

        # set vertical power
        set_rotation_power(mav, output)


if __name__ == "__main__":
    main()
