import numpy as np

counter = 0
stuck_change_counter = 0
stuck_sign = 1
found_counter = 0
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    global counter
    global stuck_change_counter
    global stuck_sign
    global found_counter

    print(Rover.near_sample)
    print(Rover.mode)
    print(Rover.send_pickup)
    
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            found_counter = 0
            if len(Rover.sample_angles) > 0:
                Rover.brake = Rover.brake_set
                Rover.throttle = 0
                Rover.mode = "found"

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                dir_angle = np.percentile(Rover.nav_angles  * 180/np.pi, np.random.randint(20, 40))
                if np.random.randint(100) % 4 == 0:
                    dir_angle = 0

                if np.random.randint(100) % 10 == 0:
                    dir_angle = Rover.nav_angles[np.argmin(Rover.nav_dists)] * 180/np.pi
                
                Rover.steer = np.clip(dir_angle, -15, 15)

                # get stuck
                if Rover.vel < 0.1:
                    print("counter", counter)
                    counter+= 1
                    if counter > 30:
                        Rover.mode = "stuck"
                else:
                    counter = 0
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = 15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = 0.5*(Rover.nav_angles[np.argmax(Rover.nav_dists)] + np.percentile(Rover.nav_angles * 180/np.pi, np.random.randint(20, 50))) # np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'

            elif Rover.near_sample > 0 and Rover.vel == 0:
                Rover.picking_up = 1

        elif Rover.mode == "stuck":
            stuck_change_counter += 1
            Rover.throttle = 0
            # Release the brake to allow turning
            Rover.brake = 0
            # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
            Rover.steer = 15 # Could be more clever here about which way to turn

            if stuck_change_counter > np.random.randint(20, 50):
                counter = 0
                stuck_change_counter = 0
                Rover.throttle = Rover.throttle_set
                # Release the brake to allow turning
                Rover.brake = 0
                # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                Rover.steer = np.mean(Rover.sample_angles) * 180 / np.pi if len(Rover.sample_angles)> 0 else 0 # Could be more clever here about which way to turn
                Rover.mode = "forward"

        elif Rover.mode == "found":
            if len(Rover.sample_angles) == 0 and found_counter < 100:
                Rover.mode = "forward"
            elif Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.2:
                Rover.brake = 0
                print(np.mean(Rover.sample_dists))
                dir_angle = np.mean(Rover.sample_angles) * 180/np.pi
                dist = np.mean(Rover.sample_dists)
                print(dir_angle, dist)
                if  dist > 80:
                    Rover.throttle = Rover.throttle_set
                    Rover.steer = np.clip(dir_angle, -15, 15)
                else:
                    Rover.mode = "picking"

                if abs(dir_angle) > 40: # stop and turn body
                    Rover.throttle = 0 
                    Rover.steer = np.clip(dir_angle, -15, 15)
            found_counter += 1

        elif Rover.mode == "picking":
            if len(Rover.sample_angles) == 0:
                Rover.mode = 'forward'
            elif Rover.near_sample == 0:
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set * 0.8
                dir_angle = np.mean(Rover.sample_angles) * 180/np.pi
                Rover.steer = np.clip(dir_angle, -15, 15)
            elif Rover.near_sample > 0:
                if Rover.vel > 0:
                    Rover.brake = Rover.brake_set
                    Rover.throttle = 0
                else:
                    Rover.send_pickup = True
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

