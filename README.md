[//]: # (Image References)

[states]: ./misc/states.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

# Project: Search and Sample Return
---

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

1. To record my data, It is by pressing two times (start and end) `r` key and select the folder where the images are saved. 

2. find the sample rock by color selection near to yellow useing function 
``` python
def color_near(img, rgb_anchor=(130, 120, 0)):
    color_select = np.zeros_like(img[:,:,0])
    near_bool = (np.abs(img - rgb_anchor) < (60, 60, 20)).all(axis=2)
    color_select[near_bool] = 1
    return color_select
```

3. find the obstacle by set substracting
``` python
obstacle = color_thresh(warped, (0, 0, 0)) - terrian - rock_sample
```


#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

1. the main part of my modification in `process_image()`
``` python
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    warped = perspect_transform(img, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrian = color_thresh(warped)
    rock_sample = color_near(warped)
    obstacle = color_thresh(warped, (0, 0, 0)) - terrian - rock_sample
    # colorfulize the navigation screen
    warped[:,:,0] = obstacle * 255.0 
    warped[:,:,1] = rock_sample * 255.0 
    warped[:,:,2] = terrian * 255.0 

    # 4) Convert thresholded image pixel values to rover-centric coords
    x_nav_rover, y_nav_rover = rover_coords(terrian)
    x_obstacle_rover, y_obstacle_rover = rover_coords(obstacle)
    x_rock_sample_rover, y_rock_sample_rover = rover_coords(rock_sample)
    # 5) Convert rover-centric pixel values to world coords
    scale = 10
    # Get navigable pixel positions in world coords
    navigable_x_world, navigable_y_world = pix_to_world(x_nav_rover, y_nav_rover, data.xpos[data.count], 
                                    data.ypos[data.count], data.yaw[data.count], 
                                    data.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(x_rock_sample_rover, y_rock_sample_rover, data.xpos[data.count], 
                                    data.ypos[data.count], data.yaw[data.count], 
                                    data.worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(x_obstacle_rover, y_obstacle_rover, data.xpos[data.count], 
                                    data.ypos[data.count], data.yaw[data.count], 
                                    data.worldmap.shape[0], scale)
    # Add pixel positions to worldmap
    # 6) Update worldmap (to be displayed on right side of screen)
    data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    data.worldmap[rock_y_world, rock_x_world, 1] += 1
    data.worldmap[navigable_y_world, navigable_x_world, 2] += 1
```

2. the movie are saved in the `output` folder


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

##### perception_step()
similar to the Notebook analysis,
1. first calibrate the camera
2. project the vision to the map 
3. use color threshold to find navigable terrian, rock sample and obstacle
4. there is a difference in code to improve the map fiedility. I use the threshold to control whether or not to update the world map.
```python
angle_thres = 0.5
if not(Rover.pitch > angle_thres and Rover.pitch < 360 - angle_thres) and not(abs(Rover.roll) > angle_thres and Rover.roll < 360 - angle_thres):
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
```

##### decision_step()
I divide the rover movements into serval different states, as following picture:
![alt text][states]

1. the `forward` state: the rover is moving with the navigable map.
2. the `stop` state: there is no enough navigable infomation to use. The rover have to change the directions.
3. the `stuck` state: the rover has enough navigable views but it was without moving for a while.
4. the 'found' state: the sample rock appears in its view. it has to brake and steer widely, avoiding to miss it.
5. the 'picking' state: the rover move to sample rock until the sensors tell it sample near to enough to pick it up.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

1. enviroments:
    2. FPS: about 16
    3. resolution: 800 x 600
    4. quality: Fantastic 


