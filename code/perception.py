import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

def color_near(img, rgb_anchor=(130, 120, 0)):
    color_select = np.zeros_like(img[:,:,0])
    near_bool = (np.abs(img - rgb_anchor) < (60, 60, 20)).all(axis=2)
    color_select[near_bool] = 1
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix * np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + (xpix_rot / scale))
    ypix_translated = np.int_(ypos + (ypix_rot / scale))
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped

def get_warp_source_destination_points(img):
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])

    return source, destination

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    source, dest = get_warp_source_destination_points(Rover.vision_image)
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, source, dest)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrian = color_thresh(warped)
    rock_sample = color_near(warped)
    obstacle = color_thresh(warped, (0, 0, 0)) - terrian - rock_sample
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle * 255.0 # obstacle color-thresholded binary image
    Rover.vision_image[:,:,1] = rock_sample * 255.0 # color-thresholded binary image
    Rover.vision_image[:,:,2] = terrian * 255.0 #navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    x_nav_rover, y_nav_rover = rover_coords(terrian)
    x_obstacle_rover, y_obstacle_rover = rover_coords(obstacle)
    x_rock_sample_rover, y_rock_sample_rover = rover_coords(rock_sample)
    # 6) Convert rover-centric pixel values to world coordinates
    scale = 10
    navigable_x_world, navigable_y_world = pix_to_world(x_nav_rover, y_nav_rover, Rover.pos[0], 
                                    Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(x_rock_sample_rover, y_rock_sample_rover, Rover.pos[0], 
                                    Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale)
    obstacle_x_world, obstacle_y_world = pix_to_world(x_obstacle_rover, y_obstacle_rover, Rover.pos[0], 
                                    Rover.pos[1], Rover.yaw, 
                                    Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    angle_thres = 0.5
    if not(Rover.pitch > angle_thres and Rover.pitch < 360 - angle_thres) and not(abs(Rover.roll) > angle_thres and Rover.roll < 360 - angle_thres):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    dist, angles = to_polar_coords(x_nav_rover, y_nav_rover)
    Rover.nav_dists = dist
    Rover.nav_angles = angles
    
    Rover.sample_dists, Rover.sample_angles = to_polar_coords(x_rock_sample_rover, y_rock_sample_rover)
    # Rover.rock_dists, Rover.rock_angles = to_polar_coords(x_obstacle_rover, y_obstacle_rover)
    
    
    return Rover