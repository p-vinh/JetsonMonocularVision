import math


def stereo_vision(spread, center_right, center_left, fov, image_width, image_height):
    # converting fov and image_width to float
    fov = fov * 1.0
    image_width = image_width * 1.0
    dpp = fov / image_width  # calculating degrees per pixel (dpp)

    # Get angle between right edge of the fov and the line of sight in the left image
    # (90 - fov / 2) -- calculates angle between the line that connects two cameras (spread/separation) and the edge of
    # the fov.
    # center_left[0] -- this is x position of the center of the detection, it is measured in pixels from leftmost edge
    # of the image.
    # (image_width - center_left[0]) -- converts x pixel value measured from left edge of the image to number of pixels
    # from right edge of the image.
    # Multiply value from above by amount of degrees per pixel (dpp) to get number angle between right edge of the fov
    # and line of sight to the target.
    # Add angle from above to angle between the line that connects two cameras (spread/separation) and the edge of the
    # fov and get angle between the spread and line of sight to the target
    angle_left = (90 - fov / 2) + (image_width - center_left[0]) * dpp
    # Do the same calculation as above but this time use angle measured from left side of the fov to the line of sight
    # to the target (uses x pixel value measured from left side of the image, so conversion is not needed).
    angle_right = (90 - fov / 2) + center_right[0] * dpp
    # Find 3rd angle.
    # Sum of all angles in a triangle is 180 deg.
    # By subtracting two angle calculated above from 180 deg get angle between two lines of sight when they converge on
    # target.
    angle_center = 180 - angle_left - angle_right

    # if angle is 0 or less tan 0, then triangle cannot be formed.
    # Triangulation fails.
    if angle_center <= 0:
        print("Object is too far!")
        return float('NaN'), float('NaN'), float('NaN')
    print("TRIANGULATION:    Central angle: ", angle_center)

    # Calculate vertical angle.
    # (center_left[1] + center_right[1]) / 2.0 -- average y pixel value from both left and right detections. This value
    # is measured in pixels from the top of the image going down.
    # (image_height / 2.0) -- subtract the amount of pixels to convert from pixels from the top, to amount of pixels
    # from central horizontal line.
    # Then multiply by degrees per pixel to get angle from ray going out of the center of that camera to the line of
    # sight to the target. Negative - target is above of where camera is looking, Positive - target is below of where
    # the camera is looking.
    # This measures angle in vertical profile only, not including targets offset left to right. Or simpli how many
    # degrees the camera should be rotated up or down so that horizontal middle line (the line that divides image into
    # top half and bottom half) will go through the center of the target.
    vertical_angle = ((center_left[1] + center_right[1]) / 2.0 - (image_height / 2.0)) * dpp
    print("TRIANGULATION:    Vertical angle:", vertical_angle)

    # Get the angle between line that connects the center of spread (line connecting two cameras) to target and another
    # line connecting center of the spread to left camera
    angle_center_target = 180 - angle_left - (angle_center / 2)
    print("TRIANGULATION:    Horizontal angle:", angle_center_target)

    # Convert angles to radians.
    # Used for sin()
    angle_left = math.radians(angle_left)
    angle_right = math.radians(angle_right)
    angle_center = math.radians(angle_center)

    # Using law of signs calculate distance from left camera to the target.
    distance_left = spread * math.sin(angle_right) / math.sin(angle_center)
    # Using law of signs calculate distance from right camera to the target.
    distance_right = spread * math.sin(angle_left) / math.sin(angle_center)

    # Calculate distance from the target to center point inbetween two cameras (center of the spread).
    # Simple take the average of distances from left camera to the target and distance from right camera to the target.
    distance_average = (distance_left + distance_right) / 2

    # Return 3 things:
    # 1. Distance from center of the camera spread to the target.
    # 2. Angle between two lines:
    #       -> one that connects center point on the spread to the target
    #       -> and another line that connects center point on the spread to the left camera.
    # 3. Angle from the horizontal centerline to the canter of the target. This angle does not include horizontal
    # deviation, and only tells how many degrees is the line of sight to the target is above the horizontal plane of
    # the camera.
    return distance_average, angle_center_target, vertical_angle


"""
A lower value for GSD means a more accurate survey. Your survey cannot be more accurate than your GSD.
The range for UAV photogrammetry typically falls between 1.5 to 2.5 cm/px (.6 to 1 inch).
Some recommend a ground sample distance of 1 cm/px for professional surveys, which is very low.
"""
def monocular_vision(drone_lat, drone_lon, drone_alt, drone_hdg, gsd, image_width, image_height, pixel_x, pixel_y):
    """
    Calculate the GPS location of a point in an image.

    Parameters:
    - drone_lat: Latitude of the drone (in decimal degrees).
    - drone_lon: Longitude of the drone (in decimal degrees).
    - drone_alt: Altitude of the drone (in meters).
    - drone_hdg: Heading of the drone (in decimal degrees).
    - gsd: Ground Sampling Distance (in meters per pixel).
    - image_width: Width of the image (in pixels).
    - image_height: Height of the image (in pixels).
    - pixel_x: X-coordinate of the target pixel in the image.
    - pixel_y: Y-coordinate of the target pixel in the image.

    Returns:
    - (target_lat, target_lon): Latitude and longitude of the target point.
    """
    EARTH_RADIUS = 6378137.0

    # Convert image pixel coordinates to normalized camera coordinates
    cx = image_width / 2 # Principal point of x
    cy = image_height / 2 # Principal point of y

    # Calculate the focal length in pixels
    fx = (focal_length / sensor_width) * image_width
    fy = fx # Assuming square pixels (fx = fy)

    # Calculate the normalized camera coordinates
    x_n = (pixel_x - cx) / fx
    y_n = (pixel_y - cy) / fy

    # # Calculate the ground distance from the image center to the target pixel
    # dx = (pixel_x - image_width / 2) * gsd
    # dy = (pixel_y - image_height / 2) * gsd

    # Estimate real-world coordinates
    dx = x_n * gsd * image_width
    dy = y_n * gsd * image_height

    target_distance = math.sqrt(dx**2 + dy**2 + drone_alt**2)
    bearing = math.radians(drone_hdg) + math.atan2(dx, dy)

    # Convert ENU (East North Up) coorinate shift
    delta_x = target_distance * math.cos(bearing)
    delta_y = target_distance * math.sin(bearing)

    # Convert ground distance to latitude and longitude offsets
    # target_lat = drone_lat + (dy / EARTH_RADIUS * (180 / math.pi))
    # target_lon = drone_lon + (dx / (E + (ARTH_RADIUS * math.cos(math.radians(drone_lat))) * (180 / math.pi))

    # ENU -> GPS
    target_lat = drone_lat + (delta_y / EARTH_RADIUS) * (180 / math.pi)
    target_lon = drone_lon + (delta_x / (EARTH_RADIUS * math.cos(math.radians(drone_lat)))) * (180 / math.pi)

    return target_lat, target_lon


def calculate_gsd(focal_length, sensor_width, image_width, altitude):
    """
    Calculate the Ground Sampling Distance (GSD) in meters per pixel.

    Parameters:
    - focal_length: Focal length of the camera (in millimeters).
    - sensor_width: Width of the camera sensor (in millimeters).
    - image_width: Width of the image (in pixels).
    - altitude: Altitude of the drone (in meters).

    Returns:
    - gsd: Ground Sampling Distance (in meters per pixel).
    
    Ex: Raspberry Pi v2 Camera Module:
    Focal length: 3.04 mm
    Sensor width: 3.68 mm
    Image width: 3280 pixels
    Altitude: 10 meters
    Source: (https://www.raspberrypi.com/documentation/accessories/camera.html)
    """
    # Convert focal length and sensor width to meters
    focal_length_m = focal_length / 1000.0
    sensor_width_m = sensor_width / 1000.0

    # Calculate GSD
    gsd = (altitude * sensor_width_m) / (focal_length_m * image_width)
    return gsd


# TESTING

# Monocular vision test
if __name__ == "__main__":
    # General Coordinates for Spadra Farm
    drone_lat = 34.045004 # Latitude is positive for North
    drone_lon = -117.811608 # Longitude is negative for West
    drone_alt = 10 # Altitude in meters
    drone_hdg = 45 # Heading in degrees
    image_width = 1920 # Image width in pixels
    image_height = 1080 # Image height in pixels
    pixel_x = 960 # Target pixel x-coordinate
    pixel_y = 540 # Target pixel y-coordinate

    # Calculate GSD test
    focal_length = 2.12 # Focal length in millimeters
    sensor_width = 4.80 # Sensor width in millimeters

    gsd = calculate_gsd(focal_length, sensor_width, image_width, drone_alt)
    print("Ground Sampling Distance:", gsd, "meters per pixel")

    target_lat, target_lon = monocular_vision(drone_lat, drone_lon, drone_alt, drone_hdg, gsd, image_width, image_height, pixel_x, pixel_y)
    print("Target GPS coordinates:", target_lat, target_lon)

