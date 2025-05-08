import math
import utm



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
Source: (https://snehilsanyal.github.io/files/paper1.pdf)
"""
""" PROTOTYPE FUNCTION """
def monocular_vision_prototype(drone_lat, drone_lon, drone_alt, drone_hdg, gsd, image_width, image_height, pixel_x, pixel_y, focal_length, sensor_width):
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
    - focal_length: Focal length of the camera (in millimeters).
    - sensor_width: Width of the camera sensor (in millimeters).

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


# Function to calculate the distance to the target pixel on the ground using camera parameters and GSD
def calculate_distance_from_pixel_to_ground(gsd, pixel_x, pixel_y, image_width, image_height, focal_length, sensor_width, drone_alt):
    """
    Calculate the distance to the target pixel on the ground in meters using the camera parameters and GSD.
    """
    # Calculate the focal length in meters
    focal_length_m = focal_length / 1000.0
    
    # The sensor size in meters (since it's given in millimeters, we divide by 1000)
    sensor_size_m = sensor_width / 1000.0

    # Field of View in the camera
    fov_x = (sensor_size_m * drone_alt) / focal_length_m  # Field of view in x direction
    fov_y = (fov_x * image_height) / image_width  # Field of view in y direction
    
    # Find the target pixel's location relative to the image center
    offset_x = (pixel_x - image_width / 2) * gsd  # In meters
    offset_y = (pixel_y - image_height / 2) * gsd  # In meters

    return offset_x, offset_y

def monocular_vision(drone_lat, drone_lon, drone_alt, drone_hdg, gsd, image_width, image_height, pixel_x, pixel_y, focal_length, sensor_width):
    """Calculate the new GPS position after moving based on the target pixel."""
    
    # Step 1: Convert the drone's GPS coordinates to UTM
    easting, northing, zone_number, zone_letter = gps_to_utm(drone_lat, drone_lon)
    
    # Step 2: Calculate the ground displacement from the pixel
    offset_x, offset_y = calculate_distance_from_pixel_to_ground(gsd, pixel_x, pixel_y, image_width, image_height, focal_length, sensor_width, drone_alt)
    
    # Step 3: Convert heading to radians
    heading_rad = math.radians(drone_hdg)

    # Calculate the UTM displacement in the east and north direction
    delta_easting = offset_x * math.sin(heading_rad) + offset_y * math.cos(heading_rad)
    delta_northing = offset_x * math.cos(heading_rad) - offset_y * math.sin(heading_rad)
    
    # Step 4: Update the drone's UTM position
    new_easting = easting + delta_easting
    new_northing = northing + delta_northing
    
    # Step 5: Convert the new UTM coordinates back to GPS coordinates
    new_lat, new_lon = utm_to_gps(new_easting, new_northing, zone_number, zone_letter)
    
    return new_lat, new_lon

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

# Function to convert GPS (latitude, longitude) to UTM coordinates
def gps_to_utm(lat, lon):
    """Convert GPS (latitude, longitude) to UTM coordinates."""
    return utm.from_latlon(lat, lon)

# Function to convert UTM coordinates back to GPS (latitude, longitude)
def utm_to_gps(easting, northing, zone_number, zone_letter):
    """Convert UTM coordinates back to GPS (latitude, longitude)."""
    return utm.to_latlon(easting, northing, zone_number, zone_letter)

# TESTING

# Monocular vision test
if __name__ == "__main__":
    drone_lat = 34.0434798 # Latitude is positive for North
    drone_lon = -117.81161689999999 # Longitude is negative for West
    drone_alt = 6.101 # Altitude in meters
    drone_hdg = 310.51 # Heading in degrees
    image_width = 1920 # Image width in pixels
    image_height = 1080 # Image height in pixels
    pixel_x = 426.6366968154907 # Target pixel x-coordinate
    pixel_y = 782.712043762207 # Target pixel y-coordinate

    # Calculate GSD test
    focal_length = 16 # Focal length in millimeters
    sensor_width = 15.6 # Sensor width in millimeters
    sensor_height = 23.5

    gsd = calculate_gsd(focal_length, sensor_width, image_width, drone_alt)
    print("Ground Sampling Distance:", gsd, "meters per pixel")
    
    new_lat, new_lon = monocular_vision(
        drone_lat, drone_lon, drone_alt, drone_hdg,
        gsd, image_width, image_height,
        pixel_x, pixel_y, focal_length, sensor_width
    )

    print(f"New GPS coordinates: Latitude = {new_lat}, Longitude = {new_lon}")



