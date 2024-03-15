import math


def stereo_vision(spread, center_right, center_left, fov, image_width, image_height):
    fov = fov * 1.0
    image_width = image_width * 1.0  # converting fov and image_width to float
    dpp = fov / image_width  # calculating degrees per pixel (dpp)

    angle_left = (90 - fov / 2) + (image_width - center_left[0]) * dpp
    # print("Left angle: ", angle_left)
    angle_right = (90 - fov / 2) + center_right[0] * dpp
    # print("Right angle: ", angle_right)
    angle_center = 180 - angle_left - angle_right

    if angle_center <= 0:
        print("Object is too far!")
        return float('NaN'), float('NaN'), float('NaN')
    print("TRIANGULATION:    Central angle: ", angle_center)

    vertical_angle = ((center_left[1] + center_right[1]) / 2.0 - (image_height / 2.0)) * dpp
    print("TRIANGULATION:    Vertical angle:", vertical_angle)

    angle_center_target = 180 - angle_left - (angle_center / 2)
    print("TRIANGULATION:    Horizontal angle:", angle_center_target)

    angle_left = math.radians(angle_left)
    angle_right = math.radians(angle_right)
    angle_center = math.radians(angle_center)

    distance_left = spread * math.sin(angle_right) / math.sin(angle_center)
    distance_right = spread * math.sin(angle_left) / math.sin(angle_center)

    distance_average = (distance_left + distance_right) / 2

    return distance_average, angle_center_target, vertical_angle


