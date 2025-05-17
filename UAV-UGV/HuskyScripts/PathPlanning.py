import geopy.distance





def get_closest_node(gps_nodes, target):
    """
    Calculate the distance between two points using geopy.
    :param current: List of GPS Node locations (latitude, longitude)
    :param target: Target location (latitude, longitude)
    :return: Closest node and its distance from the target
    """
    node_distances = []
    for node in gps_nodes:
        node_location = (node[0], node[1])
        distance = geopy.distance.distance(node_location, target).m
        node_distances.append(distance)
    closest_node_index = node_distances.index(min(node_distances))

    # Return the closest node and its distance
    return gps_nodes[closest_node_index], node_distances[closest_node_index]


def get_heading(current, target):
    """
    Calculate the heading from current location to target location.
    :param target: Heading in degrees
    :param current: Heading in degrees
    :return: Heading in degrees
    """

    row_heading = (target - current + 360) % 360 

    return row_heading



if __name__ == "__main__":
    gps_nodes = [(37.7749, -122.4194), (34.0522, -118.2437), (40.7128, -74.0060)]
    target = (36.7783, -119.4179)
    closest_node, distance = get_closest_node(gps_nodes, target)
    print(f"Closest node: {closest_node}, Distance: {distance} meters")

    current_heading = 45
    target_heading = 90
    heading = get_heading(current_heading, target_heading)
    print(f"Heading from {current_heading} to {target_heading}: {heading} degrees")