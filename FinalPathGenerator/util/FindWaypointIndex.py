def findWaypointIndex(direction_changes,path):
    waypointIndex = [0]
    for waypoint in direction_changes:
        waypointIndex.append(path.index(waypoint))
    return waypointIndex