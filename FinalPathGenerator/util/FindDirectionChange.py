def find_direction_changes(path):
    direction_changes = []
    for i in range(1, len(path) - 1):
        current = path[i]
        previous = path[i - 1]
        next_point = path[i + 1]
        
        dx1, dy1 = current[0] - previous[0], current[1] - previous[1]
        dx2, dy2 = next_point[0] - current[0], next_point[1] - current[1]
        
        if dx1 * dy2 != dy1 * dx2:
            # Yön değişikliği var
            direction_changes.append(current)
    if len(direction_changes)==0:
        print("yön değişmedi")
    
    return direction_changes