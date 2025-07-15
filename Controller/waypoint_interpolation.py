import numpy as np

class WaypointInterpolation:
    
    @staticmethod
    def interpolate_waypoints(waypoints_np, INTERP_DISTANCE_RES):
        wp_distance = []
        for i in range(1, waypoints_np.shape[0]):
            wp_distance.append(
                np.sqrt((waypoints_np[i, 0] - waypoints_np[i - 1, 0]) ** 2 +
                        (waypoints_np[i, 1] - waypoints_np[i - 1, 1]) ** 2))
        wp_distance.append(0)

        wp_interp = []  
        wp_interp_hash = []  
        interp_counter = 0
        list2 = [0, 0]
        old_wp_uvector = np.array(list2)

        for i in range(waypoints_np.shape[0] - 1):
            wp_interp.append(list(waypoints_np[i]))
            wp_interp_hash.append(interp_counter)
            interp_counter += 1

            num_pts_to_interp = int(np.floor(wp_distance[i] / float(INTERP_DISTANCE_RES)) - 1)
            wp_vector = waypoints_np[i + 1] - waypoints_np[i]
            if np.linalg.norm(wp_vector) != 0:
                wp_uvector = wp_vector / np.linalg.norm(wp_vector)
            else:
                wp_uvector = old_wp_uvector

            for j in range(num_pts_to_interp):
                next_wp_vector = INTERP_DISTANCE_RES * float(j + 1) * wp_uvector
                wp_interp.append(list(waypoints_np[i] + next_wp_vector))
                interp_counter += 1
            old_wp_uvector = wp_uvector

        wp_interp.append(list(waypoints_np[-1]))
        wp_interp_hash.append(interp_counter)
        interp_counter += 1

        return wp_interp, wp_distance, wp_interp, wp_interp_hash
