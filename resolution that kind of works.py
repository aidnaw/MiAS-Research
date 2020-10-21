 #### FIND RESOLUTION ####

            # Try type 1 resolution. Minimize heading changes
            
            resolution_type = '1'
            min_separation = min(i for i in separation_straight if i >= self.d_req)
            resolution_index = separation_straight.index(min_separation)
            resolution_angle = turn_angles[resolution_index]
            resolution_time = self.a.time_to_turn(resolution_angle) + self.get_t_smin(resolution_angle, 0) 

            # Check time constraint. If failure, fall back to 1a solution.
            if resolution_time > 1.2 * self.a.time_to_turn(turn_angle_min) and d_tmin >= self.d_req:
                resolution_type = '1a'
                min_separation = d_tmin
                resolution_angle = turn_angle_min
                resolution_time = turn_time_min

            # Check turn separation constraint. If separation is lost in turn, it's a type 2 solution.
            if d_tmin < self.d_req and abs(resolution_angle) >= abs(turn_angle_min):
                # Try for a 2a solution
                resolution_type = '2a Failed'
                req_sep = min(i for i in separation[index:] if i >= self.d_req)
                req_sep_index = separation.index(req_sep)
                resolution_angle = turn_angles[req_sep_index]
                resolution_time = self.a.time_to_turn(resolution_angle)
                min_separation = d_tmin
                # If there's no angle that fulfills the separation, a 2b maneuver is required
                if req_sep is None:
                    resolution_type = '2b Failed'
                    sep = max(separation)
                    sep_index = separation.index(sep)
                    resolution_angle = turn_angles[sep_index]
                    resolution_time = self.a.time_to_turn(resolution_angle)
                    min_separation = d_tmin 
