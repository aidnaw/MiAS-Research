import numpy as np
from scipy import signal
import matplotlib.pyplot as plt


class MainAircraft:
# Define an aircraft with a given airspeed
    
    g = 9.81 # gravitational constant

    def __init__(self, airspeed):
        self.airspeed = airspeed
        self.initial_position = np.array([0,0])
        self.initial_heading = 0
        self.turn_angle = 0
        self.bank_angle = 0
        self.turn_radius = np.inf
        
    #### Aircraft maneuverability equations

    def bank_angle_(self, bank_angle):
    # Sets bank angle and therefore turn radius of the aircraft
        self.bank_angle = bank_angle
        if bank_angle != 0:
            self.turn_radius = np.abs(self.airspeed**2/MainAircraft.g/np.tan(self.bank_angle)) # (2)
        else:
            self.turn_radius = np.inf

    def turn_angle_(self, t):
    #  returns turn angle if craft were to turn for t seconds (3)
        turn_angle = t*MainAircraft.g*np.tan(self.bank_angle)/self.airspeed
        return turn_angle

    def time_to_turn(self, turn_angle): 
    # returns time required to turn to a specified turn angle
        if self.bank_angle == 0: print('Aircraft cannot turn with no bank angle.')
        else: 
            t = turn_angle*self.airspeed/MainAircraft.g/np.tan(self.bank_angle)
            return t

    #### Function for position vectors from origin to aircraft while in turn

    def turn_position(self, t): 
    # Returns position vector wrt origin at t seconds while in turn (6)
        R = self.turn_radius
        dphi = self.turn_angle_(t)
        turn_position = np.array([1-np.cos(dphi),np.sin(dphi)])*R*np.sign(dphi)
        return turn_position

class ConflictingCraft(MainAircraft):
# Define the conflicting aircraft with given airspeed. Give position/heading relative to main
    def __init__(self, airspeed, initial_position, initial_heading):
        super().__init__(airspeed)
        self.initial_position = np.array(initial_position)
        self.initial_heading = initial_heading

    #### Function for position vectors from origin to aircraft while in turn

    def turn_position(self, t): 
    # Returns position vector wrt origin at t seconds while in turn (8)
        R = self.turn_radius
        dphi = self.turn_angle_(t)
        heading0 = self.initial_heading
        b0 = self.initial_position
        turn_position = np.array([b0[0] + R*np.sign(dphi)*np.cos(heading0) - R*np.sign(dphi)*np.cos(heading0 + dphi),
                        b0[1] - R*np.sign(dphi)*np.sin(heading0) + R*np.sign(dphi)*np.sin(heading0 + dphi)])
        return turn_position

class ManeuverData:
    def __init__(self, aircraft_a, aircraft_b):
        self.a = aircraft_a
        self.b = aircraft_b
        self.d_req = 9260 # 5 nautical miles in meters

    #### Functions for turn segment ####

    def get_da(self,t): 
    # turn separation if only A turns (10) (WORKING)
        R = self.a.turn_radius
        dphi = self.a.turn_angle_(t)
        heading0 = self.b.initial_heading
        b0 = self.b.initial_position
        airspeed = self.b.airspeed
        da = np.sqrt( (b0[0] + t*airspeed*np.sin(heading0) - R*np.sign(dphi) * (1 - np.cos(dphi)))**2 
            + (b0[1] + t*airspeed*np.cos(heading0) - R*np.sign(dphi)*np.sin(dphi))**2)
        return da

    def get_db(self,t): 
    # turn separation if only B turns (11) (WORKING)
        R = self.b.turn_radius
        dphi = self.b.turn_angle_(t)
        heading0 = self.b.initial_heading
        b0 = self.b.initial_position
        airspeed = self.a.airspeed
        db = np.sqrt( (b0[0] + R*np.sign(dphi)*np.cos(heading0) - R*np.sign(dphi) * np.cos(heading0 +dphi))**2
                    + (b0[1] - t*airspeed - R*np.sign(dphi) * np.sin(heading0) + R*np.sign(dphi)*np.sin(heading0 +dphi))**2)
        return db

    def get_dab(self, t):
    # turn separation in cooperative maneuver (13) (WORKING)
        R_b = self.b.turn_radius
        dphi_b = self.b.turn_angle_(t)
        R_a = self.a.turn_radius
        dphi_a = self.a.turn_angle_(t)
        heading0 = self.b.initial_heading
        b0 = self.b.initial_position
        dab = np.sqrt( (b0[0] + R_b*np.sign(dphi_b)*np.cos(heading0) - R_b*np.sign(dphi_b)
                         * np.cos(heading0 + dphi_b) - R_a*np.sign(dphi_a) * (1-np.cos(dphi_a)))**2
                        + (b0[1] - R_b*np.sign(dphi_b) * np.sin(heading0) + R_b*np.sign(dphi_b)
                          *np.sin(heading0 + dphi_b) - R_a*np.sign(dphi_a)*np.sin(dphi_a))**2 )
        return dab

    #### Functions for straight line segment ####

    def get_t_smin(self, turn_angle_a, turn_angle_b): 
    # time after turn ends of min separation in straight line segment (18)
        if turn_angle_a == 0 and turn_angle_b == 0:
            p_a1 = self.a.initial_position
            p_b1 = self.b.initial_position
        else:
            if turn_angle_a == 0: 
                p_a1 = np.array([0, self.a.airspeed*self.b.time_to_turn(turn_angle_b)])
            else:
                p_a1 = self.a.turn_position(self.a.time_to_turn(turn_angle_a))
            if turn_angle_b == 0:
                p_b1 = np.array([self.b.initial_position[0] + self.b.airspeed*np.sin(self.b.initial_heading)*self.a.time_to_turn(turn_angle_a),
                         self.b.initial_position[1] + self.b.airspeed*np.cos(self.b.initial_heading)*self.a.time_to_turn(turn_angle_a)])
            else:
                p_b1 = self.b.turn_position(self.b.time_to_turn(turn_angle_b))

        dx1 = p_b1[0] - p_a1[0]
        dy1 = p_b1[1] - p_a1[1]
        heading_a = turn_angle_a
        heading_b = turn_angle_b + self.b.initial_heading
        Vrx = self.b.airspeed*np.sin(heading_b) - self.a.airspeed*np.sin(heading_a)
        Vry = self.b.airspeed*np.cos(heading_b) - self.a.airspeed*np.cos(heading_a)
        t_smin = -(dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2)
        return t_smin

    def get_d_smin(self, turn_angle_a, turn_angle_b):
    # min separation in straight line segment (19) 
        if turn_angle_a == 0 and turn_angle_b == 0:
            p_a1 = self.a.initial_position
            p_b1 = self.b.initial_position
        else:
            if turn_angle_a == 0: 
                p_a1 = np.array([0, self.a.airspeed*self.b.time_to_turn(turn_angle_b)])
            else:
                p_a1 = self.a.turn_position(self.a.time_to_turn(turn_angle_a))
            if turn_angle_b == 0:
                p_b1 = np.array([self.b.initial_position[0] + self.b.airspeed*np.sin(self.b.initial_heading)*self.a.time_to_turn(turn_angle_a),
                         self.b.initial_position[1] + self.b.airspeed*np.cos(self.b.initial_heading)*self.a.time_to_turn(turn_angle_a)])
            else:
                p_b1 = self.b.turn_position(self.b.time_to_turn(turn_angle_b))

        dx1 = p_b1[0] - p_a1[0]
        dy1 = p_b1[1] - p_a1[1]
        heading_a = turn_angle_a
        heading_b = turn_angle_b + self.b.initial_heading
        Vrx = self.b.airspeed*np.sin(heading_b) - self.a.airspeed*np.sin(heading_a)
        Vry = self.b.airspeed*np.cos(heading_b) - self.a.airspeed*np.cos(heading_a)
        d_smin = np.sqrt( (dx1 - Vrx*(dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2))**2 
                        + (dy1 - Vry*(dx1*Vrx + dy1*Vry)/(Vrx**2 + Vry**2))**2 )
        return d_smin

    #### Table data and plot for each maneuver type ####

    def A_turns_B_straight(self, bank_angle):
        for i in range(2):
            if i == 0:
                direction = 'right'
                turn_angles = np.deg2rad(np.arange(0, 150, 2))
            else: 
                direction = 'left'
                bank_angle = -bank_angle
                turn_angles = np.deg2rad(np.arange(0, -150, -2))
            
            self.a.bank_angle_(np.deg2rad(bank_angle))

            # Find minimum turn separation, corresponding time, and turn angle
            separation = []
            for turn_angle in turn_angles:
                separation.append(self.get_da(self.a.time_to_turn(turn_angle)))
            index = separation.index(min(separation))
            d_tmin = separation[index]
            turn_angle_min = turn_angles[index]
            turn_time_min = self.a.time_to_turn(turn_angle_min)

            # Find time to minimum separation in straight line followed by minimum separation
            t_smin = []
            separation_straight = []
            for j, turn_angle in enumerate(turn_angles):
                t_smin.append(self.get_t_smin(turn_angle,0))
                separation_straight.append(self.get_d_smin(turn_angle, 0))
                # Plots meet at min turn separation
                if  j > index: 
                    separation_straight[-1] = separation[j] 

            # Find time to get to min separation overall w/ specified turn angle
            turn_times = []
            for turn_angle in turn_angles:
                turn_times.append(self.a.time_to_turn(turn_angle))
            resolution_times = []
            for [j, turn_time] in enumerate(turn_times):
                if t_smin[j] >= 0:
                    resolution_times.append(turn_time + t_smin[j])
                else:
                    resolution_times.append(turn_time)

            #### FIND RESOLUTION DATA ####

            # Check for any maxima prior to locus merge
            maximum = signal.argrelmax(np.array(separation_straight)) # Check if there is a local max
            maximum = maximum[0] # Isolate the index of the max
            
            # Try for type 1 resolution
            if d_tmin >= self.d_req or (maximum.size > 0 and separation_straight[maximum[0]] >= self.d_req):
                resolution_type = '1'
                min_separation = min(i for i in separation_straight if i > self.d_req)
                resolution_index = separation_straight.index(min_separation)
                resolution_angle = turn_angles[resolution_index]
                resolution_time = resolution_times[resolution_index]
                # Check resolution time constraint or if resolution is in unstable region. If violated, switch to 1a solution
                if resolution_time > 1.2 * turn_time_min or (maximum.size > 0 and resolution_index > maximum[0] and resolution_index < index):
                    if d_tmin > self.d_req:
                        resolution_type = '1a'
                        min_separation = d_tmin
                        resolution_angle = turn_angle_min
                        resolution_time = turn_time_min
                    else: resolution_type = '2'
            else: resolution_type = '2'

            # Find best parameters for type 2 resolutions.



            # Print table data
            print('A {} B straight'.format(direction))

            # Print turn data
            # print('\tMinimum Separation in Turn:')
            # print('\t\tMinimum Separation (nmi):', round(d_tmin/1852, 2))
            # print('\t\tTurn Angle (deg)', round(np.rad2deg(turn_angle_min), 2))
            # print('\t\tTime (min):', round(turn_time_min/60, 2))

            # Print resolution data
            print('\tResolution Parameters:')
            print('\t\tResolution Type', resolution_type)
            print('\t\tMinimum Separation (nmi):', round(min_separation/1852, 2))
            print('\t\tResolution Angle (deg)', round(np.rad2deg(resolution_angle), 2))
            print('\t\tResolution Time (min):', round(resolution_time/60, 2))

             # Prep plot data
            if i == 0:
                separation_right = separation
                separation_straight_right = separation_straight
                resolution_times_right = resolution_times
            else:
                separation_left = separation[::-1]
                separation_straight_left = separation_straight[::-1]
                resolution_times_left = resolution_times[::-1]


        # Plot maneuver data
        separation = np.array(separation_left + separation_right[1::]) / 1852
        separation_straight = np.array(separation_straight_left + separation_straight_right[1::]) / 1852
        resolution_times = np.array(resolution_times_left + resolution_times_right[1::])
        turn_angles = np.arange(-148,150,2)
        plt.plot(turn_angles, separation)
        plt.plot(turn_angles, separation_straight)
        plt.plot(turn_angles, np.array(resolution_times)/60)
        plt.title('A turns B straight')
        plt.ylabel('Separation (nm)')
        plt.xlabel('Turn Angle (deg)')
        plt.grid()
        plt.show()

    def A_straight_B_turns(self, bank_angle):
        for i in range(2):
            if i == 0:
                direction = 'right'
                turn_angles = np.deg2rad(np.arange(0, 150, 2))
            else: 
                direction = 'left'
                bank_angle = -bank_angle
                turn_angles = np.deg2rad(np.arange(0, -150, -2))
            
            self.b.bank_angle_(np.deg2rad(bank_angle))

            # Find minimum turn separation, corresponding time and turn angle
            separation = []
            for turn_angle in turn_angles:
                separation.append(self.get_db(self.b.time_to_turn(turn_angle)))
            index = separation.index(min(separation))
            d_tmin = separation[index]
            turn_angle_min = turn_angles[index]
            turn_time_min = self.b.time_to_turn(turn_angle_min)

            # Find time to minimum separation in straight line followed by minimum separation
            t_smin = []
            separation_straight = []
            for j, turn_angle in enumerate(turn_angles):
                t_smin.append(self.get_t_smin(0, turn_angle))
                separation_straight.append(self.get_d_smin(0, turn_angle))
                # Plots meet at min turn separation
                if  j > index: 
                    separation_straight[-1] = separation[j]

            # Find time to get to min separation w/ specified turn angle
            turn_times = []
            for turn_angle in turn_angles:
                turn_times.append(self.b.time_to_turn(turn_angle))
            resolution_times = []
            for [j, turn_time] in enumerate(turn_times):
                if t_smin[j] >= 0:
                    resolution_times.append(turn_time + t_smin[j])
                else:
                    resolution_times.append(turn_time)

           #### FIND RESOLUTION DATA ####

            # Check for any maxima prior to locus merge
            maximum = signal.argrelmax(np.array(separation_straight)) # Check if there is a local max
            maximum = maximum[0] # Isolate the index of the max
            
            # Try for type 1 resolution
            if d_tmin >= self.d_req or (maximum.size > 0 and separation_straight[maximum[0]] >= self.d_req):
                resolution_type = '1'
                min_separation = min(i for i in separation_straight if i > self.d_req)
                resolution_index = separation_straight.index(min_separation)
                resolution_angle = turn_angles[resolution_index]
                resolution_time = resolution_times[resolution_index]
                # Check resolution time constraint or if resolution is in unstable region. If violated, switch to 1a solution
                if resolution_time > 1.2 * turn_time_min or (maximum.size > 0 and resolution_index > maximum[0] and resolution_index < index):
                    if d_tmin > self.d_req:
                        resolution_type = '1a'
                        min_separation = d_tmin
                        resolution_angle = turn_angle_min
                        resolution_time = turn_time_min
                    else: resolution_type = '2'
            else: resolution_type = '2'

            # Give best resolution parameters to type 2 solutions
            if resolution_type == '2':
                min_separation = d_tmin
                resolution_time = turn_time_min
                try:
                    req_sep = min(i for i in separation[index:] if i > self.d_req)
                    resolution_index = separation.index(req_sep)
                except ValueError:
                    req_sep = np.array([])
                if req_sep.size > 0:
                    resolution_type = '2a'
                    resolution_angle = turn_angles[resolution_index]
                else:
                    resolution_type = '2b'
                    max_turn_separation = max(separation[index:])
                    resolution_index = separation.index(max_turn_separation)
                    resolution_angle = turn_angles[resolution_index]
            
            # Print table data
            print('A straight B {}'.format(direction))
            # Turn Data
            print('\tMinimum Separation in Turn:')
            print('\t\tMinimum Separation (nmi):', round(d_tmin/1852, 2))
            print('\t\tTurn Angle (deg)', round(np.rad2deg(turn_angle_min), 2))
            print('\t\tTime to min separation (min):', round(turn_time_min/60, 2))
            # Resolution Data
            print('\tResolution Parameters:')
            print('\t\tResolution Type', resolution_type)
            print('\t\tMinimum Separation (nmi):', round(min_separation/1852, 2))
            print('\t\tResolution Angle (deg)', round(np.rad2deg(resolution_angle), 2))
            print('\t\tTime to min separation (min):', round(resolution_time/60, 2))

            # Prep plot data
            if i == 0:
                separation_right = separation
                separation_straight_right = separation_straight
                resolution_times_right = resolution_times
            else:
                separation_left = separation[::-1]
                separation_straight_left = separation_straight[::-1]
                resolution_times_left = resolution_times[::-1]

        # Plot maneuver data
        separation = np.array(separation_left + separation_right[1::]) / 1852
        separation_straight = np.array(separation_straight_left + separation_straight_right[1::]) / 1852
        resolution_times = np.array(resolution_times_left + resolution_times_right[1::])
        turn_angles = np.arange(-148,150,2)
        plt.plot(turn_angles, separation)
        plt.plot(turn_angles, separation_straight)
        plt.plot(turn_angles, np.array(resolution_times)/60)
        plt.title('A straight B turns')
        plt.ylabel('Separation (nm)')
        plt.xlabel('Turn Angle (deg)')
        plt.grid()
        plt.show()        

    def A_turns_B_right(self):
        bank_angle = 30
        self.b.bank_angle_(np.deg2rad(bank_angle))

        for i in range(2):
            if i == 0:
                direction = 'right'
                turn_angles = np.deg2rad(np.arange(0, 150, 2))
            else: 
                direction = 'left'
                bank_angle = -bank_angle
                turn_angles = np.deg2rad(np.arange(0, -150, -2))

            self.a.bank_angle_(np.deg2rad(bank_angle))

            # Find minimum turn separation, corresponding time and turn angle
            separation = []
            for turn_angle in turn_angles:
                separation.append(self.get_dab(self.a.time_to_turn(turn_angle)))
            index = separation.index(min(separation))
            d_tmin = separation[index]
            turn_angle_min = turn_angles[index]
            turn_time_min = self.a.time_to_turn(turn_angle_min)
            
           # Find time to minimum separation in straight line, followed by minimum separation
            t_smin = []
            separation_straight = []
            for j, turn_angle in enumerate(turn_angles):
                t_smin.append(self.get_t_smin(turn_angle, self.b.turn_angle_(self.a.time_to_turn(turn_angle))))
                separation_straight.append(self.get_d_smin(turn_angle, self.b.turn_angle_(self.a.time_to_turn(turn_angle))))
                # Plots meet at min turn separation
                if  j > index: 
                    separation_straight[-1] = separation[j]

            # Find time to get to min separation overall w/ specified turn angle
            turn_times = []
            for turn_angle in turn_angles:
                turn_times.append(self.a.time_to_turn(turn_angle))
            resolution_times = []
            for [j, turn_time] in enumerate(turn_times):
                if t_smin[j] >= 0:
                    resolution_times.append(turn_time + t_smin[j])
                else:
                    resolution_times.append(turn_time)

            #### FIND RESOLUTION DATA ####

            # Check for any maxima prior to locus merge
            maximum = signal.argrelmax(np.array(separation_straight)) # Check if there is a local max
            maximum = maximum[0] # Isolate the index of the max
            
            # Try for type 1 resolution
            if d_tmin >= self.d_req or (maximum.size > 0 and separation_straight[maximum[0]] >= self.d_req):
                resolution_type = '1'
                min_separation = min(i for i in separation_straight if i > self.d_req)
                resolution_index = separation_straight.index(min_separation)
                resolution_angle = turn_angles[resolution_index]
                resolution_time = resolution_times[resolution_index]
                # Check resolution time constraint or if resolution is in unstable region. If violated, switch to 1a solution
                if resolution_time > 1.2 * turn_time_min or (maximum.size > 0 and resolution_index > maximum[0] and resolution_index < index):
                    if d_tmin > self.d_req:
                        resolution_type = '1a'
                        min_separation = d_tmin
                        resolution_angle = turn_angle_min
                        resolution_time = turn_time_min
                    else: resolution_type = '2'
            else: resolution_type = '2'

            # Give best resolution parameters to type 2 solutions
            if resolution_type == '2':
                min_separation = d_tmin
                resolution_time = turn_time_min
                try:
                    req_sep = min(i for i in separation[index:] if i > self.d_req)
                    resolution_index = separation.index(req_sep)
                except ValueError:
                    req_sep = np.array([])
                if req_sep.size > 0:
                    resolution_type = '2a'
                    resolution_angle = turn_angles[resolution_index]
                else:
                    resolution_type = '2b'
                    max_turn_separation = max(separation[index:])
                    resolution_index = separation.index(max_turn_separation)
                    resolution_angle = turn_angles[resolution_index]

            # Print table data
            print('A {} B right'.format(direction))
            # Print turn data
            print('\tMinimum Separation in Turn:')
            print('\t\tMinimum Separation (nmi):', round(d_tmin/1852, 2))
            print('\t\tTurn Angle (deg)', round(np.rad2deg(turn_angle_min), 2))
            print('\t\tTime (min):', round(turn_time_min/60, 2))
            # Prtint resolution data
            print('\tResolution Parameters:')
            print('\t\tResolution Type', resolution_type)
            print('\t\tMinimum Separation (nmi):', round(min_separation/1852, 2))
            print('\t\tResolution Angle (deg)', round(np.rad2deg(resolution_angle), 2))
            print('\t\tTime (min):', round(resolution_time/60, 2))

            # Prep plot data
            if i == 0:
                separation_right = separation
                separation_straight_right = separation_straight
                resolution_times_right = resolution_times
            else:
                separation_left = separation[::-1]
                separation_straight_left = separation_straight[::-1]
                resolution_times_left = resolution_times[::-1]

        # Plot maneuver data
        separation = np.array(separation_left + separation_right[1::]) / 1852
        separation_straight = np.array(separation_straight_left + separation_straight_right[1::]) / 1852
        resolution_times = np.array(resolution_times_left + resolution_times_right[1::])
        turn_angles = np.arange(-148,150,2)
        plt.plot(turn_angles, separation)
        plt.plot(turn_angles, separation_straight)
        plt.plot(turn_angles, np.array(resolution_times)/60)
        plt.title('A turns B right')
        plt.ylabel('Separation (nm)')
        plt.xlabel('Turn Angle (deg)')
        plt.grid()
        plt.show() 

    def A_turns_B_left(self):
        bank_angle = 30
        self.b.bank_angle_(np.deg2rad(-bank_angle))

        for i in range(2):
            if i == 0:
                direction = 'right'
                turn_angles = np.deg2rad(np.arange(0, 150, 2))
            else: 
                direction = 'left'
                bank_angle = -bank_angle
                turn_angles = np.deg2rad(np.arange(0, -150, -2))

            self.a.bank_angle_(np.deg2rad(bank_angle))

            # Find minimum turn separation, corresponding time and turn angle
            separation = []
            for turn_angle in turn_angles:
                separation.append(self.get_dab(self.a.time_to_turn(turn_angle)))
            index = separation.index(min(separation))
            d_tmin = separation[index]
            turn_angle_min = turn_angles[index]
            turn_time_min = self.a.time_to_turn(turn_angle_min)
            
            # Find time to minimum separation in straight line followed by minimum separation
            t_smin = []
            separation_straight = []
            for j, turn_angle in enumerate(turn_angles):
                t_smin.append(self.get_t_smin(turn_angle, self.b.turn_angle_(self.a.time_to_turn(turn_angle))))
                separation_straight.append(self.get_d_smin(turn_angle, self.b.turn_angle_(self.a.time_to_turn(turn_angle))))
                # Plots meet at min turn separation
                if  j > index: 
                    separation_straight[-1] = separation[j]

            # Find time to get to min separation overall w/ specified turn angle
            turn_times = []
            for turn_angle in turn_angles:
                turn_times.append(self.a.time_to_turn(turn_angle))
            resolution_times = []
            for [j, turn_time] in enumerate(turn_times):
                if t_smin[j] >= 0:
                    resolution_times.append(turn_time + t_smin[j])
                else:
                    resolution_times.append(turn_time)

            #### FIND RESOLUTION DATA ####

            # Check for any maxima prior to locus merge
            maximum = signal.argrelmax(np.array(separation_straight)) # Check if there is a local max
            maximum = maximum[0] # Isolate the index of the max
            
            # Try for type 1 resolution
            if d_tmin >= self.d_req or (maximum.size > 0 and separation_straight[maximum[0]] >= self.d_req):
                resolution_type = '1'
                min_separation = min(i for i in separation_straight if i > self.d_req)
                resolution_index = separation_straight.index(min_separation)
                resolution_angle = turn_angles[resolution_index]
                resolution_time = resolution_times[resolution_index]
                # Check resolution time constraint or if resolution is in unstable region. If violated, switch to 1a solution
                if resolution_time > 1.2 * turn_time_min or (maximum.size > 0 and resolution_index > maximum[0] and resolution_index < index):
                    if d_tmin > self.d_req:
                        resolution_type = '1a'
                        min_separation = d_tmin
                        resolution_angle = turn_angle_min
                        resolution_time = turn_time_min
                    else: resolution_type = '2'
            else: resolution_type = '2'

            # Give best resolution parameters to type 2 solutions
            if resolution_type == '2':
                min_separation = d_tmin
                resolution_time = turn_time_min
                try:
                    req_sep = min(i for i in separation[index:] if i > self.d_req)
                    resolution_index = separation.index(req_sep)
                except ValueError:
                    req_sep = np.array([])
                if req_sep.size > 0:
                    resolution_type = '2a'
                    resolution_angle = turn_angles[resolution_index]
                else:
                    resolution_type = '2b'
                    max_turn_separation = max(separation[index:])
                    resolution_index = separation.index(max_turn_separation)
                    resolution_angle = turn_angles[resolution_index]

            # Print table data
            print('A {} B left'.format(direction))
            # Print turn data
            print('\tMinimum Separation in Turn:')
            print('\t\tMinimum Separation (nmi):', round(d_tmin/1852, 2))
            print('\t\tTurn Angle (deg)', round(np.rad2deg(turn_angle_min), 2))
            print('\t\tTime (min):', round(turn_time_min/60, 2))
            # Prtint resolution data
            print('\tResolution Parameters:')
            print('\t\tResolution Type', resolution_type)
            print('\t\tMinimum Separation (nmi):', round(min_separation/1852, 2))
            print('\t\tResolution Angle (deg)', round(np.rad2deg(resolution_angle), 2))
            print('\t\tTime (min):', round(resolution_time/60, 2))

            # Prep plot data
            if i == 0:
                separation_right = separation
                separation_straight_right = separation_straight
                resolution_times_right = resolution_times
            else:
                separation_left = separation[::-1]
                separation_straight_left = separation_straight[::-1]
                resolution_times_left = resolution_times[::-1]

        # Plot maneuver data
        separation = np.array(separation_left + separation_right[1::]) / 1852
        separation_straight = np.array(separation_straight_left + separation_straight_right[1::]) / 1852
        resolution_times = np.array(resolution_times_left + resolution_times_right[1::])
        turn_angles = np.arange(-148,150,2)
        plt.plot(turn_angles, separation)
        plt.plot(turn_angles, separation_straight)
        plt.plot(turn_angles, np.array(resolution_times)/60)
        plt.title('A turns B left')
        plt.ylabel('Separation (nm)')
        plt.xlabel('Turn Angle (deg)')
        plt.grid()
        plt.show() 

def erz_2010_test_case_():

    # Create reference aircraft with set velocity
    a_speed = 400 # knots

    a_speed_mks = a_speed * 0.514 # m/s
    aircraft_a = MainAircraft(a_speed_mks)

    # Create conflicting aircraft with set velocity, relative position, and relative heading
    b_speed = 480 # knots
    b_position = np.array([12, 12.5]) # nmi
    b_heading = np.deg2rad(270) # deg

    b_speed_mks = b_speed * 0.514 # m/s
    b_position_mks = b_position * 1852 # m
    aircraft_b = ConflictingCraft(b_speed_mks, b_position_mks, b_heading)

    # Create conflict
    data = ManeuverData(aircraft_a, aircraft_b)

    # Testing for A turns B straight standard maneuver
    # data.A_turns_B_straight(15) # (TURN DATA WORKING)

    # Testing for A straight B turns standard maneuver 
    # data.A_straight_B_turns(15) # (TURN DATA WORKING)

    # Testing for A turns B straight expedited maneuver
    # data.A_turns_B_straight(30) # (TURN DATA WORKING)

    # Testing for A straight B turns expedited maneuver 
    # data.A_straight_B_turns(30) # (TURN DATA WORKING)

    # Testing for A turns B right expedited maneuver
    # data.A_turns_B_right() # (TURN DATA WORKING)

    # Testing for A turns B left expedited maneuver
    # data.A_turns_B_left() #(TURN DATA WORKING) (Note: A right slightly innacurate)

def erz_2010_test_case_2_():

    # Create reference aircraft with set velocity
    a_speed = 400 # knots

    a_speed_mks = a_speed * 0.514 # m/s
    aircraft_a = MainAircraft(a_speed_mks)

    # Create conflicting aircraft with set velocity, relative position, and relative heading
    b_speed = 480 # knots
    b_position = np.array([4, 5.83]) # nmi
    b_heading = np.deg2rad(270) # deg

    b_speed_mks = b_speed * 0.514 # m/s
    b_position_mks = b_position * 1852 # m
    aircraft_b = ConflictingCraft(b_speed_mks, b_position_mks, b_heading)

    # Create conflict
    data = ManeuverData(aircraft_a, aircraft_b)

    # Testing for A turns B straight standard maneuver
    # data.A_turns_B_straight(15) # (TURN DATA WORKING)

    # Testing for A straight B turns standard maneuver 
    # data.A_straight_B_turns(15) # (TURN DATA WORKING)

    # Testing for A turns B straight expedited maneuver
    # data.A_turns_B_straight(30) # (TURN DATA WORKING)

    # Testing for A straight B turns expedited maneuver 
    # data.A_straight_B_turns(30) # (TURN DATA WORKING)

    # Testing for A turns B right expedited maneuver
    # data.A_turns_B_right() # (TURN DATA WORKING)

    # Testing for A turns B left expedited maneuver
    # data.A_turns_B_left() #(TURN DATA WORKING) (Note: A right slightly innacurate)

# erz_2010_test_case_()
erz_2010_test_case_2_()
