import numpy as np

class LegStates():
    prev_coord = None
    now_coord  = None
    prev_angles = None
    now_angles = None
    in_singularity = None

class Legs():
    FR = LegStates()
    FL = LegStates()
    BR = LegStates()
    BL = LegStates()

class InverseKinematics():
    def __init__(self):
        self.L1 = 0.6
        self.L2 = 0.23
        self.L3 = 0.23
        self.BODY_LENGTH = 0.44
        self.BODY_WIDTH  = 0.24
        self.MIN_ANG_L2L3 = 50
        self.MAX_ANG_L2L3 = 130
        self.MIN_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MIN_ANG_L2L3))))
        self.MAX_LEG_R   = np.sqrt(self.L1**2 + (self.L2**2 + self.L3**2 - 2*self.L2*self.L3*np.cos(np.deg2rad(self.MAX_ANG_L2L3))))

        self.M_R = np.array([1, 1, 1])
        self.M_L = np.array([1, -1, 1])
        self.M_F = np.array([1, 1, 1])
        self.M_B = np.array([-1, 1, 1])
        self.BODY_SCALE = np.array([self.BODY_LENGTH/2, self.BODY_WIDTH/2, 0])

        self.legState = Legs()
        self.singularity = [0,0,0,0]

    def rotMat(self, eularAng):
        """
            eularAng: np array [roll, pitch, yaw]
            return : rotation matrix
        """
        M11 = np.cos(eularAng[1])*np.cos(eularAng[2])
        M12 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) - np.cos(eularAng[0])*np.sin(eularAng[2])
        M13 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.cos(eularAng[2]) + np.sin(eularAng[0])*np.sin(eularAng[2])

        M21 = np.cos(eularAng[1])*np.sin(eularAng[2])
        M22 = np.sin(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) + np.cos(eularAng[0])*np.cos(eularAng[2])
        M23 = np.cos(eularAng[0])*np.sin(eularAng[1])*np.sin(eularAng[2]) - np.sin(eularAng[0])*np.cos(eularAng[2])
        
        M31 = -np.sin(eularAng[1])
        M32 = np.sin(eularAng[0])*np.cos(eularAng[1])
        M33 = np.cos(eularAng[0])*np.cos(eularAng[1])
        rotMat = np.array([ [M11, M12, M13], [M21, M22, M23], [M31, M32, M33] ])
        return rotMat

    def get_joint_angles(self, coord, leg_side):
        """
            coord: np array with desired leg coordinate -> np.array([x, y, z])
            leg_side: mirroring array for left/right legs
            return: np array with joint angles -> np.array([th0, th1, th2])
        """
        x = coord[0]
        y = coord[1]
        z = coord[2]
        joint_ang = np.zeros(3)
        
        # Check basic singularity
        if self.is_singularity(coord):
            return joint_ang
        
        try:
            # Step 1: Solve Joint 0 (Hip joint in YZ plane)
            r_yz = np.sqrt(y**2 + z**2)
            
            # Check hip singularity
            if r_yz < self.L1 + 0.001:
                return joint_ang
            
            # Calculate the hip angle using geometry
            # th0 is the angle to rotate so L1 points toward the target
            th0 = np.arcsin(self.L1 / r_yz)
            
            # th1 is the angle from Y-axis to the target
            th1 = np.arctan2(z, y)
            
            # The final hip angle (Joint 0)
            # This depends on the leg side and quadrant
            # if leg_side[1] == 1:  # Right side
            th_final_0 = (np.pi)/2 -( th1 + th0)
            # else:  # Left side
            #     th_final_0 = th1 + th0
            
            joint_ang[0] = th_final_0
            
            # Step 2: Transform coordinates to the hip frame (after hip rotation)
            # This is your geometric transformation approach!
            # We rotate the target point by -th_final_0 and account for L1 offset
            
            changed_coord = np.zeros(3)
            changed_coord[0] = x  # X doesn't change with hip rotation
            
            # Rotate Y and Z by the hip angle and add L1 offset
            # This gives us the position in the shoulder joint frame
            changed_coord[1] = y * np.cos(th_final_0) - z * np.sin(th_final_0) - self.L1
            changed_coord[2] = y * np.sin(th_final_0) + z * np.cos(th_final_0)
            
            # Step 3: Solve the 2-link planar arm (L2-L3)
            # Now working in the plane defined by changed_coord
            
            # Project onto XZ plane (in the hip frame, Y is "out of plane")
            x_plane = changed_coord[0]
            z_plane = changed_coord[2]
            
            # Distance to target in this plane
            r_plane = np.sqrt(x_plane**2 + z_plane**2)
            
            # Check reachability
            if r_plane > (self.L2 + self.L3) - 0.001:
                return joint_ang
            if r_plane < abs(self.L2 - self.L3) + 0.001:
                return joint_ang
            
            # Angle from horizontal to the target
            alpha = np.arctan2(z_plane, x_plane)
            
            # Law of cosines for the elbow angle (Joint 2)
            cos_gamma = (self.L2**2 + self.L3**2 - r_plane**2) / (2 * self.L2 * self.L3)
            cos_gamma = np.clip(cos_gamma, -1.0, 1.0)
            gamma = np.arccos(cos_gamma)
            
            # Check joint limits
            gamma_deg = np.rad2deg(gamma)
            if gamma_deg < self.MIN_ANG_L2L3 or gamma_deg > self.MAX_ANG_L2L3:
                return joint_ang
            
            # Joint 2 angle (knee/elbow)
            th_final_2 = np.pi - gamma
            joint_ang[2] = th_final_2
            
            # Law of cosines for the shoulder angle (Joint 1)
            cos_beta = (self.L2**2 + r_plane**2 - self.L3**2) / (2 * self.L2 * r_plane)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos(cos_beta)
            
            # Joint 1 angle (shoulder/femur)
            th_final_1 = alpha - beta
            joint_ang[1] = th_final_1
            
        except (ValueError, RuntimeWarning, FloatingPointError) as e:
            print(f"IK calculation error: {e}")
            return np.zeros(3)
        
        return joint_ang

    def get_FR_joint_angles(self, coord, eularAng):
        if self.is_singularity(coord):
            self.singularity[0] = True
        else:
            self.singularity[0] = False

        if any(self.singularity):
            if self.legState.FR.prev_angles is not None:
                self.legState.FR.now_angles = self.legState.FR.prev_angles
            else:
                self.legState.FR.now_angles = np.zeros(3)
        else:
            new_angles = self.get_joint_angles(coord, self.M_R)
            if new_angles is not None:
                self.legState.FR.now_angles = new_angles
                self.legState.FR.prev_angles = new_angles
                self.singularity[0] = False
        
        return self.legState.FR.now_angles

    def get_FL_joint_angles(self, coord, eularAng):
        if self.is_singularity(coord):
            self.singularity[1] = True
        else:
            self.singularity[1] = False
        
        if any(self.singularity):
            if self.legState.FL.prev_angles is not None:
                self.legState.FL.now_angles = self.legState.FL.prev_angles
            else:
                self.legState.FL.now_angles = np.zeros(3)
        else:
            new_angles = self.get_joint_angles(coord, self.M_L)
            if new_angles is not None:
                self.legState.FL.now_angles = new_angles
                self.legState.FL.prev_angles = new_angles
                self.singularity[1] = False
        
        return self.legState.FL.now_angles

    def get_BR_joint_angles(self, coord, eularAng):
        if self.is_singularity(coord):
            self.singularity[2] = True
        else:
            self.singularity[2] = False
        
        if any(self.singularity):
            if self.legState.BR.prev_angles is not None:
                self.legState.BR.now_angles = self.legState.BR.prev_angles
            else:
                self.legState.BR.now_angles = np.zeros(3)
        else:
            new_angles = self.get_joint_angles(coord, self.M_R)
            if new_angles is not None:
                self.legState.BR.now_angles = new_angles
                self.legState.BR.prev_angles = new_angles
                self.singularity[2] = False
        
        return self.legState.BR.now_angles

    def get_BL_joint_angles(self, coord, eularAng):
        if self.is_singularity(coord):
            self.singularity[3] = True
        else:
            self.singularity[3] = False
        
        if any(self.singularity):
            if self.legState.BL.prev_angles is not None:
                self.legState.BL.now_angles = self.legState.BL.prev_angles
            else:
                self.legState.BL.now_angles = np.zeros(3)
        else:
            new_angles = self.get_joint_angles(coord, self.M_L)
            if new_angles is not None:
                self.legState.BL.now_angles = new_angles
                self.legState.BL.prev_angles = new_angles
                self.singularity[3] = False
        
        return self.legState.BL.now_angles

    def is_singularity(self, coord):
        """Check if coordinate is within reachable workspace"""
        r = np.sqrt(coord[0]**2 + coord[1]**2 + coord[2]**2)
        if r >= self.MIN_LEG_R and r <= self.MAX_LEG_R:
            return False
        else:
            return True