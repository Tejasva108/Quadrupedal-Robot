#!/usr/bin/env python3
"""
MINIMAL ROBOT DOG WALKING DEMO
===============================
Just run this script and the robot will walk!

Usage:
    python3 minimal_robotdog_demo.py

What it does:
- Automatically starts trot gait
- Moves legs in walking pattern
- Publishes joint angles to ROS2 topic
- No keyboard needed - just runs!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
a
# ============================================================================
# CONFIGURATION - CHANGE THESE IF NEEDED
# ============================================================================
ROBOT_HEIGHT = 120.0        # Standing height (mm)
STEP_LENGTH_X = 0.3         # Forward step length
STEP_LENGTH_Y = 0.0         # Sideways step length (0 = straight)
SWING_HEIGHT = 0.05         # How high feet lift (mm)
CYCLE_TIME = 0.8            # Time for one complete gait cycle (seconds)
SWING_TIME_RATIO = 0.5      # Portion of cycle spent swinging (0.5 = 50%)

# Leg dimensions (mm)
L1 = 50   # Hip offset
L2 = 150  # Upper leg
L3 = 150  # Lower leg

# Initial joint angles (degrees from horizontal)
INITIAL_UPPER_ANGLE = 20.0  # θ1 - upper link angle from horizontal
INITIAL_LOWER_ANGLE = 20.0  # θ2 - lower link angle from horizontal

# ============================================================================
# FORWARD KINEMATICS - Calculate foot position from joint angles
# ============================================================================
def forward_kinematics(th0, th1, th2_rel):
    """
    Calculate foot position from joint angles
    
    INPUT: 
        th0 - Hip abduction (radians)
        th1 - Hip pitch from horizontal (radians)
        th2_rel - Knee angle relative to upper link (radians)
    
    OUTPUT: [x, y, z] - foot position (mm)
    """
    # Calculate absolute lower link angle
    th2 = th1 + th2_rel
    
    # Forward kinematics
    x = L2 * np.cos(th1) + L3 * np.cos(th2)
    y = L1 * np.cos(th0)
    z = L1 * np.sin(th0) + L2 * np.sin(th1) + L3 * np.sin(th2)
    
    return [x, y, z]

# ============================================================================
# INVERSE KINEMATICS (Simplified)
# ============================================================================
def inverse_kinematics(x, y, z):
    """
    Calculate joint angles for desired foot position
    
    INPUT: x, y, z - foot position (mm)
    OUTPUT: [θ0, θ1, θ2] - joint angles (degrees)
    """
    # Check if reachable
    r = np.sqrt(x**2 + y**2 + z**2)
    if r < 50 or r > 350:  # Simple workspace check
        return [0, 45, 90]  # Return safe default
    
    try:
        # θ0 - Hip abduction
        r_yz = np.sqrt(y**2 + z**2)
        if r_yz < L1:
            return [0, 45, 90]
        
        alpha = np.arccos(y / r_yz)
        th0 = np.arccos(L1 / r_yz) - alpha
        
        # θ1 - Hip pitch
        a = 2 * L2 * r_yz * np.sin(th0 + alpha)
        b = 2 * L2 * x
        c = x**2 + L2**2 - L3**2 + (a/(2*L2))**2
        d = np.sqrt(a**2 + b**2)
        
        beta = np.arccos(np.clip(a/d, -1, 1))
        if x < 0:
            beta = -beta
        
        th1 = beta + np.arcsin(np.clip(c/d, -1, 1))
        
        # θ2 - Knee
        th2 = np.arccos(np.clip((x + L2*np.cos(th1))/L3, -1, 1))
        
        # Convert to degrees
        angles_deg = [np.rad2deg(th0), np.rad2deg(th1), np.rad2deg(th2)]
        return angles_deg
        
    except:
        return [0, 45, 90]  # Safe fallback


# ============================================================================
# GAIT TRAJECTORY GENERATOR
# ============================================================================
def trot_trajectory(leg_name, t, cycle_time, swing_time):
    """
    Generate trot gait trajectory for one leg
    
    TROT PATTERN:
    - FR & BL swing together, FL & BR stance
    - Then FL & BR swing together, FR & BL stance
    
    INPUT:
        leg_name: 'FR', 'FL', 'BR', or 'BL'
        t: current time in cycle (0 to cycle_time)
        cycle_time: total cycle duration
        swing_time: duration of swing phase
    
    OUTPUT:
        [dx, dy, dz] - offset from standing position
    """
    stance_time = cycle_time - swing_time
    
    # Determine if this leg is swinging or in stance
    if leg_name in ['FR', 'BL']:
        # FR and BL swing first
        if t <= swing_time:
            # SWING phase
            phase = t / swing_time
            dx = -STEP_LENGTH_X/2 + STEP_LENGTH_X * phase
            dy = -STEP_LENGTH_Y/2 + STEP_LENGTH_Y * phase
            dz = -SWING_HEIGHT * np.sin(phase * np.pi)
        else:
            # STANCE phase
            phase = (t - swing_time) / stance_time
            dx = STEP_LENGTH_X/2 - STEP_LENGTH_X * phase
            dy = STEP_LENGTH_Y/2 - STEP_LENGTH_Y * phase
            dz = 0
    else:
        # FL and BR swing second
        if t <= stance_time:
            # STANCE phase (while FR/BL swing)
            phase = t / stance_time
            dx = STEP_LENGTH_X/2 - STEP_LENGTH_X * phase
            dy = STEP_LENGTH_Y/2 - STEP_LENGTH_Y * phase
            dz = 0
        else:
            # SWING phase
            phase = (t - stance_time) / swing_time
            dx = -STEP_LENGTH_X/2 + STEP_LENGTH_X * phase
            dy = -STEP_LENGTH_Y/2 + STEP_LENGTH_Y * phase
            dz = -SWING_HEIGHT * np.sin(phase * np.pi)
    
    return [dx, dy, dz]


# ============================================================================
# ROS2 NODE
# ============================================================================
class MinimalRobotDog(Node):
    def __init__(self):
        super().__init__('minimal_robotdog')
        
        # Publisher for joint commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/gazebo_joint_controller/commands',
            10
        )
        
        # Timer - runs at 50Hz (every 0.02 seconds)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        # Gait state
        self.start_time = time.time()
        self.cycle_time = CYCLE_TIME
        self.swing_time = CYCLE_TIME * SWING_TIME_RATIO
        
        # Calculate initial foot position from specified joint angles
        # Hip abduction (th0) = 0 for neutral stance
        # Upper link angle (th1) = 20° from horizontal
        # Lower link angle = 20° from horizontal, so relative angle = 0°
        th0_rad = np.deg2rad(0)
        th1_rad = np.deg2rad(INITIAL_UPPER_ANGLE)
        th2_rel_rad = np.deg2rad(INITIAL_LOWER_ANGLE - INITIAL_UPPER_ANGLE)
        
        initial_pos = forward_kinematics(th0_rad, th1_rad, th2_rel_rad)
        
        # Standing position for each foot [x, y, z]
        self.foot_zero = {
            'FR': initial_pos.copy(),
            'FL': initial_pos.copy(),
            'BR': initial_pos.copy(),
            'BL': initial_pos.copy()
        }
        
        self.get_logger().info('='*60)
        self.get_logger().info('ROBOT DOG MINIMAL DEMO STARTED!')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Initial upper link angle: {INITIAL_UPPER_ANGLE}°')
        self.get_logger().info(f'Initial lower link angle: {INITIAL_LOWER_ANGLE}°')
        self.get_logger().info(f'Initial foot position: x={initial_pos[0]:.1f}, y={initial_pos[1]:.1f}, z={initial_pos[2]:.1f} mm')
        self.get_logger().info(f'Step length: {STEP_LENGTH_X} mm')
        self.get_logger().info(f'Cycle time: {CYCLE_TIME} s')
        self.get_logger().info('Robot will start walking in TROT gait...')
        self.get_logger().info('='*60)
    
    def control_loop(self):
        """Main control loop - runs 50 times per second"""
        
        # Get current time in gait cycle
        elapsed = time.time() - self.start_time
        t = elapsed % self.cycle_time  # Time within current cycle
        
        # Calculate trajectory offsets for each leg
        traj_FR = trot_trajectory('FR', t, self.cycle_time, self.swing_time)
        traj_FL = trot_trajectory('FL', t, self.cycle_time, self.swing_time)
        traj_BR = trot_trajectory('BR', t, self.cycle_time, self.swing_time)
        traj_BL = trot_trajectory('BL', t, self.cycle_time, self.swing_time)
        
        # Calculate actual foot positions
        pos_FR = [self.foot_zero['FR'][i] + traj_FR[i] for i in range(3)]
        pos_FL = [self.foot_zero['FL'][i] + traj_FL[i] for i in range(3)]
        pos_BR = [self.foot_zero['BR'][i] + traj_BR[i] for i in range(3)]
        pos_BL = [self.foot_zero['BL'][i] + traj_BL[i] for i in range(3)]
        
        # Calculate joint angles using IK
        ang_FR = inverse_kinematics(*pos_FR)
        ang_FL = inverse_kinematics(*pos_FL)
        ang_BR = inverse_kinematics(*pos_BR)
        ang_BL = inverse_kinematics(*pos_BL)
        
        # Prepare message - MUST match the joint order in controller config
        # Order: FL_Hip, FL_Upper, FL_Lower, FR_Hip, FR_Upper, FR_Lower,
        #        BL_Hip, BL_Upper, BL_Lower, BR_Hip, BR_Upper, BR_Lower
        msg = Float64MultiArray()
        msg.data = [
            float(ang_FL[0]), float(ang_FL[1]), float(ang_FL[1] + ang_FL[2]),  # FL
            float(ang_FR[0]), float(ang_FR[1]), float(ang_FR[1] + ang_FR[2]),  # FR
            float(ang_BL[0]), float(ang_BL[1]), float(ang_BL[1] + ang_BL[2]),  # BL
            float(ang_BR[0]), float(ang_BR[1]), float(ang_BR[1] + ang_BR[2])   # BR
        ]
        
        # Publish to controller
        self.publisher.publish(msg)
        
        # Print status every second
        if int(elapsed) != int(elapsed - 0.02):
            cycle_num = int(elapsed / self.cycle_time)
            self.get_logger().info(
                f'Cycle {cycle_num} | Time: {t:.2f}s | '
                f'FR_z: {pos_FR[2]:.1f} | FL_z: {pos_FL[2]:.1f}'
            )


# ============================================================================
# MAIN - JUST RUN THIS!
# ============================================================================
def main(args=None):
    print("\n" + "="*60)
    print("STARTING ROBOT DOG MINIMAL DEMO")
    print("="*60)
    print("The robot will automatically start walking!")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    rclpy.init(args=args)
    node = MinimalRobotDog()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nStopping robot...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Demo stopped!")


if __name__ == '__main__':
    main()