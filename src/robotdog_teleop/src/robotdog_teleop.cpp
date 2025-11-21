// Copyright 2024
// Licensed under the Apache License, Version 2.0

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "robotdog_msgs/msg/keyboard_ctrl_cmd.hpp"

// Robot limits
const float MIN_HEIGHT = 0.1;
const float MAX_HEIGHT = 0.5;
const float DEFAULT_HEIGHT = 0.3;
const float HEIGHT_STEP = 0.01;
const float ROLL_RANGE = 30.0;
const float PITCH_RANGE = 30.0;
const float YAW_RANGE = 30.0;
const float MAX_STEP_LENGTH_X = 0.3;
const float MAX_STEP_LENGTH_Y = 0.2;
const float STEP_HEIGHT_INCREMENT = 0.01;
const float SLANT_STEP = 0.01;
const float SLANT_X_MIN = -0.1;
const float SLANT_X_MAX = 0.1;
const float SLANT_Y_MIN = -0.1;
const float SLANT_Y_MAX = 0.1;

const char* msg = R"(
========================================
Robot Dog Keyboard Teleoperation
========================================
State Control:
  0 : Toggle START (robot stands up)
  1 : Toggle WALK (requires START)
  2 : Toggle SIDE_MOVE (requires START)
  
Gait Selection (requires START):
  a : Gait type 0
  b : Gait type 1
  x : Gait type 2
  y : Gait type 3
  
Height Control (requires START):
  Arrow Up   : Increase height
  Arrow Down : Decrease height
  
Step Height Control (requires START, hold 's'):
  s + Arrow Up   : Increase step height
  s + Arrow Down : Decrease step height
  
Orientation Control (requires START):
  w/s : Pitch (forward/backward tilt)
  a/d : Roll (left/right tilt)
  q/e : Yaw (rotate left/right)
  r   : Reset orientation to zero
  
Step Length Control (requires START):
  i/k : Forward/Backward step
  j/l : Left/Right step
  u   : Reset step length to zero
  
Slant Control (requires START, hold 'z'):
  z + w/s : Slant X
  z + a/d : Slant Y
  z + r   : Reset slant to zero

Other:
  SPACE  : Emergency stop (resets all states)
  CTRL-C : Quit program
========================================
)";

// For non-blocking keyboard inputs
int getch(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

void printStatus(const robotdog_msgs::msg::KeyboardCtrlCmd& cmd)
{
    printf("\r[START:%s WALK:%s SIDE:%s] Gait:%d | H:%.2f StepH:%.2f | R:%.1f P:%.1f Y:%.1f | Step(%.2f,%.2f) | Slant(%.2f,%.2f)     ",
           cmd.states[0] ? "ON" : "OFF",
           cmd.states[1] ? "ON" : "OFF",
           cmd.states[2] ? "ON" : "OFF",
           cmd.gait_type,
           cmd.pose.position.z,
           cmd.gait_step.z,
           cmd.pose.orientation.x,
           cmd.pose.orientation.y,
           cmd.pose.orientation.z,
           cmd.gait_step.x,
           cmd.gait_step.y,
           cmd.pose.position.x,
           cmd.pose.position.y);
    fflush(stdout);
}

int main(int argc, char** argv)
{
    // Node initialization
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robotdog_teleop_keyboard");
    
    // Publisher
    auto pub = node->create_publisher<robotdog_msgs::msg::KeyboardCtrlCmd>("/teleopkeyboard", 10);

    // Initialize message
    robotdog_msgs::msg::KeyboardCtrlCmd ctrl_msg;
    
    // Initialize states
    ctrl_msg.states.fill(false);
    ctrl_msg.gait_type = 0;
    
    // Initialize gait_step
    ctrl_msg.gait_step.x = 0.0;
    ctrl_msg.gait_step.y = 0.0;
    ctrl_msg.gait_step.z = 0.05;
    
    // Initialize pose
    ctrl_msg.pose.position.x = 0.0;   // slant_x
    ctrl_msg.pose.position.y = 0.0;   // slant_y
    ctrl_msg.pose.position.z = DEFAULT_HEIGHT;   // height
    ctrl_msg.pose.orientation.x = 0.0; // roll
    ctrl_msg.pose.orientation.y = 0.0; // pitch
    ctrl_msg.pose.orientation.z = 0.0; // yaw

    printf("%s", msg);
    printStatus(ctrl_msg);

    char key = ' ';
    bool step_height_mode = false;
    bool slant_mode = false;

    while(rclcpp::ok())
    {
        key = getch();

        // State control
        if(key == '0')
        {
            ctrl_msg.states[0] = !ctrl_msg.states[0];
            if(!ctrl_msg.states[0])
            {
                // If START is deactivated, deactivate WALK and SIDE_MOVE
                ctrl_msg.states[1] = false;
                ctrl_msg.states[2] = false;
                ctrl_msg.gait_step.x = 0.0;
                ctrl_msg.gait_step.y = 0.0;
            }
            printf("\n[INFO] START: %s\n", ctrl_msg.states[0] ? "ACTIVATED" : "DEACTIVATED");
        }
        else if(key == '1')
        {
            if(ctrl_msg.states[0])
            {
                ctrl_msg.states[1] = !ctrl_msg.states[1];
                printf("\n[INFO] WALK: %s\n", ctrl_msg.states[1] ? "ACTIVATED" : "DEACTIVATED");
            }
            else
            {
                printf("\n[WARN] Cannot activate WALK: START not active!\n");
            }
        }
        else if(key == '2')
        {
            if(ctrl_msg.states[0])
            {
                ctrl_msg.states[2] = !ctrl_msg.states[2];
                printf("\n[INFO] SIDE_MOVE: %s\n", ctrl_msg.states[2] ? "ACTIVATED" : "DEACTIVATED");
            }
            else
            {
                printf("\n[WARN] Cannot activate SIDE_MOVE: START not active!\n");
            }
        }
        
        // Only process other commands if START is active
        if(ctrl_msg.states[0])
        {
            // Gait type selection
            if(key == 'a') {
                ctrl_msg.gait_type = 0;
                printf("\n[INFO] Gait type: 0\n");
            }
            else if(key == 'b') {
                ctrl_msg.gait_type = 1;
                printf("\n[INFO] Gait type: 1\n");
            }
            else if(key == 'x') {
                ctrl_msg.gait_type = 2;
                printf("\n[INFO] Gait type: 2\n");
            }
            else if(key == 'y') {
                ctrl_msg.gait_type = 3;
                printf("\n[INFO] Gait type: 3\n");
            }
            
            // Mode keys
            else if(key == 's') {
                step_height_mode = true;
            }
            else if(key == 'z') {
                slant_mode = true;
            }
            
            // Arrow key sequences
            else if(key == 27) { // ESC sequence for arrow keys
                getch(); // skip '['
                key = getch();
                
                if(!step_height_mode && !slant_mode) {
                    // Height control
                    if(key == 'A') { // Up arrow
                        ctrl_msg.pose.position.z += HEIGHT_STEP;
                        if(ctrl_msg.pose.position.z > MAX_HEIGHT)
                            ctrl_msg.pose.position.z = MAX_HEIGHT;
                    }
                    else if(key == 'B') { // Down arrow
                        ctrl_msg.pose.position.z -= HEIGHT_STEP;
                        if(ctrl_msg.pose.position.z < MIN_HEIGHT)
                            ctrl_msg.pose.position.z = MIN_HEIGHT;
                    }
                }
                else if(step_height_mode) {
                    // Step height control
                    float max_step_h = ctrl_msg.pose.position.z - MIN_HEIGHT;
                    if(key == 'A') { // Up arrow
                        ctrl_msg.gait_step.z += STEP_HEIGHT_INCREMENT;
                        if(ctrl_msg.gait_step.z > max_step_h)
                            ctrl_msg.gait_step.z = max_step_h;
                    }
                    else if(key == 'B') { // Down arrow
                        ctrl_msg.gait_step.z -= STEP_HEIGHT_INCREMENT;
                        if(ctrl_msg.gait_step.z < 0.01)
                            ctrl_msg.gait_step.z = 0.01;
                    }
                    step_height_mode = false;
                }
            }
            
            // Orientation control
            else if(!slant_mode && key == 'w') {
                ctrl_msg.pose.orientation.y += 2.0;
                if(ctrl_msg.pose.orientation.y > PITCH_RANGE)
                    ctrl_msg.pose.orientation.y = PITCH_RANGE;
            }
            else if(!slant_mode && key == 's' && !step_height_mode) {
                ctrl_msg.pose.orientation.y -= 2.0;
                if(ctrl_msg.pose.orientation.y < -PITCH_RANGE)
                    ctrl_msg.pose.orientation.y = -PITCH_RANGE;
            }
            else if(!slant_mode && key == 'a') {
                ctrl_msg.pose.orientation.x -= 2.0;
                if(ctrl_msg.pose.orientation.x < -ROLL_RANGE)
                    ctrl_msg.pose.orientation.x = -ROLL_RANGE;
            }
            else if(!slant_mode && key == 'd') {
                ctrl_msg.pose.orientation.x += 2.0;
                if(ctrl_msg.pose.orientation.x > ROLL_RANGE)
                    ctrl_msg.pose.orientation.x = ROLL_RANGE;
            }
            else if(key == 'q') {
                ctrl_msg.pose.orientation.z -= 2.0;
                if(ctrl_msg.pose.orientation.z < -YAW_RANGE)
                    ctrl_msg.pose.orientation.z = -YAW_RANGE;
            }
            else if(key == 'e') {
                ctrl_msg.pose.orientation.z += 2.0;
                if(ctrl_msg.pose.orientation.z > YAW_RANGE)
                    ctrl_msg.pose.orientation.z = YAW_RANGE;
            }
            else if(!slant_mode && key == 'r') {
                ctrl_msg.pose.orientation.x = 0.0;
                ctrl_msg.pose.orientation.y = 0.0;
                ctrl_msg.pose.orientation.z = 0.0;
                printf("\n[INFO] Orientation reset\n");
            }
            
            // Step length control
            else if(key == 'i') {
                ctrl_msg.gait_step.x += 0.02;
                if(ctrl_msg.gait_step.x > MAX_STEP_LENGTH_X)
                    ctrl_msg.gait_step.x = MAX_STEP_LENGTH_X;
            }
            else if(key == 'k') {
                ctrl_msg.gait_step.x -= 0.02;
                if(ctrl_msg.gait_step.x < -MAX_STEP_LENGTH_X)
                    ctrl_msg.gait_step.x = -MAX_STEP_LENGTH_X;
            }
            else if(key == 'j') {
                ctrl_msg.gait_step.y += 0.02;
                if(ctrl_msg.gait_step.y > MAX_STEP_LENGTH_Y)
                    ctrl_msg.gait_step.y = MAX_STEP_LENGTH_Y;
            }
            else if(key == 'l') {
                ctrl_msg.gait_step.y -= 0.02;
                if(ctrl_msg.gait_step.y < -MAX_STEP_LENGTH_Y)
                    ctrl_msg.gait_step.y = -MAX_STEP_LENGTH_Y;
            }
            else if(key == 'u') {
                ctrl_msg.gait_step.x = 0.0;
                ctrl_msg.gait_step.y = 0.0;
                printf("\n[INFO] Step length reset\n");
            }
            
            // Slant control
            else if(slant_mode && key == 'w') {
                ctrl_msg.pose.position.x += SLANT_STEP;
                if(ctrl_msg.pose.position.x > SLANT_X_MAX)
                    ctrl_msg.pose.position.x = SLANT_X_MAX;
                slant_mode = false;
            }
            else if(slant_mode && key == 's') {
                ctrl_msg.pose.position.x -= SLANT_STEP;
                if(ctrl_msg.pose.position.x < SLANT_X_MIN)
                    ctrl_msg.pose.position.x = SLANT_X_MIN;
                slant_mode = false;
            }
            else if(slant_mode && key == 'a') {
                ctrl_msg.pose.position.y -= SLANT_STEP;
                if(ctrl_msg.pose.position.y < SLANT_Y_MIN)
                    ctrl_msg.pose.position.y = SLANT_Y_MIN;
                slant_mode = false;
            }
            else if(slant_mode && key == 'd') {
                ctrl_msg.pose.position.y += SLANT_STEP;
                if(ctrl_msg.pose.position.y > SLANT_Y_MAX)
                    ctrl_msg.pose.position.y = SLANT_Y_MAX;
                slant_mode = false;
            }
            else if(slant_mode && key == 'r') {
                ctrl_msg.pose.position.x = 0.0;
                ctrl_msg.pose.position.y = 0.0;
                printf("\n[INFO] Slant reset\n");
                slant_mode = false;
            }
            
            // Auto-adjust height when walking
            if(ctrl_msg.states[1] && ctrl_msg.pose.position.z < 0.1) {
                ctrl_msg.pose.position.z += 0.005;
                ctrl_msg.gait_step.z += 0.005;
            }
        }
        
        // Emergency stop
        if(key == ' ')
        {
            ctrl_msg.states[0] = false;
            ctrl_msg.states[1] = false;
            ctrl_msg.states[2] = false;
            ctrl_msg.gait_step.x = 0.0;
            ctrl_msg.gait_step.y = 0.0;
            printf("\n[EMERGENCY] All states reset!\n");
        }
        
        // Quit
        if(key == '\x03')  // CTRL-C
        {
            printf("\n\n[INFO] Shutting down teleop node...\n\n");
            break;
        }

        // Publish message
        pub->publish(ctrl_msg);
        printStatus(ctrl_msg);
        
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}