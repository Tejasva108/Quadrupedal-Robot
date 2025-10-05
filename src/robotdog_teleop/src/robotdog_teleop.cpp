#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "robotdog_msgs/msg/keyboard_ctrl_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

// Reminder message
const char* msg = R"(
========================================
Robot Dog Keyboard Teleoperation
========================================
State Control:
  0 : Activate START (robot stands up)
  1 : Activate WALK (requires START)
  2 : Activate SIDE_MOVE (requires START)
  
Gait Control:
  g : Set gait_type to 1
  
Movement Control (requires WALK to be active):
  i : Move forward
  , : Move backward
  j : Turn left
  l : Turn right
  k : Stop movement
  
Gait Step Adjustment:
  w/s : Increase/Decrease step length X by 0.05
  a/d : Increase/Decrease step length Y by 0.05
  t/b : Increase/Decrease swing height by 0.5
  
  r : Reset gait_step to default values
  
Robot Pose Control:
  Arrow Up/Down    : Adjust height (+/- 0.02)
  Arrow Left/Right : Adjust yaw (+/- 0.1 rad)
  
Other:
  SPACE : Emergency stop (resets all states)
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

void printStatus(const robotdog_msgs::msg::KeyboardCtrlCmd& msg)
{
  printf("\r[START:%s | WALK:%s | SIDE:%s] Gait:%d | Step(%.2f,%.2f,%.2f) | Pose_H:%.2f Yaw:%.2f        ",
         msg.states[0] ? "ON" : "OFF",
         msg.states[1] ? "ON" : "OFF",
         msg.states[2] ? "ON" : "OFF",
         msg.gait_type,
         msg.gait_step.x,
         msg.gait_step.y,
         msg.gait_step.z,
         msg.pose.position.z,
         msg.pose.orientation.z);
  fflush(stdout);
}

int main(int argc, char** argv)
{
  // Node initialization
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("robotdog_teleop");
  
  // Publisher
  auto pub = node->create_publisher<robotdog_msgs::msg::KeyboardCtrlCmd>("/teleopkeyboard", 10);

  // Initialize message
  robotdog_msgs::msg::KeyboardCtrlCmd ctrl_msg;
  
  // Initialize states
  ctrl_msg.states[0] = false;  // start
  ctrl_msg.states[1] = false;  // walk
  ctrl_msg.states[2] = false;  // side_move
  
  // Initialize gait
  ctrl_msg.gait_type = 0;
  
  // Initialize gait_step (default values)
  ctrl_msg.gait_step.x = 0.0;   // steplen_x
  ctrl_msg.gait_step.y = 0.0;   // steplen_y
  ctrl_msg.gait_step.z = 0.05;  // swing_height
  
  // Initialize pose
  ctrl_msg.pose.position.x = 0.0;   // slant_x
  ctrl_msg.pose.position.y = 0.0;   // slant_y
  ctrl_msg.pose.position.z = 0.3;   // height
  ctrl_msg.pose.orientation.x = 0.0; // roll
  ctrl_msg.pose.orientation.y = 0.0; // pitch
  ctrl_msg.pose.orientation.z = 0.0; // yaw

  printf("%s", msg);
  printStatus(ctrl_msg);

  char key = ' ';

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
        if(!ctrl_msg.states[1])
        {
          ctrl_msg.gait_step.x = 0.0;
          ctrl_msg.gait_step.y = 0.0;
        }
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
    
    // Gait type control
    else if(key == 'g')
    {
      ctrl_msg.gait_type = 1;
      printf("\n[INFO] Gait type set to: 1\n");
    }
    
    // Movement control (requires WALK to be active)
    else if(key == 'i')
    {
      if(ctrl_msg.states[1])
      {
        ctrl_msg.gait_step.x = 0.3;  // Forward
        ctrl_msg.gait_step.y = 0.0;
      }
      else
      {
        printf("\n[WARN] WALK not active!\n");
      }
    }
    else if(key == ',')
    {
      if(ctrl_msg.states[1])
      {
        ctrl_msg.gait_step.x = -0.3;  // Backward
        ctrl_msg.gait_step.y = 0.0;
      }
      else
      {
        printf("\n[WARN] WALK not active!\n");
      }
    }
    else if(key == 'j')
    {
      if(ctrl_msg.states[1])
      {
        ctrl_msg.gait_step.x = 0.0;
        ctrl_msg.gait_step.y = 0.2;  // Turn left
      }
      else
      {
        printf("\n[WARN] WALK not active!\n");
      }
    }
    else if(key == 'l')
    {
      if(ctrl_msg.states[1])
      {
        ctrl_msg.gait_step.x = 0.0;
        ctrl_msg.gait_step.y = -0.2;  // Turn right
      }
      else
      {
        printf("\n[WARN] WALK not active!\n");
      }
    }
    else if(key == 'k')
    {
      ctrl_msg.gait_step.x = 0.0;
      ctrl_msg.gait_step.y = 0.0;
      printf("\n[INFO] Movement stopped\n");
    }
    
    // Gait step adjustments
    else if(key == 'w')
    {
      ctrl_msg.gait_step.x += 0.05;
      printf("\n[INFO] Step X increased: %.2f\n", ctrl_msg.gait_step.x);
    }
    else if(key == 's')
    {
      ctrl_msg.gait_step.x -= 0.05;
      printf("\n[INFO] Step X decreased: %.2f\n", ctrl_msg.gait_step.x);
    }
    else if(key == 'a')
    {
      ctrl_msg.gait_step.y += 0.05;
      printf("\n[INFO] Step Y increased: %.2f\n", ctrl_msg.gait_step.y);
    }
    else if(key == 'd')
    {
      ctrl_msg.gait_step.y -= 0.05;
      printf("\n[INFO] Step Y decreased: %.2f\n", ctrl_msg.gait_step.y);
    }
    else if(key == 't')
    {
      ctrl_msg.gait_step.z += 0.01;
      printf("\n[INFO] Swing height increased: %.2f\n", ctrl_msg.gait_step.z);
    }
    else if(key == 'b')
    {
      ctrl_msg.gait_step.z -= 0.01;
      if(ctrl_msg.gait_step.z < 0.01) ctrl_msg.gait_step.z = 0.01;
      printf("\n[INFO] Swing height decreased: %.2f\n", ctrl_msg.gait_step.z);
    }
    else if(key == 'r')
    {
      ctrl_msg.gait_step.x = 0.0;
      ctrl_msg.gait_step.y = 0.0;
      ctrl_msg.gait_step.z = 0.05;
      printf("\n[INFO] Gait step reset to defaults\n");
    }
    
    // Pose adjustments (arrow keys)
    else if(key == 'A')  // Arrow Up
    {
      ctrl_msg.pose.position.z += 0.02;
      printf("\n[INFO] Height increased: %.2f\n", ctrl_msg.pose.position.z);
    }
    else if(key == 'B')  // Arrow Down
    {
      ctrl_msg.pose.position.z -= 0.02;
      if(ctrl_msg.pose.position.z < 0.1) ctrl_msg.pose.position.z = 0.1;
      printf("\n[INFO] Height decreased: %.2f\n", ctrl_msg.pose.position.z);
    }
    else if(key == 'C')  // Arrow Right
    {
      ctrl_msg.pose.orientation.z -= 0.1;
      printf("\n[INFO] Yaw adjusted: %.2f\n", ctrl_msg.pose.orientation.z);
    }
    else if(key == 'D')  // Arrow Left
    {
      ctrl_msg.pose.orientation.z += 0.1;
      printf("\n[INFO] Yaw adjusted: %.2f\n", ctrl_msg.pose.orientation.z);
    }
    
    // Emergency stop
    else if(key == ' ')
    {
      ctrl_msg.states[0] = false;
      ctrl_msg.states[1] = false;
      ctrl_msg.states[2] = false;
      ctrl_msg.gait_step.x = 0.0;
      ctrl_msg.gait_step.y = 0.0;
      printf("\n[EMERGENCY] All states reset!\n");
    }
    
    // Quit
    else if(key == '\x03')  // CTRL-C
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