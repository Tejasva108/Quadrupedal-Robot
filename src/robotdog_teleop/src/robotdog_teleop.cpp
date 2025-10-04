#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>
#include "robotdog_msgs/msg/keyboard_ctrl_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B

---------------------------
t : up (+z)
b : down (-z)
s/S : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
NOTE : Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
CTRL-C to quit
THIS IS A EXACT REPLICA OF https://github.com/ros-teleop/teleop_twist_keyboard 
WITH SOME ADD-ONS BUT IMPLEMENTED WITH C++ and ROS2-foxy.
)";

// Init variables
float x_length(0.5); // Linear velocity (m/s)
float y_length(0.0); // Angular velocity (rad/s)
float height_swing(5.0);
char key(' ');
bool start = false;
bool walk = false;
bool straf = false;

// For non-blocking keyboard inputs
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}


int main(int argc, char** argv){

  // node init
  rclcpp::init(argc,argv);
  auto node = rclcpp::Node::make_shared("teleop");
  // define publisher
  auto _pub = node->create_publisher<robotdog_msgs::msg::KeyboardCtrlMsg>("/teleopkeyboard", 10);

  robotdog_msgs::msg::KeyboardCtrlMsg Keyboardctrl;
  printf("%s", msg);
  printf("\nNow x_length is %f , y_length is %f and swing_height is %f | Last command: \n", x_length, y_length, height_swing);

  while(rclcpp::ok()){
    // get the pressed key
    key = getch();
// we have turned on the robot to just stand up
    if(key =='0'){
      start = true;
      states[0] == true;
      printf("The robot has stood up");
    }
// we have turned on the robot to be ready to walk
    if(key =='1' && start = true){
        walk = true;
        states[1] == true;
        printf("The robot is ready to walk");
    }
    if(key == 'i' && start = true && walk ==true){
        x_length = 0.5; // Linear velocity (m/s)
        y_length = 0.0; // Angular velocity (rad/s)
        height_swing = 5.0;  
    }
    if(key == ',' && start = true && walk ==true){
        x_length = -0.5; // Linear velocity (m/s)
        y_length = 0.0; // Angular velocity (rad/s)
        height_swing = 5.0;  
    }
    //'C' and 'D' represent the Right and Left arrow keys consecutively 
    else if(key=='C'||key=='D'){
      th = Avel(key,th);
      y = 0.0;
      z = 0.0;
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed*x, turn*th, key);
    }

    else if (moveBindings.count(key) == 1)
    {
      // Grab the direction data
      x = moveBindings[key][0];
      y = moveBindings[key][1];
      z = moveBindings[key][2];
      th = moveBindings[key][3];

      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed, turn, key);
    } 
    // Otherwise if it corresponds to a key in speedBindings
    else if (speedBindings.count(key) == 1)
    {
      // Grab the speed data
      speed = speed * speedBindings[key][0];
      turn = turn * speedBindings[key][1];

      printf("\nNow top Speed is %f and turn is %f | Last command: %c \n\t\tCurrent speed might be affected\n", speed, turn, key);
    }

    // Otherwise, set the robot to stop
    else
    { if (key=='s'||key=='S'){
      x = 0;
      y = 0;
      z = 0;
      th = 0;
      printf("\n\t\tRobot Stopped..!! \n");
      printf("\rCurrent: speed %f\tturn %f | Last command: %c   ", speed*x, turn*th, key);
    }
      // If ctrl-C (^C) was pressed, terminate the program
      else if (key == '\x03')
      {
        printf("\n\n    ☺  Give it a Star :: https://github.com/1at7/teleop_cpp_ros2 ☺ \n\n");
        break;
      }
      else
        printf("\rCurrent: speed %f\tturn %f | Invalid command! %c", speed*x, turn*th, key);
    }

    // Update the Twist message
    twist.linear.x = x * speed;
    twist.linear.y = y * speed;
    twist.linear.z = z * speed;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = th * turn;
    

    _pub->publish(twist);
    rclcpp::spin_some(node);  

  }
  return 0;

}