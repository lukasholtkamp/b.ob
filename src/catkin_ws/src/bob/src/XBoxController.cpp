#include <XBoxController.h>

/**
 * @brief This program is used to control the robot using XBoxOne Controller
 * @details This program is used to control the robot using XBoxOne Controller. The program is used to control the robot in the following way:
 * 1. The left joystick is used to control either the rotating on the point or left and right movement of the robot.
 * 2. The rigt trigger is used to control the forward movement of the robot (accelerate).
 * 3. The left trigger is used to control the backward movement of the robot (decelerate).
 * 4. The A button is used to pause the movement.
 * 5. The B button is used to stop the movement.
 */

// Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp
/**
 * @brief This class is used to control the robot using XBoxOne Controller
 * @details Class reads Controller Inputs and calculates Velocity then publishes it to RunMotor.cpp 
 */
int gearChanger=5;
int rightButtonState = 0;
int leftButtonState = 0;
/**
 * @brief Constructor
 * @details This constructor is used to initialize the XBoxController class. The constructor is used to set the linear and angular values, the scale values and the button value.
 * 
 */
XB::XBoxController::XBoxController():
  linear(JOY_AXIS_RT),
  angular(ANGULAR_VEL)
{
  // Getting the parameters from the parameter server
  nh.param("axis_linear", linear, linear);
  nh.param("axis_angular", angular, angular);
  nh.param("scale_angular", aScale, aScale);
  nh.param("scale_linear", lScale, lScale);

  velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10); //<-- Publisher
  joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &XBoxController::joyCallback, this); //<-- Subscriber
}



/**
 * @brief Callback function for the joystick
 * @details This function is used to get the joystick values and publish the linear and angular values to the robot.
 * 
 * @param joy 
 */
void XB::XBoxController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){

  auto rightShoulderButton = joy->buttons[RIGHT_SHOULDER_BUTTON];
  auto leftShoulderButton = joy->buttons[LEFT_SHOULDER_BUTTON];
  auto bButton = joy->buttons[B_BUTTON]; // Getting the value of the B button
  auto yButton = joy->buttons[Y_BUTTON]; // Getting the value of the Y button
  auto xButton = joy->buttons[X_BUTTON];

  //Set Linear and Angular Velocity
  geometry_msgs::Twist twist;  //<-- Twist message
  twist.angular.z = aScale*joy->axes[ANGULAR_VEL]/gearChanger;  //<-- Angular value
  twist.linear.x = setLinearSpeed(twist, joy);  //<-- Linear Value

  //--Setting all of the Buttons--
  //Right shoulder button to increase Speed
  if(rightShoulderButton == 1){
    if(rightShoulderButton != rightButtonState){
      setGearChanger(gearChanger, RIGHT_SHOULDER_BUTTON);
    }
  }
  //Left shoulder button to decrease Speed
  if(leftShoulderButton == 1){
    if(leftShoulderButton != leftButtonState){
      setGearChanger(gearChanger, LEFT_SHOULDER_BUTTON);
    }
  }
  //B Button to exit the program
  if(bButton == 1){
    twist.linear.x = 0;
    twist.angular.z = 0;
    velPub.publish(twist);
    system("rosnode kill -a"); // Shutting down the program
  }
  //Y Button to set Speed to 0
  if(yButton == 1){
    twist.linear.x = 0;
    twist.angular.z = 0;
  }
  //X Button to stop the node
  if(xButton == 1){
    twist.linear.x = 0;
    twist.angular.z = 0;
    velPub.publish(twist); // Publishing the linear and angular values
    ros::shutdown();
  }


  // Publishing the linear and angular values
  velPub.publish(twist);
  // Setting Buttonstate for Button debouncing
  rightButtonState = rightShoulderButton;
  leftButtonState = leftShoulderButton;
}

double XB::XBoxController::setLinearSpeed(geometry_msgs::Twist twist, const sensor_msgs::Joy::ConstPtr& joy){
  int val = 0;
  switch(val){ // Switch case to control the linear value
    case JOY_AXIS_RT: // If the right trigger is pressed
      twist.linear.x = (((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2)); // Setting the linear value
      break;
    case JOY_AXIS_LT: // If the left trigger is pressed
      twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
      break;
    default: // If no trigger is pressed
      twist.linear.x = ((1+(lScale*joy->axes[JOY_AXIS_RT])*-1)/2)-((1+(lScale*joy->axes[JOY_AXIS_LT])*-1)/2); // Setting the linear value
  }
  return twist.linear.x/gearChanger;
}

//Set Gear up or down depending on which ShoulderButton is pressed
void XB::XBoxController::setGearChanger(int &gearChanger, int shoulderButton){
  if(shoulderButton == RIGHT_SHOULDER_BUTTON){
    if(gearChanger > 1){
      gearChanger-- ;
    }
  }
  if(shoulderButton == LEFT_SHOULDER_BUTTON){
    if(gearChanger < 5){
      gearChanger++ ;
    }
  }
}

/**
 * @brief Main function
 * @details This is the main function of the program. This function is used to initialize the ros node and the XBoxController class.
 *
 * @param argc
 * @param argv
 * @return int
 */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xbox_controller"); //<-- Initializing the ros node
  XB:: XBoxController xbox_controller; //<-- Initializing the XBoxController class
  ros::spin(); //<-- Spin
}
