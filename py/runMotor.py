import RPi.GPIO as GPIO
import rospy
from geometry_msgs.msg import Twist

# motor(directionPin, PwmPin, minSpeed, maxSpeed)
leftMotor = Motor(31, 32, 0.0, 100.0)
rightMotor = Motor(35, 33, 0.0, 100.0)

leftMotorPwmPin = leftMotor.pwmPin
rightMotorPwmPin = rightMotor.pwmPin

leftMotorDirection = leftMotor.direction
rightMotorDirection = rightMotor.direction

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)
GPIO.setup(35, GPIO.OUT)

leftMotorPWMPin = GPIO.PWM(32, 1000)
rightMotorPWMPin = GPIO.PWM(33, 1000)

leftWheel = leftMotor.speed
rightWheel = rightMotor.speed

def messageCallback(cmd_vel):
    linearVelocityX = cmd_vel.linear.x
    angularVelocityZ = cmd_vel.angular.z

    leftWheel = (linearVelocityX - angularVelocityZ) * 100.0
    rightWheel = (linearVelocityX + angularVelocityZ) * 100.0

    leftMotor.speed = leftWheel
    rightMotor.speed = rightWheel

    newLeftWheel = leftMotor.speed
    newRightWheel = rightMotor.speed

    leftMotorPWMPin.ChangeDutyCycle(newLeftWheel)
    rightMotorPWMPin.ChangeDutyCycle(newRightWheel)

    rospy.loginfo("------------------------------------")
    rospy.loginfo("Linear velocity: %f", linearVelocityX)
    rospy.loginfo("Angular velocity: %f", angularVelocityZ)
    rospy.loginfo("Left wheel speed: %f", leftWheel)
    rospy.loginfo("New left wheelSpeed: %f", newLeftWheel)
    rospy.loginfo("Right wheel speed: %f", rightWheel)
    rospy.loginfo("New right wheelSpeed: %f", newRightWheel)

    if linearVelocityX > 0 and angularVelocityZ < 0:
        rospy.loginfo("Forward right")
    elif linearVelocityX > 0 and angularVelocityZ > 0:
        rospy.loginfo("Forward left")
    elif linearVelocityX < 0 and angularVelocityZ < 0:
        rospy.loginfo("Backwards right")
    elif linearVelocityX < 0 and angularVelocityZ > 0:
        rospy.loginfo("Backwards left")
    elif linearVelocityX == 0 and angularVelocityZ > 0:
        rospy.loginfo("Left")
    elif linearVelocityX > 0 and angularVelocityZ == 0:
        rospy.loginfo("Forward")
    elif linearVelocityX < 0 and angularVelocityZ == 0:
        rospy.loginfo("Backwards")
    elif linearVelocityX == 0 and angularVelocityZ < 0:
        rospy.loginfo("Right")
    else:
        leftMotor.stop()
        rightMotor.stop()
        rospy.loginfo("Not moving")

    rospy.loginfo("------------------------------------")

def main():
    leftMotorPWMPin.start(0.0)
    rightMotorPWMPin.start(0.0)

    rospy.init_node("run_motor")
    rospy.Subscriber("cmd_vel", Twist, messageCallback)
    
    rospy.loginfo("Left pwm pin: %d", leftMotorPwmPin)
    rospy.loginfo("Right pwm pin: %d", rightMotorPwmPin)

    rospy.spin()

if __name__ == "__main__":
    main()
