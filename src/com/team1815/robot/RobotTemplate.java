/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1815.robot;


import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends SimpleRobot {
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick leftStick = new Joystick(1);
    AxisCamera camera = AxisCamera.getInstance();
    //Joystick rightStick = new Joystick(2);
    
    /**
     * This function is called once.
     */
    public void robotInit() {
        super.robotInit();
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
    }
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        //they really recommend against doing this
        //http://thedailywtf.com/Articles/_0x2f__0x2f_TODO_0x3a__Uncomment_Later.aspx
        //but I think it might be unavoidable
        getWatchdog().setEnabled(false);
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            //drive.tankDrive(leftStick, rightStick); // drive with joysticks
            drive.arcadeDrive(leftStick);
            Timer.delay(0.005);
            if (leftStick.getTrigger() && camera.freshImage()) {
                try {
                    camera.getImage();
                    Log.log("Took a picture");
                } catch (AxisCameraException ex) {
                    ex.printStackTrace();
                } catch (NIVisionException ex) {
                    ex.printStackTrace();
                }
            }
        }
    }
    
    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {
    
    }
}
