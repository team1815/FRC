/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1815.robot;


import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.NIVisionException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class IterativeBeast1815 extends IterativeRobot {
    
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick driveStick = new Joystick(1);
    AxisCamera camera = AxisCamera.getInstance();
    
    Compressor compressor = new Compressor(1,1);   //Compressor Relay
    Solenoid fast_shoot1_fwd = new Solenoid(1);      //Fire on Output 1
    Solenoid fast_shoot1_rev = new Solenoid(2);
    Solenoid fast_shoot2_fwd = new Solenoid(3);      //Fire on Output 3
    Solenoid fast_shoot2_rev = new Solenoid(4);
    Solenoid lim_switch_fwd = new Solenoid(5);
    Solenoid lim_switch_rev = new Solenoid(6);
    Solenoid pick_upper_down = new Solenoid(7);
    Solenoid pick_upper_up = new Solenoid(8);
    SolenoidToggle pickUpperControl = new SolenoidToggle(pick_upper_up, pick_upper_down);
    SolenoidToggle limSwitchControl = new SolenoidToggle(lim_switch_fwd, lim_switch_rev);
    ShooterThread shooterThread;
    
    class SolenoidToggle {
        boolean is_up = true;
        Timer timer = new Timer();
        double prevTime = 0;
        Solenoid fwd, rev;
        void toggle() {
            if (timer.get() - prevTime > .5) {
                fwd.set(is_up);
                rev.set(!is_up);
                is_up = !is_up;
                prevTime = timer.get();
                Log.log("Successful toggle");
            } else {
                Log.log("Unsuccessful toggle" + prevTime + ", " + timer.get());
            }
        }
        void start() {
            timer.reset();
            timer.start();
        }
        public boolean getIsUp() {
            return is_up;
        }
        public SolenoidToggle(Solenoid fwd, Solenoid rev) {
            this.fwd = fwd;
            this.rev = rev;
        }
    }
    
    private void stopAllPneumatics() {
        fast_shoot1_fwd.set(false);
        fast_shoot2_fwd.set(false);
        fast_shoot1_rev.set(false);
        fast_shoot2_rev.set(false);
        pick_upper_down.set(false);
        pick_upper_up.set(false);
        lim_switch_fwd.set(false);
        lim_switch_rev.set(false);
    }
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        
        stopAllPneumatics();
        compressor.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }
    
    public void teleopInit() {
        compressor.start();
        pickUpperControl.start();
        limSwitchControl.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        drive.arcadeDrive(driveStick, true);
        
        if (driveStick.getRawButton(6)) {
            pickUpperControl.toggle();
        }
        if (driveStick.getRawButton(7)) {
            limSwitchControl.toggle();
        }
        if (driveStick.getTrigger()){
            if (shooterThread == null || !shooterThread.isAlive()) {
                shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev);
                shooterThread.start();
                Log.log("Single shot");
            } else {
                Log.log("No single shot.");
            }
        }
        
    }
    
    public void disabledInit() {
        stopAllPneumatics();
        compressor.stop();
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
