/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1815.robot;


import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class IterativeBeast1815 extends IterativeRobot {
    
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick driveStick1 = new Joystick(1);
    Joystick driveStick2 = new Joystick(2);
    AxisCamera camera = AxisCamera.getInstance();
    AxisCamera launcherCamera = AxisCamera.getInstance("10.18.15.12");
    int hotCount = 0; //if reaches 10, target is hot and go to score
    int loopCount = 0; //makes sure goal is hot 9/10 times
    
    //guys a DoubleSolenoid might've been what we wanted
    Compressor compressor = new Compressor(1,1);   //Compressor Relay
    Solenoid fast_shoot1_fwd = new Solenoid(1);      //Fire on Output 1
    Solenoid fast_shoot1_rev = new Solenoid(2);
    Solenoid fast_shoot2_fwd = new Solenoid(3);      //Fire on Output 3
    Solenoid fast_shoot2_rev = new Solenoid(4);
    Solenoid side_grabber_fwd = new Solenoid(5);
    Solenoid side_grabber_rev = new Solenoid(6);
    Solenoid pick_upper_down = new Solenoid(7);
    Solenoid pick_upper_up = new Solenoid(8);
    SolenoidToggle pickUpperControl = new SolenoidToggle(pick_upper_up, pick_upper_down);
    SolenoidToggle sideGrabberControl = new SolenoidToggle(side_grabber_fwd, side_grabber_rev);
    ShooterThread shooterThread;
    
    DigitalInput ball_lim_switch = new DigitalInput(2);
    
    Victor launch_adjuster = new Victor(5);
    DigitalInput launch_adjuster_lim_switch = new DigitalInput(3);
    TimedMotor launch_adjuster_timer = new TimedMotor(launch_adjuster);
    
    private boolean forward_is_pickupper = false;
    private boolean directionChanged = false;
    
    PWM camera_light = new PWM(10);
    
    VisionProcessor visionProcessor = new VisionProcessor(camera);
    
    boolean our_side_is_hot;
    
    /**
     * Simple toggle for a solenoid. Ensures that it it doesn't switch back and forth too quickly
     * if the button is held down for more than one iteration.
     */
    class SolenoidToggle {
        boolean is_up = true;
        Timer timer = new Timer();
        double prevTime = 0;
        Solenoid fwd, rev;
        
        void toggle() {
            if (timer.get() - prevTime > .5 || timer.get() < prevTime) {
                fwd.set(is_up);
                rev.set(!is_up);
                is_up = !is_up;
                prevTime = timer.get();
                Log.log("Successful toggle");
            } else {
                Log.log("Unsuccessful toggle" + prevTime + ", " + timer.get());
            }
        }
        
        void putUpNoMatterWhat() {
            fwd.set(false);
            rev.set(true);
            is_up = true;
        }
        
        double timeSince() {
            if (timer.get() > prevTime)
                return timer.get() - prevTime;
            else
                return 0;
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
        side_grabber_fwd.set(false);
        side_grabber_rev.set(false);
    }
    
    private void reverseMotors(boolean forward) {
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, forward);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, forward);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, forward);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, forward);
    }
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        reverseMotors(true);
        
        stopAllPneumatics();
        compressor.start();
        pickUpperControl.start();
        visionProcessor.robotInit();
    }
    
    
    public void autonomousInit() {
        compressor.start();
        pickUpperControl.start();
        camera_light.setRaw(255);
        visionProcessor.autonomousInit();
        
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        loopCount++;
        //checks image continuously until target is hot for at least 9/10 checks
        if (visionProcessor.autonomousPeriodic(null)) {
            if (loopCount <= 10 && ++hotCount >= 9) {
                //score
            }
            else if (loopCount == 10 && ++hotCount < 9) {
                //reset loopCount and hotCount for next check
                loopCount = 0;
                hotCount = 0;
            }
        }
    }
    
    public void teleopInit() {
        compressor.start();
        pickUpperControl.start();
        sideGrabberControl.start();
        getWatchdog().setEnabled(false); //TODO: true later
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {

        //Log.log(compressor.getPressureSwitchValue()? "yes" : "no");
//        if(compressor.getPressureSwitchValue() && compressor.enabled()){
//            compressor.stop();
//            Log.log("Shut off compressor");
//        }else if (!compressor.getPressureSwitchValue() && !compressor.enabled()){
//            compressor.start();
//            Log.log("Turned on comrpessor");
//        }
        
        double left = driveStick2.getY();
        double right = driveStick1.getY();
        if (driveStick1.getTop()) {
            left *= .4;
            right *= .4;
        }
        //only change the direction if the time since the last press was over 
        if (forward_is_pickupper) {
            drive.tankDrive(left, right);
        } else {
            drive.tankDrive(-right, -left);
        }
                
        
        
        if (driveStick1.getRawButton(6)) {
            pickUpperControl.toggle();
        }
        if (ball_lim_switch.get()) {
            pickUpperControl.putUpNoMatterWhat();
            Log.log("Limit switch hit");
        }
        
        if (driveStick1.getRawButton(11)) {
            if (!directionChanged) {
                forward_is_pickupper = !forward_is_pickupper;
                directionChanged = true;
            }
        } else {
            directionChanged = false;
        }
        if (driveStick1.getRawButton(3)) {
            sideGrabberControl.toggle();
        }
        if (driveStick1.getTrigger() && sideGrabberControl.getIsUp()){
            if (shooterThread == null || !shooterThread.isAlive()) {
                shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, 1);
                shooterThread.start();
                Log.log("Single shot");
            } else {
                Log.log("No single shot.");
            }
        }
        if (driveStick2.getTrigger() && sideGrabberControl.getIsUp()) {
            if (shooterThread == null || !shooterThread.isAlive()) {
                shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, .2);
                shooterThread.start();
                Log.log("Single weak shot");
            } else {
                Log.log("No single weak shot.");
            }
        }
        if (driveStick1.getRawButton(8)) {
            launch_adjuster.set(1);
            //launch_adjuster_timer.go(10, true);
        } else if (driveStick1.getRawButton(9)) {
            launch_adjuster.set(-1);
            //launch_adjuster_timer.go(10, false);
        } else {
            launch_adjuster.set(0);
        }
        
    }
    
    public void disabledInit() {
        camera_light.setRaw(0);
        stopAllPneumatics();
        compressor.stop();
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
}
