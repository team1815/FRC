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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class IterativeBeast1815 extends IterativeRobot {
    
    final static double Kp = 0.01;
    final static double Ki = 0;
    final static double Kd = 0.05;
    
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick driveStick1 = new Joystick(1);
    Joystick driveStick2 = new Joystick(2);
    AxisCamera camera = AxisCamera.getInstance();
    AxisCamera launcherCamera = AxisCamera.getInstance("10.18.15.12");
    
    //Autonomous only
    int hotCount = 0; //if reaches 10, target is hot and go to score
    int loopCount = 0; //makes sure goal is hot 9/10 times
    static int autonomousMove = State.GO_FORWARD;
    static int autonomousShoot = State.NOT_SHOT;
    
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
    SolenoidToggle pickUpperControl = new SolenoidToggle(pick_upper_up, pick_upper_down, this);
    SolenoidToggle topGrabberControl = new SolenoidToggle(side_grabber_fwd, side_grabber_rev, this);
    ShooterThread shooterThread;
    
    DigitalInput ball_lim_switch = new DigitalInput(2);
    
    Relay launch_adjuster = new Relay(2);
    //DigitalInput launch_adjuster_lim_switch = new DigitalInput(3);
    //TimedMotor launch_adjuster_timer = new TimedMotor(launch_adjuster);
    
    private boolean forward_is_pickupper = false;
    private boolean directionChanged = false;
    
    PWM camera_light = new PWM(10);
    
    VisionProcessor visionProcessor = new VisionProcessor(camera);
    
    boolean our_side_is_hot;
    
    NetworkTable server = NetworkTable.getTable("");
    CameraBallSource cameraBallSource = new CameraBallSource(server);
    DriveOut driveOut = new DriveOut(drive);
    PIDController go_fetch_pid = new PIDController(Kp, Ki, Kd, cameraBallSource, driveOut);
    
    int our_state = State.NORMAL;
    boolean shooter_is_busy = false;
    
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
        
        /**
         * Set the PID
         */
        go_fetch_pid.setSetpoint(160);
        go_fetch_pid.setInputRange(0, 320);
        go_fetch_pid.setOutputRange(-1, 1);
        go_fetch_pid.setPercentTolerance(5);
        launch_adjuster.setDirection(Relay.Direction.kBoth);
    }
    
    
    public void autonomousInit() {
        compressor.start();
        pickUpperControl.start();
        camera_light.setRaw(255);
        visionProcessor.autonomousInit();
        (new AutonomousMove()).start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        if (autonomousMove == State.GO_FORWARD){
            //move forward for 2-3s
            drive.drive(0.7, 0.0);
        }
        else if (autonomousMove == State.NORMAL && autonomousShoot == State.NOT_SHOT) {
            //stop and shoot
            drive.drive(0.0, 0.0);
            shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, 1);
            shooterThread.start();
            autonomousShoot = State.SHOT;
        }
        /*loopCount++;
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
        }*/
    }
    
    public void teleopInit() {
        compressor.start();
        pickUpperControl.start();
        topGrabberControl.start();
        getWatchdog().setEnabled(false); //TODO: true later
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        if (ball_lim_switch.get()) {
            pickUpperControl.putUpNoMatterWhat();
            Log.log("Limit switch hit");
        }
        if (our_state == State.NORMAL) {
            if (go_fetch_pid.isEnable()) {
                go_fetch_pid.disable();
                System.out.println("Disabling kill mode");
            }
            //square by magnitude but not sign
            double left = - Math.abs(driveStick2.getY())*driveStick2.getY() - Math.abs(driveStick2.getX())*driveStick2.getX();
            double right = - Math.abs(driveStick2.getY())*driveStick2.getY() + Math.abs(driveStick2.getX())*driveStick2.getX();

            //only change the direction if the time since the last press was over 
            if (forward_is_pickupper) {
                drive.tankDrive(left, right);
            } else {
                drive.tankDrive(-right, -left);
            }

            if (driveStick1.getRawButton(6)) {
                pickUpperControl.toggle();
            }

            if (driveStick1.getRawButton(11)) {
                if (!directionChanged) {
                    forward_is_pickupper = !forward_is_pickupper;
                    directionChanged = true;
                }
            } else {
                directionChanged = false;
            }
            if (driveStick2.getRawButton(3)) {
                topGrabberControl.toggle();
            }
            if (driveStick1.getTrigger() && topGrabberControl.getIsUp()){
                if (shooterThread == null || !shooterThread.isAlive()) {
                    shooter_is_busy = true;
                    fast_shoot1_fwd.set(true);
                    fast_shoot2_fwd.set(true);
                    fast_shoot2_rev.set(false);
                    fast_shoot2_rev.set(false);
                    Log.log("Single hold shot");
                } else {
                    Log.log("No single hold shot.");
                }
            } else {
                shooter_is_busy = false;
            }
            if (driveStick1.getRawButton(2) && topGrabberControl.getIsUp() && !shooter_is_busy) {
                if (shooterThread == null || !shooterThread.isAlive()) {
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, .15);
                    shooterThread.start();
                    Log.log("Single weak shot");
                } else {
                    Log.log("No single weak shot.");
                }
            }
            if (driveStick1.getRawButton(3) && topGrabberControl.getIsUp() && !shooter_is_busy) {
                if (shooterThread == null || !shooterThread.isAlive()) {
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, .22);
                    shooterThread.start();
                    Log.log("Single weak shot");
                } else {
                    Log.log("No single weak shot.");
                }
            }
            if ((shooterThread == null || !shooterThread.isAlive()) && !shooter_is_busy) {
                fast_shoot1_fwd.set(false);
                fast_shoot2_fwd.set(false);
                fast_shoot2_rev.set(true);
                fast_shoot1_rev.set(true);
            }
            if (driveStick1.getRawButton(4)) {
                launch_adjuster.set(Relay.Value.kOn);
                launch_adjuster.set(Relay.Value.kForward);
            } else {
                launch_adjuster.set(Relay.Value.kOff);
            }
            
            if (driveStick2.getRawButton(2)) {
                our_state = State.GO_FETCH;
            }
            
        } else if (our_state == State.GO_FETCH) {
            if (!go_fetch_pid.isEnable()) {
                go_fetch_pid.enable();
                System.out.println("Arf arf! Getting the ball!");
            }
            if (!driveStick2.getRawButton(2)) {
                our_state = State.NORMAL;
            }
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
