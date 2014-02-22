/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team1815.robot;


import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
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
    //distance to go forward for autonomous
    final static double DISTANCE = 100; //NEEDS TESTING
    
    RobotDrive drive = new RobotDrive(1, 2, 3, 4);
    Joystick driveStick1 = new Joystick(1);
    Joystick driveStick2 = new Joystick(2);
    AxisCamera camera = AxisCamera.getInstance();
    AxisCamera launcherCamera = AxisCamera.getInstance("10.18.15.12");
    
    //Autonomous only
    int hotCount = 0; //if reaches 10, target is hot and go to score
    int loopCount = 0; //makes sure goal is hot 9/10 times
    Encoder autoEncoder_l = new Encoder(7, 8);
    Encoder autoEncoder_r = new Encoder(13, 14);
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
    SpikeThread spikeThread;
    
    DigitalInput ball_lim_switch = new DigitalInput(2);
    
    Relay launch_adjuster = new Relay(2);
    //DigitalInput launch_adjuster_lim_switch = new DigitalInput(3);
    //TimedMotor launch_adjuster_timer = new TimedMotor(launch_adjuster);
    
    private boolean forward_is_pickupper = true;
    private boolean directionChanged = false;
    
    PWM camera_light = new PWM(10);
    Servo acquire_cam_servo = new Servo(8);
    VisionProcessor visionProcessor = new VisionProcessor(camera);
    
    Encoder encoder_l = new Encoder(7, 8);
    Encoder encoder_r = new Encoder(13, 14);
    AnalogChannel acquire_ultrasonic = new AnalogChannel(1);
    
    boolean our_side_is_hot;
    double var_power_shot = 0.20;
    
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
     * 
     * @param start true for start, false for stop
     */
    private void encoders(boolean start) {
        if (start) {
            encoder_l.start();
            encoder_r.start();
            encoder_l.setDistancePerPulse(1);
            encoder_r.setDistancePerPulse(1);
            encoder_l.reset();
            encoder_r.reset();
        } else {       
            encoder_l.stop();
            encoder_r.stop();
        }
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
        encoders(true);
    }
    
    
    public void autonomousInit() {
        compressor.start();
        pickUpperControl.start();
        camera_light.setRaw(255);
        visionProcessor.autonomousInit();
        encoders(true);
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {   
        
        if (visionProcessor.autonomousPeriodic(null)) {
            if (loopCount <= 10 && ++hotCount >= 9) {
                if (autoEncoder_l.getDistance() < DISTANCE && autoEncoder_r.getDistance() < DISTANCE) {
                    drive.drive(0.7, 0.0);
                }
                else if (autoEncoder_l.getDistance() >= DISTANCE && autoEncoder_r.getDistance() >= DISTANCE && 
                        autonomousShoot == State.NOT_SHOT) {
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, 1.0);
                    shooterThread.start();
                    drive.drive(0.0, 0.0);
                    encoders(false);
                }
            }
            else if (loopCount == 10 && ++hotCount < 9) {
                //reset loopCount and hotCount for next check
                loopCount = 0;
                hotCount = 0;
            }
        }
        
        /*if (autonomousMove == State.GO_FORWARD){
            //move forward for 2-3s
            drive.drive(0.7, 0.0);
        }
        else if (autonomousMove == State.NORMAL && (autonomousShoot == State.NOT_SHOT ||
                autonomousShoot == State.SHOT_ONCE)) {
            //stop and shoot
            shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, 1.0);
            shooterThread.start();
            if (autonomousShoot == State.NOT_SHOT) autonomousShoot = State.SHOT_ONCE;
            else if (autonomousShoot == State.SHOT_ONCE) autonomousShoot = State.SHOT_TWICE;
            drive.drive(0.7, 0.0);
        }*/
     
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
        camera_light.setRaw(255);
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
                drive.tankDrive(-right, -left);
            } else {
                drive.tankDrive(left, right);
            }

            if (driveStick1.getRawButton(6)) {
                pickUpperControl.toggle();
            }

            if (driveStick2.getRawButton(4)) {
                if (!directionChanged) {
                    forward_is_pickupper = !forward_is_pickupper;
                    directionChanged = true;
                }
            } else {
                directionChanged = false;
            }
            if (driveStick1.getRawButton(4)) {
                topGrabberControl.toggle();
            }
            if (driveStick1.getTrigger() && topGrabberControl.getIsUp()){
                if (shooterThread == null || !shooterThread.isAlive()) {
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, 1.0);
                    shooterThread.start();
                } else {
                    Log.log("No single hold shot.");
                }
            } else {
                shooter_is_busy = false;
            }
            if (driveStick1.getRawButton(5) && topGrabberControl.getIsUp()) {
                if (shooterThread == null || !shooterThread.isAlive()) {
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, var_power_shot);
                    shooterThread.start();
                    System.out.println("Shot with " + var_power_shot);
                }
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
                    shooterThread = new ShooterThread(fast_shoot1_fwd, fast_shoot2_fwd, fast_shoot1_rev, fast_shoot2_rev, .20);
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
            if (driveStick1.getRawButton(7) &&
                    (spikeThread == null || !spikeThread.isAlive())) {
                spikeThread = new SpikeThread(pick_upper_up, pick_upper_down, launch_adjuster);
                spikeThread.start();
                Log.log("spike");
            }
            
            if (driveStick2.getRawButton(2)) {
                our_state = State.GO_FETCH;
            }
            if (driveStick1.getRawButton(10)) {
                acquire_cam_servo.set(.25);
                System.out.println(acquire_cam_servo.get());
            } else if (driveStick1.getRawButton(11)) {
                acquire_cam_servo.set(0.75);
                System.out.println(acquire_cam_servo.get());
            }
            
            if (driveStick1.getRawButton(9)) {
                System.out.println("us: " + acquire_ultrasonic.getVoltage());
                System.out.println("enc1: " + encoder_l.get());
                System.out.println("enc2: " + encoder_r.get());
            }
            if (driveStick2.getRawButton(11)) {
                var_power_shot += 0.01;
                System.out.println("Changing variable shot to: " + var_power_shot);
            } else if (driveStick2.getRawButton(10)) {
                var_power_shot -= 0.01;
                System.out.println("Changing variable shot to: " + var_power_shot);
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
    private void changeServo(Servo servo, double degrees) {
        servo.set(degrees + servo.getAngle());
    }
    
    public void disabledInit() {
        //camera_light.setRaw(0);
        stopAllPneumatics();
        compressor.stop();
        //encoders(false);        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
}
