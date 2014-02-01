package com.team1815.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

public class DriveOut implements PIDOutput{
    RobotDrive robotDrive;

    public void pidWrite(double d) {
        robotDrive.arcadeDrive(-.6, -d);
    }
    
    public DriveOut(RobotDrive robotDrive) {
        this.robotDrive = robotDrive;
    }    
}
