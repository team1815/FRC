/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package com.team1815.robot;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * Separate thread, so the shooter can shoot and come back down on its own.
 * @author FRC
 */
public class ShooterThread extends Thread {
    double time_delay = 1;
    Solenoid fwd1, fwd2, rev1, rev2;
    public void run() {
        fwd1.set(true);
        fwd2.set(true);
        rev1.set(false);
        rev2.set(false);
        
        Timer.delay(time_delay);
        
        fwd1.set(false);
        fwd2.set(false);
        rev1.set(true);
        rev2.set(true);
    }
    public ShooterThread(Solenoid fwd1, Solenoid fwd2, Solenoid rev1, Solenoid rev2) {
        this.fwd1 = fwd1;
        this.fwd2 = fwd2;
        this.rev1 = rev1;
        this.rev2 = rev2;
    }
}
