package com.team1815.robot;

import edu.wpi.first.wpilibj.Victor;

/**
 *
 * @author FRC
 */
public class TimedMotor {
    Victor victor;
    boolean semaphore = false;
    
    public TimedMotor(Victor victor) {
        this.victor = victor;
    }
    public void go(final int millis, boolean forward) {
        System.out.println("launch_adjuster: " + millis + " " + forward);
        if (! semaphore) {
            semaphore = true;
            final double speed = 1.0 * (forward? 1 : -1);
            (new Thread(new Runnable() {
                public void run() {
                    victor.set(speed);
                    try {
                        Thread.sleep(millis);
                    } catch (InterruptedException ex) {
                        ex.printStackTrace();
                    }
                    victor.set(0);
                }            
            })).run();
        }
        semaphore = false;
    }    
}
