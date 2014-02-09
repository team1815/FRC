package com.team1815.robot;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class SpikeThread extends Thread {
    Solenoid pickupper_fwd, pickupper_rev;
    Relay spike;
    public void run() {
        //drop it
        pickupper_rev.set(false);
        pickupper_fwd.set(true);
        Timer.delay(0.2);
        spike.set(Relay.Value.kOn);
        spike.set(Relay.Value.kForward);
        Timer.delay(0.1);
        spike.set(Relay.Value.kOff);
        Timer.delay(0.2);
        //put it back up
        pickupper_fwd.set(false);
        pickupper_rev.set(true);
    }
    
    public SpikeThread(Solenoid pickupper_fwd, Solenoid pickupper_rev, Relay spike) {
        this.pickupper_fwd = pickupper_fwd;
        this.pickupper_rev = pickupper_rev;
        this.spike = spike;
    }
}
