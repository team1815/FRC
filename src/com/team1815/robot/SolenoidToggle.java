package com.team1815.robot;

import com.sun.squawk.debugger.Log;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simple toggle for a solenoid. Ensures that it it doesn't switch back and forth too quickly
 * if the button is held down for more than one iteration.
 */
class SolenoidToggle {
    boolean is_up = true;
    Timer timer = new Timer();
    double prevTime = 0;
    Solenoid fwd;
    Solenoid rev;
    private final IterativeBeast1815 outer;

    public SolenoidToggle(Solenoid fwd, Solenoid rev, final IterativeBeast1815 outer) {
        this.outer = outer;
        this.fwd = fwd;
        this.rev = rev;
    }

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
        if (timer.get() > prevTime) {
            return timer.get() - prevTime;
        } else {
            return 0;
        }
    }

    void start() {
        timer.reset();
        timer.start();
    }

    public boolean getIsUp() {
        return is_up;
    }
    
}
