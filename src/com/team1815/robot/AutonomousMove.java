package com.team1815.robot;

import edu.wpi.first.wpilibj.Timer;

public class AutonomousMove extends Thread {

    public void run() {
        IterativeBeast1815.autonomousMove = State.GO_FORWARD;
        IterativeBeast1815.autonomousShoot = State.NOT_SHOT;
        Timer.delay(1.0);
        IterativeBeast1815.autonomousMove = State.NORMAL;
    }    
}
