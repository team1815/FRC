package com.team1815.robot;

public class AutonomousMove extends Thread {
    
    double startTime = System.currentTimeMillis();

    public void run() {
        if (startTime > System.currentTimeMillis()) {
            startTime = System.currentTimeMillis();
        }
        try {
            Thread.sleep(1000);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
        IterativeBeast1815.autonomousMove = State.NORMAL;
    }
    
    public AutonomousMove() {
        IterativeBeast1815.autonomousMove = State.GO_FORWARD;
        IterativeBeast1815.autonomousShoot = State.NOT_SHOT;
        startTime = System.currentTimeMillis();
    }
    
}
