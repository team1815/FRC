package com.team1815.robot;

public class AutonomousMove extends Thread{
    
    double startTime = System.currentTimeMillis();

    public void run() {
        while (System.currentTimeMillis() - startTime < 2500 ) {

        }
        IterativeBeast1815.autonomousMove = State.NORMAL;
    }
    
}
