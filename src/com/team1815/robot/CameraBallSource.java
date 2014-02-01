package com.team1815.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class CameraBallSource implements PIDSource {
    NetworkTable server;

    public double pidGet() {
        return server.getNumber("COG_X");
    }
    
    public CameraBallSource(NetworkTable server) {
        this.server = server;
    }
    
}
