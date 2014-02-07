package com.team1815.robot;

public class State {
    final static int NORMAL = 0; //normal human driving in teleop mode
    final static int GO_FETCH = 1; //robot chases after ball autonomously in teleop mode
    
    final static int NOT_SHOT = 2; //ball has not been shot in autonomous mode
    final static int GO_FORWARD = 3; //robot is moving forward in autonomous mode
    final static int SHOT = 4; //ball has been shot already in autonomous mode
}
