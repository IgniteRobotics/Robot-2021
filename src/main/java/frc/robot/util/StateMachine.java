// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class StateMachine {

    private double shooterDistance;

    private static StateMachine instance;

    private StateMachine() {
        this.shooterDistance = 0.0;        
    }

    public synchronized static StateMachine getIntance(){
        if (instance == null){
            instance = new StateMachine();
        }
        return instance;
    }

    public void setShooterDistance(double dist){
        this.shooterDistance = dist;
    }

    public double getShooterDistance(){
        return this.shooterDistance;
    }
}
