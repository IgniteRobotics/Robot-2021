// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.Duration;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Limelight extends SubsystemBase {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table2 = inst.getTable("limelight");
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tx = table.getEntry("tx");
    private double previousDistance = 0.0;
    private double currentDistance = 0.0;
    private int accuracyCount = 0;

    private long lastLimelightUsage;
    private boolean ledStatus = false;
    private long limelightDuration = Duration.ofSeconds(3).toMillis();

    /**
     * Creates a new Limelight.
     */
    public Limelight() {
        table.getEntry("camMode").setNumber(1);
    }

    public void turnOnLED () {
        table.getEntry("ledMode").setNumber(3);
        ledStatus = true;
    }

    public void turnOffLED() { 
        //These can be done in pipelines, but I think this is faster?
        table.getEntry("ledMode").setNumber(1);
        ledStatus = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        table.getEntry("camMode").setNumber(1);
        //Cache previous values here
        long delta = System.currentTimeMillis() - lastLimelightUsage;
        if(delta > limelightDuration && isLedOn()) {
            turnOffLED();
        }
    }

    private void onLimelightUse() {
        if(!isLedOn()) {
            turnOnLED();
        }

        lastLimelightUsage = System.currentTimeMillis();
    }

    public double getDistancefromgoal() {
        //Current implementation requires the command to turnOffLed() when interrupted or finished.
        //TODO make this method turnOffLed() when done
       
        //need height camera is from ground
        //need height of hole from piller
        //need angle that limelight is from ground

        //Turn on limelight LED's 
        //Perhaps this should be done through commands?
        onLimelightUse();

        //a2, the difference in goal and camera is ty
        double ty = this.ty.getDouble(0.0);
      //  this.turnOffLED();
        currentDistance = (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(ty + Constants.LIMELIGHT_ANGLE));
       

        if (previousDistance == currentDistance) {
            accuracyCount++;
            
        }
        else {
            accuracyCount = 0;
            previousDistance = currentDistance;
        }

        if (accuracyCount == 4) {
            return previousDistance;
        }


        
        return -1; //Limelight does not have consistent distance
        

    }

    public double getHorizontalOffset() {
        onLimelightUse();
        return tx.getDouble(0.0);
    }

    public boolean isLedOn() {
        return ledStatus;
    }
}
