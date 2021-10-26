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
    private NetworkTableEntry tv = table.getEntry("tv");

    private double previousDistance = 0.0;
    private double currentDistance = 0.0;
    private int accuracyCount = 0;

    private long lastLimelightUsage;
    private boolean ledStatus = false;
    private long limelightDuration = Duration.ofSeconds(3).toMillis();

    private boolean camMode = false;

    /**
     * Creates a new Limelight.
     */
    public Limelight() {
        setCamMode(true);
    }

    public void setCamMode(boolean mode) {
        camMode = mode;

        if(mode) {
            turnOffLED();
            table.getEntry("camMode").setNumber(1);
        } else {
            turnOnLED();
            table.getEntry("camMode").setNumber(0);
        }
    }

    public void turnOnLED() {
        table.getEntry("ledMode").setNumber(3);
        table.getEntry("camMode").setNumber(0);
        ledStatus = true;
    }

    public void turnOffLED() {
        table.getEntry("ledMode").setNumber(1);
        ledStatus = false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        //Cache previous values here
        SmartDashboard.putNumber("Limelight-reported distance", currentDistance);
        long delta = System.currentTimeMillis() - lastLimelightUsage;
        if(delta > limelightDuration && !camMode) {
            setCamMode(true);
        }
    }

    public void onLimelightUse() {
        if(camMode) {
            setCamMode(false);
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
        return currentDistance;
    }

    public double getHorizontalOffset() {
        onLimelightUse();
        return tx.getDouble(0.0);
    }

    public double getVerticalOffset() {
        onLimelightUse();
        return ty.getDouble(0.0);
    }

    public double getTargetViewable() {
        onLimelightUse();
        return tv.getDouble(0.0);
    }

    public boolean isLedOn() {
        return ledStatus;
    }
}
