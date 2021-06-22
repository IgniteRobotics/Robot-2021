// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class Limelight extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private NetworkTableEntry ty = table.getEntry("ty");
    private NetworkTableEntry tx = table.getEntry("tx");

    /**
     * Creates a new Limelight.
     */
    public Limelight() {
    }

    public void turnOnLED () {
        table.getEntry("ledMode").setNumber(3);
    }

    public void turnOffLED() { 
        //These can be done in pipelines, but I think this is faster?
        table.getEntry("ledMode").setNumber(1);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public double getDistancefromgoal() {
        //need height camera is from ground
        //need height of hole from piller
        //need angle that limelight is from ground

        //Turn on limelight LED's 
        //Perhaps this should be done through commands?
        this.turnOnLED();

        //a2, the difference in goal and camera is ty
        double ty = this.ty.getDouble(0.0);
        this.turnOffLED();

        //turn off LED's

        //TODO confirm these values
        return (Constants.TARGET_HEIGHT - Constants.LIMELIGHT_HEIGHT) / Math.tan(Math.toRadians(ty + Constants.LIMELIGHT_ANGLE));
    }

    public double getHorizontalOffset() {
        return tx.getDouble(0.0);
    }
}
