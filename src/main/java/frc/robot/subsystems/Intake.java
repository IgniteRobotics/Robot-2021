/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.MotorConstants;

public class Intake extends SubsystemBase {
    private final WPI_VictorSPX intakeMotor;
    private DoubleSolenoid intakePistonSolenoid;

    private boolean isExtended;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        intakeMotor = new WPI_VictorSPX(MotorConstants.kIntakeMotorPort);
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        isExtended = false;

        //intakePistonSolenoid = null;
        intakePistonSolenoid = new DoubleSolenoid(44, MotorConstants.kIntakeSolenoidForwardPort, MotorConstants.kIntakeSolenoidReversePort);
        addChild("intakeMotor- Intake",intakeMotor);
    }

    private void extendIntake() {
        System.out.println("extending intake");
        isExtended = true;
        intakePistonSolenoid.set(Value.kForward);
    }

    private void retractIntake() {
        System.out.println("retracting intake");
        isExtended = false;
        intakePistonSolenoid.set(Value.kReverse);
    }

    public void toggleIntake() {
        if (isExtended) {
            retractIntake();
        } else {
            extendIntake();
        }
    }

    public void spin(double speed) {
        intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
