// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase {
  /** Creates a new AlgaeIntakeSubsystem. */
private final SparkMax intakeMotor = new SparkMax(Constants.IDConstants.Algae_Intake_ID, MotorType.kBrushless);

public enum IntakeState {
  NONE,
  INTAKE,
  OUTTAKE
}

private IntakeState mState = IntakeState.NONE;

  public AlgaeIntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }
  
  //outtake and intake methods
  public void algaeIntake() {
    mState = IntakeState.INTAKE;
    setSpeed(Constants.Coral_Algae_Constants.kAlgaeIntakeSpeed);
  }

  public void algaeOuttake() {
    mState = IntakeState.OUTTAKE;
    setSpeed(Constants.Coral_Algae_Constants.kEjectSpeed);
  }

  public void stopAlgaeIntake() {
    mState = IntakeState.NONE;
    setSpeed(0.0);
  }

  public IntakeState getState() {
    return mState;
  }

  public Command takeAlgae() {
    return run(() -> algaeIntake());
  }

public Command scoreAlgae() {
  return run(() -> algaeOuttake());
}

public Command stopIntake() {
  return run(() -> stopAlgaeIntake());
}
}
