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

  
  //outtake and intake methods
  public void algaeIntake(double speed) {
    mState = IntakeState.INTAKE;
    intakeMotor.set(speed);
  }

  public void algaeOuttake(double speed) {
    mState = IntakeState.OUTTAKE;
    intakeMotor.set(speed);
  }

  public void stopAlgaeIntake() {
    mState = IntakeState.NONE;
    intakeMotor.set(0);
  }

  public IntakeState getState() {
    return mState;
  }

  public Command takeAlgae(double speed) {
    return run(() -> algaeIntake(speed));
  }

public Command scoreAlgae(double speed) {
  return run(() -> algaeOuttake(speed));
}

public Command stopIntake() {
  return run(() -> stopAlgaeIntake());
}
}
