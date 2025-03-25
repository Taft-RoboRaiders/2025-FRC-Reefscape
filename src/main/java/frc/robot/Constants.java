// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.RPM;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMath.AlgaeArm;
public final class Constants {
  // Constants for controller IDs
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 1;
    public static final int kDriverControllerPort2 =0;
    public static final double deadband = 0.05;
  }

  // Constants for Kraken Drivetrain!
  public static final class SwerveConstants {
    public static final double MAX_SPEED = 10;
  } 

  // Constants for controller input!
  public static final class DriveConstants {
    public static final double k_driveDeadBand = 0.05;
    public static final double k_driveSpeed = 10;
  }
 


  public static class AlgaeArmConstants
  {

    // The P gain for the PID controller that drives this arm.
    public static final double  kAlgaeArmKp                     = 0.9113;
    public static final double  kAlgaeArmKi                     = 0;
    public static final double  kAlgaeArmKd                     = 0.02006;
    public static final double  kAlgaeArmkS                     = 0.17571; // volts (V)
    public static final double  kAlgaeArmKv                     = 0.0010398; // volts per velocity (V/RPM)
    public static final double  kAlgaeArmKa                     = 0.00021298; // volts per acceleration (V/(RPM/s))
    public static final double  kAlgaeArmkG                     = 0.19885; // volts (V)
    public static final double  kAlgaeArmReduction              = 109;
    public static final Angle   kAlgaeArmAllowedClosedLoopError
                                                                = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(
        0.01));
    public static final double  kAlgaeArmMass                   = Units.lbsToKilograms(15); // Kilograms
    public static final double  kAlgaeArmLength                 = Inches.of(31).in(Meters);//.7meter
    public static final Angle   kAlgaeArmStartingAngle          = Degrees.of(-84);
    public static final Angle   kAlgaeArmMinAngle               = Degrees.of(-84);
    public static final Angle   kAlgaeArmMaxAngle               = Degrees.of(60);
    public static final double  kAlgaeArmRampRate               = 0.5;
    public static final Angle   kAlgaeArmOffsetToHorizantalZero = Degrees.of(256);
    public static final boolean kAlgaeArmInverted               = true;
    public static final double  kAlgaeArmMaxVelocityRPM         = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(20)).per(Second).in(RPM);
    public static final double  kAlgaeArmMaxAccelerationRPMperSecond
                                                                = AlgaeArm.convertAlgaeAngleToSensorUnits(Degrees.of(5)).per(Second).per(Second)
                                                                          .in(RPM.per(Second));
    public static final int     kAlgaeArmStallCurrentLimitAmps  = 40;
   
    public static final double  kAlgaeAngleAllowableError       = 0.001;//degree, for testing whether it's aroundAngle
    public static final int algaeCanandColor                    = 22;
    public static       int     algaeArmMotorID                 = 16; // Checked

  }
  //Need to change all the constants later
  public static class ElevatorConstants{
    public static final double kElevatorKp = 9.7382;
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 2.0471;
    public static final double kMaxVelocity = Meters.of(0.6).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(0.6).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.13557;
    public static final double kElevatorkV = 10.517;
    public static final double kElevatorkA = 0.18635;
    public static final double kElevatorkG = 0.61846;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12;
    public static final double kElevatorSproketTeeth = 22;
    public static final double kElevatorPitch = Units.inchesToMeters(0.25);
    public static final double kElevatorCarriageMass = Units.lbsToKilograms(16);
    public static final double kElevatorDrumRadius = (kElevatorSproketTeeth * kElevatorPitch) / (2 *Math.PI);
    public static final double kElevatorMinHeightMeters = Units.inchesToMeters(3);
    public static final double kElevatorMaxHeightMeters = 0.60;
    public static final Distance kMinElevatorHeight      = Meters.of(kElevatorMinHeightMeters);
    public static final Distance kMaxElevatorHeight      = Meters.of(kElevatorMaxHeightMeters);
    public static final double   kElevatorAllowableError = Units.inchesToMeters(0.005); 
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);
    public static final double   kElevatorUnextendedHeight    = Units.inchesToMeters(41.5);
    public static double kLowerToScoreHeight = Units.inchesToMeters(6);
  }

  public static final Mechanism2d sideView = new Mechanism2d(AlgaeArmConstants.kAlgaeArmLength *2, 
                                                             ElevatorConstants.kMaxElevatorHeight.in(Meters) +
                                                            AlgaeArmConstants.kAlgaeArmLength +
                                                            ElevatorConstants.kElevatorUnextendedHeight);
                                                            
  public static final MechanismRoot2d kElevatorCarriage;
  public static final MechanismRoot2d kElevatorJoint;
  public static final MechanismLigament2d kElevatorTower;
  public static final MechanismLigament2d kAlgaeArmMech;
  //public static final MechanismLigament2d kElevatorFixed;
  public static final double              maxSpeed      = 7;

  static{

   

    kElevatorCarriage = Constants.sideView.getRoot("ElevatorCarriage",
                                                        AlgaeArmConstants.kAlgaeArmLength,
                                                     ElevatorConstants.kElevatorStartingHeightSim.in(Meters) +
                                                        ElevatorConstants.kElevatorUnextendedHeight);
    kElevatorTower = kElevatorCarriage.append(new MechanismLigament2d(
                                        "Elevator",
                                              ElevatorConstants.kElevatorStartingHeightSim.in(Meters),
                                              -90,
                                       6,
                                             new Color8Bit(Color.kRed)));
                                                  
    kElevatorJoint = Constants.sideView.getRoot("ElevatorJoint",
                                                      AlgaeArmConstants.kAlgaeArmLength,
                                                      ElevatorConstants.kElevatorUnextendedHeight);

                                                      kAlgaeArmMech = kElevatorCarriage.append(
                                                        new MechanismLigament2d(
                                                            "AlgaeArm",
                                                            AlgaeArmConstants.kAlgaeArmLength,
                                                            AlgaeArmConstants.kAlgaeArmStartingAngle.in(Degrees),
                                                            6,
                                                            new Color8Bit(Color.kGreen)));
                                                  
    /*kElevatorFixed = kElevatorJoint.append(new MechanismLigament2d("ElevatorFixed",
                                          ElevatorConstants.kElevatorUnextendedHeight,
                                          -90,
                                          6,
                                          new Color8Bit(Color.kPaleVioletRed)));*/
  }

  public static class IDConstants {
    public static final int Outtake_Left_ID = 12;
    public static final int Outtake_Right_ID = 11;
    public static final int Elevator_Left_ID = 13;
    public static final int Elevator_Right_ID = 14;
    public static final int Algae_Pivot_ID = 15;
    public static final int Algae_Intake_ID = 16;
    public static final int kLaserId =20;
  }

  public static class Coral_Algae_Constants {
    public static final double kIntakeSpeed = 0.3;
    public static final double kReverseSpeed = -0.2;
    public static final double kL1Speed = 0.75;
    public static final double kL1SpeedLow = 0.1;
    public static final double kL24Speed = 0.75;
    public static final double kL24SpeedLow = 0.1;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDiffernce = kL1Speed * 0.5;
    public static final double kWristEncoderId = 20;
    public static final int mWristEncoderID1 = 0;
    public static final int mWristEncoderID2 = 1;
    public static final boolean mWristEncoderInvert = true;
    public static final int kMaxWristCurrent = 10;
    
    public static final double kWristP = 0.01;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.0;
    public static final double kWristKG = 0.0;
    public static final double kWristKV = 0.100;
    public static final double kWristKA = 0.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    
    public static final double kStowAngle = 233.0;
    public static final double kDeAlgaeAngle = 215.0;
    public static final double kGroundIntakeAngle = 162.0;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kAlgaeEjectSpeed = 0.9;
    public static final double kAlgaeIntakeSpeed = -0.9;
    public static final double kGroundIntakeSpeed = -0.3;
    
    public static final double kWristOffset = 141.0;
  }
}
