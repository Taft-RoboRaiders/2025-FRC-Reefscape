package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class CoralSubsystem extends SubsystemBase {


    // Laser can for detecting coral and Outtake motors
    private SparkMax m_LeftMotor;
    private SparkMax m_RightMotor;
    private LaserCan m_LaserCAN;

    public enum IntakeState {
        NONE,
        INTAKE,
        REVERSE,
        INDEX,
        READY,
        SCORE
    }

    private IntakeState mState = IntakeState.NONE;

    public CoralSubsystem() {
        // Initialize hardware
        m_LeftMotor = new SparkMax(Constants.IDConstants.Outtake_Left_ID, MotorType.kBrushless);
        m_RightMotor = new SparkMax(Constants.IDConstants.Outtake_Right_ID, MotorType.kBrushless);
        m_LaserCAN = new LaserCan(Constants.IDConstants.kLaserId);

        // Laser sensor configuration
        try {
            m_LaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
            m_LaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_20MS);
            m_LaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8,15,5,15));
        } catch (Exception e) {
            System.out.println("Laser configuration failed: " + e.getMessage());
        }
    }

  

    // Method to set the speed of both motors
    public void setSpeed(double speed) {
        m_LeftMotor.set(speed);
        m_RightMotor.set(-speed);
    }

    // Outtake
    public void intake() {
        mState = IntakeState.INTAKE;
        setSpeed(Constants.Coral_Algae_Constants.kIntakeSpeed);
    }
    public Command coralIntake(){
        return run(() -> intake());
    }

    //reverse outtake
    public void reverse() {
        mState = IntakeState.REVERSE;
        setSpeed(Constants.Coral_Algae_Constants.kReverseSpeed);
    }
    public Command coralReverse(){
        return run(() -> reverse());
    }

    //index
    public void index() {
        mState = IntakeState.INDEX;
        setSpeed(Constants.Coral_Algae_Constants.kIndexSpeed);
    }
    public Command coralIndex(){
        return run(() -> index());
    }

    //L1 full speed score
    public void scoreL1() {
        mState = IntakeState.SCORE;
        double fullSpeed = Constants.Coral_Algae_Constants.kL1Speed;
        double lowSpeed = Constants.Coral_Algae_Constants.kL1SpeedLow;
    
        // Set the left motor to the full speed
        m_LeftMotor.set(fullSpeed);
    
        // Set the right motor to half speed
        m_RightMotor.set(-lowSpeed); // Assuming right motor should be reversed as in the original `setSpeed` method
    }
    public Command coralL1(){
        return run(() -> scoreL1());
    }
    
    //for autonomous use only, command for this is in command folder
    public void scoreAutoL1() {
        mState = IntakeState.SCORE;
        double fullSpeed = 0.5;
        double lowSpeed = 0.2;
    
        // Set the left motor to the full speed
        m_LeftMotor.set(fullSpeed);
    
        // Set the right motor to half speed
        m_RightMotor.set(-lowSpeed); // Assuming right motor should be reversed as in the original `setSpeed` method
    }


    //L24 full speed score
    public void scoreL24() {
        mState = IntakeState.SCORE;
        setSpeed(Constants.Coral_Algae_Constants.kL24Speed);
    }
    public Command coralL24(){
        return run(() -> scoreL24());
    }

    //for autonomous use only, command for this is in command folder
    public void scoreAutoL24() {
        mState = IntakeState.SCORE;
        setSpeed(Constants.Coral_Algae_Constants.kL24Speed);
    }

    //L24 score low speed score
    public void scoreL24LowSpeed() {
        mState = IntakeState.SCORE;
        setSpeed(Constants.Coral_Algae_Constants.kL24SpeedLow);
    }
    public Command coralL24LowSpeed(){
        return run(() -> scoreL24LowSpeed());
    }

    // Stop the motors and reset state
    public void stopCoral() {
        mState = IntakeState.NONE;
        setSpeed(0.0);
    }
    public Command coralStop(){
        return run(() -> stopCoral());
    }

    // Laser sensor logic to check if the robot is holding coral
    public boolean isHoldingCoralViaLaserCAN() {
        return m_LaserCAN.getMeasurement().distance_mm <= 0;
    }

    // Getter for the current state
    public IntakeState getState() {
        return mState;
    }

    // Output telemetry to the dashboard
    public void outputTelemetry() {
        SmartDashboard.putNumber("Laser/Distance", m_LaserCAN.getMeasurement().distance_mm);
        SmartDashboard.putBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    }

    public void sensorIntake(){
        setSpeed(0.12);
    }

}

