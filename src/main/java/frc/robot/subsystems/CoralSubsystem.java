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
        } catch (Exception e) {
            System.out.println("Laser configuration failed: " + e.getMessage());
        }
    }

  

    // Method to set the speed of both motors
    public void setSpeed(double speed) {
        m_LeftMotor.set(speed);
        m_RightMotor.set(-speed);
    }

    // Methods to control the intake and outtake
    public void intake() {
        mState = IntakeState.INTAKE;
        setSpeed(Constants.Coral_Algae_Constants.kIntakeSpeed);
    }

    public void reverse() {
        mState = IntakeState.REVERSE;
        setSpeed(Constants.Coral_Algae_Constants.kReverseSpeed);
    }

    public void index() {
        mState = IntakeState.INDEX;
        setSpeed(Constants.Coral_Algae_Constants.kIndexSpeed);
    }

    public void scoreL1() {
        mState = IntakeState.SCORE;
        double fullSpeed = Constants.Coral_Algae_Constants.kL1Speed;
        double halfSpeed = fullSpeed / 4.0;
    
        // Set the left motor to the full speed
        m_LeftMotor.set(fullSpeed);
    
        // Set the right motor to half speed
        m_RightMotor.set(-halfSpeed); // Assuming right motor should be reversed as in the original `setSpeed` method
    }

    public void scoreL24() {
        mState = IntakeState.SCORE;
        setSpeed(Constants.Coral_Algae_Constants.kL24Speed);
    }

    public void scoreL24HalfSpeed() {
        mState = IntakeState.SCORE;
        setSpeed(0.1);
    }

    // Stop the motors and reset state
    public void stopCoral() {
        mState = IntakeState.NONE;
        setSpeed(0.0);
    }

    // Laser sensor logic to check if the robot is holding coral
    public boolean isHoldingCoralViaLaserCAN() {
        return m_LaserCAN.getMeasurement().distance_mm < 75.0;
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

    public Command coralStop(){
        return run(() -> stopCoral());
    }

    
}

