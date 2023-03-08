// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ShoulderSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  public RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private final double encoderMultiplier = (1 / (ArmConstants.shoulderGearRatio)) * 360;   //Degrees

  private final float MAXPosition = 130;
  private final float MINPosition = 0;
  private final double ShoulderMarginError = 4;

  public ShoulderSubsystem() {
    
    // initialize motor
    m_motor = new CANSparkMax(ArmConstants.ShoulderId, MotorType.kBrushless);
    //

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();

    m_encoder.setPositionConversionFactor(encoderMultiplier);
    //m_encoder.setVelocityConversionFactor(ArmConstants.gearBoxRatio);

    // PID coefficients
    kP = 5e-5; 
    kI = 1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 4500; // rpm
    maxAcc = 4500;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, MAXPosition);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, MINPosition);


    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    

    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve,0);
    m_motor.setSmartCurrentLimit(20);

  }
  public void MoveShoulderDegrees(double degrees) {
        m_pidController.setReference(degrees, CANSparkMax.ControlType.kSmartMotion);
  }
  
  public void MoveShoulderSpeed(double speed){
    speed *= ArmConstants.ShoulderVelocityMultiplier;
    m_pidController.setReference(speed, ControlType.kSmartVelocity);
    
  }

  public boolean isShoulderAtSetpoint(double setPoint){
    return Math.abs(m_encoder.getPosition() - setPoint) <= ShoulderMarginError;
  }

  public double GetShoulderPosition(){
    return m_encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shoulder encord value", m_encoder.getPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
