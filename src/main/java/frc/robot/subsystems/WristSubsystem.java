// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import pabeles.concurrency.IntOperatorTask.Max;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private AbsoluteEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private final double encoderMultiplier =   (Units.radiansToDegrees(Math.PI * 2));   //Degrees

  private final float MAXPosition = 210;
  private final float MINPosition = 20;

  private final double WristMarginError = 4;

  public WristSubsystem() {
    
    // initialize motor
    m_motor = new CANSparkMax(ArmConstants.WristID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();


    // initialze PID controller and encoder objects
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);

    m_pidController.setFeedbackDevice(m_encoder);
    m_encoder.setPositionConversionFactor(encoderMultiplier);
    
    m_encoder.setInverted(true);
    m_motor.setInverted(false);

    // PID coefficients
    // kP = 0.006;
    // kI = 0;
    // kD = 0.003; 
    // kIz = 0; 
    // kFF = 0.003; 
    // kMaxOutput = 1; 
    // kMinOutput = -1;
    // maxRPM = 5700;

    kP = 0.04; 
    kI = 0;
    kD = 0.003; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.5; 
    kMinOutput = -0.5;
    //maxRPM = 5700;
    //allowedErr = 0.1;
  

    // Smart Motion Coefficients

    maxVel = 7; // rpm
    maxAcc = 1;

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
    m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, smartMotionSlot);

    m_motor.setIdleMode(IdleMode.kCoast);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, MAXPosition);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, MINPosition);

    m_motor.setClosedLoopRampRate(0.2);

    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    
  }

  public void MoveWristDegrees (double degrees) {
      if(m_encoder.getPosition() >= MAXPosition) m_pidController.setReference(MAXPosition, CANSparkMax.ControlType.kPosition); 
      else if(m_encoder.getPosition() <= MINPosition) m_pidController.setReference(MINPosition, CANSparkMax.ControlType.kPosition);  
      else m_pidController.setReference(degrees, CANSparkMax.ControlType.kPosition); 

  }

  public void MoveWristSpeed(double speed){
    speed *= ArmConstants.ArmVelocityMultiplier;
      if(m_encoder.getPosition() >= MAXPosition || m_encoder.getPosition() <= MINPosition ) m_pidController.setReference(0, ControlType.kVelocity);
      else m_pidController.setReference(speed, ControlType.kVelocity); 
  }

  public boolean isWristAtSetpoint(double setPoint){
    return Math.abs(m_encoder.getPosition() - setPoint) <= WristMarginError;
  }

  public void setForwardSoftLimit(Float degrees){
    m_motor.setSoftLimit(SoftLimitDirection.kForward, degrees);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  public double getForwardSoftLimit(){
    return m_motor.getSoftLimit(SoftLimitDirection.kForward);
  }

  public void setReverseSoftLimit(Float limit){
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, limit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);

  }

  public void setDefaultReverseSoftLimit(){
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, MINPosition);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
  }

  public double getReverseSoftLimit(){
    return m_motor.getSoftLimit(SoftLimitDirection.kReverse);
  }
  
  public double getReverseSoftLimitDefault(){
    return MINPosition;
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist encord value", m_encoder.getPosition());
    SmartDashboard.putNumber("Wrist softlimit value", getReverseSoftLimit());
    SmartDashboard.putNumber("Wrist Speed", m_encoder.getVelocity());
    SmartDashboard.putNumber("WristOutput", m_motor.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
