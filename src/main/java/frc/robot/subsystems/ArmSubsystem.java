// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;


public class ArmSubsystem extends SubsystemBase {

  ShoulderSubsystem m_shoulderSubsystem;
  WristSubsystem m_wristSubsystem;

  private final double shoulderWristBottomDegree = 45;


  /** Creates a new ExampleSubsystem. */
  public ArmSubsystem(ShoulderSubsystem shoulderSubsystem, WristSubsystem wristSubsystem ) {
    m_shoulderSubsystem = shoulderSubsystem;
    m_wristSubsystem = wristSubsystem;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public void MoveShoulderDegrees(double degrees) {
    // This method will be called once per scheduler run during simulation
    m_shoulderSubsystem.MoveShoulderDegrees(degrees);
    AdjustWristSoftLimit();
  }

  public void MoveShoulderSpeed(double speed){
      m_shoulderSubsystem.MoveShoulderSpeed(speed);
      AdjustWristSoftLimit();
  }

  public void MoveWristDegrees(double degrees){
    m_wristSubsystem.MoveWristDegrees(degrees);
  }

  public void MoveWristSpeed(double speed){
    m_wristSubsystem.MoveWristSpeed(speed);
  }

  private void AdjustWristSoftLimit(){
    if (m_shoulderSubsystem.GetShoulderPosition() <= shoulderWristBottomDegree){ 
      Double limit = (m_wristSubsystem.getReverseSoftLimint() + (m_shoulderSubsystem.GetShoulderPosition() - shoulderWristBottomDegree)); //Equation for wrist soft limit in porpotion to arm angle
      m_wristSubsystem.setReverseSoftLimit(limit.floatValue());
    }else{
      if(m_wristSubsystem.getReverseSoftLimint() != m_wristSubsystem.getReverseSoftLimintDefault()){
        m_wristSubsystem.setDefaultReverseSoftLimit();
      }
    }
  }
}
