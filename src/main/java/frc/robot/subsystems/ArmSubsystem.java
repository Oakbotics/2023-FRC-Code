// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;


public class ArmSubsystem extends SubsystemBase {

  ShoulderSubsystem m_shoulderSubsystem;
  WristSubsystem m_wristSubsystem;

  private final double shoulderWristBottomDegree = 40;


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
    // AdjustWristSoftLimit();
  }

  public void MoveShoulderSpeed(double speed){
      m_shoulderSubsystem.MoveShoulderSpeed(speed);
      // AdjustWristSoftLimit();
  }

  public void MoveWristDegrees(double degrees){
    m_wristSubsystem.MoveWristDegrees(degrees);
  }

  public void MoveWristSpeed(double speed){
    m_wristSubsystem.MoveWristSpeed(speed);
  }


  public double getWristPosition(){
    return m_wristSubsystem.getWristPosition();
  }

  public double getShoulderPosition(){
    return m_shoulderSubsystem.GetShoulderPosition();
  }

  public boolean isWristAtSetpoint(double setPoint){
    return m_wristSubsystem.isWristAtSetpoint(setPoint);
  }

  public boolean isShoulderAtSetpoint(double setPoint){
    return m_shoulderSubsystem.isShoulderAtSetpoint(setPoint);
  }

  // public double getShoulderSetpoint(){

  // }

  private void AdjustWristSoftLimit(){
    if (m_shoulderSubsystem.GetShoulderPosition() <= shoulderWristBottomDegree){ 
      //For every degree below 35, make wrist limit go up by 156/33
      Double limit = Math.abs(((shoulderWristBottomDegree - m_shoulderSubsystem.GetShoulderPosition()))*(134/40) - ArmConstants.WristDefaultMaxPosition);
      m_wristSubsystem.setReverseSoftLimit(limit.floatValue());
    }else{
      if(m_wristSubsystem.getReverseSoftLimit() != m_wristSubsystem.getReverseSoftLimitDefault()){
        m_wristSubsystem.setDefaultReverseSoftLimit();
      }
    }
  }



  // private void AdjustWristSoftLimit(double shoulderDegrees){
  //   if (shoulderDegrees <= shoulderWristBottomDegree){ 
  //     Double limit = (m_wristSubsystem.getReverseSoftLimit() + (shoulderDegrees - shoulderWristBottomDegree)); 
  //     m_wristSubsystem.setReverseSoftLimit(limit.floatValue());
  //   }else{
  //     if(m_wristSubsystem.getReverseSoftLimit() != m_wristSubsystem.getReverseSoftLimitDefault()){
  //       m_wristSubsystem.setDefaultReverseSoftLimit();
  //     }
  //   }
  // }
}
