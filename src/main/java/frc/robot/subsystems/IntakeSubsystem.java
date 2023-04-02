// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotorTop;
    private CANSparkMax m_intakeMotorBottom;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
      m_intakeMotorTop = new CANSparkMax(IntakeConstants.intakeTopID, MotorType.kBrushless);
      m_intakeMotorBottom = new CANSparkMax(IntakeConstants.intakeBottomID, MotorType.kBrushless);

      m_intakeMotorTop.restoreFactoryDefaults();
      m_intakeMotorTop.setInverted(true);
      m_intakeMotorBottom.setInverted(false);

      // m_encoderTop = m_intakeMotorTop.getEncoder();
      // m_encoderBottom = m_intakeMotorBottom.getEncoder();

      m_intakeMotorBottom.setIdleMode(IdleMode.kBrake);
      m_intakeMotorTop.setIdleMode(IdleMode.kBrake);

      m_intakeMotorBottom.setSmartCurrentLimit(20);
      m_intakeMotorTop.setSmartCurrentLimit(20);


  }


  public void runIntake(double intakeVoltage){
      m_intakeMotorTop.setVoltage(intakeVoltage);
      m_intakeMotorBottom.setVoltage(intakeVoltage);
  }


  public void reverseIntake(double intakeVoltage){
    m_intakeMotorTop.setVoltage(-(intakeVoltage));
    m_intakeMotorBottom.setVoltage(-(intakeVoltage));
}

public void stopIntake(double intakeSpeed){
    m_intakeMotorTop.setVoltage(0);
    m_intakeMotorBottom.setVoltage(0);
}


public void setIdleModeBrake(boolean brake){
  if(brake){
    m_intakeMotorBottom.setIdleMode(IdleMode.kBrake);
    m_intakeMotorTop.setIdleMode(IdleMode.kBrake);
  }
  else{
    m_intakeMotorBottom.setIdleMode(IdleMode.kCoast);
    m_intakeMotorTop.setIdleMode(IdleMode.kCoast);
  }


}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}