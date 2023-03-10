// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotorTop;
    private CANSparkMax m_intakeMotorBottom;
    private RelativeEncoder m_encoderTop;
    private RelativeEncoder m_encoderBottom;
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
      m_intakeMotorTop = new CANSparkMax(IntakeConstants.IntakeTopID, MotorType.kBrushless);
      m_intakeMotorBottom = new CANSparkMax(IntakeConstants.IntakeBottomID, MotorType.kBrushless);

      m_intakeMotorTop.restoreFactoryDefaults();
      m_intakeMotorTop.setInverted(false);
      m_intakeMotorBottom.setInverted(true);

      m_encoderTop = m_intakeMotorTop.getEncoder();
      m_encoderBottom = m_intakeMotorBottom.getEncoder();

      m_intakeMotorBottom.setIdleMode(IdleMode.kBrake);
      m_intakeMotorTop.setIdleMode(IdleMode.kBrake);

      m_intakeMotorBottom.setSmartCurrentLimit(15);
      m_intakeMotorBottom.setSmartCurrentLimit(15);


  }


  public void runIntake(double intakeSpeed){
      m_intakeMotorTop.set(intakeSpeed);
      m_intakeMotorBottom.set(intakeSpeed);
  }


  public void reverseIntake(double intakeSpeed){
    m_intakeMotorTop.set(-(intakeSpeed));
    m_intakeMotorBottom.set(-(intakeSpeed));
}

public void stopIntake(double intakeSpeed){
    m_intakeMotorTop.set(0);
    m_intakeMotorBottom.set(0);
}


public void setIdleModeBrake(boolean brake){
  if(brake ){
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