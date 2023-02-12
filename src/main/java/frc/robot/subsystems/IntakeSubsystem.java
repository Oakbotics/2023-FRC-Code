// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IntakeConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intakeMotor;
    private RelativeEncoder m_encoder;
    private final double encoderMultiplier = (1/(IntakeConstants.gearBoxRatio)) * 360;
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
      m_intakeMotor = new CANSparkMax(IntakeConstants.IntakeID, MotorType.kBrushless);
      m_intakeMotor.setInverted(true);

      m_intakeMotor.restoreFactoryDefaults();

      m_encoder = m_intakeMotor.getEncoder();
      m_encoder.setPositionConversionFactor(encoderMultiplier);
  }

  public void runIntake(double intakeSpeed){
      m_intakeMotor.set(intakeSpeed);
  }

  public void reverseIntake(double intakeSpeed){
    m_intakeMotor.set(-(intakeSpeed));
}

public void stopIntake(double intakeSpeed){
    m_intakeMotor.set(0);
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
