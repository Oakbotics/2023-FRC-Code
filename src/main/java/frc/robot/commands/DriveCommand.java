// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem m_driveTrain;
  private Double m_forward;
  private Double m_leftSpeed;
  private Double m_angularSpeed;
  private Boolean isFieldCentric;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveTrain, Double forwardSpeed, Double leftSpeed, Double angularSpeed, Boolean fieldCentric) {
    m_driveTrain = driveTrain;
    m_forward = forwardSpeed;
    m_leftSpeed = leftSpeed;
    m_angularSpeed = angularSpeed;
    isFieldCentric = fieldCentric;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("m_forward Speed", m_forward);
    SmartDashboard.putNumber("m_left Speed", m_leftSpeed);
    m_driveTrain.drive(m_forward, m_leftSpeed, m_angularSpeed, isFieldCentric );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
