// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommandGroup;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** An example command that uses an example subsystem. */
public class WristMoveDegreeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;
    private double m_degrees;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public WristMoveDegreeCommand(ArmSubsystem subsystem, double degrees) {
    m_ArmSubsystem = subsystem;
    m_degrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_ArmSubsystem.MoveWristDegrees(m_degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("End", "Mid Ended");
    

  }

  // @Override
  // public void getInterruptionBehavior(){

  // }

//   Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ArmSubsystem.isWristAtSetpoint(m_degrees);
  }
}
