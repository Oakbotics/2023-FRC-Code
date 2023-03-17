// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.ArmCommandGroup.ShoulderMoveDegreeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ShoulderDropCommand extends SequentialCommandGroup  {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final double startingDegrees;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShoulderDropCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    m_armSubsystem = armSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_intakeSubsystem);

    startingDegrees = m_armSubsystem.getShoulderPosition();

    addCommands(
      new InstantCommand(()-> m_intakeSubsystem.setIdleModeBrake(false)),
      new ShoulderMoveDegreeCommand(m_armSubsystem, 72)        
      //new IntakeCommand(m_intakeSubsystem).repeatedly()
    );
  }

  public double startingDegrees(){
    return startingDegrees;
  }

}