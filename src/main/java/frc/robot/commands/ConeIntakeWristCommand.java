// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ArmCommandGroup.ShoulderMoveDegreeCommand;
import frc.robot.commands.ArmCommandGroup.WristMoveDegreeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ConeIntakeWristCommand extends SequentialCommandGroup  {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ConeIntakeWristCommand(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {
    m_armSubsystem = armSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_intakeSubsystem);

    addCommands(
      // new WristMoveDegreeCommand(m_armSubsystem, ArmConstants.WristRestPosition),        
      new ShoulderMoveDegreeCommand(m_armSubsystem, 0.5),
      new WristMoveDegreeCommand(armSubsystem, 24)
      //new IntakeCommand(m_intakeSubsystem).repeatedly()
    );

    


  }

}