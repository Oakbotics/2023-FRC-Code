// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.ArmCommandGroup.ShoulderMoveDegreeCommand;
import frc.robot.commands.ArmCommandGroup.WristMoveDegreeCommand;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ArmCommandDoubleSubstation extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommandDoubleSubstation(ArmSubsystem subsystem) {
    m_ArmSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    addCommands(
      //new ShoulderMoveDegreeCommand(m_ArmSubsystem, m_ArmSubsystem.getShoulderPosition()),
      new ShoulderMoveDegreeCommand(m_ArmSubsystem, 83),
      new WristMoveDegreeCommand(m_ArmSubsystem, 108)
      //new WristMoveDegreeCommand(m_ArmSubsystem, 120)

    );
    
  }

  // Called when the command is initially scheduled.
}
