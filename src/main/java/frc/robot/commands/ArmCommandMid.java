// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.commands.ArmCommandGroup.ShoulderMoveDegreeCommand;
import frc.robot.commands.ArmCommandGroup.WristMoveDegreeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class ArmCommandMid extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_ArmSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommandMid(ArmSubsystem subsystem) {
    m_ArmSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);

    addCommands(
      //new ShoulderMoveDegreeCommand(m_ArmSubsystem, m_ArmSubsystem.getShoulderPosition()),
      new ShoulderMoveDegreeCommand(m_ArmSubsystem, 84),
      new WristMoveDegreeCommand(m_ArmSubsystem, 108)
      //new WristMoveDegreeCommand(m_ArmSubsystem, 120)

    );
    
  }

  // Called when the command is initially scheduled.
}
