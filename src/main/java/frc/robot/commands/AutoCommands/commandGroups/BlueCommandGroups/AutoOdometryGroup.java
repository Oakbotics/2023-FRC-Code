// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.commandGroups.BlueCommandGroups;

import frc.robot.Constants.BlueFieldConstants;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.ArmCommandMid;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.AutoCommands.GoToPositionSwerveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class AutoOdometryGroup extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoOdometryGroup(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LimelightSubsystem limelightSubsystem) {
    m_armSubsystem = armSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_driveSubsystem, m_intakeSubsystem);

    addCommands(
        new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(1),
        new InstantCommand(()-> m_driveSubsystem.zeroHeading(), m_driveSubsystem),
        new GoToPositionSwerveCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(2,0), Rotation2d.fromDegrees(0))).getAutonomousCommand()
        //new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)
        //new GoToPositionSwerveCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(BlueFieldConstants.chargeStationPosition, Rotation2d.fromDegrees(0))).getAutonomousCommand()
    );
    

    }



  // Called when the command is initially schePduled.

  
}
