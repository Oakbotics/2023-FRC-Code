// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.commandGroups;

import frc.robot.commands.ArmCommandHigh;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.ArmCommandMid;
import frc.robot.commands.ArmCommandMidCube;
import frc.robot.commands.ArmCommandSubstation;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.ShoulderDropCommand;
import frc.robot.commands.AutoCommands.GoDistanceSwerveCommand;
import frc.robot.commands.AutoCommands.GoDistanceSwerveReverseCommand;
import frc.robot.commands.AutoCommands.GoToPositionSwerveCommand;
import frc.robot.commands.AutoCommands.GoToPositionSwerveReverseCommand;
import frc.robot.commands.AutoCommands.GoToPositionSwerveReverseOverBalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class AutoSwerveCommandMidCubeOver extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_armSubsystem;
  private final DriveSubsystem m_driveSubsystem;
  private final IntakeSubsystem m_intakeSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  // private final CandleSubsystem m_candleSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public  AutoSwerveCommandMidCubeOver(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LimelightSubsystem limelightSubsystem) {
    m_armSubsystem = armSubsystem;
    m_driveSubsystem = driveSubsystem;
    m_intakeSubsystem = intakeSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_armSubsystem, m_driveSubsystem, m_intakeSubsystem);

    addCommands(
      
    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(0), m_driveSubsystem),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // // new ArmCommandLow(m_armSubsystem),
    // new GoToPositionSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-0.25,0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // // new ArmCommandMidCube(m_armSubsystem),
    // // new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
    // // new ArmCommandLow(m_armSubsystem),
    // new GoToPositionSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-4.8,0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new GoToPositionSwerveCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-2.4,0), Rotation2d.fromDegrees(0))).getAutonomousCommand()

    // // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    // // new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)


    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(0), m_driveSubsystem),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandLow(m_armSubsystem),
    // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(4.8,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(4.65, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandMidCube(m_armSubsystem),
    // new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
    // new ArmCommandLow(m_armSubsystem),
    // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(4.65,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new GoDistanceSwerveCommand(m_driveSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(2.4, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    // new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)
       


    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(0), m_driveSubsystem),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandLow(m_armSubsystem),
    // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-0.25, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandMidCube(m_armSubsystem),
    // new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
    // // new ArmCommandLow(m_armSubsystem),
    // // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-0.25,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-3.75, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new GoDistanceSwerveCommand(m_driveSubsystem, new Pose2d(new Translation2d(-3.75,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-1.6, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    // new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)


    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandLow(m_armSubsystem),
    // new GoDistanceSwerveCommand(m_driveSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(180)), new Pose2d(new Translation2d(0.25, 0), Rotation2d.fromDegrees(180))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new ArmCommandMidCube(m_armSubsystem),
    // new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
    // new ArmCommandLow(m_armSubsystem),
    // new GoDistanceSwerveCommand(m_driveSubsystem, new Pose2d(new Translation2d(0.25,0), Rotation2d.fromDegrees(180)), new Pose2d(new Translation2d(3.75, 0), Rotation2d.fromDegrees(180))).getAutonomousCommand(),
    // // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem,  new Pose2d(new Translation2d(3.75,0), Rotation2d.fromDegrees(180)), new Pose2d(new Translation2d(1.6, 0), Rotation2d.fromDegrees(180))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    // new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)

    new InstantCommand(()-> m_driveSubsystem.zeroHeading(0), m_driveSubsystem),
    new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    new ArmCommandLow(m_armSubsystem),
    new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-0.25, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    new ArmCommandMidCube(m_armSubsystem),
    new OuttakeCommand(m_intakeSubsystem).repeatedly().withTimeout(1),
    // new ArmCommandLow(m_armSubsystem),
    new ParallelCommandGroup(
          new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-0.25,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-3.75, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
          new ArmCommandLow(m_armSubsystem)
    ),
    // new GoDistanceSwerveReverseCommand(m_driveSubsystem, m_limelightSubsystem, new Pose2d(new Translation2d(-0.25,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-3.75, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    // new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    new GoDistanceSwerveCommand(m_driveSubsystem, new Pose2d(new Translation2d(-3.75,0), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(-1.5, 0), Rotation2d.fromDegrees(0))).getAutonomousCommand(),
    new DriveCommand(m_driveSubsystem, 0, 0, 0).withTimeout(0.1),
    // new InstantCommand(()-> m_driveSubsystem.zeroHeading(180), m_driveSubsystem),
    new RunCommand(()-> m_driveSubsystem.setX(), m_driveSubsystem)

        );
    }
}
