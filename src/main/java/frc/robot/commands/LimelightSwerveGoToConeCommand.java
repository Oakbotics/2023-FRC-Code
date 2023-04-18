// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.commands.ArmCommandHigh;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.ArmCommandMid;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.AutoCommands.GoDistanceSwerveCommand;
import frc.robot.commands.AutoCommands.GoToPositionSwerveReverseCommand;
import frc.robot.commands.AutoCommands.PathPlannerAutoCommands.PathPlanGoToPositionSwerveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ReflectiveTapeLimelightSubsystem;

import java.text.DecimalFormat;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** An example command that uses an example subsystem. */
public class LimelightSwerveGoToConeCommand extends SequentialCommandGroup {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final ReflectiveTapeLimelightSubsystem m_limelightSubsystem;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public LimelightSwerveGoToConeCommand( DriveSubsystem driveSubsystem, ReflectiveTapeLimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limelightSubsystem, m_driveSubsystem);
    DecimalFormat df = new DecimalFormat("#.##");
    Supplier<Double> limelightVal = () -> m_limelightSubsystem.getDistanceFromReflectiveTape();

    addCommands(
        // new InstantCommand(()-> SmartDashboard.putNumber("Limelight Distance Y (instant command)", Double.parseDouble(df.format(m_limelightSubsystem.getDistanceFromReflectiveTape())))),
        // new PathPlanGoToPositionSwerveCommand(m_driveSubsystem, () -> m_limelightSubsystem.getDistanceFromReflectiveTape()).getAutonomousCommand()
        new PathPlanGoToPositionSwerveCommand(m_driveSubsystem, new Pose2d(m_driveSubsystem.getPose().getTranslation()
            .plus(new Translation2d(0, Double.parseDouble(df.format(m_limelightSubsystem.getDistanceFromReflectiveTape())))), new Rotation2d(0))).getAutonomousCommand()

        );
    

    }

// m_limelightSubsystem.getDistanceFromReflectiveTape().getY()

  // Called when the command is initially schePduled.

  
}
