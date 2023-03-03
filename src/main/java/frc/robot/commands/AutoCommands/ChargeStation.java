// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/** An example command that uses an example subsystem. */
public class ChargeStation {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final DriveSubsystem m_driveSubsystem; 
    private final TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedFeetPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    private final Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),

        config);
        
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        
    SwerveControllerCommand swerveControllerCommand;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsytem The subsystem used by this command.
   */
   public ChargeStation(DriveSubsystem driveSubsytem) {
    m_driveSubsystem = driveSubsytem;
    // Use addRequirements() here to declare subsystem dependencies.
    
    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_driveSubsystem::setModuleStates,
        m_driveSubsystem);


    }

    public SwerveControllerCommand getAutonomousCommand(){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        return swerveControllerCommand;
    }
}