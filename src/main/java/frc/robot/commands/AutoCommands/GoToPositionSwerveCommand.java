package frc.robot.commands.AutoCommands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class GoToPositionSwerveCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final DriveSubsystem m_driveSubsystem; 
    // private final LimelightSubsystem m_limelightSubsystem;
    private final TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetresPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false );

    private Trajectory exampleTrajectory; 
        
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        
    SwerveControllerCommand swerveControllerCommand;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsytem The subsystem used by this command.
   */
   public GoToPositionSwerveCommand(DriveSubsystem driveSubsytem, LimelightSubsystem limelightSubsystem, Pose2d destination) {
    m_driveSubsystem = driveSubsytem;
    // m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.

    exampleTrajectory= TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //m_limelightSubsystem.getRobotPose(),
        new Pose2d(0,0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making ans  's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        destination,

        config);

        SmartDashboard.putNumber("BotPoseX", limelightSubsystem.getRobotPose().getX());
        SmartDashboard.putNumber("BotPoseY", limelightSubsystem.getRobotPose().getY());
        SmartDashboard.putNumber("DestinationX", destination.getX());
        SmartDashboard.putNumber("DestinationY", destination.getY());
        SmartDashboard.putString("Trajectory", exampleTrajectory.toString());

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
