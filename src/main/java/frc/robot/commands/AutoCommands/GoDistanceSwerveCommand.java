package frc.robot.commands.AutoCommands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
// import frc.robot.commands.AutoCommands.commandGroups.SwerveControllerCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class GoDistanceSwerveCommand  {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final DriveSubsystem m_driveSubsystem; 
    // private final LimelightSubsystem m_limelightSubsystem;
    private final TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetresPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

    private Trajectory exampleTrajectory; 
        
    ProfiledPIDController thetaController = new ProfiledPIDController(
        0.1, 0, 0, AutoConstants.kThetaControllerConstraints);

    SwerveControllerCommand swerveControllerCommand;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsytem The subsystem used by this command.
   */
   public GoDistanceSwerveCommand(DriveSubsystem driveSubsytem, Pose2d startingPose, Pose2d endingPose) {
    m_driveSubsystem = driveSubsytem;
    // m_limelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    exampleTrajectory= TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        //m_limelightSubsystem.getRobotPose(),
        new Pose2d(startingPose.getX(), startingPose.getY(), startingPose.getRotation()),
        // Pass through these two interior waypoints, making ans  's' curve path
        List.of(),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(endingPose.getX(), endingPose.getY(), endingPose.getRotation()),

        config);


        SmartDashboard.putNumber("DestinationX", startingPose.getX());
        SmartDashboard.putNumber("DestinationY", startingPose.getY());
        SmartDashboard.putString("Trajectory", exampleTrajectory.toString());
        SmartDashboard.putString("Theta Controller Setpoint", thetaController.getSetpoint().toString());

        Supplier<Rotation2d> rSupplier = () -> (m_driveSubsystem.getRotation());
    swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_driveSubsystem::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        thetaController),
        // rSupplier,
        m_driveSubsystem::setModuleStates,
        m_driveSubsystem);


    }

    public SwerveControllerCommand getAutonomousCommand(){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        return swerveControllerCommand;
    }

}
