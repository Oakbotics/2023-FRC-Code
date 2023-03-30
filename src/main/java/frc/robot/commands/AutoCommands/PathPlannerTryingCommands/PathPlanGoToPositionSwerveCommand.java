package frc.robot.commands.AutoCommands.PathPlannerTryingCommands;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
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


public class PathPlanGoToPositionSwerveCommand  {  //NOT GOING TO USE
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private final DriveSubsystem m_driveSubsystem; 
    private final PPSwerveControllerCommand swerveCommand;
    // private final LimelightSubsystem m_limelightSubsystem;
    private final TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetresPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);

        
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

    SwerveControllerCommand swerveControllerCommand;
    
  /**
   * Creates a new ExampleCommand.
   *
   * @param driveSubsytem The subsystem used by this command.
   */
   public PathPlanGoToPositionSwerveCommand(DriveSubsystem driveSubsytem, Pose2d  endingPose) {
    m_driveSubsystem = driveSubsytem;

    SmartDashboard.putNumber("Smart y val", endingPose.getY());

    PathPlannerTrajectory traj = PathPlanner.generatePath(
        new PathConstraints(2, 2), 
        new PathPoint(m_driveSubsystem.getPose().getTranslation(), m_driveSubsystem.getPose().getRotation()),
        // new PathPoint(endingPose, m_driveSubsystem.getPose().getRotation())
        new PathPoint(endingPose.getTranslation(), m_driveSubsystem.getPose().getRotation())
    );



    swerveCommand = new PPSwerveControllerCommand(
        traj, 
        m_driveSubsystem::getPose, // Pose supplier
        DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        m_driveSubsystem::setModuleStates, // Module states consumer
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        m_driveSubsystem // Requires this drive subsystem
    );


    }

    public PPSwerveControllerCommand getAutonomousCommand(){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        // m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());
        return swerveCommand;
    }     

}
