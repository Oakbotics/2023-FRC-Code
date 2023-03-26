package frc.robot.commands.AutoCommands.PathPlannerTryingCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoPath extends SequentialCommandGroup {

    DriveSubsystem m_driveSubsystem;

    // Creates path with default velocity settings.
    public AutoPath(String path, DriveSubsystem driveSubsystem) {
        this(path, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetresPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), driveSubsystem);
    
    }

    // Creates path with specified velocity settings.
    public AutoPath(String path, PathConstraints pathConstraints, DriveSubsystem driveSubsystem) {
        m_driveSubsystem = driveSubsystem;
        // Loads chosen auto path from PathPlanner.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                path,
                pathConstraints);

        HashMap<String, Command> eventMap = new HashMap<>();
        

        /* Creates the AutoBuilder. */
        SwerveAutoBuilder AutoBuilder = new SwerveAutoBuilder(
                m_driveSubsystem::getPose, // Pose2d supplier.
                m_driveSubsystem::resetOdometry, // Resets the odometry at the beginning of auto.
                Constants.DriveConstants.kDriveKinematics,
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (X and Y).
                new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (Creates rotation controller).
                m_driveSubsystem::setModuleStates,
                eventMap,
                true, // Mirrored depending on alliance color.
                m_driveSubsystem

        );

        Command fullAuto = AutoBuilder.fullAuto(pathGroup);
        addCommands(fullAuto);
    }
}   