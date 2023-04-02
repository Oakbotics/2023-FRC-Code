package frc.robot.commands.AutoCommands.PathPlannerAutoCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.ArmCommandLowCone;
import frc.robot.commands.ArmCommandMid;
import frc.robot.commands.IntakeCommand;

public class AutoPath extends SequentialCommandGroup {

    DriveSubsystem m_driveSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    ArmSubsystem m_armSubsystem;
    // Creates path with default velocity settings.
    public AutoPath(String path, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        this(path, new PathConstraints(Constants.AutoConstants.kMaxSpeedMetresPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared), driveSubsystem, intakeSubsystem, armSubsystem);
    
    }

    // Creates path with specified velocity settings.
    public AutoPath(String path, PathConstraints pathConstraints, DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {
        m_driveSubsystem = driveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_armSubsystem = armSubsystem;
        // Loads chosen auto path from PathPlanner.
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
                path,
                pathConstraints);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new IntakeCommand(intakeSubsystem).withTimeout(1.5));
        eventMap.put("MidCone", new ArmCommandMid(armSubsystem));
        eventMap.put("IntakeCoast", new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(false)));
        eventMap.put("Low", new ArmCommandLow(armSubsystem));
        eventMap.put("IntakeBrake",new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(true)));
        eventMap.put("LowCone", new ArmCommandLowCone(armSubsystem));
        

        

        /* Creates the AutoBuilder. */
        SwerveAutoBuilder AutoBuilder = new SwerveAutoBuilder(
                m_driveSubsystem::getPose, // Pose2d supplier.
                m_driveSubsystem::resetOdometry, // Resets the odometry at the beginning of auto.
                Constants.DriveConstants.kDriveKinematics,
                new PIDConstants(Constants.AutoConstants.kPXController, 0,Constants.AutoConstants.kDController), // PID constants to correct for translation error (X and Y).
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, Constants.AutoConstants.kDThetaControllerD), // PID constants to correct for rotation error (Creates rotation controller).
                m_driveSubsystem::setModuleStates,
                eventMap,
                true, // Mirrored depending on alliance color.
                m_driveSubsystem

        );

        Command fullAuto = AutoBuilder.fullAuto(pathGroup);
        addCommands(fullAuto);
    }
}   