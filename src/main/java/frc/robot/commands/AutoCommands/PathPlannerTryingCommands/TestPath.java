package frc.robot.commands.AutoCommands.PathPlannerTryingCommands;


import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommandHigh;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.ArmCommandLowCone;
import frc.robot.commands.ArmCommandMid;
import frc.robot.commands.ArmCommandMidCube;
import frc.robot.commands.ConeIntakeWristCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoCommands.PathPlannerTryingCommands.AutoPath;



public class TestPath extends SequentialCommandGroup {

    public TestPath(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        // Specify velocity limitations.
        PathConstraints velocity = new PathConstraints(2.0, 2.0);

        // Play command sequence.
        // Added Instant command to reset the speed of the Swerve to 100% to ensure that it is not in slowmode and can successfully auto level.
        addCommands( 
            new InstantCommand(()-> driveSubsystem.zeroHeading(), driveSubsystem),
            new ArmCommandHigh(armSubsystem),
            new OuttakeCommand(intakeSubsystem).withTimeout(1.5),
            new ParallelCommandGroup(
                new ArmCommandLow(armSubsystem),
                new AutoPath("TestPath", velocity, driveSubsystem)
            ),
            new ParallelCommandGroup(
                new ArmCommandLowCone(armSubsystem),
                new IntakeCommand(intakeSubsystem).withTimeout(2),
                new AutoPath("Intake Path", velocity, driveSubsystem)
            ),
            new ParallelCommandGroup(
                new ArmCommandMidCube(armSubsystem),
                new AutoPath("ReversePath", velocity, driveSubsystem)
            ),
            new ArmCommandHigh(armSubsystem),
            new OuttakeCommand(intakeSubsystem).withTimeout(1.5),
            new ArmCommandLow(armSubsystem)
        );
    }

}