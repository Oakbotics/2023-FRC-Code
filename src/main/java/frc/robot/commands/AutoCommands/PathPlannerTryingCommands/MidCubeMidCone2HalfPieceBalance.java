package frc.robot.commands.AutoCommands.PathPlannerTryingCommands;


import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.commands.ShoulderDropCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoCommands.PathPlannerTryingCommands.AutoPath;



public class MidCubeMidCone2HalfPieceBalance extends SequentialCommandGroup {

    public MidCubeMidCone2HalfPieceBalance(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        // Specify velocity limitations.
        PathConstraints velocity = new PathConstraints(3, 3);

        // Play command sequence.
        // Added Instant command to reset the speed of the Swerve to 100% to ensure that it is not in slowmode and can successfully auto level.
        addCommands( 
            new InstantCommand(()-> driveSubsystem.zeroHeading(), driveSubsystem),
            new ArmCommandLow(armSubsystem),
            new ParallelCommandGroup(
                new AutoPath("Back30cmCube", velocity, driveSubsystem, intakeSubsystem, armSubsystem),
                new ArmCommandMidCube(armSubsystem)
            ),
            new OuttakeCommand(intakeSubsystem).withTimeout(0.5),
            new AutoPath("ForwardPath1", velocity, driveSubsystem, intakeSubsystem, armSubsystem),
            new AutoPath("ReversePath1", velocity, driveSubsystem,intakeSubsystem, armSubsystem),
            new ParallelCommandGroup(
                new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(false)),
                new ShoulderDropCommand(armSubsystem, intakeSubsystem)
            ),
            new AutoPath("ForwardPath2", velocity, driveSubsystem, intakeSubsystem, armSubsystem),
            new AutoPath("2.5Balance", velocity, driveSubsystem, intakeSubsystem, armSubsystem),
            new InstantCommand(()-> driveSubsystem.zeroHeading(180), driveSubsystem),
            new RunCommand(()-> driveSubsystem.setX(), driveSubsystem)
        );
    }
}