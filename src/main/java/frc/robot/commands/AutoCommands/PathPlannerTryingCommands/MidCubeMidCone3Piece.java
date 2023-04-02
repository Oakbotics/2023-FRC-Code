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
import frc.robot.commands.ShoulderDropCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoCommands.PathPlannerTryingCommands.AutoPath;



public class MidCubeMidCone3Piece extends SequentialCommandGroup {

    public MidCubeMidCone3Piece(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        // Specify velocity limitations.
        PathConstraints velocity = new PathConstraints(3, 3);

        // Play command sequence.
        // Added Instant command to reset the speed of the Swerve to 100% to ensure that it is not in slowmode and can successfully auto level.
        addCommands( 
            new InstantCommand(()-> driveSubsystem.zeroHeading(), driveSubsystem),
            new ArmCommandLow(armSubsystem),
            new ParallelCommandGroup(
                new AutoPath("Back30cmCube", velocity, driveSubsystem, intakeSubsystem)
                // new ArmCommandMid(armSubsystem)
            ),
            new OuttakeCommand(intakeSubsystem).withTimeout(0.5),
            new ParallelCommandGroup(
                new ArmCommandLowCone(armSubsystem),
                new AutoPath("ForwardPath1", velocity, driveSubsystem, intakeSubsystem)
            ),
            new ParallelCommandGroup(
                new ArmCommandLow(armSubsystem),
                new AutoPath("ReversePath1", velocity, driveSubsystem,intakeSubsystem)
            ),
            // new ArmCommandMid(armSubsystem),    
            new ParallelCommandGroup(
                // new ShoulderDropCommand(armSubsystem, intakeSubsystem),
                new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(false))
            ),
            new ParallelCommandGroup(
                new ArmCommandLowCone(armSubsystem),
                new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(true)),
                new AutoPath("ForwardPath2", velocity, driveSubsystem, intakeSubsystem)
            ),
            new ParallelCommandGroup(
                // new ArmCommandMid(armSubsystem),
                // new AutoPath("ReversePath2", velocity, driveSubsystem,intakeSubsystem)
            ),
            new InstantCommand(()-> driveSubsystem.zeroHeading(180), driveSubsystem),
            new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(false)),
            // new ShoulderDropCommand(armSubsystem, intakeSubsystem),
            // new AutoPath("Back30cmBottemCone", driveSubsystem, intakeSubsystem),
            new InstantCommand(()-> intakeSubsystem.setIdleModeBrake(true)),
            new ArmCommandLow(armSubsystem)
           
            );
    }

}