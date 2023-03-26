package frc.robot.commands.AutoCommands.PathPlannerTryingCommands;


import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommandHigh;
import frc.robot.commands.ArmCommandLow;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.AutoCommands.PathPlannerTryingCommands.AutoPath;



public class BealSteal extends SequentialCommandGroup {

    public BealSteal(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem) {

        // Specify velocity limitations.
        PathConstraints velocity = new PathConstraints(0.5, 0.5);

        // Play command sequence.
        // Added Instant command to reset the speed of the Swerve to 100% to ensure that it is not in slowmode and can successfully auto level.
        addCommands( 
            new InstantCommand(()-> driveSubsystem.zeroHeading(), driveSubsystem),
            new ArmCommandLow(armSubsystem),
            new AutoPath("Back30cm", velocity, driveSubsystem),
            new ArmCommandHigh(armSubsystem),
            new AutoPath("Front30cm", velocity, driveSubsystem),
            new OuttakeCommand(intakeSubsystem).repeatedly().withTimeout(1),
            new ArmCommandLow(armSubsystem),
            new AutoPath("ToChargeStation", velocity, driveSubsystem)
            
                
        );
    }

}