package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoAlignStrafingConstants;
import frc.robot.Constants.TurningConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ReflectiveTapeLimelightSubsystem;

/** A command that will turn the robot to the specified angle. */
public class AutoAlignXSwerve extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public AutoAlignXSwerve(ReflectiveTapeLimelightSubsystem limelight, DriveSubsystem drive) {
    super(
        new PIDController(AutoAlignStrafingConstants.kP, AutoAlignStrafingConstants.kI, AutoAlignStrafingConstants.kD),
        // Close loop on heading
        limelight::getDistanceFromReflectiveTape,
        // Set reference to target
        0,
        // Pipe output to strafe robot
        output -> drive.drive(0, output, 0, true),
        // Require the drive
        drive,
        limelight);

    SmartDashboard.putString("Strafing Finished", "Just Started");
    drive.setFieldRelative();

 
    // Set the controller to be continuous (because it is an angle controller)
    // getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(AutoAlignStrafingConstants.kTurnToleranceDeg, AutoAlignStrafingConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }


}