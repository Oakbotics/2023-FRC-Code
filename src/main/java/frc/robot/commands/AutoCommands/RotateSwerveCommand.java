package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurningConstants;
import frc.robot.subsystems.DriveSubsystem;

/** A command that will turn the robot to the specified angle. */
public class RotateSwerveCommand extends PIDCommand {
  /**
   * Turns to robot to the specified angle.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive The drive subsystem to use
   */
  public RotateSwerveCommand(double targetAngleDegrees, DriveSubsystem drive) {
    super(
        new PIDController(TurningConstants.kP, TurningConstants.kI, TurningConstants.kD),
        // Close loop on heading
        drive::getHeading,
        // Set reference to target
        -targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.drive(0, 0, output, true),
        // Require the drive
        drive);

    SmartDashboard.putString("Turning Finished", "Just Started");
 
    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(TurningConstants.kTurnToleranceDeg, TurningConstants.kTurnRateToleranceDegPerS);
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atSetpoint();
  }



}