// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AprilTagGoToZero extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private  final LimelightSubsystem m_LimelightSubsystem;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotateController;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AprilTagGoToZero(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LimelightSubsystem = limelightSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, limelightSubsystem);


     xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.1
     yController = new PIDController(0, 0, 0);
     rotateController = new PIDController(0, 0, 0);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        double botPoseX = m_LimelightSubsystem.getBotPose().getX();// field relative
        double targetPoseX = m_LimelightSubsystem.getAprilTagDistance().getX() + 3;// relative to robot
        double botPoseY = m_LimelightSubsystem.getBotPose().getY();
        double botPoseRotate = m_LimelightSubsystem.getBotPose().getRotation().getDegrees();
        // m_driveSubsystem.drive(xController.calculate(botPoseX,0), yController.calculate(botPoseY, 0),  rotateController.calculate(botPoseRotate, 0), true);    
        m_driveSubsystem.drive(xController.calculate(botPoseX, 12.75), 0, 0, true);    

    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {



}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}