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
    double xSetPoint = 12.75;
    double ySetPoint = 1.5;
    double rotSetPoint = 0;

     xController = new PIDController(0.5, 0,1.15); //Best values: 0.5, 0, 1.15
     yController = new PIDController(0.5, 0,1.15);
     rotateController = new PIDController(0.5, 0, 0.2);
    
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
        double botPoseRotate = m_LimelightSubsystem.getBotPose().getRotation().getRadians();
        
        double xSetPoint = 12.75;
        double ySetPoint = 2;
        double rotSetPoint = 0;
        double errorMargin = 0.05;

        if(Math.abs(botPoseX - xSetPoint) <= errorMargin){
          botPoseX = xSetPoint;
        }
          
       m_driveSubsystem.drive(xController.calculate(botPoseX, xSetPoint), yController.calculate(botPoseY, ySetPoint), rotateController.calculate(botPoseRotate, rotSetPoint), true);    
        // m_driveSubsystem.drive(0, 0, rotateController.calculate(botPoseRotate, rotSetPoint), true);    

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