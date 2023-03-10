// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.time.Instant;
import java.util.List;
import java.util.Map;

import frc.robot.commands.ArmCommandGroup.IntakeCone;
import frc.robot.commands.ArmCommandGroup.WristMoveDegreeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.ChargeStation;
import frc.robot.commands.AutoCommands.ExperimentalGoToPositionSwerveCommand;
import frc.robot.commands.AutoCommands.commandGroups.BlueCommandGroups.AutoExperimentalSwerveCommand;
import frc.robot.commands.AutoCommands.commandGroups.BlueCommandGroups.AutoGoForward;
import frc.robot.commands.AutoCommands.commandGroups.BlueCommandGroups.AutoOuttakeReverse;
import frc.robot.commands.AutoCommands.commandGroups.BlueCommandGroups.AutoSwerveCommandMid;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();  
  private final ShoulderSubsystem m_shoulderSubsystem = new ShoulderSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_shoulderSubsystem, m_wristSubsystem);
  private final CandleSubsystem m_candleSubsystem = new CandleSubsystem(Constants.LightConstants.CANdleID);
  private final LimelightSubsystem m_limelightSubsystem = new LimelightSubsystem();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // The robot's subsystems and commands are defined here...
  private final XboxController m_opController = new XboxController(1);

 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    
  }



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
        
    // new JoystickButton(m_opController, XboxController.Button.kA.value).onTrue(new ArmCommandLow(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kA.value).onTrue(new ArmCommandLow(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kY.value).onTrue(new ArmCommandHigh(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kX.value).onTrue(new ArmCommandMid(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kB.value).onTrue(new ArmCommandSubstation(m_armSubsystem));
    

    new JoystickButton(m_opController, XboxController.Button.kRightBumper.value).onTrue(new PurpleCandleCommand(m_candleSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value).onTrue(new OrangeCandleCommand(m_candleSubsystem));
    
    new JoystickButton(m_driverController, XboxController.Button.kY.value).onTrue(new ArmCommandLowCone(m_armSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(new ArmCommandLow(m_armSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new IntakeCommand(m_intakeSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new OuttakeCommand(m_intakeSubsystem));
    
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(()-> m_robotDrive.toggleFieldRelative()));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
                  m_robotDrive));
                  
    new POVButton(m_driverController, 0)
      .onTrue(new InstantCommand(
        () -> m_robotDrive.zeroHeading()  //Add limelight, and then test
      ));

    new POVButton(m_driverController, 90)
      .onTrue(new InstantCommand(
        () -> m_robotDrive.zeroHeading(
          m_limelightSubsystem.getRobotAngle()
        )  //Add limelight, and then test
      ));

    //Needs testing
    new Trigger(
      () -> m_driverController.getLeftTriggerAxis() != 0
    ).whileTrue(
      new ConeIntakeWristCommand(m_armSubsystem, m_intakeSubsystem)
    ).onFalse(
      new WristMoveDegreeCommand(m_armSubsystem, ArmConstants.WristRestPosition)
    );

    // new JoystickButton(m_opController, XboxController.Axis.kLeftY).onTrue(new ArmSpeedCommand(m_armSubsystem, m_opController))

   
    // new JoystickButton(m_opController, XboxController.Button.kB.value).whileTrue(new IntakeCone(m_intakeSubsystem, m_armSubsystem));

  
    // new POVButton(m_opController, 0).toggleOnTrue(
    //   new RunCommand(
    //   () ->
    //     m_candleSubsystem.stopLight(), m_candleSubsystem));

    m_candleSubsystem.setDefaultCommand(new CandleAnimateCommand(m_candleSubsystem));
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // new RunCommand(
        //     () ->
            
        //       m_robotDrive.drive(
        //         -MathUtil.applyDeadband(m_driverController.getLeftY() * ((m_driverController.getLeftTriggerAxis() > 0 )? DriveConstants.kSpeedLimiter : 1), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getLeftX() * ((m_driverController.getLeftTriggerAxis() > 0 )? DriveConstants.kSpeedLimiter : 1), OIConstants.kDriveDeadband),
        //         -MathUtil.applyDeadband(m_driverController.getRightX() * ((m_driverController.getLeftTriggerAxis() > 0 )? DriveConstants.kSpeedLimiter : 1), OIConstants.kDriveDeadband),
        //         true, true),
        //     m_robotDrive));

        new RunCommand(
            () ->
            
              m_robotDrive.drive(
                //Gradual braking on trigger needs testing
                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.1 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband), 
                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.1 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * (1.1 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    
    new Trigger(
      () -> m_opController.getLeftY() != 0
    ).whileTrue(
      new ShoulderSpeedCommand(m_armSubsystem, 
        () -> m_opController.getLeftY() * 0.5
    ));

    // new Trigger(
    //   () -> m_opController.getRightY() != 0
    // ).whileTrue(
    //   new WristSpeedCommand(m_armSubsystem, 
    //     () -> m_opController.getRightY()
    // ));

   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   AutoScorePreloadMid command = new AutoScorePreloadMid(m_armSubsystem, m_robotDrive, m_intakeSubsystem, m_limelightSubsystem);
  //   return command.andThen(() -> m_robotDrive.drive(0, 0, 0, true, true));
  // }
  public Command getAutonomousCommand() {
    // m_robotDrive.zeroHeading();
    // SwerveControllerCommand command = new AutoScorePreloadMid(m_armSubsystem,m_robotDrive, m_intakeSubsystem, m_limelightSubsystem );
    return new AutoSwerveCommandMid(m_armSubsystem,m_robotDrive, m_intakeSubsystem, m_limelightSubsystem ).andThen(() -> m_robotDrive.drive(0, 0, 0, true));
  }
}
