// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.ArmCommandGroup.WristMoveDegreeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ReflectiveTapeLimelightSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.SwerveExampleAuto;
import frc.robot.commands.AutoCommands.commandGroups.AutoSwerveCommandHighCone;
import frc.robot.commands.AutoCommands.commandGroups.AutoSwerveCommandHighCube;
import frc.robot.commands.AutoCommands.commandGroups.AutoSwerveCommandMid;
import frc.robot.commands.AutoCommands.commandGroups.AutoSwerveCommandMidCube;
import frc.robot.commands.AutoCommands.commandGroups.AutoSwerveCommandNoOutake;
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
  private final ReflectiveTapeLimelightSubsystem m_reflectiveLimelight = new ReflectiveTapeLimelightSubsystem();
  // The driver's controller
  // The robot's subsystems and commands are defined here...
  private final XboxController m_driverController = new XboxController(Constants.ControllerConstants.driverControllerId);
  private final XboxController m_opController = new XboxController(Constants.ControllerConstants.operatorControllerId);

  

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands

    SmartDashboard.putNumber("Auto Distance", 1);
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
        
        // Operator Controls

    new JoystickButton(m_opController, XboxController.Button.kA.value).onTrue(new ArmCommandLow(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kY.value).onTrue(new ArmCommandDoubleSubstation(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kX.value).onTrue(new ArmCommandMid(m_armSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kB.value).onTrue(new ArmCommandMidCube(m_armSubsystem));
    
    new JoystickButton(m_opController, XboxController.Button.kRightBumper.value).onTrue(new PurpleCandleCommand(m_candleSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value).onTrue(new OrangeCandleCommand(m_candleSubsystem));

  new Trigger( 
    () -> m_opController.getLeftTriggerAxis() != 0
  ).onTrue( // sets on true
    new ShoulderDropCommand(m_armSubsystem, m_intakeSubsystem)
  );

    // new Trigger(
    //   () -> m_opController.getLeftY() != 0
    // ).whileTrue(
    //   new ShoulderSpeedCommand(m_armSubsystem, 
    //     () -> m_opController.getLeftY() * 0.5
    // ));

    // new Trigger(
    //   () -> m_opController.getRightY() != 0
    // ).whileTrue(
    //   new WristSpeedCommand(m_armSubsystem, 
    //     () -> m_opController.getRightY()
    // ));


  

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(new IntakeCommand(m_intakeSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new OuttakeCommand(m_intakeSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(new InstantCommand(()-> m_robotDrive.toggleFieldRelative()));
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .toggleOnTrue(new RunCommand(
            () -> m_robotDrive.setX(),
                  m_robotDrive));
                  
     new POVButton(m_driverController, 0)
      .onTrue(new InstantCommand(
        () -> m_robotDrive.zeroHeading()  
      ));

    // new POVButton(m_driverController, 90)
    //   .onTrue( 
    //     new PathSwerveRotateToZero(m_robotDrive, 0).getAutonomousCommand()


        // new InstantCommand(
        // () -> m_robotDrive.zeroHeading(
        //   m_limelightSubsystem.getRobotAngle()
        // )  //Add limelight, and then test
      // );
      
    new POVButton(m_driverController, 180)
      .whileTrue(
        new RotateSwerveCommand(0, m_robotDrive).withTimeout(0.5)
      .andThen(
        new InstantCommand(()-> SmartDashboard.putString("Turning Finished", "Finished"))
      )
      .andThen(
        new BetterPPSwerveControllerCommand(
          new PathConstraints(1, 1),
          m_robotDrive::getPose, // Pose supplier
          DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          new PIDController(5.0, 0.0, 0.0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(5.0, 0.0, 0.0), // Y controller (usually the same values as X controller)
          new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          m_robotDrive::setModuleStates, // Module states consumer
          true,          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          m_reflectiveLimelight,
          m_robotDrive // Requires this drive subsystem
        )
      )); //Mr Smith changed to on true
      
    
    // new POVButton(m_driverController, 270)
    //   .onTrue(new LimelightSwerveGoToConeCommand(m_robotDrive, m_reflectiveLimelight)
    //   .andThen(new InstantCommand(()-> SmartDashboard.putString("Moved to Cone", "Finished"))

    //   ));

    new POVButton(m_driverController, 270)
      .onTrue(
        
        new BetterPPSwerveControllerCommand(
          new PathConstraints(1, 1),
          m_robotDrive::getPose, // Pose supplier
          DriveConstants.kDriveKinematics, // SwerveDriveKinematics
          new PIDController(5.0, 0.0, 0.0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(5.0, 0.0, 0.0), // Y controller (usually the same values as X controller)
          new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          m_robotDrive::setModuleStates, // Module states consumer
          true,          // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          m_reflectiveLimelight,
          m_robotDrive // Requires this drive subsystem
        )
    );
      
    


    new Trigger(
      () -> m_driverController.getLeftTriggerAxis() != 0
    ).whileTrue(
      new ConeIntakeWristCommand(m_armSubsystem, m_intakeSubsystem)
    ).onFalse(
      new WristMoveDegreeCommand(m_armSubsystem, ArmConstants.WristRestPosition)
    );

     // new POVButton(m_driverController, 180)
    //   .onTrue(new GoToPositionSwerveCommand(m_robotDrive, m_limelightSubsystem, new Pose2d(new Translation2d(SmartDashboard.getNumber("Auto Distance", 1),0), Rotation2d.fromDegrees(0))).getAutonomousCommand()
    //   );
    
    
    // m_candleSubsystem.setDefaultCommand(new CandleAnimateCommand(m_candleSubsystem));
    
    
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
                //Gradual braking on trigger
                -MathUtil.applyDeadband(m_driverController.getLeftY() * (1.25 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband), 
                -MathUtil.applyDeadband(m_driverController.getLeftX() * (1.25 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * (1.25 - m_driverController.getRightTriggerAxis()), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
   public Command getAutonomousCommand() {
  //   m_robotDrive.zeroHeading();
    // new CandleAnimateCommand(m_candleSubsystem);
    // SwerveControllerCommand command = new AutoScorePreloadMid(m_armSubsystem,m_robotDrive, m_intakeSubsystem, m_limelightSubsystem );
     return new AutoSwerveCommandNoOutake(m_armSubsystem, m_robotDrive, m_intakeSubsystem, m_limelightSubsystem).andThen(() -> m_robotDrive.drive(0, 0, 0, true));
   }
  //  }
  // }

  // }

  public DriveSubsystem getDriveSubsystem(){
   return m_robotDrive;
  }
  public ArmSubsystem getArmSubsystem(){
    return m_armSubsystem;
  }
   public IntakeSubsystem getIntakeSubsystem(){
    return m_intakeSubsystem;
  }
}

// L bozo