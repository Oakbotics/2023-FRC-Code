// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController m_opController = new XboxController(0);

  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final CandleSubsystem m_candleSubsystem = new CandleSubsystem(m_opController, Constants.LightConstants.CANdleID);
  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // m_armSubsystem.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      // new RunCommand(
      //     () -> m_armSubsystem.ArmSpeedCommand(
      //     m_armSubsystem,m_opController.getLeftY()
      //     )));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_opController, XboxController.Button.kY.value).onTrue(new ArmCommandLow(m_armSubsystem, m_opController));
    new JoystickButton(m_opController, XboxController.Button.kX.value).onTrue(new ArmCommandMid(m_armSubsystem, m_opController));

    // new JoystickButton(m_opController, XboxController.Axis.kLeftY).onTrue(new ArmSpeedCommand(m_armSubsystem, m_opController))

    new JoystickButton(m_opController, XboxController.Button.kA.value).onTrue(new PurpleCandleCommand(m_candleSubsystem));
    new JoystickButton(m_opController, XboxController.Button.kB.value).onTrue(new OrangeCandleCommand(m_candleSubsystem));

    new POVButton(m_opController, 0).toggleOnTrue(
      new RunCommand(
      () ->
        m_candleSubsystem.stopLight(), m_candleSubsystem));

    m_candleSubsystem.setDefaultCommand(new AnimateCommand(m_candleSubsystem));
    m_armSubsystem.setDefaultCommand(new ArmSpeedCommand(m_armSubsystem, m_opController));
    
  }
}