// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.CandleSubsystem;
import frc.robot.commands.CandleCommands;
import frc.robot.commands.OrangeCandleCommand;
import frc.robot.commands.PurpleCandleCommand;
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
  private final XboxController joy = new XboxController(Constants.JoystickId);
  
  private final CandleSubsystem m_candleSubsystem = new CandleSubsystem(joy, Constants.LightConstants.CANdleID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // new JoystickButton(joy, Constants.CandleButton).onTrue(new CandleCommands.ConfigBrightness(m_candleSubsystem, 1.0));
    
    new JoystickButton(joy, XboxController.Button.kA.value).onTrue(new PurpleCandleCommand(m_candleSubsystem));
    new JoystickButton(joy, XboxController.Button.kB.value).onTrue(new OrangeCandleCommand(m_candleSubsystem));


    
  }
}