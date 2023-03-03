// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommandGroup;

import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;

/** An example command that uses an example subsystem. */
public class IntakeCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;
  private final ArmSubsystem m_armSubsystem;
  private double wristStartingDegrees;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public IntakeCone(IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem){
    m_intakeSubsystem = intakeSubsystem;
    m_armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem, m_armSubsystem);
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    wristStartingDegrees = m_armSubsystem.getWristPosition();
    if(m_armSubsystem.getShoulderPosition() <= 10){
    } // Might need to make this a command group for this to work
    new WristMoveDegreeCommand(m_armSubsystem, 50);  


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeSubsystem.runIntake(0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("End", "Mid Ended");
    m_intakeSubsystem.stopIntake(0);
    new WristMoveDegreeCommand(m_armSubsystem, wristStartingDegrees);
  }

//   Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
