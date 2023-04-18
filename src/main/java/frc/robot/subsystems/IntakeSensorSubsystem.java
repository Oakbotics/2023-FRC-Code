// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSensorSubsystem extends SubsystemBase {

  private DigitalInput IntakeSensor = new DigitalInput(1);

  /** Creates a new ExampleSubsystem. */
  public IntakeSensorSubsystem() {
    

  }

  @Override
  public void periodic() {


    SmartDashboard.putBoolean("Intake Sensor", IntakeSensor.get());

  }

  public boolean IntakeSensorState(){
    return IntakeSensor.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}