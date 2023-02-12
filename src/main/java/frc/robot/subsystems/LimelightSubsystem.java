// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;


  /** Creates a new ExampleSubsystem. */
  public LimelightSubsystem() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void getNetworkTableValues(){
    // NetworkTableEntry tx = m_limelightTable.getEntry("tx");
    // NetworkTableEntry ty = m_limelightTable.getEntry("ty");
    // NetworkTableEntry ta = m_limelightTable.getEntry("ta");  
  

    // //read values periodically
    // double x = tx.getDouble(0.0);
    // double y = ty.getDouble(0.0);
    // double area = ta.getDouble(0.0);

    // //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putString("Alex", "rocks");


  }

  @Override
  public void periodic() {
    double[] botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    
    double[] id = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDoubleArray(new double[6]);
    // double[] ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // double[] tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // long ty = m_limelightTable.getEntry("ty").getLastChange();
    // long ta = m_limelightTable.getEntry("ta").getLastChange();
    
    SmartDashboard.putNumberArray("LimelightX", botpose);
    SmartDashboard.putNumberArray("LimelightID", id);  
    // SmartDashboard.putNumber("LimelightY", ty);
    // SmartDashboard.putNumber("LimelightArea", ta);
    // // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
