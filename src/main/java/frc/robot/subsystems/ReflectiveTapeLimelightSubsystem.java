// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ReflectiveTapeConstants;



public class ReflectiveTapeLimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private final Field2d m_field = new Field2d();

  private Pose2d robotPose;
  private double id;
  private double x;
  private double y;
  private double v;
  private double a;
  private double strafeDistance;
  private double forwardDistance;

  
  private String networkTable;
  

  /** Creates a new ExampleSubsystem. */
  public ReflectiveTapeLimelightSubsystem() {

    SmartDashboard.putData(m_field);

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void getNetworkTableValues(){
    


  }

  @Override
  public void periodic() {
    NetworkTableEntry tx = m_limelightTable.getEntry("tx");
    NetworkTableEntry ty = m_limelightTable.getEntry("ty");
    NetworkTableEntry ta = m_limelightTable.getEntry("ta"); 
    NetworkTableEntry tv = m_limelightTable.getEntry("tv"); 

  

    //read values periodically
    x = tx.getDouble(4);
    y = ty.getDouble(4);
    a = ta.getDouble(4);
    v = tv.getDouble(4);

  forwardDistance = (
      Math.abs((ReflectiveTapeConstants.limelightHeightmetres - ReflectiveTapeConstants.poleTapeTopHeightmetres))
      /Math.tan(Units.degreesToRadians(y))  
  );

  strafeDistance = (
        
        forwardDistance * Math.tan(Units.degreesToRadians(x))
    )- ReflectiveTapeConstants.limelightHorizontalOffset;


    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", a);
    SmartDashboard.putNumber("Target Available", v);
    SmartDashboard.putNumber("Forward Distance", forwardDistance);





    // robotPose = new Pose2d(new Translation2d(Math.abs(botPose[0]), Math.abs(botPose[1])), Rotation2d.fromDegrees(Math.abs(botPose[5])));
    

    // double[] ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // double[] tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // long ty = m_limelightTable.getEntry("ty").getLastChange();
    // long ta = m_limelightTable.getEntry("ta").getLastChange();

  
    SmartDashboard.putNumber("Distance from pole, metres", forwardDistance);
    SmartDashboard.putNumber("Strafe Distance", strafeDistance);

  }




  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public double getDistanceFromReflectiveTape(){
    
    if (v == 1){
      
    
    

     strafeDistance = (
        
        forwardDistance * Math.tan(Units.degreesToRadians(x))
    )- ReflectiveTapeConstants.limelightHorizontalOffset;

  }
  else{
    forwardDistance = 0;
    strafeDistance = 0;
  }
  
  SmartDashboard.putNumber("Distance from pole, metres", forwardDistance);
  SmartDashboard.putNumber("Strafe Distance", strafeDistance);
  Pose2d conePose = new Pose2d(new Translation2d(forwardDistance, strafeDistance), new Rotation2d());

    
    SmartDashboard.putString("Limelight Destination", conePose.toString());
    return strafeDistance;
  }

}