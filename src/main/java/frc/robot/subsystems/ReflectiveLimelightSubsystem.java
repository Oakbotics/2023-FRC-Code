// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class ReflectiveLimelightSubsystem extends SubsystemBase {

  private NetworkTable m_limelightTable;

  private final Field2d m_field = new Field2d();

  private Pose2d robotPose;
  private double id;
  
  private String networkTable;
  

  /** Creates a new ExampleSubsystem. */
  public ReflectiveLimelightSubsystem() {

    SmartDashboard.putData(m_field);

    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight-reflect");
  }

  public void getNetworkTableValues(){
    
    NetworkTableEntry tx = m_limelightTable.getEntry("tx");
    NetworkTableEntry ty = m_limelightTable.getEntry("ty");
    NetworkTableEntry ta = m_limelightTable.getEntry("ta"); 
    NetworkTableEntry tv = m_limelightTable.getEntry("tv"); 

  

    //read values periodically
    double x = tx.getDouble(4);
    double y = ty.getDouble(4);
    double area = ta.getDouble(4);
    double targetAvailable = tv.getDouble(4);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Target Available", targetAvailable);



  }

  @Override
  public void periodic() {

    networkTable = DriverStation.getAlliance() == Alliance.Blue? "botpose_wpiblue" : "botpose_wpired";
    double[] botPose = m_limelightTable.getEntry(networkTable).getDoubleArray(new double[6]); 
    id = m_limelightTable.getEntry("tid").getDouble(-1);

    // robotPose = new Pose2d(new Translation2d(Math.abs(botPose[0]), Math.abs(botPose[1])), Rotation2d.fromDegrees(Math.abs(botPose[5])));
    robotPose = new Pose2d(new Translation2d((botPose[0]),(botPose[1])), Rotation2d.fromDegrees((botPose[5])));

    m_field.setRobotPose(robotPose);
    SmartDashboard.putString("Field Pose", m_field.getRobotPose().getTranslation().toString());
    

    // double[] ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // double[] tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").getDoubleArray(new double[6]);
    // long ty = m_limelightTable.getEntry("ty").getLastChange();
    // long ta = m_limelightTable.getEntry("ta").getLastChange();
    
    
    SmartDashboard.putNumber("Bot Rotation", botPose[5]);
    SmartDashboard.putNumber  ("LimelightID", id);  
    SmartDashboard.putNumber("BotPoseX", robotPose.getX());
    SmartDashboard.putNumber("BotPoseY", robotPose.getY());
    // SmartDashboard.putNumber("LimelightY", ty);
    // SmartDashboard.putNumber("LimelightArea", ta);
    // // This method will be called once per scheduler run
  }


    public Pose2d getRobotPose(){
        return robotPose; 
    }
    
    public double getRobotAngle(){
        if(id != -1){
            return robotPose.getRotation().getDegrees();
        }

        return -1;
        
    }
  




  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}