// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.atomic.DoubleAccumulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.REVPHSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveTrain extends SubsystemBase {

  private final CANSparkMax frontLeftDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontLeftDrivingMotor, MotorType.kBrushless);
  private final CANSparkMax frontLeftSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontLeftSteeringMotor, MotorType.kBrushless);

  private final CANSparkMax frontRightDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontRightDrivingMotor, MotorType.kBrushless);
  private final CANSparkMax frontRightSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontRightSteeringMotor, MotorType.kBrushless);
  
  private final CANSparkMax BackLeftDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackLeftDrivingMotor, MotorType.kBrushless);
  private final CANSparkMax BackLeftSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackLeftSteeringMotor, MotorType.kBrushless);

  private final CANSparkMax BackRightDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackRightDrivingMotor, MotorType.kBrushless);
  private final CANSparkMax BackRightSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackRightSteeringMotor, MotorType.kBrushless);
  

  private final AnalogGyro gyro = new AnalogGyro(DriveConstants.DriveCANAddresses.gyroLocation);  // Change to AHRS NavX when library is downloaded
  private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);      


  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.TranslationLocatoins.robotXlength, DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.TranslationLocatoins.robotXlength, -DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.TranslationLocatoins.robotXlength, DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.TranslationLocatoins.robotXlength, -DriveConstants.TranslationLocatoins.robotYlength);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);  

                    
  
  SwerveModulePosition[] positions = {new SwerveModulePosition(), new SwerveModulePosition()};
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));   // Creates swerve module state array in default position 
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0), positions);


  SwerveModuleState frontLeftState = moduleStates[0];
  SwerveModuleState frontRightState = moduleStates[1];
  SwerveModuleState backLeftState = moduleStates[2];
  SwerveModuleState backRightState = moduleStates[3];
  
  public Field2d field = new Field2d();

  

  /** Creates a new SwerveDriveTrain subsystem. */
  public SwerveDriveTrain() {
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    frontLeftState = moduleStates[0];
    frontRightState = moduleStates[1];
    backLeftState = moduleStates[2];
    backRightState = moduleStates[3];



  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //m_odometry.update(Rotation2d.fromDegrees(gyroSim.getAngle()), moduleStates);
    field.setRobotPose(m_odometry.getPoseMeters());
    REVPhysicsSim.getInstance().run();


  }

  public void SwerveDrive(Double forwardSpeed, Double leftSpeed, Double rotation){
    moduleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, leftSpeed, rotation, Rotation2d.fromDegrees(-gyroSim.getAngle()))       //Gyro returns CW positive, ChassisSpeed reads CCW positive
      );
    
  }
}
