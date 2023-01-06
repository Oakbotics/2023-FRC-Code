// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


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
import frc.robot.Constants.DriveConstants.DriveCANAddresses;
import frc.robot.Constants.DriveConstants.MathConstants;
import frc.robot.SwerveModule;

public class SwerveDriveTrain extends SubsystemBase {

  // private final CANSparkMax FrontLeftDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontLeftDrivingMotor, MotorType.kBrushless);
  // private final CANSparkMax FrontLeftSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontLeftSteeringMotor, MotorType.kBrushless);
  // private final CANSparkMax FrontRightDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontRightDrivingMotor, MotorType.kBrushless);
  // private final CANSparkMax FrontRightSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.FrontRightSteeringMotor, MotorType.kBrushless);
  // private final CANSparkMax BackLeftDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackLeftDrivingMotor, MotorType.kBrushless);
  // private final CANSparkMax BackLeftSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackLeftSteeringMotor, MotorType.kBrushless);
  // private final CANSparkMax BackRightDrivingMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackRightDrivingMotor, MotorType.kBrushless);
  // private final CANSparkMax BackRightSteeringMotor = new CANSparkMax(DriveConstants.DriveCANAddresses.BackRightSteeringMotor, MotorType.kBrushless);
  

  // private final SwerveModule frontLeftSwerveModule = new SwerveModule(FrontLeftDrivingMotor, FrontLeftSteeringMotor);
  // private final SwerveModule frontRightSwerveModule = new SwerveModule(FrontRightDrivingMotor, FrontRightSteeringMotor);
  // private final SwerveModule backLeftSwerveModule = new SwerveModule(BackLeftDrivingMotor, BackLeftSteeringMotor);
  // private final SwerveModule backRightSwerveModule = new SwerveModule(BackRightDrivingMotor, BackRightSteeringMotor);

  private final SwerveModule frontLeftSwerveModule = new SwerveModule(DriveCANAddresses.FrontLeftDrivingMotor, DriveCANAddresses.FrontLeftSteeringMotor);
  private final SwerveModule frontRightSwerveModule = new SwerveModule(DriveCANAddresses.FrontRightDrivingMotor, DriveCANAddresses.FrontRightSteeringMotor);
  private final SwerveModule backLeftSwerveModule = new SwerveModule(DriveCANAddresses.BackLeftDrivingMotor, DriveCANAddresses.BackLeftSteeringMotor);
  private final SwerveModule backRightSwerveModule = new SwerveModule(DriveCANAddresses.BackRightDrivingMotor, DriveCANAddresses.BackRightSteeringMotor);




  private final AnalogGyro gyro = new AnalogGyro(DriveConstants.DriveCANAddresses.gyroLocation);  // Change to AHRS NavX when library is downloaded
  private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);      


  private final Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.TranslationLocatoins.robotXlength, DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_frontRightLocation = new Translation2d(DriveConstants.TranslationLocatoins.robotXlength, -DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_backLeftLocation = new Translation2d(-DriveConstants.TranslationLocatoins.robotXlength, DriveConstants.TranslationLocatoins.robotYlength);
  private final Translation2d m_backRightLocation = new Translation2d(-DriveConstants.TranslationLocatoins.robotXlength, -DriveConstants.TranslationLocatoins.robotYlength);

  SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);  

                    
  
  SwerveModulePosition[] swervePositions = {frontLeftSwerveModule.getPos(), frontRightSwerveModule.getPos(), backLeftSwerveModule.getPos(), backRightSwerveModule.getPos()};
  SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));   // Creates swerve module state array in default position 
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, Rotation2d.fromDegrees(0), swervePositions);


  
  
  public Field2d field = new Field2d();

  

  /** Creates a new SwerveDriveTrain subsystem. */
  public SwerveDriveTrain() {
    SmartDashboard.putData("Field", field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPositionArray();
    frontLeftSwerveModule.printInfo("Front Left");
    frontRightSwerveModule.printInfo("Front Right");
    backLeftSwerveModule.printInfo("Back Left");
    backRightSwerveModule.printInfo("Back Right");

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    m_odometry.update(Rotation2d.fromDegrees(gyroSim.getAngle()), swervePositions);
    field.setRobotPose(m_odometry.getPoseMeters());
    REVPhysicsSim.getInstance().run();


  }

  public void SwerveDrive(Double forwardSpeed, Double leftSpeed, Double rotationSpeed){
    moduleStates = m_kinematics.toSwerveModuleStates(
      ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, leftSpeed, rotationSpeed, Rotation2d.fromDegrees(-gyroSim.getAngle()))       //Gyro returns CW positive, ChassisSpeed reads CCW positive
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MathConstants.moduleMaxSpeed);

    frontLeftSwerveModule.setState(moduleStates[0]);
    frontRightSwerveModule.setState(moduleStates[1]);
    backLeftSwerveModule.setState(moduleStates[2]);
    backRightSwerveModule.setState(moduleStates[3]);
  }

  public SwerveModulePosition[] getPositionArray(){
    swervePositions[0] = frontLeftSwerveModule.getPos();
    swervePositions[1] = frontRightSwerveModule.getPos();
    swervePositions[2] = backLeftSwerveModule.getPos();
    swervePositions[3] = backRightSwerveModule.getPos();
    
    return swervePositions;
  }


}
