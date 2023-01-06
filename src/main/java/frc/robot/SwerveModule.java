package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.MathConstants;



public class SwerveModule {
    
    public final CANSparkMax m_drivingMotor;
    public final CANSparkMax m_steeringMotor;
    public final RelativeEncoder m_driveEncoder;
    public final RelativeEncoder m_steeringEncoder;
    private SparkMaxPIDController m_steeringPIDController;
    private SparkMaxPIDController m_drivingPIDController;
    private double targetSteeringAngle = 0;
    private double targetVelocity = 0;

    //Compilation error because it creates new SparkMaxes
    public SwerveModule(final CANSparkMax drivingMotorAddress, CANSparkMax steeringMotorAddress){ 
        m_drivingMotor = drivingMotorAddress;
        m_steeringMotor = steeringMotorAddress;

        REVPhysicsSim.getInstance().addSparkMax(m_drivingMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_steeringMotor, DCMotor.getNEO(1));

        m_driveEncoder = m_drivingMotor.getEncoder();
        m_steeringEncoder = m_steeringMotor.getEncoder();

        m_steeringPIDController = m_steeringMotor.getPIDController();
        m_steeringPIDController.setP(0);
        m_steeringPIDController.setI(0);
        m_steeringPIDController.setD(0);
        m_steeringPIDController.setOutputRange(-1, 1);

        m_drivingPIDController = m_drivingMotor.getPIDController();
        m_drivingPIDController.setP(0);
        m_drivingPIDController.setI(0);
        m_drivingPIDController.setD(0);
        m_drivingPIDController.setOutputRange(-1, 1);

        


        if(DriverStation.isFMSAttached()){    //If on field, burn PID settings to the motor, in case sparkmax restarts cause of ie. Brownout. 
            m_steeringMotor.burnFlash();      //Apparently motor can only take a limited number of burnFlash calls, so use sparingly 
            m_drivingMotor.burnFlash();
        }

        m_steeringEncoder.setPositionConversionFactor(DriveConstants.MathConstants.steeringPositionCoefficient);
        m_driveEncoder.setPositionConversionFactor(MathConstants.drivingPositionCoefficient);
        m_driveEncoder.setVelocityConversionFactor(MathConstants.drivingRPMCoefficient);

    }


    public SwerveModule(int drivingMotorAddress, int steeringMotorAddress){

        m_drivingMotor = new CANSparkMax(drivingMotorAddress, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steeringMotorAddress, MotorType.kBrushless);

        REVPhysicsSim.getInstance().addSparkMax(m_drivingMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_steeringMotor, DCMotor.getNEO(1));

        m_driveEncoder = m_drivingMotor.getEncoder();
        m_steeringEncoder = m_steeringMotor.getEncoder();



        m_steeringPIDController = m_steeringMotor.getPIDController();
        m_steeringPIDController.setP(0);
        m_steeringPIDController.setI(0);
        m_steeringPIDController.setD(0);
        m_steeringPIDController.setOutputRange(-1, 1);
        if(DriverStation.isFMSAttached()){
            m_steeringMotor.burnFlash();
            m_drivingMotor.burnFlash();
        }

        m_steeringEncoder.setPositionConversionFactor(DriveConstants.MathConstants.steeringPositionCoefficient);
        m_driveEncoder.setPositionConversionFactor(MathConstants.drivingPositionCoefficient);
        m_driveEncoder.setVelocityConversionFactor(MathConstants.drivingRPMCoefficient);
    }


    public void setState(SwerveModuleState targetState){    
        
        targetState = SwerveModuleState.optimize(targetState, Rotation2d.fromDegrees(m_steeringEncoder.getPosition()));        
        m_steeringPIDController.setReference(targetState.angle.getDegrees(), CANSparkMax.ControlType.kPosition);
       
        m_drivingPIDController.setReference(targetState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);

        targetSteeringAngle = targetState.angle.getDegrees();
        targetVelocity = targetState.speedMetersPerSecond;

    }

    public SwerveModulePosition getPos(){
        return new SwerveModulePosition(m_driveEncoder.getPosition(), Rotation2d.fromDegrees(m_steeringEncoder.getPosition() ));
    }

    
    public void printInfo(String ModuleName){
        SmartDashboard.putNumber(ModuleName + "Steering Angle", m_steeringEncoder.getPosition());
        SmartDashboard.putNumber(ModuleName + "Steering Setpoint", targetSteeringAngle);
        SmartDashboard.putNumber(ModuleName + "Drive Position", m_driveEncoder.getPosition());
        SmartDashboard.putNumber(ModuleName + "Driving Velocity", m_driveEncoder.getVelocity());
        SmartDashboard.putNumber(ModuleName + "Target Velocity", targetVelocity);
    }









}
