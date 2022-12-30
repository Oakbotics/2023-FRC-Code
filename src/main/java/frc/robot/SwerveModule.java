package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.CAN;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveCANAddresses;
import frc.robot.Constants.DriveConstants.MathConstants;



public class SwerveModule {
    
    private final CANSparkMax m_drivingMotor;
    private final CANSparkMax m_steeringMotor;
    private final RelativeEncoder m_driveEncoder;
    private final RelativeEncoder m_steeringEncoder;


    public SwerveModule(int drivingMotorAddress, int steeringMotorAddress){

        m_drivingMotor = new CANSparkMax(drivingMotorAddress, MotorType.kBrushless);
        m_steeringMotor = new CANSparkMax(steeringMotorAddress, MotorType.kBrushless);

        REVPhysicsSim.getInstance().addSparkMax(m_drivingMotor, DCMotor.getNEO(1));
        REVPhysicsSim.getInstance().addSparkMax(m_steeringMotor, DCMotor.getNEO(1));

        m_driveEncoder = m_drivingMotor.getEncoder();
        m_steeringEncoder = m_steeringMotor.getEncoder();

        m_steeringEncoder.setPositionConversionFactor(DriveConstants.MathConstants.steeringPositionCoefficient);
        m_driveEncoder.setVelocityConversionFactor(MathConstants.drivingRPMCoefficient);




    }

    public void setState(SwerveModuleState targetState){    

        


        m_steeringMotor.set(targetState.angle.getDegrees());
        


    }











}
