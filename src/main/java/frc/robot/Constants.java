// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static final class TranslationLocatoins{
            public static final double robotYlength = 0.5;
            public static final double robotXlength = 0.5;   
        }

        public static final class DriveCANAddresses{
            public static final int FrontLeftSteeringMotor = 11;
            public static final int FrontLeftDrivingMotor = 1;

            public static final int FrontRightSteeringMotor = 12;
            public static final int FrontRightDrivingMotor = 2;

            public static final int BackLeftSteeringMotor = 13;
            public static final int BackLeftDrivingMotor = 3;

            public static final int BackRightSteeringMotor = 14;
            public static final int BackRightDrivingMotor = 4;

            public static final int gyroLocation = 1;

        }

        public static final class MathConstants{

            //What to multiply the encoderes position units by. SwerveState returns it in degrees, 4rpm = one rotation of wheel. 
            public static final double steeringPositionCoefficient = 360 / 4;  
            
            //What to set as the encoder VelocityConversionFactor. Multiplies the RPM, to give you the wheel RPM
            // 1RPM = (circumference of wheel)in/min, /5 because one rotation of motor is one fifth of wheel movement. /60 to convert /min to /sec
            public static final double drivingRPMCoefficient = (Units.inchesToMeters(Math.PI*3) / 5) /60; 

            //Multiplies the getPosition value, which returns the rotations the motor has spun.
            //Multiply by the circumference of the wheel in meters, then divides by 5 due to gear ratio(still need to find actual gear ratio)
            public static final double drivingPositionCoefficient = Units.inchesToMeters(Math.PI*3) / 5;

            public static final double moduleMaxSpeed = 3;    //in M/s. Coefficient of Joystick inputs(or of chassis speeds paramaters)

        }

        public static final class SteeringPIDConstants{
            public static final double P = 0;
            public static final double I = 0;
            public static final double D = 0;
            public static final double F = 0;

        }
        
 
    }

}
