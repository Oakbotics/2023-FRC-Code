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
            public static final double robotYlength = 0.5;   //double check what dimension this
            public static final double robotXlength = 0.5;   //double check what dimension this
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
            /*
            https://www.swervedrivespecialties.com/products/mk4-swerve-module 
            Scroll down to gear ratios and motor free speeds
            */

            //What to multiply the encoders position units by. SwervePosition returns degrees. Assuming gear ratio is 4:1. 
            //Also might just use through-bore encoder that comes with the module
            public static final double steeringPositionCoefficient = 360 / 4;  
            
            //What to set as the encoder VelocityConversionFactor. Multiplies the RPM, to give you M/s
            // 1RPM = (circumference of wheel)in/min, /8.14 because of Gear ratio according to L1 module. /60 to convert /min to /sec
            public static final double drivingRPMCoefficient = (Units.inchesToMeters(Math.PI*3) / 8.14) /60; 

            //Multiplies the getPosition value, which returns the rotations the motor has spun.
            //Multiply by the circumference of the wheel in meters, then divides by 8.14 due to gear ratio. Gear ratio is according to L1 Module
            public static final double drivingPositionCoefficient = Units.inchesToMeters(Math.PI*3) / 8.14;

            public static final double moduleMaxSpeed = Units.feetToMeters(12);    //in M/s. Coefficient of Joystick inputs(or of chassis speeds paramaters)
                                                                                         // According to MK4 Datasheet, if using L1 Module
        }
        
 
    }

}
