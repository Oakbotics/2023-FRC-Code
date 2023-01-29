// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class NAVXSubsystem extends SubsystemBase {
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP);
    private XboxController joystick;

    public NAVXSubsystem(XboxController joy) {
        m_navx.calibrate();
    }
    public double getAngle(){return m_navx.getAngle()%360;}
//    public voice setAngle(){return m_navx.set}
}
