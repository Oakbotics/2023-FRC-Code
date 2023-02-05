// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;


public class CandleSubsystem extends SubsystemBase {
    private final CANdle m_candle; //= new CANdle(Constants.CANdleID, "rio");
    private final int LedCount = 100;
    private XboxController joystick;

    public CandleSubsystem(XboxController joy, int CANdleID) {
        this.joystick = joy;
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.On;
        m_candle = new CANdle(Constants.LightConstants.CANdleID, "rio");
        m_candle.configAllSettings(configAll, 100);
        //m_candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.7, 120, Direction.Forward, 8));
        stopLight();
    }

    public void setPurple(){
        m_candle.setLEDs(148, 0, 211, 0, 0, 8);
    }

    public void setOrange(){
        m_candle.setLEDs(255, 125, 0, 0, 0, 8);
    } 

    public void setFireAnimation(){
        m_candle.animate(new FireAnimation(1, 0.4, 60, 0.7, 0.2, false, 8));
    }

    public void setRainbowAnimation(){
        m_candle.animate(new RainbowAnimation(1, 1, LedCount));
    }

    public void stopLight(){
        m_candle.animate(null);
        m_candle.setLEDs(0, 0, 0);
    }

    public void configBrightness(double percent) {
         m_candle.configBrightnessScalar(percent, 0); 
    }
}