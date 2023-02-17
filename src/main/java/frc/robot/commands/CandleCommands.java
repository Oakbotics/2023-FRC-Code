package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CandleSubsystem;

public class CandleCommands {
    static public class ConfigBrightness extends InstantCommand {
        public ConfigBrightness(CandleSubsystem candleSystem, double brightnessPercent) {
            super(() -> candleSystem.configBrightness(brightnessPercent), candleSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
