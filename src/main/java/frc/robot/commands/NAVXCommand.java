package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.NAVXSubsystem;

public class NAVXCommand {
    static public class GetAngle extends InstantCommand {
        public GetAngle(NAVXSubsystem navxSystem) {
            super(() -> System.out.println("Gyro angle is " + navxSystem.getAngle() + "degree"), navxSystem);
        }
        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
    // static public class Print5V extends InstantCommand {
    //     public Print5V(CANdleSystem candleSystem) {
    //         //super(() -> System.out.println("5V is " + candleSystem.get5V() + "V"), candleSystem);
    //     }
    //     @Override
    //     public boolean runsWhenDisabled() {
    //         return true;
    //     }
    // }
    // static public class PrintCurrent extends InstantCommand {
    //     public PrintCurrent(CANdleSystem candleSystem) {
    //         //super(() -> System.out.println("Current is " + candleSystem.getCurrent() + "A"), candleSystem);
    //     }
    //     @Override
    //     public boolean runsWhenDisabled() {
    //         return true;
    //     }
    // }
    // static public class PrintTemperature extends InstantCommand {
    //     public PrintTemperature(CANdleSystem candleSystem) {
    //         //super(() -> System.out.println("Temperature is " + candleSystem.getTemperature() + "C"), candleSystem);
    //     }
    //     @Override
    //     public boolean runsWhenDisabled() {
    //         return true;
    //     }
    // }
}
