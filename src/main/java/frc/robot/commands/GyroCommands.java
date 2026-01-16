package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gyro.Gyro;

public class GyroCommands {
    public static Command resetGyroCommand(Gyro gyro) {
        return gyro.runOnce(gyro::reset).withName("ResetGyroCommand");
    }
}
