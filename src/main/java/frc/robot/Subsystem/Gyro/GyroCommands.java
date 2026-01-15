package frc.robot.Subsystem.Gyro;

import edu.wpi.first.wpilibj2.command.Command;

public class GyroCommands {
    public static Command resetGyroCommand(Gyro gyro) {
        return gyro.runOnce(
            () -> {
                gyro.reset();
            }
        ).withName("ResetGyroCommand");
    }
}
