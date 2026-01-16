package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class DriveCommands {
    public static Command setXCommand(Drivetrain drivetrain) {
        return drivetrain.run(drivetrain::stopWithXMode).withName("XCommand");
    }

    public static Command driveCommand(Drivetrain drivetrain, Supplier<ChassisSpeeds> chassisSpeeds) {
        return drivetrain.run(
            () -> {
                drivetrain.drive(chassisSpeeds.get());
            }
        ).withName("DriveCommand");
    }
}
