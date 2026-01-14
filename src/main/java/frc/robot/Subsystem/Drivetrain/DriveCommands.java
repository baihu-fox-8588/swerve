package frc.robot.Subsystem.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveCommands {
    public static Command zeroHeadingCommand(Drivetrain drivetrain) {
        return drivetrain.runOnce(
            () -> {
                drivetrain.gyro.reset();
            }
        ).withName("ZeroHeadingCommand");
    }

    public static Command setXCommand(Drivetrain drivetrain) {
        return drivetrain.run(
            () -> {
                var swerveModuleStates = new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(45))
                };

                drivetrain.setModuleStates(swerveModuleStates);
            }
        ).withName("XCommand");
    }

    public static Command driveCommand(Drivetrain drivetrain, Supplier<ChassisSpeeds> chassisSpeeds) {
        return drivetrain.run(
            () -> {
                drivetrain.drive(chassisSpeeds.get());
            }
        ).withName("DriveCommand");
    }
}
