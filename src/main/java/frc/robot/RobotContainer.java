package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystem.Drivetrain.Constants;
import frc.robot.Subsystem.Drivetrain.DriveCommands;
import frc.robot.Subsystem.Drivetrain.Drivetrain;

public class RobotContainer {
	private final Drivetrain drivetrain = Drivetrain.getInstance();
	private final Joystick joystick = new Joystick(0);

	private boolean isFieldRelative = true;

	public RobotContainer() {
		drivetrain.setDefaultCommand(
				DriveCommands.driveCommand(
						drivetrain,
				() -> getChassisSpeeds()
			)
		);
	}

	private ChassisSpeeds getChassisSpeeds() {
		double xSpeed = -joystick.getY();
		double ySpeed = -joystick.getX();
		double rot = -joystick.getRawAxis(4);

		if (isFieldRelative) {
			return ChassisSpeeds.fromFieldRelativeSpeeds(
					MathUtil.applyDeadband(xSpeed, 0.1) * Constants.maxSpeed,
					MathUtil.applyDeadband(ySpeed, 0.1) * Constants.maxSpeed,
					MathUtil.applyDeadband(rot, 0.1) * Constants.maxAngularSpeed,
					drivetrain.getPose().getRotation());
		} else {
			return new ChassisSpeeds(
					MathUtil.applyDeadband(xSpeed, 0.1) * Constants.maxSpeed,
					MathUtil.applyDeadband(ySpeed, 0.1) * Constants.maxSpeed,
					MathUtil.applyDeadband(rot, 0.1) * Constants.maxAngularSpeed);
		}
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured!");
	}
}