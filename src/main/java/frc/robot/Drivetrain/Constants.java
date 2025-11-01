package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final int driveMotor[] = {11, 12, 13, 14};
    public static final int turningMotor[] = {21, 22, 23, 24};

    // 待修正
    public static final double GearRatio = 6.86;
    public static final double WheelCirc = Inches.of(3).times(Math.PI).in(Meters);

    public static final double PositionConversionFactor = 1.0 / GearRatio * WheelCirc;
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;

    public static final double TurnGearRatio = 12.8;
    public static final double TurnPositionConversionFactor = 1.0 / TurnGearRatio * 2.0 * Math.PI;
    public static final double TurnVelocityConversionFactor = TurnPositionConversionFactor / 60;

    public static final double maxSpeed = 3.5;

    // 待修正
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3),
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3)
    );

    public static final Pose2d InitialPose = new Pose2d(0, 0, Rotation2d.kZero);

    public static final IdleMode MotorMode = IdleMode.kBrake;

    public static final PIDController DrivePID = new PIDController(0.1, 0.0, 0.0);
    public static final PIDController TurnPID = new PIDController(0.1, 0.0, 0.0);

    static {
        TurnPID.enableContinuousInput(-Math.PI, Math.PI);
    }
}
