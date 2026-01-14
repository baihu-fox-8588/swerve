package frc.robot.Subsystem.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final int MotorID[][] = {{11, 21}, {12, 22}, {13, 23}, {14, 24}};

    public static final double MaxSpeedMetersPerSecond = 4.5;

    public static final double DriveGearRatio = 5.08;
    public static final double WheelCirc = Inches.of(3).times(Math.PI).in(Meters);

    public static final double PositionConversionFactor = 1 / DriveGearRatio * WheelCirc;
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;

    public static final double maxSpeed = 5676 * VelocityConversionFactor;

    public static final double TurnPositionConversionFactor = 2 * Math.PI;
    public static final double TurnVelocityConversionFactor = TurnPositionConversionFactor / 60;

    public static final double[] AngleOffsetRadiants = {
        Math.toRadians(0),
        Math.toRadians(0),
        Math.toRadians(0),
        Math.toRadians(0)
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(0.381, 0.381),
        new Translation2d(0.381, -0.381),
        new Translation2d(-0.381, 0.381),
        new Translation2d(-0.381, -0.381)
    );

    public static final Pose2d InitialPose = new Pose2d(0, 0, Rotation2d.kZero);

    public static final IdleMode MotorMode = IdleMode.kBrake;
}