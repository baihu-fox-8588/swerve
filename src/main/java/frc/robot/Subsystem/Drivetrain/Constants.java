package frc.robot.Subsystem.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public static final int drivingMotorID[] = {11, 12, 13, 14};
    public static final int turningMotorID[] = {21, 22, 23, 24};

    public static final double MaxSpeedMetersPerSecond = 4.5;

    public static final double DriveGearRatio = 5.08;
    public static final double WheelCirc = Inches.of(3).times(Math.PI).in(Meters);

    public static final double PositionConversionFactor = 1 / DriveGearRatio * WheelCirc;
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;

    public static final double maxSpeed = 5676 * VelocityConversionFactor;

    public static final double TurnGearRatio = 25.0;

    public static final double TurnPositionConversionFactor = 2 * Math.PI;
    public static final double TurnVelocityConversionFactor = TurnPositionConversionFactor / 60;

    public static final double maxAngularSpeed = 11000 / TurnGearRatio * TurnVelocityConversionFactor;

    public static final double[] AngleOffsetRadiants = {
        Math.toRadians(0),
        Math.toRadians(0),
        Math.toRadians(0),
        Math.toRadians(0)
    };

    public static final double TrackWidth = 0.635;
    public static final double WheelBase = 0.635;

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(WheelBase / 2, TrackWidth / 2),
        new Translation2d(WheelBase / 2, -TrackWidth / 2),
        new Translation2d(-WheelBase / 2, TrackWidth / 2),
        new Translation2d(-WheelBase / 2, -TrackWidth / 2)
    );

    public static final Pose2d InitialPose = new Pose2d(0, 0, Rotation2d.kZero);

    public static final IdleMode MotorMode = IdleMode.kBrake;
}