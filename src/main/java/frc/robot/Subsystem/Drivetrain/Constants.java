package frc.robot.Subsystem.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    // 電機 ID
    public static final int driveMotor[] = {11, 12, 13, 14};
    public static final int turningMotor[] = {21, 22, 23, 24};

    // ------------------- Swerve Wheel 物理參數 -------------------
    public static final double GearRatio = 6.86; // 齒輪比 (馬達轉速 / 輪子轉速)
    public static final double WheelCirc = Inches.of(3).times(Math.PI).in(Meters); // 輪子週長 (單位: 米)

    // 位置轉換因子: 馬達轉一圈等於車輪移動多少公尺
    public static final double PositionConversionFactor = 1.0 / GearRatio * WheelCirc; 
    // 速度轉換因子: 馬達 RPM 轉換成車輪 m/s
    public static final double VelocityConversionFactor = PositionConversionFactor / 60;

    // 轉向齒輪比
    public static final double TurnGearRatio = 12.8;
    // 轉向位置轉換因子: 馬達一圈對應的轉向角度 (單位: 弧度)
    public static final double TurnPositionConversionFactor = 1.0 / TurnGearRatio * 2.0 * Math.PI;
    // 轉向速度轉換因子: 馬達 RPM 轉換成 rad/s
    public static final double TurnVelocityConversionFactor = TurnPositionConversionFactor / 60;

    // ------------------- 車體速度限制 -------------------
    public static final double maxSpeed = 3.5;
    public static final double maxAngularSpeed = Math.PI;

    // ------------------- Swerve Kinematics -------------------
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(-0.3, 0.3),
        new Translation2d(-0.3, -0.3),
        new Translation2d(0.3, 0.3),
        new Translation2d(0.3, -0.3)
    );

    // ------------------- 初始定位 -------------------
    public static final Pose2d InitialPose = new Pose2d(0, 0, Rotation2d.kZero);

    // ------------------- 馬達模式 -------------------
    public static final IdleMode MotorMode = IdleMode.kBrake;

    // ------------------- PID 控制器 -------------------
    public static final PIDController DrivePID = new PIDController(0.1, 0.0, 0.0);
    public static final PIDController TurnPID = new PIDController(0.1, 0.0, 0.0);

    static {
        TurnPID.enableContinuousInput(-Math.PI, Math.PI);
    }
}
