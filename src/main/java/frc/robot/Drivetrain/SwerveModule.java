package frc.robot.Drivetrain;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SparkFlex driveMotor;      // 驅動輪馬達
    private final SparkMax turnMotor;        // 轉向馬達

    private final RelativeEncoder driveEncoder;  // 驅動馬達編碼器
    private final RelativeEncoder turnEncoder;   // 轉向馬達編碼器

    public SwerveModule(int driveID, int turnID, boolean driveInverted) {
        // 初始化馬達
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        // 取得編碼器
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // ---------------- 驅動馬達設定 ----------------
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.idleMode(Constants.MotorMode)           // 空轉模式: Brake/Coast
            .inverted(driveInverted)                        // 是否反向
            .voltageCompensation(12)                        // 電壓補償
            .smartCurrentLimit(44);                         // 電流限制

        // 設定編碼器轉換因子 (從馬達轉速/位置轉到車輪實際速度/距離)
        driveConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        driveMotor.configure(
            driveConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // ---------------- 轉向馬達設定 ----------------
        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(20);

        // 編碼器轉換: 馬達圈數 -> 轉向弧度
        turnConfig.encoder
            .positionConversionFactor(Constants.TurnPositionConversionFactor)
            .velocityConversionFactor(Constants.TurnVelocityConversionFactor);

        turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    // 取得 Swerve 模組位置
    // driveEncoder.getPosition(): 車輪轉過的距離 (米)
    // turnEncoder.getPosition(): 車輪轉向角度 (rad)
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(), 
            new Rotation2d(turnEncoder.getPosition())
        );
    }

    // 設定模組目標狀態
    public void setDesiredState(SwerveModuleState state) {
        // 最佳化轉向，避免反轉角度
        state.optimize(new Rotation2d(turnEncoder.getPosition()));

        driveMotor.set(state.speedMetersPerSecond / Constants.maxSpeed);
        turnMotor.set(Math.max(-1.0, Math.min(Constants.TurnPID.calculate(turnEncoder.getPosition(), state.angle.getRadians()), 1.0)));
    }
}
