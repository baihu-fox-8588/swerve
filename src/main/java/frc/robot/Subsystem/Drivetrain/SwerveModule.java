package frc.robot.Subsystem.Drivetrain;

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
    private final SparkFlex driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    /**
     * 初始化 Swerve 模組
     * @param driveID       驅動馬達CAN ID
     * @param turnID        轉向馬達CAN ID
     * @param driveInverted 驅動馬達是否反向
     */
    public SwerveModule(int driveID, int turnID, boolean driveInverted) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        // ---------------- 驅動馬達設定 ----------------
        SparkFlexConfig driveConfig = new SparkFlexConfig();
        driveConfig.idleMode(Constants.MotorMode)           // 空轉模式: Brake/Coast
            .inverted(driveInverted)                        // 是否反向
            .voltageCompensation(12)         // 電壓補償，避免電壓下降影響速度
            .smartCurrentLimit(44);              // 電流限制，保護馬達

        // 編碼器轉換: 馬達轉速/位置 -> 車輪實際速度/距離
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

    /**
     * 取得模組目前的位置
     * @return SwerveModulePosition 包含車輪位移與轉向角
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveEncoder.getPosition(),                // 車輪轉過的距離 (米)
            new Rotation2d(turnEncoder.getPosition())  // 轉向角度 (rad)
        );
    }

    /**
     * 將模組編碼器歸零或設定特定位置
     * @param drivePosition 驅動馬達位置
     * @param turnPosition 轉向馬達位置
     */
    public void setPosition(double drivePosition, double turnPosition) {
        driveEncoder.setPosition(drivePosition);
        turnEncoder.setPosition(turnPosition);
    }

    /**
     * 取得模組目前狀態
     * @return SwerveModuleState 包含車輪速度與角度
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            driveEncoder.getVelocity(),                 // 車輪速度 (米/秒)
            new Rotation2d(turnEncoder.getPosition())   // 轉向角 (rad)
        );
    }

    /**
     * 設定模組目標狀態 (速度 + 角度)
     * @param state 目標 SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        // 最佳化轉向角度，避免輪子旋轉超過180°導致反轉
        state.optimize(new Rotation2d(turnEncoder.getPosition()));
        
        turnMotor.set(Math.max(-1.0, Math.min(Constants.TurnPID.calculate(turnEncoder.getPosition(), state.angle.getRadians()), 1.0)));
        driveMotor.set(state.speedMetersPerSecond / Constants.maxSpeed);
    }
}
