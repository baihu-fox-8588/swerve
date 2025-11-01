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
    private final SparkFlex driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    public SwerveModule(int driveID, int turnID, boolean driveInverted) {
        driveMotor = new SparkFlex(driveID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        SparkFlexConfig driveConfig = new SparkFlexConfig();

        driveConfig.idleMode(Constants.MotorMode)
            .inverted(driveInverted)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        driveConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        driveMotor.configure(
            driveConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        SparkMaxConfig turnConfig = new SparkMaxConfig();

        turnConfig.idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(20);

        turnConfig.encoder
            .positionConversionFactor(Constants.TurnPositionConversionFactor)
            .velocityConversionFactor(Constants.TurnVelocityConversionFactor);

        turnMotor.configure(
            turnConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(new Rotation2d(turnEncoder.getPosition()));

        driveMotor.set(state.speedMetersPerSecond);
        turnMotor.set(Math.max(-1.0, Math.min(Constants.TurnPID.calculate(turnEncoder.getPosition(), state.angle.getRadians()), 1.0)));
    }
}
