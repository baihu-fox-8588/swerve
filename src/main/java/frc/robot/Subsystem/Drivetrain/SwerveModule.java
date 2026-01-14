package frc.robot.Subsystem.Drivetrain;

import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final AbsoluteEncoder absoluteEncoder;

    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveInverted) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        absoluteEncoder = turnMotor.getAbsoluteEncoder();

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.idleMode(Constants.MotorMode)
            .inverted(driveInverted)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig.idleMode(Constants.MotorMode)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(20);

        driveConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        turnConfig.encoder
            .positionConversionFactor(Constants.TurnPositionConversionFactor)
            .velocityConversionFactor(Constants.TurnVelocityConversionFactor);

        turnConfig.absoluteEncoder
            .positionConversionFactor(2 * Math.PI)
            .velocityConversionFactor(2 * Math.PI)
            .zeroOffset(Constants.AngleOffsetRad)
            .inverted(false);

        driveMotor.configure(
            driveConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        turnMotor.configure(
            turnConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
        );

        turnEncoder.setPosition(absoluteEncoder.getPosition());
    }

    public List<Double> getPosition() {
        return List.of(driveEncoder.getPosition(), turnEncoder.getPosition());
    }

    public List<Double> getVelocity() {
        return List.of(driveEncoder.getVelocity(), turnEncoder.getVelocity());
    }

    public List<SparkMax> getMotor() {
        return List.of(driveMotor, turnMotor);
    }
}