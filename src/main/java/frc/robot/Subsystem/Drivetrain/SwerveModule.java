package frc.robot.Subsystem.Drivetrain;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {
    private final SparkMax drivingMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingPIDController;
    private final SparkClosedLoopController turningPIDController;

    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingMotorID, int turningMotorID, double angleOffset) {
        drivingMotor = new SparkMax(drivingMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        drivingPIDController = drivingMotor.getClosedLoopController();
        turningPIDController = turningMotor.getClosedLoopController();

        drivingMotor.configure(
            DrivetrainConfigs.drivingConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        turningMotor.configure(
            DrivetrainConfigs.turningConfig(angleOffset),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            drivingEncoder.getVelocity(),
            new Rotation2d(turningEncoder.getPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingEncoder.getPosition(),
            new Rotation2d(turningEncoder.getPosition())
        );
    }

    public void resetEncoders() {
        drivingEncoder.setPosition(0);
    }

    public void stop() {
        drivingMotor.stopMotor();
        turningMotor.stopMotor();
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);

        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingPIDController.setReference(
            correctedDesiredState.speedMetersPerSecond,
            ControlType.kVelocity
        );

        turningPIDController.setReference(
            correctedDesiredState.angle.getRadians(),
            ControlType.kPosition
        );

        this.desiredState = desiredState;
    }
}