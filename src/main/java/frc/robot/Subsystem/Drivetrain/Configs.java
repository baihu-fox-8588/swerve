package frc.robot.Subsystem.Drivetrain;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Configs {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
        drivingConfig.idleMode(Constants.MotorMode)
            .voltageCompensation(12)
            .smartCurrentLimit(44);

        turningConfig.idleMode(Constants.MotorMode)
            .voltageCompensation(12)
            .smartCurrentLimit(20);

        drivingConfig.encoder
            .positionConversionFactor(Constants.PositionConversionFactor)
            .velocityConversionFactor(Constants.VelocityConversionFactor);

        turningConfig.absoluteEncoder
            .positionConversionFactor(Constants.TurnPositionConversionFactor)
            .velocityConversionFactor(Constants.TurnVelocityConversionFactor)
            .inverted(true);

        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)
            .velocityFF(1 / Constants.maxSpeed)
            .outputRange(-1, 1);

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1.0, 0, 0)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);

    }
}
