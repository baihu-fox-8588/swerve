package frc.robot.Infrastructure;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class SparkMaxConfiguration extends SparkMaxConfig {
    public SparkMaxConfiguration(int currentLimit, boolean isInverted) {
        this.idleMode(Constants.motorMode)
            .voltageCompensation(Constants.voltageCompensation)
            .smartCurrentLimit(currentLimit)
            .inverted(isInverted);
    }

    public void setEncoder(double positionConversionFactor, double velocityConversionFactor) {
        this.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
    }

    public void setAbsoluteEncoder(double positionConversionFactor, double velocityConversionFactor, double angleOffset, boolean inverted) {
        this.absoluteEncoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor)
            .zeroOffset(angleOffset)
            .inverted(inverted);

        this.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    }

    public void setPID(double p, double i, double d) {
        setPID(p, i, d, 0);
    }

    public void setPID(double p, double i, double d, double ff) {
        this.closedLoop
            .pid(p, i, d)
            .velocityFF(ff)
            .outputRange(-1, 1);
    }

    public void setPositionWrapping(double inputMin, double inputMax) {
        this.closedLoop
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(inputMin, inputMax);
    }
}
