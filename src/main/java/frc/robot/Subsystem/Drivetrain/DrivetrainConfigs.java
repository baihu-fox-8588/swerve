package frc.robot.Subsystem.Drivetrain;

import frc.robot.Infrastructure.SparkMaxConfiguration;

public class DrivetrainConfigs {
    public static SparkMaxConfiguration drivingConfig() {
        SparkMaxConfiguration drivingConfig = new SparkMaxConfiguration(44, false);
        drivingConfig.setEncoder(Constants.PositionConversionFactor, Constants.VelocityConversionFactor);
        drivingConfig.setPID(0.04, 0, 0, 1 / Constants.maxSpeed);
        return drivingConfig;
    }

    public static SparkMaxConfiguration turningConfig(double angleOffset) {
        SparkMaxConfiguration turningConfig = new SparkMaxConfiguration(20, false);
        turningConfig.setAbsoluteEncoder(Constants.TurnPositionConversionFactor, Constants.VelocityConversionFactor, angleOffset, false);
        turningConfig.setPID(1.0, 0, 0);
        turningConfig.setPositionWrapping(-Math.PI, Math.PI);
        return turningConfig;
    }
}