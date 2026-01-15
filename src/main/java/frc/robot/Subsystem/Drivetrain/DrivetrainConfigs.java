package frc.robot.Subsystem.Drivetrain;

import frc.robot.Infrastructure.SparkMaxConfigFactory;

public class DrivetrainConfigs {
    public static SparkMaxConfigFactory drivingConfig() {
        SparkMaxConfigFactory drivingConfig = new SparkMaxConfigFactory(44, false);
        drivingConfig.setEncoder(Constants.PositionConversionFactor, Constants.VelocityConversionFactor);
        drivingConfig.setPID(0.04, 0, 0, 1 / Constants.maxSpeed);
        return drivingConfig;
    }

    public static SparkMaxConfigFactory turningConfig(double angleOffset) {
        SparkMaxConfigFactory turningConfig = new SparkMaxConfigFactory(20, false);
        turningConfig.setAbsoluteEncoder(Constants.TurnPositionConversionFactor, Constants.VelocityConversionFactor, angleOffset, false);
        turningConfig.setPID(1.0, 0, 0);
        turningConfig.setPositionWrapping(-Math.PI, Math.PI);
        return turningConfig;
    }
}