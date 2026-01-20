package frc.robot.subsystems.Drivetrain.module;

import frc.robot.constants.DrivetrainConstants;
import frc.robot.utils.SparkMaxConfiguration;

public class SwerveConfigs {
    public static SparkMaxConfiguration drivingConfig() {
        SparkMaxConfiguration drivingConfig = new SparkMaxConfiguration(44, false);
        drivingConfig.setEncoder(DrivetrainConstants.PositionConversionFactor, DrivetrainConstants.VelocityConversionFactor);
        drivingConfig.setPID(0.04, 0, 0, 1 / DrivetrainConstants.maxSpeed);
        return drivingConfig;
    }

    public static SparkMaxConfiguration turningConfig(double angleOffset) {
        SparkMaxConfiguration turningConfig = new SparkMaxConfiguration(20, false);
        turningConfig.setAbsoluteEncoder(DrivetrainConstants.TurnPositionConversionFactor, DrivetrainConstants.VelocityConversionFactor, angleOffset, false);
        turningConfig.setPID(1.0, 0, 0);
        turningConfig.setPositionWrapping(-Math.PI, Math.PI);
        return turningConfig;
    }
}