package frc.robot.subsystems.Drivetrain;

import frc.robot.constants.drivetrainConstants;
import frc.robot.utils.SparkMaxConfiguration;

public class DrivetrainConfigs {
    public static SparkMaxConfiguration drivingConfig() {
        SparkMaxConfiguration drivingConfig = new SparkMaxConfiguration(44, false);
        drivingConfig.setEncoder(drivetrainConstants.PositionConversionFactor, drivetrainConstants.VelocityConversionFactor);
        drivingConfig.setPID(0.04, 0, 0, 1 / drivetrainConstants.maxSpeed);
        return drivingConfig;
    }

    public static SparkMaxConfiguration turningConfig(double angleOffset) {
        SparkMaxConfiguration turningConfig = new SparkMaxConfiguration(20, false);
        turningConfig.setAbsoluteEncoder(drivetrainConstants.TurnPositionConversionFactor, drivetrainConstants.VelocityConversionFactor, angleOffset, false);
        turningConfig.setPID(1.0, 0, 0);
        turningConfig.setPositionWrapping(-Math.PI, Math.PI);
        return turningConfig;
    }
}