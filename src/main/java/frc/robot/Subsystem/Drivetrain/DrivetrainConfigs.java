package frc.robot.Subsystem.Drivetrain;

import frc.robot.Infrastructure.SparkMaxConfigFactory;

public class DrivetrainConfigs {
    public static final SparkMaxConfigFactory drivingConfig;
    public static final SparkMaxConfigFactory turningConfig;

    static {
        drivingConfig = new SparkMaxConfigFactory(44, false);
        turningConfig = new SparkMaxConfigFactory(20, false);

        drivingConfig.setEncoder(Constants.PositionConversionFactor, Constants.VelocityConversionFactor);
        turningConfig.setAbsoluteEncoder(Constants.TurnPositionConversionFactor, Constants.VelocityConversionFactor, false);

        drivingConfig.setPID(0.04, 0, 0, 1 / Constants.maxSpeed);
        turningConfig.setPID(1.0, 0, 0);

        turningConfig.setpositionWrapping(-Math.PI, Math.PI);
    }

}
