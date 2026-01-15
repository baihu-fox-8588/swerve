package frc.robot.Subsystem.Drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystem.Gyro.Gyro;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private final SwerveModule[] swerveModules = new SwerveModule[4];
    public SwerveDrivePoseEstimator poseEstimator;

    Gyro gyro = Gyro.getInstance();

    public Drivetrain() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(
                Constants.drivingMotorID[i],
                Constants.turningMotorID[i],
                Constants.AngleOffsetRadiants[i]
            );
        }

        poseEstimator = new SwerveDrivePoseEstimator(
            Constants.kinematics,
            gyro.getRotation(),
            getModulePositions(),
            Constants.InitialPose
        );
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            gyro.getRotation(),
            getModulePositions()
        );
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = swerveModules[i].getPosition();
        return positions;
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            gyro.getRotation(),
            getModulePositions(),
            pose
        );
    }

    public void resetEncoders() {
        for (SwerveModule module : swerveModules) module.resetEncoders();
    }

    public double getHeading() {
        return gyro.getRotation().getDegrees();
    }

    public double getTurnRate() {
        return gyro.getRate();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var swerveModuleStates = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        setModuleStates(swerveModuleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < swerveModules.length; i++) swerveModules[i].setDesiredState(desiredStates[i]);
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) drivetrain = new Drivetrain();
        return drivetrain;
    }
}