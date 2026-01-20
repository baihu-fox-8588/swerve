package frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.drivetrainConstants;
import frc.robot.subsystems.Drivetrain.module.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private final SwerveModule[] swerveModules = new SwerveModule[4];
    private final SwerveDrivePoseEstimator poseEstimator;

    private RobotConfig robotConfig;

    Gyro gyro = Gyro.getInstance();

    public Drivetrain() {
        try {
            robotConfig = RobotConfig.fromGUISettings();
        }
        catch (Exception e) {
            DriverStation.reportError("PathPlanner Config Load Failed: " + e.getMessage(), true);
        }
        
        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(
                drivetrainConstants.drivingMotorID[i],
                drivetrainConstants.turningMotorID[i],
                drivetrainConstants.AngleOffsetRadiants[i]
            );
        }

        poseEstimator = new SwerveDrivePoseEstimator(
            drivetrainConstants.kinematics,
            gyro.getRotation(),
            getModulePositions(),
            drivetrainConstants.InitialPose
        );

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRelativeSpeeds,
            this::drive,
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0, 0),
                new PIDConstants(5.0, 0, 0)
            ),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
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

    private ChassisSpeeds getRelativeSpeeds() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = swerveModules[i].getState();
        return drivetrainConstants.kinematics.toChassisSpeeds(states);
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

    public void stopWithXMode() {
        var swerveModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(0, Rotation2d.fromRadians(45)),
            new SwerveModuleState(0, Rotation2d.fromRadians(-45)),
            new SwerveModuleState(0, Rotation2d.fromRadians(-45)),
            new SwerveModuleState(0, Rotation2d.fromRadians(45))
        };

        setModuleStates(swerveModuleStates);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        var swerveModuleStates = drivetrainConstants.kinematics.toSwerveModuleStates(discretizedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, drivetrainConstants.maxSpeed);

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