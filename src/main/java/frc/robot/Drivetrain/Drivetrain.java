package frc.robot.Drivetrain;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class Drivetrain {
    public SwerveModule swerveModule[] = new SwerveModule[4];
    
    AHRS gyro;

    public SwerveDrivePoseEstimator PoseEstimator;
    public static Drivetrain drivetrain;

    public Drivetrain() {
        for (int i = 0; i < 4; i++) {
            swerveModule[i] = new SwerveModule(
                Constants.driveMotor[i],
                Constants.turningMotor[i],
                i < 2
            );
        }

        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        PoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.kinematics,
                gyro.getRotation2d(),
                getPosition(),
                Constants.InitialPose
            );
    }

    public SwerveModulePosition[] getPosition() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = swerveModule[i].getPosition();
        return positions;
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) drivetrain = new Drivetrain();        
        return drivetrain;
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        // Field-Oriented control
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
        
        // ChassisSpeeds chassisSpeeds = ChassisSpeeds(xSpeed, ySpeed, rot);

        SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics .desaturateWheelSpeeds(states, Constants.maxSpeed);
        
        for(int i = 0; i < 4; i++) swerveModule[i].setDesiredState(states[i]);
    }
}
