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

    // 使用 SwerveDrivePoseEstimator 做定位融合
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

        // 初始化陀螺儀
        gyro = new AHRS(NavXComType.kMXP_SPI);
        gyro.reset();

        // 初始化定位估算器
        // 使用 kinematics 定義車輪相對位置
        // gyro.getRotation2d() 作為初始方向
        // getPosition() 取得每個模組的位置 (車輪轉角與轉速)
        // Constants.InitialPose 定義機器人初始位置
        PoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.kinematics,
                gyro.getRotation2d(),
                getPosition(),
                Constants.InitialPose
            );
    }

    // 取得四個模組的位置 (SwerveModulePosition 包含角度和位移)
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
        // Field-Oriented Control
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * Constants.maxSpeed, 
            ySpeed * Constants.maxSpeed, 
            rot * Constants.maxAngularSpeed, 
            gyro.getRotation2d()
        );
        
        // Robot-Relative
        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());

        // 將 ChassisSpeeds 轉成每個模組的目標速度與角度
        SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);
        
        // 將車輪速度標準化，避免超過最大速度
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

        for(int i = 0; i < 4; i++) swerveModule[i].setDesiredState(states[i]);
    }
}
