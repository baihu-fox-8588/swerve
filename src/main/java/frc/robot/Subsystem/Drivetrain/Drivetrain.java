package frc.robot.Subsystem.Drivetrain;

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

        // 初始化定位估算器
        // 1. kinematics: SwerveDriveKinematics 定義每個模組相對位置
        // 2. gyro.getRotation2d(): 初始朝向
        // 3. getPosition(): 每個模組的位置 (角度 + 編碼器位移)
        // 4. Constants.InitialPose: 初始位姿 (位置 + 朝向)
        PoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.kinematics,
                gyro.getRotation2d().unaryMinus(),
                getPosition(),
                Constants.InitialPose
            );
    }

    // 取得四個模組的位置
    public SwerveModulePosition[] getPosition() {
        SwerveModulePosition positions[] = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) 
            positions[i] = swerveModule[i].getPosition();
        return positions;
    }

    // 取得四個模組的速度與角度狀態    
    public SwerveModuleState[] getStates() {
        SwerveModuleState states[] = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) 
            states[i] = swerveModule[i].getState();
        return states;
    }

    public static Drivetrain getInstance() {
        if (drivetrain == null) drivetrain = new Drivetrain();        
        return drivetrain;
    }

    // 初始化角度修正    
    public void initAngleCorrection() {
        // 計算當前車身速度
        ChassisSpeeds chassisSpeeds = Constants.kinematics.toChassisSpeeds(getStates());
        for (SwerveModule module : swerveModule) module.setPosition(0, module.getState().angle.getRadians() + Math.atan2(chassisSpeeds.vxMetersPerSecond * 0.5, chassisSpeeds.vyMetersPerSecond * 0.5));
    }

    /**
     * 控制車體運動
     * @param xSpeed 前後方向速度 [-1, 1]，field-oriented
     * @param ySpeed 左右方向速度 [-1, 1]，field-oriented
     * @param rot 旋轉速度 [-1, 1]
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        // Field-Oriented Control
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * Constants.maxSpeed, 
            ySpeed * Constants.maxSpeed, 
            rot * Constants.maxAngularSpeed, 
            gyro.getRotation2d().unaryMinus() // 使用陀螺儀轉換為場地座標
        );
        
        // Robot-Relative 控制方式 (若需要改成相對座標，可啟用)
        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());

        // 將車身速度轉換為每個模組的目標速度與角度
        SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(chassisSpeeds);
        
        // 標準化車輪速度
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

        for(int i = 0; i < 4; i++) 
            swerveModule[i].setDesiredState(states[i]);
    }
}
