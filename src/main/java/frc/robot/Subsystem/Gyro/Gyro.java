package frc.robot.Subsystem.Gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
    public static Gyro gyro;

    private final AHRS ahrs;

    public Gyro() {
        ahrs = new AHRS(NavXComType.kMXP_SPI);
    }

    public Rotation2d getRotation() {
        return ahrs.getRotation2d().unaryMinus();
    }

    public double getRate() {
        return ahrs.getRate();
    }
    public void reset() {
        ahrs.reset();
    }

    public static Gyro getInstance() {
        if (gyro == null) gyro = new Gyro();
        return gyro;
    }
}