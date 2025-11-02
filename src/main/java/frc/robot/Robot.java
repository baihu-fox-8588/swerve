// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.Drivetrain;

public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  private Drivetrain drivetrain;

  private Joystick joystick;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    joystick = new Joystick(0);

    drivetrain = Drivetrain.getInstance();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  private Timer initAngleCorrection;
  private boolean hasAngleCorrected;

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }

    initAngleCorrection = new Timer();
    initAngleCorrection.reset();
    initAngleCorrection.start();

    hasAngleCorrected = false;
  }
  
  @Override
  public void autonomousPeriodic() {
    if (!hasAngleCorrected) {
      if (initAngleCorrection.hasElapsed(0.5)) {
        drivetrain.drive(0.0, 0.0, 0.0);
        drivetrain.initAngleCorrection();
        hasAngleCorrected = true;
      }
      else drivetrain.drive(0.0, 0.2, 0.0);
    }
    else {

    }
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    drivetrain.drive(joystick.getX(), joystick.getY(), joystick.getZ());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
