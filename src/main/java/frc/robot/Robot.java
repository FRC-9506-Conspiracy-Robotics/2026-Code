// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightNames;
import frc.robot.subsystems.PositionData;
import swervelib.SwerveDrive;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final PositionData positionData;
  private final SwerveDrive swerveDrive;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    this.positionData = m_robotContainer.positionData;
    this.swerveDrive = m_robotContainer.swerveDrive;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    double correction = 0;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      correction = 180;
    }

    LimelightHelpers.SetRobotOrientation(LimelightNames.limelight4AFront, 
    this.swerveDrive.getGyroRotation3d().getZ() * 180/Math.PI + correction, 
    0, 
    0, 
    0, 
    0, 
    0);
    LimelightHelpers.SetRobotOrientation(LimelightNames.limelight3ALeft, 
    this.swerveDrive.getGyroRotation3d().getZ() * 180/Math.PI + correction, 
    0, 
    0, 
    0, 
    0, 
    0);
    this.positionData.updatePose();
    
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

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
