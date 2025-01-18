// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_rightMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_leftBackMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_rightBackMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final XboxController m_driverController = new XboxController(0);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    // SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    var lbc = new SparkMaxConfig();
    // lbc.follow(1);
    m_leftMotor.configure(lbc, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    var rbc = new SparkMaxConfig();
    // rbc.follow(3);
    m_rightMotor.configure(rbc, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftBackMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightBackMotor);
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.
    System.out.println("9jadwuih9huj98fe89f");
    m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());
  }
}
