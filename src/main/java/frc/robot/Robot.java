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
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.external.LIDARLite;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private final SparkMax m_leftMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_rightMotor = new SparkMax(3, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_leftBackMotor = new SparkMax(4, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_rightBackMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushed);
  private final SparkMax m_coralLoader = new SparkMax(6, SparkLowLevel.MotorType.kBrushed);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
  private final XboxController m_driverController = new XboxController(0);
  private final LIDARLite lidar = new LIDARLite(Port.kOnboard);

  private boolean coralLoaderRunning = false;

  private int disableAccumulator = 0;

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  private SlewRateLimiter leftSRL = new SlewRateLimiter(0.5);
  private SlewRateLimiter rightSRL = new SlewRateLimiter(0.5);

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    // SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    var lbc = new SparkMaxConfig();
    //lbc.follow(1);
    m_leftMotor.configure(lbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    var rbc = new SparkMaxConfig();
    //rbc.follow(3);
    rbc.inverted(true);
    m_rightMotor.configure(rbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftBackMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightBackMotor);

    // lidar.startMeasuring();
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true);
  }

  @Override
  public void disabledInit() {
    lidar.stopMeasuring();
  }

  @Override
  public void teleopInit() {
    lidar.startMeasuring();
  }

  @Override
  public void teleopPeriodic() {
    // Drive with tank drive.
    // That means that the Y axis of the left stick moves the left side
    // of the robot forward and backward, and the Y axis of the right stick
    // moves the right side of the robot forward and backward.

    // Calculates the next value of the output
    double filteredLeft = leftSRL.calculate(m_driverController.getLeftY() * 0.8);
    double filteredRight = rightSRL.calculate(m_driverController.getRightY() * 0.8);

    m_robotDrive.tankDrive(filteredLeft, filteredRight);

    if (m_driverController.getRightBumperButton()) {
      DriverStationDataJNI.setEStop(true);
    }
    if (m_driverController.getLeftBumperButton()) disableAccumulator++;
    else disableAccumulator = 0;

    if (disableAccumulator > (int) ((1.0 / (20.0/1000.0)) * 3.0)) DriverStationDataJNI.setEStop(true); // if the disable accumulator has been held for more than 3 seconds, disable the emergency stop

    // int dist = lidar.getDistance();

    if (m_driverController.getYButtonPressed()) {
      coralLoaderRunning = !coralLoaderRunning;
    }

    if (m_driverController.getBButtonPressed()) coralLoaderRunning = true;
    if (m_driverController.getBButtonReleased()) coralLoaderRunning = false;

    /*if (coralLoaderRunning && dist < 65) {
      coralLoaderRunning = false;
    }*/

    if (coralLoaderRunning) {
      m_coralLoader.set(.5);

    } else {
      m_coralLoader.set(0.0);
    }

  }
}
