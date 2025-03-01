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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SpeedController;

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
  private double coralDirection = 1.0; // 1.0 or -1.0
  private boolean run = true;

  private int enableAccumulator = 0;
  private final int enableThreshold = (int) ((1.0 / (20.0/1000.0)) * 3.0);

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  private SlewRateLimiter leftSRL = new SlewRateLimiter(2.0);
  private SlewRateLimiter rightSRL = new SlewRateLimiter(2.0);

  private SpeedController speedController = new SpeedController();

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    // SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    var lbc = new SparkMaxConfig();
    //lbc.follow(1);
    m_leftMotor.configure(lbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    var rbc = new SparkMaxConfig();
    // rbc.follow(3);
    rbc.inverted(true);
    m_rightMotor.configure(rbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    lbc = new SparkMaxConfig();
    lbc.follow(m_leftMotor);
    m_leftBackMotor.configure(lbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rbc = new SparkMaxConfig();
    rbc.follow(m_rightMotor);
    m_rightBackMotor.configure(rbc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
    SendableRegistry.addChild(m_robotDrive, m_leftBackMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightBackMotor);

    SmartDashboard.putData(speedController);

    // lidar.startMeasuring();
    
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // m_rightMotor.setInverted(true);
  }

  @Override
  public void robotPeriodic() {
    speedController.update();
    SmartDashboard.putBoolean("Coral loader enabled", coralLoaderRunning);
    SmartDashboard.putNumber("Coral loader direction", coralDirection);
    SmartDashboard.putString("Robot state", run ? "Enabled" : String.format("Disabled, enabling %d / %d", enableAccumulator, enableThreshold));

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

    double speed = speedController.getSpeed.getAsDouble();

    double filteredLeft = leftSRL.calculate(m_driverController.getLeftY() * speed);
    double filteredRight = rightSRL.calculate(m_driverController.getRightY() * speed);

    if (m_driverController.getRightBumperButton()) {
      run = false;
      System.out.println("emergency stop!");
    }
    if (m_driverController.getLeftBumperButton()) enableAccumulator++;
    else enableAccumulator = 0;

    if (enableAccumulator > enableThreshold) run = true; // if the enable accumulator has been held for more than 3 seconds, enable the emergency stop

    if (run) {
      // Drive with tank drive.
      // That means that the Y axis of the left stick moves the left side
      // of the robot forward and backward, and the Y axis of the right stick
      // moves the right side of the robot forward and backward.
      int pov = m_driverController.getPOV();
      if (pov== 0) m_robotDrive.tankDrive(-speed, -speed);
      else if (pov == 90) m_robotDrive.tankDrive(-speed, speed);
      else if (pov == 270) m_robotDrive.tankDrive(speed, -speed);
      else if (pov == 180) m_robotDrive.tankDrive(speed, speed);
      else m_robotDrive.tankDrive(filteredLeft, filteredRight);

      // m_leftBackMotor.set(0.2);
      if (m_driverController.getLeftTriggerAxis() > 0.2) speedController.setSpeed.accept(speed - 0.005);
      if (m_driverController.getRightTriggerAxis() > 0.2) speedController.setSpeed.accept(speed + 0.005);
      speed = speedController.getSpeed.getAsDouble();
      speed = Math.max(0.01, Math.min(0.5, speed));

      /*if (speed != oldSpeed) {
        System.out.println(String.format("Set speed to %.3f / 0.5", speed));
      }*/

      // int dist = lidar.getDistance();

      if (m_driverController.getYButtonPressed()) {
        coralLoaderRunning = !coralLoaderRunning;
      }

      if (m_driverController.getBButtonPressed()) coralLoaderRunning = true;
      if (m_driverController.getBButtonReleased()) coralLoaderRunning = false;

      if (m_driverController.getXButtonPressed()) coralDirection *= -1.0;

      /*if (coralLoaderRunning && dist < 65) {
        coralLoaderRunning = false;
      }*/

      if (coralLoaderRunning) {
        m_coralLoader.set(.5 * coralDirection);

      } else {
        m_coralLoader.set(0.0);
      }
    }
    else {
      m_robotDrive.tankDrive(0.0, 0.0); // to keep the "differential drive not updated enough" warning quiet
      m_rightMotor.set(0);
      m_leftMotor.set(0);
      m_leftBackMotor.set(0);
      m_rightBackMotor.set(0.0);
      m_coralLoader.set(0.0);
    }

  }
}
