/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import jaci.pathfinder.*;
import jaci.pathfinder.modifiers.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import jaci.pathfinder.followers.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Trajectory _currentAutoTrajectory;
  private WPI_TalonSRX _leftMasterDrive;
  private WPI_TalonSRX _rightMasterDrive;
  private DifferentialDrive _drive;
  private AHRS _ahrs;
  private boolean _isNavX;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    _leftMasterDrive = new WPI_TalonSRX(10);
    _rightMasterDrive = new WPI_TalonSRX(11);

    m_chooser.addDefault("Default Auto", kDefaultAuto);
    m_chooser.addObject("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    Waypoint[] points = new Waypoint[] {
      new Waypoint(-4, -1, Pathfinder.d2r(-45)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(-2, -2, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
      new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
      };
      
      Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);
      Trajectory trajectory = Pathfinder.generate(points, config);

      TankModifier modifier = new TankModifier(trajectory).modify(0.5);

      //NavX Initialization
      try {
        _ahrs = new AHRS(SPI.Port.kMXP);
        _isNavX = true;
    } catch (RuntimeException ex) {
         DriverStation.reportError("Error Connecting to navX" + ex.getMessage(), true);
         _isNavX = false;
    }
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, 0),
      new Waypoint(1, 1, Pathfinder.d2r(45))    
    };
    _currentAutoTrajectory = this.setMotionProfile(points);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    this.runMotionProfile(_currentAutoTrajectory);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Left Encoder", _leftMasterDrive.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder", _rightMasterDrive.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Heading",  _ahrs.getAngle());
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public Trajectory setMotionProfile(Waypoint[] waypoints) {
    _rightMasterDrive.setSelectedSensorPosition(0, 0, 0);
    _leftMasterDrive.setSelectedSensorPosition(0, 0, 0);
    
    _ahrs.zeroYaw();        
    
    SmartDashboard.putBoolean("Generated Profile", false);

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 1.7, 2.0, 60.0);
    
    Trajectory autonomousTrajectory = Pathfinder.generate(waypoints, config);
    
    SmartDashboard.putBoolean("Generated Profile", true);

    return autonomousTrajectory;
  }

  public void runMotionProfile(Trajectory trajectory) {
    TankModifier modifier = new TankModifier(trajectory);

    modifier.modify(0.71);

    EncoderFollower leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
    EncoderFollower rightFollower = new EncoderFollower(modifier.getRightTrajectory());

    leftFollower.configureEncoder(_leftMasterDrive.getSelectedSensorPosition(0), 1024, 0.1016);
    rightFollower.configureEncoder(_rightMasterDrive.getSelectedSensorPosition(0), 1024, 0.1016);

    leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0.0);
    rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / 1.7, 0.0);
  //        right.configurePIDVA(0.01, 0.00003, 0.1, 1 / max_velocity, 0);

    double outputLeft = leftFollower.calculate(_leftMasterDrive.getSelectedSensorPosition(0));
    double outputRight = rightFollower.calculate(_rightMasterDrive.getSelectedSensorPosition(0));

    double gyro_heading = _ahrs.getAngle();    // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(leftFollower.getHeading());  // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = 0.8 * (-1.0/80.0) * angleDifference;

    SmartDashboard.putNumber("Right Profile", (outputRight + turn));
    SmartDashboard.putNumber("Left Profile", (outputLeft - turn));
    SmartDashboard.putNumber("Output Left", outputLeft);
    SmartDashboard.putNumber("Output Right", outputRight);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("Desired Heading", leftFollower.getHeading());
    SmartDashboard.putNumber("Right Drive", -(outputRight + turn));
    SmartDashboard.putNumber("Left Drive", -(outputLeft - turn));
    
    _drive.tankDrive(outputLeft - turn, outputRight - turn);
  }
}