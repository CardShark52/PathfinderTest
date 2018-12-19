/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import jaci.pathfinder.Trajectory;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import jaci.pathfinder.*;
import jaci.pathfinder.modifiers.*;
import jaci.pathfinder.followers.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final SendableChooser<Waypoint[]> autoChooser = new SendableChooser<>();
  private Trajectory _currentAutoTrajectory;
  private WPI_TalonSRX _leftMasterDrive = new WPI_TalonSRX(Constants.leftMotorChannel);
  private WPI_TalonSRX _rightMasterDrive = new WPI_TalonSRX(Constants.rightMotorChannel);
  private DifferentialDrive _drive = new DifferentialDrive(_rightMasterDrive, _leftMasterDrive);
  EncoderFollower rightFollower;
  EncoderFollower leftFollower;
  private Joystick joystick = new Joystick(0);
  public static final ADIS16448_IMU gyro = new ADIS16448_IMU();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    autoChooser.addDefault("Default Auto", Waypoints.defaultAuto);
    autoChooser.addObject("Auto 2", Waypoints.longerAuto);
    autoChooser.addObject("Auto 3", Waypoints.auto3);
    autoChooser.addObject("newPath", Waypoints.newPath);
    autoChooser.addObject("CircleAuto", Waypoints.circle);
    SmartDashboard.putData(autoChooser);

    gyro.reset();
    _leftMasterDrive.setSelectedSensorPosition(0, 0, 0);
    _rightMasterDrive.setSelectedSensorPosition(0, 0, 0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Left Encoder", _leftMasterDrive.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Right Encoder", _rightMasterDrive.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("Heading X", gyro.getAngleX());
    SmartDashboard.putNumber("Heading Y", gyro.getAngleY());
    SmartDashboard.putNumber("Heading Z", gyro.getAngleZ());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    gyro.reset();    
    _currentAutoTrajectory = this.setMotionProfile(autoChooser.getSelected());
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
    
		/* get gamepad stick values */
		double forwMod = -1;
		double turnMod = 1;
		if (joystick.getRawButton(12)) {
			 turnMod = 0.70;
		}
		double forw = +forwMod * joystick.getRawAxis(1); /* positive is forward */
		double turn = +turnMod * joystick.getRawAxis(2); /* positive is right */

		SmartDashboard.putNumber("Left Drive Volts", _leftMasterDrive.getMotorOutputVoltage());
		SmartDashboard.putNumber("Right Drive Volts", _rightMasterDrive.getMotorOutputVoltage());
		SmartDashboard.putNumber("Left Drive Amps", _leftMasterDrive.getOutputCurrent());
		SmartDashboard.putNumber("Rigt Drive Amps", _rightMasterDrive.getOutputCurrent());

		/* drive robot */
		_drive.arcadeDrive(forw, turn, false);
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

    SmartDashboard.putBoolean("Generated Profile", false);

    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST,
        Constants.frequency, Constants.maxSpeed, Constants.acceleration, Constants.jerk);

    Trajectory autonomousTrajectory = Pathfinder.generate(waypoints, config);

    SmartDashboard.putBoolean("Generated Profile", true);

    leftFollower = new EncoderFollower();
    rightFollower = new EncoderFollower();

    leftFollower.configureEncoder(_leftMasterDrive.getSelectedSensorPosition(0), Constants.encoderTicksPerRotation,
        Constants.wheelDiameter);
    rightFollower.configureEncoder(_rightMasterDrive.getSelectedSensorPosition(0), Constants.encoderTicksPerRotation,
        Constants.wheelDiameter);

    leftFollower.configurePIDVA(Constants.kP, Constants.kI, Constants.kD, Constants.kV, Constants.kA);
    rightFollower.configurePIDVA(Constants.kP, Constants.kI, Constants.kD, Constants.kV, Constants.kA);

    TankModifier modifier = new TankModifier(autonomousTrajectory);
    modifier.modify(Constants.wheelBase);
    leftFollower.setTrajectory(modifier.getLeftTrajectory());
    rightFollower.setTrajectory(modifier.getRightTrajectory());
    return autonomousTrajectory;
  }

  public void runMotionProfile(Trajectory trajectory) {

    double outputLeft = leftFollower.calculate(_leftMasterDrive.getSelectedSensorPosition(0));
    double outputRight = rightFollower.calculate(-_rightMasterDrive.getSelectedSensorPosition(0));

    double gyro_heading = gyro.getAngleZ(); // Assuming the gyro is giving a value in degrees
    double desired_heading = Pathfinder.r2d(leftFollower.getHeading()); // Should also be in degrees

    double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
    double turn = 0.8 * (-1.0 / 80.0) * angleDifference;

    SmartDashboard.putNumber("Right Profile", (outputRight + turn));
    SmartDashboard.putNumber("Left Profile", (outputLeft - turn));
    SmartDashboard.putNumber("Output Left", outputLeft);
    SmartDashboard.putNumber("Output Right", outputRight);
    SmartDashboard.putNumber("Turn", turn);
    SmartDashboard.putNumber("Desired Heading Radians", leftFollower.getHeading());
    SmartDashboard.putNumber("Desired Heading Degrees", Pathfinder.r2d(leftFollower.getHeading()));
    SmartDashboard.putNumber("Right Drive", outputRight + turn);
    SmartDashboard.putNumber("Left Drive", outputLeft - turn);
    SmartDashboard.putNumber("Angle Difference", angleDifference);

    _drive.tankDrive(outputLeft - turn, outputRight + turn);
  }
}