package frc.robot.components;

import frc.robot.control.DriverXbox;
import frc.robot.control.Xbox;
import frc.robot.sensors.AMSColorSensor;
import frc.robot.util.Colors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PWM.PeriodMultiplier;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * This class represents the robot's drivetrain.
 * It contains all the code for properly controlling
 * and measuring the movements of the robot.
 */
public class Drivetrain extends MecanumDrive implements Component
{
	private DriverXbox xbox = DriverXbox.getInstance();

	private double previousNavXValue = 999.999;
	private boolean abortAutonomous = false;

	private boolean aButton = false;
	private boolean bButton = false;
	private boolean xButton = false;
	private boolean yButton = false;

	private double rightXAxis = 0.0;
	private double rightYAxis = 0.0;
	private double leftXAxis = 0.0;
	private double leftYAxis = 0.0;

	private static WPI_TalonSRX frontLeftMasterMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_MASTER_MOTOR_PORT);
	private static WPI_TalonSRX frontLeftFollowerMotor = new WPI_TalonSRX(Constants.FRONT_LEFT_FOLLOWER_MOTOR_PORT);

	private static WPI_TalonSRX frontRightMasterMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_MASTER_MOTOR_PORT);
	private static WPI_TalonSRX frontRightFollowerMotor = new WPI_TalonSRX(Constants.FRONT_RIGHT_FOLLOWER_MOTOR_PORT);

	private static WPI_TalonSRX rearLeftMasterMotor = new WPI_TalonSRX(Constants.REAR_LEFT_MASTER_MOTOR_PORT);
	private static WPI_TalonSRX rearLeftFollowerMotor = new WPI_TalonSRX(Constants.REAR_LEFT_FOLOWER_MOTOR_PORT);

	private static WPI_TalonSRX rearRightMasterMotor = new WPI_TalonSRX(Constants.REAR_RIGHT_MASTER_MOTOR_PORT);
	private static WPI_TalonSRX rearRightFollowerMotor = new WPI_TalonSRX(Constants.REAR_RIGHT_FOLLOWER_MOTOR_PORT);

	private static Servo servo = new Servo(Constants.SERVO_PORT);
	private double servoPosition = 0.5;

	private Encoder dropDownEncoder = new Encoder(0, 1, false, EncodingType.k4X);
	private AHRS navX = new AHRS(I2C.Port.kMXP);

	private Timer startUpTimer = new Timer();
	private Timer t = new Timer();
	private Timer timer = new Timer();

	private boolean doRestartSpinTimer = true;
	private boolean doResetTimer = true;
	private boolean isTimerDone = false;

	//Color sensor
	private AMSColorSensor colorSensor = new AMSColorSensor(AMSColorSensor.Constants.PORT, AMSColorSensor.Constants.ADDRESS);
	private Colors crgb = new Colors();
	private Colors crgbPrevious = new Colors();
	private Colors crgbUpperThreshold = new Colors();
	private Colors crgbLowerThreshold = new Colors();
	private int upperThresholdFactor;
	private int lowerThresholdFactor;

	private static Drivetrain instance = new Drivetrain();

	/**
	 * Returns the singleton instance of Drivetrain.
	 * @return The singleton instance of Drivetrain.
	 */
	public static Drivetrain getInstance()
	{
		return instance;
	}

    /**
     * Prints the current of all drivetrain Talon SRXs.
     */
	public void debugPrintCurrent()
	{
		System.out.println("Talon 0: " + rearRightMasterMotor.getOutputCurrent());
		System.out.println("Talon 1: " + rearRightFollowerMotor.getOutputCurrent());
		System.out.println("Talon 2: " + rearLeftMasterMotor.getOutputCurrent());
		System.out.println("Talon 3: " + rearLeftFollowerMotor.getOutputCurrent());

		System.out.println("Talon 12: " + frontLeftFollowerMotor.getOutputCurrent());
		System.out.println("Talon 13: " + frontLeftMasterMotor.getOutputCurrent());
		System.out.println("Talon 14: " + frontRightFollowerMotor.getOutputCurrent());
		System.out.println("Talon 15: " + frontRightMasterMotor.getOutputCurrent() + "\n\n");
	}

    /**
     * Private constructor for Drivetrain.
     */
	private Drivetrain()
	{
		super(frontLeftMasterMotor, rearLeftMasterMotor, frontRightMasterMotor, rearRightMasterMotor);
		this.setSafetyEnabled(false);

		startUpTimer.stop();
		startUpTimer.reset();

		//Set follower talons to their respective masters
		frontLeftFollowerMotor.follow(frontLeftMasterMotor);
		frontRightFollowerMotor.follow(frontRightMasterMotor);
		rearLeftFollowerMotor.follow(rearLeftMasterMotor);
		rearRightFollowerMotor.follow(rearRightMasterMotor);

		//Set current limits for each talon
		frontLeftMasterMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		frontLeftMasterMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		frontLeftMasterMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		frontLeftFollowerMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		frontLeftFollowerMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		frontLeftFollowerMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		rearLeftMasterMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		rearLeftMasterMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		rearLeftMasterMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		rearLeftFollowerMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		rearLeftFollowerMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		rearLeftFollowerMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		frontRightMasterMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		frontRightMasterMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		frontRightMasterMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		frontRightFollowerMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		frontRightFollowerMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		frontRightFollowerMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		rearRightMasterMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		rearRightMasterMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		rearRightMasterMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		rearRightFollowerMotor.configContinuousCurrentLimit(Constants.DRIVE_40_AMP_LIMIT, 10);
		rearRightFollowerMotor.configPeakCurrentLimit(Constants.DRIVE_40_AMP_TRIGGER, Constants.DRIVE_40_AMP_TIME);
		rearRightFollowerMotor.configOpenloopRamp(Constants.DRIVE_RAMP_TIME, Constants.DRIVE_RAMP_RATE_TIMEOUT);

		//Enable encoder for front right master talon
		frontRightMasterMotor.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		frontRightMasterMotor.setSensorPhase(true);

		servo.setPeriodMultiplier(PeriodMultiplier.k1X);
		servo.setBounds(2.4, 0, 0, 0, 0.6);		// Current bounds are 8.5 turns, (2.5, 0, 0, 0, 0.5) for 9 turns

		//this.calibrateNavX();

		this.calibrateColorSensor();
	}

    /**
     * Gets the distance the robot has driven converted to inches.
     * @return Distance traveled.
     */
	public double getEncoderDistance()
	{
		return frontRightMasterMotor.getSelectedSensorPosition(0) / 135.0;
	}

    /**
     * Main loop method for teleoperated mode
     */
	public void teleop()
	{
		try
		{
			//System.out.println("Encoder tick: " + frontRightMasterMotor.getSelectedSensorPosition(0) + "  Distance: " + getEncoderDistance());

			//System.out.println(DriverStation.getInstance().isOperatorControl() + " " + DriverStation.getInstance().isEnabled());

            //Make sure robot is actually in teleop mode
			if (DriverStation.getInstance().isOperatorControl() && DriverStation.getInstance().isEnabled())
			{
				if (Math.abs(xbox.getRawAxis(1)) > 0.2)
				{
					leftYAxis = -xbox.getRawAxis(Xbox.Constants.LEFT_STICK_Y_AXIS);
				}
				else leftYAxis = 0;

				if (Math.abs(xbox.getRawAxis(0)) > 0.2)
				{
					leftXAxis = xbox.getRawAxis(Xbox.Constants.LEFT_STICK_X_AXIS) * 0.8;
				}
				else leftXAxis = 0;

				if (Math.abs(xbox.getRawAxis(4)) > 0.2)
				{
					rightXAxis = xbox.getRawAxis(Xbox.Constants.RIGHT_STICK_X_AXIS);
				}
				else rightXAxis = 0;


				if(xbox.getRawButton(Xbox.Constants.RIGHT_BUMPER))
				{
					this.driveCartesian(leftXAxis, leftYAxis, (this.getNavXYaw()) / 50);
				}
				else
				{
					this.driveCartesian(leftXAxis, leftYAxis, rightXAxis);
				}
			}
			//drivetrain.debugPrintCurrent();
		}
		catch(Exception e)
		{
			e.printStackTrace();
		}
	}

	/**
	 * Drive the distance passed into the method.
	 * @return If the robot has completed the drive.
	 */
	public boolean driveDistance(int inches, double maxSpeed, int heading, int stoppingDistance)
	{
		boolean isDoneDriving = false;
		double x = Math.abs(getEncoderDistance());
		double startingSpeed = 0.3;
		double stoppingSpeed = 0.175;
		int startingDistance = 12;
		int direction = 1;
		double rotate = (navX.getYaw() - heading) / 50;

		if(maxSpeed < 0)
		{
			direction = -1;
		}

		if (x <= inches)
		{
			if(x <= startingDistance)
			{
				driveCartesian(0, ((maxSpeed - (startingSpeed * direction)) / startingDistance) * x + (startingSpeed * direction), -rotate);
			}
			else if(x >= startingDistance && x <= inches - stoppingDistance)
			{
				driveCartesian(0, maxSpeed, -rotate);
			}
			else
			{
				driveCartesian(0, stoppingSpeed * direction, -rotate);
			}
		}
		else
		{
			driveCartesian(0, 0, 0);
			isDoneDriving = true;
		}
		//System.out.println("Distance: " + x);
		return isDoneDriving;
	}

	/**
	 * Strafe perpendicular to robot. 0 degrees is North.
	 * @return If the robot has completed the strafe.
	 */
	public boolean strafeSeconds(double time, double strafeSpeed, double heading)
	{
		boolean isTimerDone = false;
		double rotate = (navX.getYaw() - heading) / 50;

		if(doResetTimer)
		{
			t.stop();
			t.reset();
			t.start();
		}

		if(strafeSpeed > 0 && t.get() < time)
		{
			driveCartesian(strafeSpeed, 0, rotate);
		}
		else if(strafeSpeed < 0 && t.get() < time)
		{
			driveCartesian(-strafeSpeed, 0, rotate);
		}
		else
		{
			driveCartesian(0, 0, 0);
			isTimerDone = true;
			doResetTimer = true;
		}

		return isTimerDone;
	}

	/**
	 * Strafe at a specific angle. 0 degrees is North
	 * @return If the robot has completed the strafe.
	 */
	public boolean strafeDistanceAtAngle(int inches, double angle, double speed, int heading)
	{
		boolean isDoneDriving = false;
		double x = Math.abs(getEncoderDistance());
		double rotate = (navX.getYaw() - heading) / 50;
		double strafeSpeed = Math.sin(Math.abs(angle)) * speed;
		double forwardSpeed = Math.cos(Math.abs(angle)) * speed;

		if(angle < 0)
		{
			strafeSpeed *= -1;
		}

		if(speed < 0)
		{
			forwardSpeed *= -1;
		}

		if(x < inches)
		{
			driveCartesian(strafeSpeed, forwardSpeed, rotate);
		}
		else
		{
			driveCartesian(0, 0, 0);
			isDoneDriving = true;
		}

		return isDoneDriving;
	}

    /**
     * Drive the robot forward at the specified speed and at the specified heading.
     * @param speed Speed at which the robot should drive.
     * @param time The maximum time for which the robot can drive.
     * @param heading Heading the robot should drive at
     * @return If the robot has finished the drive.
     */
	public boolean driveSeconds(double speed, double time, int heading)
	{
		isTimerDone = false;
		double rotate = (navX.getYaw() - heading) / 50;

		if(doResetTimer)
		{
			t.stop();
			t.reset();
			t.start();
			doResetTimer = false;
		}

		if(t.get() <= time)
		{
			driveCartesian(0, speed, -rotate);
		}
		else
		{
			driveCartesian(0, 0, 0);
			isTimerDone = true;
			doResetTimer = true;
		}
		return isTimerDone;
	}

    /**
     * Method to return whether the robot should abort autonomous.
     * @return Whether to abort autonomous or not.
     */
	public boolean abortAutonomous()
	{
		return abortAutonomous;
	}

    /**
     * Restart the timer.
     */
	public void restartTimer()
	{
		timer.stop();
		timer.reset();
		timer.start();
	}
	
	/**
	 * Rotate to the bearing passed into the method. 0 degrees is North.
	 * @return If the robot has finished the drive.
	 */
	public boolean spinToBearing(int bearing, double speed)
	{
		boolean isDoneSpinning = false;
		boolean spin = true;
		
		if(navX.isConnected())
		{
			double heading = navX.getYaw();
			
			if(doRestartSpinTimer)
			{
				restartTimer();
				doRestartSpinTimer = false;
			}
			
			if(timer.get() >= 0.2)
			{
				if(previousNavXValue == heading)
				{
					spin = false;
				}
				else
				{
					restartTimer();
					previousNavXValue = heading;
				}
			}
			else
			{
				spin = true;
			}

			if(spin)
			{

				int threshold = 20;
				if(bearing - heading > 0)
				{
					speed *= -1;
				}

				if(Math.abs(bearing - heading) >= threshold)
				{
					driveCartesian(0, 0, -speed);
				}
				else
				{
					driveCartesian(0, 0, 0);
					isDoneSpinning = true;
				}
			}
			else
			{
				abortAutonomous = true;
				System.out.println("\nNAVX REPEATED VALUES.\n");
				driveCartesian(0, 0, 0);
			}
		}
		else
		{
			abortAutonomous = true;
			System.out.println("\nNAVX DISCONNECTED.\n");
			driveCartesian(0, 0, 0);
		}

		return isDoneSpinning;
	}

    /**
     * Drive at the specified speed and heading until the color sensor detects the specified color.
     * @param color Color to look for.
     * @param speed Speed at which to drive.
     * @param heading Heading at which to drive.
     * @return If the robot has finished the drive.
     */
	public boolean driveToColor(AMSColorSensor.Constants.Color color, double speed, int heading)
	{
		boolean isDoneDriving = false;
		boolean foundTape = false;
		colorSensor.get(crgb);

		double rotate = (navX.getYaw() - heading) / 50;

		if(color == AMSColorSensor.Constants.Color.kRed)
		{
			foundTape = crgb.R > crgbUpperThreshold.R;
		System.out.println("RED COLOR FOUND: " + foundTape);
		}
		else if(color == AMSColorSensor.Constants.Color.kBlue)
		{
			foundTape = crgb.B > crgbUpperThreshold.B;
		System.out.println("BLUE COLOR FOUND: " + foundTape);
		}
		else if(color == AMSColorSensor.Constants.Color.kWhite)
		{
			foundTape = crgb.C > crgbUpperThreshold.C;
		System.out.println("WHITE COLOR FOUND: " + foundTape);
		}
		if(!foundTape)
		{

			driveCartesian(0, speed, -rotate);
		}
		else
		{
			driveCartesian(0, 0, 0);
			isDoneDriving = true;
		}
		return isDoneDriving;
	}

    /**
     * Returns the rotation about the Y axis of the NavX gyro.
     * @return The Y axis rotation of the NavX.
     */
	public double getNavXYaw()
	{
		return navX.getYaw();
	}

    /**
     * Zero the NavX
     */
	public void resetNavX()
	{
		navX.reset();
	}

    /**
     * Reset the drop-down wheel encoder.
     */
	public void resetEncoder()
	{
		frontRightMasterMotor.setSelectedSensorPosition(0, 0, 0);
		Timer.delay(0.06);
	}

    /**
     * Print the current colors detected by the color sensor.
     */
	public void printColors()
	{
		System.out.print(colorSensor);
	}

    /**
     * Prints the heading of the robot.
     */
	public void printHeading()
	{
		System.out.print("Heading: " + navX.getYaw());
	}

    /**
     * Prints the value of the drop-down wheel encoder.
     */
	public void printEncoder()
	{
		System.out.println("Encoder: " + dropDownEncoder.getRaw());
	}

    /**
     * Gets the value of the drop-down encoder wheel's servo.
     * @return
     */
	public double getServo()
	{
		return servo.get();
	}

    /**
     * Calibrate the NavX.
     * The NavX is allowed a maximum of five seconds to calibrate
     * before an error message displayed and the calibration is
     * marked as bad.
     */
	public void calibrateNavX()
	{	
		boolean goodCalibration = true;
		System.out.println("Calibrating NavX...");

		startUpTimer.start();
		while(navX.isCalibrating() && startUpTimer.get() < 5.0)
		{
			Timer.delay(0.005);
		}
		if (startUpTimer.get() >= 5.0)
		{
			System.out.println("ERROR! [Drivetrain] Error while calibrating NavX!");
			goodCalibration = false;
		}
		System.out.println("Calibration done... "  + "Did NavX calibrate? " + goodCalibration);
	}

    /**
     * Calibrates the color sensor based on the current color of the floor.
     */
	public void calibrateColorSensor()
	{

		Timer.delay(0.25); // wait for thread to start
		colorSensor.get(crgb); // get initial data to assure there is something for any other robot methods the first time through since robotPeriodic runs after others

		// establish floor color and lighting conditions
		// loop to get average where the robot is parked at the start
		crgbUpperThreshold.C = 0;
		crgbUpperThreshold.R = 0;
		crgbUpperThreshold.G = 0;
		crgbUpperThreshold.B = 0;

		crgbLowerThreshold.C = 0;
		crgbLowerThreshold.R = 0;
		crgbLowerThreshold.G = 0;
		crgbLowerThreshold.B = 0;
		int numSamples = 60;
		for (int i = 1; i <= numSamples; i++)
		{
			colorSensor.get(crgb);
			crgbUpperThreshold.C += crgb.C;
			crgbUpperThreshold.R += crgb.R;
			crgbUpperThreshold.G += crgb.G;
			crgbUpperThreshold.B += crgb.B;

			crgbLowerThreshold.C += crgb.C;
			crgbLowerThreshold.R += crgb.R;
			crgbLowerThreshold.G += crgb.G;
			crgbLowerThreshold.B += crgb.B;
			Timer.delay(0.018);
		}

		// gap between upper and lower thresholds to prevent jitter between the two states
		upperThresholdFactor = 3; //  must be exceeded to determine tape found
		lowerThresholdFactor = 2; // must be below to reset tape found

		crgbUpperThreshold.C = crgbUpperThreshold.C* upperThresholdFactor /numSamples; // compute the average and include the tape threshold factor
		crgbUpperThreshold.R = crgbUpperThreshold.R* upperThresholdFactor /numSamples;
		crgbUpperThreshold.G = crgbUpperThreshold.G* upperThresholdFactor /numSamples;
		crgbUpperThreshold.B = crgbUpperThreshold.B* upperThresholdFactor /numSamples;

		crgbLowerThreshold.C = crgbLowerThreshold.C* lowerThresholdFactor /numSamples; // compute the average and include the tape threshold factor
		crgbLowerThreshold.R = crgbLowerThreshold.R* lowerThresholdFactor /numSamples;
		crgbLowerThreshold.G = crgbLowerThreshold.G* lowerThresholdFactor /numSamples;
		crgbLowerThreshold.B = crgbLowerThreshold.B* lowerThresholdFactor /numSamples;

		crgbUpperThreshold.C = 3000;
		crgbUpperThreshold.R = 1000;
		crgbUpperThreshold.G = 0;
		crgbUpperThreshold.B = 800;

		crgbLowerThreshold.C = 0;
		crgbLowerThreshold.R = 0;
		crgbLowerThreshold.G = 0;
		crgbLowerThreshold.B = 0;

		//System.out.println("robotInit " + mAMSColorSensor);
		//System.out.println("crgbUpperThreshold " + crgbUpperThreshold + "; crgbLowerThreshold " + crgbLowerThreshold);
	}

    /**
     * Raises the drop-down encoder wheel.
     */
	public void omniWheelUp()
	{
		servoPosition = 0.5 + (1.0 / 8.5) * 0.5;
		servo.set(servoPosition);
	}

    /**
     * Lowers the drop-down encoder wheel.
     */
	public void omniWheelDown()
	{
		servoPosition = 0.5;
		servo.set(servoPosition);
	}

    /**
     * Rotates the servo clockwise.
     * @param angle The angle to add to the current position of the servo.
     */
	public void rotateServoClockwise(int angle)
	{
		double rotation = (double)angle / 360.0;
		servoPosition += ((1.0/8.5) * rotation);
		servo.set(servoPosition);
	}

    /**
     * Rotates the servo counterclockwise.
     * @param angle The angle to add to the current position of the servo.
     */
	public void rotateServoCounterClockwise(int angle)
	{
		double rotation = (double)angle / 360.0;
		servoPosition -= ((1.0/8.5) * rotation);
		servo.set(servoPosition);
	}

	@Override
	public void printTestInfo()
	{
		System.out.println("[Drivetrain] Encoder position: " + getEncoderDistance() + " NavX: " + navX.getYaw() + " colors: " + colorSensor.toString());
	}

	/**
	 * Class for constant variables related to Drivetrain.
	 * @author Mark Washington
	 */
	public static class Constants
	{
		public static final int FRONT_LEFT_MASTER_MOTOR_PORT = 13;
		public static final int FRONT_LEFT_FOLLOWER_MOTOR_PORT = 12;

		public static final int FRONT_RIGHT_MASTER_MOTOR_PORT = 15;
		public static final int FRONT_RIGHT_FOLLOWER_MOTOR_PORT = 14;

		public static final int REAR_LEFT_MASTER_MOTOR_PORT = 2;
		public static final int REAR_LEFT_FOLOWER_MOTOR_PORT = 3;

		public static final int REAR_RIGHT_MASTER_MOTOR_PORT = 0;
		public static final int REAR_RIGHT_FOLLOWER_MOTOR_PORT = 1;

		public static final int DRIVE_40_AMP_TRIGGER = 60;
		public static final int DRIVE_40_AMP_LIMIT = 30;
		public static final int DRIVE_40_AMP_TIME = 4000;

		public static final int DRIVE_30_AMP_TRIGGER = 45;
		public static final int DRIVE_30_AMP_LIMIT = 25;
		public static final int DRIVE_30_AMP_TIME = 3000;

		public static final int DRIVE_RAMP_RATE_TIMEOUT = 10; //ms

		public static final double DRIVE_RAMP_TIME = 0.25;

		public static final int SERVO_PORT = 0;
	}




}