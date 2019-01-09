package frc.robot;

import frc.robot.components.Drivetrain;
import frc.robot.components.Elevator;
import frc.robot.components.Gripper;

import edu.wpi.first.wpilibj.IterativeRobot;

/**
 * Main robot class
 * Controls main loops for autonomous and teleoperated mode.
 */
public class Robot extends IterativeRobot
{
	private Autonomous autonomous = Autonomous.getInstance();

	private Drivetrain drivetrain = Drivetrain.getInstance();
	private Gripper gripper = Gripper.getInstance();
	private Elevator elevator = Elevator.getInstance();

	private double previousNanoTime;
	private double currentNanoTime;

    /**
     * Constructor for robot class
     */
	public Robot()
	{
		previousNanoTime = System.nanoTime();
		currentNanoTime = System.nanoTime();
	}

    /**
     * Method to initialize components on robot
     */
	@Override
	public void robotInit()
	{
		drivetrain.resetNavX();
		drivetrain.calibrateNavX();
		drivetrain.calibrateColorSensor();

		System.out.println("Starting robot!");
	}

    /**
     * Method that runs once when robot is first disabled
     */
	@Override
	public void disabledInit()
	{
		System.out.println("Robot is disabled");
	}

    /**
     * Method that runs periodically when robot is disabled
     */
	@Override
	public void disabledPeriodic()
	{
		//printSensorValues();
	}

    /**
     * Method that runs once when robot first enters teleoperated mode
     */
	@Override
	public void teleopInit()
	{
		System.out.println("Entering teleop");
		gripper.setTeleopLimits();
		autonomous.turnLightRingsOff();
		drivetrain.omniWheelUp();
	}

    /**
     * Method that runs periodically when robot is in teleoperated mode.
     * This method calls all the methods required for driving the robot,
     * as well as printing sensor values.
     */
	@Override
	public void teleopPeriodic()
	{
		drivetrain.teleop();
		elevator.teleop();
		gripper.teleop();
		printSensorValues();
	}

    /**
     * Method that runs once when robot first enters autonomous mode.
     */
	@Override
	public void autonomousInit()
	{
		System.out.println("Entering autonomous");
		gripper.setAutoLimits();
		autonomous.init();
	}

    /**
     * Method that runs periodically when robot is in autonomous mode.
     * This method calls all the methods required for controlling the robot in autonomous mode.
     */
	@Override
	public void autonomousPeriodic()
	{
		printSensorValues();
		autonomous.periodic();
	}

    /**
     * Method to print all commonly used sensor values.
	 * * Drivetrain encoder value
	 * * NavX yaw value
	 * * Alpha value from color sensor
	 * * Elevator potentiometer value
	 * * Pivot potentiometer value
	 * * eft intake encoder value
	 * * Right intake encoder value
     */
	public void printSensorValues()
	{
		System.out.print("Encoder: " + Drivetrain.getInstance().getEncoderDistance() +
				"\tNavX: " + Drivetrain.getInstance().getNavXYaw() + 
				"\tColors: "); Drivetrain.getInstance().printColors();
				System.out.print("\tElevator pot: " + Elevator.getInstance().getPosition() +
						"\tPivot pot: " + Gripper.getInstance().getPivotPotentiometer() +
						"\tIntake encoder left: " + Gripper.getInstance().getLeftIntakeEncoder() + 
						"\tIntake encoder right: " + Gripper.getInstance().getRightIntakeEncoder() + '\n');
	}
}
