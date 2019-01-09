package frc.robot.components;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.HashMap;

import frc.robot.control.OperatorXbox;
import frc.robot.control.Xbox;

import edu.wpi.first.wpilibj.Timer;

/**
 * Class to control the intake and grip mechanism on the robot's elevator.
 * @author Julien Thrum, Ben Puzycki, Darryl Wong, Mark Washington
 */
public class Gripper implements Component
{
	private OperatorXbox xbox = OperatorXbox.getInstance();

	private WPI_TalonSRX leftIntakeTalon = new WPI_TalonSRX(Constants.LEFT_INTAKE_MOTOR_PORT);
	private WPI_TalonSRX rightIntakeTalon = new WPI_TalonSRX(Constants.RIGHT_INTAKE_MOTOR_PORT);
	private WPI_TalonSRX pivotTalon = new WPI_TalonSRX(Constants.PIVOTER_MOTOR_PORT);

	private Timer pulsateTimer = new Timer();

	private HashMap<Integer, WPI_TalonSRX> talonSRXHashMap = new HashMap<Integer, WPI_TalonSRX>();

	private boolean isAutoEjecting = false;
	private boolean isAutoIntaking = false;
	private boolean isPivoting = false;

	private int currentValue = 0;

	private Constants.Range currentRange = Constants.Range.raisedRange;
	private int[] targetRange = Constants.Range.raisedRange.range;
	private Constants.Direction currentDirection = Constants.Direction.None;

	private int currentTestKeyPosition = 0;

	private boolean leftBumper;
	private boolean rightBumper;
	private boolean aButton;

	//Quarantine
	boolean isIntakeDone = true;
	boolean isPivoterRaisedDone = true;
	boolean isPivoterMiddleDone = true;
	boolean isPivoterFloorDone = true;

	boolean isAutoPivotRaisedDone = false;
	boolean isAutoPivotMiddleDone = false;
	boolean isAutoPivotFloorDone = false;
	//End quarantine

	private boolean isFloorTarget = false; 	//Use with a button auto floor 

	private boolean isPulsateTimerReset = false;

	private static Gripper instance = new Gripper();
	public static Gripper getInstance()
	{
		return instance;
	}

	/**
	 * Constructor for the Gripper class.
	 */
	private Gripper()
	{
		//Left & right intake talon settings
		leftIntakeTalon.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		leftIntakeTalon.setSensorPhase(false);
		leftIntakeTalon.configForwardSoftLimitEnable(false, 0);
		leftIntakeTalon.configReverseSoftLimitEnable(false, 0);
		leftIntakeTalon.setInverted(false);
		leftIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_40_AMP_LIMIT, 10);
		leftIntakeTalon.configPeakCurrentLimit(Constants.INTAKE_40_AMP_TRIGGER, Constants.INTAKE_40_AMP_TIME);
		leftIntakeTalon.configOpenloopRamp(Constants.INTAKE_RAMP_TIME, Constants.INTAKE_RAMP_RATE_TIMEOUT);

		rightIntakeTalon.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.QuadEncoder, 0, 0);
		rightIntakeTalon.setSensorPhase(true);
		rightIntakeTalon.configForwardSoftLimitEnable(false, 0);
		rightIntakeTalon.configReverseSoftLimitEnable(false, 0);
		rightIntakeTalon.setInverted(false);
		rightIntakeTalon.configContinuousCurrentLimit(Constants.INTAKE_40_AMP_LIMIT, 10);
		rightIntakeTalon.configPeakCurrentLimit(Constants.INTAKE_40_AMP_TRIGGER, Constants.INTAKE_40_AMP_TIME);
		rightIntakeTalon.configOpenloopRamp(Constants.INTAKE_RAMP_TIME, Constants.INTAKE_RAMP_RATE_TIMEOUT);

		//Pivoter settings
		pivotTalon.setNeutralMode(NeutralMode.Brake);
		pivotTalon.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.Analog, 0, 0);
		pivotTalon.setSensorPhase(false);
		pivotTalon.configForwardSoftLimitThreshold(Constants.RAISED, 0);
		pivotTalon.configReverseSoftLimitThreshold(Constants.FLOOR, 0);
		pivotTalon.configForwardSoftLimitEnable(true, 0);	//FIXME: change back to true with working sensor
		pivotTalon.configReverseSoftLimitEnable(true, 0);	//FIXME: change back to true with working sensor

		talonSRXHashMap.put(Constants.LEFT_INTAKE_MOTOR_PORT, leftIntakeTalon);
		talonSRXHashMap.put(Constants.RIGHT_INTAKE_MOTOR_PORT, rightIntakeTalon);
		talonSRXHashMap.put(Constants.PIVOTER_MOTOR_PORT, pivotTalon);
	}


	/**
	 * Intake function for autonomous.
	 * @return State of intake, whether it's done or not.
	 */
	public boolean autoIntake()
	{
		if(!isAutoIntaking())
		{
			rightIntakeTalon.setSelectedSensorPosition(0,0,0);		// set encoder position
			setAutoIntaking(true);
			intake();
			//leftIntakeTalon.setSelectedSensorPosition(0,0,0);

			//			leftIntakeTalon.set(ControlMode.Velocity, 1);
			//			rightIntakeTalon.set(ControlMode.Velocity, 1);		// set motor
		}
		if(rightIntakeTalon.getSelectedSensorPosition(0) >= Constants.AUTO_INTAKE_ENCODER_STOP_VALUE)
		{
			intakeOff();
			setAutoIntaking(false);
		}
		return isAutoIntaking();

	}
	public double autoEncoder()
	{
		return rightIntakeTalon.getSelectedSensorPosition(0);
	}


	/** 
	 * Eject function for autonomous.
	 * @return State of eject, whether it's done or not.
	 */
	public boolean autoEject()
	{
		boolean done = false;
		if (rightIntakeTalon.getSelectedSensorPosition(0) >= Constants.AUTO_EJECT_ENCODER_STOP_VALUE)
		{
			ejectShoot();
		}
		else
		{
			intakeOff();
			setAutoEjecting(false);
			done = true;
		}
		
		return done;
	}

	/**
	 * Method for gently dropping a cube in autonomous.
	 * @return State of drop, whether it's done or not.
	 */
	public boolean autoDrop()
	{
		boolean done = false;
		if (rightIntakeTalon.getSelectedSensorPosition(0) >= Constants.AUTO_EJECT_ENCODER_STOP_VALUE)
		{
			ejectDrop();
		}
		else
		{
			intakeOff();
			setAutoEjecting(false);
			done = true;
		}
		
		return done;
	}


	/**
	 * Method to shut off the intake motors.
	 */
	public void intakeOff()
	{
		//TODO: Make velocity-based control work
		//leftIntakeTalon.set(ControlMode.Velocity, 0);
		//rightIntakeTalon.set(ControlMode.Velocity, 0);

		//		leftIntakeTalon.set(ControlMode.PercentOutput, 0);
		//		rightIntakeTalon.set(ControlMode.PercentOutput, 0);

		leftIntakeTalon.set(0);
		rightIntakeTalon.set(0);
	}

	/**
	 * Intake with different speed in order to rotate the power cube for a better grip.
	 */
	public void intakeDifferentSpeed()
	{
		leftIntakeTalon.set(-0.45);
		rightIntakeTalon.set(-0.7);
	}
	
	
	/**
	 * Intake method for teleoperated mode.
	 */
	public void intake()
	{
		leftIntakeTalon.set(-0.50);
		rightIntakeTalon.set(-0.50);
	}

	/**
	 * Method to shoot cube for use in teleoperated mode.
	 */
	public void ejectShoot()
	{
		leftIntakeTalon.set(1);
		rightIntakeTalon.set(1);
	}

	/**
	 * Method to gently drop cube for use in teleoperated mode.
	 */
	public void ejectDrop()
	{
		leftIntakeTalon.set(0.25);
		rightIntakeTalon.set(0.25);
	}

	/**
	 * Method to spin cube clockwise into left arm.
	 */
	public void intakeRotateCubeLeft()
	{
		leftIntakeTalon.set(0.0);
		rightIntakeTalon.set(0.70);
	}

	/**
	 * Method to spin cube counterclockwise into right arm.
	 */
	public void intakeRotateCubeRight()
	{
		leftIntakeTalon.set(0.0);
		rightIntakeTalon.set(-0.70);
	}

	/**
	 * Method to move pivot arm at a specified speed.
	 * @param value Speed at which to move the arm.
	 */
	public void pivot(double value)
	{
		//setPivoting(true);
		pivotTalon.set(value);
	}

	/**
	 * Method to angle the pivoter up.
	 */
	public void raise()
	{	
		pivot(0.75);
	}

	/**
	 * Method to angle the pivoter down.
	 */
	public void lower()
	{
		pivot(-0.75);
	}

	/**
	 * Method to stop the pivoter.
	 */
	public void pivotOff()
	{
		currentDirection = Constants.Direction.None;
		setPivoting(false);
		pivotTalon.set(0.0);
	}

	/**
	 * Method to pulsate the intake motors.
	 * Can be used to ensure the power cube is securely held in the robot.
	 */
	public void pulsateIntake()
	{
		leftIntakeTalon.set(-0.3);
		rightIntakeTalon.set(-0.3);
	}

	/**
	 * Method to get the tick value of the left intake motor's encoder.
	 * @return The encoder's value.
	 */
	public int getLeftIntakeEncoder()
	{
		return leftIntakeTalon.getSelectedSensorPosition(0);
	}

	/*
     * Method to get the tick value of the right intake motor's encoder.
	 * @return The encoder's value.
	 */
	public int getRightIntakeEncoder()
	{
		return rightIntakeTalon.getSelectedSensorPosition(0);
	}

	/**
	 * Method to set the upper and lower bounds for the arm in autonomous mode.
	 */
	public void setAutoLimits()
	{
		pivotTalon.configForwardSoftLimitThreshold(Constants.RAISED, 0);
	}

	/**
	 * Method to set the upper and lower bounds for the arm in teleoperated mode.
	 */
	public void setTeleopLimits()
	{
		pivotTalon.configForwardSoftLimitThreshold(Constants.TELEOP_MAX, 0);
	}

	/**
	 * Main loop method for teleoperated mode.
	 * Handles input from the Operator's Xbox controller to control the movement of the gripper and pivoting arm.
	 */
	public void teleop()
	{

		//Pivot down one level
		boolean aButton = xbox.getRawButton(Xbox.Constants.A_BUTTON);

		//Spin Cube right
		boolean bButton = xbox.getRawButton(Xbox.Constants.B_BUTTON);

		//Pivot up one level
		boolean yButton = xbox.getRawButton(Xbox.Constants.Y_BUTTON);

		//Spin Cube left
		boolean xButton = xbox.getRawButton(Xbox.Constants.X_BUTTON);

		//Free move for pivoter
		double rightYAxis = xbox.getRawAxis(Xbox.Constants.RIGHT_STICK_Y_AXIS);

		//Eject
		double rightTrigger = xbox.getRawAxis(Xbox.Constants.RIGHT_TRIGGER_AXIS);

		//Intake
		double leftTrigger = xbox.getRawAxis(Xbox.Constants.LEFT_TRIGGER_AXIS);

		updateCurrentRange();

		if (Math.abs(rightYAxis) > 0.2)
		{
			pivot(-rightYAxis);
			isFloorTarget = false;
		}
		else if (aButton || isFloorTarget)
		{
			isFloorTarget = true;
			if(currentValue > Constants.FLOOR + Constants.THRESHOLD)
			{
				lower();
			}
			else
			{
				pivotOff();
				isFloorTarget = false;
			}
		}
		else
		{
			pivotOff();
		}

		//Intake
		if (Math.abs(rightTrigger) > 0.3)
		{
			ejectShoot();
		}
		else if (Math.abs(leftTrigger) > 0.3)
		{
			if (bButton)
			{
				intakeRotateCubeRight();
			}
			else
			{
				intake();
			}
		}
		else if (xButton)
		{
			ejectDrop();
		}
		else if (yButton)
		{
			if(!isPulsateTimerReset)
			{
				pulsateTimer.stop();
				pulsateTimer.reset();
				pulsateTimer.start();

				isPulsateTimerReset = true;
			}
			if(pulsateTimer.get() < 0.1 )
			{
				pulsateIntake();
			}
			else
			{
				intakeOff();

				if(pulsateTimer.get() > 0.35)
				{
					isPulsateTimerReset = false;
				}
			}
		}
		else
		{
			intakeOff();
			isPulsateTimerReset = false;
		}
	}

	public void test()
	{
		leftBumper = xbox.getRawButtonPressed(Xbox.Constants.LEFT_BUMPER);
		rightBumper = xbox.getRawButtonPressed(Xbox.Constants.RIGHT_BUMPER);
		aButton = xbox.getRawButton(Xbox.Constants.A_BUTTON);

		if (leftBumper) 
		{
			currentTestKeyPosition--;

		}
		else if (rightBumper) 
		{
			currentTestKeyPosition++;
		}

		if (currentTestKeyPosition >= talonSRXHashMap.keySet().size()) 
		{
			currentTestKeyPosition = talonSRXHashMap.size() - 1;
		}
		else if (currentTestKeyPosition < 0) 
		{
			currentTestKeyPosition = 0;
		}

		if (aButton)
		{
			talonSRXHashMap.get(currentTestKeyPosition).set(ControlMode.PercentOutput, 0.3);
		}
		else
		{
			for (int i : talonSRXHashMap.keySet())
			{
				talonSRXHashMap.get(i).set(ControlMode.PercentOutput, 0.0);
			}
		}
		printTestInfo();
	}

	/**
	 * Updates the current positional range of the pivoting arm.
	 */
	public void updateCurrentRange()
	{
		currentValue = getPivotPotentiometer();
		//System.out.println("Pivot pot: " + getPivotPotentiometer());
		if (currentValue <= Constants.Range.floorRange.topValue())
		{
			currentRange = Constants.Range.floorRange;
		}
		else if (currentValue <= Constants.Range.floorHorizontalRange.topValue())
		{
			currentRange = Constants.Range.floorHorizontalRange;
		}
		else if (currentValue <= Constants.Range.horizontalRange.topValue())
		{
			currentRange = Constants.Range.horizontalRange;
		}
		else if (currentValue <= Constants.Range.horizontalMiddleRange.topValue())
		{
			currentRange = Constants.Range.horizontalMiddleRange;
		}
		else if (currentValue <= Constants.Range.middleRange.topValue())
		{
			currentRange = Constants.Range.middleRange;
		}
		else if (currentValue <= Constants.Range.middleTeleopMaxRange.topValue())
		{
			currentRange = Constants.Range.middleTeleopMaxRange;
		}
		else if (currentValue <= Constants.Range.teleopMaxRange.topValue())
		{
			currentRange = Constants.Range.teleopMaxRange;
		}
		else if (currentValue <= Constants.Range.teleopMaxRaisedRange.topValue())
		{
			currentRange = Constants.Range.teleopMaxRaisedRange;
		}
		else if (currentValue <= Constants.Range.raisedRange.topValue())
		{
			currentRange = Constants.Range.raisedRange;
		}
	}

    /**
     * Method to check if autonomous pivot to middle is finished.
     * @return State of pivot, if it's finished or not.
     */
	public boolean isAutoMiddlePivotDone()
	{
		return isAutoPivotMiddleDone;
	}

    /**
     * Method to check if autonomous pivot to floor is finished.
     * @return State of pivot, if it's finished or not.
     */
	public boolean isAutoFloorPivotDone()
	{
		return isAutoPivotFloorDone;
	}

    /**
     * Method to check if autonomous pivot to raised position is finished.
     * @return State of pivot, if it's finished or not.
     */
	public boolean isAutoRaisedPivotDone()
	{
		return isAutoPivotRaisedDone;
	}

    /**
     * Method to get the velocities of the intake motors.
     * @return An array containing the velocities of the two intake motors.
     */
	public int[] getVelocityArray()
	{
		return new int[] {leftIntakeTalon.getSelectedSensorVelocity(0), rightIntakeTalon.getSelectedSensorVelocity(0)};
	}

    /**
     * Method to get the value of the pivoting arm's potentiometer.
     * @return The value of the pivoting arm's potentiometer.
     */
	public int getPivotPotentiometer()
	{
		return pivotTalon.getSelectedSensorPosition(0);
	}

    /**
     * Method to check whether or not the intake is autonomously ejecting a power cube.
     * @return State of ejection, whether is ejecting or not.
     */
	public boolean isAutoEjecting()
	{
		return isAutoEjecting;
	}

	public void setAutoEjecting(boolean isAutoEjecting)
	{
		resetIntakeEncoder();
		this.isAutoEjecting = isAutoEjecting;
	}

	public boolean isAutoIntaking()
	{
		return isAutoIntaking;
	}

	public void setAutoIntaking(boolean isAutoIntaking)
	{
		this.isAutoIntaking = isAutoIntaking;
	}

	public boolean isPivoting()
	{
		return isPivoting;
	}

	public void setPivoting(boolean isPivoting)
	{
		this.isPivoting = isPivoting;
	}

	public boolean inTargetRange()
	{
		return  (currentValue >= targetRange[0]) && (currentValue <= targetRange[1]);
	}

	public void autoSetRaisedTargetRange()
	{
		currentDirection = Constants.Direction.Down;
		targetRange = Constants.Range.raisedRange.range;
	}

	public void autoSetMiddleTargetRange()
	{
		currentDirection = Constants.Direction.Down;
		targetRange = Constants.Range.middleRange.range;
	}

	public void autoSetHorizontalTargetRange()
	{
		currentDirection = Constants.Direction.Down;
		targetRange = Constants.Range.horizontalRange.range;
	}

	public void autoSetFloorTargetRange()
	{
		currentDirection = Constants.Direction.Down;
		targetRange = Constants.Range.floorRange.range;
	}

	public Constants.Range getCurrentRange()
	{
		return this.currentRange;
	}

	public void printTestInfo()
	{
		System.out.printf("Pivot = %5d		Left Intake = %5d		Right Intake = %5d", getPivotPotentiometer(), getLeftIntakeEncoder(), getRightIntakeEncoder());
		System.out.println("[Gripper] Pivot arm potentiometer position: " + getPivotPotentiometer());
	}

	public void resetIntakeEncoder()
	{
		rightIntakeTalon.setSelectedSensorPosition(0, 0, 0);		// set encoder position
	}

	public boolean autoHorizontal()
	{
		boolean inHorizontalRange = false;
		updateCurrentRange();
		if(currentValue > Constants.HORIZONTAL + Constants.THRESHOLD)
		{
			lower();
		}
		else if(currentValue < Constants.HORIZONTAL - Constants.THRESHOLD)
		{
			raise();
		}
		else
		{
			inHorizontalRange = true;
			pivotOff();
		}

		return inHorizontalRange;
	}

	public boolean autoMiddle()
	{
		boolean inMiddleRange = false;
		updateCurrentRange();
		if(currentValue > Constants.MIDDLE + Constants.THRESHOLD)
		{
			lower();
		}
		else if(currentValue < Constants.MIDDLE - Constants.THRESHOLD)
		{
			raise();
		}
		else
		{
			inMiddleRange = true;
			pivotOff();
		}

		return inMiddleRange;
	}

	public boolean autoFloor()
	{
		boolean inFloorRange = false;
		updateCurrentRange();
		if(currentValue > Constants.FLOOR + Constants.THRESHOLD)
		{
			lower();
		}
		else if(currentValue < Constants.FLOOR - Constants.THRESHOLD)
		{
			raise();
		}
		else
		{
			inFloorRange = true;
			pivotOff();
		}

		return inFloorRange;
	}

	/**
	 * Constants class for Gripper
	 */
	public static class Constants
	{
		public enum Direction
		{
			Up,
			Down,
			None
		}

		private enum InitRange
		{	
			floorRange(Constants.FLOOR, Constants.FLOOR + Constants.THRESHOLD),
			floorHorizontalRange(Constants.FLOOR + Constants.THRESHOLD, Constants.HORIZONTAL - Constants.THRESHOLD),
			horizontalRange(Constants.HORIZONTAL - Constants.THRESHOLD, Constants.HORIZONTAL + Constants.THRESHOLD),
			horizontalMiddleRange(Constants.HORIZONTAL + Constants.THRESHOLD, Constants.MIDDLE - Constants.THRESHOLD),
			middleRange(Constants.MIDDLE - Constants.THRESHOLD, Constants.MIDDLE + Constants.THRESHOLD),
			middleTeleopMaxRange(Constants.MIDDLE + Constants.THRESHOLD, Constants.TELEOP_MAX - Constants.THRESHOLD),
			teleopMaxRange(Constants.TELEOP_MAX - Constants.THRESHOLD, Constants.TELEOP_MAX + Constants.THRESHOLD),
			teleopMaxRaisedRange(Constants.TELEOP_MAX + Constants.THRESHOLD, Constants.RAISED - Constants.THRESHOLD),
			raisedRange(Constants.RAISED - Constants.THRESHOLD, Constants.RAISED),
			none(-1, -1),
			error(-1, -1);

			private final int[] range;

			InitRange(int bottomValue, int topValue)
			{
				this.range = new int[] {bottomValue, topValue};
			}

			public int[] range()
			{
				return this.range;
			}

			public double bottomValue()
			{
				return this.range[0];
			}

			public double topValue()
			{
				return this.range[1];
			}
		}

		//Sets lower and upper limits for each range
		public enum Range
		{
			floorRange(       		InitRange.floorRange.range(),        		InitRange.floorRange, 		InitRange.horizontalRange),
			floorHorizontalRange(  	InitRange.floorHorizontalRange.range(),		InitRange.floorRange,  		InitRange.horizontalRange),
			horizontalRange(		InitRange.horizontalRange.range(), 			InitRange.floorRange,		InitRange.middleRange),
			horizontalMiddleRange(	InitRange.horizontalMiddleRange.range(),	InitRange.horizontalRange,	InitRange.middleRange),
			middleRange(       		InitRange.middleRange.range(),       		InitRange.horizontalRange,  InitRange.teleopMaxRange),
			middleTeleopMaxRange( 	InitRange.middleTeleopMaxRange.range(), 	InitRange.middleRange,		InitRange.teleopMaxRange),
			teleopMaxRange(			InitRange.teleopMaxRange.range(),			InitRange.middleRange,		InitRange.raisedRange),
			teleopMaxRaisedRange(	InitRange.teleopMaxRaisedRange.range(),		InitRange.teleopMaxRange,	InitRange.raisedRange),
			raisedRange(      		InitRange.raisedRange.range(),       		InitRange.teleopMaxRange,	InitRange.raisedRange),
			error(             		InitRange.error.range(),             		InitRange.error,      		InitRange.error);

			private final int[] range;
			private final InitRange higherNeighbor;
			private final InitRange lowerNeighbor;

			Range(int[] range, Constants.InitRange lowerNeighbor, Constants.InitRange higherNeighbor)
			{
				this.range = range;
				this.higherNeighbor = higherNeighbor;
				this.lowerNeighbor = lowerNeighbor;
			}

			public int[] range()
			{
				return this.range;
			}

			public double bottomValue()
			{
				return this.range[0];
			}

			public double topValue()
			{
				return this.range[1];
			}

			public InitRange lowerNeighbor()
			{
				return this.lowerNeighbor;
			}

			public InitRange higherNeighbor()
			{
				return this.higherNeighbor;
			}
		}

		public static final int AUTO_EJECT_ENCODER_STOP_VALUE = -2000;
		public static final int AUTO_INTAKE_ENCODER_STOP_VALUE = 2000;

		public static final int POTENTIOMETER_FLOOR_POSITION = 210;
		public static final int POTENTIOMETER_MIDDLE_POSITION = 190;
		public static final int POTENTIOMETER_RAISED_POSITION = 94;

		public static final int LEFT_INTAKE_MOTOR_PORT = 8;
		public static final int RIGHT_INTAKE_MOTOR_PORT = 9;
		public static final int PIVOTER_MOTOR_PORT = 6;

		public static final int PID_SLOT_ID = 0;

		public static final int INTAKE_40_AMP_TRIGGER = 60;
		public static final int INTAKE_40_AMP_LIMIT = 30;
		public static final int INTAKE_40_AMP_TIME = 4000;

		public static final double INTAKE_RAMP_TIME = 0.125;
		public static final int INTAKE_RAMP_RATE_TIMEOUT = 10;
		public static final int THRESHOLD = 5;
		public static final int FLOOR = 280;	//was 269
		public static final int HORIZONTAL = 333;
		public static final int MIDDLE  = 435;
		public static final int TELEOP_MAX = 465;
		public static final int RAISED = 465;
	}
}