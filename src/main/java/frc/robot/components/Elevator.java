package frc.robot.components;

import java.util.HashMap;

import frc.robot.control.DriverXbox;
import frc.robot.control.OperatorXbox;
import frc.robot.control.Xbox;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This class represents the robot's elevator component.
 * It contains all the methods needed to accurately control and position the elevator.
 * @author Ben Puzycki, Darryl Wong, Mark Washington
 *
 */
public class Elevator implements Component
{
	private OperatorXbox operatorXbox = OperatorXbox.getInstance();
	private DriverXbox driverXbox = DriverXbox.getInstance();
	
	private WPI_TalonSRX masterTalonSRX = new WPI_TalonSRX(Constants.MASTER_MOTOR_PORT); 
	private WPI_TalonSRX slaveTalonSRX = new WPI_TalonSRX(Constants.SLAVE_MOTOR_PORT);

	private HashMap<Integer, WPI_TalonSRX> talonSRXHashMap = new HashMap<Integer, WPI_TalonSRX>();

	private double currentValue;
	private double[] targetRange = Constants.Range.floorRange.range;

	private Constants.Range currentRange;
	private Constants.Direction currentDirection = Constants.Direction.None;

	private boolean isMoving = false;

	private Constants.InitRange autoTargetRange = Constants.InitRange.error;

	private boolean limitsEnabled = true;

	private int currentTestKeyPosition = 0;

	//Joystick buttons
	private boolean leftBumper;
	private boolean rightBumper;
	private boolean leftStickButton;
	private boolean aButton;

	private boolean isFloorTarget = false;

	private double leftYAxis;

	private static Elevator instance = new Elevator();

    /**
     * Method to return the singleton instance of the elevator class.
     * @return Instance of Elevator.
     */
	public static Elevator getInstance()
	{
		return instance;
	}

	/**
	 * Constructor for Elevator, called only once by getInstance(). It initializes the keys and values in levelTicks and
	 * tickLevels.
	 */
	private Elevator()
	{
		masterTalonSRX.setNeutralMode(NeutralMode.Brake);
		slaveTalonSRX.setNeutralMode(NeutralMode.Brake);
		
		masterTalonSRX.configSelectedFeedbackSensor(com.ctre.phoenix.motorcontrol.FeedbackDevice.Analog, 0, 0);
		//masterTalonSRX.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0x00, 0x00, 0);
		masterTalonSRX.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 20);
		masterTalonSRX.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 20);

		masterTalonSRX.configForwardSoftLimitThreshold(Constants.ABSOLUTE_TOP, 0);
		masterTalonSRX.configReverseSoftLimitThreshold(Constants.FLOOR, 0);
		masterTalonSRX.configForwardSoftLimitEnable(limitsEnabled, 0);
		masterTalonSRX.configReverseSoftLimitEnable(limitsEnabled, 0);

		slaveTalonSRX.follow(masterTalonSRX); // Sets slaveTalonSRX to follow masterTalonSrx

		talonSRXHashMap.put(Constants.MASTER_MOTOR_PORT, masterTalonSRX);
		talonSRXHashMap.put(Constants.SLAVE_MOTOR_PORT, slaveTalonSRX);
	}

	/**
	 * Method to raise the elevator.
	 */
	public void raise()
	{
		masterTalonSRX.set(1.0);
	}

	/**
	 * Method to lower the elevator.
	 */
	public void lower()
	{
		masterTalonSRX.set(-1.0);
	}

	public void overrideRaise()
	{
		masterTalonSRX.set(0.5);
	}

	public void overrideLower()
	{
		masterTalonSRX.set(-0.5);
	}

	/**
	 * Main loop method for teleoperated mode.
     * Handles input from the Operator's Xbox controller to control the movement of the elevator.
	 */
	public void teleop()
	{
		leftStickButton = operatorXbox.getRawButton(Xbox.Constants.LEFT_STICK_BUTTON);

		aButton = operatorXbox.getRawButton(Xbox.Constants.A_BUTTON);

		leftYAxis = operatorXbox.getRawAxis(Xbox.Constants.LEFT_STICK_Y_AXIS);

		updateCurrentRange();

		if(Math.abs(leftYAxis) > 0.2)	
		{
			if (leftStickButton) //If left stick button is pressed
			{
				if (limitsEnabled) //and if the talon limits are enabled
				{
					limitsEnabled = false; //Set limits to disabled

					masterTalonSRX.configForwardSoftLimitEnable(limitsEnabled, 0); //Actually disable limits
					masterTalonSRX.configReverseSoftLimitEnable(limitsEnabled, 0);
				}
				masterTalonSRX.set(-leftYAxis * Constants.OVERRIDE_SPEED_SCALE); //Set elevator motor to value of joystick * scaling constant
			}
			else if (!leftStickButton && !limitsEnabled) //otherwise, if the left stick button is not pressed and the limits are not enabled
			{
				limitsEnabled = true; //Set limits to enabled
				masterTalonSRX.configForwardSoftLimitEnable(limitsEnabled, 0); //Actually enable limits
				masterTalonSRX.configReverseSoftLimitEnable(limitsEnabled, 0); 
			}
			else //if left stick button is not pressed AND limits are not enabled
			{
				masterTalonSRX.set(-leftYAxis); //Set elevator motor to value of joystick
			}

			isFloorTarget = false;
		}
		else if(aButton || isFloorTarget)
		{
			isFloorTarget = true;
			if(currentValue > Constants.FLOOR + Constants.THRESHOLD)
			{
				lower();
			}
			else
			{
				isFloorTarget = false;
				stopMoving();
			}
		}
		else
		{
			stopMoving();
			//xbox.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
		}
	}

    /**
     * Main loop method for autonomous mode.
	 * Does not directly control the elevator. It updates the current position of the elevator.
     */
	public void autonomous()
	{
		updateCurrentRange();
	}

	public void test()
	{
		leftBumper = operatorXbox.getRawButtonPressed(Xbox.Constants.LEFT_BUMPER);
		rightBumper = operatorXbox.getRawButtonPressed(Xbox.Constants.RIGHT_BUMPER);
		aButton = operatorXbox.getRawButton(Xbox.Constants.A_BUTTON);

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
	 * Method to update the current position of the elevator.
	 */
	public void updateCurrentRange()
	{
		currentValue = getPosition();
		//System.out.println("Elevator pot: " + currentValue);
		if (currentValue <= Constants.Range.floorRange.topValue())
		{
			currentRange = Constants.Range.floorRange;
			//			System.out.println("Current Range: " + Constants.Range.floorRange);
		}
		else if (currentValue <= Constants.Range.floorExchangeAndSwitchAndPortalRange.topValue())
		{
			currentRange = Constants.Range.floorExchangeAndSwitchAndPortalRange;
			//			System.out.println("Current Range: " + Constants.Range.floorAndSwitchExchangeAndPortalRange);
		}
		else if (currentValue <= Constants.Range.exchangeAndSwitchAndPortalRange.topValue())
		{
			currentRange = Constants.Range.exchangeAndSwitchAndPortalRange;
			//			System.out.println("Current Range: " + Constants.Range.exchangeAndPortalRange);
		}
		else if (currentValue <= Constants.Range.exchangeAndSwitchAndPortalBottomScaleRange.topValue())
		{
			currentRange = Constants.Range.exchangeAndSwitchAndPortalBottomScaleRange;
			//			System.out.println("Current Range: " + Constants.Range.exchangeAndPortalSwitchRange);
		}
		else if (currentValue <= Constants.Range.bottomScaleRange.topValue())
		{
			currentRange = Constants.Range.bottomScaleRange;
			//			System.out.println("Current Range: " + Constants.Range.bottomScaleRange);
		}
		else if (currentValue <= Constants.Range.bottomScaleTopScaleRange.topValue())
		{
			currentRange = Constants.Range.bottomScaleTopScaleRange;
			//			System.out.println("Current Range: " + Constants.Range.bottomScaleTopScaleRange);
		}
		else //if (currentValue <= Constants.Range.topScaleRange.topValue())
		{
			currentRange = Constants.Range.topScaleRange;
			//			System.out.println("Current Range: " + Constants.Range.topScaleRange);
		}

	}

	/**
	 * Gets the current range of the elevator.
	 * @return The current range of the elevator.
	 */
	public Constants.Range getCurrentRange()
	{
		return currentRange;
	}

	/**
	 * Method to stop the elevator.
	 */
	public void stopMoving()
	{
		currentDirection = Constants.Direction.None;
		masterTalonSRX.set(0.0);
		setMoving(false);
	}

	/**
	 * Gets the potentiometer value of the elevator.
	 * @return The potentiometer value of the elevator.
	 */
	public double getPosition()
	{
		return masterTalonSRX.getSelectedSensorPosition(0);
	}

	/**
	 * Checks if elevator is moving or not.
	 * @return If elevator is moving.
	 */
	public boolean isMoving()
	{
		return isMoving;
	}

	/**
	 * Method to set whether elevator is moving or not.
	 * @param isMoving Whether the elevator is moving or not.
	 */
	public void setMoving(boolean isMoving)
	{
		this.isMoving = isMoving;
	}

	/**
	 * Method to check if elevator is in the set target range.
	 * @return If elevator is in current range.
	 */
	public boolean inTargetRange()
	{
		return  (currentValue >= targetRange[0]) && (currentValue <= targetRange[1]);
	}

	/**
	 * Set the target range of the elevator to the scale position. For use in autonomous.
	 */
	public void autoSetScaleTargetRange()
	{
		currentDirection = Constants.Direction.Up;
		targetRange = Constants.Range.topScaleRange.range();
	}

	/**
	 * Sets the target range of the elevator to the switch position. For use in autonomous.
	 */
	public void autoSetSwitchTargetRange()
	{
		targetRange = Constants.Range.exchangeAndSwitchAndPortalRange.range;
		currentDirection = Constants.Direction.Up;
	}

	/**
	 * Sets the target range of the elevator to the floor position. For use in autonomous.
	 */
	public void autoSetFloorTargetRange()
	{
		currentDirection = Constants.Direction.Up;
		targetRange = Constants.Range.floorRange.range;
	}

	/**
	 * Prints important data about the operation of the elevator.
	 */
	public void printTestInfo()
	{
		//System.out.printf("ID: %2d Potentiometer Position: %.2f", talonSRXHashMap.keySet().toArray()[currentTestKeyPosition], getPosition());
		System.out.println("[Elevator] String-potentiometer position: " + getPosition());
	}

	/**
	 * Automatically move the elevator to the floor position.
	 * @return Whether the elevator is in the floor position or not.
	 */
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
			stopMoving();
		}

		return inFloorRange;
	}

	/**
	 * Automatically move the elevator to the switch position.
	 * @return Whether the elevator is in the switch position or not.
	 */
	public boolean autoSwitch()
	{
		boolean inSwitchRange = false;
		updateCurrentRange();
		if(currentValue < Constants.SWITCH - Constants.THRESHOLD)
		{
			raise();
		}
		else if(currentValue > Constants.SWITCH + Constants.THRESHOLD)
		{
			lower();
		}
		else
		{
			inSwitchRange = true;
			stopMoving();
		}

		return inSwitchRange;
	}

	/**
	 * Automatically move the elevator to the high scale position.
	 * @return Whether the elevator is in the high scale position or not.
	 */
	public boolean autoTopScale()
	{
		boolean inScaleRange = false;
		updateCurrentRange();
		if(currentValue < Constants.TOP_SCALE - Constants.THRESHOLD)
		{
			raise();
		}
		else if(currentValue > Constants.TOP_SCALE + Constants.THRESHOLD)
		{
			lower();
		}
		else
		{
			inScaleRange = true;
			stopMoving();
		}

		return inScaleRange;
	}

    /**
     * Class to store constant variables used by the Elevator class.
     */
	public static class Constants
	{		
		private enum InitRange
		{
			//			floorRange(118, 138),
			//			floorExchangeAndSwitchAndPortalRange(138, 218),
			//			exchangeAndSwitchAndPortalRange(218, 258),
			//			exchangeAndSwitchAndPortalBottomScaleRange(258, 406),
			//			bottomScaleRange(406, 446),
			//			bottomScaleTopScaleRange(446, 568),
			//			topScaleRange(568, 588),
			//			none(-1, -1),
			//			error(-1, -1);

			floorRange(Constants.FLOOR, Constants.FLOOR + Constants.THRESHOLD),
			floorExchangeAndSwitchAndPortalRange(Constants.FLOOR + Constants.THRESHOLD, Constants.SWITCH - Constants.THRESHOLD),
			exchangeAndSwitchAndPortalRange(Constants.SWITCH - Constants.THRESHOLD, Constants.SWITCH + Constants.THRESHOLD),
			exchangeAndSwitchAndPortalBottomScaleRange(Constants.SWITCH + Constants.THRESHOLD, Constants.BOTTOM_SCALE - Constants.THRESHOLD),
			bottomScaleRange(Constants.BOTTOM_SCALE - Constants.THRESHOLD, Constants.BOTTOM_SCALE + Constants.THRESHOLD),
			bottomScaleTopScaleRange(Constants.BOTTOM_SCALE + Constants.THRESHOLD, Constants.TOP_SCALE - Constants.THRESHOLD),
			topScaleRange(Constants.TOP_SCALE - Constants.THRESHOLD, Constants.TOP_SCALE),
			none(-1, -1),
			error(-1, -1);

			private final double[] range;

			InitRange(double bottomValue, double topValue)
			{
				this.range = new double[] {bottomValue, topValue};
			}

			public double[] range()
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

		public enum Range
		{
			floorRange(     								InitRange.floorRange.range(),                 					InitRange.floorRange,        				InitRange.exchangeAndSwitchAndPortalRange),
			floorExchangeAndSwitchAndPortalRange(   		InitRange.floorExchangeAndSwitchAndPortalRange.range(),			InitRange.floorRange,        				InitRange.exchangeAndSwitchAndPortalRange),
			exchangeAndSwitchAndPortalRange(         		InitRange.exchangeAndSwitchAndPortalRange.range(),				InitRange.floorRange,        				InitRange.bottomScaleRange),
			exchangeAndSwitchAndPortalBottomScaleRange(   	InitRange.exchangeAndSwitchAndPortalBottomScaleRange.range(),	InitRange.exchangeAndSwitchAndPortalRange,  InitRange.bottomScaleRange),
			bottomScaleRange(           					InitRange.bottomScaleRange.range(),          					InitRange.exchangeAndSwitchAndPortalRange, 	InitRange.topScaleRange), 
			bottomScaleTopScaleRange(   					InitRange.bottomScaleTopScaleRange.range(),   					InitRange.bottomScaleRange,  				InitRange.topScaleRange),
			topScaleRange(              					InitRange.topScaleRange.range(),              					InitRange.bottomScaleRange,  				InitRange.topScaleRange),
			error(                      					InitRange.error.range(),                      					InitRange.error,             				InitRange.error);

			private final double[] range;
			private final InitRange higherNeighbor;
			private final InitRange lowerNeighbor;

			Range(double[] range, Constants.InitRange lowerNeighbor, Constants.InitRange higherNeighbor)
			{
				this.range = range;
				this.higherNeighbor = higherNeighbor;
				this.lowerNeighbor = lowerNeighbor;
			}

			public double[] range()
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

		public enum Direction
		{
			Up,
			Down,
			None
		}

		public static final int ACCEPTABLE_TICK_RANGE = 5;

		public static final int MASTER_MOTOR_PORT = 10;
		public static final int SLAVE_MOTOR_PORT = 11;
		public static final int STRING_POT_PORT = 3;
		public static final int OTHER_STRING_POT_PORT = 7;

		public static final int CLIMBER_DEPLOY_SOLENOID_PORT = 0;

		public static final double STRING_POT_SCALE = 1.0;

		public static final double SPEED = 0.5;

		public static final int THRESHOLD = 15;
		public static final int FLOOR = 145;
		public static final int SWITCH = 360;
		public static final int BOTTOM_SCALE = 426;
		public static final int TOP_SCALE = 780;
		public static final int ABSOLUTE_TOP = 780;

		public static final double OVERRIDE_SPEED_SCALE = 0.7;

	}
}

