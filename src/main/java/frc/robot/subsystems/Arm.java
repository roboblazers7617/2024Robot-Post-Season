// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.shuffleboard.MotorTab;
// import frc.robot.util.TunableNumber;

public class Arm extends SubsystemBase {
	// Arm
	/** this is the right arm motor */
	private final SparkMax leaderArmMotor = new SparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax followerArmMotor = new SparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	
	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor.getAbsoluteEncoder();
	private final SparkClosedLoopController armPIDController = leaderArmMotor.getClosedLoopController();
	
	private ArmFeedforward extendedArmFeedForward = new ArmFeedforward(ArmConstants.EXTENDED_KS, ArmConstants.EXTENDED_KG, ArmConstants.EXTENDED_KV);
	private ArmFeedforward retractedArmFeedForward = new ArmFeedforward(ArmConstants.RETRACTED_KS, ArmConstants.RETRACTED_KG, ArmConstants.RETRACTED_KV);
	
	/** the current target for the arm, in degrees, it is within the total bounds of the arm but may not be a currently safe move */
	// of the arm so the arm doesn't try to move on boot-up
	private double armTarget;
	/** the last actual arm target */
	private double lastAcutalArmTarget;
	/** arm angle based on distance interpolation table */
	private final InterpolatingDoubleTreeMap armAngleBasedOnDistanceExtended = new InterpolatingDoubleTreeMap();
	private final InterpolatingDoubleTreeMap armAngleBasedOnDistanceRetracted = new InterpolatingDoubleTreeMap();
	
	// Elevator
	/** the right motor */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** the left motor */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	
	private final RelativeEncoder elevatorEncoder = leaderElevatorMotor.getEncoder();
	private final SparkClosedLoopController elevatorPIDController = leaderElevatorMotor.getClosedLoopController();
	
	InterpolatingDoubleTreeMap elevatorKSTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKGTable = new InterpolatingDoubleTreeMap();
	InterpolatingDoubleTreeMap elevatorKVTable = new InterpolatingDoubleTreeMap();
	
	private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	
	/** the current target for the elevator, it is within the total bounds of the arm but may not be a currently safe move */
	private double elevatorTarget;
	/** the last actual elevator target */
	private double lastAcutalElevatorTarget;
	
	/** this is used for the position setpoint, in degrees, for setVelocity() */
	private double dt, lastTime;
	private Timer time = new Timer();
	
	private final MotorTab motorTab = new MotorTab(4, "arm", 2);
	
	/** Creates a new Arm. */
	public Arm() {
		// setup arm motors
		ClosedLoopConfig armControllerConfig = new ClosedLoopConfig()
				.p(ArmConstants.KP)
				.i(ArmConstants.KI)
				.d(ArmConstants.KD)
				.outputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
				.positionWrappingEnabled(false);
		
		AbsoluteEncoderConfig armAbsoluteEncoderConfig = new AbsoluteEncoderConfig()
				.positionConversionFactor(ArmConstants.ABS_POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(ArmConstants.ABS_VELOCITY_CONVERSION_FACTOR)
				.inverted(true)
				.zeroOffset(ArmConstants.ARM_OFFSET);
		
		SparkBaseConfig armMotorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(ArmConstants.MAX_AMPERAGE);
		
		SparkBaseConfig leaderArmMotorConfig = new SparkMaxConfig()
				.apply(armMotorConfig)
				.inverted(true)
				.apply(armControllerConfig)
				.apply(armAbsoluteEncoderConfig);
		
		SparkBaseConfig followerArmMotorConfig = new SparkMaxConfig()
				.apply(armMotorConfig)
				.follow(leaderArmMotor, true)
				.apply(MotorConstants.SLOW_SIGNALS_CONFIG);
		
		leaderArmMotor.configure(leaderArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		followerArmMotor.configure(followerArmMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		
		armTarget = armAbsoluteEncoder.getPosition();
		
		armAngleBasedOnDistanceExtended.put(1.27, ShootingPosition.SUBWOOFER.arm_angle());
		armAngleBasedOnDistanceExtended.put(2.9, 33.5);
		armAngleBasedOnDistanceExtended.put(3.1, 36.0);
		
		// TODO: Add Treemap values
		armAngleBasedOnDistanceRetracted.put(1.96, 18.6);
		armAngleBasedOnDistanceRetracted.put(2.47, 27.0);
		armAngleBasedOnDistanceRetracted.put(2.92, 31.8);
		armAngleBasedOnDistanceRetracted.put(2.96, 30.9);
		armAngleBasedOnDistanceRetracted.put(3.51, 35.1);
		armAngleBasedOnDistanceRetracted.put(3.61, 32.7);
		armAngleBasedOnDistanceRetracted.put(4.11, 37.3);
		armAngleBasedOnDistanceRetracted.put(4.25, 38.3);
		armAngleBasedOnDistanceRetracted.put(4.29, 38.3);
		armAngleBasedOnDistanceRetracted.put(4.31, 37.6);
		
		// setup elevator motors
		ClosedLoopConfig elevatorControllerConfig = new ClosedLoopConfig()
				.p(ElevatorConstants.KP)
				.i(ElevatorConstants.KI)
				.d(ElevatorConstants.KD)
				.outputRange(ElevatorConstants.kMinOutput, ElevatorConstants.kMaxOutput);
		
		SparkBaseConfig elevatorMotorConfig = new SparkMaxConfig()
				.smartCurrentLimit(ElevatorConstants.MAX_AMPERAGE)
				.idleMode(IdleMode.kBrake);
		
		SparkBaseConfig leaderElevatorMotorConfig = new SparkMaxConfig()
				.apply(elevatorMotorConfig)
				.inverted(true);
		
		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig()
				.apply(elevatorMotorConfig)
				.follow(leaderArmMotor, true)
				.apply(MotorConstants.SLOW_SIGNALS_CONFIG);
		
		leaderElevatorMotor.configure(leaderElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		followerElevatorMotor.configure(followerElevatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		
		elevatorTarget = elevatorEncoder.getPosition();
		
		time.reset();
		time.start();
	}
	
	private ElevatorFeedforward getElevatorFeedforward() {
		return elevatorFeedforward;
	}
	
	private ArmFeedforward getArmFeedforward() {
		return elevatorEncoder.getPosition() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT || ElevatorConstants.KILL_IT_ALL ? extendedArmFeedForward : retractedArmFeedForward;
		// use just one feedforward for now, if we need 2, use line above
		// return extendedArmFeedForward;
	}
	
	// do something functions
	
	/**
	 * safely set the target angle for the arm
	 * 
	 * @param targetDegrees
	 *            the target angle for the arm in degrees
	 */
	public void setArmTarget(double targetDegrees) {
		// make sure the move can be done safely
		targetDegrees = MathUtil.clamp(targetDegrees, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
		
		armTarget = targetDegrees;
	}
	
	/**
	 * sets the arm target based on the distance to the speaker and the interpolation table
	 * 
	 * @param distance
	 *            the distance to the speaker in meters
	 */
	public void setArmTargetByDistanceExtended(double distance) {
		armTarget = MathUtil.clamp(armAngleBasedOnDistanceExtended.get(distance), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	public void setArmTargetByDistanceRetracted(double distance) {
		armTarget = MathUtil.clamp(armAngleBasedOnDistanceRetracted.get(distance), ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	public Command RaiseElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MAX_HEIGHT));
	}
	
	public Command lowerElevator() {
		return this.runOnce(() -> setElevatorTarget(ElevatorConstants.MIN_HEIGHT));
	}
	
	/**
	 * stows the arm and elevator
	 * 
	 * @return a command to stow the arm and elevator
	 */
	public Command Stow() {
		return this.runOnce(() -> {
			setArmTarget(ArmConstants.STOW_ANGLE);
			setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
		});
	}
	
	/**
	 * sets the velocity for the arm by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the arm in degrees per second
	 */
	public void setArmVelocity(double velocityDegreesPerSec) {
		armTarget = armTarget + velocityDegreesPerSec * dt;
		armTarget = MathUtil.clamp(armTarget, ArmConstants.MIN_ANGLE, ArmConstants.MAX_ANGLE);
	}
	
	/**
	 * safely set the target height for the elevator
	 * 
	 * @param target
	 *            the target height for the elevator in inches
	 */
	public void setElevatorTarget(double target) {
		// make sure the move can be done safely
		// if the target is greater than the max height, set the target to the max
		if (target > ElevatorConstants.MAX_HEIGHT) {
			target = ElevatorConstants.MAX_HEIGHT;
		}
		// if the target is less than the min height, set the target to the min height
		if (target < ElevatorConstants.MIN_HEIGHT) {
			target = ElevatorConstants.MIN_HEIGHT;
		}
		elevatorTarget = target;
	}
	
	public Command SetTargets(ShootingPosition position) {
		return Commands.runOnce(() -> {
			setArmTarget(position.arm_angle());
			setElevatorTarget(position.elevator_target());
		});
	}
	
	public Command SetTargets(Supplier<Double> distance) {
		return Commands.runOnce(() -> {
			// TODO: Remove me!
			System.out.println("Distance is " + distance.get());
			setArmTargetByDistanceRetracted(distance.get());
			setElevatorTarget(ElevatorConstants.MIN_HEIGHT);
		});
	}
	
	public Command SetTargetsAuto(Supplier<Double> distance) {
		return Commands.runOnce(() -> {
			setArmTargetByDistanceExtended(distance.get());
			setElevatorTarget(ElevatorConstants.MAX_HEIGHT);
		});
	}
	
	/**
	 * sets the velocity for the elevator by moving a position setpoint
	 * 
	 * @param velocityDegreesPerSec
	 *            the velocity for the elevator in degrees per second
	 */
	public void setElevatorVelocity(double velocityDegreesPerSec) {
		elevatorTarget = elevatorTarget + velocityDegreesPerSec * dt;
		elevatorTarget = MathUtil.clamp(elevatorTarget, ElevatorConstants.MIN_HEIGHT, ElevatorConstants.MAX_HEIGHT);
		// System.out.println("elevator target in volocity: " + elevatorTarget);
	}
	
	public Command ArmDefaultCommand(Supplier<Double> armVelocity, Supplier<Double> elevatorVelocity) {
		Command command = new RunCommand(() -> {
			setArmVelocity(armVelocity.get());
			setElevatorVelocity(elevatorVelocity.get());
		});
		command.addRequirements(this);
		
		return command;
	}
	
	public Command ToggleBrakeModes() {
		return this.runOnce(() -> {
			SparkBaseConfig newArmConfig = new SparkMaxConfig();
			if (leaderArmMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newArmConfig.idleMode(IdleMode.kCoast);
			} else {
				newArmConfig.idleMode(IdleMode.kBrake);
			}
			leaderArmMotor.configure(newArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			followerArmMotor.configure(newArmConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			
			SparkBaseConfig newElevatorConfig = new SparkMaxConfig();
			if (leaderElevatorMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newElevatorConfig.idleMode(IdleMode.kCoast);
			} else {
				newElevatorConfig.idleMode(IdleMode.kBrake);
			}
			leaderElevatorMotor.configure(newElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			followerElevatorMotor.configure(newElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}
	
	public Command EnableBrakeMode() {
		return this.runOnce(() -> {
			SparkBaseConfig newConfig = new SparkMaxConfig()
					.idleMode(IdleMode.kBrake);
			leaderArmMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			followerArmMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			leaderElevatorMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
			followerElevatorMotor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}
	
	@Override
	public void periodic() {
		dt = time.get() - lastTime;
		lastTime = time.get();
		
		// Arm
		
		// current arm target will be the reference set by the PID controller, based on what is currently safe
		double currentArmTarget = armTarget;
		
		// if the arm is less than the threshold to go over the bumper
		if (currentArmTarget < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
			if (elevatorEncoder.getPosition() < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) { // and the elevator is not
				// extended
				currentArmTarget = ArmConstants.MIN_ABOVE_PASS_ANGLE;
			}
		}
		if (lastAcutalArmTarget != currentArmTarget) {
			ArmFeedforward armFeedFoward = getArmFeedforward();
			double velocity = 0;
			if (Math.abs(currentArmTarget - armAbsoluteEncoder.getPosition()) > ArmConstants.ARM_VELOCITY_DEADBAND) {
				velocity = armAbsoluteEncoder.getVelocity();
			}
			double armFeedFowardValue = armFeedFoward.calculate(Units.degreesToRadians(currentArmTarget), velocity);
			// System.out.println("arm feed foward: " + armFeedFowardValue);
			
			armPIDController.setReference(currentArmTarget, SparkMax.ControlType.kPosition, 0, armFeedFowardValue, ArbFFUnits.kVoltage);
			lastAcutalArmTarget = currentArmTarget;
		}
		if (!ElevatorConstants.KILL_IT_ALL) {
			// Elevator
			// current elevator target will be the reference set by the PID controller, based on what is currently safe
			double currentElevatorTarget = elevatorTarget;
			// if the arm is less than the threshold to go over the bumper, than the elevator needs to stay on its current side of the bumper
			if (armAbsoluteEncoder.getPosition() < ArmConstants.MIN_ABOVE_PASS_ANGLE) {
				if (currentElevatorTarget < ElevatorConstants.MIN_ABOVE_PASS_HEIGHT && elevatorEncoder.getPosition() > ElevatorConstants.MIN_ABOVE_PASS_HEIGHT) {
					currentElevatorTarget = ElevatorConstants.MIN_ABOVE_PASS_HEIGHT;
				}
			}
			
			if (currentElevatorTarget != lastAcutalElevatorTarget) {
				double elevatorFeedFowardValue = getElevatorFeedforward().calculate(elevatorEncoder.getVelocity());
				elevatorPIDController.setReference(currentElevatorTarget, SparkMax.ControlType.kPosition, 0, elevatorFeedFowardValue, ArbFFUnits.kVoltage);
				lastAcutalElevatorTarget = currentElevatorTarget;
			}
			
			motorTab.update();
		}
	}
	
	// get info functions
	public double getArmAbsoluteEncoderPosition() {
		return armAbsoluteEncoder.getPosition();
	}
	
	public double getElevatorAbsoluteEncoderPosition() {
		return elevatorEncoder.getPosition();
	}
	
	public MotorTab getMotorTab() {
		return motorTab;
	}
	
	public boolean areArmAndElevatorAtTarget() {
		return (Math.abs(armTarget - armAbsoluteEncoder.getPosition()) < ArmConstants.ARM_AT_TARGET_DEADBAND) && (Math.abs(elevatorTarget - elevatorEncoder.getPosition()) < ElevatorConstants.ELEVATOR_AT_TARGET_DEADBAND || ElevatorConstants.KILL_IT_ALL);
	}
}
