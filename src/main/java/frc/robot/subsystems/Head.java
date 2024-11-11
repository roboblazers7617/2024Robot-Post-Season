// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;

/**
 * Shooter and intake.
 */
public class Head extends SubsystemBase {
	// Shooter
	private final CANSparkMax shooterMotorBottom = new CANSparkMax(ShooterConstants.MOTOR_BOTTOM_CAN_ID, MotorType.kBrushless);
	private final CANSparkMax shooterMotorTop = new CANSparkMax(ShooterConstants.MOTOR_TOP_CAN_ID, MotorType.kBrushless);
	private final RelativeEncoder shooterEncoderBottom = shooterMotorBottom.getEncoder();
	private final RelativeEncoder shooterEncoderTop = shooterMotorTop.getEncoder();
	private final SparkPIDController shooterControllerBottom = shooterMotorBottom.getPIDController();
	private final SparkPIDController shooterControllerTop = shooterMotorTop.getPIDController();
	
	private final CANSparkMax intakeMotor = new CANSparkMax(IntakeConstants.MOTOR_CAN_ID, MotorType.kBrushless);
	/** Sensor to tell whether the note is aligned to shoot. */
	private final DigitalInput isNoteInShootPosition = new DigitalInput(IntakeConstants.NOTE_SENSOR_DIO);
	/** Sensor to tell whether the note is in the intake. Used to slow down the intake so the note is aligned correctly. */
	private final DigitalInput isNoteInIntake = new DigitalInput(IntakeConstants.NOTE_ALIGNMENT_SENSOR_DIO);
	
	/** Shooter speed in RPM. */
	private double shooterSetPoint = 0;
	
	/** Creates a new Head. */
	public Head() {
		// Shooter motor
		shooterMotorBottom.restoreFactoryDefaults();
		shooterMotorTop.restoreFactoryDefaults();
		
		shooterMotorBottom.setIdleMode(IdleMode.kCoast);
		shooterMotorTop.setIdleMode(IdleMode.kCoast);
		
		shooterMotorBottom.setInverted(false);
		shooterMotorTop.setInverted(false);
		
		shooterMotorBottom.setSmartCurrentLimit(40);
		shooterMotorTop.setSmartCurrentLimit(40);
		
		// Intake motor
		intakeMotor.restoreFactoryDefaults();
		intakeMotor.setIdleMode(IdleMode.kBrake);
		intakeMotor.setInverted(true);
		intakeMotor.setSmartCurrentLimit(40);
		
		// Shooter controller
		shooterControllerBottom.setP(ShooterConstants.BOTTOM_kP);
		shooterControllerTop.setP(ShooterConstants.TOP_kP);
		shooterControllerBottom.setI(ShooterConstants.BOTTOM_kI);
		shooterControllerTop.setI(ShooterConstants.TOP_kI);
		shooterControllerBottom.setD(ShooterConstants.BOTTOM_kD);
		shooterControllerTop.setD(ShooterConstants.TOP_kD);
		shooterControllerBottom.setOutputRange(ShooterConstants.BOTTOM_kMinOutput, ShooterConstants.BOTTOM_kMaxOutput);
		shooterControllerTop.setOutputRange(ShooterConstants.TOP_kMinOutput, ShooterConstants.TOP_kMaxOutput);
		
		// Burn motor controller configuration to flash
		Timer.delay(0.005);
		intakeMotor.burnFlash();
		Timer.delay(0.005);
		shooterMotorBottom.burnFlash();
		Timer.delay(0.005);
		shooterMotorTop.burnFlash();
	}
	
	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
	
	/**
	 * Sets the intake speed.
	 * 
	 * @param intakeSpeed
	 *            new intake speed [-1.0,1.0]
	 */
	private void setIntakeSpeed(double intakeSpeed) {
		intakeMotor.set(intakeSpeed);
	}
	
	/**
	 * Starts intaking.
	 * 
	 * @return
	 *         command to run
	 */
	public Command StartIntake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
		}, this);
	}
	
	/**
	 * Starts outtaking.
	 * 
	 * @return
	 *         command to run
	 */
	public Command StartOutake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTAKE_SPEED);
		}, this);
	}
	
	/**
	 * Stops the intake.
	 * 
	 * @return
	 *         command to run
	 */
	public Command StopIntake() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(0);
		}, this);
	}
	
	/**
	 * Intakes a piece and stops.
	 * 
	 * @return
	 *         command to run
	 */
	public Command IntakePiece() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.INTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> isNoteWithinAlignmentSensor()))
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.ALIGMNMENT_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				.andThen(() -> {
					setIntakeSpeed(0);
				});
	}
	
	/**
	 * Outtakes a piece and stops.
	 * 
	 * @return
	 *         command to run
	 */
	public Command OutakePiece() {
		return Commands.runOnce(() -> {
			setIntakeSpeed(IntakeConstants.OUTAKE_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> !isNoteWithinSensor()))
				.andThen(Commands.waitSeconds(3))
				.finallyDo(() -> {
					setIntakeSpeed(0);
				});
	}
	
	/**
	 * Sets the shooter speed.
	 * 
	 * @param rpm
	 *            new speed in RPM
	 */
	private void setShooterSpeed(double rpm) {
		shooterSetPoint = rpm;
		shooterControllerBottom.setReference(shooterSetPoint, ControlType.kVelocity);
		shooterControllerTop.setReference(shooterSetPoint, ControlType.kVelocity);
	}
	
	/**
	 * Stops the shooter.
	 * 
	 * @implNote
	 *           This removes power from the motors, allowing them to spin down freely, so the shooter will take some time to spin down after calling this method.
	 */
	public void stopShooter() {
		shooterSetPoint = 0.0;
		shooterMotorBottom.setVoltage(0);
		shooterMotorTop.setVoltage(0);
	}
	
	/**
	 * Gets the speed of the bottom shooter motor.
	 * 
	 * @return
	 *         motor velocity
	 */
	public double getShooterBottomSpeed() {
		return shooterEncoderBottom.getVelocity();
	}
	
	/**
	 * Gets the speed of the top shooter motor.
	 * 
	 * @return
	 *         motor velocity
	 */
	public double getShooterTopSpeed() {
		return shooterEncoderTop.getVelocity();
	}
	
	/**
	 * Gets the current setpoint of the shooter.
	 * 
	 * @return
	 *         setpoint in RPM
	 */
	public double getShooterSetPoint() {
		return shooterSetPoint;
	}
	
	/**
	 * Spins up the shooter to a given speed.
	 * 
	 * @param rpm
	 *            new speed in RPM
	 * @return
	 *         command to run
	 */
	public Command SpinUpShooter(double rpm) {
		return Commands.runOnce(() -> {
			setShooterSpeed(rpm);
		}, this);
	}
	
	/**
	 * Spins up the shooter to shoot from a certain shooting position.
	 * 
	 * @param position
	 *            position to shoot from
	 * @return
	 *         command to run
	 */
	public Command SpinUpShooter(ShootingPosition position) {
		return SpinUpShooter(position.rpm());
	}
	
	/**
	 * Spins down the shooter.
	 * 
	 * @return
	 *         command to run
	 */
	public Command SpinDownShooter() {
		return Commands.runOnce(() -> {
			shooterSetPoint = 0.0;
			shooterMotorBottom.setVoltage(0);
			shooterMotorTop.setVoltage(0);
		}, this);
	}
	
	/**
	 * Checks if the shooter speed is within the range to shoot.
	 * 
	 * @return
	 *         Is the shooter ready to shoot?
	 */
	public boolean isReadyToShoot() {
		// @formatter:off
		return (
				(getShooterBottomSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM)) &&
				(getShooterBottomSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM))
			) && (
				(getShooterTopSpeed() >= (shooterSetPoint * ShooterConstants.VELOCITY_MINIMUM)) &&
				(getShooterTopSpeed() <= (shooterSetPoint * ShooterConstants.VELOCITY_MAXIMUM))
			);
		// @formatter:on
	}
	
	/**
	 * Shoots and then stops the shooter.
	 * 
	 * @return
	 *         command to run
	 */
	public Command Shoot() {
		return Shoot(true);
	}
	
	/**
	 * Shoots.
	 * 
	 * @param stopShooter
	 *            if true, shooter is spun down after shoot
	 * @return
	 *         command to run
	 */
	public Command Shoot(boolean stopShooter) {
		return Commands.waitUntil(() -> isReadyToShoot())
				.andThen(Commands.runOnce(() -> {
					setIntakeSpeed(IntakeConstants.FEEDER_SPEED);
				}))
				.andThen(Commands.waitUntil(() -> isNoteWithinSensor()))
				.andThen(Commands.waitUntil(() -> !isNoteWithinSensor()))
				.andThen(Commands.waitSeconds(0.2))
				.andThen(Commands.either(SpinDownShooter().andThen(() -> setIntakeSpeed(0.0)), Commands.none(), () -> stopShooter));
	}
	
	/**
	 * Gets the value of the shooting position sensor.
	 * 
	 * @return
	 *         Is the note in the position to shoot?
	 */
	public boolean isNoteWithinSensor() {
		return !isNoteInShootPosition.get();
	}
	
	/**
	 * Gets the value of the alignment sensor.
	 * 
	 * @return
	 *         Is the note in the alignment sensor?
	 */
	public boolean isNoteWithinAlignmentSensor() {
		return !isNoteInIntake.get();
	}
	
	/**
	 * Toggles brake mode on the intake motor.
	 * 
	 * @return
	 *         command to run
	 */
	public Command ToggleBreakModes() {
		return new InstantCommand(() -> {
			if (intakeMotor.getIdleMode() == IdleMode.kBrake) {
				intakeMotor.setIdleMode(IdleMode.kCoast);
			} else {
				intakeMotor.setIdleMode(IdleMode.kBrake);
			}
		}).ignoringDisable(true);
	}
	
	/**
	 * Enables brake mode on the intake motor.
	 * 
	 * @return
	 *         command to run
	 */
	public Command EnableBrakeMode() {
		return new InstantCommand(() -> {
			intakeMotor.setIdleMode(IdleMode.kBrake);
		});
	}
}
