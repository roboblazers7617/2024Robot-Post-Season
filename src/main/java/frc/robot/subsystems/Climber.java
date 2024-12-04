// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
	private final SparkMax rightClimber = new SparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder rightClimberEncoder;
	
	private final SparkMax leftClimber = new SparkMax(Constants.ClimberConstants.LEFT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder leftClimberEncoder;
	
	/** Creates a new Climber. */
	public Climber() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake);
		
		leftClimber.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightClimber.configure(motorConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		
		leftClimberEncoder = leftClimber.getEncoder();
		rightClimberEncoder = rightClimber.getEncoder();
		leftClimberEncoder.setPosition(0.0);
		rightClimberEncoder.setPosition(0.0);
		
		// balanceController.setSetpoint(0.0);
	}
	
	public void setSpeed(double leftSpeed, double rightSpeed) {
		// leftClimber.set(speed);
		rightClimber.set(rightSpeed);
		leftClimber.set(leftSpeed);
	}
	
	public double getSpeedRight() {
		return rightClimberEncoder.getVelocity();
	}
	
	public double getSpeedLeft() {
		return leftClimberEncoder.getVelocity();
	}
	
	public void setSpeedLeft(double leftSpeed) {
		leftClimber.set(leftSpeed);
	}
	
	public void setSpeedRight(double rightSpeed) {
		rightClimber.set(rightSpeed);
	}
	
	// public void resetClimberEncoders() {
	// leftClimberEncoder.setPosition(0.0);
	// rightClimberEncoder.setPosition(0.0);
	// }
	
	public double getPositionRightMotor() {
		return rightClimberEncoder.getPosition();
	}
	
	public double getPositionLeftMotor() {
		return leftClimberEncoder.getPosition();
	}
	
	@Override
	public void periodic() {}
	
	public SparkMax[] getMotors() {
		return new SparkMax[] { leftClimber, rightClimber };
	}
}
