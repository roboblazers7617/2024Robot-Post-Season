package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Constants.ShootingConstants.ShootingPosition;
import frc.robot.commands.controls.HapticController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Head;

public class MechanismCommands {
	private final HapticController driverHapticController;
	private final HapticController operatorHapticController;
	private final Arm arm;
	private final Head head;
	
	public MechanismCommands(Arm arm, Head head, HapticController driverHapticController, HapticController operatorHapticController) {
		this.arm = arm;
		this.head = head;
		this.driverHapticController = driverHapticController;
		this.operatorHapticController = operatorHapticController;
	}
	
	/**
	 * Will finish after piece has been intaken.
	 * 
	 * @param stopShooter
	 *            Should the shooter be stopped before intaking a piece?
	 * @param ground
	 *            Should the piece be intaken from the ground or the source?
	 * @return
	 *         Command
	 */
	public Command IntakePiece(boolean stopShooter, boolean ground) {
		return Commands.either(head.SpinDownShooter(), Commands.none(), () -> stopShooter)
				.andThen(Commands.either(Commands.runOnce(() -> arm.setArmTarget(ArmConstants.FLOOR_PICKUP)), Commands.runOnce(() -> arm.setArmTarget(ArmConstants.SOURCE_ANGLE)), () -> ground))
				.andThen(() -> arm.setElevatorTarget(ElevatorConstants.MAX_HEIGHT))
				.andThen(head.IntakePiece())
				.andThen(new ScheduleCommand(driverHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(operatorHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * Will finish after the piece has been intaken.
	 * 
	 * @param stopShooter
	 *            Should the shooter be stopped before intaking a piece?
	 * @return
	 *         Command
	 */
	public Command IntakeSource(boolean stopShooter) {
		return IntakePiece(stopShooter, false);
	}
	
	/**
	 * Will finish after the piece has been intaken.
	 * 
	 * @return
	 *         Command
	 */
	public Command IntakeSource() {
		return IntakeSource(true);
	}
	
	/**
	 * Will finish after the piece has been intaken.
	 * 
	 * @param stopShooter
	 *            Should the shooter be stopped before intaking a piece?
	 * @return
	 *         Command
	 */
	public Command IntakeGround(boolean stopShooter) {
		return IntakePiece(stopShooter, true);
	}
	
	/**
	 * Will finish after the piece has been intaken.
	 * 
	 * @return
	 *         Command
	 */
	public Command IntakeGround() {
		return IntakeGround(true);
	}
	
	/**
	 * cancel shoot and intake and stow
	 * 
	 * @return
	 */
	public Command StowStopIntakeAndShooter() {
		return arm.Stow()
				.andThen(head.StopIntake())
				.andThen(head.SpinDownShooter());
	}
	
	// THIS ISNT CODE DUPLICATION IT DOES A FUNDAMENTALLY DIFFERENT THING!!!!!
	// TODO: I see that this is different, but when would it be used? Should be renamed to better describe
	// what it does. Why doesn't it stop the shooter? Will it be used for auto?
	public Command AutoStowAndStopIntake() {
		return arm.Stow()
				.andThen(head.StopIntake());
	}
	
	/**
	 * will finish when ready
	 * 
	 * @param position
	 */
	public Command PrepareShoot(ShootingPosition position) {
		return arm.SetTargets(position)
				.andThen(head.SpinUpShooter(position))
				.andThen(new ScheduleCommand(operatorHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	/**
	 * will finish when ready
	 * 
	 * @param distance
	 */
	public Command PrepareShoot(Supplier<Double> distance) {
		return arm.SetTargets(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.VARIABLE_DISTANCE_SHOT))
				.andThen(new ScheduleCommand(operatorHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public Command AutonomousPrepareShoot(Supplier<Double> distance) {
		return arm.SetTargetsAuto(distance)
				.andThen(head.SpinUpShooter(ShootingConstants.AUTO_SHOOT_SPEED));
	}
	
	/**
	 * will finish after piece has been shot
	 * 
	 * @return Command
	 */
	public Command Shoot() {
		return Shoot(true)
				.andThen(new ScheduleCommand(driverHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)))
				.andThen(new ScheduleCommand(operatorHapticController.HapticTap(RumbleType.kBothRumble, 0.3, 0.3)));
	}
	
	public Command Shoot(boolean stopShooter) {
		return Commands.waitUntil(() -> arm.areArmAndElevatorAtTarget())
				.andThen(head.Shoot(stopShooter));
	}
	
	public Command AutonomousShoot(ShootingPosition position) {
		return PrepareShoot(position)
				.andThen(Shoot(false));
	}
	
	public Command AutonomousShoot(Drivetrain drivetrain) {
		return AutonomousPrepareShoot(() -> drivetrain.getDistanceToSpeaker())
				.andThen(Shoot(false));
	}
}
