package frc.robot.commands.controls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class HapticController {
	private final XboxController controller;
	
	public HapticController(XboxController controller) {
		this.controller = controller;
	}
	
	/**
	 * Trigger rumble on the controller for a specified amount of time.
	 * 
	 * @param type
	 *            rumble type
	 * @param strength
	 *            rumble strength (0 to 1)
	 * @param length
	 *            rumble time (seconds)
	 * @return
	 *         command group (synchronous)
	 */
	public Command HapticTap(RumbleType type, double strength, double length) {
		return Commands.runOnce(() -> controller.setRumble(type, strength))
				.andThen(Commands.waitSeconds(length))
				.finallyDo(() -> controller.setRumble(RumbleType.kBothRumble, 0))
				.ignoringDisable(true);
	}
}
