package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalSource;
import java.util.function.BiConsumer;

public class WaitUntilInterrupt extends Command {
	/**
	 * Set to true after the callback is called.
	 */
	private boolean hasFinished = false;
	private final AsynchronousInterrupt interrupt;
	
	/**
	 * Creates a new WaitUntilInterrupt command that finishes when the interrupt is triggered.
	 *
	 * @param source
	 *            digital source to await an interrupt on
	 * @param callback
	 *            callback to call on interrupt
	 * @param risingEdge
	 *            trigger on the rising edge
	 * @param fallingEdge
	 *            trigger on the falling edge
	 */
	public WaitUntilInterrupt(DigitalSource source, BiConsumer<Boolean, Boolean> callback, boolean risingEdge, boolean fallingEdge) {
		interrupt = new AsynchronousInterrupt(source, (rising, falling) -> {
			callback.accept(rising, falling);
			hasFinished = true;
		});
		interrupt.setInterruptEdges(risingEdge, fallingEdge);
	}
	
	@Override
	public void initialize() {
		hasFinished = false;
		interrupt.enable();
	}
	
	@Override
	public void end(boolean interrupted) {
		interrupt.disable();
	}
	
	@Override
	public boolean isFinished() {
		return hasFinished;
	}
}