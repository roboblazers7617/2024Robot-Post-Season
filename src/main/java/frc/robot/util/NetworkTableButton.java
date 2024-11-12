// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Creates a button on the specified network table that will run a command when clicked */
public class NetworkTableButton {
	private final NetworkTableEntry buttonEntry;
	
	/**
	 * Creates a button on the specified network table that will run a command when clicked
	 * 
	 * @param networkTablePath
	 *            The path to the network table to create the button on
	 * @param name
	 *            The name of the button
	 * @param command
	 *            The command to run when the button is clicked
	 * @param runsWhileDisabled
	 *            Whether the command should run while the robot is disabled
	 */
	public NetworkTableButton(String networkTablePath, String name, Command command, boolean runsWhileDisabled) {
		NetworkTable networkTable = NetworkTableInstance.getDefault().getTable(networkTablePath);
		
		// configure the trigger
		buttonEntry = networkTable.getEntry(name);
		buttonEntry.setBoolean(false);
		new Trigger(() -> buttonEntry.getBoolean(false))
				.and(() -> runsWhileDisabled || DriverStation.isEnabled())
				.onTrue(new InstantCommand(() -> buttonEntry.setBoolean(false)).andThen(command));
	}
	
	// Overloaded constructor?
	// check the status of the button?
}
