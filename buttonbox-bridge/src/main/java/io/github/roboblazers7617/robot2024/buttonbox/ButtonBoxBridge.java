package io.github.roboblazers7617.robot2024.buttonbox;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;

import java.util.Vector;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;

import javax.sound.midi.MidiDevice;
import javax.sound.midi.MidiDevice.Info;
import javax.sound.midi.MidiSystem;
import javax.sound.midi.MidiUnavailableException;
import javax.sound.midi.Synthesizer;
import javax.sound.midi.ShortMessage;

import io.github.roboblazers7617.buttonbox.ButtonBoxClient;
import io.github.roboblazers7617.buttonbox.controls.TestControlMIDI;
import io.github.roboblazers7617.buttonbox.controls.ButtonMIDI;
import io.github.roboblazers7617.buttonbox.midi.MIDIDevice;
import io.github.roboblazers7617.buttonbox.midi.MIDIAddress;

/**
 * Bridge program to connect the 2024Robot ButtonBox hardware to NetworkTables.
 */
public class ButtonBoxBridge {
	public static void main(String[] args) throws IOException, MidiUnavailableException {
		NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
		WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
		WPIMathJNI.Helper.setExtractOnStaticLoad(false);
		
		CombinedRuntimeLoader.loadLibraries(ButtonBoxBridge.class, "wpiutiljni", "wpimathjni", "ntcorejni");
		
		new ButtonBoxBridge().run();
	}
	
	public void run() throws MidiUnavailableException {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		inst.startClient4("ButtonBox Bridge");
		inst.setServer("localhost"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
		inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS
		
		// Obtain information about all the installed synthesizers.
		Vector<Info> synthInfos = new Vector<>();
		MidiDevice device;
		MidiDevice.Info[] infos = MidiSystem.getMidiDeviceInfo();
		for (int i = 0; i < infos.length; i++) {
			try {
				device = MidiSystem.getMidiDevice(infos[i]);
				if (device instanceof Synthesizer) {
					synthInfos.add(infos[i]);
				}
				System.out.printf("%d: %s %s\n", i, infos[i].getClass(), infos[i].getName());
			} catch (MidiUnavailableException ex) {
				// Handle or throw exception...
			}
		}
		
		Synthesizer synth = MidiSystem.getSynthesizer();
		synth.open();
		MidiDevice rxDevice = MidiSystem.getMidiDevice(infos[18]);
		MidiDevice txDevice = MidiSystem.getMidiDevice(infos[20]);
		rxDevice.open();
		txDevice.open();
		MIDIDevice midiDevice = new MIDIDevice(rxDevice, txDevice);
		
		ButtonBoxClient client = new ButtonBoxClient(inst);
		
		configureControls(client, midiDevice);
		
		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException ex) {
				System.out.println("interrupted");
				return;
			}
			client.periodic();
		}
	}
	
	/**
	 * Configures the ButtonBox controls.
	 */
	private void configureControls(ButtonBoxClient client, MIDIDevice midiDevice) {
		// Test controls
		client.addControl(new TestControlMIDI("Test Control", new MIDIAddress(midiDevice, ShortMessage.CONTROL_CHANGE, 1, 0)));
		client.addControl(new ButtonMIDI("Test Button", new MIDIAddress(midiDevice, ShortMessage.NOTE_ON, 1, 0)));
		
		// Head controls
		client.addControl(new ButtonMIDI("Shoot Button", new MIDIAddress(midiDevice, ShortMessage.NOTE_ON, 0, 0)));
		client.addControl(new ButtonMIDI("Intake Button Ground", new MIDIAddress(midiDevice, ShortMessage.NOTE_ON, 0, 1)));
		client.addControl(new ButtonMIDI("Intake Button Source", new MIDIAddress(midiDevice, ShortMessage.NOTE_ON, 0, 2)));
		client.addControl(new ButtonMIDI("Outake Button", new MIDIAddress(midiDevice, ShortMessage.NOTE_ON, 0, 3)));
	}
}
