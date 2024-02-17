package frc.robot.shuffleboard;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Head;
import frc.robot.util.TunableNumber;

public class HeadTab extends ShuffleboardTabBase {
	private final Head head;
	
	private final BooleanPublisher noteWithinHeadPublisher;
	private final BooleanPublisher noteAlignedPublisher;
	// private final BooleanPublisher noteInShooterPublisher;
	private final BooleanPublisher noteAcquiredPublisher;
	
	private final DoublePublisher shooterBottomSpeedPublisher;
	private final DoublePublisher shooterTopSpeedPublisher;
	private final DoublePublisher shooterSetPointPublisher;
	
	private final BooleanPublisher readyToShootPublisher;
	
	private TunableNumber shootingPosition;
	private final DoublePublisher shooterSpeedAtPositionPublisher;
	private TunableNumber shootingPositionSpeedTuning;
	
	public HeadTab(Head head) {
		this.head = head;
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		
		NetworkTable networkTable = inst.getTable("logging/Head");
		
		noteWithinHeadPublisher = networkTable.getBooleanTopic("Note Within Head").publish();
		noteAlignedPublisher = networkTable.getBooleanTopic("Note Aligned").publish();
		// noteInShooterPublisher = networkTable.getBooleanTopic("Note in Shooter").publish();
		noteAcquiredPublisher = networkTable.getBooleanTopic("Note Acquired").publish();
		
		shooterBottomSpeedPublisher = networkTable.getDoubleTopic("Shooter Bottom Speed").publish();
		shooterTopSpeedPublisher = networkTable.getDoubleTopic("Shooter Top Speed").publish();
		shooterSetPointPublisher = networkTable.getDoubleTopic("Shooter Setpoint").publish();
		
		readyToShootPublisher = networkTable.getBooleanTopic("Ready To Shoot").publish();
		
		shooterSpeedAtPositionPublisher = networkTable.getDoubleTopic("Shooter Speed At Position").publish();
		
		shootingPosition = new TunableNumber("Head", "Shooting Position", 0);
		shootingPositionSpeedTuning = new TunableNumber("Head", "Shooting Position Speed Tuning", 0);
	}
	
	@Override
	public void update() {
		noteWithinHeadPublisher.set(head.isNoteWithinHead());
		noteAlignedPublisher.set(head.isNoteAligned());
		// noteInShooterPublisher.set(head.isNoteInShooter());
		noteAcquiredPublisher.set(head.isNoteAcquired());
		
		shooterBottomSpeedPublisher.set(head.getShooterBottomSpeed());
		shooterTopSpeedPublisher.set(head.getShooterTopSpeed());
		shooterSetPointPublisher.set(head.getShooterSetPoint());
		
		readyToShootPublisher.set(head.isReadyToShoot());
		
		shooterSpeedAtPositionPublisher.set(head.getShooterSpeedAtPosition(shootingPosition.get()));
	}
	
	@Override
	public void activateShuffleboard() {
		ShuffleboardTab tab = Shuffleboard.getTab("Head");
		tab.add("Intake from Ground", head.intakePiece(false));
		tab.add("Intake from Source", head.intakePiece(true));
		tab.add("Outake", head.outakePiece());
		
		tab.add("Spin Up", head.spinUpShooter(shootingPosition.get()));
		tab.add("Spin Down", head.spinDownShooter());
		tab.add("Shoot", head.shoot());
		tab.add("Set Speed at Position (tuning)", new InstantCommand(() -> head.setShooterSpeedAtPosition(shootingPosition.get(), shootingPositionSpeedTuning.get())));
	}
	
	@Override
	public String getNetworkTable() {
		return "Head";
	}
}
