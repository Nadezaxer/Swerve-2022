package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public final class UpdateManager {

    public final List<Updatable> updatables = new ArrayList<>();

	@FunctionalInterface
	public interface Updatable {
		void Update( double time, double dt );
	}

	private double lastTimestamp = 0.0;

	private final Notifier updaterThread = new Notifier( () -> {
		synchronized ( UpdateManager.this ) {
			final double timestamp = Timer.getFPGATimestamp();
			final double dt = timestamp - lastTimestamp;
			lastTimestamp = timestamp;
			updatables.forEach( s -> s.Update( timestamp, dt ) );
		}
	});

	public UpdateManager(Updatable... updatables) {
		this( Arrays.asList( updatables ) );
	}

	public UpdateManager(List<Updatable> updatables) {
		this.updatables.addAll( updatables );
	}

	public void StartLoop( double period ) {
		updaterThread.startPeriodic( period );
	}

	public void StopLoop() {
		updaterThread.stop();
	}
}
