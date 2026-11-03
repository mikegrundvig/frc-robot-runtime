package com.michaelgrundvig.frc.runtime.subsystems.vision;

import java.util.function.LongSupplier;
import org.wpilib.system.RobotController;

/**
 * A camera running Visor, the AprilTag vision program on a coprocessor, or Visor's path through the
 * simulator. It reads every frame since the last loop, each tag with its corners and both of its
 * candidate poses, stamped with its capture time on the robot's clock.
 *
 * <p>Visor doesn't solve tags together, so a frame never has a multi-tag solution: the pipeline
 * solves with the gyro's heading, or tag by tag.
 */
public class CameraIOVisor implements CameraIO {
  private final VisorFeed feed;
  private final int index;
  private final LongSupplier clock;

  /**
   * @param feed the frames from every camera
   * @param index the camera's index in Visor's configuration
   */
  public CameraIOVisor(VisorFeed feed, int index) {
    this(feed, index, RobotController::getTime);
  }

  CameraIOVisor(VisorFeed feed, int index, LongSupplier clock) {
    this.feed = feed;
    this.index = index;
    this.clock = clock;
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    feed.read(index, inputs, clock.getAsLong());
  }
}
