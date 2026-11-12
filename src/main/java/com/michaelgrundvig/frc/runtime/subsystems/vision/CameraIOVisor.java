package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.visor.VisorStream;
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
  private final VisorFeed.Reader reader;
  private final LongSupplier clock;

  /**
   * @param feed the frames from every camera: from now on, it keeps this index's
   * @param index the camera's index in Visor's configuration
   */
  public CameraIOVisor(VisorFeed feed, int index) {
    this(feed, index, RobotController::getTime);
  }

  CameraIOVisor(VisorFeed feed, int index, LongSupplier clock) {
    this.feed = feed;
    this.reader = feed.reader(index);
    this.clock = clock;
  }

  /**
   * Where Visor serves a camera's live view: Visor's own address for it, so where it streams has
   * one owner.
   *
   * @param board the coprocessor the camera is plugged into, by its address
   * @param camera the camera's name
   */
  public static String liveView(String board, String camera) {
    return VisorStream.url(board, camera);
  }

  @Override
  public void updateInputs(CameraIOInputs inputs) {
    feed.read(reader, inputs, clock.getAsLong());
  }
}
