package com.michaelgrundvig.frc.runtime.subsystems.vision;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.VisorFrame;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.Test;

/** A camera on Visor, read from frames as its coprocessor would send them. */
class CameraIOVisorTest {
  private static final long SECOND = 1_000_000_000L;

  /** A tenth of a second before now: a frame just taken. */
  private static final long RECENT = 10 * SECOND - SECOND / 10;

  private final VisorFeed feed = VisorFeed.inMemory();
  private final AtomicLong now = new AtomicLong(10 * SECOND);
  private final CameraIOVisor front = new CameraIOVisor(feed, 0, now::get);
  private final CameraIOVisor back = new CameraIOVisor(feed, 1, now::get);

  /** A frame from a camera, captured at a time, with a tag solved 3 m ahead and one unsolved. */
  private static VisorFrame frame(int camera, long sequence, long capturedNanos, boolean synced) {
    VisorFrame frame = new VisorFrame();
    frame.setHeader(sequence, 1600, 1304, capturedNanos, camera, synced, false);
    // Corners in Visor's order: top left, top right, bottom right, bottom left.
    int solved = frame.addTag(7, 0, 80, new float[] {10, 20, 30, 20, 30, 40, 10, 40});
    CameraToTag ahead = new CameraToTag();
    ahead.set(3, 0.5, 0.25, 0, 0, 0, 1);
    CameraToTag other = new CameraToTag();
    other.set(3, 0.5, 0.25, 0.1, 0, 0, 0.995);
    frame.setPoses(solved, ahead, 0.1f, other, 0.5f);
    frame.addTag(8, 0, 80, new float[8]);
    return frame;
  }

  private static CameraIO.CameraIOInputs read(CameraIOVisor camera) {
    CameraIO.CameraIOInputs inputs = new CameraIO.CameraIOInputs();
    camera.updateInputs(inputs);
    return inputs;
  }

  @Test
  void eachCameraGetsItsOwnFramesWithTheirSolvedTags() {
    feed.offer(frame(0, 0, RECENT, true));
    feed.offer(frame(1, 0, RECENT + 1, true));
    feed.offer(frame(0, 1, RECENT + 2, true));

    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames)
        .extracting(CameraIO.Frame::timestampNanos)
        .containsExactly(RECENT, RECENT + 2);
    assertThat(inputs.frames).allSatisfy(frame -> assertThat(frame.hasMultiTag()).isFalse());
    // The unsolved tag is left out: it has no pose to use.
    assertThat(inputs.frames).extracting(CameraIO.Frame::tagCount).containsExactly(1, 1);
    CameraIO.SeenTag tag = inputs.tags[0];
    assertThat(tag.id()).isEqualTo(7);
    // PhotonVision's order, as the rest of the robot code reads corners: Visor's reversed.
    assertThat(new double[] {tag.corner0X(), tag.corner0Y(), tag.corner3X(), tag.corner3Y()})
        .containsExactly(10, 40, 10, 20);
    assertThat(tag.bestCameraToTag().getX()).isCloseTo(3, within(1e-6));
    assertThat(tag.bestCameraToTag().getY()).isCloseTo(0.5, within(1e-6));
    assertThat(tag.ambiguity()).isCloseTo(0.2, within(1e-6));
    assertThat(inputs.connected).isTrue();
    assertThat(inputs.error).isEmpty();

    assertThat(read(back).frames).hasSize(1);
    assertThat(read(front).frames).as("taken already").isEmpty();
  }

  @Test
  void framesFromAClockNotYetSyncedAreCountedAndLeftOut() {
    feed.offer(frame(0, 0, RECENT, false));
    feed.offer(frame(0, 1, RECENT, true));
    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames).hasSize(1);
    assertThat(inputs.skippedFrames).isEqualTo(1);
  }

  @Test
  void aCameraThatStopsSendingIsDisconnected() {
    assertThat(read(front).connected).as("before any frame").isFalse();
    feed.offer(frame(0, 0, RECENT, true));
    assertThat(read(front).connected).isTrue();
    now.addAndGet(2 * SECOND);
    assertThat(read(front).connected).as("nothing for two seconds").isFalse();
  }

  @Test
  void aCoprocessorOnAnotherVersionIsNamed() {
    VisorFrame stale = frame(0, 0, RECENT, true);
    stale.setVersionId(20251225);
    feed.offer(stale);
    CameraIO.CameraIOInputs inputs = read(front);
    assertThat(inputs.frames).isEmpty();
    assertThat(inputs.error).contains("20251225").contains("deploy");
    assertThat(read(front).error).as("said once, as it happened").isEmpty();
  }
}
