package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.runtime.lib.health.FrameNumbers;
import com.michaelgrundvig.frc.visor.CameraToTag;
import com.michaelgrundvig.frc.visor.Candidate;
import com.michaelgrundvig.frc.visor.VisorFrame;
import com.michaelgrundvig.frc.visor.VisorReceiver;
import com.michaelgrundvig.frc.visor.VisorVersion;
import com.michaelgrundvig.frc.world.robot.CameraConfig;
import java.net.InetAddress;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

/**
 * Every camera's frames from Visor, the AprilTag vision program on the coprocessors, sorted by
 * camera for each one's {@link CameraIOVisor}. One receiver serves them all: the coprocessors send
 * every camera's frames to the same port, each frame naming its camera by its index.
 *
 * <p>A frame is checked before it's believed. It's kept for a camera only when a reader reads that
 * index, at the camera's configured size, when its number wasn't seen before, when it comes from
 * where the camera can be (from one of the robot's listed boards, and, once a board has reported
 * the camera plugged in, from that board), and when it names a configuration the robot gave the
 * board it came from. Everything else is left out, counted by why ({@link Refusal}) under {@code
 * Vision/Visor/Refused/}. So a board still running a camera the robot no longer has, or anything
 * else on the network, can't put frames into localization, nor fill the robot's memory with frames
 * nobody reads. At most {@link #WAITING_FRAMES} wait for a camera between its reads.
 *
 * <p>Where the robot doesn't know, it believes: a robot that lists no boards takes frames from
 * anyone, a camera no board has reported yet takes them from any listed board, and a board listed
 * by a name not yet looked up can't be told from a stranger, so a frame from an address no found
 * board has is taken while any name is still unfound. A frame is refused for where it came from
 * only when that's known to be wrong. A working camera is never left out because Spotter is quiet.
 * So too a frame's configuration: until the board it came from has reported with this robot's Visor
 * on it, which configurations it may run isn't known, and its frames aren't refused for it.
 *
 * <p>A feed {@link #inMemory} has no network, so its frames come from no board and neither their
 * sender nor their configuration is checked: the simulator's, and the tests'.
 *
 * <p>Each loop, the first camera to read takes every frame that arrived and sorts them; each camera
 * then takes its own (its tags without poses as unsolved), with the exposure, gain and brightness
 * of its newest frame, the highest frame number and Visor's count of the camera's dropped frames,
 * each frame's detection time, and the address of the board it's on, where its live view is.
 */
public final class VisorFeed implements AutoCloseable {
  /**
   * The most frames that wait for a camera between its reads: one of the receiver's rings, a
   * quarter second of four cameras at 60 frames a second. A camera read every loop never comes near
   * it.
   */
  static final int WAITING_FRAMES = VisorReceiver.RING_FRAMES;

  /** How long a reason frames are refused is said after the last one: a few of Visor's frames. */
  static final long REFUSED_HOLD_NANOS = 2_000_000_000L;

  /** How long without a frame before a camera counts as disconnected. */
  private static final long STILL_SENDING_NANOS = 500_000_000L;

  /** A frame names its camera in one byte. */
  private static final int CAMERA_INDICES = 256;

  /** PhotonVision's order of a tag's corners, from Visor's (OpenCV's): reversed. */
  private static final int[] PHOTONVISION_ORDER = {3, 2, 1, 0};

  /** Why a frame was left out before a camera read it, each counted under its log key. */
  enum Refusal {
    /** No camera reads its index: one the robot doesn't have, or one on PhotonVision. */
    NO_READER("NoReader"),
    /** It came from an address none of the robot's listed boards has. */
    NOT_A_LISTED_BOARD("NotAListedBoard"),
    /** It came from another address than the board its camera is reported plugged into. */
    NOT_FROM_ITS_BOARD("NotFromItsBoard"),
    /** It names another configuration than one the robot gave the board it came from. */
    OTHER_CONFIG("OtherConfig"),
    /** Its image is another size than its camera's configured one. */
    OTHER_SIZE("OtherSize"),
    /** Its number was seen before for its camera: the same datagram again. */
    SEEN_BEFORE("SeenBefore"),
    /** Its camera already had {@link #WAITING_FRAMES} frames waiting. */
    WAITING_FULL("WaitingFull");

    final String key;

    Refusal(String name) {
      key = "Vision/Visor/Refused/" + name;
    }
  }

  private static final Refusal[] REFUSALS = Refusal.values();

  /** Whether a frame's sender is checked: on a port it is; in memory there's no sender. */
  private enum Senders {
    CHECKED,
    NONE_IN_MEMORY
  }

  private final VisorReceiver receiver;
  private final Senders senders;
  private final List<CameraConfig> cameras;
  private final VisorBoards boards;
  private final VisorSenders listed;
  private final VisorFrame frame = new VisorFrame();
  private final CameraToTag pose = new CameraToTag();

  /** Each camera index's reader, or null for an index nobody reads. */
  private final @Nullable Reader[] readers = new Reader[CAMERA_INDICES];

  /** The readers, in the order they were made, to go through without the empty indices. */
  private final List<Reader> reading = new ArrayList<>();

  /** Frames refused, by {@link Refusal}'s ordinal, since the robot started. */
  private final long[] refused = new long[REFUSALS.length];

  /** The receiver's count of other-version frames as last seen, and when it last rose. */
  private long otherVersionSeen;

  private long otherVersionHeardNanos;
  private boolean otherVersionHeard;
  private int versionErrorId;
  private int versionErrorCamera = -1;
  private String versionError = "";

  /** One camera's reader: its frames waiting for it, what they said, and what it's checked by. */
  static final class Reader {
    final int index;

    /** Its configured image size, when the feed knows it. */
    final Optional<CameraConfig.ImageSensor> sensor;

    final FrameNumbers numbers = new FrameNumbers();

    final List<CameraIO.Frame> frames = new ArrayList<>();
    final List<CameraIO.SeenTag> tags = new ArrayList<>();

    /** The numbers of the waiting frames' tags Visor sent without poses, and how many there are. */
    int[] unsolved = new int[0];

    int unsolvedCount;
    int skipped;

    /** Each waiting frame's detection time, in milliseconds, in the order they arrived. */
    final double[] detectMs = new double[WAITING_FRAMES];

    int detectCount;

    /** Visor's dropped count, as the newest frame said; -1 until one does. */
    long dropped = -1;

    /** When its last frame arrived, on the robot's clock: a frame's own stamp may mean nothing. */
    long lastHeardNanos;

    boolean heard;

    // The newest frame's camera settings and brightness, kept until another frame says, as the
    // inputs carry them: -1 where unknown.
    double exposureMs = -1;
    int gain = -1;
    boolean autoExposure;
    int brightnessMedian = -1;
    int brightnessHigh = -1;
    double clippedShare = -1;

    // The board it's plugged into, as last reported, and the listed board that is; and where its
    // live view is: that board, or the listed board its frames come from.
    Optional<String> placement = Optional.empty();
    VisorSenders.@Nullable Board placedOn;
    String boardAddress = "";

    // The sender of its last frame, and its address as text, made once a sender.
    @Nullable InetAddress sender;
    String senderAddress = "";

    // Why its frames were last refused, when, and the reason in words, made once for each reason.
    @Nullable Refusal problem;
    long problemHeardNanos;
    @Nullable Refusal problemSaid;
    String problemText = "";

    // The arrays its inputs were last given, used again while their lengths hold.
    CameraIO.Frame[] framesArray = {};
    CameraIO.SeenTag[] tagsArray = {};
    int[] unsolvedArray = {};
    double[] detectArray = {};

    Reader(int index, Optional<CameraConfig.ImageSensor> sensor) {
      this.index = index;
      this.sensor = sensor;
    }

    int waiting() {
      return frames.size() + skipped;
    }

    /** Notes a tag Visor sent without poses, making room only when there's none. */
    void unsolved(int tag) {
      if (unsolvedCount == unsolved.length) {
        unsolved = Arrays.copyOf(unsolved, Math.max(8, 2 * unsolved.length));
      }
      unsolved[unsolvedCount] = tag;
      unsolvedCount++;
    }

    /** Takes the settings, brightness and health of the newest frame. */
    void newest(VisorFrame frame) {
      exposureMs = frame.exposureKnown() ? frame.exposureMicros() / 1000.0 : -1;
      gain = frame.gainKnown() ? frame.gain() : -1;
      autoExposure = frame.autoExposure();
      brightnessMedian = frame.medianBrightness();
      brightnessHigh = frame.highBrightness();
      clippedShare = frame.clippedShare();
      dropped = frame.dropped();
    }

    /** Places it on the listed board the boards now report it plugged into. */
    void place(Optional<String> placement, VisorSenders listed) {
      this.placement = placement;
      problemSaid = null;
      placedOn = placement.map(listed::listedAs).orElse(null);
      if (placedOn != null) {
        boardAddress = placedOn.listed;
      }
    }

    /** The text of a sender's address, made only when the sender changes. */
    String addressOf(InetAddress from) {
      if (from != sender) {
        sender = from;
        senderAddress = from.getHostAddress();
      }
      return senderAddress;
    }
  }

  private VisorFeed(
      VisorReceiver receiver,
      Senders senders,
      List<CameraConfig> cameras,
      VisorBoards boards,
      VisorSenders.Resolver resolver) {
    this.receiver = receiver;
    this.senders = senders;
    this.cameras = List.copyOf(cameras);
    this.boards = boards;
    this.listed = new VisorSenders(boards.listed(), resolver);
  }

  /**
   * Frames from the coprocessors, on Visor's port, each checked against its camera.
   *
   * @param cameras the robot's cameras, in its definition's order: a frame's index is its place
   * @param boards the robot's listed boards, and which each camera is plugged into
   */
  public static VisorFeed listen(int port, List<CameraConfig> cameras, VisorBoards boards) {
    return listen(port, cameras, boards, VisorSenders.DNS);
  }

  /** The same, with the boards' names looked up by this. */
  static VisorFeed listen(
      int port, List<CameraConfig> cameras, VisorBoards boards, VisorSenders.Resolver resolver) {
    return new VisorFeed(VisorReceiver.listen(port), Senders.CHECKED, cameras, boards, resolver);
  }

  /**
   * Frames a simulation gives it ({@link #offer}), through the same decoding real ones take. They
   * come from no board, and their sizes are the simulation's own.
   */
  public static VisorFeed inMemory() {
    return inMemory(List.of());
  }

  /** Frames given it, checked against these cameras' configured sizes. */
  static VisorFeed inMemory(List<CameraConfig> cameras) {
    return new VisorFeed(
        VisorReceiver.inMemory(),
        Senders.NONE_IN_MEMORY,
        cameras,
        VisorBoards.none(),
        VisorSenders.DNS);
  }

  /** The UDP port it listens on, as bound, or -1 for one in memory. */
  public int port() {
    return receiver.port();
  }

  /** Takes a frame as if a coprocessor had sent it. For a feed {@link #inMemory}. */
  public void offer(VisorFrame sent) {
    receiver.offer(sent);
  }

  /**
   * The reader for one camera index, made the first time it's asked for: only an index with a
   * reader keeps its frames.
   */
  Reader reader(int camera) {
    if (camera < 0 || camera >= CAMERA_INDICES) {
      throw new IllegalArgumentException(
          "a Visor camera's index is 0 to " + (CAMERA_INDICES - 1) + ", not " + camera);
    }
    Reader made = readers[camera];
    if (made != null) {
      return made;
    }
    made = new Reader(camera, sensorOf(camera));
    readers[camera] = made;
    reading.add(made);
    return made;
  }

  /** A camera's configured sensor, when the feed has its configuration and it's on Visor. */
  private Optional<CameraConfig.ImageSensor> sensorOf(int camera) {
    if (camera >= cameras.size()) {
      return Optional.empty();
    }
    CameraConfig config = cameras.get(camera);
    if (!(config.software() instanceof CameraConfig.Software.Visor)) {
      return Optional.empty();
    }
    return Optional.of(config.sensor());
  }

  /**
   * Moves one camera's frames since its last read into its inputs, after sorting whatever arrived.
   *
   * @param nowNanos the robot's clock, for whether the camera is still sending
   */
  void read(Reader mine, CameraIO.CameraIOInputs inputs, long nowNanos) {
    sortWhatArrived(nowNanos);
    mine.framesArray = mine.frames.toArray(sized(mine.framesArray, mine.frames.size()));
    mine.tagsArray = mine.tags.toArray(sized(mine.tagsArray, mine.tags.size()));
    if (mine.detectArray.length != mine.detectCount) {
      mine.detectArray = new double[mine.detectCount];
    }
    System.arraycopy(mine.detectMs, 0, mine.detectArray, 0, mine.detectCount);
    if (mine.unsolvedArray.length != mine.unsolvedCount) {
      mine.unsolvedArray = new int[mine.unsolvedCount];
    }
    System.arraycopy(mine.unsolved, 0, mine.unsolvedArray, 0, mine.unsolvedCount);
    inputs.frames = mine.framesArray;
    inputs.tags = mine.tagsArray;
    inputs.unsolvedTags = mine.unsolvedArray;
    inputs.detectMs = mine.detectArray;
    inputs.skippedFrames = mine.skipped;
    // A camera is connected while its frames keep coming: Visor sends one for every image. Judged
    // by when they arrive, so a coprocessor whose clock isn't synced yet still counts as sending.
    inputs.connected = mine.heard && nowNanos - mine.lastHeardNanos < STILL_SENDING_NANOS;
    inputs.error = whyRefused(mine, nowNanos);
    inputs.exposureMs = mine.exposureMs;
    inputs.gain = mine.gain;
    inputs.autoExposure = mine.autoExposure;
    inputs.brightnessMedian = mine.brightnessMedian;
    inputs.brightnessHigh = mine.brightnessHigh;
    inputs.clippedShare = mine.clippedShare;
    inputs.sequence = mine.numbers.highest();
    inputs.dropped = mine.dropped;
    inputs.visorAddress = mine.boardAddress;
    mine.frames.clear();
    mine.tags.clear();
    mine.unsolvedCount = 0;
    mine.skipped = 0;
    mine.detectCount = 0;
  }

  /**
   * The array to fill with this many elements: the one given while it has the length, and a new one
   * otherwise. The log copies what it's given, so an array can be filled again next loop.
   */
  private static <T> T[] sized(T[] array, int length) {
    if (array.length == length) {
      return array;
    }
    return Arrays.copyOf(array, length);
  }

  private void sortWhatArrived(long nowNanos) {
    placeTheCameras();
    while (receiver.poll(frame)) {
      Reader into = readers[frame.camera()];
      if (into == null) {
        refused[Refusal.NO_READER.ordinal()]++;
        continue;
      }
      Refusal refusal = check(into);
      if (refusal != null) {
        refuse(into, refusal, nowNanos);
        continue;
      }
      take(into, nowNanos);
    }
    noteOtherVersions(nowNanos);
    logCounts();
  }

  /** Asks where each camera's board is, and places it again where that changed. */
  private void placeTheCameras() {
    if (senders == Senders.NONE_IN_MEMORY) {
      return;
    }
    for (int i = 0; i < reading.size(); i++) {
      Reader reader = reading.get(i);
      Optional<String> placement = boards.placed(reader.index);
      // Only a change is looked at again: comparing allocates nothing.
      if (!placement.equals(reader.placement)) {
        reader.place(placement, listed);
      }
    }
  }

  /** Why a frame for this camera can't be believed, or null when it can. */
  private @Nullable Refusal check(Reader into) {
    if (senders == Senders.CHECKED && !listed.listNone()) {
      Refusal wrongSender = whyNotFromWhereItCanBe(into);
      if (wrongSender != null) {
        return wrongSender;
      }
      if (fromAnotherConfig()) {
        return Refusal.OTHER_CONFIG;
      }
    }
    if (into.sensor.isPresent() && !hasItsSize(into.sensor.get())) {
      return Refusal.OTHER_SIZE;
    }
    if (into.waiting() >= WAITING_FRAMES) {
      return Refusal.WAITING_FULL;
    }
    return null;
  }

  /**
   * Why a frame can't be from where its camera is, or null when it can: refused only where that's
   * known. A camera reported on a board whose address is found takes frames from that board alone.
   * Any other takes them from any listed board, and from an address no found board has while a
   * listed name is still unfound, since that might be its board.
   */
  private @Nullable Refusal whyNotFromWhereItCanBe(Reader into) {
    InetAddress sender = frame.sender();
    VisorSenders.Board on = into.placedOn;
    if (on != null && on.isFound()) {
      if (sender == null || !on.sent(sender)) {
        return Refusal.NOT_FROM_ITS_BOARD;
      }
      return null;
    }
    if (listed.from(sender) == null && listed.unfound() == 0) {
      return Refusal.NOT_A_LISTED_BOARD;
    }
    return null;
  }

  /**
   * Whether the frame names another configuration than one the robot gave the listed board it came
   * from: only where that's known. A frame from an address no found board has (while a listed name
   * is still unfound) isn't refused for it.
   */
  private boolean fromAnotherConfig() {
    VisorSenders.Board from = listed.from(frame.sender());
    if (from == null) {
      return false;
    }
    return boards.otherConfig(from.listed, frame.configId());
  }

  /**
   * Where a camera's live view is, once a frame from it is taken: the board it's reported on, else
   * the listed board the frame came from, or its sender when the robot lists none. Never a sender
   * that isn't a listed board.
   */
  private void noteWhereItIs(Reader into) {
    if (senders == Senders.NONE_IN_MEMORY) {
      return;
    }
    VisorSenders.Board on = into.placedOn;
    if (on != null) {
      into.boardAddress = on.listed;
      return;
    }
    InetAddress sender = frame.sender();
    if (sender == null) {
      return;
    }
    if (listed.listNone()) {
      into.boardAddress = into.addressOf(sender);
      return;
    }
    VisorSenders.Board from = listed.from(sender);
    if (from != null) {
      into.boardAddress = from.listed;
    }
  }

  private boolean hasItsSize(CameraConfig.ImageSensor sensor) {
    return frame.width() == sensor.widthPixels() && frame.height() == sensor.heightPixels();
  }

  /** Keeps a checked frame for its camera, unless its number was seen before. */
  private void take(Reader into, long nowNanos) {
    FrameNumbers.Step step = into.numbers.take(frame.sequence(), frame.captureNanos());
    if (step == FrameNumbers.Step.SEEN_BEFORE) {
      refuse(into, Refusal.SEEN_BEFORE, nowNanos);
      return;
    }
    into.heard = true;
    into.lastHeardNanos = nowNanos;
    noteWhereItIs(into);
    // Its exposure and brightness hold whatever its clock says; a frame that arrived late doesn't
    // replace the newest's.
    if (step != FrameNumbers.Step.LATE) {
      into.newest(frame);
    }
    into.detectMs[into.detectCount] = frame.detectMicros() / 1000.0;
    into.detectCount++;
    // A frame from a clock not yet synced to the robot's has a capture time that means nothing.
    if (!frame.clockSynced() || frame.captureNanos() < 0) {
      into.skipped++;
      return;
    }
    int solved = 0;
    int unsolved = 0;
    for (int tag = 0; tag < frame.tagCount(); tag++) {
      // A tag Visor couldn't solve (no lens calibration, or seen edge-on) has no pose to use, and
      // goes on as unsolved, so the pipeline can say why its image placed nothing.
      if (frame.posesSolved(tag)) {
        into.tags.add(seenTag(tag));
        solved++;
      } else {
        into.unsolved(frame.tagId(tag));
        unsolved++;
      }
    }
    into.frames.add(
        new CameraIO.Frame(frame.captureNanos(), solved, unsolved, false, Transform3d.ZERO, 0));
  }

  /** Counts a refused frame, and, for a reason the team must fix, says it on the camera. */
  private void refuse(Reader into, Refusal refusal, long nowNanos) {
    refused[refusal.ordinal()]++;
    if (refusal == Refusal.SEEN_BEFORE || refusal == Refusal.WAITING_FULL) {
      return;
    }
    into.problem = refusal;
    into.problemHeardNanos = nowNanos;
  }

  /**
   * Why the camera's frames are left out, while they keep being left out (the last 2 s) and none of
   * its frames is taken; empty otherwise. A camera whose frames still arrive from its board isn't
   * said to be unreadable for a stray board's frames, which are only counted. A coprocessor on
   * another Visor than this robot's is said on every camera, since its frames can't be read for
   * which camera they are.
   */
  private String whyRefused(Reader mine, long nowNanos) {
    if (otherVersionHeard && nowNanos - otherVersionHeardNanos < REFUSED_HOLD_NANOS) {
      return versionError;
    }
    Refusal problem = mine.problem;
    if (problem == null || nowNanos - mine.problemHeardNanos >= REFUSED_HOLD_NANOS) {
      return "";
    }
    if (mine.heard && nowNanos - mine.lastHeardNanos < REFUSED_HOLD_NANOS) {
      return "";
    }
    if (problem != mine.problemSaid) {
      mine.problemText = said(mine, problem);
      mine.problemSaid = problem;
    }
    return mine.problemText;
  }

  /** A refusal in words, made once each time the reason or the camera's board changes. */
  private static String said(Reader mine, Refusal problem) {
    return switch (problem) {
      case NOT_A_LISTED_BOARD ->
          "frames for it arrive only from an address none of the robot's coprocessors has, and are"
              + " left out: a board missing from the robot's coprocessors, or one at another"
              + " address (docs/visor.md)";
      case NOT_FROM_ITS_BOARD ->
          "frames for it arrive from another board than "
              + mine.boardAddress
              + ", the one it's plugged into, and are left out: a board still running a camera"
              + " the robot no longer has? (docs/visor.md)";
      case OTHER_CONFIG ->
          "frames for it arrive from a configuration the robot didn't give its board, and are left"
              + " out: one from before the robot program started (cameras reordered, a lens"
              + " calibrated again, a tuning), or one that didn't reach it. Disable the robot off"
              + " the field to configure it (docs/visor.md)";
      case OTHER_SIZE ->
          "frames for it arrive at another size than its configured "
              + mine.sensor.map(VisorFeed::size).orElse("one")
              + ", and are left out: its board runs an old configuration (docs/visor.md)";
      case NO_READER, SEEN_BEFORE, WAITING_FULL -> "";
    };
  }

  private static String size(CameraConfig.ImageSensor sensor) {
    return sensor.widthPixels() + "x" + sensor.heightPixels();
  }

  /**
   * Notes frames from another Visor version, which the receiver drops: the version error is said
   * while they keep coming.
   */
  private void noteOtherVersions(long nowNanos) {
    long otherVersion = receiver.otherVersion();
    if (otherVersion == otherVersionSeen) {
      return;
    }
    otherVersionSeen = otherVersion;
    otherVersionHeard = true;
    otherVersionHeardNanos = nowNanos;
    int id = receiver.lastOtherVersionId();
    int camera = receiver.lastOtherVersionCamera();
    if (id != versionErrorId || camera != versionErrorCamera) {
      versionErrorId = id;
      versionErrorCamera = camera;
      versionError =
          "a coprocessor runs Visor "
              + id
              + ", the robot "
              + VisorVersion.ID
              + " (its frames name camera "
              + camera
              + "): deploy to it again";
    }
  }

  /** The receiver's counts, and the frames refused here, under {@code Vision/Visor/}. */
  private void logCounts() {
    Logger.recordOutput("Vision/Visor/Dropped", receiver.dropped());
    Logger.recordOutput("Vision/Visor/Malformed", receiver.malformed());
    Logger.recordOutput("Vision/Visor/Unsynced", receiver.unsynced());
    Logger.recordOutput("Vision/Visor/Gaps", receiver.gaps());
    Logger.recordOutput("Vision/Visor/OtherVersion", receiver.otherVersion());
    for (Refusal refusal : REFUSALS) {
      Logger.recordOutput(refusal.key, refused[refusal.ordinal()]);
    }
    Logger.recordOutput("Vision/Visor/CamerasUnplaced", unplaced());
    Logger.recordOutput("Vision/Visor/BoardsUnfound", listed.unfound());
  }

  /**
   * How many cameras read here no board has reported plugged in yet, while the robot lists boards:
   * their frames are taken from any listed board. Normal for the first seconds, and survivable for
   * a match, so it's logged and not raised.
   */
  int unplaced() {
    if (senders == Senders.NONE_IN_MEMORY || listed.listNone()) {
      return 0;
    }
    int unplaced = 0;
    for (int i = 0; i < reading.size(); i++) {
      if (reading.get(i).placedOn == null) {
        unplaced++;
      }
    }
    return unplaced;
  }

  /** How many listed boards' names haven't been found yet. */
  int boardsUnfound() {
    return listed.unfound();
  }

  /** Frames refused for a reason since the robot started. */
  long refused(Refusal refusal) {
    return refused[refusal.ordinal()];
  }

  /** How many frames wait for a camera index now. */
  int waiting(int camera) {
    Reader reader = readers[camera];
    if (reader == null) {
      return 0;
    }
    return reader.waiting();
  }

  private CameraIO.SeenTag seenTag(int tag) {
    double[] corners = new double[8];
    for (int i = 0; i < 4; i++) {
      corners[2 * i] = frame.cornerX(tag, PHOTONVISION_ORDER[i]);
      corners[2 * i + 1] = frame.cornerY(tag, PHOTONVISION_ORDER[i]);
    }
    return new CameraIO.SeenTag(
        frame.tagId(tag),
        corners[0],
        corners[1],
        corners[2],
        corners[3],
        corners[4],
        corners[5],
        corners[6],
        corners[7],
        transform(tag, Candidate.BEST),
        transform(tag, Candidate.OTHER),
        frame.ambiguity(tag),
        false);
  }

  private Transform3d transform(int tag, Candidate candidate) {
    frame.cameraToTag(tag, candidate, pose);
    return new Transform3d(
        new Translation3d(pose.x(), pose.y(), pose.z()),
        new Rotation3d(new Quaternion(pose.qw(), pose.qx(), pose.qy(), pose.qz())));
  }

  @Override
  public void close() {
    receiver.close();
    listed.close();
  }
}
