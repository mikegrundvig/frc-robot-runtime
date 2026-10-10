package com.michaelgrundvig.frc.runtime.venues;

import com.michaelgrundvig.frc.world.field.Venue;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;

/**
 * Where venue files live on the robot, and which one is selected.
 *
 * <ul>
 *   <li><b>Stored:</b> venues uploaded in the pit, kept outside the deploy folder so a code deploy
 *       can't wipe them, each named by its contents' hash.
 *   <li><b>Committed:</b> venues in the repository's {@code src/main/deploy/venues/}, which ship
 *       with the build.
 *   <li><b>The selection:</b> a file in the stored folder holding the selected venue's hash. With
 *       none, the robot uses the official layout. It stays across reboots and deploys.
 * </ul>
 */
public final class VenueStore {
  /** The robot user's venue folder on Systemcore (the robot user's home: to confirm, S6). */
  public static final Path ROBOT_FOLDER = Path.of("/home/systemcore/venues");

  /**
   * The most venues the robot keeps from uploads: an event's few, with room to spare, and few
   * enough that listing them for the pit stays quick.
   */
  public static final int MOST_STORED = 32;

  private static final String SELECTION = "selected";

  private final Path stored;
  private final Path committed;

  /**
   * @param stored where uploaded venues and the selection are kept
   * @param committed the venues that ship with the build
   */
  public VenueStore(Path stored, Path committed) {
    this.stored = stored;
    this.committed = committed;
  }

  /**
   * The selected venue's file, or empty for the official layout.
   *
   * @throws IOException if the selection names a venue no folder has, or a file can't be read
   */
  public Optional<String> selectedText() throws IOException {
    Path selection = stored.resolve(SELECTION);
    if (!Files.isRegularFile(selection)) {
      return Optional.empty();
    }
    String hash = Files.readString(selection, StandardCharsets.UTF_8).strip();
    if (hash.isEmpty()) {
      return Optional.empty();
    }
    for (Path file : files()) {
      String text = Files.readString(file, StandardCharsets.UTF_8);
      if (hash(text).equals(hash)) {
        return Optional.of(text);
      }
    }
    throw new IOException("the selected venue (" + hash + ") isn't on the robot");
  }

  /** Whether a venue file's contents ship with the build. */
  public boolean committed(String text) {
    String hash = hash(text);
    try (Stream<Path> files = list(committed)) {
      for (Path file : files.toList()) {
        if (hash(Files.readString(file, StandardCharsets.UTF_8)).equals(hash)) {
          return true;
        }
      }
    } catch (IOException e) {
      return false;
    }
    return false;
  }

  /**
   * Selects a venue by its contents' hash, or the official layout with null. The robot takes it at
   * the start of a loop while disabled.
   */
  public void select(@Nullable String hash) throws IOException {
    write(SELECTION, hash == null ? "" : hash);
  }

  /**
   * Keeps a venue file on the robot, named by its hash, and says the hash. The caller checks it
   * first.
   */
  public String save(String text) throws IOException {
    String hash = hash(text);
    write(hash + ".json", text);
    return hash;
  }

  /**
   * Why the robot can't keep another venue file, or null if it can: it keeps at most {@value
   * #MOST_STORED} uploads, and one it keeps already takes no more room.
   */
  public @Nullable String whyNoRoomFor(String text) throws IOException {
    if (Files.isRegularFile(stored.resolve(hash(text) + ".json"))) {
      return null;
    }
    try (Stream<Path> files = list(stored)) {
      return files.count() < MOST_STORED
          ? null
          : "the robot keeps "
              + MOST_STORED
              + " uploaded venues already, the most it keeps: delete those you don't need from "
              + stored
              + ", then upload again";
    }
  }

  /** Every venue file the robot has: stored, then committed. */
  public List<Path> files() throws IOException {
    List<Path> files = new ArrayList<>();
    for (Path folder : List.of(stored, committed)) {
      try (Stream<Path> listed = list(folder)) {
        files.addAll(listed.toList());
      }
    }
    return files;
  }

  private static Stream<Path> list(Path folder) throws IOException {
    if (!Files.isDirectory(folder)) {
      return Stream.empty();
    }
    return Files.list(folder)
        .filter(file -> file.getFileName().toString().endsWith(".json"))
        .filter(Files::isRegularFile)
        .sorted();
  }

  /** Written whole, then moved into place, so a power cut never leaves half a file. */
  private void write(String name, String text) throws IOException {
    Files.createDirectories(stored);
    Path temporary = stored.resolve(name + ".tmp");
    Files.writeString(temporary, text, StandardCharsets.UTF_8);
    Files.move(
        temporary,
        stored.resolve(name),
        StandardCopyOption.REPLACE_EXISTING,
        StandardCopyOption.ATOMIC_MOVE);
  }

  private static String hash(String text) {
    return Venue.hash(text);
  }
}
