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

/**
 * Where venue files live on the robot, and which one is selected.
 *
 * <ul>
 *   <li><b>Stored</b> venues are the ones uploaded in the pit. They are kept outside the deploy
 *       folder, so that deploying code cannot wipe them, and each is named by the hash of its
 *       contents.
 *   <li><b>Committed</b> venues are the ones in the repository's {@code src/main/deploy/venues/},
 *       which ship with the build.
 *   <li><b>The selection</b> is a file in the stored folder that holds the selected venue's hash.
 *       With no selection the robot uses the official layout. The selection survives reboots and
 *       deploys.
 * </ul>
 */
public final class VenueStore {
  /**
   * The stored venues' folder on Systemcore, in the robot user's home. Where that home is has yet
   * to be confirmed on a Systemcore (S6).
   */
  public static final Path ROBOT_FOLDER = Path.of("/home/systemcore/venues");

  /**
   * The most uploaded venues the robot keeps. An event needs only a few, so this leaves room to
   * spare, and it is few enough that listing them for the pit stays quick.
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

  /** Whether a venue file with these contents ships with the build. */
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
   * Selects a venue by the hash of its contents. The robot takes the selection at the start of a
   * loop while it is disabled.
   */
  public void select(String hash) throws IOException {
    write(SELECTION, hash);
  }

  /** Selects the official layout, which is to select no venue. */
  public void selectOfficial() throws IOException {
    write(SELECTION, "");
  }

  /**
   * Keeps a venue file on the robot, named by its hash. The caller checks the file first.
   *
   * @return the file's hash
   */
  public String save(String text) throws IOException {
    String hash = hash(text);
    write(hash + ".json", text);
    return hash;
  }

  /**
   * Why the robot cannot keep this venue file, if it cannot. It keeps at most {@value #MOST_STORED}
   * uploads. A file it already keeps takes no more room, so it can always be kept again.
   */
  public Optional<String> whyNoRoomFor(String text) throws IOException {
    boolean isKeptAlready = Files.isRegularFile(stored.resolve(hash(text) + ".json"));
    if (isKeptAlready) {
      return Optional.empty();
    }
    try (Stream<Path> files = list(stored)) {
      if (files.count() < MOST_STORED) {
        return Optional.empty();
      }
    }
    return Optional.of(
        "the robot keeps "
            + MOST_STORED
            + " uploaded venues already, the most it keeps: delete those you don't need from "
            + stored
            + ", then upload again");
  }

  /** Every venue file the robot has: the stored ones, then the committed ones. */
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

  /**
   * Writes a file in the stored folder. It is written whole under another name and then moved into
   * place, so a power cut never leaves half a file.
   */
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
