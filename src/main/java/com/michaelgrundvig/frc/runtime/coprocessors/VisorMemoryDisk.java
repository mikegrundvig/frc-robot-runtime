package com.michaelgrundvig.frc.runtime.coprocessors;

import java.io.IOException;
import java.io.Reader;
import java.io.StringWriter;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.concurrent.Executor;

/**
 * {@link VisorMemory} in a file on the robot's disk, beside the venues the pit keeps: {@code
 * /home/systemcore/visor/configurations.properties} on a robot. It's read once as the program
 * starts, and written whole, to a file beside it and then moved over it, on a thread of its own
 * whenever what's kept changes. A file that's missing or can't be read is nothing kept, said in one
 * log line; a write that fails is said by {@link #problem}, never thrown.
 *
 * <p>Each board is two keys, by its address: {@code <board>.committed}, the committed
 * configuration's ID, and {@code <board>.ids}, the IDs kept, comma-separated; each ID in hex, as
 * its first eight digits of the configuration's SHA-256 read.
 */
final class VisorMemoryDisk implements VisorMemory {
  /** The folder on Systemcore, in the robot user's home, as the venues' is. */
  static final Path ROBOT_FOLDER = Path.of("/home/systemcore/visor");

  /** The file's name in its folder. */
  static final String FILE = "configurations.properties";

  private static final String COMMITTED = ".committed";
  private static final String IDS = ".ids";

  private final Path file;
  private final Executor writing;
  private final Map<String, Kept> recalled;
  private volatile String problem = "";

  /**
   * Reads what was kept, now.
   *
   * @param folder where the file is
   * @param writing where each write runs: off the loop, one at a time, in order
   */
  VisorMemoryDisk(Path folder, Executor writing) {
    this.file = folder.resolve(FILE);
    this.writing = writing;
    this.recalled = read(file);
  }

  @Override
  public Map<String, Kept> recalled() {
    return recalled;
  }

  @Override
  public void keep(Map<String, Kept> boards) {
    Map<String, Kept> copy = Map.copyOf(boards);
    writing.execute(() -> write(copy));
  }

  @Override
  public String problem() {
    return problem;
  }

  private void write(Map<String, Kept> boards) {
    try {
      Files.createDirectories(file.getParent());
      Path partial = file.resolveSibling(FILE + ".new");
      Files.writeString(partial, text(boards), StandardCharsets.UTF_8);
      Files.move(
          partial, file, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
      problem = "";
    } catch (IOException | RuntimeException e) {
      problem = file + ": " + e.getMessage();
    }
  }

  /** The file's text for these boards. */
  static String text(Map<String, Kept> boards) {
    Properties properties = new Properties();
    for (Map.Entry<String, Kept> board : boards.entrySet()) {
      Kept kept = board.getValue();
      properties.setProperty(board.getKey() + COMMITTED, hex(kept.committedId()));
      List<String> ids = kept.ids().stream().map(VisorMemoryDisk::hex).toList();
      properties.setProperty(board.getKey() + IDS, String.join(",", ids));
    }
    StringWriter text = new StringWriter();
    try {
      properties.store(text, "The Visor configurations the robot takes each board's frames from");
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return text.toString();
  }

  /** What the file keeps; nothing, said in the log, when it's missing or can't be read. */
  private static Map<String, Kept> read(Path file) {
    if (!Files.isRegularFile(file)) {
      return Map.of();
    }
    try (Reader in = Files.newBufferedReader(file, StandardCharsets.UTF_8)) {
      Properties properties = new Properties();
      properties.load(in);
      return boards(properties);
    } catch (IOException | RuntimeException e) {
      System.out.println(
          "Visor: " + file + " can't be read (" + e.getMessage() + "), so nothing is recalled");
      return Map.of();
    }
  }

  /**
   * Each board in the file, with at most {@link #MOST_PER_BOARD} IDs.
   *
   * @throws IllegalArgumentException for a value that isn't an ID
   */
  static Map<String, Kept> boards(Properties properties) {
    Map<String, Kept> boards = new LinkedHashMap<>();
    for (String key : properties.stringPropertyNames()) {
      if (!key.endsWith(COMMITTED)) {
        continue;
      }
      String board = key.substring(0, key.length() - COMMITTED.length());
      int committed = id(properties.getProperty(key));
      List<Integer> ids = new ArrayList<>();
      for (String id : properties.getProperty(board + IDS, "").split(",", -1)) {
        if (!id.isEmpty() && ids.size() < MOST_PER_BOARD) {
          ids.add(id(id));
        }
      }
      boards.put(board, new Kept(committed, ids));
    }
    return Map.copyOf(boards);
  }

  private static String hex(int id) {
    return HexFormat.of().toHexDigits(id);
  }

  private static int id(String hex) {
    // fromHexDigits refuses anything but hex digits, and takes up to eight.
    if (hex.length() != 8) {
      throw new IllegalArgumentException("an ID is eight hex digits, not " + hex);
    }
    return HexFormat.fromHexDigits(hex);
  }
}
