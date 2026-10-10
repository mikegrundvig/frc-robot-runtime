package com.michaelgrundvig.frc.toolconfigs;

import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.PathMatcher;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HexFormat;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.stream.Stream;

/**
 * Brings a tool's committed files in line with what the model generates. Only the files that differ
 * are written, so a pull that changed the model and its files together changes nothing. What it
 * last wrote is remembered in a sidecar (each file's SHA-256), kept in the project's {@code
 * .gradle/} folder, which a clean leaves.
 *
 * <p><b>Plain</b> (AdvantageScope's assets, which nobody edits with a tool): the model's files are
 * written over whatever is there, and a file the writer can write that the model no longer gives is
 * removed.
 *
 * <p><b>Strict</b> (for a tool whose GUI rewrites its file, as a path tool's does), the sidecar
 * tells a file the build wrote from one someone changed:
 *
 * <ul>
 *   <li>a file that already matches the model is left as it is;
 *   <li>a file that's what the build last wrote, while the model changed, is rewritten;
 *   <li>a file that matches neither was edited outside the build (or, with no sidecar on a fresh
 *       clone, may have been): nothing is written, and the failure names both versions and the two
 *       ways out, unless the edit is to be discarded;
 *   <li>a new file is written;
 *   <li>a file in one of the generated folders that the model no longer gives is removed if it's
 *       what the build last wrote, and refused like an edit if not.
 * </ul>
 *
 * <p>Either way, only names the writer can write count: what a computer leaves in a folder ({@code
 * .DS_Store}, {@code Thumbs.db}, {@code desktop.ini}) is no one's, and stays.
 *
 * <p>A folder the build wrote that the model no longer gives (a robot renamed or dropped) is
 * removed with the writer's files in it, as each file would be. Only a folder the record shows the
 * build wrote: a folder a team added by hand is never touched, and a folder holding a file the
 * build didn't write is kept, with that file.
 */
final class ToolConfigSync {
  /**
   * What became of the files.
   *
   * @param written the files written or removed, each with why
   * @param refused why each refused file was refused, in a sentence or two; nothing was written
   *     when any was
   */
  record Outcome(List<String> written, List<String> refused) {}

  private final Path generated;
  private final Path committed;
  private final Path sidecar;
  private final String source;
  private final String committedName;
  private final List<PathMatcher> writerFiles;
  private final boolean strict;

  /** What computers leave in folders: removed with a folder the build no longer gives, no more. */
  private static final Set<String> CLUTTER = Set.of(".DS_Store", "Thumbs.db", "desktop.ini");

  /**
   * @param generated where the generator wrote the files, freshly
   * @param committed where the tool's files are kept, committed
   * @param sidecar what the build last wrote into {@code committed}, by each file's hash
   * @param source where the model comes from, as a person would change it ({@code Robots.java})
   * @param committedName how to name the committed folder in messages ({@code ascope_assets})
   * @param writerFiles the names of the files the writer can write, as globs ({@code model*.glb})
   * @param strict whether an edit made outside the build is refused rather than written over
   */
  ToolConfigSync(
      Path generated,
      Path committed,
      Path sidecar,
      String source,
      String committedName,
      List<String> writerFiles,
      boolean strict) {
    this.strict = strict;
    this.generated = generated;
    this.committed = committed;
    this.sidecar = sidecar;
    this.source = source;
    this.committedName = committedName;
    this.writerFiles =
        writerFiles.stream()
            .map(glob -> FileSystems.getDefault().getPathMatcher("glob:" + glob))
            .toList();
  }

  /** Whether a file's name is one the writer can write. */
  private boolean writable(String path) {
    Path name = Path.of(path.substring(path.lastIndexOf('/') + 1));
    return writerFiles.stream().anyMatch(matcher -> matcher.matches(name));
  }

  /**
   * Compares, then writes what it may.
   *
   * @param discardEdits strict, write the model's files over any edit, as {@code
   *     -PdiscardToolEdits} asks
   */
  Outcome run(boolean discardEdits) throws IOException {
    boolean overwrite = !strict || discardEdits;
    Map<String, String> remembered = readSidecar();
    Map<String, byte[]> wanted = files(generated);
    Map<String, byte[]> writes = new TreeMap<>();
    List<String> removals = new ArrayList<>();
    List<String> written = new ArrayList<>();
    List<String> refused = new ArrayList<>();
    for (Map.Entry<String, byte[]> file : wanted.entrySet()) {
      String path = file.getKey();
      Path target = committed.resolve(path);
      if (!Files.exists(target)) {
        writes.put(path, file.getValue());
        written.add(path + ": new");
        continue;
      }
      byte[] now = Files.readAllBytes(target);
      if (Arrays.equals(now, file.getValue())) {
        continue;
      }
      if (hash(now).equals(remembered.get(path))) {
        writes.put(path, file.getValue());
        written.add(path + ": " + source + " changed");
      } else if (overwrite) {
        writes.put(path, file.getValue());
        written.add(path + (strict ? ": an edit discarded" : ": written from " + source));
      } else {
        refused.add(different(path, now, file.getValue(), remembered.containsKey(path)));
      }
    }
    for (String path : stale(wanted)) {
      byte[] now = Files.readAllBytes(committed.resolve(path));
      if (hash(now).equals(remembered.get(path)) || overwrite) {
        removals.add(path);
        written.add(path + ": removed, " + source + " no longer gives it");
      } else {
        refused.add(
            committedName
                + "/"
                + path
                + " is in a folder the build writes, but "
                + source
                + " doesn't give it, and the build didn't write it.");
      }
    }
    List<String> goneFolders = goneFolders(remembered, wanted);
    for (String folder : goneFolders) {
      for (String path : files(committed.resolve(folder)).keySet()) {
        String full = folder + "/" + path;
        if (!writable(full)) {
          continue;
        }
        byte[] now = Files.readAllBytes(committed.resolve(full));
        if (hash(now).equals(remembered.get(full)) || overwrite) {
          removals.add(full);
          written.add(full + ": removed, " + source + " no longer gives its folder");
        } else {
          refused.add(
              committedName
                  + "/"
                  + full
                  + " was edited outside the build, and "
                  + source
                  + " no longer gives its folder. Delete it if the edit can go, or discard the"
                  + " edit: ./gradlew build -PdiscardToolEdits");
        }
      }
    }
    if (!refused.isEmpty()) {
      return new Outcome(List.of(), refused);
    }
    for (Map.Entry<String, byte[]> write : writes.entrySet()) {
      Path target = committed.resolve(write.getKey());
      Files.createDirectories(target.getParent());
      Files.write(target, write.getValue());
    }
    for (String path : removals) {
      Files.delete(committed.resolve(path));
    }
    for (String folder : goneFolders) {
      Path gone = committed.resolve(folder);
      List<String> left = new ArrayList<>(files(gone).keySet());
      if (CLUTTER.containsAll(left)) {
        deleteFolder(gone);
        written.add(folder + "/: removed, " + source + " no longer gives it");
      } else {
        written.add(folder + "/: kept, for the files in it the build didn't write");
      }
    }
    Map<String, String> remember = new TreeMap<>();
    for (Map.Entry<String, byte[]> file : wanted.entrySet()) {
      remember.put(file.getKey(), hash(file.getValue()));
    }
    writeSidecar(remember);
    return new Outcome(written, List.of());
  }

  /**
   * The folders the build last wrote, by the record, that the model no longer gives and that are
   * still there.
   */
  private List<String> goneFolders(Map<String, String> remembered, Map<String, byte[]> wanted) {
    Set<String> now = folders(wanted.keySet());
    List<String> gone = new ArrayList<>();
    for (String folder : folders(remembered.keySet())) {
      if (!now.contains(folder) && Files.isDirectory(committed.resolve(folder))) {
        gone.add(folder);
      }
    }
    return gone;
  }

  /** The top-level folders of some paths. */
  private static Set<String> folders(Set<String> paths) {
    Set<String> folders = new TreeSet<>();
    for (String path : paths) {
      int slash = path.indexOf('/');
      if (slash > 0) {
        folders.add(path.substring(0, slash));
      }
    }
    return folders;
  }

  /** A folder, and everything left in it: only what computers leave there. */
  private static void deleteFolder(Path folder) throws IOException {
    try (Stream<Path> walk = Files.walk(folder)) {
      for (Path path : walk.sorted(Comparator.reverseOrder()).toList()) {
        Files.delete(path);
      }
    }
  }

  /** The files in the committed copies of the generated folders that weren't generated. */
  private List<String> stale(Map<String, byte[]> wanted) throws IOException {
    List<String> stale = new ArrayList<>();
    for (String folder : folders(wanted.keySet())) {
      Path inCommitted = committed.resolve(folder);
      if (Files.isDirectory(inCommitted)) {
        for (String path : files(inCommitted).keySet()) {
          String full = folder + "/" + path;
          if (!wanted.containsKey(full) && writable(full)) {
            stale.add(full);
          }
        }
      }
    }
    return stale;
  }

  /**
   * Why a file is refused: the first place it differs, the file's version beside the model's, and
   * the two ways out.
   *
   * @param remembered whether the sidecar knew the file: without it, nothing says who changed what
   */
  private String different(String path, byte[] now, byte[] wanted, boolean remembered) {
    String name = committedName + "/" + path;
    StringBuilder why = new StringBuilder(name);
    if (remembered) {
      why.append(" was edited outside the build, and differs from what ").append(source);
      why.append(" gives.");
    } else {
      why.append(" differs from what ").append(source).append(" gives, and nothing records what");
      why.append(" the build last wrote there (a fresh clone, or after a clean), so it may have");
      why.append(" been edited.");
    }
    if (isText(path)) {
      String[] fileLines = new String(now, StandardCharsets.UTF_8).split("\n", -1);
      String[] modelLines = new String(wanted, StandardCharsets.UTF_8).split("\n", -1);
      int line = 0;
      while (line < fileLines.length
          && line < modelLines.length
          && fileLines[line].equals(modelLines[line])) {
        line++;
      }
      String theFile = "line " + (line + 1) + " of the file:";
      String theModel = source + " gives:";
      int width = Math.max(theFile.length(), theModel.length()) + 2;
      why.append("\n  ").append(padded(theFile, width)).append(lineOf(fileLines, line));
      why.append("\n  ").append(padded(theModel, width)).append(lineOf(modelLines, line));
    } else {
      int at = 0;
      while (at < now.length && at < wanted.length && now[at] == wanted[at]) {
        at++;
      }
      why.append("\n  the file's ").append(now.length).append(" bytes against the ");
      why.append(wanted.length).append(" ").append(source).append(" gives, apart from byte ");
      why.append(at);
    }
    why.append("\nChange ").append(source).append(" if the file is right, or discard the edit:");
    why.append(" ./gradlew build -PdiscardToolEdits");
    return why.toString();
  }

  private static String lineOf(String[] lines, int line) {
    return line < lines.length ? lines[line].strip() : "(nothing: it ends before)";
  }

  private static String padded(String text, int width) {
    return text + " ".repeat(width - text.length());
  }

  private static boolean isText(String path) {
    return path.endsWith(".json") || path.endsWith(".md") || path.endsWith(".txt");
  }

  /** Every file under a folder, by its path from it with forward slashes, in order. */
  private static Map<String, byte[]> files(Path folder) throws IOException {
    Map<String, byte[]> files = new TreeMap<>();
    if (!Files.isDirectory(folder)) {
      return files;
    }
    try (Stream<Path> walk = Files.walk(folder)) {
      for (Path file : walk.filter(Files::isRegularFile).toList()) {
        String path = folder.relativize(file).toString().replace('\\', '/');
        files.put(path, Files.readAllBytes(file));
      }
    }
    return files;
  }

  private Map<String, String> readSidecar() throws IOException {
    Map<String, String> remembered = new LinkedHashMap<>();
    if (Files.exists(sidecar)) {
      Properties properties = new Properties();
      try (Reader in = Files.newBufferedReader(sidecar, StandardCharsets.UTF_8)) {
        properties.load(in);
      }
      for (String path : properties.stringPropertyNames()) {
        remembered.put(path, properties.getProperty(path));
      }
    }
    return remembered;
  }

  private void writeSidecar(Map<String, String> remember) throws IOException {
    Files.createDirectories(sidecar.getParent());
    Properties properties = new Properties();
    properties.putAll(remember);
    try (Writer out = Files.newBufferedWriter(sidecar, StandardCharsets.UTF_8)) {
      properties.store(out, "What the build last wrote into " + committedName + ", by SHA-256");
    }
  }

  static String hash(byte[] bytes) {
    try {
      return HexFormat.of().formatHex(MessageDigest.getInstance("SHA-256").digest(bytes));
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }
}
