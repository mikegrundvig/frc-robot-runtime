package com.michaelgrundvig.frc.toolconfigs;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.security.DigestInputStream;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.HexFormat;
import java.util.Properties;
import java.util.jar.JarFile;
import java.util.zip.ZipEntry;
import org.gradle.api.Action;
import org.gradle.api.GradleException;
import org.gradle.api.Task;
import org.gradle.api.file.FileCollection;

/**
 * Refuses to stage a Visor release whose SHA-256 isn't the one the robot's Visor client was
 * published with. The client's jar carries it, in the resource its {@code VisorVersion} reads
 * ({@code release-sha256}); the archive comes from the FRC libraries' repository beside it. So an
 * archive replaced there, or on its way, never reaches the robot's deploy, and the robot names the
 * same hash to each board, which checks it again before anything in the archive runs.
 */
final class VisorReleaseCheck implements Action<Task> {
  /** Where the client's jar keeps its version and its release's SHA-256: the client's own name. */
  static final String RESOURCE = "com/michaelgrundvig/frc/visor/version.properties";

  /** The key of the release's SHA-256 in that resource. */
  static final String KEY = "release-sha256";

  private final FileCollection release;
  private final FileCollection client;

  /**
   * @param release the release's archive: one file
   * @param client the Visor client's jar on the robot's runtime classpath: one file
   */
  VisorReleaseCheck(FileCollection release, FileCollection client) {
    this.release = release;
    this.client = client;
  }

  @Override
  public void execute(Task task) {
    File archive = release.getSingleFile();
    File jar = client.getSingleFile();
    String expected = releaseSha256(jar);
    if (expected.isEmpty()) {
      throw new GradleException(
          "The Visor client "
              + jar.getName()
              + " names no SHA-256 for its release, so "
              + archive.getName()
              + " can't be checked and isn't staged: build with a published client");
    }
    String actual = sha256(archive);
    if (!actual.equals(expected)) {
      throw new GradleException(
          archive.getName()
              + "'s SHA-256 is "
              + actual
              + ", not the "
              + expected
              + " its Visor client was published with: it isn't that release, so it isn't staged");
    }
  }

  /** The release's SHA-256 as the client's jar names it; empty when it names none. */
  static String releaseSha256(File jar) {
    try (JarFile file = new JarFile(jar)) {
      ZipEntry entry = file.getEntry(RESOURCE);
      if (entry == null) {
        return "";
      }
      Properties properties = new Properties();
      try (InputStream in = file.getInputStream(entry)) {
        properties.load(in);
      }
      return properties.getProperty(KEY, "");
    } catch (IOException e) {
      throw new UncheckedIOException("reading " + jar, e);
    }
  }

  /** A file's SHA-256, in lowercase hex. */
  static String sha256(File file) {
    try (DigestInputStream in =
        new DigestInputStream(
            Files.newInputStream(file.toPath()), MessageDigest.getInstance("SHA-256"))) {
      in.transferTo(OutputStream.nullOutputStream());
      return HexFormat.of().formatHex(in.getMessageDigest().digest());
    } catch (IOException e) {
      throw new UncheckedIOException("reading " + file, e);
    } catch (NoSuchAlgorithmException e) {
      throw new IllegalStateException("every Java has SHA-256", e);
    }
  }
}
