package com.michaelgrundvig.frc.toolconfigs;

import java.io.File;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;
import javax.inject.Inject;
import org.gradle.api.DefaultTask;
import org.gradle.api.GradleException;
import org.gradle.api.file.ConfigurableFileCollection;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.file.RegularFileProperty;
import org.gradle.api.provider.ListProperty;
import org.gradle.api.provider.Property;
import org.gradle.api.tasks.Classpath;
import org.gradle.api.tasks.Input;
import org.gradle.api.tasks.InputFiles;
import org.gradle.api.tasks.Internal;
import org.gradle.api.tasks.Nested;
import org.gradle.api.tasks.OutputDirectory;
import org.gradle.api.tasks.OutputFile;
import org.gradle.api.tasks.PathSensitive;
import org.gradle.api.tasks.PathSensitivity;
import org.gradle.api.tasks.TaskAction;
import org.gradle.jvm.toolchain.JavaLauncher;
import org.gradle.process.ExecOperations;
import org.gradle.work.DisableCachingByDefault;

/**
 * One tool's files, generated from the robot's model and brought in line with what's committed, in
 * one step: what every tool-config plugin registers (the AdvantageScope plugin's {@code
 * advantageScopeAssets}).
 *
 * <p>It runs a writer's {@code main} on the compiled program, in a Java process of its own with a
 * small, fixed heap, which builds the robots' configuration records and writes the tool's files
 * into a folder under {@code build/}; then {@link ToolConfigSync} brings the committed files in
 * line. A plain plugin's files are simply written over. A strict one's ({@link #getStrict}), for a
 * tool whose GUI rewrites its file, fail the build when edited outside it, naming the difference
 * and the two ways out: change {@code Robots.java}, or {@code -PdiscardToolEdits}.
 *
 * <p><b>A merge conflict in a strict plugin's files:</b> take either side, and build. If the file
 * then matches the merged {@code Robots.java}, the build passes. If not, it fails naming the line:
 * fix {@code Robots.java}, or take the model's version with {@code -PdiscardToolEdits}. A plain
 * plugin's conflicts need no care: take either side, build, and commit what it writes.
 *
 * <p>It's up to date while the program's classes, its libraries (the writers' among them), and the
 * committed files are unchanged, so an unchanged model costs nothing. The classes count without the
 * build's stamp ({@code BuildConstants}, which every build compiles anew with its time): only what
 * the model is built from.
 */
@DisableCachingByDefault(because = "it writes committed files, which no cache should restore")
public abstract class ToolConfigTask extends DefaultTask {
  /** The program's own compiled classes and resources: what the model is built from. */
  @InputFiles
  @PathSensitive(PathSensitivity.RELATIVE)
  public abstract ConfigurableFileCollection getProgram();

  /**
   * The libraries the program runs on: WPILib, the vendors', the template's own projects. The
   * writers are among them, in the world model's jar, so a change to a writer reruns it too.
   */
  @Classpath
  public abstract ConfigurableFileCollection getLibraries();

  /** Everything the writer runs on: the program and its libraries, as they're run. */
  @Internal
  public abstract ConfigurableFileCollection getRuntimeClasspath();

  /**
   * The Java the writer runs on: the project's toolchain (Java 25 for the template), not whatever
   * Java runs Gradle.
   */
  @Nested
  public abstract Property<JavaLauncher> getLauncher();

  /** The writer's class, with its {@code main}. */
  @Input
  public abstract Property<String> getMainClass();

  /** What the writer is told, beside where to write ({@code --out=}). */
  @Input
  public abstract ListProperty<String> getArguments();

  /** The writer's heap: it builds a few records, so a little is plenty. */
  @Input
  public abstract Property<String> getMaxHeapSize();

  /**
   * Whether an edit made outside the build fails it, rather than being written over: for a tool
   * whose GUI rewrites its file. The plugin decides.
   */
  @Input
  public abstract Property<Boolean> getStrict();

  /**
   * Strict, write the model's files over any edit made outside the build: {@code
   * -PdiscardToolEdits}.
   */
  @Input
  public abstract Property<Boolean> getDiscardEdits();

  /**
   * The names of the files the writer can write, as globs on the name: only these are its to remove
   * from a folder it writes.
   */
  @Input
  public abstract ListProperty<String> getWriterFiles();

  /** Where the model comes from, as a person would change it: {@code Robots.java}. */
  @Input
  public abstract Property<String> getSource();

  /** Where the tool's files are kept, committed. */
  @OutputDirectory
  public abstract DirectoryProperty getCommitted();

  /** Where the writer writes, freshly, each time. */
  @Internal
  public abstract DirectoryProperty getGenerated();

  /**
   * What the build last wrote into the committed folder, by each file's hash: in the project's
   * {@code .gradle/} folder, which a clean leaves, so a clean doesn't make the build forget.
   */
  @OutputFile
  public abstract RegularFileProperty getSidecar();

  @Inject
  protected abstract ExecOperations getExecOperations();

  @TaskAction
  public void generate() throws IOException {
    Path generated = getGenerated().get().getAsFile().toPath();
    deleteFolder(generated);
    Files.createDirectories(generated);
    List<String> arguments = new ArrayList<>(getArguments().get());
    arguments.add("--out=" + generated);
    long started = System.nanoTime();
    getExecOperations()
        .javaexec(
            spec -> {
              spec.setExecutable(getLauncher().get().getExecutablePath().getAsFile());
              spec.classpath(getRuntimeClasspath());
              spec.getMainClass().set(getMainClass());
              spec.args(arguments);
              spec.setMaxHeapSize(getMaxHeapSize().get());
              // A short run: the quickest start, and the least memory.
              spec.jvmArgs("-XX:+UseSerialGC", "-XX:TieredStopAtLevel=1", "-Xss512k");
            });
    getLogger()
        .info("{}: the writer ran in {} ms", getName(), (System.nanoTime() - started) / 1_000_000);

    File committed = getCommitted().get().getAsFile();
    ToolConfigSync.Outcome outcome =
        new ToolConfigSync(
                generated,
                committed.toPath(),
                getSidecar().get().getAsFile().toPath(),
                getSource().get(),
                committed.getName(),
                getWriterFiles().get(),
                getStrict().get())
            .run(getDiscardEdits().get());
    if (!outcome.refused().isEmpty()) {
      throw new GradleException(String.join("\n\n", outcome.refused()));
    }
    for (String written : outcome.written()) {
      getLogger().info("{}: {}", getName(), written);
    }
  }

  private static void deleteFolder(Path folder) throws IOException {
    if (!Files.exists(folder)) {
      return;
    }
    try (Stream<Path> walk = Files.walk(folder)) {
      walk.sorted(Comparator.reverseOrder())
          .forEach(
              path -> {
                try {
                  Files.delete(path);
                } catch (IOException e) {
                  throw new UncheckedIOException(e);
                }
              });
    }
  }
}
