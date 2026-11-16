package com.michaelgrundvig.frc.toolconfigs;

import java.io.File;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import org.gradle.api.Project;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.plugins.JavaPluginExtension;
import org.gradle.api.provider.ListProperty;
import org.gradle.api.provider.Property;
import org.gradle.api.tasks.SourceSet;
import org.gradle.api.tasks.SourceSetContainer;
import org.gradle.api.tasks.TaskProvider;
import org.gradle.jvm.toolchain.JavaToolchainService;

/**
 * Registers a tool-config plugin's task ({@link ToolConfigTask}): its writer run on the compiled
 * program, its files kept in a committed folder, wired into {@code build}, never into {@code run}
 * or {@code test}, so the simulator and the edit-and-test loop never wait on it. Each plugin says
 * only what's its own.
 *
 * <p>The writer is the plugin's, in its own jar, and the robots are the program's ({@code
 * frc.robot.ToolRobots}), so it runs on the program's runtime classpath, which carries the
 * program's classes and the world model's jar it's built on, with the plugin's jar after them.
 */
final class WriterTask {
  /**
   * What a plugin's writer is and writes.
   *
   * @param task the task's name, such as {@code advantageScopeAssets}
   * @param description what the task does, as {@code ./gradlew tasks} lists it
   * @param main the writer, a class with a {@code main} that takes {@link WriterArguments}
   * @param heap the writer's heap, such as {@code 32m}
   * @param files the names the writer writes, as globs: what in the committed folder is its own
   * @param name the plugin's own name, for its folder under {@code build/tool-configs/} and its
   *     sidecar
   */
  record Writer(
      String task,
      String description,
      Class<?> main,
      String heap,
      List<String> files,
      String name) {}

  private WriterTask() {}

  /**
   * Registers the writer's task, wired into {@code build}.
   *
   * @param catalog the robot program's catalog of its robots, by its class's name
   * @param robots the robots to write, by name; empty for every robot in the catalog
   * @param folder where the files are kept, committed
   */
  static TaskProvider<ToolConfigTask> register(
      Project project,
      Writer writer,
      Property<String> catalog,
      ListProperty<String> robots,
      DirectoryProperty folder) {
    SourceSet main = project.getExtensions().getByType(SourceSetContainer.class).getByName("main");
    File writerJar = jarOf(writer.main());
    TaskProvider<ToolConfigTask> task =
        project
            .getTasks()
            .register(
                writer.task(),
                ToolConfigTask.class,
                registered -> {
                  registered.setGroup("build");
                  registered.setDescription(writer.description());
                  // The program first: its robots and the model they're built on, then the writer.
                  registered.getRuntimeClasspath().from(main.getRuntimeClasspath(), writerJar);
                  registered
                      .getProgram()
                      .from(
                          main.getOutput()
                              .getClassesDirs()
                              .getAsFileTree()
                              .matching(classes -> classes.exclude("**/BuildConstants*")));
                  registered.getProgram().from(main.getOutput().getResourcesDir());
                  registered
                      .getLibraries()
                      .from(main.getRuntimeClasspath().minus(main.getOutput()));
                  registered.getLibraries().from(writerJar);
                  registered.getMainClass().set(writer.main().getName());
                  registered
                      .getLauncher()
                      .convention(
                          project
                              .getExtensions()
                              .getByType(JavaToolchainService.class)
                              .launcherFor(
                                  project
                                      .getExtensions()
                                      .getByType(JavaPluginExtension.class)
                                      .getToolchain()));
                  registered.getArguments().set(catalog.zip(robots, WriterTask::arguments));
                  registered.getMaxHeapSize().set(writer.heap());
                  registered.getStrict().set(false);
                  registered
                      .getDiscardEdits()
                      .set(
                          project
                              .getProviders()
                              .gradleProperty("discardToolEdits")
                              .map(value -> !value.equals("false"))
                              .orElse(false));
                  registered.getSource().set("Robots.java");
                  registered.getWriterFiles().set(writer.files());
                  registered.getCommitted().set(folder);
                  registered
                      .getGenerated()
                      .set(
                          project
                              .getLayout()
                              .getBuildDirectory()
                              .dir("tool-configs/" + writer.name()));
                  registered
                      .getSidecar()
                      .set(
                          project
                              .getLayout()
                              .getProjectDirectory()
                              .file(".gradle/tool-configs/" + writer.name() + ".sha256"));
                });
    project.getTasks().named("build").configure(build -> build.dependsOn(task));
    return task;
  }

  /** The writer's command line: the catalog, then each robot named. */
  private static List<String> arguments(String catalog, List<String> robots) {
    List<String> arguments = new ArrayList<>();
    arguments.add("--catalog=" + catalog);
    for (String robot : robots) {
      arguments.add("--robot=" + robot);
    }
    return arguments;
  }

  /**
   * Where a class of this plugin's was loaded from: its jar, or in a build of the plugin itself,
   * its classes' folder.
   */
  private static File jarOf(Class<?> writer) {
    try {
      return new File(writer.getProtectionDomain().getCodeSource().getLocation().toURI());
    } catch (URISyntaxException e) {
      throw new IllegalStateException("the plugin's own jar has no path: " + e.getMessage(), e);
    }
  }
}
