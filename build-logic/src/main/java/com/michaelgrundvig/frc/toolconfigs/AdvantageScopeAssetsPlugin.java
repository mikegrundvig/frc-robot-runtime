package com.michaelgrundvig.frc.toolconfigs;

import java.util.ArrayList;
import java.util.List;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.plugins.JavaPlugin;
import org.gradle.api.plugins.JavaPluginExtension;
import org.gradle.api.provider.ListProperty;
import org.gradle.api.provider.Property;
import org.gradle.api.tasks.SourceSet;
import org.gradle.api.tasks.SourceSetContainer;
import org.gradle.api.tasks.TaskProvider;
import org.gradle.jvm.toolchain.JavaToolchainService;

/**
 * {@code frc.advantagescope-assets}: AdvantageScope's robot assets ({@code ascope_assets/}),
 * generated from the robot's configuration and checked against what's committed, as part of every
 * build. A project that doesn't use AdvantageScope leaves the plugin out, and nothing of it runs.
 *
 * <pre>{@code
 * plugins { id "frc.advantagescope-assets" }
 * advantageScope {
 *     robots = ["competition", "practice"] // which robots; by default every one in Robots.all()
 * }
 * }</pre>
 *
 * <p>The assets are plain generated files, which nobody edits with a tool: the build writes over an
 * edit (and a clean, a merge, or a pull never trips it up). A strict plugin, for a tool whose GUI
 * rewrites its file, would refuse one ({@link ToolConfigTask#getStrict}).
 *
 * <p>Its task, {@code advantageScopeAssets}, runs the writer ({@code
 * com.michaelgrundvig.frc.world.bridges.AdvantageScopeAssets}) on the compiled program, which only
 * builds the robots' configuration records and writes files: no robot program, HAL, NetworkTables
 * or simulator. The writer is the robot model's ({@code model/}) and the robots are the program's
 * ({@code frc.robot.ToolRobots}), so it runs on the program's runtime classpath, which carries
 * both: the program's classes, and the world model's jar among its libraries. It's wired into
 * {@code build}, never into {@code run} or {@code test}, so the simulator and the edit-and-test
 * loop never wait on it.
 */
public final class AdvantageScopeAssetsPlugin implements Plugin<Project> {
  /** The writer the task runs: the world model's, found on the program's runtime classpath. */
  static final String WRITER = "com.michaelgrundvig.frc.world.bridges.AdvantageScopeAssets";

  /**
   * The writer's heap. It builds a few records and writes about 130 kB: measured (2026-10-01) to
   * run in 8 MB, in 0.3 s and 54 MB of memory all told, on the template's robot.
   */
  static final String HEAP = "32m";

  /** What the build sees of the plugin: {@code advantageScope { ... }}. */
  public abstract static class Extension {
    /**
     * The robots whose assets are written, by name, as {@code Robots.all()} names them: one folder
     * each. Empty, the default, is every robot in {@code Robots.all()}. (Not {@code
     * Robots.current()}, which a robot decides by what it reads of itself, and a laptop can't.)
     */
    public abstract ListProperty<String> getRobots();

    /**
     * The robot program's catalog of its robots, a {@code
     * com.michaelgrundvig.frc.world.bridges.RobotCatalog}: {@code frc.robot.ToolRobots} by default.
     */
    public abstract Property<String> getCatalog();

    /** Where the assets are kept, committed: {@code ascope_assets/} by default. */
    public abstract DirectoryProperty getFolder();
  }

  @Override
  public void apply(Project project) {
    project.getPluginManager().apply(JavaPlugin.class);
    Extension extension = project.getExtensions().create("advantageScope", Extension.class);
    extension.getRobots().convention(List.of());
    extension.getCatalog().convention("frc.robot.ToolRobots");
    extension
        .getFolder()
        .convention(project.getLayout().getProjectDirectory().dir("ascope_assets"));

    SourceSet main = project.getExtensions().getByType(SourceSetContainer.class).getByName("main");
    TaskProvider<ToolConfigTask> assets =
        project
            .getTasks()
            .register(
                "advantageScopeAssets",
                ToolConfigTask.class,
                task -> {
                  task.setGroup("build");
                  task.setDescription(
                      "Generate AdvantageScope's robot assets (ascope_assets/) from Robots.java.");
                  task.getRuntimeClasspath().from(main.getRuntimeClasspath());
                  task.getProgram()
                      .from(
                          main.getOutput()
                              .getClassesDirs()
                              .getAsFileTree()
                              .matching(classes -> classes.exclude("**/BuildConstants*")));
                  task.getProgram().from(main.getOutput().getResourcesDir());
                  task.getLibraries().from(main.getRuntimeClasspath().minus(main.getOutput()));
                  task.getMainClass().set(WRITER);
                  task.getLauncher()
                      .convention(
                          project
                              .getExtensions()
                              .getByType(JavaToolchainService.class)
                              .launcherFor(
                                  project
                                      .getExtensions()
                                      .getByType(JavaPluginExtension.class)
                                      .getToolchain()));
                  task.getArguments()
                      .set(
                          extension
                              .getCatalog()
                              .zip(
                                  extension.getRobots(),
                                  (catalog, robots) -> {
                                    List<String> arguments = new ArrayList<>();
                                    arguments.add("--catalog=" + catalog);
                                    for (String robot : robots) {
                                      arguments.add("--robot=" + robot);
                                    }
                                    return arguments;
                                  }));
                  task.getMaxHeapSize().set(HEAP);
                  task.getStrict().set(false);
                  task.getDiscardEdits()
                      .set(
                          project
                              .getProviders()
                              .gradleProperty("discardToolEdits")
                              .map(value -> !value.equals("false"))
                              .orElse(false));
                  task.getSource().set("Robots.java");
                  task.getWriterFiles()
                      .set(List.of("config.json", "model.glb", "model_*.glb", "GENERATED.md"));
                  task.getCommitted().set(extension.getFolder());
                  task.getGenerated()
                      .set(
                          project
                              .getLayout()
                              .getBuildDirectory()
                              .dir("tool-configs/advantagescope-assets"));
                  task.getSidecar()
                      .set(
                          project
                              .getLayout()
                              .getProjectDirectory()
                              .file(".gradle/tool-configs/advantagescope-assets.sha256"));
                });
    project.getTasks().named("build").configure(build -> build.dependsOn(assets));
  }
}
