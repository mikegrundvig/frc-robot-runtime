package com.michaelgrundvig.frc.toolconfigs;

import com.michaelgrundvig.frc.toolconfigs.advantagescope.AdvantageScopeAssets;
import java.util.List;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.plugins.JavaPlugin;
import org.gradle.api.provider.ListProperty;
import org.gradle.api.provider.Property;

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
 * <p>Its task, {@code advantageScopeAssets}, runs the writer ({@link AdvantageScopeAssets}) on the
 * compiled program, which only builds the robots' configuration records and writes files: no robot
 * program, HAL, NetworkTables or simulator ({@link WriterTask} says how it's run and wired).
 */
public final class AdvantageScopeAssetsPlugin implements Plugin<Project> {
  /** The writer the task runs: the plugin's own, in its jar. */
  static final Class<AdvantageScopeAssets> WRITER = AdvantageScopeAssets.class;

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
     * com.michaelgrundvig.frc.world.tools.RobotCatalog}: {@code frc.robot.ToolRobots} by default.
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

    WriterTask.register(
        project,
        new WriterTask.Writer(
            "advantageScopeAssets",
            "Generate AdvantageScope's robot assets (ascope_assets/) from Robots.java.",
            WRITER,
            HEAP,
            List.of("config.json", "model.glb", "model_*.glb", "GENERATED.md"),
            "advantagescope-assets"),
        extension.getCatalog(),
        extension.getRobots(),
        extension.getFolder());
  }
}
