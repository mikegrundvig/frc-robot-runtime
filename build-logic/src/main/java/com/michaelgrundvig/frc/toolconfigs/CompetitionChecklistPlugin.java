package com.michaelgrundvig.frc.toolconfigs;

import com.michaelgrundvig.frc.toolconfigs.checklist.CompetitionChecklist;
import java.util.List;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.plugins.JavaPlugin;
import org.gradle.api.provider.ListProperty;
import org.gradle.api.provider.Property;

/**
 * {@code frc.competition-checklist}: each robot's pre-competition checklist ({@code checklists/}),
 * generated from its definition and checked against what's committed, as part of every build: which
 * camera goes in each station, and what to check in the pit and on the field. A project that
 * doesn't want it leaves the plugin out, and nothing of it runs.
 *
 * <pre>{@code
 * plugins { id "frc.competition-checklist" }
 * competitionChecklist {
 *     robots = ["competition"] // which robots; by default every one in Robots.all()
 * }
 * }</pre>
 *
 * <p>The checklists are plain generated files: the build writes over an edit. Its task, {@code
 * competitionChecklist}, runs the writer ({@link CompetitionChecklist}) on the compiled program
 * ({@link WriterTask} says how it's run and wired).
 */
public final class CompetitionChecklistPlugin implements Plugin<Project> {
  /** The writer the task runs: the plugin's own, in its jar. */
  static final Class<CompetitionChecklist> WRITER = CompetitionChecklist.class;

  /** The writer's heap: it builds a few records and writes a few kilobytes of text. */
  static final String HEAP = "32m";

  /** What the build sees of the plugin: {@code competitionChecklist { ... }}. */
  public abstract static class Extension {
    /**
     * The robots whose checklists are written, by name, as {@code Robots.all()} names them: one
     * file each. Empty, the default, is every robot in {@code Robots.all()}.
     */
    public abstract ListProperty<String> getRobots();

    /**
     * The robot program's catalog of its robots, a {@code
     * com.michaelgrundvig.frc.world.tools.RobotCatalog}: {@code frc.robot.ToolRobots} by default.
     */
    public abstract Property<String> getCatalog();

    /** Where the checklists are kept, committed: {@code checklists/} by default. */
    public abstract DirectoryProperty getFolder();
  }

  @Override
  public void apply(Project project) {
    project.getPluginManager().apply(JavaPlugin.class);
    Extension extension = project.getExtensions().create("competitionChecklist", Extension.class);
    extension.getRobots().convention(List.of());
    extension.getCatalog().convention("frc.robot.ToolRobots");
    extension.getFolder().convention(project.getLayout().getProjectDirectory().dir("checklists"));

    WriterTask.register(
        project,
        new WriterTask.Writer(
            "competitionChecklist",
            "Generate each robot's pre-competition checklist (checklists/) from Robots.java.",
            WRITER,
            HEAP,
            List.of("*.md"),
            "competition-checklist"),
        extension.getCatalog(),
        extension.getRobots(),
        extension.getFolder());
  }
}
