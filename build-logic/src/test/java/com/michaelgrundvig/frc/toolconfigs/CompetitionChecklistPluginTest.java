package com.michaelgrundvig.frc.toolconfigs;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.gradle.testkit.runner.BuildResult;
import org.gradle.testkit.runner.BuildTask;
import org.gradle.testkit.runner.GradleRunner;
import org.gradle.testkit.runner.TaskOutcome;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The checklist plugin in a real build (Gradle TestKit), on a small project standing in for the
 * robot's, whose stand-in writer has the real one's name: the build writes each robot's checklist
 * into {@code checklists/}, an edit there is written over, and an unchanged robot costs nothing.
 * How a writer's files are kept is the AdvantageScope plugin's tests' to show in full; this shows
 * the checklist's plugin runs its writer that way.
 */
class CompetitionChecklistPluginTest {
  @TempDir Path project;

  private static final String WRITER =
      """
      package com.michaelgrundvig.frc.toolconfigs.checklist;

      import java.nio.file.Files;
      import java.nio.file.Path;

      /** Stands in for the real writer: the robot's one station, in a checklist. */
      public final class CompetitionChecklist {
        public static void main(String... args) throws Exception {
          String out = null;
          for (String arg : args) {
            if (arg.startsWith("--out=")) out = arg.substring(6);
          }
          Files.createDirectories(Path.of(out));
          Files.writeString(Path.of(out, "comp.md"), "- [ ] " + fixture.Robots.STATION + "\\n");
        }
      }
      """;

  private void write(String path, String text) throws IOException {
    Path file = project.resolve(path);
    Files.createDirectories(file.getParent());
    Files.writeString(file, text);
  }

  private void station(String name) throws IOException {
    write(
        "src/main/java/fixture/Robots.java",
        "package fixture;\n\npublic final class Robots {\n"
            + "  public static final String STATION = \""
            + name
            + "\";\n}\n");
  }

  private GradleRunner gradle(String... arguments) {
    String[] all = new String[arguments.length + 1];
    System.arraycopy(arguments, 0, all, 0, arguments.length);
    all[arguments.length] = "--no-watch-fs";
    return GradleRunner.create()
        .withProjectDir(project.toFile())
        .withTestKitDir(new File(System.getProperty("frc.testkit.dir", "build/testkit")))
        .withPluginClasspath()
        .withArguments(all);
  }

  private static TaskOutcome outcome(BuildResult result) {
    BuildTask task = result.task(":competitionChecklist");
    assertThat(task).as("competitionChecklist ran\n" + result.getOutput()).isNotNull();
    return task.getOutcome();
  }

  private String checklist() throws IOException {
    return Files.readString(project.resolve("checklists/comp.md"));
  }

  @Test
  void theBuildWritesEachChecklistAndWritesOverAnEdit() throws IOException {
    write("settings.gradle", "rootProject.name = 'fixture'\n");
    write(
        "gradle.properties",
        "org.gradle.jvmargs=-Xmx256m -XX:MaxMetaspaceSize=256m\n"
            + "org.gradle.daemon.idletimeout=30000\n");
    write(
        "build.gradle",
        """
        plugins {
            id "java"
            id "frc.competition-checklist"
        }
        competitionChecklist {
            catalog = "fixture.Robots"
        }
        """);
    write(
        "src/main/java/com/michaelgrundvig/frc/toolconfigs/checklist/CompetitionChecklist.java",
        WRITER);
    station("front-left");

    assertThat(outcome(gradle("build").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(checklist()).isEqualTo("- [ ] front-left\n");
    assertThat(outcome(gradle("build").build())).as("unchanged").isEqualTo(TaskOutcome.UP_TO_DATE);

    write("checklists/comp.md", "- [x] front-left\n");
    gradle("build").build();
    assertThat(checklist()).as("an edit written over").isEqualTo("- [ ] front-left\n");

    station("back");
    gradle("build").build();
    assertThat(checklist()).as("the robot changed").isEqualTo("- [ ] back\n");
  }
}
