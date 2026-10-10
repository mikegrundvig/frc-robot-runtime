package com.michaelgrundvig.frc.toolconfigs;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.gradle.testkit.runner.BuildResult;
import org.gradle.testkit.runner.BuildTask;
import org.gradle.testkit.runner.GradleRunner;
import org.gradle.testkit.runner.TaskOutcome;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The plugin in a real build (Gradle TestKit), on a small project standing in for the robot's: its
 * "writer" writes a config.json and a model.glb from a mass compiled into its "Robots", as the real
 * writer writes from Robots.java. Each behavior the build promises, as a team meets it.
 */
class AdvantageScopeAssetsPluginTest {
  @TempDir Path project;

  private static final String WRITER =
      """
      package com.michaelgrundvig.frc.world.bridges;

      import java.nio.file.Files;
      import java.nio.file.Path;
      import java.util.ArrayList;
      import java.util.List;

      /** Stands in for the real writer: the robot's mass, in a config and a model. */
      public final class AdvantageScopeAssets {
        public static void main(String... args) throws Exception {
          String out = null;
          List<String> robots = new ArrayList<>();
          for (String arg : args) {
            if (arg.startsWith("--out=")) out = arg.substring(6);
            if (arg.startsWith("--robot=")) robots.add(arg.substring(8));
          }
          if (robots.isEmpty()) robots.add(fixture.Robots.CURRENT);
          for (String robot : robots) {
            Path folder = Path.of(out, "Robot_" + robot);
            Files.createDirectories(folder);
            Files.writeString(folder.resolve("config.json"), config(robot));
            Files.write(folder.resolve("model.glb"), new byte[] {1, 2, (byte) fixture.Robots.MASS});
          }
        }

        public static String config(String robot) {
          return "{\\n  \\"name\\": \\"" + robot + "\\",\\n  \\"mass\\": " + fixture.Robots.MASS + "\\n}\\n";
        }
      }
      """;

  @BeforeEach
  void setUp() throws IOException {
    write("settings.gradle", "rootProject.name = 'fixture'\n");
    // TestKit's daemon: small, and gone soon after the tests.
    write(
        "gradle.properties",
        "org.gradle.jvmargs=-Xmx256m -XX:MaxMetaspaceSize=256m\n"
            + "org.gradle.daemon.idletimeout=30000\n");
    write(
        "build.gradle",
        """
        plugins {
            id "java"
            id "frc.advantagescope-assets"
        }
        advantageScope {
            catalog = "fixture.Robots"
        }
        """);
    write("src/main/java/com/michaelgrundvig/frc/world/bridges/AdvantageScopeAssets.java", WRITER);
    mass("61.0");
    stamp("first");
  }

  /** The robot's mass, as Robots.java would set it. */
  private void mass(String kilograms) throws IOException {
    write(
        "src/main/java/fixture/Robots.java",
        "package fixture;\n\npublic final class Robots {\n"
            + "  public static final String CURRENT = \"comp\";\n"
            + "  public static final double MASS = "
            + kilograms
            + ";\n}\n");
  }

  /** The build's stamp, compiled anew by every build, as the robot program's BuildConstants is. */
  private void stamp(String when) throws IOException {
    write(
        "src/main/java/frc/robot/BuildConstants.java",
        "package frc.robot;\n\npublic final class BuildConstants {\n"
            + "  public static final String BUILD_DATE = \""
            + when
            + "\";\n}\n");
  }

  private void write(String path, String text) throws IOException {
    Path file = project.resolve(path);
    Files.createDirectories(file.getParent());
    Files.writeString(file, text);
  }

  private String config() throws IOException {
    return Files.readString(project.resolve("ascope_assets/Robot_comp/config.json"));
  }

  /**
   * A build of the fixture project. Each build reads the project's files from disk: these tests
   * change and delete files between builds that follow within milliseconds, and a daemon that
   * watches the file system can run the next build before it has heard of the change. On a busy
   * machine it then calls the task up to date when a file it wrote is gone.
   */
  private GradleRunner gradle(String... arguments) {
    List<String> all = new ArrayList<>(List.of(arguments));
    all.add("--no-watch-fs");
    return GradleRunner.create()
        .withProjectDir(project.toFile())
        .withTestKitDir(new File(System.getProperty("frc.testkit.dir", "build/testkit")))
        .withPluginClasspath()
        .withArguments(all);
  }

  private static TaskOutcome outcome(BuildResult result) {
    BuildTask task = result.task(":advantageScopeAssets");
    assertThat(task).as("advantageScopeAssets ran\n" + result.getOutput()).isNotNull();
    return task.getOutcome();
  }

  @Test
  void theBuildGeneratesThemAndAnUnchangedModelCostsNothing() throws IOException {
    assertThat(outcome(gradle("build").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).isEqualTo("{\n  \"name\": \"comp\",\n  \"mass\": 61.0\n}\n");

    stamp("second"); // a new build's stamp isn't a new model
    assertThat(outcome(gradle("build").build())).isEqualTo(TaskOutcome.UP_TO_DATE);

    BuildResult tests = gradle("test").build();
    assertThat(tests.task(":advantageScopeAssets")).as("not part of test").isNull();
  }

  @Test
  void aModelChangeRewritesTheFilesSilently() throws IOException {
    gradle("advantageScopeAssets").build();

    mass("67.8");
    BuildResult result = gradle("advantageScopeAssets").build();

    assertThat(outcome(result)).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 67.8");
    assertThat(result.getOutput()).doesNotContain("Robot_comp");
  }

  /** Nobody edits AdvantageScope's assets with a tool: the build writes over an edit. */
  @Test
  void anEditOutsideTheBuildIsWrittenOver() throws IOException {
    gradle("advantageScopeAssets").build();
    write(
        "ascope_assets/Robot_comp/config.json", "{\n  \"name\": \"comp\",\n  \"mass\": 59.5\n}\n");

    assertThat(outcome(gradle("build").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 61.0");
  }

  /** A merge conflict resolved with the other side's files, older than the merged model. */
  @Test
  void aMergeResolvedWithEitherSideIsWrittenFromTheModel() throws IOException {
    gradle("advantageScopeAssets").build();
    mass("67.8");
    write(
        "ascope_assets/Robot_comp/config.json", "{\n  \"name\": \"comp\",\n  \"mass\": 64.0\n}\n");

    assertThat(outcome(gradle("advantageScopeAssets").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 67.8");
  }

  /** A clean doesn't make the build forget what it wrote: the record is in .gradle/, not build/. */
  @Test
  void aCleanBuildAfterChangingTheModelPasses() throws IOException {
    strict();
    gradle("advantageScopeAssets").build();

    mass("67.8");
    assertThat(outcome(gradle("clean", "build").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 67.8");
  }

  /** The strict check, for a plugin whose tool's GUI rewrites its file. */
  @Test
  void strictAnEditOutsideTheBuildFailsItNamingBothValuesAndTheTwoFixes() throws IOException {
    strict();
    gradle("advantageScopeAssets").build();
    write(
        "ascope_assets/Robot_comp/config.json", "{\n  \"name\": \"comp\",\n  \"mass\": 59.5\n}\n");

    BuildResult failed = gradle("build").buildAndFail();

    assertThat(failed.getOutput())
        .contains("ascope_assets/Robot_comp/config.json was edited outside the build")
        .contains("line 3 of the file:  \"mass\": 59.5")
        .contains("Robots.java gives:   \"mass\": 61.0")
        .contains("Change Robots.java if the file is right")
        .contains("./gradlew build -PdiscardToolEdits");
    assertThat(config()).as("the edit is kept").contains("59.5");

    assertThat(outcome(gradle("build", "-PdiscardToolEdits").build()))
        .isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 61.0");
  }

  /** Strict, on a fresh clone nothing says who changed what: a mismatch fails, a match passes. */
  @Test
  void strictWithoutARecordAMismatchFailsAndAMatchPasses() throws IOException {
    strict();
    gradle("advantageScopeAssets").build();
    forget();

    assertThat(outcome(gradle("advantageScopeAssets").build())).isEqualTo(TaskOutcome.SUCCESS);

    forget();
    write(
        "ascope_assets/Robot_comp/config.json", "{\n  \"name\": \"comp\",\n  \"mass\": 59.5\n}\n");
    BuildResult failed = gradle("advantageScopeAssets").buildAndFail();
    assertThat(failed.getOutput())
        .contains(
            "ascope_assets/Robot_comp/config.json differs from what Robots.java gives, and nothing"
                + " records what the build last wrote there (a fresh clone, or after a clean)");
  }

  /** As a fresh clone has it: no record of what the build wrote. */
  private void forget() throws IOException {
    Files.deleteIfExists(project.resolve(".gradle/tool-configs/advantagescope-assets.sha256"));
  }

  /** Makes the fixture's task strict, as a path tool's plugin would be. */
  private void strict() throws IOException {
    Files.writeString(
        project.resolve("build.gradle"),
        Files.readString(project.resolve("build.gradle"))
            + "tasks.named(\"advantageScopeAssets\") { strict = true }\n");
  }

  /** Someone else changed the model and built and committed its files: both arrive together. */
  @Test
  void aPullThatChangedBothSidesAlikePasses() throws IOException {
    gradle("advantageScopeAssets").build();

    mass("67.8");
    write(
        "ascope_assets/Robot_comp/config.json", "{\n  \"name\": \"comp\",\n  \"mass\": 67.8\n}\n");
    Files.write(project.resolve("ascope_assets/Robot_comp/model.glb"), new byte[] {1, 2, 67});
    BuildResult result = gradle("advantageScopeAssets").build();

    assertThat(outcome(result)).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("\"mass\": 67.8");
  }

  @Test
  void whatTheComputerLeavesInAFolderDoesntFailTheBuild() throws IOException {
    gradle("advantageScopeAssets").build();
    write("ascope_assets/Robot_comp/.DS_Store", "Finder's");
    write("ascope_assets/Robot_comp/Thumbs.db", "Explorer's");
    write("ascope_assets/Robot_comp/desktop.ini", "Explorer's");
    mass("67.8"); // so the check runs past them

    assertThat(outcome(gradle("advantageScopeAssets").build())).isEqualTo(TaskOutcome.SUCCESS);
    assertThat(config()).contains("67.8");
    assertThat(project.resolve("ascope_assets/Robot_comp/.DS_Store")).exists();
  }

  /** A robot renamed: its old folder doesn't linger in AdvantageScope's list of robots. */
  @Test
  void aRenamedRobotsOldFolderIsRemovedAndATeamsOwnLeftAlone() throws IOException {
    write("ascope_assets/Robot_Cad/config.json", "{\"name\": \"a team's own CAD robot\"}\n");
    gradle("advantageScopeAssets").build();

    write(
        "src/main/java/fixture/Robots.java",
        Files.readString(project.resolve("src/main/java/fixture/Robots.java"))
            .replace("\"comp\"", "\"competition\""));
    gradle("advantageScopeAssets").build();

    assertThat(project.resolve("ascope_assets/Robot_competition/config.json")).exists();
    assertThat(project.resolve("ascope_assets/Robot_comp")).doesNotExist();
    assertThat(project.resolve("ascope_assets/Robot_Cad/config.json")).exists();
  }

  @Test
  void eachRobotNamedGetsItsFolder() throws IOException {
    write(
        "build.gradle",
        """
        plugins {
            id "java"
            id "frc.advantagescope-assets"
        }
        advantageScope {
            catalog = "fixture.Robots"
            robots = ["competition", "practice"]
        }
        """);

    gradle("advantageScopeAssets").build();

    assertThat(project.resolve("ascope_assets/Robot_competition/config.json")).exists();
    assertThat(project.resolve("ascope_assets/Robot_practice/config.json")).exists();
    assertThat(project.resolve("ascope_assets/Robot_comp")).doesNotExist();
  }
}
