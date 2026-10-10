package com.michaelgrundvig.frc.toolconfigs;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The check itself, without Gradle: what's written, and, strict, what's refused and how it's said.
 */
class ToolConfigSyncTest {
  @TempDir Path dir;
  private Path generated;
  private Path committed;
  private ToolConfigSync sync;

  @BeforeEach
  void setUp() {
    generated = dir.resolve("generated");
    committed = dir.resolve("ascope_assets");
    sync =
        new ToolConfigSync(
            generated,
            committed,
            dir.resolve("sidecar"),
            "Robots.java",
            "ascope_assets",
            List.of("config.json", "model*.glb", "notes.txt"),
            true);
  }

  private ToolConfigSync plain() {
    return new ToolConfigSync(
        generated,
        committed,
        dir.resolve("sidecar"),
        "Robots.java",
        "ascope_assets",
        List.of("config.json", "model*.glb"),
        false);
  }

  private void generate(String path, String text) throws IOException {
    write(generated.resolve(path), text);
  }

  private static void write(Path file, String text) throws IOException {
    Files.createDirectories(file.getParent());
    Files.writeString(file, text);
  }

  private String committed(String path) throws IOException {
    return Files.readString(committed.resolve(path));
  }

  @Test
  void theFirstRunWritesAndASecondChangesNothing() throws IOException {
    generate("Robot_a/config.json", "{\n  \"mass\": 61\n}\n");

    assertThat(sync.run(false).written()).containsExactly("Robot_a/config.json: new");
    assertThat(committed("Robot_a/config.json")).isEqualTo("{\n  \"mass\": 61\n}\n");
    assertThat(sync.run(false).written()).isEmpty();
  }

  @Test
  void aModelChangeRewritesWhatTheBuildWrote() throws IOException {
    generate("Robot_a/config.json", "{\n  \"mass\": 61\n}\n");
    sync.run(false);
    generate("Robot_a/config.json", "{\n  \"mass\": 67.8\n}\n");

    ToolConfigSync.Outcome outcome = sync.run(false);

    assertThat(outcome.refused()).isEmpty();
    assertThat(outcome.written()).containsExactly("Robot_a/config.json: Robots.java changed");
    assertThat(committed("Robot_a/config.json")).contains("67.8");
  }

  @Test
  void anEditOutsideTheBuildIsRefusedNamingBothValuesAndTheTwoWaysOut() throws IOException {
    generate("Robot_a/config.json", "{\n  \"mass\": 61\n}\n");
    sync.run(false);
    write(committed.resolve("Robot_a/config.json"), "{\n  \"mass\": 59.5\n}\n");

    ToolConfigSync.Outcome outcome = sync.run(false);

    assertThat(outcome.written()).isEmpty();
    assertThat(outcome.refused())
        .containsExactly(
            """
            ascope_assets/Robot_a/config.json was edited outside the build, and differs from what \
            Robots.java gives.
              line 2 of the file:  "mass": 59.5
              Robots.java gives:   "mass": 61
            Change Robots.java if the file is right, or discard the edit: \
            ./gradlew build -PdiscardToolEdits""");
    assertThat(committed("Robot_a/config.json")).as("left as it was").contains("59.5");

    assertThat(sync.run(true).written()).containsExactly("Robot_a/config.json: an edit discarded");
    assertThat(committed("Robot_a/config.json")).contains("61");
  }

  @Test
  void withoutASidecarAMismatchIsRefusedAndAMatchRecorded() throws IOException {
    generate("Robot_a/config.json", "{\n  \"mass\": 61\n}\n");
    generate("Robot_a/model.glb", "binary");
    write(committed.resolve("Robot_a/config.json"), "{\n  \"mass\": 61\n}\n");
    write(committed.resolve("Robot_a/model.glb"), "binarY!");

    ToolConfigSync.Outcome outcome = sync.run(false);

    assertThat(outcome.refused())
        .singleElement()
        .asString()
        .startsWith(
            "ascope_assets/Robot_a/model.glb differs from what Robots.java gives, and nothing"
                + " records what the build last wrote there (a fresh clone, or after a clean)")
        .contains("the file's 7 bytes against the 6 Robots.java gives, apart from byte 5")
        .endsWith("./gradlew build -PdiscardToolEdits");
    assertThat(Files.exists(dir.resolve("sidecar"))).as("nothing written").isFalse();

    write(committed.resolve("Robot_a/model.glb"), "binary");
    assertThat(sync.run(false).written()).isEmpty();
    assertThat(Files.readString(dir.resolve("sidecar"), StandardCharsets.UTF_8))
        .contains("Robot_a/config.json=")
        .contains("Robot_a/model.glb=" + ToolConfigSync.hash("binary".getBytes()));
  }

  @Test
  void aFileTheModelNoLongerGivesIsRemovedOnlyIfTheBuildWroteIt() throws IOException {
    generate("Robot_a/model.glb", "base");
    generate("Robot_a/model_4.glb", "cones");
    sync.run(false);
    Files.delete(generated.resolve("Robot_a/model_4.glb"));
    write(committed.resolve("Robot_a/notes.txt"), "mine");
    write(committed.resolve("Robot_Cad/config.json"), "a team's own robot, untouched");

    ToolConfigSync.Outcome refused = sync.run(false);

    assertThat(refused.refused())
        .containsExactly(
            "ascope_assets/Robot_a/notes.txt is in a folder the build writes, but Robots.java"
                + " doesn't give it, and the build didn't write it.");
    Files.delete(committed.resolve("Robot_a/notes.txt"));
    assertThat(sync.run(false).written())
        .containsExactly("Robot_a/model_4.glb: removed, Robots.java no longer gives it");
    assertThat(Files.exists(committed.resolve("Robot_a/model_4.glb"))).isFalse();
    assertThat(Files.exists(committed.resolve("Robot_Cad/config.json"))).isTrue();
  }

  /** What a computer leaves in a folder is no one's edit, and stays. */
  @Test
  void filesTheWriterNeverWritesAreLeftAlone() throws IOException {
    generate("Robot_a/model.glb", "base");
    sync.run(false);
    for (String clutter : List.of(".DS_Store", "Thumbs.db", "desktop.ini")) {
      write(committed.resolve("Robot_a/" + clutter), "the computer's");
    }

    ToolConfigSync.Outcome outcome = sync.run(false);

    assertThat(outcome.refused()).isEmpty();
    assertThat(outcome.written()).isEmpty();
    assertThat(committed.resolve("Robot_a/.DS_Store")).exists();
  }

  /** Plain, the model's files are written over whatever is there, after an edit or a merge. */
  @Test
  void plainWritesOverAnEditAndRemovesWhatTheModelNoLongerGives() throws IOException {
    generate("Robot_a/config.json", "{\n  \"mass\": 61\n}\n");
    generate("Robot_a/model_4.glb", "cones");
    write(committed.resolve("Robot_a/config.json"), "{\n  \"mass\": 59.5\n}\n");
    write(committed.resolve("Robot_a/model_4.glb"), "someone else's cones");
    write(committed.resolve("Robot_a/model_5.glb"), "a stale component");

    ToolConfigSync.Outcome outcome = plain().run(false);

    assertThat(outcome.refused()).isEmpty();
    assertThat(outcome.written())
        .containsExactly(
            "Robot_a/config.json: written from Robots.java",
            "Robot_a/model_4.glb: written from Robots.java",
            "Robot_a/model_5.glb: removed, Robots.java no longer gives it");
    assertThat(committed("Robot_a/config.json")).contains("61");
    assertThat(committed.resolve("Robot_a/model_5.glb")).doesNotExist();
  }

  /** A robot renamed: its old folder goes, unless it holds something the build didn't write. */
  @Test
  void aFolderTheModelNoLongerGivesIsRemovedIfTheBuildWroteIt() throws IOException {
    generate("Robot_old/config.json", "old");
    generate("Robot_old/model.glb", "old model");
    generate("Robot_kept/config.json", "kept");
    plain().run(false);
    write(committed.resolve("Robot_old/.DS_Store"), "Finder's");
    write(committed.resolve("Robot_kept/notes.md"), "a team's own notes");
    write(committed.resolve("Robot_Cad/config.json"), "a team's own robot");
    deleteFolder(generated);
    generate("Robot_new/config.json", "new");

    ToolConfigSync.Outcome outcome = plain().run(false);

    assertThat(outcome.written())
        .contains(
            "Robot_new/config.json: new",
            "Robot_old/config.json: removed, Robots.java no longer gives its folder",
            "Robot_old/: removed, Robots.java no longer gives it",
            "Robot_kept/: kept, for the files in it the build didn't write");
    assertThat(committed.resolve("Robot_old")).doesNotExist();
    assertThat(committed.resolve("Robot_kept/config.json")).doesNotExist();
    assertThat(committed.resolve("Robot_kept/notes.md")).exists();
    assertThat(committed.resolve("Robot_Cad/config.json")).exists();
    assertThat(plain().run(false).written()).as("the record moved on").isEmpty();
  }

  /** Strict, an edited file in a folder the model no longer gives is refused, not removed. */
  @Test
  void strictAnEditedFileInAFolderTheModelDroppedIsRefused() throws IOException {
    generate("Robot_old/config.json", "old");
    sync.run(false);
    write(committed.resolve("Robot_old/config.json"), "edited");
    deleteFolder(generated);
    generate("Robot_new/config.json", "new");

    ToolConfigSync.Outcome outcome = sync.run(false);

    assertThat(outcome.refused())
        .containsExactly(
            "ascope_assets/Robot_old/config.json was edited outside the build, and Robots.java no"
                + " longer gives its folder. Delete it if the edit can go, or discard the edit:"
                + " ./gradlew build -PdiscardToolEdits");
    assertThat(committed.resolve("Robot_new")).as("nothing written").doesNotExist();
  }

  private static void deleteFolder(Path folder) throws IOException {
    try (Stream<Path> walk = Files.walk(folder)) {
      for (Path path : walk.sorted(Comparator.reverseOrder()).toList()) {
        Files.delete(path);
      }
    }
  }
}
