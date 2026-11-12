package com.michaelgrundvig.frc.runtime.coprocessors;

import static org.assertj.core.api.Assertions.assertThat;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * What the robot keeps of Visor's configurations, on its disk: read as the program starts, written
 * whole whenever it changes. A file that's missing or can't be read is nothing kept, and a write
 * that fails is said, never thrown. Each write runs at once here, as the robot's writing thread
 * would run it.
 */
class VisorMemoryDiskTest {
  @TempDir Path folder;

  private static final Map<String, VisorMemory.Kept> KEPT =
      Map.of(
          "10.12.34.11",
          new VisorMemory.Kept(0x6a5d27df, List.of(0xa6738b08, 0x6a5d27df)),
          "visor-left.local:5808",
          new VisorMemory.Kept(0xe3b0c442, List.of()));

  @Test
  void whatsKeptIsRecalledByTheNextProgram() {
    new VisorMemoryDisk(folder, Runnable::run).keep(KEPT);

    VisorMemoryDisk restarted = new VisorMemoryDisk(folder, Runnable::run);

    assertThat(restarted.recalled()).isEqualTo(KEPT);
    assertThat(restarted.problem()).isEmpty();
    assertThat(folder.resolve(VisorMemoryDisk.FILE + ".new")).as("moved into place").doesNotExist();
  }

  /** A file's text, and what's recalled from it: nothing, unless it's whole. */
  private record Read(String why, String text, Map<String, VisorMemory.Kept> recalled) {}

  @Test
  void aFileThatsMissingOrCantBeReadRecallsNothing() throws Exception {
    List<Read> rows =
        List.of(
            new Read("no file", "", Map.of()),
            new Read("cut off", "10.12.34.11.committed=6a5d\n", Map.of()),
            new Read("not hex", "10.12.34.11.committed=nothexid\n", Map.of()),
            new Read("an ID that isn't one", "b.committed=6a5d27df\nb.ids=6a5d27df,zz\n", Map.of()),
            new Read(
                "more IDs than a board keeps: the first four",
                "b.committed=00000001\nb.ids=00000001,00000002,00000003,00000004,00000005\n",
                Map.of("b", new VisorMemory.Kept(1, List.of(1, 2, 3, 4)))));
    for (Read row : rows) {
      Path file = folder.resolve(VisorMemoryDisk.FILE);
      Files.deleteIfExists(file);
      if (!row.text().isEmpty()) {
        Files.writeString(file, row.text());
      }
      assertThat(new VisorMemoryDisk(folder, Runnable::run).recalled())
          .as(row.why())
          .isEqualTo(row.recalled());
    }
  }

  @Test
  void aWriteThatFailsIsSaidAndNeverThrown() throws Exception {
    // A file where the folder should be: nothing can be written inside it.
    Path notAFolder = Files.writeString(folder.resolve("in-the-way"), "");
    VisorMemoryDisk memory = new VisorMemoryDisk(notAFolder, Runnable::run);

    memory.keep(KEPT);

    assertThat(memory.problem()).startsWith(notAFolder.resolve(VisorMemoryDisk.FILE).toString());
    VisorMemoryDisk elsewhere = new VisorMemoryDisk(folder, Runnable::run);
    elsewhere.keep(KEPT);
    assertThat(elsewhere.problem()).isEmpty();
  }
}
