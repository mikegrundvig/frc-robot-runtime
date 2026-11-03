package com.michaelgrundvig.frc.toolconfigs;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.PosixFilePermissions;
import java.util.ArrayList;
import java.util.List;
import java.util.zip.GZIPOutputStream;
import org.apache.sshd.server.SshServer;
import org.apache.sshd.server.keyprovider.SimpleGeneratorHostKeyProvider;
import org.apache.sshd.server.shell.ProcessShellFactory;
import org.apache.sshd.sftp.server.SftpSubsystemFactory;
import org.gradle.testkit.runner.BuildResult;
import org.gradle.testkit.runner.GradleRunner;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

/**
 * The plugin in a real build (Gradle TestKit), deploying over SSH to a coprocessor that's a server
 * in this test (Apache MINA's, with SFTP and a shell): a small project stands in for the robot's,
 * its "writer" writing one board's configuration from a line compiled into its "Robots", and a
 * shell script stands in for Visor's program, refusing a configuration that says "bad". The board
 * already runs an old program and configuration, as it would after the last deploy.
 */
class VisorDeployPluginTest {
  @TempDir Path project;
  @TempDir Path board;

  private SshServer server;

  /** Every command the board was asked to run, in order. */
  private final List<String> commands = new ArrayList<>();

  private static final String WRITER =
      """
      package com.michaelgrundvig.frc.runtime.deploy;

      import java.nio.file.Files;
      import java.nio.file.Path;

      /** Stands in for the runtime's writer: vision-a's file, from the robot's one line. */
      public final class VisorConfigs {
        public static void main(String... args) throws Exception {
          String out = null;
          for (String arg : args) {
            if (arg.startsWith("--out=")) out = arg.substring(6);
          }
          Path folder = Path.of(out, "vision-a");
          Files.createDirectories(folder);
          Files.writeString(folder.resolve("visor.toml"), fixture.Robots.CONFIG + "\\n");
        }
      }
      """;

  /** Visor's program, as far as a deploy sees it: it checks a configuration. */
  private static final String PROGRAM =
      """
      #!/bin/sh
      # The new program, checking the new configuration as `visor check` does.
      if [ "$1" = check ] && grep -q bad "$3"; then
        echo "$3: no, this is bad"
        exit 1
      fi
      echo "checked $3"
      """;

  @BeforeEach
  void setUp() throws IOException {
    server = SshServer.setUpDefaultServer();
    server.setHost("127.0.0.1");
    server.setPort(0);
    server.setKeyPairProvider(new SimpleGeneratorHostKeyProvider(project.resolve("host-key")));
    server.setPasswordAuthenticator(
        (user, password, session) -> user.equals("visor") && password.equals("secret"));
    server.setSubsystemFactories(List.of(new SftpSubsystemFactory()));
    server.setCommandFactory(
        (channel, command) -> {
          synchronized (commands) {
            commands.add(command);
          }
          return new ProcessShellFactory(command, "/bin/sh", "-c", command).createShell(channel);
        });
    server.start();

    // What the board runs today.
    Files.writeString(board.resolve("visor"), "old program\n");
    Files.writeString(board.resolve("visor.toml"), "old configuration\n");

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
            id "org.wpilib.DeployUtils"
            id "frc.visor-deploy"
        }
        repositories {
            maven { url = file("repo") }
        }
        dependencies {
            implementation "com.michaelgrundvig.frc:frc-visor-client:2099.1.2"
        }
        visor {
            team = 1234
            catalog = "fixture.Robots"
            directory = "%s"
            password = "secret"
            coprocessors {
                "vision-a" { address = "127.0.0.1:%d" }
                // None of the robot's cameras is on it, so it's never reached: nothing listens there.
                "vision-spare" { address = teamAddress(99) }
            }
        }
        """
            .formatted(board, server.getPort()));
    write("src/main/java/com/michaelgrundvig/frc/runtime/deploy/VisorConfigs.java", WRITER);
    robot("team = 1234");
    Path program = write("visor-built-here", PROGRAM);
    Files.setPosixFilePermissions(program, PosixFilePermissions.fromString("rwxr-xr-x"));
    released("2099.1.2");
  }

  @AfterEach
  void tearDown() throws IOException {
    server.stop(true);
  }

  @Test
  void aDeployPutsTheNewProgramAndConfigurationInPlace() throws IOException {
    BuildResult result = gradle("deploy", "-PvisorProgram=" + project.resolve("visor-built-here"));

    assertThat(board.resolve("visor")).content().isEqualTo(PROGRAM);
    assertThat(board.resolve("visor.toml")).content().isEqualTo("team = 1234\n");
    assertThat(board.resolve("visor.new")).doesNotExist();
    assertThat(board.resolve("visor.toml.new")).doesNotExist();
    assertThat(commands)
        .anySatisfy(command -> assertThat(command).contains("check --config visor.toml.new"));
    assertThat(result.getOutput()).doesNotContain("vision-spare refused");
  }

  @Test
  void aConfigurationTheNewProgramRefusesLeavesTheBoardAsItWas() throws IOException {
    robot("bad = true");

    BuildResult result =
        runner("deploy", "-PvisorProgram=" + project.resolve("visor-built-here")).buildAndFail();

    assertThat(result.getOutput())
        .contains("Visor on vision-a refused the new configuration, and runs what it had")
        .contains("no, this is bad");
    assertThat(board.resolve("visor")).content().isEqualTo("old program\n");
    assertThat(board.resolve("visor.toml")).content().isEqualTo("old configuration\n");
  }

  @Test
  void theReleasedProgramIsTheOneForTheRobotsVisorClient() throws IOException {
    gradle("visorStage");

    Path staged = project.resolve("build/visor/staged/vision-a");
    assertThat(staged.resolve("visor.new")).content().isEqualTo("program 2099.1.2\n");
    assertThat(staged.resolve("visor.toml.new")).content().isEqualTo("team = 1234\n");
    assertThat(project.resolve("build/visor/staged/vision-spare")).doesNotExist();
  }

  /** The robot's one line, which the writer puts in vision-a's configuration. */
  private void robot(String config) throws IOException {
    write(
        "src/main/java/fixture/Robots.java",
        "package fixture;\n\npublic final class Robots {\n  public static final String CONFIG = \""
            + config.replace("\"", "\\\"")
            + "\";\n}\n");
  }

  /**
   * A release in the project's own repository, as the FRC libraries' holds one: the client, and
   * beside it the program at the same version, a tarball as the release workflow makes it.
   */
  private void released(String version) throws IOException {
    String group = "repo/com/michaelgrundvig/frc/";
    write(
        group + "frc-visor-client/" + version + "/frc-visor-client-" + version + ".pom",
        pom("frc-visor-client", version, "jar"));
    Files.write(
        project.resolve(
            group + "frc-visor-client/" + version + "/frc-visor-client-" + version + ".jar"),
        emptyJar());
    write(group + "visor/" + version + "/visor-" + version + ".pom", pom("visor", version, "pom"));
    Files.write(
        project.resolve(group + "visor/" + version + "/visor-" + version + "-linux-arm64.tar.gz"),
        tarGz("visor", "program " + version + "\n"));
  }

  private static String pom(String artifact, String version, String packaging) {
    return """
        <?xml version="1.0" encoding="UTF-8"?>
        <project xmlns="http://maven.apache.org/POM/4.0.0">
          <modelVersion>4.0.0</modelVersion>
          <groupId>com.michaelgrundvig.frc</groupId>
          <artifactId>%s</artifactId>
          <version>%s</version>
          <packaging>%s</packaging>
        </project>
        """
        .formatted(artifact, version, packaging);
  }

  private static byte[] emptyJar() throws IOException {
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (var jar = new java.util.jar.JarOutputStream(bytes)) {
      jar.putNextEntry(new java.util.jar.JarEntry("META-INF/"));
    }
    return bytes.toByteArray();
  }

  /**
   * A gzipped tar of one file, written by hand: a 512-byte header, the content, padding, the end.
   */
  private static byte[] tarGz(String name, String content) throws IOException {
    byte[] data = content.getBytes(StandardCharsets.UTF_8);
    byte[] header = new byte[512];
    put(header, 0, name);
    put(header, 100, "0000755");
    put(header, 108, "0000000");
    put(header, 116, "0000000");
    put(header, 124, String.format("%011o", data.length));
    put(header, 136, "00000000000");
    header[156] = '0';
    put(header, 257, "ustar");
    put(header, 263, "00");
    for (int i = 148; i < 156; i++) {
      header[i] = ' ';
    }
    int sum = 0;
    for (byte b : header) {
      sum += b & 0xff;
    }
    put(header, 148, String.format("%06o", sum));
    header[154] = 0;
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (OutputStream out = new GZIPOutputStream(bytes)) {
      out.write(header);
      out.write(data);
      out.write(new byte[(512 - data.length % 512) % 512]);
      out.write(new byte[1024]);
    }
    return bytes.toByteArray();
  }

  private static void put(byte[] header, int at, String text) {
    byte[] bytes = text.getBytes(StandardCharsets.US_ASCII);
    System.arraycopy(bytes, 0, header, at, bytes.length);
  }

  private BuildResult gradle(String... arguments) {
    return runner(arguments).build();
  }

  private GradleRunner runner(String... arguments) {
    List<String> all = new ArrayList<>(List.of(arguments));
    all.add("--stacktrace");
    return GradleRunner.create()
        .withProjectDir(project.toFile())
        .withTestKitDir(new File(System.getProperty("frc.testkit.dir")))
        .withPluginClasspath()
        .withArguments(all)
        .forwardOutput();
  }

  private Path write(String path, String text) throws IOException {
    Path file = project.resolve(path);
    Files.createDirectories(file.getParent());
    Files.writeString(file, text);
    return file;
  }
}
