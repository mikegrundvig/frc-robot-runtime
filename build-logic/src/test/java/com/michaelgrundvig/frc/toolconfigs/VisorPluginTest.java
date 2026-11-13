package com.michaelgrundvig.frc.toolconfigs;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
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
 * The plugin in a real build (Gradle TestKit): a small project standing in for the robot's, built
 * with a Visor client from a repository of its own that also holds that client's release, and
 * deployed over SSH to a robot controller that's a server in this test (Apache MINA's, with SFTP
 * and a shell). The release reaches the controller's deploy folder, and its Spotter pack beside it.
 */
class VisorPluginTest {
  @TempDir Path project;
  @TempDir Path controller;

  private SshServer server;

  @BeforeEach
  void setUp() throws Exception {
    server = SshServer.setUpDefaultServer();
    server.setHost("127.0.0.1");
    server.setPort(0);
    server.setKeyPairProvider(new SimpleGeneratorHostKeyProvider(project.resolve("host-key")));
    server.setPasswordAuthenticator(
        (user, password, session) -> user.equals("systemcore") && password.equals("secret"));
    server.setSubsystemFactories(List.of(new SftpSubsystemFactory()));
    server.setCommandFactory((channel, command) ->
        new ProcessShellFactory(command, "/bin/sh", "-c", command).createShell(channel));
    server.start();

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
            id "frc.visor"
        }
        repositories {
            maven { url = file("repo") }
        }
        dependencies {
            implementation "com.michaelgrundvig.frc:frc-visor-client:2099.1.2"
        }
        // The robot controller, as GradleRIO's systemcore target is.
        deploy {
            targets {
                systemcore(getTargetTypeClass("RemoteTarget")) {
                    directory = "%s"
                    locations {
                        ssh(getLocationTypeClass("SshDeployLocation")) {
                            address = "127.0.0.1:%d"
                            user = "systemcore"
                            password = "secret"
                        }
                    }
                }
            }
        }
        visor {
            deployDirectory = "%s/deploy"
        }
        """
            .formatted(controller, server.getPort(), controller));
    released("2099.1.2", Hash.ITS_OWN);
  }

  /** What the client's jar says of its release's SHA-256. */
  private enum Hash {
    /** The archive's own, as a release publishes the client. */
    ITS_OWN,
    /** Another archive's: the one in the repository was replaced. */
    ANOTHERS,
    /** None: a client built without its release. */
    NONE
  }

  @AfterEach
  void tearDown() throws IOException {
    server.stop(true);
  }

  @Test
  void theRobotsDeployCarriesTheReleaseForItsVisorClientAndItsPack() throws Exception {
    gradle("deploy");

    Path deploy = controller.resolve("deploy");
    assertThat(deploy.resolve("visor/visor-2099.1.2-linux-arm64.tar.gz"))
        .hasSameBinaryContentAs(
            project.resolve(
                "repo/com/michaelgrundvig/frc/visor/2099.1.2/visor-2099.1.2-linux-arm64.tar.gz"));
    assertThat(deploy.resolve("spotter-packs/visor/pack.yaml"))
        .content()
        .isEqualTo("pack: visor\n");
    assertThat(deploy.resolve("spotter-packs/visor/status")).content().startsWith("#!/bin/sh");
    assertThat(deploy.resolve("spotter-packs/visor/visor")).as("only the pack").doesNotExist();
  }

  @Test
  void everyBuildStagesItSoALaptopThatBuiltCanDeployOffline() throws Exception {
    BuildResult result = gradle("assemble");

    assertThat(result.getOutput()).contains(":visorStage");
    assertThat(project.resolve("build/visor/deploy/visor/visor-2099.1.2-linux-arm64.tar.gz"))
        .exists();
    assertThat(project.resolve("build/visor/deploy/spotter-packs/visor/pack.yaml")).exists();
  }

  /** An archive the client wasn't published with is never staged, nor deployed. */
  @Test
  void aReleaseWhoseHashIsntTheClientsIsRefused() throws Exception {
    record Row(String why, Hash hash, String says) {}
    List<Row> rows =
        List.of(
            new Row(
                "the archive in the repository was replaced",
                Hash.ANOTHERS,
                "its Visor client was published with: it isn't that release, so it isn't staged"),
            new Row(
                "a client that names no hash",
                Hash.NONE,
                "names no SHA-256 for its release, so visor-2099.1.2-linux-arm64.tar.gz can't be"
                    + " checked"));
    for (Row row : rows) {
      released("2099.1.2", row.hash());

      BuildResult result = runner("assemble").buildAndFail();

      assertThat(result.getOutput()).as(row.why()).contains(row.says());
      assertThat(project.resolve("build/visor/deploy/visor")).as(row.why()).doesNotExist();
    }
  }

  @Test
  void aTargetTheDeployDoesntHaveIsNamed() throws Exception {
    write(
        "build.gradle",
        Files.readString(project.resolve("build.gradle")) + "visor { target = \"roborio\" }\n");

    BuildResult result = runner("tasks").buildAndFail();

    assertThat(result.getOutput())
        .contains("visor { target = ... } names roborio, which isn't one of the deploy's targets");
  }

  /**
   * A release in the project's own repository, as the FRC libraries' holds one: the client, naming
   * its release's SHA-256 as the hash says, and beside it its release, an archive as the release
   * workflow makes one, the program and the pack.
   */
  private void released(String version, Hash hash) throws Exception {
    String group = "repo/com/michaelgrundvig/frc/";
    write(group + "visor/" + version + "/visor-" + version + ".pom", pom("visor", version, "pom"));
    Path archive =
        project.resolve(group + "visor/" + version + "/visor-" + version + "-linux-arm64.tar.gz");
    archive(version, archive);
    String named =
        switch (hash) {
          case ITS_OWN -> VisorReleaseCheck.sha256(archive.toFile());
          case ANOTHERS -> "0".repeat(64);
          case NONE -> "";
        };
    write(
        group + "frc-visor-client/" + version + "/frc-visor-client-" + version + ".pom",
        pom("frc-visor-client", version, "jar"));
    Files.write(
        project.resolve(
            group + "frc-visor-client/" + version + "/frc-visor-client-" + version + ".jar"),
        clientJar(version, named));
  }

  /** A release's archive, as the release workflow makes one: the program and the pack. */
  private void archive(String version, Path archive) throws Exception {
    Path contents = Files.createDirectories(project.resolve("release"));
    Files.writeString(contents.resolve("visor"), "program " + version + "\n");
    Files.createDirectories(contents.resolve("spotter/visor"));
    Files.writeString(contents.resolve("spotter/visor/pack.yaml"), "pack: visor\n");
    Files.writeString(contents.resolve("spotter/visor/status"), "#!/bin/sh\necho {}\n");
    Process tar =
        new ProcessBuilder("tar", "-czf", archive.toString(), "-C", contents.toString(), ".")
            .inheritIO()
            .start();
    assertThat(tar.waitFor()).isZero();
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

  /** The client's jar: only what its build writes of its version and its release's SHA-256. */
  private static byte[] clientJar(String version, String releaseSha256) throws IOException {
    ByteArrayOutputStream bytes = new ByteArrayOutputStream();
    try (var jar = new java.util.jar.JarOutputStream(bytes)) {
      jar.putNextEntry(new java.util.jar.JarEntry("META-INF/"));
      jar.putNextEntry(new java.util.jar.JarEntry(VisorReleaseCheck.RESOURCE));
      String properties = "version=" + version + "\n" + VisorReleaseCheck.KEY + "=" + releaseSha256;
      jar.write((properties + "\n").getBytes(java.nio.charset.StandardCharsets.UTF_8));
    }
    return bytes.toByteArray();
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
