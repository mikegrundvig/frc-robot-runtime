package com.michaelgrundvig.frc.toolconfigs;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.gradle.api.Action;
import org.gradle.api.GradleException;
import org.gradle.api.NamedDomainObjectContainer;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.Configuration;
import org.gradle.api.artifacts.component.ModuleComponentIdentifier;
import org.gradle.api.file.Directory;
import org.gradle.api.file.DirectoryProperty;
import org.gradle.api.file.RegularFileProperty;
import org.gradle.api.plugins.JavaPlugin;
import org.gradle.api.plugins.JavaPluginExtension;
import org.gradle.api.provider.Property;
import org.gradle.api.provider.Provider;
import org.gradle.api.tasks.JavaExec;
import org.gradle.api.tasks.SourceSet;
import org.gradle.api.tasks.SourceSetContainer;
import org.gradle.api.tasks.Sync;
import org.gradle.api.tasks.TaskProvider;
import org.gradle.jvm.toolchain.JavaToolchainService;
import org.wpilib.deployutils.DeployUtils;
import org.wpilib.deployutils.deploy.CommandDeployResult;
import org.wpilib.deployutils.deploy.DeployExtension;
import org.wpilib.deployutils.deploy.artifact.FileTreeArtifact;
import org.wpilib.deployutils.deploy.context.DeployContext;
import org.wpilib.deployutils.deploy.target.RemoteTarget;
import org.wpilib.deployutils.deploy.target.location.SshDeployLocation;

/**
 * {@code frc.visor-deploy}: Visor, the AprilTag vision program, put on each of the robot's
 * coprocessors by the robot's own deploy ({@code ./gradlew deploy}), with a configuration written
 * from the robot's definition. A robot program and its coprocessors deploy together, so they never
 * run different versions or a configuration that's fallen behind the robot's.
 *
 * <pre>{@code
 * plugins { id "frc.visor-deploy" }
 * visor {
 *     team = deploy.targets.systemcore.team
 *     coprocessors {
 *         "vision-a" { address = teamAddress(11) }   // 10.TE.AM.11
 *         "vision-b" { address = teamAddress(12) }
 *     }
 * }
 * }</pre>
 *
 * <p>A camera says which board it's on in its definition ({@code
 * Software.visor().coprocessor("vision-a").device(...)}); the build says how to reach each board.
 * For each board, the deploy:
 *
 * <ol>
 *   <li>writes its configuration ({@code visorConfigs}: the runtime's {@code VisorConfigs} run on
 *       the compiled program, as the AdvantageScope plugin runs its writer), for the robot named
 *       with {@code -Probot=NAME}, or the only robot with cameras on Visor;
 *   <li>takes the program for the Visor client the robot is built with, from the FRC libraries'
 *       repository ({@code com.michaelgrundvig.frc:visor}, the arm64 build), or a program of one's
 *       own ({@code -PvisorProgram=<file>});
 *   <li>puts both beside the running ones ({@code visor.new}, {@code visor.toml.new}), has the new
 *       program check the new configuration ({@code visor check}), and only then moves them into
 *       place and stops the old program, which systemd starts again as the new one. A refused
 *       configuration fails the deploy and leaves the board running what it had.
 * </ol>
 *
 * <p>A board none of the robot's cameras is on gets nothing, and isn't connected to. One that can't
 * be reached fails the deploy, as the robot controller does: a coprocessor left on the last
 * deploy's configuration is the mistake this prevents.
 *
 * <p>The board's image provides the rest: a {@code visor} user who owns the directory ({@code
 * /opt/visor}, kept across boots on a read-only root) and logs in over SSH with the password the
 * build gives ({@code -PvisorPassword}, none unless given), and Visor's systemd unit, which runs
 * the program from there (frc-visor's {@code deploy/visor.service}).
 */
public final class VisorDeployPlugin implements Plugin<Project> {
  /** The writer the configuration task runs: the runtime's, on the program's runtime classpath. */
  static final String WRITER = "com.michaelgrundvig.frc.runtime.deploy.VisorConfigs";

  /** The Visor client the robot is built with, whose program goes on the coprocessors. */
  static final String CLIENT_GROUP = "com.michaelgrundvig.frc";

  static final String CLIENT_NAME = "frc-visor-client";

  /** The coprocessor program, beside the client at its version: the arm64 build. */
  static final String PROGRAM = "com.michaelgrundvig.frc:visor:%s:linux-arm64@tar.gz";

  /** What the build says of Visor's coprocessors: {@code visor { ... }}. */
  public abstract static class Extension {
    /**
     * The team number, which finds the robot controller: {@code deploy.targets.systemcore.team}.
     */
    public abstract Property<Integer> getTeam();

    /** The boards Visor runs on, each by the name the robot's cameras give it. */
    public abstract NamedDomainObjectContainer<Coprocessor> getCoprocessors();

    /** Where Visor lives on each board: {@code /opt/visor} unless given. */
    public abstract Property<String> getDirectory();

    /** The user the deploy logs in as, who owns the directory: {@code visor} unless given. */
    public abstract Property<String> getUser();

    /** Its password: {@code -PvisorPassword}, or none. */
    public abstract Property<String> getPassword();

    /**
     * The robot whose cameras are deployed, by name: {@code -Probot=NAME}, as the simulator takes
     * it. Unless given, the only robot with cameras on Visor.
     */
    public abstract Property<String> getRobot();

    /**
     * The robot program's catalog of its robots, a {@code
     * com.michaelgrundvig.frc.world.bridges.RobotCatalog}: {@code frc.robot.ToolRobots} by default.
     */
    public abstract Property<String> getCatalog();

    /**
     * The program to deploy in place of the released one: {@code -PvisorProgram=<file>}, for a
     * Visor built from its source.
     */
    public abstract RegularFileProperty getProgram();

    /** The address of the board numbered {@code host} on the team's network: 10.TE.AM.host. */
    public Provider<String> teamAddress(int host) {
      return getTeam().map(team -> "10." + team / 100 + "." + team % 100 + "." + host);
    }
  }

  /** One board Visor runs on. */
  public abstract static class Coprocessor {
    private final String name;

    @javax.inject.Inject
    public Coprocessor(String name) {
      this.name = name;
    }

    public String getName() {
      return name;
    }

    /** How the deploy reaches it: an address, or a name the laptop resolves; host:port works. */
    public abstract Property<String> getAddress();
  }

  @Override
  public void apply(Project project) {
    project.getPluginManager().apply(JavaPlugin.class);
    Extension visor = project.getExtensions().create("visor", Extension.class);
    visor.getDirectory().convention("/opt/visor");
    visor.getUser().convention("visor");
    visor
        .getPassword()
        .convention(project.getProviders().gradleProperty("visorPassword").orElse(""));
    visor.getRobot().convention(project.getProviders().gradleProperty("robot"));
    visor.getCatalog().convention("frc.robot.ToolRobots");
    visor
        .getProgram()
        .convention(
            project
                .getLayout()
                .file(project.getProviders().gradleProperty("visorProgram").map(File::new)));

    DirectoryProperty work = project.getObjects().directoryProperty();
    work.set(project.getLayout().getBuildDirectory().dir("visor"));
    Provider<Directory> configs = work.dir("configs");
    Provider<Directory> staged = work.dir("staged");

    SourceSet main = project.getExtensions().getByType(SourceSetContainer.class).getByName("main");
    TaskProvider<JavaExec> write =
        project
            .getTasks()
            .register(
                "visorConfigs",
                JavaExec.class,
                task -> {
                  task.setGroup("DeployUtils");
                  task.setDescription(
                      "Write Visor's configuration for each coprocessor, from the robot's definition.");
                  task.classpath(main.getRuntimeClasspath());
                  task.getMainClass().set(WRITER);
                  task.setMaxHeapSize("64m");
                  task.getJavaLauncher()
                      .convention(
                          project
                              .getExtensions()
                              .getByType(JavaToolchainService.class)
                              .launcherFor(
                                  project
                                      .getExtensions()
                                      .getByType(JavaPluginExtension.class)
                                      .getToolchain()));
                  task.getOutputs().dir(configs);
                  task.getArgumentProviders()
                      .add(
                          () -> {
                            if (!visor.getTeam().isPresent() || visor.getTeam().get() <= 0) {
                              throw new GradleException(
                                  "visor { team = ... } is needed to deploy Visor: set the team number first");
                            }
                            List<String> arguments = new ArrayList<>();
                            arguments.add("--catalog=" + visor.getCatalog().get());
                            arguments.add("--team=" + visor.getTeam().get());
                            arguments.add("--out=" + configs.get().getAsFile());
                            arguments.add("--robot=" + visor.getRobot().getOrElse(""));
                            for (Coprocessor board : visor.getCoprocessors()) {
                              arguments.add("--board=" + board.getName());
                            }
                            return arguments;
                          });
                });

    // The released program for the client on the robot's runtime classpath, at its version.
    Configuration released =
        project
            .getConfigurations()
            .create(
                "visorProgram",
                configuration -> {
                  configuration.setCanBeConsumed(false);
                  configuration.setTransitive(false);
                  configuration.setDescription(
                      "Visor's coprocessor program, at the version of the robot's Visor client.");
                });
    Configuration runtime =
        project.getConfigurations().getByName(main.getRuntimeClasspathConfigurationName());
    released.withDependencies(
        dependencies ->
            dependencies.add(
                project.getDependencies().create(String.format(PROGRAM, clientVersion(runtime)))));

    TaskProvider<Sync> stage =
        project
            .getTasks()
            .register(
                "visorStage",
                Sync.class,
                task -> {
                  task.setGroup("DeployUtils");
                  task.setDescription(
                      "Put each coprocessor's program and configuration together, to deploy.");
                  task.dependsOn(write);
                  task.into(staged);
                  for (Coprocessor board : visor.getCoprocessors()) {
                    Provider<File> config =
                        configs.map(
                            folder -> folder.file(board.getName() + "/visor.toml").getAsFile());
                    task.from(
                        config.map(file -> file.exists() ? List.of(file) : List.of()),
                        spec -> {
                          spec.into(board.getName());
                          spec.rename(name -> "visor.toml.new");
                        });
                    // The program only beside a configuration: a board with no camera gets nothing.
                    task.from(
                        project.provider(
                            () ->
                                config.get().exists()
                                    ? program(project, visor, released)
                                    : List.of()),
                        spec -> {
                          spec.into(board.getName());
                          spec.rename(name -> "visor.new");
                        });
                  }
                });

    // A board's target needs its address, which its block sets after the board is added: so once
    // the build script has said everything.
    project.afterEvaluate(
        evaluated -> {
          if (!project.getPlugins().hasPlugin(DeployUtils.class)) {
            return;
          }
          for (Coprocessor board : visor.getCoprocessors()) {
            target(project, visor, board, staged, stage);
          }
        });
  }

  /** The program: the one given, or the released one's, from its archive. */
  private static Object program(Project project, Extension visor, Configuration released) {
    if (visor.getProgram().isPresent()) {
      return project.files(visor.getProgram());
    }
    return project.tarTree(released.getSingleFile()).matching(only -> only.include("visor"));
  }

  /** The version of the Visor client on the robot's runtime classpath. */
  static String clientVersion(Configuration runtime) {
    Optional<String> version =
        runtime.getIncoming().getResolutionResult().getAllComponents().stream()
            .map(component -> component.getId())
            .filter(ModuleComponentIdentifier.class::isInstance)
            .map(ModuleComponentIdentifier.class::cast)
            .filter(id -> id.getGroup().equals(CLIENT_GROUP) && id.getModule().equals(CLIENT_NAME))
            .map(ModuleComponentIdentifier::getVersion)
            .findFirst();
    return version.orElseThrow(
        () ->
            new GradleException(
                "The robot isn't built with Visor's client ("
                    + CLIENT_GROUP
                    + ":"
                    + CLIENT_NAME
                    + "), so there's no Visor to deploy"));
  }

  /** The deploy target for one board: its files, then the swap. */
  private static void target(
      Project project,
      Extension visor,
      Coprocessor board,
      Provider<Directory> staged,
      TaskProvider<Sync> stage) {
    DeployExtension deploy = project.getExtensions().getByType(DeployExtension.class);
    Provider<File> folder = staged.map(dir -> dir.dir(board.getName()).getAsFile());
    Provider<Boolean> hasCameras = folder.map(dir -> new File(dir, "visor.toml.new").exists());
    deploy
        .getTargets()
        .create(
            "visor-" + board.getName(),
            RemoteTarget.class,
            target -> {
              target.setDirectory(visor.getDirectory().get());
              // Whether there's anything to deploy is known once the files are staged; a board with
              // nothing isn't connected to at all.
              target
                  .getTargetDiscoveryTask()
                  .configure(
                      discover -> {
                        discover.dependsOn(stage);
                        discover.onlyIf(
                            "a camera is on " + board.getName(), spec -> hasCameras.get());
                      });
              target
                  .getLocations()
                  .create(
                      "ssh",
                      SshDeployLocation.class,
                      location -> {
                        location.setAddress(board.getAddress().get());
                        location.setUser(visor.getUser().get());
                        location.setPassword(visor.getPassword().get());
                      });
              target
                  .getArtifacts()
                  .create(
                      "visor",
                      FileTreeArtifact.class,
                      artifact -> {
                        artifact.dependsOn(stage);
                        artifact.getDirectory().set(visor.getDirectory());
                        artifact.getFiles().set(project.fileTree(folder));
                        artifact
                            .getDeployTask()
                            .configure(
                                task ->
                                    task.onlyIf(
                                        "a camera is on " + board.getName(),
                                        spec -> hasCameras.get()));
                        artifact.getPostdeploy().add(swap(board, visor.getDirectory().get()));
                      });
            });
  }

  /**
   * The new program checks the new configuration; only then do both replace the old, and the old
   * program stops, for systemd to start the new one.
   */
  private static Action<DeployContext> swap(Coprocessor board, String directory) {
    String command =
        "cd '"
            + directory
            + "' && chmod 755 visor.new && ./visor.new check --config visor.toml.new 2>&1"
            + " && mv -f visor.new visor && mv -f visor.toml.new visor.toml"
            + " && { pkill -x visor; true; }";
    return context -> {
      CommandDeployResult result = context.execute(command);
      if (result.getExitCode() != 0) {
        throw new GradleException(
            "Visor on "
                + board.getName()
                + " refused the new configuration, and runs what it had:\n"
                + result.getResult());
      }
    };
  }
}
