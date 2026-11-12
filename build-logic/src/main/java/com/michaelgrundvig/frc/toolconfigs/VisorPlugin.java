package com.michaelgrundvig.frc.toolconfigs;

import java.util.Optional;
import org.gradle.api.GradleException;
import org.gradle.api.Plugin;
import org.gradle.api.Project;
import org.gradle.api.artifacts.Configuration;
import org.gradle.api.artifacts.component.ComponentIdentifier;
import org.gradle.api.artifacts.component.ModuleComponentIdentifier;
import org.gradle.api.file.Directory;
import org.gradle.api.file.FileCollection;
import org.gradle.api.plugins.JavaPlugin;
import org.gradle.api.provider.Property;
import org.gradle.api.provider.Provider;
import org.gradle.api.tasks.SourceSet;
import org.gradle.api.tasks.SourceSetContainer;
import org.gradle.api.tasks.Sync;
import org.gradle.api.tasks.TaskProvider;
import org.wpilib.deployutils.DeployUtils;
import org.wpilib.deployutils.deploy.DeployExtension;
import org.wpilib.deployutils.deploy.artifact.FileTreeArtifact;
import org.wpilib.deployutils.deploy.target.RemoteTarget;

/**
 * {@code frc.visor}: Visor, the AprilTag vision program for the coprocessors, carried in the robot
 * program's deploy. The robot program installs and configures it on each coprocessor through
 * Spotter's agent as it starts (the runtime's {@code Coprocessors}), so the coprocessors never run
 * another version than the robot's client, nor a configuration behind the robot's definition.
 *
 * <pre>{@code
 * plugins { id "frc.visor" }
 * }</pre>
 *
 * <p>It puts two things in the deploy folder, beside the robot program's own files:
 *
 * <ul>
 *   <li>{@code visor/visor-<version>-linux-arm64.tar.gz}: the Visor release for the Visor client on
 *       the robot's runtime classpath, from the FRC libraries' repository ({@code
 *       com.michaelgrundvig.frc:visor}, built for arm64);
 *   <li>{@code spotter-packs/visor/}: that release's Spotter pack, which the robot pushes to its
 *       boards with the team's other packs.
 * </ul>
 *
 * <p>They're staged as part of every build ({@code visorStage}), so a laptop that has built once
 * can deploy offline, and deployed with the robot program ({@code ./gradlew deploy}). An archive
 * whose SHA-256 isn't the one the Visor client was published with isn't staged: the build fails,
 * saying so ({@link VisorReleaseCheck}).
 */
public final class VisorPlugin implements Plugin<Project> {
  /** The Visor client the robot is built with, whose release goes with it. */
  static final String CLIENT_GROUP = "com.michaelgrundvig.frc";

  static final String CLIENT_NAME = "frc-visor-client";

  /** The release, beside the client at its version: the arm64 build, with its Spotter pack. */
  static final String RELEASE = "com.michaelgrundvig.frc:visor:%s:linux-arm64@tar.gz";

  /** What the build says of Visor: {@code visor { ... }}, which needs nothing. */
  public abstract static class Extension {
    /** The deploy target that carries it: {@code systemcore} unless given. */
    public abstract Property<String> getTarget();

    /**
     * The robot's deploy folder on the target, where the robot program finds it ({@code
     * Filesystem.getDeployDirectory()}): {@code /home/systemcore/deploy} unless given.
     */
    public abstract Property<String> getDeployDirectory();
  }

  @Override
  public void apply(Project project) {
    project.getPluginManager().apply(JavaPlugin.class);
    Extension visor = project.getExtensions().create("visor", Extension.class);
    visor.getTarget().convention("systemcore");
    visor.getDeployDirectory().convention("/home/systemcore/deploy");

    SourceSet main = project.getExtensions().getByType(SourceSetContainer.class).getByName("main");
    Configuration runtime =
        project.getConfigurations().getByName(main.getRuntimeClasspathConfigurationName());
    Configuration release =
        project
            .getConfigurations()
            .create(
                "visorRelease",
                configuration -> {
                  configuration.setCanBeConsumed(false);
                  configuration.setTransitive(false);
                  configuration.setDescription(
                      "Visor's release, at the version of the robot's Visor client.");
                });
    release.withDependencies(
        dependencies ->
            dependencies.add(
                project.getDependencies().create(String.format(RELEASE, clientVersion(runtime)))));

    Provider<Directory> staged = project.getLayout().getBuildDirectory().dir("visor/deploy");
    TaskProvider<Sync> stage =
        project
            .getTasks()
            .register(
                "visorStage",
                Sync.class,
                task -> {
                  task.setGroup("build");
                  task.setDescription(
                      "Put Visor's release and its Spotter pack in the robot's deploy.");
                  task.doFirst(
                      "Check Visor's release against its client",
                      new VisorReleaseCheck(release, clientJar(runtime)));
                  task.into(staged);
                  task.from(release, spec -> spec.into("visor"));
                  task.from(
                      project.provider(
                          () ->
                              project
                                  .tarTree(release.getSingleFile())
                                  .matching(pack -> pack.include("spotter/visor/**"))),
                      spec -> {
                        // spotter/visor/... in the archive is spotter-packs/visor/... deployed.
                        spec.eachFile(
                            file ->
                                file.setPath(
                                    file.getPath().replaceFirst("^spotter/", "spotter-packs/")));
                        spec.setIncludeEmptyDirs(false);
                      });
                });
    project.getTasks().named("assemble").configure(assemble -> assemble.dependsOn(stage));

    // With the robot's deploy, the staged files beside the program's own.
    project
        .getPlugins()
        .withType(
            DeployUtils.class,
            applied ->
                project.afterEvaluate(
                    evaluated -> {
                      DeployExtension deploy =
                          project.getExtensions().getByType(DeployExtension.class);
                      RemoteTarget target = deploy.getTargets().findByName(visor.getTarget().get());
                      if (target == null) {
                        throw new GradleException(
                            "visor { target = ... } names "
                                + visor.getTarget().get()
                                + ", which isn't one of the deploy's targets");
                      }
                      target
                          .getArtifacts()
                          .create(
                              "visor",
                              FileTreeArtifact.class,
                              artifact -> {
                                artifact.dependsOn(stage);
                                artifact.getDirectory().set(visor.getDeployDirectory());
                                artifact.getFiles().set(project.fileTree(staged));
                              });
                    }));
  }

  /** The Visor client's jar on the robot's runtime classpath. */
  static FileCollection clientJar(Configuration runtime) {
    return runtime
        .getIncoming()
        .artifactView(view -> view.componentFilter(VisorPlugin::isClient))
        .getFiles();
  }

  private static boolean isClient(ComponentIdentifier component) {
    return component instanceof ModuleComponentIdentifier id
        && id.getGroup().equals(CLIENT_GROUP)
        && id.getModule().equals(CLIENT_NAME);
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
                    + "), so there's no Visor to carry"));
  }
}
