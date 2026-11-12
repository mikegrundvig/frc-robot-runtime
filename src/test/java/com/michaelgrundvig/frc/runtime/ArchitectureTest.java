package com.michaelgrundvig.frc.runtime;

import static com.tngtech.archunit.base.DescribedPredicate.not;
import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAPackage;
import static com.tngtech.archunit.core.domain.JavaClass.Predicates.resideInAnyPackage;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;
import static com.tngtech.archunit.library.dependencies.SlicesRuleDefinition.slices;
import static org.assertj.core.api.Assertions.assertThatThrownBy;

import com.revrobotics.spark.SparkLowLevel;
import com.tngtech.archunit.base.DescribedPredicate;
import com.tngtech.archunit.core.domain.Dependency;
import com.tngtech.archunit.core.domain.JavaAccess;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import com.tngtech.archunit.core.domain.JavaCodeUnit;
import com.tngtech.archunit.core.domain.JavaConstructorCall;
import com.tngtech.archunit.core.domain.JavaField;
import com.tngtech.archunit.core.domain.JavaMethodCall;
import com.tngtech.archunit.core.importer.ClassFileImporter;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.CompositeArchRule;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import java.security.SecureRandom;
import java.time.Clock;
import java.time.Instant;
import java.time.LocalDate;
import java.time.LocalDateTime;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Timer;
import java.util.UUID;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.EdgeCounterFilter;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.util.MathSharedStore;

/**
 * The runtime's architecture rules, checked on every build. Each rule prevents a kind of bug that
 * robot code otherwise accumulates quietly until it bites at an event. The rules that hold for
 * every class of a robot program ({@link ProductionRules}) are checked here on the runtime's
 * classes and in the team's own architecture test on the team's.
 */
class ArchitectureTest {
  private static final JavaClasses PRODUCTION_CODE = ProductionCode.CLASSES;

  /**
   * The team's robot program: the runtime is given what it needs from it and never names any of it.
   */
  private static final String TEAM_CODE = "frc.robot..";

  /** The runtime's own code. */
  private static final String RUNTIME = "com.michaelgrundvig.frc.runtime..";

  /** The robot's coprocessors: Spotter's boards, and Visor installed on them. */
  private static final String COPROCESSORS = "com.michaelgrundvig.frc.runtime.coprocessors..";

  /** The simulator: the simulated world, behind the IO interfaces. */
  private static final String SIM = "com.michaelgrundvig.frc.runtime.sim..";

  /** The simulator's classes: the runtime's, and the simulation library's. */
  private static final String[] SIMULATOR = {SIM, "com.michaelgrundvig.frc.simulation.."};

  /**
   * The one call into the simulator the robot program makes: {@code Robot.simulation}, in the
   * simulator's mode.
   */
  private static final String SIMULATOR_FACTORY =
      "com.michaelgrundvig.frc.runtime.Robot.simulation";

  /** The vision package, whose Visor IO reads Visor's frames. */
  private static final String VISION = "com.michaelgrundvig.frc.runtime.subsystems.vision";

  /**
   * What runs the same on the robot, in a replay, and in each of the simulator's worlds side by
   * side: the runtime's components and the libraries (the world model, localization, the
   * simulation).
   */
  private static final String[] DETERMINISTIC = {
    "com.michaelgrundvig.frc.runtime.lib..",
    "com.michaelgrundvig.frc.world..",
    "com.michaelgrundvig.frc.localization..",
    "com.michaelgrundvig.frc.simulation.."
  };

  /**
   * The runtime is the robot program every robot shares and no team writes: the team's code hands
   * it which robot this is, this season's field and what the robot does (RobotProgram.start). The
   * runtime's project can't see the team's, so Gradle keeps this too; the rule names it, and holds
   * for code that is compiled against the team's classes.
   */
  @Test
  void theRuntimeNeverReachesTheTeamsCode() {
    noClasses()
        .that()
        .resideInAPackage(RUNTIME)
        .should()
        .dependOnClassesThat()
        .resideInAPackage(TEAM_CODE)
        .because(
            "a runtime that names a team's class can't be built, tested or shared without that"
                + " team's robot: hand it what it needs, as RobotProgram.start does")
        .check(PRODUCTION_CODE);
  }

  /**
   * The robot program's own components (com.michaelgrundvig.frc.runtime.lib): its loop timing, the
   * checks across its wiring, and the vendors' helpers, which build a joint's settings for the IO
   * layer to apply. Each is plain functions of what it's handed.
   */
  @Test
  void componentsReachNoDeviceAndNoRobotCode() {
    noClasses()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.runtime.lib..")
        .should()
        .dependOnClassesThat(
            resideInAPackage("com.michaelgrundvig.frc.runtime..")
                .and(not(resideInAPackage("com.michaelgrundvig.frc.runtime.lib..")))
                .as("are the rest of the robot program")
                .or(
                    resideInAnyPackage(
                        "com.ctre.phoenix6.hardware..", // Phoenix's devices
                        "org.littletonrobotics..", // AdvantageKit
                        "org.wpilib.networktables..",
                        "org.wpilib.command3..",
                        "com.michaelgrundvig.frc.pitwall.."))
                .or(aSpark()))
        .because(
            "a component that reaches a device, the robot's other code, its log, NetworkTables,"
                + " commands or Pitwall can't be unit-tested without a robot")
        .check(PRODUCTION_CODE);
  }

  /**
   * The same inputs give the same answer, every run: nothing reads the clock, draws from a
   * generator nobody seeded, hands work to another thread, keeps a static collection anyone can
   * change, or reads WPILib's utilities (its logs, its alerts, its own clock).
   */
  @Test
  void theComponentsAndLibrariesAnswerTheSameEveryRun() {
    CompositeArchRule.of(
            noClasses()
                .that()
                .resideInAnyPackage(DETERMINISTIC)
                .should()
                .callMethod(System.class, "nanoTime")
                .orShould()
                .callMethod(System.class, "currentTimeMillis")
                .orShould()
                .callMethod(Instant.class, "now")
                .orShould()
                .callMethod(LocalDate.class, "now")
                .orShould()
                .callMethod(LocalDateTime.class, "now")
                .orShould()
                .dependOnClassesThat()
                .belongToAnyOf(Clock.class)
                // WPILib's math reads the robot's clock in a few places: its shared timestamp, the
                // filters that time what they see, and its pose estimators' update without a time.
                .orShould()
                .callMethod(MathSharedStore.class, "getTimestamp")
                .orShould()
                .callMethodWhere(wpilibPoseEstimatorUpdateWithoutATime())
                .orShould()
                .dependOnClassesThat()
                .belongToAnyOf(Debouncer.class, SlewRateLimiter.class, EdgeCounterFilter.class)
                .orShould()
                .callMethod(Math.class, "random")
                .orShould()
                .callMethod(StrictMath.class, "random")
                .orShould()
                .callConstructor(Random.class)
                .orShould()
                .callConstructor(SecureRandom.class)
                .orShould()
                .callMethod(UUID.class, "randomUUID")
                // WPILib's random draws (Normal) come from a generator of its own that nobody
                // seeds.
                .orShould()
                .dependOnClassesThat()
                .resideInAPackage("org.wpilib.math.random..")
                .orShould()
                .dependOnClassesThat(threadsOrWhatHandsWorkToThem())
                .orShould()
                .callConstructorWhere(aStaticInitializerMakingAMutableCollection())
                .orShould()
                .dependOnClassesThat()
                .resideInAPackage("org.wpilib.util.."))
        .and(
            fields()
                .that()
                .areDeclaredInClassesThat()
                .resideInAnyPackage(DETERMINISTIC)
                .and()
                .areStatic()
                .should(notHaveAMutableCollectionType()))
        .because(
            "a clock, an unseeded draw, another thread or shared state answers differently in a"
                + " replay, a test, or beside another simulated world: take each as an argument")
        .check(PRODUCTION_CODE);
  }

  @Test
  void theAllianceIsReadWhenItsNeeded() {
    ProductionRules.ALLIANCE_READ_WHEN_NEEDED.check(PRODUCTION_CODE);
  }

  /**
   * The rule catches a class that reads the alliance as it loads, and one that does as it's built.
   */
  @Test
  void theAllianceRuleCatchesAGuessKeptFromTheStart() {
    JavaClasses guessing =
        new ClassFileImporter().importClasses(GuessesAtLoad.class, GuessesWhenBuilt.class);

    assertThatThrownBy(() -> ProductionRules.ALLIANCE_READ_WHEN_NEEDED.check(guessing))
        .hasMessageContaining("GuessesAtLoad")
        .hasMessageContaining("GuessesWhenBuilt");
  }

  /** Keeps the alliance from when the class loaded: a guess before the Driver Station connects. */
  static final class GuessesAtLoad {
    static final boolean RED =
        MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
  }

  /** Keeps the alliance from when it was built. */
  static final class GuessesWhenBuilt {
    final boolean red =
        MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
  }

  @Test
  void subsystemsStandAlone() {
    slices()
        .matching("com.michaelgrundvig.frc.runtime.subsystems.(*)..")
        .should()
        .notDependOnEachOther()
        .because(
            "a subsystem that reaches into another can't be tested or changed without it: hand it"
                + " what it needs, as RobotContainer does")
        .check(PRODUCTION_CODE);
  }

  /**
   * Spotter, the coprocessors' monitor, is reached through the coprocessors' package alone: it
   * watches the boards through Spotter's manager and maps what it says onto the runtime's own words
   * ({@code BoardHealth}), so a change to Spotter touches one package.
   */
  @Test
  void onlyTheCoprocessorsNameSpotter() {
    noClasses()
        .that()
        .resideInAPackage(RUNTIME)
        .and()
        .resideOutsideOfPackage(COPROCESSORS)
        .should()
        .dependOnClassesThat()
        .resideInAPackage("com.michaelgrundvig.frc.spotter..")
        .because(
            "Spotter spread through the runtime means a change to it touches every package that"
                + " names it: the coprocessors' package reaches it and hands the rest the runtime's"
                + " own types")
        .check(PRODUCTION_CODE);
  }

  /**
   * Visor's client is named in three places: the coprocessors' package, which installs and
   * configures Visor on the boards; the vision package's Visor IO, which reads its frames, answers
   * its clock and knows where it streams; and the simulator, whose cameras on Visor's path send
   * Visor's own frames.
   */
  @Test
  void onlyTheCoprocessorsAndVisorsIoNameVisor() {
    noClasses()
        .that()
        .resideInAPackage(RUNTIME)
        .and()
        .resideOutsideOfPackages(COPROCESSORS, SIM)
        .and(not(visionsVisorIo()))
        .should()
        .dependOnClassesThat()
        .resideInAPackage("com.michaelgrundvig.frc.visor..")
        .because(
            "Visor spread through the runtime means a change to it touches every package that"
                + " names it: the vision package's Visor IO (VisorFeed, VisorClock, VisorPorts,"
                + " CameraIOVisor) hands the rest the runtime's own types")
        .check(PRODUCTION_CODE);
  }

  /**
   * The robot program, its top level too, reaches the simulator in one place: the call that makes
   * the simulator in its mode ({@code Robot.simulation}). Everything else sees the simulated world
   * through the hardware package's {@code Simulation} and {@code World}, as it sees a robot's
   * devices through the IO interfaces.
   */
  @Test
  void theSimulatorStaysOutOfTheRobotsCode() {
    classes()
        .that()
        .resideInAPackage(RUNTIME)
        .and()
        .resideOutsideOfPackage(SIM)
        .should(reachTheSimulatorOnlyThrough(SIMULATOR_FACTORY))
        .because(
            "robot code that reaches the simulator runs differently on the robot than in it: the"
                + " simulated world plugs in behind the IO interfaces, through"
                + " com.michaelgrundvig.frc.runtime.sim, made by Robot.simulation alone")
        .check(PRODUCTION_CODE);
  }

  @Test
  void onlyTheTopLevelUsesTheTopLevel() {
    noClasses()
        .that()
        .resideInAPackage("com.michaelgrundvig.frc.runtime.*..")
        .should()
        .dependOnClassesThat()
        .resideInAPackage("com.michaelgrundvig.frc.runtime")
        .because(
            "a part that reaches up into Robot, RobotContainer or RobotConfig can't be built or"
                + " tested without the whole robot: hand it what it needs")
        .check(PRODUCTION_CODE);
  }

  @Test
  void vendorLibrariesStayBehindIoInterfaces() {
    ProductionRules.VENDOR_LIBRARIES_STAY_BEHIND_IO_INTERFACES.check(PRODUCTION_CODE);
  }

  @Test
  void noStaticMutableState() {
    ProductionRules.NO_STATIC_MUTABLE_STATE.check(PRODUCTION_CODE);
  }

  @Test
  void configurationIsBuiltByName() {
    ProductionRules.CONFIGURATION_IS_BUILT_BY_NAME.check(PRODUCTION_CODE);
  }

  /**
   * A SPARK itself, a device: a {@code SparkMax} or {@code SparkFlex}, not REV's settings or
   * values. Asked of the class itself, since the architecture tests don't import REVLib's
   * (archunit.properties).
   */
  private static DescribedPredicate<JavaClass> aSpark() {
    return DescribedPredicate.describe(
        "are a SPARK",
        type ->
            !type.isArray()
                && type.getPackageName().startsWith("com.revrobotics.")
                && SparkLowLevel.class.isAssignableFrom(type.reflect()));
  }

  /**
   * The vision package's Visor IO: its classes named for Visor ({@code VisorFeed}, {@code
   * VisorClock}, ...) and {@code CameraIOVisor}, with their nested classes.
   */
  private static DescribedPredicate<JavaClass> visionsVisorIo() {
    return DescribedPredicate.describe(
        "are the vision package's Visor IO",
        type -> {
          if (!type.getPackageName().equals(VISION)) {
            return false;
          }
          String outermost = type.getName().substring(VISION.length() + 1).split("\\$")[0];
          return outermost.startsWith("Visor") || outermost.equals("CameraIOVisor");
        });
  }

  /**
   * Depends on the simulator's classes nowhere but in what one method does: a field, a signature, a
   * type parameter, a cast or an access anywhere else is a dependency on the simulator.
   *
   * @param factory the method allowed to reach it, by its owner's full name and its own
   */
  private static ArchCondition<JavaClass> reachTheSimulatorOnlyThrough(String factory) {
    DescribedPredicate<JavaClass> simulator = resideInAnyPackage(SIMULATOR);
    return new ArchCondition<>("reach the simulator only through " + factory) {
      @Override
      public void check(JavaClass type, ConditionEvents events) {
        for (Dependency dependency : type.getDirectDependenciesFromSelf()) {
          if (!simulator.test(dependency.getTargetClass())) {
            continue;
          }
          if (madeBy(dependency, factory)) {
            continue;
          }
          events.add(SimpleConditionEvent.violated(dependency, dependency.getDescription()));
        }
      }
    };
  }

  /** Whether a dependency is an access that a method makes, from inside its body. */
  private static boolean madeBy(Dependency dependency, String method) {
    for (JavaAccess<?> access : dependency.convertTo(JavaAccess.class)) {
      JavaCodeUnit origin = access.getOrigin();
      if ((origin.getOwner().getName() + "." + origin.getName()).equals(method)) {
        return true;
      }
    }
    return false;
  }

  /** A thread, or what hands work to one: a Timer, an executor, a CompletableFuture. */
  private static DescribedPredicate<JavaClass> threadsOrWhatHandsWorkToThem() {
    List<Class<?>> kinds =
        List.of(
            Thread.class,
            ThreadLocal.class,
            Timer.class,
            Executor.class,
            Executors.class,
            CompletableFuture.class);
    return DescribedPredicate.describe(
        "are a thread, a ThreadLocal, a Timer, an executor or a CompletableFuture",
        type ->
            !type.isArray()
                && type.getPackageName().startsWith("java.")
                && kinds.stream().anyMatch(kind -> kind.isAssignableFrom(type.reflect())));
  }

  /** Typed as a collection or map class that can be changed, rather than an interface. */
  private static ArchCondition<JavaField> notHaveAMutableCollectionType() {
    return new ArchCondition<>("not have a mutable collection class as their type") {
      @Override
      public void check(JavaField field, ConditionEvents events) {
        JavaClass type = field.getRawType();
        if (isCollection(type) && !isInterface(type)) {
          events.add(
              SimpleConditionEvent.violated(
                  field,
                  field.getFullName() + " is a " + type.getName() + ", which can be changed"));
        }
      }
    };
  }

  /** A static initializer making a collection or map that can be changed. */
  private static DescribedPredicate<JavaConstructorCall>
      aStaticInitializerMakingAMutableCollection() {
    return DescribedPredicate.describe(
        "a static initializer making a mutable collection",
        call ->
            call.getOrigin().getName().equals("<clinit>") && isCollection(call.getTargetOwner()));
  }

  /**
   * Whether a class is a collection or a map. The JDK's own aren't imported (archunit.properties
   * leaves what the project references unresolved), so their supertypes are asked of the class
   * itself.
   */
  private static boolean isCollection(JavaClass type) {
    if (!type.getPackageName().startsWith("java.")) {
      return type.isAssignableTo(Collection.class) || type.isAssignableTo(Map.class);
    }
    Class<?> loaded = type.reflect();
    return Collection.class.isAssignableFrom(loaded) || Map.class.isAssignableFrom(loaded);
  }

  /** Whether a class is an interface, asked of the JDK's own classes themselves, as above. */
  private static boolean isInterface(JavaClass type) {
    return type.getPackageName().startsWith("java.")
        ? type.reflect().isInterface()
        : type.isInterface();
  }

  /**
   * A WPILib pose estimator's {@code update}, which stamps what it's given with the clock's time.
   */
  private static DescribedPredicate<JavaMethodCall> wpilibPoseEstimatorUpdateWithoutATime() {
    return DescribedPredicate.describe(
        "a WPILib pose estimator's update, which reads the clock (updateWithTime doesn't)",
        call ->
            call.getName().equals("update")
                && call.getTargetOwner().getPackageName().equals("org.wpilib.math.estimator")
                && call.getTargetOwner().getSimpleName().contains("PoseEstimator"));
  }
}
