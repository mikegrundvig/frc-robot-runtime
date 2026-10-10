package com.michaelgrundvig.frc.runtime;

import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.classes;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.fields;
import static com.tngtech.archunit.lang.syntax.ArchRuleDefinition.noClasses;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaCodeUnit;
import com.tngtech.archunit.core.domain.JavaConstructorCall;
import com.tngtech.archunit.core.domain.JavaMethodCall;
import com.tngtech.archunit.lang.ArchCondition;
import com.tngtech.archunit.lang.ArchRule;
import com.tngtech.archunit.lang.ConditionEvents;
import com.tngtech.archunit.lang.SimpleConditionEvent;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.driverstation.MatchState;

/**
 * The architecture rules that hold for every class of a robot program, the runtime's and the team's
 * alike: each is checked by the runtime's architecture test on the runtime's classes, and by the
 * team's on its own. Each rule prevents a kind of bug that robot code otherwise accumulates quietly
 * until it bites at an event.
 */
public final class ProductionRules {
  private ProductionRules() {}

  /** Hardware vendors' libraries: each belongs behind an IO interface. */
  private static final String[] VENDOR_LIBRARIES = {
    "com.ctre..", "org.photonvision..", "com.revrobotics..", "com.studica.."
  };

  /**
   * The alliance isn't known until the Driver Station connects, often after the program starts. A
   * class that reads it as it loads, or an object as it's built, keeps that guess all match: the
   * spectrum survey found teams whose red alliance aimed at blue's goal, patched by hand-editing
   * the fallback before each event.
   */
  public static final ArchRule ALLIANCE_READ_WHEN_NEEDED =
      classes()
          .should(neverReadTheAllianceAsTheyreBuilt())
          .because(
              "an alliance read before the Driver Station connects is a guess kept all match: read"
                  + " it when it's needed, as RobotContainer.periodic does each loop");

  private static ArchCondition<JavaClass> neverReadTheAllianceAsTheyreBuilt() {
    return new ArchCondition<>("never read the alliance as they load or are built") {
      @Override
      public void check(JavaClass item, ConditionEvents events) {
        List<JavaCodeUnit> building = new ArrayList<>(item.getConstructors());
        item.getStaticInitializer().ifPresent(building::add);
        for (JavaCodeUnit unit : building) {
          for (JavaMethodCall call : unit.getMethodCallsFromSelf()) {
            if (call.getTargetOwner().isEquivalentTo(MatchState.class)
                && call.getName().equals("getAlliance")) {
              events.add(SimpleConditionEvent.violated(item, call.getDescription()));
            }
          }
        }
      }
    };
  }

  /** Vendors' libraries stay behind IO interfaces. */
  public static final ArchRule VENDOR_LIBRARIES_STAY_BEHIND_IO_INTERFACES =
      noClasses()
          .that()
          .haveNameNotMatching(".*\\.(\\w+IO\\w+|Phoenix\\w*|Rev\\w*|\\w+Sim)(\\$.*)?")
          .should()
          .dependOnClassesThat()
          .resideInAnyPackage(VENDOR_LIBRARIES)
          .because(
              "a vendor's library outside an IO implementation keeps the robot's code from running in"
                  + " the simulator or replaying a log (AdvantageKit's IO pattern): name the"
                  + " implementation for its hardware, like ModuleIOTalonFX, and a vendor's helper for"
                  + " its vendor, like PhoenixOdometryThread");

  /** Nothing keeps mutable state in a static field. */
  public static final ArchRule NO_STATIC_MUTABLE_STATE =
      fields()
          .that()
          .areStatic()
          .should()
          .beFinal()
          .because(
              "static mutable state makes behavior depend on what ran before; pass state in through"
                  + " constructors instead");

  /** A configuration record is built by its builder, which names every value. */
  public static final ArchRule CONFIGURATION_IS_BUILT_BY_NAME =
      classes()
          .that()
          .areAnnotatedWith(ConfigRecord.class)
          .should(onlyBeConstructedByTheirBuilder())
          .because(
              "a constructor takes its values by position, so two can swap without a compile error;"
                  + " the builder names every value (Body.builder().frameLength(...)...)");

  /** A record {@code a.b.Outer$Name} is built by {@code a.b.OuterNameBuilder}, and nothing else. */
  private static ArchCondition<JavaClass> onlyBeConstructedByTheirBuilder() {
    return new ArchCondition<>("only be constructed by their generated builder") {
      @Override
      public void check(JavaClass record, ConditionEvents events) {
        String builder = record.getName().replace("$", "") + "Builder";
        for (JavaConstructorCall call : record.getConstructorCallsToSelf()) {
          String caller = call.getOriginOwner().getName();
          if (!caller.equals(builder) && !caller.startsWith(builder + "$")) {
            events.add(SimpleConditionEvent.violated(call, call.getDescription()));
          }
        }
      }
    };
  }
}
