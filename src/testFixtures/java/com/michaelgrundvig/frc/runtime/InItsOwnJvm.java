package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.io.PrintWriter;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.extension.ExtendWith;
import org.junit.jupiter.api.extension.ExtensionContext;
import org.junit.jupiter.api.extension.InvocationInterceptor;
import org.junit.jupiter.api.extension.ReflectiveInvocationContext;
import org.junit.platform.engine.discovery.DiscoverySelectors;
import org.junit.platform.launcher.LauncherDiscoveryRequest;
import org.junit.platform.launcher.core.LauncherDiscoveryRequestBuilder;
import org.junit.platform.launcher.core.LauncherFactory;
import org.junit.platform.launcher.listeners.SummaryGeneratingListener;
import org.junit.platform.launcher.listeners.TestExecutionSummary;

/**
 * Marks a test class each of whose tests runs in a JVM started for that test alone, and passes or
 * fails by how it went there.
 *
 * <p>It is for a test of what the compiled code does: how much a loop allocates, say. What the JIT
 * has compiled by the time such a test measures depends on two things that are not the test's:
 *
 * <ul>
 *   <li>everything the JVM ran before. A test JVM runs twenty test classes, and the drive's loop
 *       measured 4,540 bytes run alone and 5,230 run after the test of its paths, past its bound.
 *       So the test gets a JVM that has run nothing else.
 *   <li>how busy the computer is. The JIT compiles on threads of its own while the code runs on,
 *       and on a busy computer it had not finished by the time the test measured: the same loop
 *       measured 6,900 bytes beside another build. So that JVM compiles each method in the thread
 *       that runs it, before it runs on ({@code -Xbatch}), and the test measures the same code
 *       however busy the computer is.
 * </ul>
 *
 * <p>The test's output is printed with the other tests', and a failure there fails the test here,
 * with what the test said there.
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.RUNTIME)
@ExtendWith(InItsOwnJvm.Runner.class)
public @interface InItsOwnJvm {
  /** Runs each test of a class it marks in a JVM of its own. */
  final class Runner implements InvocationInterceptor {
    /** The system property that says this JVM was started for one test. */
    private static final String STARTED_FOR_A_TEST = "frc.startedForATest";

    /**
     * The Java option that has the JIT compile each method in the thread that runs it, which waits
     * for the compiled code, and not on a thread of its own while the code runs on.
     */
    private static final String COMPILE_IN_THE_RUNNING_THREAD = "-Xbatch";

    /** How long a test may take in its JVM, a little under the two minutes any test may take. */
    private static final long SECONDS_ALLOWED = 110;

    @Override
    public void interceptTestMethod(
        Invocation<Void> invocation,
        ReflectiveInvocationContext<Method> invocationContext,
        ExtensionContext context)
        throws Throwable {
      boolean startedForThisTest = System.getProperty(STARTED_FOR_A_TEST) != null;
      if (startedForThisTest) {
        invocation.proceed();
        return;
      }
      invocation.skip();
      runInAJvmOfItsOwn(context.getRequiredTestClass(), context.getRequiredTestMethod());
    }

    /** Starts a JVM that runs one test, waits for it, and fails if the test failed there. */
    private static void runInAJvmOfItsOwn(Class<?> testClass, Method test)
        throws IOException, InterruptedException {
      String name = testClass.getSimpleName() + "." + test.getName();
      Path output = Files.createTempFile(Path.of("."), name + "-", ".out");
      List<String> command =
          ChildJvm.command(
              List.of(COMPILE_IN_THE_RUNNING_THREAD),
              Map.of(STARTED_FOR_A_TEST, name),
              Runner.class,
              testClass.getName(),
              test.getName());
      Process process =
          new ProcessBuilder(command)
              .redirectErrorStream(true)
              .redirectOutput(output.toFile())
              .start();
      boolean finished = process.waitFor(SECONDS_ALLOWED, TimeUnit.SECONDS);
      if (!finished) {
        process.destroyForcibly();
      }
      String said = Files.readString(output, StandardCharsets.UTF_8);
      Files.delete(output);
      System.out.print(said);
      assertThat(finished && process.exitValue() == 0)
          .as(name + " passed in a JVM of its own; what it said there:\n" + said)
          .isTrue();
    }

    /**
     * Runs one test, in a JVM started for it: the test class's name, then the test method's. It
     * exits with 0 only if that one test ran and passed.
     */
    public static void main(String[] arguments) throws ClassNotFoundException {
      Class<?> testClass = Class.forName(arguments[0]);
      LauncherDiscoveryRequest request =
          LauncherDiscoveryRequestBuilder.request()
              .selectors(DiscoverySelectors.selectMethod(testClass, arguments[1]))
              .build();
      SummaryGeneratingListener listener = new SummaryGeneratingListener();
      LauncherFactory.create().execute(request, listener);
      TestExecutionSummary summary = listener.getSummary();
      PrintWriter out = new PrintWriter(System.out, true, StandardCharsets.UTF_8);
      summary.printFailuresTo(out, 40);
      boolean passed = summary.getTestsSucceededCount() == 1 && summary.getTotalFailureCount() == 0;
      // The simulated HAL's threads would keep the JVM alive past its test.
      System.exit(passed ? 0 : 1);
    }
  }
}
