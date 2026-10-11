package com.michaelgrundvig.frc.runtime;

import java.lang.management.ManagementFactory;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * The command that starts another JVM like the one the tests run in: the same Java, the same
 * arguments and the same classpath. A test that needs a process of its own starts it with this (a
 * robot program run as a program, {@link SessionRecording}; a test that must not share its JVM,
 * {@link InItsOwnJvm}).
 */
final class ChildJvm {
  /** Where a child JVM's coverage goes, beside the tests' own; build.gradle reports it. */
  static final String COVERAGE = "end-to-end.exec";

  private ChildJvm() {}

  /**
   * The command for a JVM like this one that runs a class's {@code main}.
   *
   * @param options options the child's Java is given, after the ones it shares with this JVM
   * @param properties system properties the child is given, over the ones it shares with this JVM
   * @param main the class whose {@code main} it runs
   * @param arguments what that {@code main} is handed
   */
  static List<String> command(
      List<String> options, Map<String, String> properties, Class<?> main, String... arguments) {
    List<String> command = new ArrayList<>();
    command.add(ProcessHandle.current().info().command().orElseThrow());
    for (String argument : ManagementFactory.getRuntimeMXBean().getInputArguments()) {
      if (!argument.startsWith("-javaagent")) {
        command.add(argument);
      } else if (argument.contains("jacoco")) {
        command.add(coverageOfItsOwn(argument));
      }
    }
    command.addAll(options);
    properties.forEach((key, value) -> command.add("-D" + key + "=" + value));
    command.add("-cp");
    command.add(System.getProperty("java.class.path"));
    command.add(main.getName());
    command.addAll(List.of(arguments));
    return command;
  }

  /**
   * The tests' coverage agent (JaCoCo's), for a child JVM: it writes beside the tests' own
   * coverage, to {@value #COVERAGE}, which the coverage report counts too.
   */
  private static String coverageOfItsOwn(String agent) {
    Matcher destination = Pattern.compile("destfile=([^,]*)").matcher(agent);
    if (!destination.find()) {
      return agent;
    }
    Path own = Path.of(destination.group(1)).toAbsolutePath().normalize().resolveSibling(COVERAGE);
    return destination.replaceFirst(Matcher.quoteReplacement("destfile=" + own));
  }
}
