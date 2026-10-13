package com.michaelgrundvig.frc.runtime;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * The TCP ports a process is listening on, read from Linux's {@code /proc}. A test that must know
 * what a process it started has opened asks this, and so it sees that process's sockets and not
 * whatever else on the computer holds a port: a port a simulator already has open does not count
 * against a test, and does not hide one the test opened.
 */
public final class ListeningSockets {
  private static final Path PROC = Path.of("/proc");

  /** The state {@code /proc/net/tcp} gives a socket that is listening. */
  private static final String LISTEN = "0A";

  private ListeningSockets() {}

  /** Whether this computer can say ({@code /proc} is Linux's). */
  public static boolean available() {
    return Files.isReadable(PROC.resolve("net/tcp"));
  }

  /**
   * The ports a process is listening on at this moment, on any address.
   *
   * @return the ports, or none if the process has just ended
   */
  public static Set<Integer> of(ProcessHandle process) {
    Set<String> sockets = socketsOf(process.pid());
    Set<Integer> ports = new HashSet<>();
    for (String file : List.of("net/tcp", "net/tcp6")) {
      try (Stream<String> rows = Files.lines(PROC.resolve(file))) {
        rows.skip(1).forEach(row -> listeningPort(row, sockets).ifPresent(ports::add));
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      }
    }
    return ports;
  }

  /** The kernel's number for each socket a process has open, as {@code /proc} spells it. */
  private static Set<String> socketsOf(long pid) {
    try (Stream<Path> descriptors = Files.list(PROC.resolve(Long.toString(pid)).resolve("fd"))) {
      return descriptors
          .map(ListeningSockets::target)
          .filter(target -> target.startsWith("socket:["))
          .map(target -> target.substring("socket:[".length(), target.length() - 1))
          .collect(Collectors.toSet());
    } catch (IOException e) {
      return Set.of(); // the process ended
    }
  }

  private static String target(Path descriptor) {
    try {
      return Files.readSymbolicLink(descriptor).toString();
    } catch (IOException e) {
      return ""; // closed since it was listed
    }
  }

  /**
   * The port of one row of {@code /proc/net/tcp}, if the row is a listening socket among a
   * process's. A row reads: slot, local address and port in hex, remote, state, queues, timer,
   * retransmits, user, timeout, then the socket's number.
   */
  private static Optional<Integer> listeningPort(String row, Set<String> sockets) {
    String[] columns = row.trim().split("\\s+");
    if (!columns[3].equals(LISTEN) || !sockets.contains(columns[9])) {
      return Optional.empty();
    }
    String local = columns[1];
    return Optional.of(Integer.parseInt(local.substring(local.indexOf(':') + 1), 16));
  }

  /**
   * Watches a process from a thread of its own, from the moment it is told of the process until the
   * process ends, and keeps every port it saw the process listening on.
   */
  public static final class Watch {
    private static final long LOOK_AGAIN_MILLIS = 2;

    private final Set<Integer> seen = ConcurrentHashMap.newKeySet();
    private Thread thread = new Thread();

    /** Starts watching: {@code SessionRecording.record(dir, watch::start)}. */
    public void start(ProcessHandle process) {
      thread =
          new Thread(
              () -> {
                do {
                  seen.addAll(of(process));
                  try {
                    Thread.sleep(LOOK_AGAIN_MILLIS);
                  } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                  }
                } while (process.isAlive());
              },
              "Watching " + process.pid());
      thread.setDaemon(true);
      thread.start();
    }

    /** Waits until the process has ended and the watcher has seen its last moment. */
    public Set<Integer> portsSeen() throws InterruptedException {
      thread.join();
      return Set.copyOf(seen);
    }
  }
}
