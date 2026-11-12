package com.michaelgrundvig.frc.runtime.subsystems.vision;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * The robot's listed coprocessors, as the address a Visor frame came from is matched to them.
 *
 * <p>A board listed by its IP address is known at once. One listed by a name is looked up on a
 * thread of its own, since a lookup can take seconds and the loop can't wait for it: at the start,
 * and again every {@link #RETRY_NANOS} until it's found. Until then, a frame from an address no
 * found board has might be that board's, so it isn't refused for where it came from.
 */
final class VisorSenders implements AutoCloseable {
  /** How long before a name that wasn't found is looked up again. */
  static final long RETRY_NANOS = 5_000_000_000L;

  /** Looks a board's name up: its address, or empty when it isn't found. */
  interface Resolver {
    Optional<InetAddress> resolve(String name);
  }

  /** The network's own lookup. */
  static final Resolver DNS =
      name -> {
        try {
          return Optional.of(InetAddress.getByName(name));
        } catch (UnknownHostException notFound) {
          return Optional.empty();
        }
      };

  /** One listed board: its address as listed, and the address that turned out to be. */
  static final class Board {
    final String listed;
    private volatile @Nullable InetAddress found;

    Board(String listed) {
      this.listed = listed;
    }

    boolean isFound() {
      return found != null;
    }

    /** Whether a frame from this address came from it. */
    boolean sent(InetAddress sender) {
      InetAddress address = found;
      return address != null && address.equals(sender);
    }
  }

  private final List<Board> boards = new ArrayList<>();
  private final @Nullable Thread lookingUp;

  VisorSenders(List<String> listed, Resolver resolver) {
    for (String address : listed) {
      Board board = new Board(address);
      try {
        board.found = InetAddress.ofLiteral(address);
      } catch (IllegalArgumentException aName) {
        // A name, looked up below.
      }
      boards.add(board);
    }
    if (unfound() == 0) {
      lookingUp = null;
      return;
    }
    Thread thread = new Thread(() -> lookUp(resolver), "visor-board-names");
    thread.setDaemon(true);
    lookingUp = thread;
    thread.start();
  }

  /** Whether the robot lists no boards, so a frame may come from anyone. */
  boolean listNone() {
    return boards.isEmpty();
  }

  /**
   * The listed board a frame from this address came from, or null for one that no found board has.
   */
  @Nullable Board from(@Nullable InetAddress sender) {
    if (sender == null) {
      return null;
    }
    for (int i = 0; i < boards.size(); i++) {
      Board board = boards.get(i);
      if (board.sent(sender)) {
        return board;
      }
    }
    return null;
  }

  /** The listed board with this address, as listed, or null for none. */
  @Nullable Board listedAs(String address) {
    for (Board board : boards) {
      if (board.listed.equals(address)) {
        return board;
      }
    }
    return null;
  }

  /** How many listed boards' names haven't been found yet. */
  int unfound() {
    int unfound = 0;
    for (int i = 0; i < boards.size(); i++) {
      if (!boards.get(i).isFound()) {
        unfound++;
      }
    }
    return unfound;
  }

  private void lookUp(Resolver resolver) {
    while (unfound() > 0) {
      for (Board board : boards) {
        if (!board.isFound()) {
          resolver.resolve(board.listed).ifPresent(address -> board.found = address);
        }
      }
      if (unfound() == 0) {
        return;
      }
      try {
        Thread.sleep(RETRY_NANOS / 1_000_000L);
      } catch (InterruptedException closed) {
        Thread.currentThread().interrupt();
        return;
      }
    }
  }

  @Override
  public void close() {
    if (lookingUp != null) {
      lookingUp.interrupt();
    }
  }
}
