package com.michaelgrundvig.frc.runtime.coprocessors;

import com.michaelgrundvig.frc.spotter.manager.Board;
import com.michaelgrundvig.frc.spotter.manager.Connection;
import com.michaelgrundvig.frc.spotter.manager.Level;
import com.michaelgrundvig.frc.spotter.manager.Run;
import com.michaelgrundvig.frc.spotter.manager.Value;
import com.michaelgrundvig.frc.spotter.protocol.Spotter;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * A Spotter board as the Visor installer sees it, through Visor's Spotter pack: its {@code
 * visor.status} values, and its {@code visor.install} and {@code visor.configure} actions.
 */
final class SpotterBoards implements VisorInstaller.Board {
  private final Board board;
  private final Value version;
  private final Value config;
  private final Value cameras;

  SpotterBoards(Board board) {
    this.board = board;
    this.version = board.value("visor.status.version");
    this.config = board.value("visor.status.config");
    this.cameras = board.value("visor.status.cameras");
  }

  @Override
  public String name() {
    return board.name();
  }

  @Override
  public Optional<VisorInstaller.Report> report() {
    if (board.connection() != Connection.CONNECTED
        || !version.available()
        || !config.available()
        || !cameras.available()) {
      return Optional.empty();
    }
    List<String> plugged =
        cameras.text().isEmpty() ? List.of() : Arrays.asList(cameras.text().split(","));
    return Optional.of(new VisorInstaller.Report(version.text(), config.text(), plugged));
  }

  @Override
  public VisorInstaller.Step install(byte[] release) {
    return new Running(board.run("visor.install", release));
  }

  @Override
  public VisorInstaller.Step configure(String configuration) {
    return new Running(board.run("visor.configure", configuration));
  }

  /** A Spotter run, as one of the installer's steps. */
  private record Running(Run run) implements VisorInstaller.Step {
    @Override
    public boolean done() {
      return run.done();
    }

    @Override
    public boolean succeeded() {
      return run.level() == Level.OK;
    }

    /** The run's verdict, and the last thing its action said, which is Visor's reason. */
    @Override
    public String why() {
      List<Spotter.LogEntry> log = run.log();
      String said = log.isEmpty() ? "" : ": " + log.get(log.size() - 1).getMessage();
      return run.reason() + said;
    }
  }
}
