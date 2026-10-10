package com.michaelgrundvig.frc.runtime;

import static org.assertj.core.api.Assertions.assertThat;

import com.michaelgrundvig.frc.runtime.lib.timing.LoopMonitor;
import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.UncheckedIOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.TreeMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.RegisterExtension;

/**
 * The World view's cost on the template's robot, in the simulator, measured as a page sees it: its
 * description once, nothing while the robot stands still, and what driving costs a second, which
 * the design holds under 250 kbps (architecture, "Pitwall: contributions and scenes"). The loop's
 * part is Pitwall's section of the loop monitor: next to nothing with no page open.
 */
@SimulationTest
class WorldViewStreamSimTest {
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

  @RegisterExtension final RobotSim sim = new RobotSim();

  private final HttpClient client = HttpClient.newHttpClient();
  private final List<AutoCloseable> open = new ArrayList<>();

  /** Each event's name and data, as the page receives them. */
  private final List<String[]> events = new CopyOnWriteArrayList<>();

  @AfterEach
  void close() throws Exception {
    for (AutoCloseable stream : open) {
      stream.close();
    }
  }

  @Test
  void theWorldCostsLittleToSendAndNothingWhenStill() throws Exception {
    sim.robot();
    sim.loops(250); // settled: placed, the cameras' tags have narrowed the estimate
    double[] noPage = pitwallMillis(250);
    openStream("/api/events");
    waitFor(() -> has("snapshot"), "the snapshot");
    double[] otherView = pitwallMillis(250);

    events.clear();
    openStream("/api/events?views=world");
    waitFor(() -> has("replace"), "the scene's state");
    String snapshot = data("snapshot");
    int description = snapshot.indexOf("\"scenes\":");
    String replace = data("replace");
    sim.loops(100);
    int mark = events.size();
    double[] still = pitwallMillis(250);
    List<String> stillPatches = scenePatches(mark);

    // Driving: across and turning, so every wheel turns and the cameras see new tags.
    mark = events.size();
    RobotSim.setEnabled(true);
    sim.driver().setLeftY(-0.6);
    sim.driver().setLeftX(-0.3);
    sim.driver().setRightX(-0.4);
    sim.driver().notifyNewData();
    int loops = 250;
    double[] driving = pitwallMillis(loops);
    sim.driver().setLeftY(0);
    sim.driver().setLeftX(0);
    sim.driver().setRightX(0);
    sim.driver().notifyNewData();
    sim.loops(10);
    List<String> drivingPatches = scenePatches(mark);
    long bytes = 0;
    int largest = 0;
    for (String patch : drivingPatches) {
      int size = ("event: patch\ndata: " + patch + "\n\n").getBytes(StandardCharsets.UTF_8).length;
      bytes += size;
      largest = Math.max(largest, size);
    }
    double seconds = loops * 0.02;
    double kbps = bytes * 8 / seconds / 1000;
    System.out.printf(
        Locale.ROOT,
        "World view: snapshot %d B, of it the scene's description about %d B; replace %d B;"
            + " still: %d patches with the scene in 250 loops; driving: %d in %.0f s, %.0f B each"
            + " on average, %d B at most, %.1f kbps (all of each patch); Pitwall's section of the"
            + " loop: %.3f ms mean, %.3f ms at most with no page open; %.3f ms mean, %.3f ms at"
            + " most with a page on another view; %.3f ms mean, %.3f ms at most still with the"
            + " World shown too; %.3f ms mean, %.3f ms at most driving%n",
        snapshot.length(),
        description < 0 ? 0 : snapshot.length() - description,
        replace.length(),
        stillPatches.size(),
        drivingPatches.size(),
        seconds,
        drivingPatches.isEmpty() ? 0.0 : (double) bytes / drivingPatches.size(),
        largest,
        kbps,
        noPage[0],
        noPage[1],
        otherView[0],
        otherView[1],
        still[0],
        still[1],
        driving[0],
        driving[1]);

    System.out.println("Still, what changed: " + changed(stillPatches));
    System.out.println("Driving, what changed: " + changed(drivingPatches));
    assertThat(snapshot.length() - description)
        .as("the scene's description, once")
        .isLessThan(60_000);
    assertThat(stillPatches).as("patches with the scene while the robot stood still").isEmpty();
    assertThat(drivingPatches.size()).as("patches while driving").isGreaterThan(20);
    assertThat(drivingPatches.size() / seconds).as("patches a second").isLessThanOrEqualTo(20);
    assertThat(kbps).as("kbps while driving").isLessThan(250);
    assertThat(drivingPatches).anyMatch(patch -> patch.contains("\"sight-lines\":"));
  }

  /** Runs loops, and Pitwall's section's mean and longest time, in milliseconds. */
  private double[] pitwallMillis(int loops) {
    double[] each = new double[loops];
    for (int i = 0; i < loops; i++) {
      sim.loop();
      for (LoopMonitor.Section section : sim.lastLoop().sections()) {
        if (section.name().equals("Pitwall")) {
          each[i] = section.seconds() * 1000;
        }
      }
    }
    return new double[] {
      Arrays.stream(each).average().orElse(0), Arrays.stream(each).max().orElse(0)
    };
  }

  /** How often each part of the scene's state changed in some patches: frame, value, or layer. */
  @SuppressWarnings("unchecked") // the patches are JSON objects
  private static Map<String, Integer> changed(List<String> patches) {
    Map<String, Integer> changed = new TreeMap<>();
    for (String patch : patches) {
      Map<String, Object> all = (Map<String, Object>) JSON.fromJson(patch);
      Object scenes = all.get("scenes");
      if (!(scenes instanceof Map<?, ?> byId)
          || !(byId.get("world") instanceof Map<?, ?> world)
          || !(world.get("state") instanceof Map<?, ?> state)) {
        continue;
      }
      state.forEach(
          (part, keys) -> {
            if (keys instanceof Map<?, ?> map) {
              map.keySet().forEach(key -> changed.merge(part + "/" + key, 1, Integer::sum));
            }
          });
    }
    return changed;
  }

  /** The patches since an event that carry the scene's state. */
  private List<String> scenePatches(int from) {
    return events.subList(from, events.size()).stream()
        .filter(event -> event[0].equals("patch") && event[1].contains("\"scenes\":"))
        .map(event -> event[1])
        .toList();
  }

  private boolean has(String name) {
    return events.stream().anyMatch(event -> event[0].equals(name));
  }

  private String data(String name) {
    return events.stream().filter(event -> event[0].equals(name)).findFirst().orElseThrow()[1];
  }

  private void openStream(String path) throws Exception {
    CompletableFuture<HttpResponse<Stream<String>>> connecting =
        client.sendAsync(
            HttpRequest.newBuilder(URI.create("http://127.0.0.1:" + sim.pitwall().port() + path))
                .GET()
                .build(),
            HttpResponse.BodyHandlers.ofLines());
    waitFor(connecting::isDone, "the stream");
    Stream<String> lines = connecting.get().body();
    open.add(lines::close);
    Thread.ofVirtual()
        .start(
            () -> {
              List<String> event = new ArrayList<>();
              try {
                lines.forEach(
                    line -> {
                      if (line.isEmpty()) {
                        if (event.size() == 2) {
                          events.add(
                              new String[] {
                                event.get(0).substring("event: ".length()),
                                event.get(1).substring("data: ".length())
                              });
                        }
                        event.clear();
                      } else {
                        event.add(line);
                      }
                    });
              } catch (UncheckedIOException closed) {
                // The test is over.
              }
            });
  }

  /** Runs loops until something has arrived, for at most ten seconds of real time. */
  private void waitFor(BooleanSupplier arrived, String what) {
    long end = System.nanoTime() + Duration.ofSeconds(10).toNanos();
    while (!arrived.getAsBoolean() && System.nanoTime() < end) {
      sim.loop();
    }
    assertThat(arrived.getAsBoolean()).as(what).isTrue();
  }
}
