package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.pitwall.Devices;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth.Link;
import com.michaelgrundvig.frc.runtime.lib.health.CameraHealth.Warning;
import com.michaelgrundvig.frc.runtime.lib.health.Episodes;

/**
 * A camera's USB link on its card: its speed and the format Visor runs it in ("USB 3, GREY"), and,
 * where the link is wrong, what that means and what to do. A USB 3 camera that fell to USB 2 is a
 * warning, and stays one for the session like the card's other warnings, with when; a link at USB
 * 1's speed is a problem while it lasts.
 */
final class LinkLine {
  static final String LABEL = "USB link";

  private LinkLine() {}

  /** The line as of now, from the camera's inputs and its health. */
  static Devices.Line of(CameraIO.CameraIOInputs inputs, CameraHealth health) {
    Link link = health.link();
    String value = speed(link, inputs.pixelFormat) + meaning(link);
    value += history("on USB 2 ", health, Warning.ON_USB_2);
    value += history("too slow ", health, Warning.LINK_TOO_SLOW);
    return new Devices.Line(LABEL, value, level(link, health));
  }

  /** The link's speed, and the format in use where it's known. */
  private static String speed(Link link, UsbLink.Format format) {
    String speed =
        switch (link) {
          case NOT_KNOWN -> "not known";
          case USB_3 -> "USB 3";
          case USB_2_CAMERA, USB_3_CAMERA_ON_USB_2, USB_3_CAMERA_ON_USB_2_AFTER_A_RESET -> "USB 2";
          case TOO_SLOW -> "USB 1's speed";
        };
    if (format == UsbLink.Format.UNKNOWN) {
      if (link == Link.NOT_KNOWN) {
        return "not reported";
      }
      return speed;
    }
    return speed + ", " + format.name();
  }

  /** What a link that's wrong means, and what to do about it; nothing for one that's fine. */
  private static String meaning(Link link) {
    return switch (link) {
      case NOT_KNOWN, USB_3, USB_2_CAMERA -> "";
      case USB_3_CAMERA_ON_USB_2 ->
          ": a USB 3 camera on USB 2, working at a cost; reseat its plug at the next chance";
      case USB_3_CAMERA_ON_USB_2_AFTER_A_RESET ->
          ": a USB 3 camera still on USB 2 after Visor tried to bring it back, so its USB 3"
              + " contacts probably aren't touching; reseat its plug or change its cable";
      case TOO_SLOW ->
          ": a wiring fault, expect few or no frames; reseat its plug or change its cable";
    };
  }

  /** When a link warning held this session, if it ever did. */
  private static String history(String what, CameraHealth health, Warning warning) {
    Episodes episodes = health.episodes(warning);
    if (episodes.count() == 0) {
      return "";
    }
    return "; " + what + episodes.describe(health.nowNanos());
  }

  /**
   * A problem while it's too slow; a warning while it's on USB 2, and for the rest of the session
   * after either; otherwise fine, or a note while it's not known.
   */
  private static Devices.Level level(Link link, CameraHealth health) {
    if (link == Link.TOO_SLOW) {
      return Devices.Level.PROBLEM;
    }
    boolean everWrong =
        health.episodes(Warning.ON_USB_2).count() > 0
            || health.episodes(Warning.LINK_TOO_SLOW).count() > 0;
    if (everWrong) {
      return Devices.Level.WARNING;
    }
    if (link == Link.NOT_KNOWN) {
      return Devices.Level.NOTE;
    }
    return Devices.Level.OK;
  }
}
