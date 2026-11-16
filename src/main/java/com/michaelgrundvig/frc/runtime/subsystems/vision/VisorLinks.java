package com.michaelgrundvig.frc.runtime.subsystems.vision;

import com.michaelgrundvig.frc.visor.VisorLink;

/** Visor's words for a camera's USB link, in the runtime's own ({@link UsbLink}). */
final class VisorLinks {
  private VisorLinks() {}

  static UsbLink.Speed speed(VisorLink.Speed speed) {
    return switch (speed) {
      case UNKNOWN -> UsbLink.Speed.UNKNOWN;
      case LOW_OR_FULL -> UsbLink.Speed.LOW_OR_FULL;
      case HIGH -> UsbLink.Speed.HIGH;
      case SUPER -> UsbLink.Speed.SUPER;
      case FASTER -> UsbLink.Speed.FASTER;
    };
  }

  static UsbLink.Format format(VisorLink.Format format) {
    return switch (format) {
      case UNKNOWN -> UsbLink.Format.UNKNOWN;
      case GREY -> UsbLink.Format.GREY;
      case YUYV -> UsbLink.Format.YUYV;
      case MJPG -> UsbLink.Format.MJPG;
    };
  }

  static UsbLink.Headroom headroom(VisorLink.Headroom headroom) {
    return switch (headroom) {
      case NONE_KNOWN -> UsbLink.Headroom.NONE_KNOWN;
      case MAY_DO_BETTER -> UsbLink.Headroom.MAY_DO_BETTER;
    };
  }

  static UsbLink.Recovery recovery(VisorLink.Recovery recovery) {
    return switch (recovery) {
      case NOT_TRIED -> UsbLink.Recovery.NOT_TRIED;
      case TRIED_STILL_SLOW -> UsbLink.Recovery.TRIED_STILL_SLOW;
    };
  }
}
