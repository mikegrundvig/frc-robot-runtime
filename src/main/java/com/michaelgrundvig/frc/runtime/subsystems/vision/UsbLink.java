package com.michaelgrundvig.frc.runtime.subsystems.vision;

/**
 * A camera's USB link, as its vision program reports it, in the runtime's own words: Visor's IO
 * maps Visor's onto these ({@link VisorLinks}), so the rest of the runtime doesn't name Visor.
 * Visor's {@code docs/wire.md}, the link at offset 27, says what each means.
 */
public final class UsbLink {
  private UsbLink() {}

  /** The link's speed class. */
  public enum Speed {
    /** Not known: no frame yet, not on USB, unreadable, an older Visor, or PhotonVision. */
    UNKNOWN,
    /** USB 1's low or full speed, 12 Mb/s or less: far too slow for a full-size camera. */
    LOW_OR_FULL,
    /** USB 2's high speed, 480 Mb/s. */
    HIGH,
    /** USB 3's SuperSpeed, 5000 Mb/s. */
    SUPER,
    /** Faster than SuperSpeed. */
    FASTER
  }

  /** The pixel format the camera runs in. */
  public enum Format {
    /** Not known. */
    UNKNOWN,
    /** 8-bit mono, the luma itself. */
    GREY,
    /** 4:2:2 colour. */
    YUYV,
    /** Motion JPEG, decoded on the board. */
    MJPG
  }

  /** Whether the camera looks able to do better than the link it's on. */
  public enum Headroom {
    /** At its best, or not known. */
    NONE_KNOWN,
    /** A USB 3 camera on a slower link. */
    MAY_DO_BETTER
  }

  /** Whether its vision program tried to bring it back to USB 3 since it last appeared. */
  public enum Recovery {
    /** Not tried, or tried and it worked. */
    NOT_TRIED,
    /** A reset or a power cycle of its port was tried, and it stayed slow. */
    TRIED_STILL_SLOW
  }
}
