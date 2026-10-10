// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package com.michaelgrundvig.frc.runtime;

import java.util.Locale;
import org.wpilib.framework.RobotBase;

/**
 * What the program is running as: on a robot, in a physics simulation, or replaying a log. On a
 * robot it is always {@link #REAL}. Off one it is {@link #SIM}, or {@link #REPLAY} when the {@code
 * frc.simMode} system property says so, which {@code ./gradlew run -Preplay} sets.
 */
public enum RunMode {
  /** Running on a real robot. */
  REAL,

  /** Running a physics simulator. */
  SIM,

  /** Replaying from a log file. */
  REPLAY;

  /** The mode this program is running in. */
  public static RunMode current() {
    return RobotBase.isReal()
        ? REAL
        : valueOf(System.getProperty("frc.simMode", "SIM").toUpperCase(Locale.ROOT));
  }
}
