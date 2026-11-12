package com.michaelgrundvig.frc.runtime.subsystems.drive;

import java.util.concurrent.Executor;

/**
 * How a module's devices are sent their settings: how many times each is tried as the robot starts,
 * and where a send while the robot runs waits for its answer ({@link DeviceSettings}).
 *
 * @param tries how many times each device is sent its settings at startup, until it confirms: a few
 *     on a bus that is seen, once on a CANivore that isn't ({@code PhoenixStartup#tries})
 * @param sender where a send while the robot runs waits for its answer: on a robot a thread of its
 *     own ({@link PhoenixDrivetrain}), so the loop never waits on a device; in a test, the test
 *     itself, so it knows the answer at once
 */
record ModuleStartup(int tries, Executor sender) {}
