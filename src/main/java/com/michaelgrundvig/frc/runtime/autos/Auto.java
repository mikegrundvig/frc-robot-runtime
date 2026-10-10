package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.world.field.Place;
import java.util.function.Supplier;
import org.jspecify.annotations.Nullable;
import org.wpilib.command3.Command;

/**
 * An autonomous routine: its name, the {@link Place} the robot starts on, and a factory that makes
 * its command. The factory makes a fresh command every run, so running autonomous twice in one boot
 * is the same as once: a command that has run is never composed or scheduled again.
 *
 * @param name what the pit and the log call it
 * @param start where the drive team places the robot for it, written for blue like any place; null
 *     for {@value Autos#DO_NOTHING}, which starts wherever the Driver Station says
 * @param command makes the auto's command, a new one each time it's called
 */
public record Auto(String name, @Nullable Place start, Supplier<Command> command) {}
