package com.michaelgrundvig.frc.runtime.autos;

import com.michaelgrundvig.frc.world.field.Place;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command3.Command;

/**
 * An autonomous routine: its name, the {@link Place} the robot starts on, and a factory that makes
 * its command. The factory makes a fresh command for every run. A command that has run is never
 * composed or scheduled again, so running autonomous twice in one boot is the same as running it
 * once.
 *
 * @param name what the pit and the log call it
 * @param start where the drive team places the robot for it, written for blue like any place. Empty
 *     for {@value Autos#DO_NOTHING}, which starts wherever the Driver Station's station says.
 * @param command makes the auto's command, a new one each time it is called
 */
public record Auto(String name, Optional<Place> start, Supplier<Command> command) {}
