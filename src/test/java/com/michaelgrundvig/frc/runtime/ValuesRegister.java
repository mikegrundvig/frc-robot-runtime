package com.michaelgrundvig.frc.runtime;

import com.michaelgrundvig.frc.world.config.ConfigRecord;
import com.tngtech.archunit.core.domain.JavaClass;
import com.tngtech.archunit.core.domain.JavaClasses;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.RecordComponent;
import java.lang.reflect.Type;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.jspecify.annotations.Nullable;

/**
 * The values register (docs/values.md) as its test reads it, and what it lacks or has too many of
 * against the code.
 *
 * <p>The register is Markdown, written for people. The test reads these tables, told apart by the
 * first cell of their header row, and leaves every other table alone:
 *
 * <ul>
 *   <li><b>Value</b>: the values of the record the {@code ###} heading above names in backticks,
 *       one per row, with a <b>Kind</b> and a <b>Source</b> column.
 *   <li><b>Knob</b>: the knobs of the mechanism tuning the heading above names, with a
 *       <b>Source</b> column.
 *   <li><b>Record</b>: the records that aren't configuration.
 *   <li><b>Switch</b>: the switches the running robot publishes, with a <b>Source</b> column.
 *   <li><b>Kind</b> and <b>Source</b>: the legends, which must list the kinds and sources the test
 *       accepts.
 * </ul>
 *
 * <p>A row's first cell is a name in backticks. Cells split at pipes, except an escaped one ({@code
 * \|}). Fenced code blocks ({@code ```} or {@code ~~~}) and HTML comments ({@code <!-- -->}) are
 * skipped, so an example table in either isn't read. The numbers themselves are the code's, so no
 * cell is compared with one.
 */
final class ValuesRegister {
  /** The register, for messages. */
  static final String FILE = "docs/values.md";

  /** What kind of value a row is. */
  static final List<String> KINDS = List.of("calibrated", "characterized", "chosen", "as built");

  /** Where a row's number came from: the source cell's first words, before any detail. */
  static final List<String> SOURCES =
      List.of(
          "measured",
          "2026 code",
          "top team",
          "datasheet",
          "rules",
          "math",
          "simulation",
          "guess",
          "unknown",
          "none");

  /** A robot's whole configuration: every record it holds, however deep, is configuration. */
  static final String ROOT = "RobotConfig";

  private static final Pattern HEADING = Pattern.compile("^(#{1,6})\\s+(.*)$");
  private static final Pattern CODE = Pattern.compile("`([^`]+)`");
  private static final Pattern NAME_CELL = Pattern.compile("^`([^`]+)`$");
  private static final Pattern SEPARATOR_CELL = Pattern.compile("^:?-+:?$");
  private static final Pattern UNESCAPED_PIPE = Pattern.compile("(?<!\\\\)\\|");

  /**
   * A record marked {@link ConfigRecord}, as the register names it.
   *
   * @param name its simple name, with its outer record's for a nested one: {@code
   *     DrivetrainControl.Limits}
   * @param className its binary name, for a message to point at
   * @param components its values, in order
   */
  record CodeRecord(String name, String className, List<Component> components) {}

  /**
   * One of a record's values.
   *
   * @param holds the register name of the {@link ConfigRecord} it holds, directly or in a list, an
   *     optional, or a map; null if it holds none
   */
  record Component(String name, @Nullable String holds) {}

  /**
   * A row of the register.
   *
   * @param subject the record or tuning its table belongs to; empty for a record that isn't
   *     configuration
   * @param name its first cell's name
   * @param kind its Kind cell; empty in a table without one
   * @param source its Source cell; empty in a table without one
   * @param line where it is, counting from 1
   */
  record Row(String subject, String name, String kind, String source, int line) {}

  private enum Table {
    VALUES,
    KNOBS,
    RECORDS,
    SWITCHES,
    KINDS_LEGEND,
    SOURCES_LEGEND,
    OTHER
  }

  private final List<Row> values = new ArrayList<>();
  private final List<Row> knobs = new ArrayList<>();
  private final List<Row> notConfiguration = new ArrayList<>();
  private final List<Row> switches = new ArrayList<>();
  private final List<String> legendKinds = new ArrayList<>();
  private final List<String> legendSources = new ArrayList<>();

  /** Each {@code ###} heading that names something in backticks, by that name, with its line. */
  private final Map<String, Integer> headings = new LinkedHashMap<>();

  private final List<String> formatProblems = new ArrayList<>();

  private ValuesRegister() {}

  /** Reads a register's tables. Problems with the tables' form come back from {@link #problems}. */
  static ValuesRegister parse(String markdown) {
    ValuesRegister register = new ValuesRegister();
    String subject = "";
    Table table = Table.OTHER;
    int kindColumn = -1;
    int sourceColumn = -1;
    boolean inTable = false;
    String fence = "";
    boolean inComment = false;
    String[] lines = markdown.split("\n", -1);
    for (int i = 0; i < lines.length; i++) {
      String line = lines[i].strip();
      int number = i + 1;
      if (inComment) {
        inComment = !line.contains("-->");
        continue;
      }
      if (!fence.isEmpty()) {
        if (line.startsWith(fence)) {
          fence = "";
        }
        continue;
      }
      if (line.startsWith("<!--")) {
        inComment = !line.contains("-->");
        inTable = false;
        continue;
      }
      if (line.startsWith("```") || line.startsWith("~~~")) {
        fence = line.substring(0, 3);
        inTable = false;
        continue;
      }
      Matcher heading = HEADING.matcher(line);
      if (heading.matches()) {
        inTable = false;
        int level = heading.group(1).length();
        if (level <= 3) {
          Matcher code = CODE.matcher(heading.group(2));
          subject = level == 3 && code.find() ? code.group(1) : "";
          if (!subject.isEmpty()) {
            register.headings.putIfAbsent(subject, number);
          }
        }
        continue;
      }
      if (!line.startsWith("|")) {
        inTable = false;
        continue;
      }
      List<String> cells = cells(line);
      if (!inTable) {
        inTable = true;
        table = kindOf(cells.get(0));
        kindColumn = cells.indexOf("Kind");
        sourceColumn = cells.indexOf("Source");
        if (!register.readable(table, subject, cells.get(0), kindColumn, sourceColumn, number)) {
          table = Table.OTHER; // its rows can't be read; the problem says why
        }
        continue;
      }
      if (table == Table.OTHER
          || cells.stream().allMatch(c -> SEPARATOR_CELL.matcher(c).matches())) {
        continue;
      }
      String first = cells.get(0).replace("**", "").strip();
      if (table == Table.KINDS_LEGEND) {
        register.legendKinds.add(first);
        continue;
      }
      if (table == Table.SOURCES_LEGEND) {
        register.legendSources.add(first);
        continue;
      }
      Matcher name = NAME_CELL.matcher(cells.get(0));
      if (!name.matches()) {
        register.formatProblems.add(
            FILE
                + " line "
                + number
                + ": a row must start with a name in backticks, like `frameLength`, not \""
                + cells.get(0)
                + "\"");
        continue;
      }
      Row row =
          new Row(
              table == Table.VALUES || table == Table.KNOBS ? subject : "",
              name.group(1),
              cell(cells, kindColumn),
              cell(cells, sourceColumn),
              number);
      switch (table) {
        case VALUES -> register.values.add(row);
        case KNOBS -> register.knobs.add(row);
        case RECORDS -> register.notConfiguration.add(row);
        case SWITCHES -> register.switches.add(row);
        default -> {}
      }
    }
    return register;
  }

  /** A row's cells, split at its unescaped pipes, each unescaped and trimmed. */
  static List<String> cells(String line) {
    String inner = line.substring(1);
    if (inner.endsWith("|") && !inner.endsWith("\\|")) {
      inner = inner.substring(0, inner.length() - 1);
    }
    List<String> cells = new ArrayList<>();
    for (String cell : UNESCAPED_PIPE.split(inner, -1)) {
      cells.add(cell.replace("\\|", "|").strip());
    }
    return cells;
  }

  private static String cell(List<String> cells, int column) {
    return column >= 0 && column < cells.size() ? cells.get(column) : "";
  }

  private static Table kindOf(String firstHeaderCell) {
    return switch (firstHeaderCell) {
      case "Value" -> Table.VALUES;
      case "Knob" -> Table.KNOBS;
      case "Record" -> Table.RECORDS;
      case "Switch" -> Table.SWITCHES;
      case "Kind" -> Table.KINDS_LEGEND;
      case "Source" -> Table.SOURCES_LEGEND;
      default -> Table.OTHER;
    };
  }

  /** Whether a table's rows can be read, adding a problem for each reason they can't. */
  private boolean readable(
      Table table, String subject, String firstHeaderCell, int kind, int source, int line) {
    String where = FILE + " line " + line + ": ";
    if (table == Table.OTHER) {
      if (!subject.isEmpty()) {
        formatProblems.add(
            where
                + "the table under ### `"
                + subject
                + "` must start its header with Value (a record's values) or Knob (a tuning's"
                + " knobs), not \""
                + firstHeaderCell
                + "\"");
      }
      return false;
    }
    if (table == Table.SWITCHES && source < 0) {
      formatProblems.add(where + "a table of switches needs a Source column");
      return false;
    }
    if (table != Table.VALUES && table != Table.KNOBS) {
      return true;
    }
    int before = formatProblems.size();
    String rows = table == Table.VALUES ? "values" : "knobs";
    if (subject.isEmpty()) {
      formatProblems.add(
          where
              + "a table of "
              + rows
              + " must follow a ### heading that names its "
              + (table == Table.VALUES ? "record" : "tuning")
              + " in backticks");
    }
    if (table == Table.VALUES && kind < 0) {
      formatProblems.add(where + "a table of values needs a Kind column");
    }
    if (source < 0) {
      formatProblems.add(where + "a table of " + rows + " needs a Source column");
    }
    return formatProblems.size() == before;
  }

  /**
   * Everything the register lacks or has too many of, each saying what to add or remove: empty when
   * it matches the code. The tables' form, the records, and the tunings; the legends are checked
   * apart ({@link #legendProblems}).
   *
   * @param records every record marked {@link ConfigRecord}
   * @param tuning each mechanism tuning's name, with the paths of the values it tunes, such as
   *     {@code gains.steerKp}
   */
  List<String> problems(List<CodeRecord> records, Map<String, List<String>> tuning) {
    List<String> problems = new ArrayList<>(formatProblems);
    problems.addAll(recordProblems(records));
    problems.addAll(tuningProblems(tuning));
    return problems;
  }

  /** Whether the legends list exactly the kinds and sources the test accepts, in order. */
  List<String> legendProblems() {
    List<String> problems = new ArrayList<>();
    if (!legendKinds.equals(KINDS)) {
      problems.add(
          FILE
              + "'s legend lists the kinds "
              + legendKinds
              + ", but the test accepts "
              + KINDS
              + ": make them the same (ValuesRegister.KINDS)");
    }
    if (!legendSources.equals(SOURCES)) {
      problems.add(
          FILE
              + "'s legend lists the sources "
              + legendSources
              + ", but the test accepts "
              + SOURCES
              + ": make them the same (ValuesRegister.SOURCES)");
    }
    return problems;
  }

  /**
   * Every configuration record has its section, and every value its row, with a kind and a source
   * from the lists; nothing is listed that the code doesn't have.
   */
  List<String> recordProblems(List<CodeRecord> records) {
    List<String> problems = new ArrayList<>();
    Map<String, List<CodeRecord>> byName =
        records.stream()
            .collect(Collectors.groupingBy(CodeRecord::name, TreeMap::new, Collectors.toList()));
    sameNames(byName, problems);
    Map<String, Row> notConfigured = notConfigured(byName, problems);
    heldByARobot(byName, notConfigured, problems);
    Map<String, CodeRecord> configuration = new TreeMap<>();
    byName.forEach(
        (name, same) -> {
          if (!notConfigured.containsKey(name)) {
            configuration.put(name, same.get(0));
          }
        });
    Map<String, Map<String, Row>> rows = grouped(values, problems);
    staleSections(rows, configuration, notConfigured, problems);
    configuration.forEach(
        (name, record) ->
            recordRows(name, record, rows.getOrDefault(name, Map.of()), configuration, problems));
    for (Row row : values) {
      String value = row.subject() + "." + row.name();
      if (!KINDS.contains(row.kind())) {
        problems.add(
            at(row)
                + value
                + "'s kind is \""
                + row.kind()
                + "\": it must be one of "
                + String.join(", ", KINDS));
      }
      checkSource(row, value, problems);
    }
    return problems;
  }

  private static void sameNames(Map<String, List<CodeRecord>> byName, List<String> problems) {
    byName.forEach(
        (name, same) -> {
          if (same.size() > 1) {
            problems.add(
                "Two records are named "
                    + name
                    + " ("
                    + same.stream().map(CodeRecord::className).collect(Collectors.joining(", "))
                    + "): rename one, so "
                    + FILE
                    + " can tell them apart");
          }
        });
  }

  /** The records listed as not configuration, by name; a listing of no record is a problem. */
  private Map<String, Row> notConfigured(
      Map<String, List<CodeRecord>> byName, List<String> problems) {
    Map<String, Row> notConfigured = new LinkedHashMap<>();
    for (Row row : notConfiguration) {
      Row earlier = notConfigured.putIfAbsent(row.name(), row);
      if (earlier != null) {
        problems.add(twice(row.name() + " (a record that isn't configuration)", earlier, row));
      } else if (!byName.containsKey(row.name())) {
        problems.add(
            at(row)
                + "remove "
                + row.name()
                + " from the records that aren't configuration: no record marked @ConfigRecord is"
                + " named "
                + row.name());
      }
    }
    return notConfigured;
  }

  /** How a record is reached from a robot's configuration: the record and value that hold it. */
  private record Via(String record, String component) {}

  /** A record a robot's configuration holds is configuration, whatever the register says. */
  private static void heldByARobot(
      Map<String, List<CodeRecord>> byName, Map<String, Row> notConfigured, List<String> problems) {
    if (!byName.containsKey(ROOT)) {
      return;
    }
    Map<String, Via> reached = new LinkedHashMap<>();
    Deque<String> queue = new ArrayDeque<>(List.of(ROOT));
    while (!queue.isEmpty()) {
      String name = queue.removeFirst();
      for (CodeRecord record : byName.getOrDefault(name, List.of())) {
        for (Component component : record.components()) {
          String held = component.holds();
          if (held != null && !held.equals(ROOT) && !reached.containsKey(held)) {
            reached.put(held, new Via(name, component.name()));
            queue.addLast(held);
          }
        }
      }
    }
    for (Row row : notConfigured.values()) {
      if (row.name().equals(ROOT)) {
        problems.add(
            at(row) + ROOT + " can't be listed as not configuration: it's a robot's configuration");
        continue;
      }
      if (!reached.containsKey(row.name())) {
        continue;
      }
      List<String> path = new ArrayList<>();
      for (Via via = reached.get(row.name()); via != null; via = reached.get(via.record())) {
        path.add(0, via.record() + "." + via.component());
      }
      problems.add(
          at(row)
              + row.name()
              + " can't be listed as not configuration: a robot's configuration holds it ("
              + String.join(" → ", path)
              + ")");
    }
  }

  /** Sections for records the code doesn't have, or that are listed as not configuration. */
  private static void staleSections(
      Map<String, Map<String, Row>> rows,
      Map<String, CodeRecord> configuration,
      Map<String, Row> notConfigured,
      List<String> problems) {
    rows.forEach(
        (subject, named) -> {
          if (configuration.containsKey(subject)) {
            return;
          }
          Row first = first(named);
          Row listed = notConfigured.get(subject);
          if (listed != null) {
            problems.add(
                at(first)
                    + subject
                    + " has values here, but line "
                    + listed.line()
                    + " lists it as a record that isn't configuration: keep one");
          } else {
            problems.add(
                at(first)
                    + "remove the section for "
                    + subject
                    + ": no configuration record is named "
                    + subject);
          }
        });
  }

  /** One configuration record's rows: one for each value, and none for anything else. */
  private void recordRows(
      String name,
      CodeRecord record,
      Map<String, Row> named,
      Map<String, CodeRecord> configuration,
      List<String> problems) {
    List<String> missing = new ArrayList<>();
    Map<String, Component> components = new LinkedHashMap<>();
    for (Component component : record.components()) {
      components.put(component.name(), component);
      String holds = component.holds();
      boolean ownSection = holds != null && configuration.containsKey(holds);
      if (!ownSection && !named.containsKey(component.name())) {
        missing.add(component.name());
      }
    }
    if (named.isEmpty() && !missing.isEmpty()) {
      Integer heading = headings.get(name);
      problems.add(
          heading != null
              ? FILE
                  + " line "
                  + heading
                  + ": the section for "
                  + name
                  + " has no table of its values: add one whose header starts with Value, with a"
                  + " row for each: "
                  + String.join(", ", missing)
              : "Add a section for "
                  + name
                  + " ("
                  + record.className()
                  + ") to "
                  + FILE
                  + ": a heading ### `"
                  + name
                  + "` and a table of its values with a row for each: "
                  + String.join(", ", missing)
                  + ". Or, if it carries readings rather than configuration, list it with the"
                  + " records that aren't configuration");
    } else {
      for (String value : missing) {
        problems.add(
            "Add "
                + name
                + "."
                + value
                + " to "
                + FILE
                + ": a row | `"
                + value
                + "` | kind | source | how | in "
                + name
                + "'s table");
      }
    }
    for (Row row : named.values()) {
      Component component = components.get(row.name());
      if (component == null) {
        problems.add(
            at(row)
                + "remove "
                + name
                + "."
                + row.name()
                + ": "
                + name
                + " has no value named "
                + row.name());
        continue;
      }
      String holds = component.holds();
      if (holds != null && configuration.containsKey(holds)) {
        problems.add(
            at(row)
                + "remove "
                + name
                + "."
                + row.name()
                + ": it holds "
                + holds
                + ", which has its own section");
      }
    }
  }

  /**
   * Every value a mechanism tunes has a row under its tuning, with a source from the list, and no
   * row names one it doesn't tune.
   *
   * @param tuning each tuning's name, with the paths of the values it tunes
   */
  List<String> tuningProblems(Map<String, List<String>> tuning) {
    List<String> problems = new ArrayList<>();
    Map<String, Map<String, Row>> knobRows = grouped(knobs, problems);
    knobRows.forEach(
        (subject, named) -> {
          if (!tuning.containsKey(subject)) {
            problems.add(
                at(first(named))
                    + "remove the tuning section `"
                    + subject
                    + "`: no mechanism's tuning is named "
                    + subject);
          }
        });
    new TreeMap<>(tuning)
        .forEach(
            (name, paths) ->
                tuningRows(name, paths, knobRows.getOrDefault(name, Map.of()), problems));
    for (Row row : knobs) {
      checkSource(row, "the " + row.subject() + " tuning's " + row.name(), problems);
    }
    return problems;
  }

  /** One tuning's rows: one for each knob, and none for anything else. */
  private void tuningRows(
      String name, List<String> paths, Map<String, Row> named, List<String> problems) {
    List<String> missing = paths.stream().filter(p -> !named.containsKey(p)).toList();
    if (named.isEmpty() && !missing.isEmpty()) {
      Integer heading = headings.get(name);
      problems.add(
          heading != null
              ? FILE
                  + " line "
                  + heading
                  + ": the section for the "
                  + name
                  + " tuning has no table of its knobs: add one whose header starts with Knob,"
                  + " with a row for each: "
                  + String.join(", ", missing)
              : "Add a section for the "
                  + name
                  + " tuning to "
                  + FILE
                  + ": a heading ### `"
                  + name
                  + "` tuning and a table of its knobs with a row for each: "
                  + String.join(", ", missing));
    } else {
      for (String path : missing) {
        problems.add(
            "Add the "
                + name
                + " tuning's "
                + path
                + " to "
                + FILE
                + ": a row | `"
                + path
                + "` | source | why | in its table");
      }
    }
    for (Row row : named.values()) {
      if (!paths.contains(row.name())) {
        problems.add(
            at(row)
                + "remove the "
                + name
                + " tuning's "
                + row.name()
                + ": it tunes no value by that path");
      }
    }
  }

  /**
   * Every switch the running robot publishes has a row, with a source from the list, and no row
   * names one it doesn't publish.
   *
   * @param published the path of each switch the robot publishes, with each camera's name as {@code
   *     <camera>}
   */
  List<String> switchProblems(Set<String> published) {
    List<String> problems = new ArrayList<>();
    Map<String, Row> named = grouped(switches, problems).getOrDefault("", Map.of());
    for (String path : new TreeSet<>(published)) {
      if (!named.containsKey(path)) {
        problems.add(
            "Add the switch "
                + path
                + " to "
                + FILE
                + ": a row | `"
                + path
                + "` | source | why | in the table of switches");
      }
    }
    for (Row row : named.values()) {
      if (!published.contains(row.name())) {
        problems.add(
            at(row) + "remove the switch " + row.name() + ": the robot publishes no such switch");
      }
      checkSource(row, "the switch " + row.name(), problems);
    }
    return problems;
  }

  /** Rows by subject, then name, in order; a name twice in one subject is a problem. */
  private static Map<String, Map<String, Row>> grouped(List<Row> rows, List<String> problems) {
    Map<String, Map<String, Row>> grouped = new LinkedHashMap<>();
    for (Row row : rows) {
      Map<String, Row> named = grouped.computeIfAbsent(row.subject(), s -> new LinkedHashMap<>());
      Row earlier = named.putIfAbsent(row.name(), row);
      if (earlier != null) {
        problems.add(twice(row.subject() + "." + row.name(), earlier, row));
      }
    }
    return grouped;
  }

  private static void checkSource(Row row, String value, List<String> problems) {
    int detail = row.source().indexOf(" (");
    String source = (detail < 0 ? row.source() : row.source().substring(0, detail)).strip();
    if (!SOURCES.contains(source)) {
      problems.add(
          at(row)
              + value
              + "'s source is \""
              + row.source()
              + "\": it must start with one of "
              + String.join(", ", SOURCES)
              + ", with any detail after it in parentheses");
    }
  }

  private static Row first(Map<String, Row> named) {
    return named.values().stream().min(Comparator.comparingInt(Row::line)).orElseThrow();
  }

  private static String twice(String what, Row first, Row second) {
    return FILE
        + " lines "
        + first.line()
        + " and "
        + second.line()
        + ": "
        + what
        + " is listed twice; remove one";
  }

  private static String at(Row row) {
    return FILE + " line " + row.line() + ": ";
  }

  /** Every record marked {@link ConfigRecord} among these classes, with its values. */
  static List<CodeRecord> scan(JavaClasses classes) {
    List<String> classNames =
        classes.stream()
            .filter(c -> c.isAnnotatedWith(ConfigRecord.class))
            .map(JavaClass::getName)
            .sorted()
            .toList();
    Map<String, String> names = new LinkedHashMap<>();
    for (String className : classNames) {
      names.put(className, registerName(className));
    }
    List<CodeRecord> records = new ArrayList<>();
    for (String className : classNames) {
      Class<?> type = load(className);
      List<Component> components = new ArrayList<>();
      for (RecordComponent component : recordComponents(type)) {
        components.add(new Component(component.getName(), held(component.getGenericType(), names)));
      }
      records.add(new CodeRecord(registerName(className), className, components));
    }
    return records;
  }

  /**
   * {@code com.michaelgrundvig.frc.world.robot.DrivetrainControl$Limits} is {@code
   * DrivetrainControl.Limits}.
   */
  static String registerName(String className) {
    return className.substring(className.lastIndexOf('.') + 1).replace('$', '.');
  }

  private static Class<?> load(String className) {
    try {
      return Class.forName(className, false, ValuesRegister.class.getClassLoader());
    } catch (ClassNotFoundException e) {
      throw new IllegalStateException("can't load " + className, e);
    }
  }

  private static List<RecordComponent> recordComponents(Class<?> type) {
    if (!type.isRecord()) {
      throw new IllegalStateException(
          type.getName() + " is marked @ConfigRecord but isn't a record");
    }
    return List.of(type.getRecordComponents());
  }

  /** The register name of the configuration record a value's type holds, if it holds one. */
  private static @Nullable String held(Type type, Map<String, String> names) {
    if (type instanceof Class<?> plain) {
      return names.get(plain.getName());
    }
    if (type instanceof ParameterizedType generic) {
      for (Type argument : generic.getActualTypeArguments()) {
        String held = held(argument, names);
        if (held != null) {
          return held;
        }
      }
    }
    return null;
  }
}
