package com.michaelgrundvig.frc.runtime.examples;

import static org.assertj.core.api.Assertions.assertThat;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Stream;
import org.jspecify.annotations.Nullable;
import org.junit.jupiter.api.Test;

/**
 * Every example of code in the guides is code that compiles: each ```java block in README.md, docs/
 * and pitwall/README.md appears in a compiled source as it's written there, whitespace and comments
 * aside. The guides' examples live in this package, each in the example it came from, so a change
 * that breaks one fails the build, and the guide is fixed with the code.
 *
 * <p>The Javadoc's examples are compiled code already: {@code {@snippet class=... region=...}}
 * shows a region of a source, marked {@code // @start region=...} and {@code // @end}, and javadoc,
 * which {@code ci} builds, fails on a class or a region that isn't there (gradle/quality.gradle).
 * Pitwall's Javadoc still writes its examples inline ({@code {@snippet :}}), so those are checked
 * here as a guide's are, until they're regions of compiled code too.
 *
 * <p>A guide's block that needs code around it to compile (a call in a joint's block, a step in a
 * command, alternatives side by side) is a fragment: its fence says {@code ```java fragment}. Each
 * of its lines is then found in one source, in order, with anything between them, and "..." stands
 * for whatever the guide leaves out; a line of braces alone isn't looked for. An inline snippet is
 * always one. A block of another kind, JSON or a shell's, isn't Java, and isn't checked: {@code
 * <pre>} in Javadoc, another fence in a guide.
 */
class DocExamplesTest {
  private static final Path PROJECT =
      Path.of(System.getProperty("frc.projectDir", ".")).toAbsolutePath().normalize();

  /**
   * Where the model's and Pitwall's sources are: this project's own submodules, or, when a robot
   * program's build includes this one, the folders of its own (build.gradle says which).
   */
  private static final Path MODEL = located("frc.modelDir", "model");

  private static final Path PITWALL = located("frc.pitwallDir", "pitwall");

  /**
   * The sources an example is looked for in: the runtime's (the guide's robot and its examples
   * among them), the libraries' and Pitwall's.
   */
  private static final List<String> SOURCES =
      List.of(
          "src/main/java",
          "src/test/java",
          "src/testFixtures/java",
          "model/src/main/java",
          "model/src/test/java",
          "lib/localization/src/main/java",
          "lib/localization/src/test/java",
          "lib/simulation/src/main/java",
          "lib/simulation/src/test/java",
          "pitwall/src/main/java",
          "pitwall/src/test/java");

  /** The sources whose Javadoc still writes its snippets inline: Pitwall's. */
  private static final List<String> INLINE = List.of("pitwall/src/main/java");

  private static Path located(String property, String folder) {
    return PROJECT.resolve(System.getProperty(property, folder)).normalize();
  }

  /** A guide's fence: the line a block opens or closes on, and its info string. */
  private static final Pattern FENCE = Pattern.compile("^\\s*```(.*)$");

  /**
   * An inline Javadoc snippet's opening: the tag, its attributes, and the colon its body follows.
   */
  private static final Pattern SNIPPET = Pattern.compile("\\{@snippet[^:}]*:\\s*$");

  /** A Javadoc line's margin. */
  private static final Pattern MARGIN = Pattern.compile("^\\s*\\*( |$)");

  /** An example of code in the documentation, from where it opens. */
  record Example(String file, int line, boolean fragment, String code) {
    /** Where it is, and how it starts, for a message. */
    String where() {
      String first = code.strip().lines().findFirst().orElse("").strip();
      return file + ":" + line + " (" + first + ")";
    }
  }

  @Test
  void everyGuidesBlockIsInACompiledSource() {
    assertThat(missing(guides().stream().flatMap(guide -> blocks(guide).stream()).toList()))
        .as(
            "each guide's block below isn't in a compiled source. Copy it from the code it shows,"
                + " or put it in an example (src/test/java/com/michaelgrundvig/frc/runtime/examples) as the guide has it,"
                + " so it compiles")
        .isEmpty();
  }

  @Test
  void everyInlineJavadocSnippetIsInACompiledSource() {
    assertThat(missing(files(INLINE).stream().flatMap(file -> snippets(file).stream()).toList()))
        .as(
            "each Javadoc snippet below isn't in a compiled source. Show a region of the code it"
                + " shows instead, {@snippet class=... region=...}, marking it in an example"
                + " (src/test/java/com/michaelgrundvig/frc/runtime/examples) or the source itself")
        .isEmpty();
  }

  @Test
  void theDocumentationHasExamplesToCheck() {
    List<Example> blocks = guides().stream().flatMap(guide -> blocks(guide).stream()).toList();
    List<Example> snippets =
        files(INLINE).stream().flatMap(file -> snippets(file).stream()).toList();

    assertThat(blocks)
        .as("the guides' blocks")
        .hasSizeGreaterThan(20)
        .anyMatch(Example::fragment)
        .anyMatch(block -> block.file().startsWith("pitwall"));
    assertThat(snippets)
        .as("Pitwall's inline snippets")
        .hasSizeGreaterThan(5)
        .anyMatch(snippet -> snippet.file().startsWith("pitwall"));
  }

  /** Whitespace and comments aren't code; a string's text is. */
  @Test
  void codeIsComparedWithoutItsWhitespaceAndComments() {
    assertThat(code("Joint.pivot(\"pivot\",  PIVOT)   // a pivot\n    .range(r) /* its range */"))
        .isEqualTo("Joint.pivot(\"pivot\",PIVOT).range(r)");
    assertThat(code("name(\"a // b\", 'c', \"d \\\" e\")"))
        .isEqualTo("name(\"a // b\",'c',\"d \\\" e\")");
  }

  /** A whole block is found as it is; a fragment, line by line in order, "..." left out. */
  @Test
  void aFragmentsLinesAreFoundInOrderInOneSource() {
    List<String> sources =
        List.of(
            code("class A { void f() { a(); b(); c(); } }"), code("class B { void g() { d(); } }"));

    assertThat(whyMissing(new Example("g.md", 1, false, "a();\nb();"), sources)).isNull();
    assertThat(whyMissing(new Example("g.md", 1, false, "a();\nc();"), sources))
        .startsWith("isn't in any source as it's written");
    assertThat(whyMissing(new Example("g.md", 1, true, "a();\nc(); // and on"), sources)).isNull();
    assertThat(whyMissing(new Example("g.md", 1, true, "void f() { ... }"), sources)).isNull();
    assertThat(whyMissing(new Example("g.md", 1, true, "c();\na();"), sources))
        .isEqualTo("its line \"a();\" isn't in any source after the lines before it");
    assertThat(whyMissing(new Example("g.md", 1, true, "a();\nd();"), sources))
        .isEqualTo("its line \"d();\" isn't in any source after the lines before it");
    assertThat(whyMissing(new Example("g.md", 1, false, "a();\n...\nc();"), sources))
        .startsWith("leaves code out (\"...\")");
    assertThat(whyMissing(new Example("g.md", 1, false, "a(); // ...and on\nb();"), sources))
        .isNull();
    assertThat(whyMissing(new Example("g.md", 1, true, "b(); // ...and on"), sources)).isNull();
    assertThat(whyMissing(new Example("g.md", 1, true, "void f() {\n}\n}\nc();"), sources))
        .as("a closing brace is no line to find")
        .isNull();
  }

  /** A guide's blocks and a Javadoc's snippets are read as written. */
  @Test
  void examplesAreReadFromTheirFences() {
    assertThat(
            blocks(
                "guide.md",
                "Some text.\n\n```java\nint a = 1;\n```\n\n  ```java fragment\n  .b()\n  ```\n"
                    + "```json\n{}\n```\n"))
        .containsExactly(
            new Example("guide.md", 3, false, "int a = 1;\n"),
            new Example("guide.md", 7, true, "  .b()\n"));
    assertThat(
            snippets(
                "A.java",
                "/**\n * Text.\n *\n * {@snippet :\n * a(\n *     b);\n * }\n * <pre>{@code\n * x\n"
                    + " * }</pre>\n */\n"))
        .containsExactly(new Example("A.java", 4, true, "a(\n    b);\n"));
  }

  /** What's wrong: each example that isn't in a source, where it is, and why. */
  private static List<String> missing(List<Example> examples) {
    List<String> sources = files(SOURCES).stream().map(file -> code(read(file))).toList();
    List<String> problems = new ArrayList<>();
    for (Example example : examples) {
      String why = whyMissing(example, sources);
      if (why != null) {
        problems.add(example.where() + ": " + why);
      }
    }
    return problems;
  }

  /**
   * Why an example isn't in the sources, or null when it is: a whole block as it is, in one; a
   * fragment's lines in order, in one.
   */
  static @Nullable String whyMissing(Example example, List<String> sources) {
    if (!example.fragment()) {
      if (strip(example.code(), false).contains("...")) {
        return "leaves code out (\"...\"): write it all, or mark it ```java fragment";
      }
      String whole = code(example.code());
      return sources.stream().anyMatch(source -> source.contains(whole))
          ? null
          : "isn't in any source as it's written: if it needs code around it, mark it ```java"
              + " fragment";
    }
    List<String> pieces = new ArrayList<>();
    List<String> lines = new ArrayList<>();
    for (String line : example.code().lines().toList()) {
      for (String piece : strip(line, false).split("\\.\\.\\.")) {
        // A brace or a semicolon alone says nothing, and would match the wrong one.
        if (code(piece).chars().anyMatch(Character::isLetterOrDigit)) {
          pieces.add(code(piece));
          lines.add(line.strip());
        }
      }
    }
    int best = 0;
    for (String source : sources) {
      int found = 0;
      int at = 0;
      while (found < pieces.size() && source.indexOf(pieces.get(found), at) >= 0) {
        at = source.indexOf(pieces.get(found), at) + pieces.get(found).length();
        found++;
      }
      if (found == pieces.size()) {
        return null;
      }
      best = Math.max(best, found);
    }
    return "its line \"" + lines.get(best) + "\" isn't in any source after the lines before it";
  }

  /**
   * Java as the comparison sees it: no comments, and no whitespace outside string and character
   * literals, so a line the formatter broke elsewhere, or a comment beside a call, doesn't count.
   */
  static String code(String java) {
    return strip(java, true);
  }

  /** Java without its comments, and without its whitespace too outside its literals if asked. */
  private static String strip(String java, boolean whitespace) {
    StringBuilder code = new StringBuilder();
    int i = 0;
    while (i < java.length()) {
      char c = java.charAt(i);
      if (java.startsWith("//", i)) {
        int end = java.indexOf('\n', i);
        i = end < 0 ? java.length() : end;
      } else if (java.startsWith("/*", i)) {
        int end = java.indexOf("*/", i + 2);
        i = end < 0 ? java.length() : end + 2;
      } else if (c == '"' || c == '\'') {
        int end = i + 1;
        while (end < java.length() && java.charAt(end) != c) {
          end += java.charAt(end) == '\\' ? 2 : 1;
        }
        end = Math.min(end + 1, java.length());
        code.append(java, i, end);
        i = end;
      } else {
        if (!whitespace || !Character.isWhitespace(c)) {
          code.append(c);
        }
        i++;
      }
    }
    return code.toString();
  }

  /** The guides: README.md, everything in docs/, and Pitwall's README. */
  private static List<Path> guides() {
    List<Path> guides = new ArrayList<>();
    guides.add(PROJECT.resolve("README.md"));
    guides.add(PITWALL.resolve("README.md"));
    try (Stream<Path> docs = Files.walk(PROJECT.resolve("docs"))) {
      docs.filter(path -> path.toString().endsWith(".md")).sorted().forEach(guides::add);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
    return guides;
  }

  private static List<Example> blocks(Path guide) {
    return blocks(shown(guide), read(guide));
  }

  /** A guide's ```java blocks, each from the line its fence opens on. */
  static List<Example> blocks(String file, String text) {
    List<String> lines = text.lines().toList();
    List<Example> blocks = new ArrayList<>();
    for (int i = 0; i < lines.size(); i++) {
      Matcher fence = FENCE.matcher(lines.get(i));
      if (!fence.matches()) {
        continue;
      }
      List<String> info = List.of(fence.group(1).strip().split("\\s+"));
      int open = i;
      StringBuilder code = new StringBuilder();
      for (i++; i < lines.size() && !FENCE.matcher(lines.get(i)).matches(); i++) {
        code.append(lines.get(i)).append('\n');
      }
      if (info.get(0).equals("java")) {
        blocks.add(new Example(file, open + 1, info.contains("fragment"), code.toString()));
      }
    }
    return blocks;
  }

  private static List<Example> snippets(Path file) {
    return snippets(shown(file), read(file));
  }

  /**
   * A source's inline Javadoc snippets ({@code {@snippet :}}), each from the line it opens on, its
   * lines without their margins, to the brace that closes it.
   */
  static List<Example> snippets(String file, String text) {
    List<String> lines = text.lines().toList();
    List<Example> snippets = new ArrayList<>();
    for (int i = 0; i < lines.size(); i++) {
      if (!SNIPPET.matcher(lines.get(i)).find()) {
        continue;
      }
      int open = i;
      int depth = 1;
      StringBuilder code = new StringBuilder();
      for (i++; i < lines.size(); i++) {
        String line = MARGIN.matcher(lines.get(i)).replaceFirst("");
        for (char c : line.toCharArray()) {
          depth += c == '{' ? 1 : c == '}' ? -1 : 0;
        }
        if (depth <= 0) {
          break;
        }
        code.append(line).append('\n');
      }
      snippets.add(new Example(file, open + 1, true, code.toString()));
    }
    return snippets;
  }

  /** A folder named from the project's root, "model/..." and "pitwall/..." where those are. */
  private static Path locate(String directory) {
    if (directory.startsWith("model/")) {
      return MODEL.resolve(directory.substring("model/".length()));
    }
    if (directory.startsWith("pitwall/")) {
      return PITWALL.resolve(directory.substring("pitwall/".length()));
    }
    return PROJECT.resolve(directory);
  }

  /** A file as the messages name it: from the project's root, with Pitwall's as "pitwall/...". */
  private static String shown(Path file) {
    return file.startsWith(PITWALL)
        ? "pitwall/" + PITWALL.relativize(file)
        : PROJECT.relativize(file).toString();
  }

  /** Every Java source under these directories of the project. */
  private static List<Path> files(List<String> directories) {
    List<Path> files = new ArrayList<>();
    for (String directory : directories) {
      try (Stream<Path> walk = Files.walk(locate(directory))) {
        walk.filter(path -> path.toString().endsWith(".java")).sorted().forEach(files::add);
      } catch (IOException e) {
        throw new UncheckedIOException(e);
      }
    }
    return files;
  }

  private static String read(Path path) {
    try {
      return Files.readString(path);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
