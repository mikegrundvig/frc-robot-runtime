# Rule requests

A check that blocks the natural solution (an architecture rule, the coverage floor, the values
register, a lint) is recorded here instead of written around. At each milestone, read this file and
every exemption list in the checks, and change each rule or write down why not.

Each entry says:

- **Rule:** the check, and where it is.
- **Blocked:** what it stopped.
- **Natural fix:** the code you'd have written without it.
- **Workaround cost:** what writing around it would take, or took.
- **Status:** open, or what was decided and when.

## The model's allow-list banned avaje-jsonb

- **Rule:** the world model's `theModelSeesOnlyWpilibsMathAndFieldLayouts` (`model/`'s
  `ArchitectureTest`), an allow-list.
- **Blocked:** avaje-jsonb, already the model's `api` dependency, for reading venue files and
  writing AdvantageScope's assets.
- **Natural fix:** avaje-jsonb's `Jsonb.instance().type(Object.class)`.
- **Workaround cost:** about 520 lines of hand-written JSON (`JsonText`, `Json`, `VenueFile.quoted`)
  and their tests.
- **Status:** fixed 2026-10-07: the model reads and writes JSON with avaje-jsonb.

## The documentation processor's allow-list banned `com.sun.source`

- **Rule:** the documentation processor's `itNeedsNothingButJava` (`pitwall/docs-processor`'s
  `ArchitectureTest`), an allow-list, deleted 2026-10-07: Gradle already keeps it to Java.
- **Blocked:** the compiler's own Javadoc trees (`DocTrees`, in `com.sun.source`).
- **Natural fix:** read each record's Javadoc with `DocTrees.getDocCommentTree`.
- **Workaround cost:** `DocComment` parses Javadoc by hand, 621 lines.
- **Status:** open: whether to move `DocComment` onto `DocTrees` is the owner's decision.
