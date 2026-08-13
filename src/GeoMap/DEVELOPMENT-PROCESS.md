# Development Process — GeoMap continuous drape

The stepwise working method used on this branch. Applies to every change; a
fresh session (or machine) should follow this verbatim. Delete before the
final PR.

## Small steps with review gates

- Work proceeds in **small, numbered steps**, each one buildable, tested, and
  reviewable on its own (e.g. "Step 6: patch stitching", "Step 7: pinning
  cutover").
- **Stop and report after each step.** The user reviews before the next step
  begins. Never chain multiple design-level changes without a gate.
- When a fix isn't converging (fix-after-fix pattern), **step back and
  question the design** rather than adding another mechanism. The
  feathering → pinning cutover is the precedent: prefer removing a mechanism
  over patching one.
- Design changes get written up (markdown in this directory) and approved
  before implementation.

## TDD — red first, always

For any new behavior or invariant in production code:

1. Write the unit test(s) first, asserting the desired post-condition.
2. **Run them and verify they fail** for the right reason before touching
   production code.
3. Implement the minimal production change to make them pass.
4. Re-run the suite; then the neighboring GeoMap suites.

Rules:

- Never write the production fix first and back-fill a confirming test.
- Skip ceremony tests: only add a test when it genuinely fails for the bug
  being fixed and covers meaningful behavior.
- UI behavior is tested through the real QGC UI (C++ `QmlUITestBase` tests),
  never via side QML harness files.

## Build & test loop

- **Check for a running build first**: `pgrep -fl "ninja|cmake --build"` —
  the user often builds via VS Code; never start a second build.
- Coverage-enabled build tree: `cmake --build build-coverage --target QGroundControl`
- One suite at a time while iterating:

  ```sh
  ./build-coverage/Debug/QGroundControl.app/Contents/MacOS/QGroundControl \
      --unittest:<SuiteName> --allow-multiple 2>&1 \
      | grep -vE '^profiling' | grep -E "FAIL|Totals"
  ```

- Full GeoMap regression before declaring a step done (11 suites):

  ```sh
  ctest --test-dir build-coverage -R \
    "SurfaceModel|SurfacePatchModel|SurfaceAnalysis|SurfacePatchImagery|TerrariumTileFetcher|HeightSource|PatchGeometry|HeightField|ElevationTilePyramid|TileMath|GeoMapCamera|GeoMapProj"
  ```

- Rebuild every few file edits during multi-file work, not just at the end.
- Final pass per step: `just build`, `just lint` (pre-commit needs
  `source .venv/bin/activate` first).
- clang-format on touched files:
  `CF=$(ls ~/.cache/pre-commit/*/py_env-*/bin/clang-format | head -1); "$CF" -i <files>`

## Coverage

- All test running happens in the `build-coverage` tree so coverage data
  accumulates for the GeoMap code under development.
- New production branches (error paths, eviction paths, boundary cases) get
  a test that exercises them — coverage gaps in new code are treated as
  missing tests, not as acceptable.

## Logging conventions

- Two categories per module: a normal one and a `.Verbose` one, e.g.
  `GeoMap.HeightField` and `GeoMap.HeightField.Verbose`.
- `qCDebug` for routine flow (off by default); `qCWarning` only for messages
  that must be visible without runtime filter configuration (rejected inputs,
  contract violations). Never `qCInfo`.
- Per-frame / per-insert chatter goes to the `.Verbose` category.
- Tests run in **strict log mode**: any unexpected `qCWarning` fails the
  test. Expected warnings are declared with `expectLogMessage` /
  `ignoreLogMessage` — never `QTest::ignoreMessage` (banned by pre-commit).
- Temporary diagnostics in tests are labeled `// TEMP DIAGNOSTIC` and must be
  removed before the step is declared done.

## Test hygiene (enforced by pre-commit)

- No `Q_ASSERT` in production code — defensive checks with early returns.
- No fixed-delay `QTest::qWait(n)` — use `QTRY_*_WITH_TIMEOUT` or
  `QSignalSpy::wait`.
- Always null-check `Vehicle*` / `activeVehicle()`.

## Commits & handoff

- Conventional Commits (`feat`, `fix`, `refactor`, `test`, ...); the type
  drives release automation.
- **No commit or push without explicit user instruction.**
- Review fixes amend the existing commit (`git commit --amend` +
  `git push --force-with-lease`), not new commits.
- Cross-machine handoff: WIP commit on the branch + the markdown notes in
  this directory (`HANDOFF-STATUS.md`, `DESIGN-cesium-style-sampling.md`,
  `CESIUM-ANALYSIS.md`, this file).
