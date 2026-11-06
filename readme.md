# SVGnest — STBBS 2D Irregular Nesting Algorithm

A 2D irregular piece nesting algorithm based on Stagnation-Triggered Tabu-Backtracking-Enhanced Beam Search (STBBS), targeting the ESICUP open board length problem: the board width W is fixed, the x direction is open, and the occupied length L is minimized.

## Entry Point

Entry file: `stbbs-cli.js`.

```bash
# 1. Run a built-in ESICUP instance (instance data in paperCalculateExample/)
node stbbs-cli.js --instance fu
node stbbs-cli.js --instance fu,mao,shapes   # comma-separated multiple instances
node stbbs-cli.js --instance all             # run all 14 instances in order

# 2. File mode (custom sheet + pieces)
node stbbs-cli.js <shape.txt> <pieces.txt>

# 3. Diagnostic mode (multi-seed parallel + offline aggregation, pure bypass, does not affect the solving trajectory)
node stbbs-cli.js --diagnose <name|all|a,b,c> [--budget <ms|0>] [--threads <n>] [--seeds <n>] [--workers <n>] [--level summary|full]
```

### Built-in Instances (paperCalculateExample/)

14 ESICUP instances, each with a `shape_<name>.txt` (sheet outline) and a `pieces_<name>.txt` (piece list):

albano, blaz, dagli, fu, guillotine, han, jakobs, mao, marques, poly, shapes, shirts, swim, trousers

The allowed rotation angle set Θ for each instance is defined in `ESICUP_ROTATIONS` in `stbbs-cli.js` (e.g. blaz/dagli/fu/han/mao use [0,90,180,270]; guillotine/shapes only allow [0]).

### Input File Format

Sheet `shape.txt`:

```
NUMBER OF VERTICES
4
VERTICES (X,Y)
0   0
33  0
33  15
0   15
```

Pieces `pieces.txt` (each PIECE has QUANTITY and an OUTER contour, optionally HOLEs):

```
PIECE 1
QUANTITY
4
OUTER
NUMBER OF VERTICES
6
VERTICES (X,Y)
0 0
2 -1
4 0
4 3
2 4
0 3
```

### Output

Results are written to `output/`:

- `stbbs_<instance>.txt` — placement result (sheet outline + each piece's ID / position / rotation / vertices)
- `stbbs_<instance>.svg` — visualization (sheet + piece outlines)
- Diagnostic mode additionally writes `metadata.json`, `beam_layers.csv`, `placement_trace.csv`, `tabu_events.jsonl`, `lns_events.jsonl`, `geometry_validation.json`, `source_layout.json`, `final_layout.json`, `result.svg` under `output/diagnostics/<instance>/run_<runId>/`, plus cross-run aggregates `runs.csv` / `area_order_analysis.csv` / `report.md`

## Algorithm

### Problem

Given a sheet (polygon; open in the x direction in open-board mode) and a set of irregular polygonal pieces (with optional holes, quantities, and an allowed rotation angle set Θ), place the pieces inside the sheet without overlap so that the occupied length L (the maximum x of any placed vertex) is minimized.

### Solving Pipeline (main flow of stbbs-cli.js)

1. **Preprocessing** (`nest.js`): parse the sheet and pieces → polygon cleaning (ClipperLib `SimplifyPolygon`) → offset in/out by the spacing (`offsetTree`) → move the sheet to the origin and enforce counter-clockwise orientation.
2. **NFP precomputation** (`nfpWorker.js`, multi-core parallel): precompute the No-Fit Polygon (NFP) for every (piece, rotation) pair — in-board NFP via `noFitPolygon`, inter-piece NFP via Minkowski difference — sharded across 32 worker threads and merged into a cache.
3. **STBBS forward search** (`stbbs.js` + `geometry.js`): beam search places pieces one by one, keeping ω candidates per layer (`omega`, default `ceil(M/3)`, capped by `omegaCap`). Candidate positions come from NFP reference points with a "slide down first" projection (`slideProjection`); the placement cost is `J = α·Δ_A + β·Δ_L` (area term + length term).
   - **Area-priority secondary objective** (areaPriority): when Q/L scores are comparable, prefer "large pieces first" (area inversion term).
   - **Large-branch preservation** (preserveLargeBranch): keep 1 exploration branch of the largest remaining area type per layer, preventing all large-piece branches from being pruned and leaving a sparse tail.
   - **Multi-pose expansion** (enablePoseBranching): keep K spatially distinct legal poses for key large pieces.
   - **Beam diversity retention** (enableBranchDiversity): reserve 1 slot each for the large / rigid / gap-fit labels.
4. **Stagnation-triggered tabu backtracking** (`stbbs.js`, §6/§7): when branch concentration `D_t(s)=U_t(s)/ω ≤ ρ` (stagnation) is detected, tabu backtracking is triggered with backtrack window `ε` and tabu list length `tabuLength`; repaired nodes are re-injected with the `mergePolicy` (biased_sample).
5. **LNS destroy-rebuild** (`lns.js`, after a complete layout): 4 operators (largeForward / tailCompress / gapRebuild / globalRebuild) iteratively rebuild the layout; termination after `lnsNoImproveLimit` consecutive non-improving iterations.
6. **Validation & output** (`validate.js` + `nestingGenerator.js`): overlap / out-of-bounds / area reconciliation checks, then TXT result and SVG visualization.

### Core Parameters (CONFIG in stbbs-cli.js)

| Parameter | Default | Description |
| --- | --- | --- |
| `alpha` / `beta` | 0.4 / 0.6 | Weights of placement cost J = α·Δ_A + β·Δ_L (β should be ≥ α) |
| `omega` / `omegaCap` | 0 / 12 | Candidates kept per layer (0 = ceil(M/3)) / cap protection |
| `rho` | 0.50 | Branch concentration stagnation trigger threshold |
| `epsilon` | 5 | Tabu backtracking window |
| `tabuLength` | 5 | Tabu list memory length |
| `noImproveLimit` | 20 | Forward-search consecutive no-improvement threshold |
| `areaPriority` / `areaPriorityWeight` | true / 0.5 | Area-priority secondary objective on / weight |
| `preserveLargeBranch` | true | Large-branch preservation |
| `enablePoseBranching` / `poseK` | true / 3 | Multi-pose expansion for key pieces / pose count |
| `enableBranchDiversity` | true | Beam diversity retention |
| `enableLns` / `lnsNoImproveLimit` | true / 5000 | LNS on / no-improvement termination threshold |
| `maxThreads` | 32 | NFP precompute worker threads |
| `seed` | 42 | Random seed (same params + same seed is reproducible) |
| `timeBudgetMs` | 0 | Algorithm time budget (0 = unlimited) |
| `curveTolerance` / `spacing` | 0.3 / 0 | Curve tolerance / piece spacing |
| `useHoles` / `exploreConcave` | false / false | Allow placement into holes / NFP edge search (false = Minkowski difference, faster and more stable) |

All parameters are centralized in the CONFIG block at the top of `stbbs-cli.js` for parameter-sensitivity experiments.

## Modules

| File | Responsibility |
| --- | --- |
| `stbbs-cli.js` | CLI entry, CONFIG, instance list, main solving pipeline, diagnostic scheduling |
| `stbbs.js` | STBBS beam search core (beam expansion, stagnation trigger, tabu backtracking, LNS invocation) |
| `geometry.js` | Geometry placement logic (placeOne / placeOneTopK, NFP reference point generation, projection sliding) |
| `lns.js` | LNS destroy-rebuild (4 operators) |
| `nest.js` | Nesting engine core (Nest class: polygon cleaning, offsetting, NFP computation, placement worker) |
| `txtparser.js` | TXT sheet/piece parsing (TxtParser) |
| `geometryutil.js` | Geometry utilities (area, point-in-polygon, NFP, arc linearization, rotation, etc.) |
| `nestingGenerator.js` | Result SVG visualization (NestingGenerator) |
| `validate.js` | Result validation (overlap / out-of-bounds / area reconciliation) |
| `nfpWorker.js` | NFP precomputation worker thread (multi-core parallel) |
| `clipper.js` | ClipperLib (polygon boolean operations / offset / simplification, third-party library) |
| `diagnostics.js` | Diagnostic context (pure bypass observability, RNG-free) |
| `areaOrderReport.js` | Offline area-order analysis (Spearman / large-piece early placement rate, etc.) |

## Requirements

Node.js (uses `worker_threads`; no additional dependencies). `clipper.js` is a bundled third-party library.
