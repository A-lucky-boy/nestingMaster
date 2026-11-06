/**
 * stbbs-cli.js —— STBBS nesting algorithm entry point
 *
 * Algorithm: stagnation-triggered tabu-backtracking-enhanced beam search for 2D irregular nesting (STBBS)
 *       based on the paper 202610080913.docx and 算法文档.md.
 *
 * Usage:
 *   node stbbs-cli.js <shape.txt> <pieces.txt>          # file mode
 *   node stbbs-cli.js --instance fu                     # by ESICUP instance name (paperCalculateExample/)
 *   node stbbs-cli.js --instance all                    # run all 14 instance groups in order
 *   node stbbs-cli.js --instance fu,mao,shapes          # comma-separated multiple instances
 *
 * All tunable parameters are centralized in CONFIG below (see the consumption points in stbbs.js and geometry.js).
 * Correctness phase: single process, single thread, fixed seed (§9.2 recommended preferred approach).
 */

// ============================================
// 1. Configuration parameters (all centralized here, for parameter sensitivity experiments)
// ============================================
const CONFIG = {
    // ---- Geometry / preprocessing (§11.1) ----
    curveTolerance: 0.3,        // curve tolerance (polygon cleaning); higher = smoother but slower
    clipperScale: 10000000,     // Clipper integerization scale; higher = more precise but slower, large coords prone to int32 overflow (keep 1e7, do not change)
    spacing: 0,                 // piece spacing (0 = tight fit)
    useHoles: false,            // whether to allow pieces to be placed into holes; true = utilization up but NFP slower
    exploreConcave: false,      // NFP method: true = edge search, false = Minkowski difference (false is faster and more stable)
    openBoardLength: true,      // ESICUP standard: W fixed, x open, minimize L; true = expand the sheet in the x direction

    // ---- Rotation angle set Θ (instance attribute; --instance mode is overridden by ESICUP_ROTATIONS) ----
    rotationAngles: [],         // explicit angle array (effective in file mode and when empty)
    rotations: 4,               // when rotationAngles is empty, i*360/rotations

    // ---- Chapter 3 hybrid heuristic placement evaluation (§4.3 eq. 8) J = α·Δ_A + β·Δ_L ----
    alpha: 0.4,                 // area term weight Δ_A; higher = more emphasis on filling area
    beta: 0.6,                  // length term weight Δ_L; higher = more emphasis on shortening occupied length (β should be ≥ α)

    // ---- Engineering acceleration: candidate reference point budget ----
    candidateBudget: 40,        // number of candidate reference points kept per rotation angle (truncated by bottom-left sort); higher = more accurate but slower
    candidateEvalCap: 2000,     // cap on the number of candidates per placement evaluation, to prevent explosion; higher = more accurate but slower

    // ---- Chapter 3 candidate projection slide ("down first") ----
    slideProjection: true,      // whether to project vertex candidates downward; true = significant quality improvement
    slideStep0: 1.0,            // initial step size of the exponential stepping
    slideTol: 0.005,            // projection convergence tolerance; lower = more precise but slower
    slideMaxCands: 8,           // at most the first N candidates per rotation angle get sliding (protects large instances)

    // ---- Compactness penalty (optional orthogonal quality term) ----
    localCompactEnabled: false, // whether to enable; penalizes the envelope gap between two adjacent pieces
    localCompactWeight: 0.05,   // weight γ
    localCompactNeighbors: 2,   // take the nearest K neighbors to compute the envelope gap

    // ============================================================
    // STBBS core algorithm parameters (§2 parameter table / §5~§8)
    // ============================================================
    omega: 0,                   // max candidates kept in the same layer ω; 0 = by ceil(M/3) (empirical formula), >0 = explicitly specified. ω≥1
    omegaCap: 12,               // ω upper-bound protection; 0 = unlimited. When M≫N (e.g. guillotine M=149)
                                // ceil(M/3)=50 is too large and makes each layer's beam×M expansion explode, so the beam width is capped to prevent runaway
    s: 3,                       // ancestor observation span s; the trigger condition uses t≥s (§6.1)
    rho: 0.50,                  // branch concentration trigger threshold ρ; 0<ρ≤1; triggered when D_t(s)=U_t(s)/ω ≤ρ
    epsilon: 5,                 // max backtrack window length ε; r=min(ε,t)
    tabuLength: 5,              // max tabu list memory length l (integer, §2 / §7.3)
    noImproveLimit: 20,         // consecutive no-improvement threshold k (§2 / §7.5)
    mergePolicy: 'biased_sample', // repair re-injection policy: 'biased_sample' (algorithm 3, default) / 'deterministic_top'

    // ---- Area priority ordering (user heuristic: place large pieces first, fill with small ones) ----
    // As a secondary objective of beam sorting: when Q/L (compactness) are comparable, favor the "large-first" placement sequence
    // (fewer inversions areaInv is better). Combined with the placement cost J: score = J + areaPriorityWeight·invNorm.
    // The primary criterion Q/L is unchanged (compactness first); the area order only affects within-beam selection as a secondary objective.
    areaPriority: true,         // true = enable the area priority secondary objective; false = sort purely by Q/L/J
    areaPriorityWeight: 0.5,    // weight w of the area inversion normalized term; higher = more biased toward "large-first".
                                // 0 = degenerates to pure J sorting; recommended 0.3~1.0 (too large may sacrifice compactness)

    // ---- New spec §4.3: area-aware Beam retention (retain 1 large-piece exploration branch, no global forced area descending order) ----
    // When ω≥2, each layer picks 1 fixed retained branch by beamRank from the legal children of the "largest remaining area type of the parent node",
    // the other ω−1 slots use the original quality-biased weighted sampling. Prevents all large-piece branches from being pruned causing a sparse tail.
    preserveLargeBranch: true,  // true = enable area branch retention (new spec); false = pure original STBBS sampling

    // ============================================================
    // test1_3 additions: multi-pose for key pieces / diversity retention / lookahead completion / LNS
    // ============================================================

    // §1 Key piece multi-pose expansion: large pieces retain K_pos spatially distinct legal poses
    enablePoseBranching: true,  // true = enable multi-pose expansion; false = single greedy pose (original behavior)
    poseK: 3,                   // number of poses K_pos retained for key pieces (initial engineering value, not an optimization parameter)

    // §2 Beam spatial diversity retention: reserve 1 slot each for the large/rigid/gap_fit labels
    enableBranchDiversity: true, // true = enable label diversity retention (effective when ω≥3); false = only quality-biased sampling

    // §3 Lookahead completion sorting: greedily complete the remaining pieces from a partial layout, using the full L to assist sorting
    enableRollout: false,       // true = enable lookahead completion sorting; false = off (high compute cost, default off)
    rolloutMaxPieces: 0,        // max pieces for lookahead completion (0 = complete all; >0 = only approximately complete the first N)

    // §4 LNS large-neighborhood destroy-rebuild: executed after the complete layout
    enableLns: true,            // true = enable LNS; false = STBBS forward search only
    lnsBudgetMs: 0,             // LNS time budget (0 = unlimited; >0 and finite to enable the timeout branch)
    lnsMaxIters: 0,             // LNS max iterations (0 = unlimited; >0 and finite to enable the hard cap)
    lnsNoImproveLimit: 5000,    // LNS consecutive no-improvement termination threshold (the sole termination condition, an execution control parameter, not an optimality claim)
    lnsOperators: ['largeForward', 'tailCompress', 'gapRebuild', 'globalRebuild'],

    // ---- Multi-core parallelism (§9.2 acceleration phase, user machine 32 threads) ----
    maxThreads: 32,             // number of NFP precompute worker threads; 0 = single-thread

    // ---- Random and run ----
    seed: 42,                   // random seed ξ for a single run; same params + same seed should be replayable (§9.1)
    timeBudgetMs: 0,            // algorithm time budget (0 = unlimited; >0 and finite to enable the timeout branch); complete nesting of N pieces

    // ============================================================
    // test1_5: observability and defect localization (diagnostic mode, pure bypass)
    // Diagnostics only bypass-output, do not write back candidate scores, do not participate in sorting, do not change geometric legality;
    // when the switch is off the solving trajectory and layout fingerprint are exactly identical; event collection is RNG-free.
    // ============================================================
    enableDiagnostics: false,   // true = enable diagnostic output; false = pure solving (default)
    diagnosticLevel: 'summary', // 'summary' = layers + placement trace (default) / 'full' = also save all candidate stage transitions
    diagnosticDir: 'output/diagnostics', // diagnostic root directory (organized by instance/run_<runId>/)
    maxWorkers: 31,             // diagnostic parallel worker cap (≤31; actual depends on seed count and memory)
    diagSeeds: 2,               // number of independent seeds used for diagnostic runs (--diagnose default)
    diagSeedsBase: 1000,        // diagnostic seed base (actual seed = base + i)
    diagTimeBudgetMs: 0,        // diagnostic solving time budget (0 = unlimited; no longer truncates at 300000, complete nesting of N pieces)

    // ---- Output / validation ----
    outputDir: 'output',        // result output directory
    overlapTol: 1e-3,           // overlap area tolerance (for validation)
    debug: process.env.STBBS_DEBUG === '1'  // diagnostic log switch (STBBS_DEBUG=1 to enable)
};

// ============================================
// 2. Import dependencies
// ============================================
const fs = require('fs');
const path = require('path');

const TxtParser = require('./txtparser.js');
const GeometryUtil = require('./geometryutil.js');
const { Nest, rotatePolygon, calculateNfpForPair, createNfpCacheKey } = require('./nest.js');
const { Worker } = require('worker_threads');
const ClipperLib = require('./clipper.js');
const { createGeometry } = require('./geometry.js');
const { createRng, createSolver } = require('./stbbs.js');
const { validateResult } = require('./validate.js');
const { createDiagnostics } = require('./diagnostics.js');
const areaOrderReport = require('./areaOrderReport.js');
const crypto = require('crypto');
const os = require('os');

// Mount globals (geometry library dependencies)
if (ClipperLib.ClipperLib) {
    global.ClipperLib = ClipperLib.ClipperLib;
} else {
    global.ClipperLib = ClipperLib;
}
global.GeometryUtil = GeometryUtil;
if (typeof global.alert !== 'function') {
    global.alert = function (message) {
        console.error('[ClipperJS alert]', message);
    };
}

const ESICUP_INSTANCES = [
    'albano', 'blaz', 'dagli', 'fu', 'guillotine',
    'han', 'jakobs', 'mao', 'marques', 'poly',
    'shapes', 'shirts', 'swim', 'trousers'
];

// Allowed rotation angles for each instance (paper Table 1, instance attribute)
const ESICUP_ROTATIONS = {
    albano: [0, 180],
    blaz: [0, 90, 180, 270],
    dagli: [0, 90, 180, 270],
    fu: [0, 90, 180, 270],
    guillotine: [0],
    han: [0, 90, 180, 270],
    jakobs: [0, 180],
    mao: [0, 90, 180, 270],
    marques: [0, 180],
    poly: [0, 180],
    shapes: [0],
    shirts: [0, 180],
    swim: [0, 180],
    trousers: [0, 180]
};

const EXAMPLE_DIR = path.join(__dirname, 'paperCalculateExample');
const OUTPUT_COORD_DECIMALS = 8;

function formatCoordinate(v) {
    return Number(v).toFixed(OUTPUT_COORD_DECIMALS);
}

function getRotationAngles() {
    if (CONFIG.rotationAngles && CONFIG.rotationAngles.length > 0) {
        return CONFIG.rotationAngles.slice(0);
    }
    const rotations = Math.max(1, parseInt(CONFIG.rotations, 10) || 1);
    const angles = [];
    for (let i = 0; i < rotations; i++) {
        angles.push(i * (360 / rotations));
    }
    return angles;
}

function clonePolygonWithChildren(polygon) {
    const cloned = polygon.map(v => ({ x: v.x, y: v.y }));
    if (polygon.children && polygon.children.length > 0) {
        cloned.children = polygon.children.map(c => clonePolygonWithChildren(c));
    }
    return cloned;
}

// ============================================
// 4.5 Diagnostics helpers (test1_5: machine info / code fingerprint / input hash)
// ============================================
const CODE_FILES_FOR_FINGERPRINT = [
    'stbbs-cli.js', 'geometry.js', 'stbbs.js',
    'lns.js', 'validate.js', 'diagnostics.js',
    'areaOrderReport.js', 'nfpWorker.js', 'nest.js', 'geometryutil.js'
];

// Code content fingerprint (when no git, use the first 12 hex chars of the SHA256 of the source files that participate in the computation as code_commit)
function computeCodeFingerprint() {
    const h = crypto.createHash('sha256');
    for (const rel of CODE_FILES_FOR_FINGERPRINT) {
        const p = path.join(__dirname, rel);
        if (fs.existsSync(p)) {
            h.update(fs.readFileSync(p));
        }
    }
    return h.digest('hex').slice(0, 12);
}

// Input instance hash (SHA256 of the shape + pieces files combined)
function computeInstanceSha256(shapeFile, piecesFile) {
    const h = crypto.createHash('sha256');
    if (fs.existsSync(shapeFile)) {
        h.update(fs.readFileSync(shapeFile));
    }
    if (fs.existsSync(piecesFile)) {
        h.update(fs.readFileSync(piecesFile));
    }
    return h.digest('hex');
}

// Machine info (actual collected values)
function collectMachineInfo() {
    let cpuModel = null;
    try {
        const cpus = os.cpus();
        if (cpus && cpus.length > 0) {
            cpuModel = cpus[0].model || null;
        }
    } catch (e) { /* ignore */ }
    return {
        node_version: process.version,
        os: `${os.platform()} ${os.release()}`,
        cpu_model: cpuModel,
        cpus_logical: (os.cpus() || []).length
    };
}

// Run identifier: instance_id + code_version + seed + start tag
function makeRunId(instanceName, codeVersion, seed, startedAt) {
    const startTag = startedAt.replace(/[:.]/g, '').replace('T', '_').slice(0, 15);
    return `${instanceName}_${codeVersion}_seed${seed}_${startTag}`;
}

// ============================================
// 3. Parse the instance (§10 input_loader)
// ============================================
function loadInstance(instanceName, shapeFile, piecesFile) {
    const parser = new TxtParser();
    parser.config({ tolerance: CONFIG.curveTolerance });
    const bin = parser.preparePolygon(parser.parseShape(fs.readFileSync(shapeFile, 'utf8')));
    const pieces = parser.parsePiecesFile(fs.readFileSync(piecesFile, 'utf8'));
    const allParts = [];
    pieces.forEach(piece => {
        const preparedPiece = parser.preparePiece(piece);
        for (let i = 0; i < piece.quantity; i++) {
            allParts.push(clonePolygonWithChildren(preparedPiece));
        }
    });
    return { bin, allParts, piecesCount: pieces.length };
}

// ============================================
// 4. Open board length + preprocessing (reuses Nest.start(0))
// ============================================
function scalePolygonXToLength(polygon, targetLen) {
    const xs = polygon.map(v => v.x);
    const xMin = Math.min.apply(null, xs);
    const xMax = Math.max.apply(null, xs);
    const len = xMax - xMin;
    if (len <= 1e-9) {
        return polygon.map(v => ({ x: v.x, y: v.y }));
    }
    const f = targetLen / len;
    return polygon.map(v => ({ x: xMin + (v.x - xMin) * f, y: v.y }));
}

function openBoardLength(bin, allParts) {
    if (!CONFIG.openBoardLength) {
        return { bin, note: null };
    }
    const angles = getRotationAngles();
    let need = 0;
    for (let i = 0; i < allParts.length; i++) {
        let wMax = 0;
        for (let a = 0; a < angles.length; a++) {
            const r = rotatePolygon(allParts[i], angles[a]);
            const b = GeometryUtil.getPolygonBounds(r);
            if (b.width > wMax) wMax = b.width;
        }
        need += wMax;
    }
    const xs = bin.map(v => v.x);
    const len0 = Math.max.apply(null, xs) - Math.min.apply(null, xs);
    const target = Math.max(len0, need + Math.max(20, len0 * 0.1));
    if (target <= len0 + 1e-9) {
        return { bin, note: null };
    }
    const scaled = scalePolygonXToLength(bin, target);
    return { bin: scaled, note: 'Open board length: original ' + len0.toFixed(2) + ' -> ' + target.toFixed(2) };
}

function prepareInputs(bin, allParts) {
    const opened = openBoardLength(bin, allParts);
    if (opened.note) {
        console.log('  ' + opened.note);
    }
    const nest = new Nest();
    nest.config(CONFIG);
    nest.setBin(opened.bin);
    nest.setParts(allParts);
    const prepared = nest.start(0);
    if (!prepared || !prepared.binPolygon || !prepared.parts) {
        throw new Error('Preprocessing failed (binPolygon or parts is empty)');
    }
    return {
        binPolygon: prepared.binPolygon,
        originalBinPolygon: prepared.originalBinPolygon,
        parts: prepared.parts,
        originalParts: prepared.originalParts,
        partOutputOffset: prepared.partOutputOffset
    };
}

// ============================================
// 5. NFP precomputation (multi-core parallel, §9.2 acceleration phase)
// ============================================
function precomputeNfp(binPolygon, parts) {
    const angles = getRotationAngles();
    const t0 = Date.now();
    // Build all jobs: {A, B, aId, bId, inside, Arotation, Brotation}
    // key is determined by aId/bId/inside/Arotation/Brotation, exactly matching the single-threaded createNfpCacheKey.
    const jobs = [];
    // In-board NFP (inside=true): bin (id=-1) with each piece, each angle
    for (let i = 0; i < parts.length; i++) {
        for (let br = 0; br < angles.length; br++) {
            jobs.push({ A: binPolygon, B: parts[i], aId: binPolygon.id, bId: parts[i].id, inside: true, Arotation: 0, Brotation: angles[br] });
        }
    }
    // Inter-piece NFP (inside=false): each angle combination (a!==b)
    for (let a = 0; a < parts.length; a++) {
        for (let b = 0; b < parts.length; b++) {
            if (a === b) continue;
            for (let ar = 0; ar < angles.length; ar++) {
                for (let br = 0; br < angles.length; br++) {
                    jobs.push({ A: parts[a], B: parts[b], aId: parts[a].id, bId: parts[b].id, inside: false, Arotation: angles[ar], Brotation: angles[br] });
                }
            }
        }
    }
    const total = jobs.length;
    const maxThreads = Math.max(0, parseInt(CONFIG.maxThreads, 10) || 0);
    // Single-thread fallback (maxThreads=0 or small instances)
    if (maxThreads === 0 || jobs.length <= maxThreads * 4) {
        const cache = {};
        let done = 0;
        for (let i = 0; i < jobs.length; i++) {
            const job = jobs[i];
            const pair = { A: job.A, B: job.B, key: job };
            const nfp = calculateNfpForPair(pair, CONFIG);
            if (nfp && nfp.length > 0) {
                cache[createNfpCacheKey(job.aId, job.bId, job.inside, job.Arotation, job.Brotation)] = nfp;
            }
            done++;
        }
        console.log(`NFP precompute done (single-thread): ${Object.keys(cache).length}/${done} items, elapsed ${((Date.now() - t0) / 1000).toFixed(2)}s`);
        return cache;
    }
    // Multi-core parallel: split jobs evenly across maxThreads workers, dispatch and collect
    return new Promise((resolve, reject) => {
        const n = Math.min(maxThreads, jobs.length);
        const workerFile = path.join(__dirname, 'nfpWorker.js');
        const shards = [];
        for (let i = 0; i < n; i++) {
            shards.push(jobs.slice(i * Math.ceil(jobs.length / n), (i + 1) * Math.ceil(jobs.length / n)));
        }
        const cache = {};
        let done = 0;
        let finished = 0;
        function onDone(w) {
            return (err, result) => {
                if (err) {
                    reject(err);
                    return;
                }
                done += result.done;
                for (const k in result.results) {
                    cache[k] = result.results[k];
                }
                finished++;
                if (finished === n) {
                    console.log(`NFP precompute done (${n} threads): ${Object.keys(cache).length}/${done} items, elapsed ${((Date.now() - t0) / 1000).toFixed(2)}s`);
                    resolve(cache);
                }
            };
        }
        for (let i = 0; i < n; i++) {
            const w = new Worker(workerFile);
            w.on('error', reject);
            w.on('message', (msg) => {
                if (msg.type === 'result') {
                    done += msg.done;
                    for (const k in msg.results) {
                        cache[k] = msg.results[k];
                    }
                    finished++;
                    w.terminate();
                    if (finished === n) {
                        console.log(`NFP precompute done (${n} threads): ${Object.keys(cache).length}/${done} items, elapsed ${((Date.now() - t0) / 1000).toFixed(2)}s`);
                        resolve(cache);
                    }
                }
            });
            w.postMessage({ type: 'compute', jobs: shards[i], config: CONFIG });
        }
    });
}

// ============================================
// 6. Type recognition (§3.1): translation-normalized geometry signature + area
// ============================================
function groupTypes(parts, geomCtx) {
    function normSig(poly) {
        if (!poly || !poly.length) return '';
        // Translation normalization: use the minimum corner as the origin
        let minX = Infinity, minY = Infinity;
        for (let i = 0; i < poly.length; i++) {
            if (poly[i].x < minX) minX = poly[i].x;
            if (poly[i].y < minY) minY = poly[i].y;
        }
        let s = poly.length + ':';
        for (let i = 0; i < poly.length; i++) {
            s += (poly[i].x - minX).toFixed(4) + ',' + (poly[i].y - minY).toFixed(4) + ';';
        }
        if (poly.children) {
            for (let c = 0; c < poly.children.length; c++) {
                s += '[' + normSig(poly.children[c]) + ']';
            }
        }
        return s;
    }
    const sigToType = {};
    const typeInstances = [];
    const typeOfInstance = {};
    const areaById = {};
    for (let i = 0; i < parts.length; i++) {
        const poly = parts[i];
        areaById[poly.id] = geomCtx.netArea(poly);
        const sig = normSig(poly) + '|' + areaById[poly.id].toFixed(6);
        if (sigToType[sig] === undefined) {
            sigToType[sig] = typeInstances.length;
            typeInstances.push([]);
        }
        const t = sigToType[sig];
        typeInstances[t].push(poly.id);
        typeOfInstance[poly.id] = t;
    }
    typeInstances.forEach(list => list.sort((a, b) => a - b));
    return { typeInstances, typeOfInstance, areaById, M: typeInstances.length };
}

// ============================================
// 7. Save result + SVG (§12)
// ============================================
function saveResult(result, instanceName, prepared) {
    const outDir = path.join(__dirname, CONFIG.outputDir);
    fs.mkdirSync(outDir, { recursive: true });
    const outputBinPolygon = prepared.originalBinPolygon || prepared.binPolygon;
    const outputParts = prepared.originalParts || prepared.parts;
    const partOutputOffset = prepared.partOutputOffset || { x: 0, y: 0 };

    const lines = [];
    lines.push('========================================');
    lines.push('           Nesting result (STBBS)');
    lines.push('========================================\n');
    lines.push(`Instance: ${instanceName}`);
    lines.push(`Occupied length L: ${result.usedLength.toFixed(4)}`);
    lines.push(`Fixed board width W: ${result.binW}`);
    lines.push(`Total piece area S: ${result.materialArea.toFixed(4)}`);
    lines.push(`Board utilization: ${result.utilizationPercent.toFixed(4)}%`);
    lines.push(`Placed: ${result.placements.length}`);
    lines.push('');
    lines.push('Sheet outline:');
    lines.push('NUMBER OF VERTICES');
    lines.push(String(outputBinPolygon.length));
    lines.push('VERTICES (X,Y)');
    outputBinPolygon.forEach(v => lines.push(`${formatCoordinate(v.x)}  ${formatCoordinate(v.y)}`));
    lines.push('');
    lines.push('Pieces on sheet 1:');
    result.placements.forEach((part, idx) => {
        lines.push(`\nPiece ${idx + 1}:`);
        lines.push(`ID: ${part.partId}`);
        lines.push(`Position: (${formatCoordinate(part.x + partOutputOffset.x)}, ${formatCoordinate(part.y + partOutputOffset.y)})`);
        lines.push(`Rotation angle: ${part.rotation}°`);
        const polygon = outputParts[part.partId];
        const rotated = rotatePolygon(polygon, part.rotation);
        const offX = part.x + partOutputOffset.x;
        const offY = part.y + partOutputOffset.y;
        lines.push('OUTER');
        lines.push('NUMBER OF VERTICES');
        lines.push(String(rotated.length));
        lines.push('VERTICES (X,Y)');
        rotated.forEach(v => lines.push(`${formatCoordinate(v.x + offX)}  ${formatCoordinate(v.y + offY)}`));
        if (rotated.children && rotated.children.length > 0) {
            for (let i = 0; i < rotated.children.length; i++) {
                lines.push('');
                lines.push(`HOLE ${i + 1}`);
                lines.push('NUMBER OF VERTICES');
                lines.push(String(rotated.children[i].length));
                lines.push('VERTICES (X,Y)');
                rotated.children[i].forEach(v => lines.push(`${formatCoordinate(v.x + offX)}  ${formatCoordinate(v.y + offY)}`));
            }
        }
    });

    const txtPath = path.join(outDir, `stbbs_${instanceName}.txt`);
    fs.writeFileSync(txtPath, lines.join('\n'), 'utf8');

    let svgPath = null;
    try {
        const { NestingGenerator } = require('./nestingGenerator.js');
        const gen = new NestingGenerator();
        gen.parseFile(fs.readFileSync(txtPath, 'utf8'));
        const svg = gen.generate();
        svgPath = path.join(outDir, `stbbs_${instanceName}.svg`);
        fs.writeFileSync(svgPath, svg, 'utf8');
    } catch (e) {
        console.log('SVG generation failed (does not affect the result): ' + e.message);
    }
    return { txtPath, svgPath };
}

// ============================================
// 8. Solving core (shared by normal and diagnostic modes, ensuring the same-seed trajectory is consistent)
//    When diagConfig is non-empty, inject the diagnostic context into solverConfig.diag and the LNS config.diag.
// ============================================
async function coreSolve(instanceName, shapeFile, piecesFile, diagConfig) {
    if (ESICUP_ROTATIONS[instanceName]) {
        CONFIG.rotationAngles = ESICUP_ROTATIONS[instanceName];
        CONFIG.rotations = ESICUP_ROTATIONS[instanceName].length;
    }
    const angles = getRotationAngles();
    const loaded = loadInstance(instanceName, shapeFile, piecesFile);
    const N = loaded.allParts.length;
    const prepared = prepareInputs(loaded.bin, loaded.allParts);
    const binW = prepared.binPolygon.height;
    const nfpCache = await precomputeNfp(prepared.binPolygon, prepared.parts);

    // Geometry context
    const geomCtx = createGeometry({
        parts: prepared.parts,
        binW: binW,
        nfpCache: nfpCache,
        config: CONFIG
    });

    // Type recognition
    const typeInfo = groupTypes(prepared.parts, geomCtx);
    const M = typeInfo.M;

    // ω: 0 = by ceil(M/3); then capped by omegaCap
    let omega = CONFIG.omega > 0 ? CONFIG.omega : Math.max(1, Math.ceil(M / 3));
    omega = Math.max(1, omega);
    if (CONFIG.omegaCap > 0) {
        omega = Math.min(omega, CONFIG.omegaCap);
    }

    const solverConfig = {
        alpha: CONFIG.alpha,
        beta: CONFIG.beta,
        omega: omega,
        omegaCap: CONFIG.omegaCap,
        timeBudgetMs: CONFIG.timeBudgetMs,
        s: CONFIG.s,
        rho: CONFIG.rho,
        epsilon: CONFIG.epsilon,
        tabuLength: CONFIG.tabuLength,
        noImproveLimit: CONFIG.noImproveLimit,
        mergePolicy: CONFIG.mergePolicy,
        areaPriority: CONFIG.areaPriority,
        areaPriorityWeight: CONFIG.areaPriorityWeight,
        preserveLargeBranch: CONFIG.preserveLargeBranch,
        enablePoseBranching: CONFIG.enablePoseBranching,
        poseK: CONFIG.poseK,
        enableBranchDiversity: CONFIG.enableBranchDiversity,
        enableRollout: CONFIG.enableRollout,
        rolloutMaxPieces: CONFIG.rolloutMaxPieces,
        enableLns: CONFIG.enableLns,
        lnsBudgetMs: CONFIG.lnsBudgetMs,
        lnsMaxIters: CONFIG.lnsMaxIters,
        lnsNoImproveLimit: CONFIG.lnsNoImproveLimit,
        lnsOperators: CONFIG.lnsOperators,
        seed: CONFIG.seed,
        debug: process.env.STBBS_DEBUG === '1'
    };

    // test1_5: inject the diagnostic context (pure bypass, RNG-free).
    // The diagnostic switch only changes "whether to record events", it does not alter any sorting/sampling/RNG call order.
    // diagConfig is {enabled, level, runId, runDir}; when non-empty, create a unique diag,
    // inject it into solverConfig.diag (shared by stbbs.js and lns.js), and return it with ctx for assembly reuse.
    var createdDiag = null;
    if (diagConfig) {
        createdDiag = createDiagnostics({
            enabled: diagConfig.enabled,
            level: diagConfig.level || 'summary',
            runId: diagConfig.runId,
            runDir: diagConfig.runDir,
            areaById: typeInfo.areaById
        });
        solverConfig.diag = createdDiag;
        solverConfig.lnsBudgetMs = CONFIG.lnsBudgetMs; // LNS reuses the same diag
    }

    const instance = {
        N: N,
        M: M,
        angles: angles,
        areaById: typeInfo.areaById,
        typeOfInstance: typeInfo.typeOfInstance,
        typeInstances: typeInfo.typeInstances
    };

    const rng = createRng(CONFIG.seed);
    const solver = createSolver(geomCtx, instance, solverConfig, rng);
    const algoStart = Date.now();
    const result = solver.runStbbs();
    const algoSec = (Date.now() - algoStart) / 1000;

    // Layout geometry fingerprint (deterministic, independent of the diagnostic switch; used to verify non-perturbation)
    const layoutFingerprint = (result.placements || []).map(p =>
        `${p.partId}@${Math.round(p.x * 1e4)},${Math.round(p.y * 1e4)}@${p.rotation}`
    ).join(';');

    return {
        instanceName, N, M, binW, prepared, angles, typeInfo,
        result, algoSec, omega, solverConfig, layoutFingerprint, nfpCache,
        diag: createdDiag
    };
}

// ============================================
// 8.1 Single instance solve (normal mode, wraps coreSolve)
// ============================================
async function solveInstance(instanceName, shapeFile, piecesFile) {
    console.log('\n========================================');
    console.log(`  Instance: ${instanceName} (Θ=${JSON.stringify(getRotationAngles())})`);
    console.log('========================================');

    const startedAt = Date.now();
    const ctx = await coreSolve(instanceName, shapeFile, piecesFile, null);
    const { instanceName: name, N, M, result } = ctx;
    const algoSec = ctx.algoSec;
    const fullSec = (Date.now() - startedAt) / 1000;
    console.log(`\n---- ${instanceName} result ----`);
    if (CONFIG.debug) {
        console.log(`  Layout fingerprint: ${ctx.layoutFingerprint.slice(0, 64)}...`);
    }

    // TIME_BUDGET_EXCEEDED: time budget exhausted, return the best "partial" layout in the beam.
    if (result.status === 'TIME_BUDGET_EXCEEDED') {
        const placedN = result.placements.length;
        const validation = validateResult({
            placements: result.placements,
            parts: ctx.prepared.parts,
            binW: ctx.binW,
            expectedN: placedN,
            overlapTol: CONFIG.overlapTol
        });
        console.log(`  [Time budget exhausted] Partial layout: placed ${placedN}/${N}, elapsed ${algoSec.toFixed(2)}s (budget=${CONFIG.timeBudgetMs}ms)`);
        console.log(`  Occupied length L = ${result.usedLength.toFixed(4)}, partial utilization = ${result.utilizationPercent.toFixed(4)}%`);
        console.log(`  Overlapping piece pairs = ${validation.overlaps.length} (max overlap area=${validation.maxOverlap.toExponential(2)})`);
        console.log(`  Out-of-bounds pieces = ${validation.outOfBounds.length}`);
        console.log(`  Partial layout integrity check = ${validation.valid ? 'PASS ✔' : 'FAIL ✘'}`);
        const files = saveResult(result, instanceName, ctx.prepared);
        console.log(`  Result: ${files.txtPath}`);
        if (files.svgPath) console.log(`  SVG: ${files.svgPath}`);
        return {
            instance: instanceName,
            status: 'TIME_BUDGET_EXCEEDED',
            utilization: result.utilizationPercent / 100,
            occupiedLength: result.usedLength,
            placed: placedN,
            total: N,
            valid: validation.valid,
            overlaps: validation.overlaps.length,
            outOfBounds: validation.outOfBounds.length,
            elapsedSec: fullSec,
            algoSec: algoSec,
            layoutFingerprint: ctx.layoutFingerprint
        };
    }

    if (result.status !== 'OK' && result.status !== 'OK_LNS_IMPROVED') {
        console.log(`  Status: ${result.status} (no complete layout found)`);
        return { instance: instanceName, status: result.status, placed: 0, total: N, elapsedSec: fullSec, algoSec: algoSec, layoutFingerprint: ctx.layoutFingerprint };
    }
    // LNS improvement info
    if (result.stats.lnsImproved) {
        console.log(`  [LNS] iterations=${result.stats.lnsIterations} elapsed=${(result.stats.lnsElapsedMs / 1000).toFixed(2)}s improved=${result.stats.lnsImproved}`);
    }

    // Integrity check (overlap / out-of-bounds / count)
    const validation = validateResult({
        placements: result.placements,
        parts: ctx.prepared.parts,
        binW: ctx.binW,
        expectedN: N,
        overlapTol: CONFIG.overlapTol
    });

    console.log(`  Occupied length L = ${result.usedLength.toFixed(4)}`);
    console.log(`  Board utilization = ${result.utilizationPercent.toFixed(4)}%`);
    console.log(`  Placed = ${result.placements.length}/${N}`);
    console.log(`  Area check: resultS=${result.materialArea.toFixed(4)}  checkS=${validation.area.toFixed(4)}`);
    console.log(`  Overlapping piece pairs = ${validation.overlaps.length} (max overlap area=${validation.maxOverlap.toExponential(2)})`);
    console.log(`  Out-of-bounds pieces = ${validation.outOfBounds.length}`);
    console.log(`  Trigger layers (Tabu) = [${result.stats.triggerLayers.join(', ')}], Tabu calls=${result.stats.tabuCalls}, nodes=${result.stats.totalNodes}`);
    console.log(`  Algorithm elapsed = ${algoSec.toFixed(2)}s, full pipeline = ${fullSec.toFixed(2)}s`);
    console.log(`  Integrity check = ${validation.valid ? 'PASS ✔' : 'FAIL ✘'}`);

    const files = saveResult(result, instanceName, ctx.prepared);
    console.log(`  Result: ${files.txtPath}`);
    if (files.svgPath) console.log(`  SVG: ${files.svgPath}`);

    return {
        instance: instanceName,
        status: 'OK',
        utilization: result.utilizationPercent / 100,
        occupiedLength: result.usedLength,
        placed: result.placements.length,
        total: N,
        valid: validation.valid,
        overlaps: validation.overlaps.length,
        outOfBounds: validation.outOfBounds.length,
        elapsedSec: fullSec,
        algoSec: algoSec,
        layoutFingerprint: ctx.layoutFingerprint
    };
}

// ============================================
// 8.2 Diagnostic assembly (test1_5: pure bypass, generates diagnostic files from real run records)
//    Does not re-solve, only consumes the real products of coreSolve (result/prepared/typeInfo).
// ============================================

// Geometry fingerprint (normalized string of partId,x,y,rot)
function geomFpFromPlacements(preparedParts, placements) {
    const ps = [];
    for (let i = 0; i < placements.length; i++) {
        const p = placements[i];
        ps.push(`${p.partId}@${Math.round(p.x * 1e4)},${Math.round(p.y * 1e4)}@${p.rotation}`);
    }
    return ps.length + ':' + ps.join(';');
}

// Geometry fingerprint (built from the GeometryState replay product, format consistent with geomFpFromPlacements)
function geomFpFromState(g) {
    if (!g || !g.placed || g.placed.length === 0) return '0:';
    const ps = [];
    for (let i = 0; i < g.placed.length; i++) {
        const p = g.placed[i], pos = g.placements[i];
        ps.push(`${p.partId}@${Math.round(pos.x * 1e4)},${Math.round(pos.y * 1e4)}@${p.rot}`);
    }
    return ps.length + ':' + ps.join(';');
}

// Single-stage placement trace (stage='beam_completed'|'final_after_lns')
// Deterministic replay of placeOneFn in the order of placements[i].partId (does not consume the solving RNG).
// x/y/rot and L_before/L_after/delta_L all come from the same replay geometry (g.placed[i]/g.placements[i]),
// ensuring the positions and occupied length of this stage are self-consistent—even if LNS reordered things, the beam stage is still the real geometry replayed in beam order,
// rather than pairing final's x,y with beam's delta_L.
// placeOneFn(state, partId, angles) must be the real placeOne from the same geomCtx.
function assemblePlacementTrace(stage, preparedParts, binW, angles, placements, areaById, typeOfInstance, areaRankById, placeOneFn, newStateFn) {
    const N = placements.length;
    const rows = [];
    let g = newStateFn();
    for (let i = 0; i < N; i++) {
        const pl = placements[i];
        const partId = pl.partId;
        // Deterministic replay: placeOne places the (i+1)-th piece on the real geometry (order determined by pl.partId)
        const L_before = g.L;
        const r = placeOneFn(g, partId, angles);
        if (r) {
            g = r.geometry;
        }
        const L_after = g.L;
        const delta_L = L_after - L_before;
        // Position/pose taken from the replay geometry (g.placed[i]/g.placements[i]); falls back to the passed-in values on replay failure
        const gp = (g.placed && g.placed[i]) ? g.placed[i] : null;
        const gpos = (g.placements && g.placements[i]) ? g.placements[i] : null;
        const rot = gp ? gp.rot : pl.rotation;
        const px = gpos ? gpos.x : pl.x;
        const py = gpos ? gpos.y : pl.y;
        const part = preparedParts[partId];
        const rotPoly = GeometryUtil.rotatePolygon(part, rot);
        const bb = GeometryUtil.getPolygonBounds(rotPoly);
        const bboxMinX = bb.x + px, bboxMaxX = bboxMinX + bb.width;
        const bboxMinY = bb.y + py, bboxMaxY = bboxMinY + bb.height;
        const area = areaById[partId] || 0;
        rows.push({
            run_id: null, // filled in by the caller
            stage: stage,
            instance_id: partId,
            type_id: typeOfInstance[partId] != null ? typeOfInstance[partId] : null,
            net_area: area,
            area_rank: areaRankById[partId] != null ? areaRankById[partId] : null,
            area_percentile: null, // derived by diagnostics
            placement_index: i + 1,
            theta: rot,
            x: px, y: py,
            bbox_min_x: bboxMinX, bbox_max_x: bboxMaxX,
            bbox_min_y: bboxMinY, bbox_max_y: bboxMaxY,
            L_before: L_before, L_after: L_after, delta_L: delta_L,
            old_length_feasible: Math.abs(delta_L) < 1e-6,
            old_length_selected: (stage === 'final_after_lns') && (Math.abs(delta_L) < 1e-6),
            part_fingerprint: `${partId}@${Math.round(px * 1e4)},${Math.round(py * 1e4)}@${rot}`,
            geometry_fingerprint: geomFpFromState(g)
        });
    }
    return rows;
}

// Single diagnostic run (worker or in-process call)
async function solveForDiag(job, instancePayload) {
    const { instanceName, shapeFile, piecesFile, seed, codeVersion, machineInfo, totalMaterialArea, nfpPrecomputeScope } = job;
    CONFIG.seed = seed;
    // Per-run override (passed via workerData; the CONFIG inside a worker is independent and does not interfere):
    //   job.budgetMs   —— solving time budget; 0 = unlimited (complete nesting, ensuring LNS is triggered only after all N pieces are placed)
    //   job.maxThreads —— number of NFP precompute threads; avoids oversubscription of 28 workers × 31 threads
    //   job.level      —— diagnostic level ('full' collects the candidate elimination details)
    // Default (no override) is equivalent to the original behavior: budgetMs=diagTimeBudgetMs, threads=maxThreads, level=diagnosticLevel.
    const budgetMs = (job.budgetMs != null) ? job.budgetMs : (CONFIG.diagTimeBudgetMs || CONFIG.timeBudgetMs);
    CONFIG.timeBudgetMs = budgetMs;
    if (job.maxThreads != null) CONFIG.maxThreads = job.maxThreads;
    const startedAt = new Date().toISOString();
    // test1_5: the diagnostic context is uniformly created by coreSolve before coreSolve and injected into solverConfig.diag,
    // so that beam_candidates / beam_layers / tabu_events are captured during solving (pure bypass, RNG-free).
    // solveForDiag reuses the same diag to complete the placement_trace / layouts / validation / metadata assembly.
    const diagConfig = {
        enabled: true,
        level: (job.level != null) ? job.level : (CONFIG.diagnosticLevel || 'summary'),
        runId: job.runId,
        runDir: job.runDir
    };
    const ctx = await coreSolve(instanceName, shapeFile, piecesFile, diagConfig);
    const diag = ctx.diag;

    const result = ctx.result;
    const prepared = ctx.prepared;
    const placements = result.placements;

    // placeOne for deterministic replay (reuses the precomputed NFP cache, does not consume the solving RNG).
    // placeOne is internally deterministic (topCands takes the K smallest); same geometry + same input always yields the same pose.
    const replayGeom = createGeometry({
        parts: prepared.parts,
        binW: ctx.binW,
        nfpCache: ctx.nfpCache,
        config: CONFIG
    });
    const placeOneFn = (state, partId, angles) => replayGeom.placeOne(state, partId, angles);
    const newStateFn = () => replayGeom.newState();

    // ---- placement_trace: beam_completed (original STBBS winner) + final_after_lns ----
    // test1_5 §1: save the four orders separately.
    //   beam_placement_order   = the STBBS beam-search winner order before LNS (result.stats.beamOrder;
    //                            equals result.sequence when truncated by the time budget or when there is no LNS).
    //   tabu_redecoded_order   = Tabu repair happens inside the beam search, and the winner is already its result, so it equals the beam order;
    //                            listed separately to indicate that Tabu can reorder and that this is reflected here.
    //   lns_reconstructed_order= the reconstructed order after LNS improvement (exists only when LNS truly improved; otherwise null).
    //   final_placement_order  = the actual final order = result.sequence (the LNS order after improvement, otherwise the beam order).
    const lnsImproved = !!result.stats.lnsImproved;
    const finalOrder = (result.sequence || []).slice();
    const beamOrder = (result.stats.beamOrder && result.stats.beamOrder.length > 0)
        ? result.stats.beamOrder.slice()
        : finalOrder.slice();
    const tabuOrder = beamOrder.slice();
    const lnsOrder = lnsImproved ? finalOrder.slice() : null;

    // Two-stage placement trace: beam_completed replayed in beam order, final_after_lns replayed in the final order.
    // The replay drives placeOne with partId internally; x/y/rot are taken from the replay geometry (self-consistent, no mixing).
    const beamOrderPlacements = beamOrder.map(id => ({ partId: id }));
    const finalOrderPlacements = finalOrder.map(id => ({ partId: id }));
    const beamRows = assemblePlacementTrace('beam_completed', prepared.parts, ctx.binW, ctx.angles, beamOrderPlacements, ctx.typeInfo.areaById, ctx.typeInfo.typeOfInstance, diag.areaRankById, placeOneFn, newStateFn);
    const finalRows = assemblePlacementTrace('final_after_lns', prepared.parts, ctx.binW, ctx.angles, finalOrderPlacements, ctx.typeInfo.areaById, ctx.typeInfo.typeOfInstance, diag.areaRankById, placeOneFn, newStateFn);
    for (const r of beamRows) { r.run_id = job.runId; diag.appendPlacementTrace(r); }
    for (const r of finalRows) { r.run_id = job.runId; diag.appendPlacementTrace(r); }

    // Order fingerprint (used for LNS distinguishability: sequence change vs geometry change)
    const seqFp = (arr) => (arr ? arr.length + ':' + arr.join(',') : null);

    // ---- test1_5 §5: independently rebuild L_geom (the true max_x of the rotated contour vertices, trusting no cached L) ----
    // Compute max_x using the rotated actual contour vertices of all placed pieces, independent of L_cached (the state.L cache);
    // the two must be consistent; inconsistency means placements and used_length come from different geometry versions (the mixing forbidden by §5).
    // Only outer contour vertices (hole children do not exceed the outer contour's x range).
    function computeLGeom(partList) {
        let mx = 0;
        for (const pl of partList) {
            const part = prepared.parts[pl.partId];
            if (!part) continue;
            const rot = (pl.rot != null) ? pl.rot : pl.rotation;
            const rotPoly = GeometryUtil.rotatePolygon(part, rot);
            for (const v of rotPoly) {
                const wx = v.x + pl.x;
                if (wx > mx) mx = wx;
            }
        }
        return mx;
    }
    // Real-geometry placements (same source and version as used_length):
    //   source = the real geometry of the Beam winner (before LNS, result.stats.beamWinnerPlacements/L)
    //   final  = the final real geometry (result.placements/usedLength; the Beam winner when LNS did not improve)
    const beamRealPlacements = (result.stats.beamWinnerPlacements && result.stats.beamWinnerPlacements.length > 0)
        ? result.stats.beamWinnerPlacements
        : placements; // fallback: use result.placements when there is no Beam snapshot (e.g. a partial layout)
    const beamRealL = (result.stats.beamWinnerL != null) ? result.stats.beamWinnerL
        : (beamRows.length > 0 ? beamRows[beamRows.length - 1].L_after : null);

    // ---- source_layout.json (Beam complete, before LNS, real winner geometry) ----
    // placement_order and placements share the same source (the real winner prefix order), ensuring the sequence and geometry are the same version.
    const sourceRealOrder = beamRealPlacements.map(p => p.partId);
    const sourceLayout = {
        run_id: job.runId,
        stage: 'beam_completed',
        placement_order: sourceRealOrder,
        placements: beamRealPlacements.map(p => ({ instance_id: p.partId, theta: (p.rot != null) ? p.rot : p.rotation, x: p.x, y: p.y })),
        used_length: beamRealL,                       // L_cached: the Beam winner's cached L
        L_geom: computeLGeom(beamRealPlacements),     // independently rebuilt true max_x of the rotated contour
        geometry_fingerprint: (beamRows.length > 0) ? beamRows[beamRows.length - 1].geometry_fingerprint : null,
        sequence_fingerprint: seqFp(sourceRealOrder)
    };
    // ---- final_layout.json (after LNS, real final geometry) ----
    const finalLayout = {
        run_id: job.runId,
        stage: 'final_after_lns',
        placement_order: finalOrder,
        placements: placements.map(p => ({ instance_id: p.partId, theta: (p.rotation != null) ? p.rotation : p.rot, x: p.x, y: p.y })),
        used_length: result.usedLength,               // L_cached: the final winner's cached L
        L_geom: computeLGeom(placements),             // independently rebuilt true max_x of the rotated contour
        geometry_fingerprint: (finalRows.length > 0) ? finalRows[finalRows.length - 1].geometry_fingerprint : geomFpFromPlacements(prepared.parts, placements),
        sequence_fingerprint: seqFp(finalOrder),
        lns_improved: lnsImproved,
        // test1_5 §1: save the four orders separately (lns_reconstructed_order is null when LNS did not improve)
        placement_orders: {
            beam_placement_order: beamOrder,
            tabu_redecoded_order: tabuOrder,
            lns_reconstructed_order: lnsOrder,
            final_placement_order: finalOrder
        }
    };
    diag.writeLayout('source_layout.json', sourceLayout);
    diag.writeLayout('final_layout.json', finalLayout);

    // ---- result.svg (bound to run_id) ----
    let svgHash = null;
    let svgGeometryConsistent = null;
    try {
        const files = saveResult(result, instanceName, prepared);
        if (files.svgPath) {
            const svgText = fs.readFileSync(files.svgPath, 'utf8');
            diag.writeImage(svgText);
            svgHash = crypto.createHash('sha256').update(svgText).digest('hex');
            // SVG geometry consistency: SVG piece count == placed piece count
            const { NestingGenerator } = require('./nestingGenerator.js');
            const gen = new NestingGenerator();
            gen.parseFile(fs.readFileSync(files.txtPath, 'utf8'));
            svgGeometryConsistent = (gen.pieces.length === placements.length);
        }
    } catch (e) {
        svgHash = null;
    }

    // ---- geometry_validation.json ----
    // expectedN is always the full piece count ctx.N: a partial layout (actual_count<N) is judged INCOMPLETE and must not masquerade as success.
    // Even if overlap_count=0, the completeness check must not be bypassed (§5).
    const validation = validateResult({
        placements: placements,
        parts: prepared.parts,
        binW: ctx.binW,
        expectedN: ctx.N,
        overlapTol: CONFIG.overlapTol
    });
    // L_geom true value: independently rebuilt from the rotated contour vertex max_x (trusts no cached L, and contains no SVG background padding).
    let lGeom = 0;
    for (const p of placements) {
        const rot = GeometryUtil.rotatePolygon(prepared.parts[p.partId], p.rotation);
        for (const v of rot) {
            const wx = v.x + p.x;
            if (wx > lGeom) lGeom = wx;
        }
    }
    const lCached = result.usedLength;
    // The area reconciliation must use the same geometry version: the placement's x/y are translations relative to the
    // "offset contour" (nest.js offsetTree expanded by 0.5*spacing), so the layout-side area must be computed with
    // prepared.parts (the offset geometry); totalMaterialArea is the original geometry (loadInstance).
    // The old code subtracted totalMaterialArea (original geometry) from validation.area (which internally reads
    // prepared.parts, i.e. the offset geometry); the outer-contour expansion made the offset area > the original area,
    // and the difference grew with perimeter/spacing: rectangular instances (large perimeter) differed by 0.03~0.40 and were
    // judged false, while swim (the difference between the original and offset areas, a negative difference) was also judged false.
    // Both are geometry-version mixing, with the same origin as the L_geom/L_cached fixed by §5.
    // Fix: independently rebuild layoutArea with prepared.parts, and reconcile it against the same-version result.materialArea
    // (computed by the solver with prepared.parts). Both are offset geometry, so the reconciliation only verifies "layout geometry
    // self-consistency"; the inherent difference between original and offset is explained by a field other than area_consistent (area_source_raw).
    let layoutArea = 0;
    for (const pl of placements) {
        const polyZ = prepared.parts[pl.partId];
        if (!polyZ) continue;
        let aZ = Math.abs(GeometryUtil.polygonArea(polyZ));
        if (polyZ.children) {
            for (const hz of polyZ.children) aZ -= Math.abs(GeometryUtil.polygonArea(hz));
        }
        layoutArea += Math.max(0, aZ);
    }
    const areaConsistent = Math.abs(layoutArea - result.materialArea) <= Math.max(1e-3, result.materialArea * 1e-6);
    const complete = (placements.length === ctx.N) && (validation.overlaps.length === 0)
        && (validation.outOfBounds.length === 0) && areaConsistent;
    const status = (result.status === 'TIME_BUDGET_EXCEEDED') ? 'INCOMPLETE'
        : (complete ? 'COMPLETE_VALID' : (validation.overlaps.length === 0 && validation.outOfBounds.length === 0 ? 'INCOMPLETE' : 'INVALID_GEOMETRY'));
    const utilRecalced = (ctx.binW * lGeom > 0) ? (result.materialArea / (ctx.binW * lGeom) * 100) : null;
    diag.writeGeometryValidation({
        expected_count: ctx.N,
        actual_count: placements.length,
        unique_instance_count: new Set(placements.map(p => p.partId)).size,
        total_area_source: totalMaterialArea,
        total_area_layout: validation.area,
        area_consistent: areaConsistent,
        L_geom: lGeom,                       // independently rebuilt true max_x of the rotated contour
        L_cached: lCached,                   // search-cached L (winner.geometry.L)
        L_geom_consistent: Math.abs(lGeom - lCached) <= Math.max(1e-3, lCached * 1e-6),
        strip_width: ctx.binW,
        max_x: lGeom,
        length_reported: lCached,
        utilization_recalculated: utilRecalced,
        overlap_count: validation.overlaps.length,
        max_overlap_area: validation.maxOverlap,
        out_of_bounds_count: validation.outOfBounds.length,
        status: status,
        svg_file_hash: svgHash,
        svg_geometry_consistent: svgGeometryConsistent
    });

    // ---- metadata.json ----
    diag.setMeta({
        run_id: job.runId,
        instance_name: instanceName,
        instance_sha256: job.instanceSha256,
        algorithm_name: 'STBBS',
        code_commit: codeVersion,
        code_dirty: true,
        seed: seed,
        part_count: ctx.N,
        type_count: ctx.M,
        strip_width: ctx.binW,
        total_material_area: totalMaterialArea,
        config: {
            omega: ctx.omega,
            alpha: CONFIG.alpha, beta: CONFIG.beta,
            areaPriority: CONFIG.areaPriority,
            areaPriorityWeight: CONFIG.areaPriorityWeight,
            preserveLargeBranch: CONFIG.preserveLargeBranch,
            enablePoseBranching: CONFIG.enablePoseBranching,
            poseK: CONFIG.poseK,
            enableBranchDiversity: CONFIG.enableBranchDiversity,
            enableRollout: CONFIG.enableRollout,
            enableLns: CONFIG.enableLns,
            lnsBudgetMs: CONFIG.lnsBudgetMs,
            timeBudgetMs: CONFIG.timeBudgetMs,
            s: CONFIG.s, rho: CONFIG.rho, epsilon: CONFIG.epsilon,
            tabuLength: CONFIG.tabuLength, noImproveLimit: CONFIG.noImproveLimit,
            mergePolicy: CONFIG.mergePolicy, maxThreads: CONFIG.maxThreads
        },
        ...machineInfo,
        worker_threads_requested: CONFIG.maxThreads,
        worker_threads_actual: CONFIG.maxThreads,
        nfp_precompute_scope: nfpPrecomputeScope,
        started_at: startedAt,
        completed_at: new Date().toISOString()
    });

    diag.complete();

    return {
        runId: job.runId,
        seed: seed,
        status: result.status,
        utilization: result.utilizationPercent / 100,
        placed: placements.length,
        total: ctx.N,
        layoutFingerprint: ctx.layoutFingerprint,
        elapsedMs: Date.now() - new Date(startedAt).getTime()
    };
}

// ============================================
// 8.3 Diagnostic parallel worker (worker_threads, independent seeds, independent run directories)
// ============================================
function diagWorkerMain() {
    const { parentPort, workerData } = require('worker_threads');
    const job = workerData;
    (async () => {
        try {
            const out = await solveForDiag(job, null);
            parentPort.postMessage({ ok: true, runId: job.runId, out });
        } catch (e) {
            parentPort.postMessage({ ok: false, runId: job.runId, error: e.message });
        }
    })();
}

// ============================================
// 8.4 Diagnostic mode entry (--diagnose <inst>: multi-seed parallel + aggregation)
// ============================================
// options: { budgetMs, maxThreads, level, seeds, seedsBase, workers } —— optional overrides
//   budgetMs   : solving time budget; 0 = unlimited (complete nesting, ensuring LNS is triggered after all N pieces are placed). Default diagTimeBudgetMs
//   maxThreads : number of NFP precompute threads. Default auto-converges to min(CONFIG.maxThreads, max(1, cpus/nWorkers)),
//                avoiding oversubscription of multiple workers × multiple NFP threads (resource contention for a single instance).
//   level      : diagnostic level ('full' collects the candidate elimination details). Default CONFIG.diagnosticLevel
//   seeds      : number of seeds. Default CONFIG.diagSeeds
//   seedsBase  : seed base. Default CONFIG.diagSeedsBase
//   workers    : concurrent worker cap. Default CONFIG.maxWorkers
async function runDiagnostics(instanceNames, options) {
    const opts = options || {};
    const codeVersion = computeCodeFingerprint();
    const machineInfo = collectMachineInfo();
    const startedAt = new Date().toISOString();
    const nSeeds = (opts.seeds != null) ? opts.seeds : Math.max(1, CONFIG.diagSeeds);
    const seedsBase = (opts.seedsBase != null) ? opts.seedsBase : CONFIG.diagSeedsBase;
    const seeds = [];
    for (let i = 0; i < nSeeds; i++) {
        seeds.push(seedsBase + i);
    }
    // Concurrent worker cap and NFP threads per worker:
    //   workers = min(opts.workers or maxWorkers, total job count)
    //   nfpThreads = min(opts.maxThreads or maxThreads, max(1, floor(cpus / workers)))
    //   The product is ≤ cpus (approximately), avoiding oversubscription like 28×31; a single instance is not preempted by other workers.
    const nWorkers = Math.min((opts.workers != null) ? opts.workers : CONFIG.maxWorkers, Math.max(1, instanceNames.length * nSeeds));
    const cpus = (typeof os !== 'undefined' && os.cpus && os.cpus().length) ? os.cpus().length : 32;
    let nfpThreads = (opts.maxThreads != null) ? opts.maxThreads : Math.min(CONFIG.maxThreads, Math.max(1, Math.floor(cpus / nWorkers)));
    nfpThreads = Math.max(0, nfpThreads);
    const budgetMs = (opts.budgetMs != null) ? opts.budgetMs : (CONFIG.diagTimeBudgetMs || CONFIG.timeBudgetMs);
    const level = (opts.level != null) ? opts.level : (CONFIG.diagnosticLevel || 'summary');

    // ---- Phase 1: flatten all jobs across instances (one job per (instance, seed)) ----
    // Instances are no longer serial; all jobs go into the same worker pool and run concurrently (capped at maxWorkers=31),
    // to fully utilize the 31 threads. Workers do not share mutable state; each writes its own run directory.
    const allJobs = [];
    const baseDirByName = {};
    for (const name of instanceNames) {
        const shapeFile = path.join(EXAMPLE_DIR, 'shape_' + name + '.txt');
        const piecesFile = path.join(EXAMPLE_DIR, 'pieces_' + name + '.txt');
        if (!fs.existsSync(shapeFile) || !fs.existsSync(piecesFile)) {
            console.error(`[diagnose] Instance file missing: ${shapeFile}`);
            continue;
        }
        const instanceSha256 = computeInstanceSha256(shapeFile, piecesFile);
        // Net area
        const loaded = loadInstance(name, shapeFile, piecesFile);
        let totalMaterialArea = 0;
        for (const part of loaded.allParts) {
            let a = Math.abs(GeometryUtil.polygonArea(part));
            if (part.children) {
                for (const c of part.children) a -= Math.abs(GeometryUtil.polygonArea(c));
            }
            totalMaterialArea += Math.max(0, a);
        }
        const baseDir = path.join(__dirname, CONFIG.diagnosticDir, name);
        baseDirByName[name] = baseDir;

        for (const seed of seeds) {
            const runId = makeRunId(name, codeVersion, seed, startedAt);
            const runDir = path.join(baseDir, 'run_' + runId);
            allJobs.push({
                instanceName: name, shapeFile, piecesFile, seed,
                codeVersion, machineInfo, totalMaterialArea,
                nfpPrecomputeScope: 'all-pairs-per-angle (worker pool, cap ' + nfpThreads + ')',
                runId, runDir, instanceSha256,
                budgetMs: budgetMs, maxThreads: nfpThreads, level: level
            });
        }
    }

    if (allJobs.length === 0) {
        console.error('[diagnose] No runnable instances (all files missing)');
        return;
    }

    console.log('\n========================================');
    console.log(`  Diagnostic mode (all-instance parallel pool): ${instanceNames.length} instances × ${seeds.length} seeds = ${allJobs.length} jobs`);
    console.log(`  Concurrent worker cap: ${nWorkers}; NFP threads per worker: ${nfpThreads} (workers×threads=${nWorkers * nfpThreads} ≤ cpus=${cpus})`);
    console.log(`  Solving budget: ${budgetMs === 0 ? 'unlimited (complete nesting+LNS)' : budgetMs + 'ms'}; level=${level}`);
    console.log(`  code_commit=${codeVersion}`);
    console.log('========================================');

    // ---- Phase 2: worker pool concurrent execution (at most nWorkers concurrent, refill one as one completes) ----
    const results = [];
    const workerFile = __filename; // stbbs-cli.js itself (contains diagWorkerMain)
    let nextJob = 0;
    let finished = 0;
    let active = 0;

    await new Promise((resolve) => {
        const doneCheck = () => {
            if (finished === allJobs.length) resolve();
        };
        const spawnNext = () => {
            if (nextJob >= allJobs.length) return;
            const idx = nextJob++;
            const job = allJobs[idx];
            active++;
            const w = new Worker(workerFile, {
                workerData: { ...job, _diagWorker: true }
            });
            w.on('message', (msg) => {
                results.push(msg);
                active--;
                finished++;
                console.log(`  [diag ${finished}/${allJobs.length}] ${job.instanceName} seed=${job.seed} ${msg.ok ? 'OK' : 'ERR:' + msg.error}`);
                spawnNext();
                doneCheck();
            });
            w.on('error', (e) => {
                results.push({ ok: false, runId: job.runId, error: e.message });
                active--;
                finished++;
                console.error(`  [diag ${finished}/${allJobs.length}] ${job.instanceName} seed=${job.seed} WORKER_ERROR: ${e.message}`);
                spawnNext();
                doneCheck();
            });
        };
        // Initially fill the worker pool
        for (let i = 0; i < nWorkers && i < allJobs.length; i++) spawnNext();
    });

    // ---- Phase 3: per-instance aggregation (main thread, after all jobs finish) ----
    for (const name of instanceNames) {
        const baseDir = baseDirByName[name];
        if (!baseDir) continue;
        const agg = areaOrderReport.analyzeInstanceDir(baseDir);
        areaOrderReport.writeAggregate(baseDir, agg);
        console.log(`  Aggregation done: ${name} (${agg.runs.length} runs) → ${path.join(baseDir, 'aggregate')}`);
    }
    const okCount = results.filter(r => r.ok).length;
    console.log(`\n  All diagnostics complete: ${okCount}/${allJobs.length} jobs succeeded.`);
}

// ============================================
// 9. Main function
// ============================================
async function main() {
    console.log('========================================');
    console.log('  STBBS nesting algorithm');
    console.log('  Stagnation-triggered tabu-backtracking-enhanced beam search');
    console.log('========================================');

    const argMode = process.argv[2];
    const arg2 = process.argv[3];
    let jobs = [];

    if (argMode === '--instance') {
        const which = (arg2 || '').trim();
        if (!which) {
            console.error('Error: --instance requires an instance name / all / comma-separated list');
            process.exit(1);
        }
        const names = (which === 'all') ? ESICUP_INSTANCES : which.split(',').map(s => s.trim()).filter(Boolean);
        jobs = names.map(name => ({
            name,
            shapeFile: path.join(EXAMPLE_DIR, 'shape_' + name + '.txt'),
            piecesFile: path.join(EXAMPLE_DIR, 'pieces_' + name + '.txt')
        }));
    } else if (argMode === '--diagnose') {
        // test1_5 diagnostic mode: multi-seed parallel + offline aggregation, pure bypass that does not affect the solving trajectory
        // Optional override flags: --budget <ms|0> (0 = unlimited, complete nesting+LNS) --threads <n> (NFP threads)
        //              --level <summary|full> --seeds <n> --workers <n> --seeds-base <n>
        const which = (arg2 || '').trim();
        if (!which) {
            console.error('Error: --diagnose requires an instance name / all / comma-separated list');
            process.exit(1);
        }
        const names = (which === 'all') ? ESICUP_INSTANCES : which.split(',').map(s => s.trim()).filter(Boolean);
        const opts = {};
        for (let ai = 4; ai < process.argv.length; ai++) {
            const a = process.argv[ai];
            const v = process.argv[ai + 1];
            if (a === '--budget' && v != null) { const n = parseInt(v, 10); if (!isNaN(n) && n >= 0) { opts.budgetMs = n; ai++; } }
            else if (a === '--threads' && v != null) { const n = parseInt(v, 10); if (!isNaN(n) && n >= 0) { opts.maxThreads = n; ai++; } }
            else if (a === '--seeds' && v != null) { const n = parseInt(v, 10); if (!isNaN(n) && n >= 1) { opts.seeds = n; ai++; } }
            else if (a === '--seeds-base' && v != null) { const n = parseInt(v, 10); if (!isNaN(n)) { opts.seedsBase = n; ai++; } }
            else if (a === '--workers' && v != null) { const n = parseInt(v, 10); if (!isNaN(n) && n >= 1) { opts.workers = n; ai++; } }
            else if (a === '--level' && (v === 'summary' || v === 'full')) { opts.level = v; ai++; }
        }
        await runDiagnostics(names, opts);
        return;
    } else {
        if (!argMode || !arg2) {
            console.error('Usage: node stbbs-cli.js <shape.txt> <pieces.txt> | --instance <name|all|a,b,c>');
            process.exit(1);
        }
        const name = path.basename(argMode).replace(/^shape_?/, '').replace(/\.txt$/i, '');
        jobs = [{ name: name || 'custom', shapeFile: argMode, piecesFile: arg2 }];
    }

    const summary = [];
    for (const job of jobs) {
        try {
            if (!fs.existsSync(job.shapeFile) || !fs.existsSync(job.piecesFile)) {
                console.error(`\n[${job.name}] Instance file missing: ${job.shapeFile}`);
                summary.push({ instance: job.name, status: 'MISSING_FILE' });
                continue;
            }
            const s = await solveInstance(job.name, job.shapeFile, job.piecesFile);
            summary.push(s);
        } catch (e) {
            console.error(`\n[${job.name}] Run failed: ${e.message}`);
            console.error(e.stack);
            summary.push({ instance: job.name, status: 'ERROR', error: e.message });
        }
    }

    if (jobs.length > 1) {
        console.log('\n========================================');
        console.log('  STBBS instance summary');
        console.log('========================================');
        console.log('Instance      Utilization      L          Placed    Overlap  OOB  Check   Algo(s)  Full(s)');
        summary.forEach(s => {
            if (s.status !== 'OK' && s.status !== 'OK_LNS_IMPROVED' && s.status !== 'TIME_BUDGET_EXCEEDED') {
                console.log((s.instance || '?').padEnd(12) + ' ' + (s.status || '').padEnd(28) + (s.error || ''));
                return;
            }
            const tag = s.status === 'TIME_BUDGET_EXCEEDED' ? '(partial)' : (s.status === 'OK_LNS_IMPROVED' ? '(LNS)' : '');
            console.log(
                (s.instance + tag).padEnd(16) +
                (s.utilization != null ? (s.utilization * 100).toFixed(2).padStart(8) + '%' : '        -') +
                (s.occupiedLength != null ? s.occupiedLength.toFixed(2) : '-').padStart(10) +
                `${s.placed}/${s.total}`.padStart(10) +
                String(s.overlaps).padStart(7) +
                String(s.outOfBounds).padStart(7) +
                (s.valid ? '✔' : '✘').padStart(6) +
                (s.algoSec != null ? s.algoSec.toFixed(1) : '-').padStart(9) +
                (s.elapsedSec != null ? s.elapsedSec.toFixed(1) : '-').padStart(11)
            );
        });
    }
}

// Main-thread entry vs diagnostic worker entry.
// When worker_threads loads this file, require.main === module is also true (the worker script is the entry module),
// so isMainThread must be used to distinguish: run main() when the main thread runs directly; run diagWorkerMain() when it is a worker thread and _diagWorker is true.
(function () {
    const wt = require('worker_threads');
    if (wt.isMainThread) {
        if (require.main === module) {
            main();
        }
    } else {
        if (wt.workerData && wt.workerData._diagWorker) {
            diagWorkerMain();
        }
    }
})();

module.exports = { main, CONFIG, ESICUP_INSTANCES, solveInstance, runDiagnostics, coreSolve, solveForDiag };
