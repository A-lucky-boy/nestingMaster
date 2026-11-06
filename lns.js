/**
 * lns.js
 * test1_3 §4: large-neighborhood destroy and rebuild (LNS) on the complete layout
 *
 * The legal layout produced by the full N-layer STBBS is strengthened by LNS.
 * Four destroy-rebuild operators:
 *   1. largeForward  - large-piece forward move and local blocking relief
 *   2. tailCompress  - key tail-end compression
 *   3. gapRebuild    - large-gap region rebuild
 *   4. globalRebuild - large-scale order perturbation and global rebuild
 *
 * Each LNS iteration:
 *   - selects an operator
 *   - generates a destroy proposal (modifies the placement order)
 *   - replays all N pieces from scratch (correct but slow)
 *   - validates the new layout's legality (count = N, no out-of-bounds)
 *   - if legal and L is better, updates GlobalBest
 *
 * Randomness: controlled by the injected rng, replayable with the same seed.
 */
(function (root) {
    'use strict';

    /**
     * Run LNS strengthening from a complete layout
     * @param {Object} geomCtx - geometry context returned by createGeometry
     * @param {Object} instance - { N, M, angles, areaById, typeOfInstance, typeInstances }
     * @param {Object} layout - complete layout { prefix: [instId,...], geometry: GeometryState }
     * @param {Object} config - LNS configuration
     * @param {Function} rng - random number generator
     * @returns {Object} { bestLayout, bestL, iterations, improved, history }
     */
    function runLns(geomCtx, instance, layout, config, rng) {
        var N = instance.N;
        var angles = instance.angles;
        var areaById = instance.areaById;

        // Time budget explicitly disabled: the timeout exit is only enabled when lnsBudgetMs is a finite positive number;
        // 0 / null / undefined / Infinity are all unlimited (start is only used for logging).
        var _lnsLimitRaw = config.lnsBudgetMs;
        var hasLnsTimeLimit = Number.isFinite(_lnsLimitRaw) && _lnsLimitRaw > 0;
        var lnsBudgetMs = hasLnsTimeLimit ? _lnsLimitRaw : 0;
        // Iteration hard cap explicitly disabled: only enabled when lnsMaxIters is a finite positive number; 0 = unlimited iterations.
        // The sole termination condition is noImproveLimit (consecutive no-improvement rebuilds, default 20000).
        var _lnsItersRaw = config.lnsMaxIters;
        var hasLnsIterLimit = Number.isFinite(_lnsItersRaw) && _lnsItersRaw > 0;
        var lnsMaxIters = hasLnsIterLimit ? _lnsItersRaw : 0;
        var lnsOperators = config.lnsOperators || ['largeForward', 'tailCompress', 'gapRebuild', 'globalRebuild'];
        var lnsDebug = config.debug;
        var binW = geomCtx.binW;
        // test1_5: LNS diagnostic hook (RNG-free, pure bypass). config.diag is injected by solveInstance.
        var diag = (config.diag && config.diag.enabled) ? config.diag : null;
        var lnsEventSeq = 0;
        // Geometry fingerprint: uses the normalized string of placed pieces (partId,x,y,rot) to distinguish "sequence change" from "geometry change"
        function _geomFp(g) {
            if (!g || !g.placed || g.placed.length === 0) {
                return 'EMPTY';
            }
            var ps = [];
            for (var i = 0; i < g.placed.length; i++) {
                var p = g.placed[i], pl = g.placements[i];
                ps.push(p.partId + '@' + Math.round(pl.x * 1e4) + ',' + Math.round(pl.y * 1e4) + '@' + p.rot);
            }
            return ps.length + ':' + ps.join(';');
        }
        // Moved instances (the differing positions between the source and candidate sequences)
        function _movedInstances(srcPrefix, candPrefix) {
            var moved = [];
            var n = Math.max(srcPrefix.length, candPrefix.length);
            for (var i = 0; i < n; i++) {
                if (srcPrefix[i] !== candPrefix[i]) {
                    if (candPrefix[i] != null) {
                        moved.push(candPrefix[i]);
                    }
                }
            }
            return moved;
        }

        var startL = layout.geometry.L;
        var bestLayout = layout;
        var bestL = startL;
        var history = [];
        var start = Date.now();
        var iter = 0;
        var noImprove = 0;
        // Sole termination condition: 20000 consecutive full-rebuild attempts without improving the global best occupied length.
        // Attempts judged invalid/infeasible/equal-length/worse all count toward noImprove; only a successful shorter complete legal solution resets it.
        var noImproveLimit = (Number.isFinite(config.lnsNoImproveLimit) && config.lnsNoImproveLimit > 0)
            ? config.lnsNoImproveLimit : 20000;

        function replayFull(prefix) {
            var g = geomCtx.newState();
            for (var i = 0; i < prefix.length; i++) {
                var res = geomCtx.placeOne(g, prefix[i], angles);
                if (!res) return null;
                g = res.geometry;
            }
            return g;
        }

        function isValidLayout(geom) {
            if (!geom) return false;
            if (geom.placed.length !== N) return false;
            for (var i = 0; i < geom.placed.length; i++) {
                var pb = geom.placed[i]._bounds;
                var pl = geom.placements[i];
                if (pb.y + pl.y < -1e-6) return false;
                if (pb.y + pl.y + pb.height > binW + 1e-6) return false;
                if (pb.x + pl.x < -1e-6) return false;
            }
            return true;
        }

        // ============================================================
        // Operator 1: large-piece forward move and local blocking relief
        // ============================================================
        function opLargeForward(prefix) {
            var half = Math.floor(N / 2);
            var bestIdx = -1;
            var bestArea = 0;
            for (var i = half; i < N; i++) {
                var a = areaById[prefix[i]] || 0;
                if (a > bestArea) { bestArea = a; bestIdx = i; }
            }
            if (bestIdx < 0) return null;
            var targetIdx = Math.floor(rng() * half);
            if (targetIdx >= bestIdx) targetIdx = Math.max(0, bestIdx - 1);
            if (targetIdx >= bestIdx) return null;
            var newPrefix = prefix.slice(0);
            var moved = newPrefix.splice(bestIdx, 1)[0];
            newPrefix.splice(targetIdx, 0, moved);
            var g = replayFull(newPrefix);
            if (!g) return null;
            return { prefix: newPrefix, geometry: g };
        }

        // ============================================================
        // Operator 2: key tail-end compression
        // ============================================================
        function opTailCompress(prefix) {
            var tailCount = Math.max(2, Math.min(5, Math.floor(N * 0.1)));
            var newPrefix = prefix.slice(0, N - tailCount);
            var tailPieces = prefix.slice(N - tailCount);
            for (var i = tailPieces.length - 1; i > 0; i--) {
                var j = Math.floor(rng() * (i + 1));
                var tmp = tailPieces[i]; tailPieces[i] = tailPieces[j]; tailPieces[j] = tmp;
            }
            newPrefix = newPrefix.concat(tailPieces);
            var g = replayFull(newPrefix);
            if (!g) return null;
            return { prefix: newPrefix, geometry: g };
        }

        // ============================================================
        // Operator 3: large-gap region rebuild
        // ============================================================
        function opGapRebuild(prefix) {
            var segLen = Math.min(N, 3 + Math.floor(rng() * 6));
            var segStart = Math.floor(rng() * Math.max(1, N - segLen));
            var newPrefix = prefix.slice(0, segStart);
            var segment = prefix.slice(segStart, segStart + segLen);
            for (var i = segment.length - 1; i > 0; i--) {
                var j = Math.floor(rng() * (i + 1));
                var tmp = segment[i]; segment[i] = segment[j]; segment[j] = tmp;
            }
            newPrefix = newPrefix.concat(segment).concat(prefix.slice(segStart + segLen));
            var g = replayFull(newPrefix);
            if (!g) return null;
            return { prefix: newPrefix, geometry: g };
        }

        // ============================================================
        // Operator 4: large-scale order perturbation and global rebuild
        // ============================================================
        function opGlobalRebuild(prefix) {
            if (rng() < 0.5) {
                var shuffled = prefix.slice(0);
                for (var i = shuffled.length - 1; i > 0; i--) {
                    var j = Math.floor(rng() * (i + 1));
                    var tmp = shuffled[i]; shuffled[i] = shuffled[j]; shuffled[j] = tmp;
                }
                var g1 = replayFull(shuffled);
                if (g1) return { prefix: shuffled, geometry: g1 };
                return null;
            }
            var byArea = prefix.slice(0).sort(function (a, b) {
                return (areaById[b] || 0) - (areaById[a] || 0);
            });
            var g2 = replayFull(byArea);
            if (g2) return { prefix: byArea, geometry: g2 };
            return null;
        }

        var ops = {
            largeForward: opLargeForward,
            tailCompress: opTailCompress,
            gapRebuild: opGapRebuild,
            globalRebuild: opGlobalRebuild
        };

        // Main loop: the only default termination = noImprove >= noImproveLimit (20000 consecutive no-improvement full rebuilds).
        // The iteration hard cap / time exit are only enabled when the corresponding budget is a finite positive number (hasLnsIterLimit / hasLnsTimeLimit).
        var currentLayout = layout;
        while (noImprove < noImproveLimit && (!hasLnsIterLimit || iter < lnsMaxIters)) {
            if (hasLnsTimeLimit && (Date.now() - start) > lnsBudgetMs) break;
            iter++;

            var opName = lnsOperators[iter % lnsOperators.length];
            if (rng() < 0.3) {
                opName = lnsOperators[Math.floor(rng() * lnsOperators.length)];
            }
            var op = ops[opName];
            if (!op) continue;

            var proposal;
            try {
                proposal = op(currentLayout.prefix);
            } catch (e) {
                if (lnsDebug) console.log('[LNS] op ' + opName + ' error: ' + e.message);
                continue;
            }
            if (!proposal) { noImprove++; continue; }
            if (!isValidLayout(proposal.geometry)) { noImprove++; continue; }

            var newL = proposal.geometry.L;
            var improvedThisIter = (newL < bestL - 1e-9);
            history.push({ iter: iter, op: opName, L: newL, improved: improvedThisIter });
            // test1_5: LNS rebuild event (RNG-free). source = current layout, candidate = operator output.
            // same_geometry = true when the candidate pose is exactly identical to the source (sequence changed but geometry did not).
            if (diag) {
                lnsEventSeq++;
                var _srcFpL = _geomFp(currentLayout.geometry);
                var _candFpL = _geomFp(proposal.geometry);
                var _movedL = _movedInstances(currentLayout.prefix, proposal.prefix);
                var _movedAreasL = _movedL.map(function (x) { return areaById[x] || 0; });
                diag.appendLnsEvent({
                    run_id: diag.runId, stage: 'lns_rebuild', event_id: lnsEventSeq, iter: iter,
                    operator: opName,
                    source_layout_fingerprint: _srcFpL,
                    candidate_layout_fingerprint: _candFpL,
                    moved_instance_ids: _movedL, moved_net_areas: _movedAreasL,
                    old_positions: null, new_positions: null,
                    geometry_redecoded: true,
                    geometry_valid: true,
                    same_geometry: (_candFpL === _srcFpL),
                    length_before: currentLayout.geometry.L,
                    length_after: newL,
                    global_best_updated: improvedThisIter
                });
            }

            if (newL < bestL - 1e-9) {
                bestLayout = proposal;
                bestL = newL;
                noImprove = 0;
                if (lnsDebug) console.log('[LNS] iter ' + iter + ' op=' + opName + ' L=' + bestL.toFixed(4) + ' improved');
            } else {
                if (rng() < 0.1) { currentLayout = proposal; }
                noImprove++;
            }
        }

        if (lnsDebug) {
            console.log('[LNS] done: iter=' + iter + ' bestL=' + bestL.toFixed(4) + ' startL=' + startL.toFixed(4) +
                ' improved=' + (bestL < startL - 1e-9) + ' elapsed=' + ((Date.now() - start) / 1000).toFixed(2) + 's');
        }

        return {
            bestLayout: bestLayout,
            bestL: bestL,
            iterations: iter,
            improved: bestL < startL - 1e-9,
            history: history
        };
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = { runLns: runLns };
    } else {
        root.StbbsLns = { runLns: runLns };
    }
})(typeof window !== 'undefined' ? window : global);
