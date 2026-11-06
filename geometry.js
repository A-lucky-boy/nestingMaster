/**
 * geometry.js
 * STBBS geometry placement layer (PlacementDecoder, corresponding to chapter 4 of the algorithm doc)
 *
 * Self-contained module: ports the tuned geometry placement logic from beamTabuNest.js, interfaced as
 *   placeOne(state, partId, angles, ctx) -> PlaceResult | null
 * Does not modify the passed-in state (copy-on-write, returns a new GeometryState).
 *
 * Coordinate system convention (consistent with the old algorithm/CLI):
 *   x axis = open advancement / occupied-length direction; y axis = fixed board width direction; legal region x>=0, 0<=y<=binW.
 *   state.L = the maximum x-coordinate of the rightmost end of the placed contour (occupied length); state.W = the envelope width of the placed contour.
 *
 * Depends on globals: ClipperLib, GeometryUtil (mounted onto global by the entry point).
 */
(function (root) {
    'use strict';

    // NFP cache key (kept consistent with the precomputation in nest.js / nfpWorker.js)
    function createNfpCacheKey(A, B, inside, Arotation, Brotation) {
        return A + '|' + B + '|' + (inside ? 1 : 0) + '|' + Arotation + '|' + Brotation;
    }

    /**
     * Create a geometry placement context.
     * @param {Object} opts
     *   parts:        parts array, indexed by id (parts[id] is a polygon, with .children holes)
     *   binW:         fixed board width W (y direction)
     *   nfpCache:     precomputed NFP cache { key: Clipper path array }
     *   config:       the tunable geometry-related parameters object (see stbbs-cli.js CONFIG)
     */
    function createGeometry(opts) {
        var parts = opts.parts;
        var binW = opts.binW;
        var nfpCache = opts.nfpCache || {};
        var config = opts.config || {};
        var scale = config.clipperScale || 10000000;

        // Rotated bounding-box cache: key = partId + '_' + rot
        var partBoundsCache = {};
        var evalCounter = { place: 0, candEval: 0 };

        function cfg(name, def) {
            if (Object.prototype.hasOwnProperty.call(config, name)) {
                return config[name];
            }
            return def;
        }

        // ============================================================
        // Basic geometry
        // ============================================================
        // Net area (outer contour - holes)
        function netArea(poly) {
            var a = Math.abs(GeometryUtil.polygonArea(poly));
            if (poly.children) {
                for (var i = 0; i < poly.children.length; i++) {
                    a -= Math.abs(GeometryUtil.polygonArea(poly.children[i]));
                }
            }
            return Math.max(a, 0);
        }

        // Rotated bounding box (cached)
        function rotatedBounds(partId, rot) {
            var key = partId + '_' + rot;
            if (partBoundsCache[key]) {
                return partBoundsCache[key];
            }
            var r = GeometryUtil.rotatePolygon(parts[partId], rot);
            var b = GeometryUtil.getPolygonBounds(r);
            var out = { x: b.x, y: b.y, width: b.width, height: b.height };
            partBoundsCache[key] = out;
            return out;
        }

        function toClipperAt(poly, ox, oy) {
            var cp = [];
            for (var i = 0; i < poly.length; i++) {
                cp.push({ X: poly[i].x + ox, Y: poly[i].y + oy });
            }
            ClipperLib.JS.ScaleUpPath(cp, scale);
            return cp;
        }

        function fromClipper(poly) {
            var out = [];
            for (var i = 0; i < poly.length; i++) {
                out.push({ x: poly[i].X / scale, y: poly[i].Y / scale });
            }
            return out;
        }

        // The true intersection area of two polygons under their respective translations (Clipper exact intersection)
        function clipIntersectionAt(rotPoly, ox, oy, otherPart, opx, opy, otherRot) {
            var other = GeometryUtil.rotatePolygon(otherPart, otherRot);
            var pa = toClipperAt(rotPoly, ox, oy);
            var pb = toClipperAt(other, opx, opy);
            if (!pa || pa.length < 3 || !pb || pb.length < 3) {
                return 0;
            }
            var clipper = new ClipperLib.Clipper();
            clipper.AddPath(pa, ClipperLib.PolyType.ptSubject, true);
            clipper.AddPath(pb, ClipperLib.PolyType.ptClip, true);
            var sol = new ClipperLib.Paths();
            if (!clipper.Execute(ClipperLib.ClipType.ctIntersection, sol,
                ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)) {
                return 0;
            }
            var total = 0;
            for (var i = 0; i < sol.length; i++) {
                total += Math.abs(ClipperLib.Clipper.Area(sol[i]));
            }
            return total / (scale * scale);
        }

        // Whether the candidate (at translation shift) truly area-overlaps any placed piece
        function overlapsPlaced(candRot, shift, state) {
            var cb = candRot._bounds;
            var candBox = { x: cb.x + shift.x, y: cb.y + shift.y, w: cb.width, h: cb.height };
            for (var p = 0; p < state.placed.length; p++) {
                var pl = state.placed[p];
                var pb = pl._bounds;
                var box = { x: pb.x + state.placements[p].x, y: pb.y + state.placements[p].y, w: pb.width, h: pb.height };
                if (candBox.x + candBox.w <= box.x || box.x + box.w <= candBox.x ||
                    candBox.y + candBox.h <= box.y || box.y + box.h <= candBox.y) {
                    continue; // bounding boxes do not intersect
                }
                var inter = clipIntersectionAt(candRot, shift.x, shift.y, parts[pl.partId], state.placements[p].x, state.placements[p].y, pl.rot);
                if (inter > 1e-4) {
                    return true;
                }
            }
            return false;
        }

        // ============================================================
        // Candidate projection slide ("down first")
        // ============================================================
        function pipCrossing(pts, px, py) {
            var inside = false;
            for (var i = 0, j = pts.length - 1; i < pts.length; j = i++) {
                var xi = pts[i].x, yi = pts[i].y;
                var xj = pts[j].x, yj = pts[j].y;
                if ((yi > py) !== (yj > py)) {
                    var xint = (xj - xi) * (py - yi) / (yj - yi) + xi;
                    if (px <= xint + 1e-12) {
                        inside = !inside;
                    }
                }
            }
            return inside;
        }

        function buildOmegaRings(finalNfp) {
            var rings = [];
            for (var i = 0; i < finalNfp.length; i++) {
                var pts = fromClipper(finalNfp[i]);
                if (pts.length < 3) {
                    continue;
                }
                var minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
                for (var k = 0; k < pts.length; k++) {
                    if (pts[k].x < minX) { minX = pts[k].x; }
                    if (pts[k].x > maxX) { maxX = pts[k].x; }
                    if (pts[k].y < minY) { minY = pts[k].y; }
                    if (pts[k].y > maxY) { maxY = pts[k].y; }
                }
                rings.push({ pts: pts, box: { x: minX, y: minY, w: maxX - minX, h: maxY - minY } });
            }
            return rings;
        }

        function pointInOmega(pt, omegaRings) {
            var count = 0;
            for (var i = 0; i < omegaRings.length; i++) {
                var rb = omegaRings[i].box;
                if (pt.x < rb.x || pt.x > rb.x + rb.w || pt.y < rb.y || pt.y > rb.y + rb.h) {
                    continue;
                }
                if (pipCrossing(omegaRings[i].pts, pt.x, pt.y)) {
                    count++;
                }
            }
            return (count & 1) === 1;
        }

        // Project along the -Y direction to the farthest feasible point (exponential stepping + interval bisection), board-bottom floor = floorY
        function slideDown(sh, omegaRings, floorY) {
            if (!pointInOmega(sh, omegaRings)) {
                return null;
            }
            var tol = cfg('slideTol', 0.005);
            var s = cfg('slideStep0', 1.0);
            var last = { x: sh.x, y: sh.y };
            var bad = null;
            var floor = (floorY != null) ? floorY : -Infinity;
            while (s >= tol) {
                var probeY = last.y - s;
                if (probeY < floor) {
                    probeY = floor;
                }
                var probe = { x: last.x, y: probeY };
                if (pointInOmega(probe, omegaRings)) {
                    last = probe;
                    if (probeY < floor + 1e-12) {
                        break;
                    }
                    s *= 2;
                } else {
                    bad = probe;
                    break;
                }
            }
            if (bad != null) {
                for (var it = 0; it < 16; it++) {
                    var midY = (last.y + bad.y) / 2;
                    if (midY < floor) {
                        midY = floor;
                    }
                    var mid = { x: last.x, y: midY };
                    if (pointInOmega(mid, omegaRings)) {
                        last = mid;
                    } else {
                        bad = mid;
                    }
                    if (last.y - bad.y < tol) {
                        break;
                    }
                }
            }
            if (last.y < floor) {
                last.y = floor;
            }
            return last;
        }

        // ============================================================
        // Feasible region Ω(pθ)=IFP∖∪int(NFP) (eq. 5)
        // Returns a Clipper path array (already scaled) or null
        // ============================================================
        function computeFinalNfp(state, partId, rot, binNfp) {
            var clipperUnion = new ClipperLib.Clipper();
            for (var j = 0; j < state.placed.length; j++) {
                var key = createNfpCacheKey(state.placed[j].partId, partId, false, state.placed[j].rot, rot);
                var nfp = nfpCache[key];
                if (!nfp) {
                    return null;
                }
                for (var k = 0; k < nfp.length; k++) {
                    var clone = toClipperAt(nfp[k], state.placements[j].x, state.placements[j].y);
                    var area = Math.abs(ClipperLib.Clipper.Area(clone));
                    if (clone.length > 2 && area > 0.1 * scale * scale) {
                        clipperUnion.AddPath(clone, ClipperLib.PolyType.ptSubject, true);
                    }
                }
            }
            var combined = new ClipperLib.Paths();
            if (!clipperUnion.Execute(ClipperLib.ClipType.ctUnion, combined,
                ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)) {
                return null;
            }
            var clipperDiff = new ClipperLib.Clipper();
            var clipperBinNfp = [];
            for (j = 0; j < binNfp.length; j++) {
                clipperBinNfp.push(toClipperAt(binNfp[j], 0, 0));
            }
            clipperDiff.AddPaths(combined, ClipperLib.PolyType.ptClip, true);
            clipperDiff.AddPaths(clipperBinNfp, ClipperLib.PolyType.ptSubject, true);
            var finalNfp = new ClipperLib.Paths();
            if (!clipperDiff.Execute(ClipperLib.ClipType.ctDifference, finalNfp,
                ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)) {
                return null;
            }
            var rawRings = finalNfp.length;
            finalNfp = ClipperLib.Clipper.CleanPolygons(finalNfp, 0.0001 * scale);
            var cleanedRings = finalNfp.length;
            var minArea = 0.1 * scale * scale;
            for (j = 0; j < finalNfp.length; j++) {
                if (finalNfp[j].length < 3 || Math.abs(ClipperLib.Clipper.Area(finalNfp[j])) < minArea) {
                    finalNfp.splice(j, 1);
                    j--;
                }
            }
            if (cfg('debug')) {
                console.log('    [computeFinalNfp] part ' + partId + ' rot ' + rot + ' placed ' + state.placed.length +
                    ': unionRings ' + combined.length + ' -> diffRaw ' + rawRings + ' -> cleaned ' + cleanedRings + ' -> final ' + finalNfp.length);
            }
            if (!finalNfp || finalNfp.length === 0) {
                return null;
            }
            return finalNfp;
        }

        // Compactness penalty (optional, γ·combined envelope gap of two pieces)
        function localCompactPenalty(state, ext, sh, L0, W0) {
            var gamma = cfg('localCompactWeight', 0.05);
            var K = Math.max(1, parseInt(cfg('localCompactNeighbors', 2), 10) || 2);
            var n = state.placed.length;
            if (n === 0) {
                return 0;
            }
            var cMinX = ext.x + sh.x, cMaxX = cMinX + ext.width;
            var cMinY = ext.y + sh.y, cMaxY = cMinY + ext.height;
            var cArea = ext.width * ext.height;
            var cx = (cMinX + cMaxX) / 2, cy = (cMinY + cMaxY) / 2;
            var list = new Array(n);
            for (var p = 0; p < n; p++) {
                var pb = state.placed[p]._bounds;
                var px = state.placements[p].x, py = state.placements[p].y;
                var pMinX = pb.x + px, pMaxX = pMinX + pb.width;
                var pMinY = pb.y + py, pMaxY = pMinY + pb.height;
                var pArea = pb.width * pb.height;
                var combMinX = cMinX < pMinX ? cMinX : pMinX;
                var combMaxX = cMaxX > pMaxX ? cMaxX : pMaxX;
                var combMinY = cMinY < pMinY ? cMinY : pMinY;
                var combMaxY = cMaxY > pMaxY ? cMaxY : pMaxY;
                var gap = (combMaxX - combMinX) * (combMaxY - combMinY) - cArea - pArea;
                if (gap < 0) {
                    gap = 0;
                }
                var pcx = (pMinX + pMaxX) / 2, pcy = (pMinY + pMaxY) / 2;
                var dx = cx - pcx, dy = cy - pcy;
                list[p] = { gap: gap, dist: dx * dx + dy * dy };
            }
            list.sort(function (a, b) { return a.dist - b.dist; });
            if (K > list.length) {
                K = list.length;
            }
            var sc = (L0 * W0) > 1e-12 ? (L0 * W0) : 1;
            var sum = 0;
            for (var q = 0; q < K; q++) {
                sum += list[q].gap / sc;
            }
            return gamma * sum;
        }

        // ============================================================
        // Geometry state (GeometryState, algorithm doc 3.2)
        // ============================================================
        function newState() {
            return { L: 0, W: 0, placed: [], placements: [] };
        }

        function cloneState(st) {
            var c = { L: st.L, W: st.W, placed: [], placements: [] };
            for (var i = 0; i < st.placed.length; i++) {
                c.placed.push(st.placed[i]); // elements are immutable
            }
            for (var j = 0; j < st.placements.length; j++) {
                c.placements.push({ x: st.placements[j].x, y: st.placements[j].y });
            }
            return c;
        }

        // Candidate reference points: feasible-region vertices + the centroid of each connected ring (old-gap backfill candidates).
        // §4.1: not only enumerate the new boundary vertices near the farthest end, but also the feasible backfill positions
        // inside the gaps within [0,L_old]. The centroid points into the gap interior, and is the key candidate for old-gap backfill.
        // After sorting vertices+centroids by bottom-left, truncate by budget; centroids are preferentially retained (marked _isGap) to prevent truncation from dropping old gaps.
        function collectCandidates(finalNfp, r, budget) {
            var seen = {};
            var cands = [];
            var refX = r[0].x;
            var refY = r[0].y;
            function add(x, y, isGap) {
                var key = Math.round(x * 10) + '_' + Math.round(y * 10);
                if (seen[key]) {
                    return;
                }
                seen[key] = true;
                cands.push({ x: x, y: y, _isGap: !!isGap });
            }
            for (var j = 0; j < finalNfp.length; j++) {
                var nf = fromClipper(finalNfp[j]);
                if (nf.length < 3) {
                    continue;
                }
                // ① boundary vertices (contact candidates)
                for (var k = 0; k < nf.length; k++) {
                    add(nf[k].x - refX, nf[k].y - refY, false);
                }
                // ② connected-ring centroid (old-gap backfill candidate) —— compute the polygon centroid with the shoelace formula
                var a2 = 0, cx = 0, cy = 0;
                for (var q = 0; q < nf.length; q++) {
                    var qn = nf[(q + 1) % nf.length];
                    var cross = nf[q].x * qn.y - qn.x * nf[q].y;
                    a2 += cross;
                    cx += (nf[q].x + qn.x) * cross;
                    cy += (nf[q].y + qn.y) * cross;
                }
                if (Math.abs(a2) > 1e-9) {
                    add(cx / (3 * a2) - refX, cy / (3 * a2) - refY, true);
                }
            }
            // Bottom-left sort (y ascending, x ascending); on budget truncation retain all centroid candidates
            cands.sort(function (a, b) {
                if (a._isGap !== b._isGap) {
                    return a._isGap ? -1 : 1; // centroid first
                }
                if (Math.abs(a.y - b.y) > 1e-9) {
                    return a.y - b.y;
                }
                return a.x - b.x;
            });
            if (cands.length > budget) {
                var gaps = [];
                var verts = [];
                for (var ci = 0; ci < cands.length; ci++) {
                    if (cands[ci]._isGap) {
                        gaps.push(cands[ci]);
                    } else {
                        verts.push(cands[ci]);
                    }
                }
                var keep = Math.min(verts.length, Math.max(0, budget - gaps.length));
                cands = gaps.concat(verts.slice(0, keep));
            }
            return cands;
        }

        function bestBinOnly(binNfp, r) {
            var pos = null;
            for (var j = 0; j < binNfp.length; j++) {
                for (var k = 0; k < binNfp[j].length; k++) {
                    var p = { x: binNfp[j][k].x - r[0].x, y: binNfp[j][k].y - r[0].y };
                    if (!pos || p.x < pos.x - 1e-9 || (Math.abs(p.x - pos.x) <= 1e-9 && p.y < pos.y - 1e-9)) {
                        pos = p;
                    }
                }
            }
            return pos;
        }

        function applyPlace(state, r, pos, partId, rot) {
            var ext = r._bounds;
            var maxX = ext.x + pos.x + ext.width;
            var maxY = ext.y + pos.y + ext.height;
            state.L = Math.max(state.L, maxX);
            state.W = Math.max(state.W, maxY);
            state.placed.push({ partId: partId, rot: rot, _bounds: ext });
            state.placements.push({ x: pos.x, y: pos.y });
        }

        // ============================================================
        // placeOne: place partId on state (without modifying it), return an independent new geometry state
        // Special rule for the first piece (per the paper): BL positioning at each angle, first pick the minimum L1, then the minimum L1*W1.
        // Non-first pieces: pick the minimum J = α·Δ_A + β·Δ_L across (angle, candidate).
        // ============================================================
        function placeOne(state, partId, angles) {
            var alpha = cfg('alpha', 0.4);
            var beta = cfg('beta', 0.6);
            var candBudget = cfg('candidateBudget', 40);
            var localCompactOn = cfg('localCompactEnabled', false);

            // ---- Empty layout: first-piece BL rule (minimum L1, then minimum L1*W1, then minimum y) ----
            if (state.placed.length === 0) {
                var best0 = null;
                var best0L1 = Infinity;
                var best0Area = Infinity;
                var best0Y = Infinity;
                for (var a0 = 0; a0 < angles.length; a0++) {
                    var rot0 = angles[a0];
                    var r0 = GeometryUtil.rotatePolygon(parts[partId], rot0);
                    var rb0 = GeometryUtil.getPolygonBounds(r0);
                    r0._bounds = rb0;
                    if (rb0.height > binW + 1e-6) {
                        continue;
                    }
                    var binKey0 = createNfpCacheKey(-1, partId, true, 0, rot0);
                    var binNfp0 = nfpCache[binKey0];
                    if (!binNfp0 || binNfp0.length === 0) {
                        continue;
                    }
                    var pos0 = bestBinOnly(binNfp0, r0);
                    if (!pos0) {
                        continue;
                    }
                    if (rb0.y + pos0.y + rb0.height > binW + 1e-6) {
                        continue;
                    }
                    var l1 = rb0.x + pos0.x + rb0.width; // occupied length L1
                    var w1 = rb0.y + pos0.y + rb0.height; // envelope width W1
                    // First-piece rule: first minimum L1, then minimum L1*W1, then minimum y
                    if (l1 < best0L1 - 1e-9 ||
                        (Math.abs(l1 - best0L1) <= 1e-9 && l1 * w1 < best0Area - 1e-9) ||
                        (Math.abs(l1 - best0L1) <= 1e-9 && Math.abs(l1 * w1 - best0Area) <= 1e-9 && pos0.y < best0Y - 1e-9)) {
                        best0L1 = l1;
                        best0Area = l1 * w1;
                        best0Y = pos0.y;
                        best0 = { x: pos0.x, y: pos0.y, rot: rot0, r: r0 };
                    }
                }
                if (!best0) {
                    return null;
                }
                best0.id = partId;
                best0.rotation = best0.rot;
                evalCounter.place++;
                var g0 = newState();
                applyPlace(g0, best0.r, best0, partId, best0.rot);
                // test1_5: attach the length increment (first piece has input L=0). Pure incremental fields, do not affect sorting/RNG.
                return { geometry: g0, placement: best0, J: null, dL: null, dA: null, L1: g0.L, W1: g0.W,
                    L_before: state.L, delta_L: g0.L - state.L };
            }

            // ---- Non-empty: find the minimum J across (angle, candidate), then randomly pick one from top-diversityK by seed ----
            var candEvalCap = cfg('candidateEvalCap', 2000);
            var doSlide = cfg('slideProjection', true);
            var slideMaxCands = Math.max(1, parseInt(cfg('slideMaxCands', 8), 10) || 8);
            var divK = Math.max(1, parseInt(cfg('diversityK', 1), 10) || 1);
            var L0 = state.L > 0 ? state.L : 1e-9;
            var W0 = state.W > 0 ? state.W : 1e-9;
            var topCands = [];
            var dbg = cfg('debug');
            var dbgInfo = { perAngle: [], totalCand: 0, outBound: 0, overlap: 0, yNeg: 0, xNeg: 0, inserted: 0, topCands: 0 };

            // §4.2 lexicographic position selection key K=(δL, J, y, x, θ), smaller is better.
            // δL = L1 - L0 (occupied-length increment) is the primary criterion: prefer positions that do not additionally increase the occupied length, or that have the smallest increment;
            // for the same increment, compare by the original J (eq. 8 compactness proxy); then use y, x, θ as deterministic tie-breakers.
            function keyCmp(a, b) {
                var daL = a.L1 - b.L1;
                if (Math.abs(daL) > 1e-9) { return daL; } // δL first
                if (Math.abs(a.J - b.J) > 1e-12) { return a.J - b.J; }
                if (Math.abs(a.y - b.y) > 1e-9) { return a.y - b.y; }
                if (Math.abs(a.x - b.x) > 1e-9) { return a.x - b.x; }
                return a.rot - b.rot;
            }
            function insertTop(cand) {
                if (topCands.length < divK) {
                    var ins = false;
                    for (var i = 0; i < topCands.length; i++) {
                        if (keyCmp(cand, topCands[i]) < 0) {
                            topCands.splice(i, 0, cand);
                            ins = true;
                            break;
                        }
                    }
                    if (!ins) {
                        topCands.push(cand);
                    }
                } else if (keyCmp(cand, topCands[divK - 1]) < 0) {
                    topCands[divK - 1] = cand;
                    topCands.sort(keyCmp);
                }
            }

            var candEval = 0; // candidate count for a single placement evaluation (to prevent explosion); reset on each placeOne, not accumulated across calls
            for (var ai = 0; ai < angles.length; ai++) {
                var slideUsed = 0;
                var rot = angles[ai];
                var r = GeometryUtil.rotatePolygon(parts[partId], rot);
                var rb = GeometryUtil.getPolygonBounds(r);
                r._bounds = rb;
                if (rb.height > binW + 1e-6) {
                    if (dbg) dbgInfo.perAngle.push({ rot: rot, skip: 'height>' + binW + ' (h=' + rb.height.toFixed(2) + ')' });
                    continue;
                }
                var binKey = createNfpCacheKey(-1, partId, true, 0, rot);
                var binNfp = nfpCache[binKey];
                if (!binNfp || binNfp.length === 0) {
                    if (dbg) dbgInfo.perAngle.push({ rot: rot, skip: 'binNfp-missing' });
                    continue;
                }
                var finalNfp = computeFinalNfp(state, partId, rot, binNfp);
                if (!finalNfp) {
                    if (dbg) dbgInfo.perAngle.push({ rot: rot, skip: 'finalNfp-null' });
                    continue;
                }
                var cands = collectCandidates(finalNfp, r, candBudget);
                var omegaRings = doSlide ? buildOmegaRings(finalNfp) : null;
                if (dbg) {
                    dbgInfo.perAngle.push({ rot: rot, candCount: cands.length, finalNfpRings: finalNfp.length });
                    dbgInfo.totalCand += cands.length;
                }
                for (var c = 0; c < cands.length; c++) {
                    if (candEval >= candEvalCap) {
                        break;
                    }
                    var sh = cands[c];
                    if (overlapsPlaced(r, sh, state)) {
                        if (dbg) dbgInfo.overlap++;
                        continue;
                    }
                    var ext = r._bounds;
                    var candMaxY = ext.y + sh.y + ext.height;
                    if (candMaxY > binW + 1e-6) {
                        if (dbg) dbgInfo.outBound++;
                        continue;
                    }
                    if (ext.y + sh.y < -1e-6) {
                        if (dbg) dbgInfo.yNeg++;
                        continue;
                    }
                    if (ext.x + sh.x < -1e-6) {
                        if (dbg) dbgInfo.xNeg++;
                        continue;
                    }
                    var candMaxX = ext.x + sh.x + ext.width;
                    var L1 = Math.max(L0, candMaxX);
                    var W1 = Math.max(W0, candMaxY);
                    var dL = (L1 - L0) / L0;
                    var dA = (L1 * W1 - L0 * W0) / (L0 * W0);
                    var J = alpha * dA + beta * dL;
                    if (localCompactOn) {
                        J += localCompactPenalty(state, ext, sh, L0, W0);
                    }
                    candEval++;
                    insertTop({ J: J, L1: L1, y: sh.y, x: sh.x, rot: rot, r: r });
                    if (omegaRings && slideUsed < slideMaxCands) {
                        slideUsed++;
                        candEval++;
                        var slid = slideDown(sh, omegaRings, -ext.y);
                        if (slid && (Math.abs(slid.y - sh.y) > 1e-9)) {
                            if (overlapsPlaced(r, slid, state)) {
                                continue;
                            }
                            if (ext.x + slid.x < -1e-6 || ext.y + slid.y + ext.height > binW + 1e-6 || ext.y + slid.y < -1e-6) {
                                continue;
                            }
                            var sMaxX = ext.x + slid.x + ext.width;
                            var sL1 = Math.max(L0, sMaxX);
                            var sW1 = Math.max(W0, ext.y + slid.y + ext.height);
                            var sJ = alpha * ((sL1 * sW1 - L0 * W0) / (L0 * W0)) + beta * ((sL1 - L0) / L0);
                            if (localCompactOn) {
                                sJ += localCompactPenalty(state, ext, slid, L0, W0);
                            }
                            // §4.2 lexicographic: after sliding, if (δL,J,y,x) is better (can reduce the occupied-length increment), include it as a candidate
                            if (keyCmp({ J: sJ, L1: sL1, y: slid.y, x: slid.x, rot: rot }, { J: J, L1: L1, y: sh.y, x: sh.x, rot: rot }) < 0) {
                                insertTop({ J: sJ, L1: sL1, y: slid.y, x: slid.x, rot: rot, r: r });
                            }
                        }
                    }
                }
            }

            if (dbg) {
                dbgInfo.topCands = topCands.length;
            }
            if (topCands.length === 0) {
                if (dbg) {
                    console.log('    [placeOne NULL] part ' + partId + ' L0=' + L0.toFixed(2) + ' W0=' + W0.toFixed(2) +
                        ' totalCand=' + dbgInfo.totalCand + ' outBound=' + dbgInfo.outBound + ' overlap=' + dbgInfo.overlap +
                        ' yNeg=' + dbgInfo.yNeg + ' xNeg=' + dbgInfo.xNeg +
                        ' perAngle=' + JSON.stringify(dbgInfo.perAngle));
                }
                return null;
            }
            // The paper's placement layer is deterministic: take the minimum J across (angle, candidate) (eq. 8).
            // Randomness belongs only to the Beam weighted sampling and Tabu neighborhood sampling (§9.1), not introduced at the placement layer.
            var best = topCands[0];
            best.id = partId;
            best.rotation = best.rot;
            evalCounter.place++;
            var g = cloneState(state);
            applyPlace(g, best.r, best, partId, best.rot);
            // test1_5: attach the length increment. L_before = input state.L, delta_L = new L - L_before.
            // Pure incremental fields, do not affect topCands sorting or RNG calls.
            return { geometry: g, placement: best, J: best.J, dL: best.L1, dA: null, L1: g.L, W1: g.W,
                L_before: state.L, delta_L: g.L - state.L };
        }

        // ============================================================
        // placeOneTopK: return up to K spatially distinct legal poses (test1_3 §1)
        // Used for multi-pose branch expansion of key pieces (large/hard-to-place pieces).
        // Dedup rule: same rotation angle + position distance < dedupTol is treated as the same pose.
        // ============================================================
        function placeOneTopK(state, partId, angles, K, dedupTol) {
            K = Math.max(1, K | 0);
            dedupTol = (dedupTol != null) ? dedupTol : 2.0;
            var alpha = cfg('alpha', 0.4);
            var beta = cfg('beta', 0.6);
            var candBudget = cfg('candidateBudget', 40);
            var localCompactOn = cfg('localCompactEnabled', false);

            // First piece: same as placeOne (BL rule)
            if (state.placed.length === 0) {
                var r0 = placeOne(state, partId, angles);
                return r0 ? [r0] : [];
            }

            var candEvalCap = cfg('candidateEvalCap', 2000);
            var doSlide = cfg('slideProjection', true);
            var slideMaxCands = Math.max(1, parseInt(cfg('slideMaxCands', 8), 10) || 8);
            var L0 = state.L > 0 ? state.L : 1e-9;
            var W0 = state.W > 0 ? state.W : 1e-9;
            var allCands = []; // collect all legal candidates (not truncated to top-diversityK)

            function keyCmp(a, b) {
                var daL = a.L1 - b.L1;
                if (Math.abs(daL) > 1e-9) { return daL; }
                if (Math.abs(a.J - b.J) > 1e-12) { return a.J - b.J; }
                if (Math.abs(a.y - b.y) > 1e-9) { return a.y - b.y; }
                if (Math.abs(a.x - b.x) > 1e-9) { return a.x - b.x; }
                return a.rot - b.rot;
            }

            var candEval = 0;
            for (var ai = 0; ai < angles.length; ai++) {
                var slideUsed = 0;
                var rot = angles[ai];
                var r = GeometryUtil.rotatePolygon(parts[partId], rot);
                var rb = GeometryUtil.getPolygonBounds(r);
                r._bounds = rb;
                if (rb.height > binW + 1e-6) { continue; }
                var binKey = createNfpCacheKey(-1, partId, true, 0, rot);
                var binNfp = nfpCache[binKey];
                if (!binNfp || binNfp.length === 0) { continue; }
                var finalNfp = computeFinalNfp(state, partId, rot, binNfp);
                if (!finalNfp) { continue; }
                var cands = collectCandidates(finalNfp, r, candBudget);
                var omegaRings = doSlide ? buildOmegaRings(finalNfp) : null;
                for (var c = 0; c < cands.length; c++) {
                    if (candEval >= candEvalCap) { break; }
                    var sh = cands[c];
                    if (overlapsPlaced(r, sh, state)) { continue; }
                    var ext = r._bounds;
                    var candMaxY = ext.y + sh.y + ext.height;
                    if (candMaxY > binW + 1e-6) { continue; }
                    if (ext.y + sh.y < -1e-6) { continue; }
                    if (ext.x + sh.x < -1e-6) { continue; }
                    var candMaxX = ext.x + sh.x + ext.width;
                    var L1 = Math.max(L0, candMaxX);
                    var W1 = Math.max(W0, candMaxY);
                    var dL = (L1 - L0) / L0;
                    var dA = (L1 * W1 - L0 * W0) / (L0 * W0);
                    var J = alpha * dA + beta * dL;
                    if (localCompactOn) { J += localCompactPenalty(state, ext, sh, L0, W0); }
                    candEval++;
                    allCands.push({ J: J, L1: L1, y: sh.y, x: sh.x, rot: rot, r: r });
                    // Slide-down candidate
                    if (omegaRings && slideUsed < slideMaxCands) {
                        slideUsed++;
                        candEval++;
                        var slid = slideDown(sh, omegaRings, -ext.y);
                        if (slid && (Math.abs(slid.y - sh.y) > 1e-9)) {
                            if (overlapsPlaced(r, slid, state)) { continue; }
                            if (ext.x + slid.x < -1e-6 || ext.y + slid.y + ext.height > binW + 1e-6 || ext.y + slid.y < -1e-6) { continue; }
                            var sMaxX = ext.x + slid.x + ext.width;
                            var sL1 = Math.max(L0, sMaxX);
                            var sW1 = Math.max(W0, ext.y + slid.y + ext.height);
                            var sJ = alpha * ((sL1 * sW1 - L0 * W0) / (L0 * W0)) + beta * ((sL1 - L0) / L0);
                            if (localCompactOn) { sJ += localCompactPenalty(state, ext, slid, L0, W0); }
                            if (keyCmp({ J: sJ, L1: sL1, y: slid.y, x: slid.x, rot: rot }, { J: J, L1: L1, y: sh.y, x: sh.x, rot: rot }) < 0) {
                                allCands.push({ J: sJ, L1: sL1, y: slid.y, x: slid.x, rot: rot, r: r });
                            }
                        }
                    }
                }
            }

            if (allCands.length === 0) { return []; }
            allCands.sort(keyCmp);

            // Spatial dedup: same rotation + position distance < dedupTol is treated as the same pose
            var unique = [];
            for (var ui = 0; ui < allCands.length && unique.length < K; ui++) {
                var cand = allCands[ui];
                var isDup = false;
                for (var uj = 0; uj < unique.length; uj++) {
                    if (unique[uj].rot === cand.rot) {
                        var dx = unique[uj].x - cand.x, dy = unique[uj].y - cand.y;
                        if (Math.sqrt(dx * dx + dy * dy) < dedupTol) { isDup = true; break; }
                    }
                }
                if (!isDup) {
                    cand.id = partId;
                    cand.rotation = cand.rot;
                    evalCounter.place++;
                    var gg = cloneState(state);
                    applyPlace(gg, cand.r, cand, partId, cand.rot);
                    // test1_5: attach the length increment for each pose (L_before = input state.L)
                    unique.push({ geometry: gg, placement: cand, J: cand.J, dL: cand.L1, dA: null, L1: gg.L, W1: gg.W,
                        L_before: state.L, delta_L: gg.L - state.L });
                }
            }
            return unique;
        }

        // Checkpoint storage: nodeId -> GeometryState snapshot (a depth-d node stores the real geometry after d pieces are placed)
        var checkpoints = {};

        return {
            placeOne: placeOne,
            placeOneTopK: placeOneTopK,
            newState: newState,
            cloneState: cloneState,
            netArea: netArea,
            rotatedBounds: rotatedBounds,
            createNfpCacheKey: createNfpCacheKey,
            clipIntersectionAt: clipIntersectionAt,
            overlapsPlaced: overlapsPlaced,
            evalCounter: evalCounter,
            checkpoints: checkpoints,
            parts: parts,
            binW: binW,
            scale: scale
        };
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = { createGeometry: createGeometry, createNfpCacheKey: createNfpCacheKey };
    } else {
        root.StbbsGeometry = { createGeometry: createGeometry, createNfpCacheKey: createNfpCacheKey };
    }

})(typeof window !== 'undefined' ? window : global);
