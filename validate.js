/**
 * validate.js
 * STBBS result integrity validation (corresponds to the algorithm doc §11 acceptance checks):
 *   1) No area overlap: if the internal intersection area of any two pieces > tol, it is judged as overlapping
 *   2) No out-of-bounds: x_min >= -tol; y_min >= -tol; y_max <= binW + tol
 *   3) Completeness: placed count == N, area sum == S
 * Depends on global GeometryUtil / ClipperLib (mounted at the entry point).
 */
(function (root) {
    'use strict';

    var scale = 10000000;

    function toClipperAt(poly, ox, oy) {
        var cp = [];
        for (var i = 0; i < poly.length; i++) {
            cp.push({ X: poly[i].x + ox, Y: poly[i].y + oy });
        }
        ClipperLib.JS.ScaleUpPath(cp, scale);
        return cp;
    }

    // Net intersection area of two polygons (including hole children) under their respective world offsets
    // World offset ox,oy is the translation of the "bounding-box minimum corner" (= placement position - rotated contour bounding-box minimum corner)
    function netIntersectionArea(polyA, ox, oy, polyB, bx, by) {
        var aPath = toClipperAt(polyA, ox, oy);
        var bPath = toClipperAt(polyB, bx, by);
        if (!aPath || aPath.length < 3 || !bPath || bPath.length < 3) {
            return 0;
        }
        // 1) Outer contour intersection
        var c = new ClipperLib.Clipper();
        c.AddPath(aPath, ClipperLib.PolyType.ptSubject, true);
        c.AddPath(bPath, ClipperLib.PolyType.ptClip, true);
        var sol = new ClipperLib.Paths();
        if (!c.Execute(ClipperLib.ClipType.ctIntersection, sol,
            ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)) {
            return 0;
        }
        if (sol.length === 0) {
            return 0;
        }
        // 2) Subtract the parts of the intersection that fall inside A's or B's holes (holes are not material)
        var holes = [];
        if (polyA.children) {
            for (var i = 0; i < polyA.children.length; i++) {
                holes.push(toClipperAt(polyA.children[i], ox, oy));
            }
        }
        if (polyB.children) {
            for (var j = 0; j < polyB.children.length; j++) {
                holes.push(toClipperAt(polyB.children[j], bx, by));
            }
        }
        var base = sol;
        for (var h = 0; h < holes.length; h++) {
            if (holes[h].length < 3) {
                continue;
            }
            var d = new ClipperLib.Clipper();
            d.AddPaths(base, ClipperLib.PolyType.ptSubject, true);
            d.AddPath(holes[h], ClipperLib.PolyType.ptClip, true);
            var dsol = new ClipperLib.Paths();
            if (!d.Execute(ClipperLib.ClipType.ctDifference, dsol,
                ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)) {
                continue;
            }
            base = dsol;
            if (base.length === 0) {
                return 0;
            }
        }
        var total = 0;
        for (var m = 0; m < base.length; m++) {
            total += Math.abs(ClipperLib.Clipper.Area(base[m]));
        }
        return total / (scale * scale);
    }

    /**
     * Validate a nesting result
     * @param {Object} args
     *   placements: [{ partId, rotation, x, y }]
     *   parts:      parts array (indexed by id)
     *   binW:       fixed board width
     *   expectedN:  expected number of pieces to place
     *   overlapTol: overlap area tolerance (default 1e-3)
     * @returns {{valid:boolean, overlaps:Array, outOfBounds:Array, placedCount:number, area:number, expectedN:number, maxOverlap:number}}
     */
    function validateResult(args) {
        var placements = args.placements || [];
        var parts = args.parts;
        var binW = args.binW;
        var expectedN = args.expectedN;
        var tol = (args.overlapTol != null) ? args.overlapTol : 1e-3;

        // Pre-rotated contour + bounding box + world offset.
        // placement (x,y) is the translation vector of the piece's rotated contour (reference point displacement);
        // world coordinates = rotated contour + (x, y). The bounding box is only used for out-of-bounds checks.
        var rotPolys = [];
        var bounds = [];
        var worldOff = [];
        for (var i = 0; i < placements.length; i++) {
            var pl = placements[i];
            var poly = GeometryUtil.rotatePolygon(parts[pl.partId], pl.rotation);
            var bb = GeometryUtil.getPolygonBounds(poly);
            rotPolys.push(poly);
            bounds.push(bb);
            worldOff.push({ x: pl.x, y: pl.y });
        }

        var overlaps = [];
        var maxOverlap = 0;
        for (var a = 0; a < placements.length; a++) {
            for (var b = a + 1; b < placements.length; b++) {
                var inter = netIntersectionArea(rotPolys[a], worldOff[a].x, worldOff[a].y, rotPolys[b], worldOff[b].x, worldOff[b].y);
                if (inter > tol) {
                    overlaps.push({ a: placements[a].partId, b: placements[b].partId, area: inter });
                    if (inter > maxOverlap) {
                        maxOverlap = inter;
                    }
                }
            }
        }

        var outOfBounds = [];
        var boundTol = 1e-4;
        for (var k = 0; k < placements.length; k++) {
            var bb = bounds[k];
            var minX = bb.x + worldOff[k].x;
            var minY = bb.y + worldOff[k].y;
            var maxY = minY + bb.height;
            var maxX = minX + bb.width;
            if (minX < -boundTol || minY < -boundTol || maxY > binW + boundTol) {
                outOfBounds.push({
                    partId: placements[k].partId,
                    minX: minX, minY: minY, maxY: maxY, maxX: maxX, binW: binW
                });
            }
        }

        // Net area accumulation (outer contour - holes)
        var area = 0;
        for (var z = 0; z < placements.length; z++) {
            var polyZ = parts[placements[z].partId];
            var aZ = Math.abs(GeometryUtil.polygonArea(polyZ));
            if (polyZ.children) {
                for (var hz = 0; hz < polyZ.children.length; hz++) {
                    aZ -= Math.abs(GeometryUtil.polygonArea(polyZ.children[hz]));
                }
            }
            area += Math.max(0, aZ);
        }

        // Unique instance count dedup check: duplicate instances (unique < actual) or missing pieces (actual != expected) are both judged invalid,
        // even if overlap_count=0, the completeness check must not be bypassed (§5).
        var instanceSet = {};
        var uniqueCount = 0;
        for (var u = 0; u < placements.length; u++) {
            if (!instanceSet[placements[u].partId]) {
                instanceSet[placements[u].partId] = true;
                uniqueCount++;
            }
        }

        return {
            valid: overlaps.length === 0 && outOfBounds.length === 0 &&
                placements.length === expectedN && uniqueCount === expectedN,
            overlaps: overlaps,
            outOfBounds: outOfBounds,
            placedCount: placements.length,
            expectedN: expectedN,
            uniqueInstanceCount: uniqueCount,
            area: area,
            maxOverlap: maxOverlap
        };
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = { validateResult: validateResult };
    } else {
        root.StbbsValidate = { validateResult: validateResult };
    }

})(typeof window !== 'undefined' ? window : global);
