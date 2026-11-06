/**
 * diagnostics.js
 * test1_5: STBBS run diagnostics module (observability and defect localization)
 *
 * Responsibilities (corresponding to sections 2~3, 5 of the STBBS_test1_5 spec):
 *   - unified log event format, each worker writes independently to its own run directory
 *   - structured JSONL / CSV buffered batch writes
 *   - field validation (missing fields output null with a recorded reason, no inference)
 *
 * Strict constraints (section 6 / verification checklist of the test1_5 spec):
 *   - diagnostics are pure bypass: no method in this module modifies candidate scores, participates in sorting, or changes geometric legality
 *   - event collection functions must not call the solving RNG (this module references no random source at all)
 *   - when enabled=false all methods are no-ops, guaranteeing the solving trajectory is exactly identical when the switch is off
 *   - only writes to its own run directory; aggregation is performed by the main thread after all jobs finish (areaOrderReport.js)
 *
 * Directory structure (section 2 of the spec):
 *   <diagnosticDir>/<instance>/run_<runId>/
 *     metadata.json
 *     placement_trace.csv
 *     beam_candidates.jsonl
 *     beam_layers.csv
 *     tabu_events.jsonl
 *     lns_events.jsonl
 *     geometry_validation.json
 *     source_layout.json
 *     final_layout.json
 *     result.svg
 */
(function (root) {
    'use strict';

    var fs = null;
    var path = null;
    try {
        fs = require('fs');
        path = require('path');
    } catch (e) {
        // Browser environment: the diagnostics module degrades to no-op
    }

    // ---- Deterministic geometry fingerprint (does not depend on ClipperLib, pure JS) ----
    // Represents a geometry layout by the normalized string of placed pieces (partId, x, y, rot).
    // Used to distinguish "sequence fingerprint change" from "geometry fingerprint change" (LNS distinguishability).
    // Coordinates are rounded to 1e-4 to avoid tiny floating-point differences causing the same geometry to be judged different.
    function geometryFingerprint(geometry) {
        if (!geometry || !geometry.placed || geometry.placed.length === 0) {
            return 'EMPTY';
        }
        var parts = [];
        for (var i = 0; i < geometry.placed.length; i++) {
            var p = geometry.placed[i];
            var pl = geometry.placements[i];
            parts.push(
                p.partId + '@' + Math.round(pl.x * 1e4) + ',' +
                Math.round(pl.y * 1e4) + '@' + p.rot
            );
        }
        return parts.length + ':' + parts.join(';');
    }

    // ---- Sequence fingerprint: by instance order only (no positions) ----
    function sequenceFingerprint(prefix) {
        if (!prefix || prefix.length === 0) {
            return 'EMPTY';
        }
        return prefix.length + ':' + prefix.join(',');
    }

    // ---- CSV cell escaping ----
    function csvEscape(v) {
        if (v == null) {
            return '';
        }
        var s = String(v);
        if (s.indexOf(',') >= 0 || s.indexOf('"') >= 0 || s.indexOf('\n') >= 0) {
            return '"' + s.replace(/"/g, '""') + '"';
        }
        return s;
    }

    /**
     * Create a diagnostic context.
     * @param {Object} opts
     *   enabled:                bool; false = all no-op
     *   level:                  'summary' | 'full' (full saves all candidate stage transitions)
     *   runId:                  run identifier
     *   runDir:                 absolute directory of this run (created if it does not exist)
     *   instanceName:           instance name
     *   instanceSha256:         SHA256 of the input shape+pieces
     *   codeCommit:             code version identifier (content fingerprint when no git)
     *   codeDirty:              whether there are uncommitted changes
     *   seed:                   random seed
     *   partCount:              N
     *   typeCount:              M
     *   stripWidth:             fixed board width W
     *   totalMaterialArea:      sum of all instance net areas S
     *   config:                 the full parameters object relevant to the result
     *   nodeVersion / os / cpuModel: machine info
     *   workerThreadsRequested / workerThreadsActual: thread info
     *   nfpPrecomputeScope:     NFP precompute scope description
     *   startedAt:              ISO start time
     *   areaById:               instId -> net area (used for area_rank computation, optional)
     */
    function createDiagnostics(opts) {
        opts = opts || {};
        var enabled = !!opts.enabled;
        var level = opts.level || 'summary';

        var ctx = {
            enabled: enabled,
            level: level,
            runId: opts.runId || '',
            runDir: opts.runDir || '',
            _meta: {},
            _placementRows: [],
            _beamCandLines: [],
            _beamLayerRows: [],
            _tabuLines: [],
            _lnsLines: [],
            _geoValidation: null,
            _layoutFiles: {},
            _closed: false,
            _missingFields: [] // missing-field registry (no inference)
        };

        // Area ranking table (by net area descending, ties stable by instance id ascending), used for area_rank / percentile
        var areaRankById = {};
        if (enabled && opts.areaById) {
            var ids = Object.keys(opts.areaById);
            ids.sort(function (a, b) {
                var da = opts.areaById[a], db = opts.areaById[b];
                if (db !== da) {
                    return db - da;
                }
                return (a < b) ? -1 : ((a > b) ? 1 : 0);
            });
            for (var ri = 0; ri < ids.length; ri++) {
                areaRankById[ids[ri]] = ri + 1; // 1-based
            }
        }
        ctx._areaRankById = areaRankById;

        // Create the run directory
        if (enabled && ctx.runDir && fs && fs.mkdirSync) {
            try {
                fs.mkdirSync(ctx.runDir, { recursive: true });
            } catch (e) {
                ctx._missingFields.push('runDir-create-failed: ' + e.message);
            }
        }

        function fileAt(name) {
            return ctx.runDir ? path.join(ctx.runDir, name) : name;
        }

        function recordMissing(field, reason) {
            if (ctx._missingFields.length < 200) {
                ctx._missingFields.push(field + ': ' + reason);
            }
        }

        // ---- metadata.json ----
        function setMeta(metaObj) {
            if (!ctx.enabled) {
                return;
            }
            for (var k in metaObj) {
                if (Object.prototype.hasOwnProperty.call(metaObj, k)) {
                    ctx._meta[k] = metaObj[k];
                }
            }
        }

        function writeMetadata() {
            if (!ctx.enabled) {
                return;
            }
            // Required field validation: if missing, output null and register it (no inference)
            var required = [
                'run_id', 'instance_name', 'instance_sha256', 'algorithm_name',
                'code_commit', 'code_dirty', 'seed', 'part_count', 'type_count',
                'strip_width', 'total_material_area', 'config', 'node_version', 'os',
                'cpu_model', 'worker_threads_requested', 'worker_threads_actual',
                'nfp_precompute_scope', 'started_at', 'completed_at'
            ];
            for (var i = 0; i < required.length; i++) {
                var f = required[i];
                if (ctx._meta[f] === undefined) {
                    ctx._meta[f] = null;
                    recordMissing('metadata.' + f, 'not-provided');
                }
            }
            try {
                fs.writeFileSync(fileAt('metadata.json'), JSON.stringify(ctx._meta, null, 2), 'utf8');
            } catch (e) {
                recordMissing('metadata.json-write', e.message);
            }
        }

        // ---- placement_trace.csv ----
        var PLACEMENT_HEADER = [
            'run_id', 'stage', 'instance_id', 'type_id', 'net_area', 'area_rank',
            'area_percentile', 'placement_index', 'theta', 'x', 'y',
            'bbox_min_x', 'bbox_max_x', 'bbox_min_y', 'bbox_max_y',
            'L_before', 'L_after', 'delta_L', 'old_length_feasible', 'old_length_selected',
            'part_fingerprint', 'geometry_fingerprint'
        ].join(',');

        function appendPlacementTrace(row) {
            if (!ctx.enabled) {
                return;
            }
            // Missing-field registry (no inference)
            var req = ['run_id', 'stage', 'instance_id', 'type_id', 'net_area',
                'area_rank', 'placement_index', 'theta', 'x', 'y',
                'L_before', 'L_after', 'delta_L', 'geometry_fingerprint'];
            for (var i = 0; i < req.length; i++) {
                if (row[req[i]] === undefined || row[req[i]] === null) {
                    recordMissing('placement_trace.' + req[i], 'null-at-index-' + row.placement_index);
                }
            }
            // area_percentile derived: area_rank / N (the average rank for equal areas is guaranteed by the caller via area_rank)
            if (row.area_percentile == null && row.area_rank != null) {
                var n = ctx._meta.part_count || 0;
                row.area_percentile = (n > 0) ? (row.area_rank / n) : null;
            }
            ctx._placementRows.push(row);
        }

        // ---- beam_candidates.jsonl ----
        function appendBeamCandidate(obj) {
            if (!ctx.enabled) {
                return;
            }
            // When level=summary, do not write candidate details (to prevent a huge amount of JSONL); only full writes
            if (ctx.level !== 'full') {
                return;
            }
            if (!obj.run_id) {
                obj.run_id = ctx.runId;
            }
            // Required field validation
            var req = ['depth', 'candidate_id', 'parent_id', 'instance_id', 'type_id',
                'net_area', 'final_disposition'];
            for (var i = 0; i < req.length; i++) {
                if (obj[req[i]] === undefined) {
                    obj[req[i]] = null;
                    recordMissing('beam_candidates.' + req[i], 'null-depth-' + obj.depth);
                }
            }
            ctx._beamCandLines.push(JSON.stringify(obj));
        }

        // ---- beam_layers.csv ----
        var BEAM_LAYER_HEADER = [
            'run_id', 'depth', 'parent_node_count', 'expand_attempt_count',
            'legal_node_count', 'candidate_pool_count', 'retained_count',
            'top_quartile_candidate_count', 'top_quartile_retained_count',
            'trigger_state', 'actual_ancestor_count', 'beam_insufficient',
            'cumulative_placed_area'
        ].join(',');

        function appendBeamLayer(row) {
            if (!ctx.enabled) {
                return;
            }
            if (!row.run_id) {
                row.run_id = ctx.runId;
            }
            var req = ['depth', 'parent_node_count', 'expand_attempt_count',
                'legal_node_count', 'candidate_pool_count', 'retained_count',
                'trigger_state', 'cumulative_placed_area'];
            for (var i = 0; i < req.length; i++) {
                if (row[req[i]] === undefined) {
                    row[req[i]] = null;
                    recordMissing('beam_layers.' + req[i], 'depth-' + row.depth);
                }
            }
            ctx._beamLayerRows.push(row);
        }

        // ---- tabu_events.jsonl ----
        function appendTabuEvent(obj) {
            if (!ctx.enabled) {
                return;
            }
            if (!obj.run_id) {
                obj.run_id = ctx.runId;
            }
            var req = ['event_id', 'operator', 'source_layout_fingerprint',
                'candidate_layout_fingerprint', 'geometry_redecoded', 'geometry_valid',
                'same_geometry', 'length_before', 'length_after'];
            for (var i = 0; i < req.length; i++) {
                if (obj[req[i]] === undefined) {
                    obj[req[i]] = null;
                    recordMissing('tabu_events.' + req[i], 'event-' + obj.event_id);
                }
            }
            ctx._tabuLines.push(JSON.stringify(obj));
        }

        // ---- lns_events.jsonl ----
        function appendLnsEvent(obj) {
            if (!ctx.enabled) {
                return;
            }
            if (!obj.run_id) {
                obj.run_id = ctx.runId;
            }
            var req = ['event_id', 'operator', 'source_layout_fingerprint',
                'candidate_layout_fingerprint', 'geometry_redecoded', 'geometry_valid',
                'same_geometry', 'length_before', 'length_after', 'global_best_updated'];
            for (var i = 0; i < req.length; i++) {
                if (obj[req[i]] === undefined) {
                    obj[req[i]] = null;
                    recordMissing('lns_events.' + req[i], 'event-' + obj.event_id);
                }
            }
            ctx._lnsLines.push(JSON.stringify(obj));
        }

        // ---- geometry_validation.json ----
        function writeGeometryValidation(obj) {
            if (!ctx.enabled) {
                return;
            }
            var required = [
                'expected_count', 'actual_count', 'unique_instance_count',
                'total_area_source', 'total_area_layout', 'area_consistent',
                'L_geom', 'L_cached', 'L_geom_consistent',
                'strip_width', 'max_x', 'length_reported', 'utilization_recalculated',
                'overlap_count', 'max_overlap_area', 'out_of_bounds_count',
                'status', 'svg_file_hash', 'svg_geometry_consistent'
            ];
            var out = {};
            for (var i = 0; i < required.length; i++) {
                var f = required[i];
                out[f] = (obj && obj[f] !== undefined) ? obj[f] : null;
                if (out[f] === null) {
                    recordMissing('geometry_validation.' + f, 'missing');
                }
            }
            out._missing_field_notes = ctx._missingFields.slice(0, 50);
            ctx._geoValidation = out;
            try {
                fs.writeFileSync(fileAt('geometry_validation.json'), JSON.stringify(out, null, 2), 'utf8');
            } catch (e) {
                recordMissing('geometry_validation.json-write', e.message);
            }
        }

        // ---- source_layout.json / final_layout.json ----
        function writeLayout(name, layoutObj) {
            if (!ctx.enabled) {
                return;
            }
            try {
                fs.writeFileSync(fileAt(name), JSON.stringify(layoutObj, null, 2), 'utf8');
            } catch (e) {
                recordMissing(name + '-write', e.message);
            }
        }

        // ---- result.svg (bound to run_id) ----
        function writeImage(imageText) {
            if (!ctx.enabled) {
                return;
            }
            // Embed a run_id comment in the image header (does not change the geometric appearance).
            // The comment must be placed 【after】 the <?xml ...?> declaration; otherwise any character
            // before the XML declaration makes the file non-well-formed, causing browsers/Inkscape/AI to fail to open it.
            var meta = '<!-- stbbs-run-id: ' + ctx.runId +
                ' | instance: ' + (ctx._meta.instance_name || '') +
                ' | seed: ' + (ctx._meta.seed != null ? ctx._meta.seed : '') + ' -->';
            var out = imageText;
            if (imageText && imageText.indexOf('<?xml') === 0) {
                // Insert after the XML declaration line
                var nl = imageText.indexOf('\n');
                out = (nl >= 0 ? imageText.slice(0, nl + 1) : imageText) +
                    meta + '\n' + (nl >= 0 ? imageText.slice(nl + 1) : '');
            } else {
                // No XML declaration: prepend the comment directly
                out = meta + '\n' + imageText;
            }
            try {
                fs.writeFileSync(fileAt('result.svg'), out, 'utf8');
            } catch (e) {
                recordMissing('result.svg-write', e.message);
            }
        }

        // ---- Batch flush to disk ----
        function writeCsv(name, header, rows, keyFn) {
            if (!ctx.enabled || rows.length === 0) {
                return;
            }
            var lines = [header];
            for (var i = 0; i < rows.length; i++) {
                lines.push(keyFn(rows[i]));
            }
            try {
                fs.writeFileSync(fileAt(name), lines.join('\n') + '\n', 'utf8');
            } catch (e) {
                recordMissing(name + '-write', e.message);
            }
        }

        function writeJsonl(name, lines) {
            if (!ctx.enabled || lines.length === 0) {
                return;
            }
            try {
                fs.writeFileSync(fileAt(name), lines.join('\n') + '\n', 'utf8');
            } catch (e) {
                recordMissing(name + '-write', e.message);
            }
        }

        function flushAll() {
            if (!ctx.enabled || ctx._closed) {
                return;
            }
            writeCsv('placement_trace.csv', PLACEMENT_HEADER, ctx._placementRows, function (r) {
                return PLACEMENT_HEADER.split(',').map(function (k) {
                    return csvEscape(r[k]);
                }).join(',');
            });
            writeCsv('beam_layers.csv', BEAM_LAYER_HEADER, ctx._beamLayerRows, function (r) {
                return BEAM_LAYER_HEADER.split(',').map(function (k) {
                    return csvEscape(r[k]);
                }).join(',');
            });
            writeJsonl('beam_candidates.jsonl', ctx._beamCandLines);
            writeJsonl('tabu_events.jsonl', ctx._tabuLines);
            writeJsonl('lns_events.jsonl', ctx._lnsLines);
            writeMetadata();
            // Missing-field summary (if any)
            if (ctx._missingFields.length > 0) {
                try {
                    fs.writeFileSync(fileAt('_missing_fields.json'),
                        JSON.stringify({ run_id: ctx.runId, missing: ctx._missingFields }, null, 2), 'utf8');
                } catch (e) { /* ignore */ }
            }
        }

        function complete(metaUpdates) {
            if (metaUpdates) {
                setMeta(metaUpdates);
            }
            flushAll();
            ctx._closed = true;
        }

        return {
            enabled: enabled,
            level: level,
            runId: ctx.runId,
            runDir: ctx.runDir,
            setMeta: setMeta,
            writeMetadata: writeMetadata,
            appendPlacementTrace: appendPlacementTrace,
            appendBeamCandidate: appendBeamCandidate,
            appendBeamLayer: appendBeamLayer,
            appendTabuEvent: appendTabuEvent,
            appendLnsEvent: appendLnsEvent,
            writeGeometryValidation: writeGeometryValidation,
            writeLayout: writeLayout,
            writeImage: writeImage,
            flushAll: flushAll,
            complete: complete,
            areaRankById: areaRankById,
            _geometryFingerprint: geometryFingerprint,
            _sequenceFingerprint: sequenceFingerprint
        };
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = {
            createDiagnostics: createDiagnostics,
            geometryFingerprint: geometryFingerprint,
            sequenceFingerprint: sequenceFingerprint
        };
    } else {
        root.StbbsDiagnostics = {
            createDiagnostics: createDiagnostics,
            geometryFingerprint: geometryFingerprint,
            sequenceFingerprint: sequenceFingerprint
        };
    }

})(typeof window !== 'undefined' ? window : global);
