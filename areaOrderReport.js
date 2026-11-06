/**
 * areaOrderReport.js
 * test1_5: offline area-order analysis module (corresponding to section 4 of the STBBS_test1_5 spec)
 *
 * Responsibilities: offline-compute the correlation metrics between area and order
 * from the stored raw diagnostic records (placement_trace.csv / beam_candidates.jsonl /
 * lns_events.jsonl), and aggregate multi-run results.
 *
 * Strict constraints (test1_5 spec):
 *   - Pure offline analysis: only reads the stored records, does not write back candidates, does not participate in sorting, does not change geometric legality
 *   - Does not call any solving RNG (this module references no random source at all)
 *   - When data is missing, output null with a stated reason; do not fabricate, impute, or reuse the paper's old tables
 *   - Only read the final_after_lns stage for the "final actual order" analysis (LNS may reorder all instances,
 *     so the initial Beam order cannot be a substitute); also keep the beam_completed stage for comparison
 *
 * Metric definitions (section 4 of the spec):
 *   1. Spearman(k, A_(π_k)): the correlation between area and placement index; equal areas use average rank;
 *      report null when all areas are equal (rank variance is zero).
 *   2. R_A = Σ(k-1)·A_(π_k) / ((N-1)·Σ A_(π_k)), N>1 and total area>0; the smaller the value, the earlier the area weight is biased.
 *   3. Large-piece early-entry rate: the top ceil(N/4) by net area of all instances (ties fixed by instance id) form
 *      the large-piece group; count the number/proportion among them that are in the first ceil(N/4) placement positions, and output each piece's placement_index.
 *   4. delta_L association: (net_area, placement_index, delta_L) aggregated by area group and placement depth.
 */
(function (root) {
    'use strict';

    var fs = null;
    var path = null;
    try {
        fs = require('fs');
        path = require('path');
    } catch (e) {
        // Browser environment: offline analysis degrades to unavailable
    }

    // ============================================================
    // CSV parsing (handles commas inside quotes / escaped quotes)
    // ============================================================
    function parseCsvLine(line) {
        var out = [];
        var cur = '';
        var inQ = false;
        for (var i = 0; i < line.length; i++) {
            var ch = line[i];
            if (inQ) {
                if (ch === '"') {
                    if (line[i + 1] === '"') {
                        cur += '"';
                        i++;
                    } else {
                        inQ = false;
                    }
                } else {
                    cur += ch;
                }
            } else {
                if (ch === '"') {
                    inQ = true;
                } else if (ch === ',') {
                    out.push(cur);
                    cur = '';
                } else {
                    cur += ch;
                }
            }
        }
        out.push(cur);
        return out;
    }

    function readCsv(file) {
        if (!fs || !file) {
            return { header: [], rows: [] };
        }
        if (!fs.existsSync(file)) {
            return { header: [], rows: [] };
        }
        var text = fs.readFileSync(file, 'utf8');
        var lines = text.split('\n').filter(function (l) {
            return l.length > 0;
        });
        if (lines.length === 0) {
            return { header: [], rows: [] };
        }
        var header = parseCsvLine(lines[0]);
        var rows = [];
        for (var i = 1; i < lines.length; i++) {
            var cells = parseCsvLine(lines[i]);
            var obj = {};
            for (var j = 0; j < header.length; j++) {
                obj[header[j]] = cells[j];
            }
            rows.push(obj);
        }
        return { header: header, rows: rows };
    }

    function num(v) {
        if (v == null || v === '') {
            return null;
        }
        var n = parseFloat(v);
        return isNaN(n) ? null : n;
    }

    function readJsonl(file) {
        if (!fs || !file) {
            return [];
        }
        if (!fs.existsSync(file)) {
            return [];
        }
        var text = fs.readFileSync(file, 'utf8');
        var lines = text.split('\n').filter(function (l) {
            return l.length > 0;
        });
        var out = [];
        for (var i = 0; i < lines.length; i++) {
            try {
                out.push(JSON.parse(lines[i]));
            } catch (e) {
                // Skip corrupted lines
            }
        }
        return out;
    }

    // ============================================================
    // Rank computation (average rank for equal values)
    // ============================================================
    function averageRanks(values) {
        var n = values.length;
        var idx = [];
        for (var i = 0; i < n; i++) {
            idx.push(i);
        }
        idx.sort(function (a, b) {
            return values[a] - values[b];
        });
        var ranks = new Array(n);
        var i = 0;
        while (i < n) {
            var j = i;
            while (j + 1 < n && values[idx[j + 1]] === values[idx[i]]) {
                j++;
            }
            var avg = (i + j) / 2 + 1; // 1-based average rank
            for (var k = i; k <= j; k++) {
                ranks[idx[k]] = avg;
            }
            i = j + 1;
        }
        return ranks;
    }

    function variance(arr) {
        var n = arr.length;
        if (n === 0) {
            return 0;
        }
        var m = 0;
        for (var i = 0; i < n; i++) {
            m += arr[i];
        }
        m /= n;
        var s = 0;
        for (var k = 0; k < n; k++) {
            s += (arr[k] - m) * (arr[k] - m);
        }
        return s / n;
    }

    // ============================================================
    // Spearman correlation (average rank for equal values; null when rank variance is zero)
    // ============================================================
    function spearman(x, y) {
        var n = x.length;
        if (n < 2 || n !== y.length) {
            return null;
        }
        var rx = averageRanks(x);
        var ry = averageRanks(y);
        // If all values of a sequence are identical (ranks all equal, variance zero) -> null
        if (variance(rx) < 1e-12 || variance(ry) < 1e-12) {
            return null;
        }
        var mx = 0, my = 0;
        for (var i = 0; i < n; i++) {
            mx += rx[i];
            my += ry[i];
        }
        mx /= n;
        my /= n;
        var num2 = 0, dx = 0, dy = 0;
        for (var k = 0; k < n; k++) {
            num2 += (rx[k] - mx) * (ry[k] - my);
            dx += (rx[k] - mx) * (rx[k] - mx);
            dy += (ry[k] - my) * (ry[k] - my);
        }
        if (dx <= 0 || dy <= 0) {
            return null;
        }
        return num2 / Math.sqrt(dx * dy);
    }

    // ============================================================
    // Core: compute area-order metrics from placement_trace rows (same stage)
    // ============================================================
    function computeAreaOrderMetrics(rows, stage) {
        // Select the rows for the given stage (placement_index starts at 1)
        var sel = rows.filter(function (r) {
            return r.stage === stage;
        });
        if (sel.length === 0) {
            return { stage: stage, defined: false, reason: 'no-rows-for-stage' };
        }
        // Sort by placement_index
        sel.sort(function (a, b) {
            return (num(a.placement_index) || 0) - (num(b.placement_index) || 0);
        });
        var N = sel.length;
        var areas = sel.map(function (r) {
            return num(r.net_area);
        });
        var idxs = sel.map(function (r) {
            return num(r.placement_index);
        });
        var deltas = sel.map(function (r) {
            return num(r.delta_L);
        });

        // 1. Spearman(k, A)
        var sp = null;
        var spNote = null;
        if (N >= 2 && areas.every(function (a) {
            return a != null;
        })) {
            sp = spearman(idxs, areas);
            if (sp === null) {
                spNote = 'rank-variance-zero (all areas equal)';
            }
        } else {
            spNote = 'N<2 or area missing';
        }

        // 2. R_A = Σ(k-1)·A / ((N-1)·ΣA)
        var RA = null;
        var RAReason = null;
        var totalArea = 0;
        var okArea = true;
        for (var i = 0; i < N; i++) {
            if (areas[i] == null || idxs[i] == null) {
                okArea = false;
                break;
            }
            totalArea += areas[i];
        }
        if (okArea && N > 1 && totalArea > 0) {
            var sumWA = 0;
            for (var k = 0; k < N; k++) {
                sumWA += (idxs[k] - 1) * areas[k];
            }
            RA = sumWA / ((N - 1) * totalArea);
        } else {
            RAReason = (N <= 1) ? 'N<=1' : ((totalArea <= 0) ? 'totalArea<=0' : 'area-missing');
        }

        // 3. Large-piece early-entry rate: top ceil(N/4) by area (ties fixed by instance id)
        var bigSetSize = Math.ceil(N / 4);
        var bigGroup = [];
        var bigInFront = 0;
        var bigPlacementIndexes = [];
        if (N > 0 && areas.every(function (a) {
            return a != null;
        })) {
            // Sort by area descending, ties by instance id ascending; take the top bigSetSize
            var sorted = sel.slice(0).sort(function (a, b) {
                var da = num(a.net_area), db = num(b.net_area);
                if (db !== da) {
                    return db - da;
                }
                return String(a.instance_id) < String(b.instance_id) ? -1 : 1;
            });
            var bigSet = sorted.slice(0, bigSetSize);
            var bigIdSet = {};
            for (var b = 0; b < bigSet.length; b++) {
                bigIdSet[bigSet[b].instance_id] = true;
            }
            // Count the number of pieces in the large-piece group that sit in the first bigSetSize placement positions
            var frontIds = {};
            for (var f = 0; f < Math.min(bigSetSize, N); f++) {
                frontIds[sel[f].instance_id] = true;
            }
            for (var g = 0; g < bigSet.length; g++) {
                var inst = bigSet[g];
                var pi = num(inst.placement_index) || 0;
                bigPlacementIndexes.push({ instance_id: inst.instance_id, placement_index: pi, net_area: num(inst.net_area) });
                if (frontIds[inst.instance_id]) {
                    bigInFront++;
                }
            }
            bigGroup = bigSet.map(function (x) {
                return String(x.instance_id);
            });
        }

        // 4. delta_L grouping (area group + placement depth bucket)
        var deltaByGroup = {
            big: [],
            rest: []
        };
        var deltaByDepth = {};
        if (deltas.every(function (d) {
            return d != null;
        }) && areas.every(function (a) {
            return a != null;
        })) {
            // Large-piece set (same definition as above)
            var bigIdSet2 = {};
            if (bigGroup.length > 0) {
                bigGroup.forEach(function (id) {
                    bigIdSet2[id] = true;
                });
            }
            for (var d = 0; d < N; d++) {
                var isBig = !!bigIdSet2[sel[d].instance_id];
                deltaByGroup[isBig ? 'big' : 'rest'].push(deltas[d]);
                var depthBucket = idxs[d] <= bigSetSize ? 'front' : (idxs[d] > N - bigSetSize ? 'tail' : 'mid');
                if (!deltaByDepth[depthBucket]) {
                    deltaByDepth[depthBucket] = [];
                }
                deltaByDepth[depthBucket].push(deltas[d]);
            }
        }
        function summarize(arr) {
            if (!arr || arr.length === 0) {
                return { count: 0, mean: null, max: null, zeroCount: 0 };
            }
            var s = 0, mx = -Infinity, zc = 0;
            for (var i = 0; i < arr.length; i++) {
                s += arr[i];
                if (arr[i] > mx) {
                    mx = arr[i];
                }
                if (Math.abs(arr[i]) < 1e-9) {
                    zc++;
                }
            }
            return { count: arr.length, mean: s / arr.length, max: mx, zeroCount: zc };
        }

        return {
            stage: stage,
            defined: true,
            N: N,
            totalArea: totalArea,
            spearman: sp,
            spearman_note: spNote,
            areaWeightedNormPos: RA,
            areaWeightedNormPos_reason: RAReason,
            bigSetSize: bigSetSize,
            bigGroup: bigGroup,
            bigInFront: bigInFront,
            bigEarlyRate: (bigGroup.length > 0) ? (bigInFront / bigGroup.length) : null,
            bigPlacementIndexes: bigPlacementIndexes,
            delta_L_by_areaGroup: {
                big: summarize(deltaByGroup.big),
                rest: summarize(deltaByGroup.rest)
            },
            delta_L_by_depth: {
                front: summarize(deltaByDepth.front),
                mid: summarize(deltaByDepth.mid),
                tail: summarize(deltaByDepth.tail)
            }
        };
    }

    // ============================================================
    // Large-piece candidate disposal stage stats (beam_candidates.jsonl, requires full level)
    // ============================================================
    function computeCandidateDisposal(beamCands) {
        if (!beamCands || beamCands.length === 0) {
            return { defined: false, reason: 'no-beam-candidates (level=summary did not collect candidate detail)' };
        }
        var byDisp = {};
        var byDispAreaGroup = {};
        // Area grouping: top quartile / rest (based on net_area)
        var areas = beamCands.map(function (c) {
            return num(c.net_area);
        }).filter(function (a) {
            return a != null;
        }).sort(function (a, b) {
            return b - a;
        });
        var qCut = areas.length > 0 ? areas[Math.floor(areas.length * 0.25)] : null;
        function areaGroup(area) {
            if (qCut == null || area == null) {
                return 'unknown';
            }
            return area >= qCut ? 'top_quartile' : 'rest';
        }
        for (var i = 0; i < beamCands.length; i++) {
            var c = beamCands[i];
            var disp = c.final_disposition || 'UNKNOWN';
            byDisp[disp] = (byDisp[disp] || 0) + 1;
            var ag = areaGroup(num(c.net_area));
            if (!byDispAreaGroup[disp]) {
                byDispAreaGroup[disp] = {};
            }
            byDispAreaGroup[disp][ag] = (byDispAreaGroup[disp][ag] || 0) + 1;
        }
        return {
            defined: true,
            total: beamCands.length,
            byDisposition: byDisp,
            byDispositionAndAreaGroup: byDispAreaGroup
        };
    }

    // ============================================================
    // LNS distinguishability (lns_events.jsonl): sequence fingerprint vs geometry fingerprint
    // ============================================================
    function computeLnsDistinguishability(lnsEvents) {
        if (!lnsEvents || lnsEvents.length === 0) {
            return { defined: false, reason: 'no-lns-events (LNS did not run or was not collected)' };
        }
        var seqChanged = 0;
        var geomChanged = 0;
        var sameGeom = 0;
        var improved = 0;
        for (var i = 0; i < lnsEvents.length; i++) {
            var e = lnsEvents[i];
            var seqSame = (e.source_layout_fingerprint === e.candidate_layout_fingerprint);
            var geomSame = (e.same_geometry === true);
            if (!seqSame) {
                seqChanged++;
            }
            if (e.same_geometry === false) {
                geomChanged++;
            }
            if (geomSame) {
                sameGeom++;
            }
            if (e.global_best_updated) {
                improved++;
            }
        }
        return {
            defined: true,
            total: lnsEvents.length,
            sequenceChanged: seqChanged,
            geometryChanged: geomChanged,
            sameGeometryCount: sameGeom,
            globalBestUpdated: improved,
            note: 'sequenceChanged - geometryChanged = number of times only the sequence changed while geometry stayed the same (cannot be counted as a layout improvement)'
        };
    }

    // ============================================================
    // Aggregate all runs under an instance directory
    // ============================================================
    /**
     * @param {string} instanceDir A directory such as output/diagnostics/<instance>/
     * @returns {Object} { runs: [...], areaOrderAnalysis: [...], disposal: {...}, lns: {...} }
     */
    function analyzeInstanceDir(instanceDir) {
        if (!fs || !instanceDir) {
            return { runs: [], areaOrderAnalysis: [], disposal: null, lns: null };
        }
        var runs = [];
        var analysisRows = [];
        var allBeamCands = [];
        var allLnsEvents = [];

        var entries = fs.readdirSync(instanceDir).filter(function (n) {
            return n.indexOf('run_') === 0;
        });
        for (var i = 0; i < entries.length; i++) {
            var runDir = path.join(instanceDir, entries[i]);
            if (!fs.statSync(runDir).isDirectory()) {
                continue;
            }
            var metaFile = path.join(runDir, 'metadata.json');
            var meta = {};
            if (fs.existsSync(metaFile)) {
                try {
                    meta = JSON.parse(fs.readFileSync(metaFile, 'utf8'));
                } catch (e) { /* ignore */ }
            }
            // One runs.csv row
            runs.push({
                run_id: meta.run_id || entries[i],
                seed: meta.seed != null ? meta.seed : null,
                instance_name: meta.instance_name || null,
                part_count: meta.part_count != null ? meta.part_count : null,
                type_count: meta.type_count != null ? meta.type_count : null,
                strip_width: meta.strip_width != null ? meta.strip_width : null,
                total_material_area: meta.total_material_area != null ? meta.total_material_area : null,
                completed_at: meta.completed_at || null
            });

            // placement_trace.csv -> area-order analysis (one each for beam_completed and final_after_lns)
            var pt = readCsv(path.join(runDir, 'placement_trace.csv'));
            var beamMetrics = computeAreaOrderMetrics(pt.rows, 'beam_completed');
            var finalMetrics = computeAreaOrderMetrics(pt.rows, 'final_after_lns');
            analysisRows.push({
                run_id: meta.run_id || entries[i],
                stage: 'beam_completed',
                N: beamMetrics.N != null ? beamMetrics.N : null,
                spearman: beamMetrics.spearman,
                spearman_note: beamMetrics.spearman_note || null,
                areaWeightedNormPos: beamMetrics.areaWeightedNormPos,
                bigEarlyRate: beamMetrics.bigEarlyRate,
                bigInFront: beamMetrics.bigInFront != null ? beamMetrics.bigInFront : null,
                bigSetSize: beamMetrics.bigSetSize != null ? beamMetrics.bigSetSize : null
            });
            analysisRows.push({
                run_id: meta.run_id || entries[i],
                stage: 'final_after_lns',
                N: finalMetrics.N != null ? finalMetrics.N : null,
                spearman: finalMetrics.spearman,
                spearman_note: finalMetrics.spearman_note || null,
                areaWeightedNormPos: finalMetrics.areaWeightedNormPos,
                bigEarlyRate: finalMetrics.bigEarlyRate,
                bigInFront: finalMetrics.bigInFront != null ? finalMetrics.bigInFront : null,
                bigSetSize: finalMetrics.bigSetSize != null ? finalMetrics.bigSetSize : null,
                delta_L_by_areaGroup_big_mean: (finalMetrics.delta_L_by_areaGroup && finalMetrics.delta_L_by_areaGroup.big) ? finalMetrics.delta_L_by_areaGroup.big.mean : null,
                delta_L_by_areaGroup_rest_mean: (finalMetrics.delta_L_by_areaGroup && finalMetrics.delta_L_by_areaGroup.rest) ? finalMetrics.delta_L_by_areaGroup.rest.mean : null
            });

            // Candidate detail (full level)
            allBeamCands = allBeamCands.concat(readJsonl(path.join(runDir, 'beam_candidates.jsonl')));
            // LNS events
            allLnsEvents = allLnsEvents.concat(readJsonl(path.join(runDir, 'lns_events.jsonl')));
        }

        return {
            runs: runs,
            areaOrderAnalysis: analysisRows,
            disposal: computeCandidateDisposal(allBeamCands),
            lns: computeLnsDistinguishability(allLnsEvents)
        };
    }

    // ============================================================
    // Write runs.csv / area_order_analysis.csv / report.md
    // ============================================================
    function writeAggregate(instanceDir, result) {
        if (!fs || !instanceDir) {
            return;
        }
        var aggDir = path.join(instanceDir, 'aggregate');
        fs.mkdirSync(aggDir, { recursive: true });

        // runs.csv
        if (result.runs.length > 0) {
            var runHeader = 'run_id,seed,instance_name,part_count,type_count,strip_width,total_material_area,completed_at';
            var runLines = [runHeader];
            result.runs.forEach(function (r) {
                runLines.push([
                    r.run_id, r.seed, r.instance_name, r.part_count,
                    r.type_count, r.strip_width, r.total_material_area, r.completed_at
                ].map(function (v) {
                    return v == null ? '' : String(v);
                }).join(','));
            });
            fs.writeFileSync(path.join(aggDir, 'runs.csv'), runLines.join('\n') + '\n', 'utf8');
        }

        // area_order_analysis.csv
        if (result.areaOrderAnalysis.length > 0) {
            var aoHeader = 'run_id,stage,N,spearman,spearman_note,areaWeightedNormPos,bigEarlyRate,bigInFront,bigSetSize,delta_L_big_mean,delta_L_rest_mean';
            var aoLines = [aoHeader];
            result.areaOrderAnalysis.forEach(function (r) {
                aoLines.push([
                    r.run_id, r.stage, r.N,
                    r.spearman != null ? r.spearman.toFixed(6) : 'null',
                    r.spearman_note || '',
                    r.areaWeightedNormPos != null ? r.areaWeightedNormPos.toFixed(6) : 'null',
                    r.bigEarlyRate != null ? r.bigEarlyRate.toFixed(4) : 'null',
                    r.bigInFront, r.bigSetSize,
                    r.delta_L_by_areaGroup_big_mean != null ? r.delta_L_by_areaGroup_big_mean.toFixed(4) : '',
                    r.delta_L_by_areaGroup_rest_mean != null ? r.delta_L_by_areaGroup_rest_mean.toFixed(4) : ''
                ].join(','));
            });
            fs.writeFileSync(path.join(aggDir, 'area_order_analysis.csv'), aoLines.join('\n') + '\n', 'utf8');
        }

        // report.md
        var md = [];
        md.push('# STBBS test1_5 Area-Order Analysis Report');
        md.push('');
        md.push('> Offline analysis: computed only from the stored raw records; does not call the solving RNG and does not change the solving trajectory.');
        md.push('> All values come from actual runs; missing fields are output as null with a stated reason, never fabricated or imputed.');
        md.push('');
        md.push('## 1. Run Summary');
        md.push('');
        md.push('| run_id | seed | parts | types | strip width | total area |');
        md.push('|--------|------|------|--------|------|--------|');
        result.runs.forEach(function (r) {
            md.push('| ' + (r.run_id || '') + ' | ' + (r.seed != null ? r.seed : 'null') +
                ' | ' + (r.part_count != null ? r.part_count : 'null') +
                ' | ' + (r.type_count != null ? r.type_count : 'null') +
                ' | ' + (r.strip_width != null ? r.strip_width : 'null') +
                ' | ' + (r.total_material_area != null ? r.total_material_area : 'null') + ' |');
        });
        md.push('');
        md.push('## 2. Area-Order Metrics (by stage)');
        md.push('');
        md.push('- `spearman` = `Spearman(placement_index, area)` (1 = placed first, average rank for equal areas). **Negative correlation ⟺ large pieces are placed earlier (large-first / moved up); positive correlation ⟺ large pieces are deferred.**');
        md.push('- `areaWeightedNormPos (R_A)`: area-weighted normalized placement position; smaller means the area weight is biased earlier (consistent with negative spearman).');
        md.push('- `bigEarlyRate`: proportion of the large-piece group (top ceil(N/4) by area) that sit in the first ceil(N/4) placement positions (higher = larger pieces placed earlier, consistent with negative spearman).');
        md.push('');
        md.push('| run_id | stage | N | spearman | R_A | big early rate | big in-front count / big group |');
        md.push('|--------|-------|---|----------|-----|------------|--------------------|');
        result.areaOrderAnalysis.forEach(function (r) {
            md.push('| ' + r.run_id + ' | ' + r.stage +
                ' | ' + (r.N != null ? r.N : 'null') +
                ' | ' + (r.spearman != null ? r.spearman.toFixed(4) : ('null(' + (r.spearman_note || '') + ')')) +
                ' | ' + (r.areaWeightedNormPos != null ? r.areaWeightedNormPos.toFixed(4) : 'null') +
                ' | ' + (r.bigEarlyRate != null ? (r.bigEarlyRate * 100).toFixed(1) + '%' : 'null') +
                ' | ' + (r.bigInFront != null ? r.bigInFront : 'null') + '/' + (r.bigSetSize != null ? r.bigSetSize : 'null') +
                ' |');
        });
        md.push('');
        md.push('## 3. Large-Piece Candidate Disposal Stage (beam_candidates, requires level=full)');
        md.push('');
        if (result.disposal && result.disposal.defined) {
            md.push('Total candidates: ' + result.disposal.total);
            md.push('');
            md.push('| final disposition | total | top_quartile | rest |');
            md.push('|----------|------|--------------|------|');
            Object.keys(result.disposal.byDisposition).forEach(function (d) {
                var g = result.disposal.byDispositionAndAreaGroup[d] || {};
                md.push('| ' + d + ' | ' + result.disposal.byDisposition[d] +
                    ' | ' + (g.top_quartile || 0) + ' | ' + (g.rest || 0) + ' |');
            });
        } else {
            md.push('Not collected: ' + (result.disposal ? result.disposal.reason : 'no data'));
            md.push('(level=summary does not save candidate detail; requires level=full enabled for the target seeds)');
        }
        md.push('');
        md.push('## 4. LNS Distinguishability (sequence fingerprint vs geometry fingerprint)');
        md.push('');
        if (result.lns && result.lns.defined) {
            md.push('- Sequence changes: ' + result.lns.sequenceChanged);
            md.push('- Geometry changes: ' + result.lns.geometryChanged);
            md.push('- same_geometry (all poses unchanged) count: ' + result.lns.sameGeometryCount);
            md.push('- Global best updates: ' + result.lns.globalBestUpdated);
            md.push('- ' + result.lns.note);
        } else {
            md.push('Not collected: ' + (result.lns ? result.lns.reason : 'no data'));
        }
        md.push('');
        md.push('## 5. delta_L Grouping (final_after_lns)');
        md.push('');
        var finalRows = result.areaOrderAnalysis.filter(function (r) {
            return r.stage === 'final_after_lns';
        });
        if (finalRows.length > 0) {
            md.push('| run_id | big-group delta_L mean | rest-group delta_L mean |');
            md.push('|--------|--------------------|--------------------|');
            finalRows.forEach(function (r) {
                md.push('| ' + r.run_id +
                    ' | ' + (r.delta_L_by_areaGroup_big_mean != null ? r.delta_L_by_areaGroup_big_mean.toFixed(4) : 'null') +
                    ' | ' + (r.delta_L_by_areaGroup_rest_mean != null ? r.delta_L_by_areaGroup_rest_mean.toFixed(4) : 'null') +
                    ' |');
            });
        }
        md.push('');

        fs.writeFileSync(path.join(aggDir, 'report.md'), md.join('\n'), 'utf8');
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = {
            parseCsvLine: parseCsvLine,
            readCsv: readCsv,
            readJsonl: readJsonl,
            averageRanks: averageRanks,
            spearman: spearman,
            computeAreaOrderMetrics: computeAreaOrderMetrics,
            computeCandidateDisposal: computeCandidateDisposal,
            computeLnsDistinguishability: computeLnsDistinguishability,
            analyzeInstanceDir: analyzeInstanceDir,
            writeAggregate: writeAggregate
        };
    } else {
        root.StbbsAreaOrderReport = {
            parseCsvLine: parseCsvLine,
            readCsv: readCsv,
            readJsonl: readJsonl,
            averageRanks: averageRanks,
            spearman: spearman,
            computeAreaOrderMetrics: computeAreaOrderMetrics,
            computeCandidateDisposal: computeCandidateDisposal,
            computeLnsDistinguishability: computeLnsDistinguishability,
            analyzeInstanceDir: analyzeInstanceDir,
            writeAggregate: writeAggregate
        };
    }

})(typeof window !== 'undefined' ? window : global);
