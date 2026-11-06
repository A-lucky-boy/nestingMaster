/**
 * stbbs.js
 * STBBS (Stagnation-Triggered Tabu Backtracking Beam Search) algorithm core
 *
 * Implemented per "A 2D Irregular Nesting Algorithm Based on Stagnation-Triggered Tabu Backtracking Enhanced Beam Search" (202610080913.docx)
 * and chapters 5~8 of 算法文档.md:
 *   - BeamNode: with checkpoint history, ancestor chain ancestor_node_ids, node_id (§3.3)
 *   - beam_expand: expand representative instances by "remaining types", Top(2ω) weighted sampling without replacement pruning (§5)
 *   - diversity: trace back s-level ancestors, D_t(s)=U_t(s)/ω, triggered when D<=ρ and t>=max(s,ε) (§6)
 *   - tabu_repair: restore from checkpoint G_(t-r), freeze the prefix, re-decode the window of r pieces,
 *                  swap/insert neighborhood, tabu list + aspiration, no-improvement counting (§7)
 *   - merge: deduplicate the repaired set with the original Beam, then resample by merge_policy (§7.6)
 *   - run_stbbs: N-layer main loop (§8 / paper algorithm 3)
 *
 * Sole entry point for randomness: the rng (mulberry32) injected by the caller; all sampling is executed through it (§9.1).
 */
(function (root) {
    'use strict';

    function createRng(seed) {
        var a = (parseInt(seed, 10) || 1) >>> 0;
        return function () {
            a |= 0; a = a + 0x6D2B79F5 | 0;
            var t = Math.imul(a ^ a >>> 15, 1 | a);
            t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
            return ((t ^ t >>> 14) >>> 0) / 4294967296;
        };
    }

    // Weighted sampling without replacement, returns the selected indices (weights in the same order as items)
    function weightedSampleWithoutReplacement(items, weights, b, rng) {
        if (items.length === 0) {
            return [];
        }
        b = Math.min(b, items.length);
        var idxs = [];
        var remainingIdx = [];
        var remainingW = [];
        for (var i = 0; i < items.length; i++) {
            remainingIdx.push(i);
            remainingW.push(weights[i]);
        }
        for (var pick = 0; pick < b; pick++) {
            var total = 0;
            for (var k = 0; k < remainingW.length; k++) {
                total += remainingW[k];
            }
            var r = rng() * total;
            var acc = 0;
            var chosen = remainingIdx.length - 1;
            for (var m = 0; m < remainingW.length; m++) {
                acc += remainingW[m];
                if (r <= acc) {
                    chosen = m;
                    break;
                }
            }
            idxs.push(remainingIdx[chosen]);
            remainingIdx.splice(chosen, 1);
            remainingW.splice(chosen, 1);
        }
        return idxs;
    }

    // Partial layout utilization Q_t = A_t/(W·L_t) (eq. 9). A_t is the placed net area, W the fixed board width, L the occupied length
    function partialUtilization(geom, areaPlaced, binW) {
        if (geom.L <= 0) {
            return 0;
        }
        return (areaPlaced / (binW * geom.L)) * 100;
    }

    function createSolver(geomCtx, instance, config, rng) {
        var parts = geomCtx.parts;
        var binW = geomCtx.binW;
        var N = instance.N;
        var M = instance.M;
        var angles = instance.angles;
        var areaById = instance.areaById;
        var typeOfInstance = instance.typeOfInstance; // instId -> typeId
        var typeInstances = instance.typeInstances;   // typeId -> [instId,...] ascending

        var nodeCounter = 0;

        // ---- test1_5 diagnostic hook (observability, pure bypass) ----
        // diag is injected by solveInstance (config.diag). All diag calls are RNG-free:
        // they do not call rng, do not change candidate scores, do not participate in sorting, do not change geometric legality.
        // When diag=null they are all no-ops, guaranteeing the solving trajectory is exactly identical when the switch is off.
        var diag = config.diag || null;
        var diagOn = (diag && diag.enabled) ? diag : null;
        // Area quartile threshold (the top 25% of net area is recorded as top_quartile), used for candidate area grouping
        var _areaQuartile = null;
        if (diagOn) {
            var _areas = [];
            for (var _ai = 0; _ai < parts.length; _ai++) {
                _areas.push(areaById[parts[_ai].id] || 0);
            }
            _areas.sort(function (a, b) { return b - a; });
            _areaQuartile = _areas.length ? _areas[Math.floor(_areas.length * 0.25)] : 0;
        }
        function diagOn2() { return diagOn; }
        // This layer's beam_layers row (built by _annotate; runStbbs fills in the trigger field after shouldTrigger before writing)
        var lastLayerStats = null;
        // test1_5: Tabu event sequence number (increments within each runStbbs, guaranteeing uniqueness within a run)
        var tabuEventSeq = 0;

        // ---- Node construction: immutable snapshot + checkpoint + ancestor chain ----
        function makeNode(parent, placedInst, geom, J, depth) {
            var id = nodeCounter++;
            var prefix = parent.prefix.concat([placedInst]);
            var rem = parent.remaining.slice(0);
            var tid = typeOfInstance[placedInst];
            rem[tid]--;
            var areaPlaced = 0;
            for (var i = 0; i < geom.placed.length; i++) {
                areaPlaced += areaById[geom.placed[i].partId] || 0;
            }
            var Q = partialUtilization(geom, areaPlaced, binW);
            // Area priority ordering (user heuristic: place large pieces first, fill with small ones):
            // areaInv = the number of inversions in the prefix where "small pieces are before, large pieces are after" (fewer is closer to "large-first").
            // Incremental computation: newly placing aNew, new inversions = the count of pieces in the parent prefix with area < aNew (O(d)).
            var aNew = areaById[placedInst] || 0;
            var newInv = 0;
            for (var pv = 0; pv < parent.prefix.length; pv++) {
                if ((areaById[parent.prefix[pv]] || 0) < aNew) newInv++;
            }
            var areaInv = (parent.areaInv || 0) + newInv;
            // Ancestor chain: parent node's ancestors + the parent node itself (a depth-d node contains exactly d ancestor node_ids)
            var ancestors = parent.ancestorNodeIds.concat([parent.nodeId]);
            var node = {
                nodeId: id,
                depth: depth,
                prefix: prefix,
                remaining: rem,
                geometry: geom,
                Q: Q,
                L: geom.L,
                lastJ: J,
                areaPlaced: areaPlaced,
                areaInv: areaInv,
                ancestorNodeIds: ancestors,
                history: parent.history, // shared reference; checkpoints stored in ctx.checkpoints[id]
                isRepaired: false
            };
            // Save the checkpoint at this depth (depth d corresponds to the real geometry after d pieces are placed)
            geomCtx.checkpoints[id] = geom;
            return node;
        }

        function makeRoot() {
            var id = nodeCounter++;
            var rem = [];
            for (var t = 0; t < M; t++) {
                rem.push(typeInstances[t].length);
            }
            var geom = geomCtx.newState();
            var node = {
                nodeId: id,
                depth: 0,
                prefix: [],
                remaining: rem,
                geometry: geom,
                Q: 0,
                L: 0,
                lastJ: null,
                areaPlaced: 0,
                areaInv: 0,
                ancestorNodeIds: [],
                history: {},
                isRepaired: false
            };
            geomCtx.checkpoints[id] = geom; // depth-0 checkpoint: empty layout
            return node;
        }

        // ---- Stable sort key (§5.1): Q descending, L ascending, J ascending, then by area priority order (areaInv ascending), prefix string and node_id ----
        // Area priority order (user heuristic): areaInv is the inversion count of "small pieces before, large pieces after";
        // fewer means closer to "place large pieces first, fill with small ones". As a secondary criterion after Q/L/J,
        // it only favors the "large-first" sequence when quality is comparable, without sacrificing the proven better compactness.
        function prefixKey(node) {
            return node.prefix.join(',');
        }
        function beamRank(a, b) {
            if (Math.abs(b.Q - a.Q) > 1e-12) {
                return b.Q - a.Q;
            }
            if (Math.abs(a.L - b.L) > 1e-9) {
                return a.L - b.L;
            }
            var ja = (a.lastJ == null) ? 0 : a.lastJ;
            var jb = (b.lastJ == null) ? 0 : b.lastJ;
            if (config.areaPriority) {
                // Area priority order (user heuristic: place large pieces first, fill with small ones):
                // areaInv is the inversion count of "small pieces before, large pieces after"; divided by the maximum
                // inversion count at the given depth, depth*(depth-1)/2, normalized to [0,1] (0 = fully large-first, 1 = fully small-first).
                // Combined with the placement cost J into a same-level secondary objective score = J + w·invNorm,
                // so that the area order truly affects "which layout stays in the beam", rather than only taking effect when J happens to be equal.
                var ma = Math.max(1, a.depth * (a.depth - 1) / 2);
                var mb = Math.max(1, b.depth * (b.depth - 1) / 2);
                var scoreA = ja + config.areaPriorityWeight * (a.areaInv / ma);
                var scoreB = jb + config.areaPriorityWeight * (b.areaInv / mb);
                if (Math.abs(scoreA - scoreB) > 1e-9) {
                    return scoreA - scoreB;
                }
            } else {
                if (Math.abs(ja - jb) > 1e-12) {
                    return ja - jb;
                }
            }
            var ka = prefixKey(a), kb = prefixKey(b);
            if (ka < kb) {
                return -1;
            }
            if (ka > kb) {
                return 1;
            }
            return a.nodeId - b.nodeId;
        }

        // ---- Beam forward expansion (§5.4 / algorithm 1 + test1_3 §2 labels and diversity retention) ----
        function beamExpand(beam) {
            var children = [];
            var preserveLarge = config.preserveLargeBranch !== false && config.omega >= 2;
            var enableDiversity = config.enableBranchDiversity !== false;
            // test1_5: expansion attempt count and geometry-infeasible events (RNG-free)
            var expandAttempts = 0;
            var geoInfeasible = []; // {depth, parentNodeId, instanceId, typeId, netArea}
            var sortedBeam = beam.slice(0).sort(beamRank);
            for (var pi = 0; pi < sortedBeam.length; pi++) {
                var parent = sortedBeam[pi];
                // All types of this parent node with remaining > 0, sorted by typeId ascending (deterministic)
                var activeTypes = [];
                for (var t = 0; t < M; t++) {
                    if (parent.remaining[t] > 0) {
                        activeTypes.push(t);
                    }
                }
                // §4.3 area branch retention: the maximum area among this parent node's remaining types (used to mark is_large_branch)
                var maxRemArea = 0;
                for (var at = 0; at < activeTypes.length; at++) {
                    var ta = typeInstances[activeTypes[at]][0];
                    var area = areaById[ta] || 0;
                    if (area > maxRemArea) {
                        maxRemArea = area;
                    }
                }
                for (var ti = 0; ti < activeTypes.length; ti++) {
                    var typeId = activeTypes[ti];
                    // Representative instance = the smallest "not yet placed" instance id of this type (§5.2).
                    var placedSet = {};
                    for (var ps = 0; ps < parent.prefix.length; ps++) {
                        placedSet[parent.prefix[ps]] = true;
                    }
                    var instList = typeInstances[typeId];
                    var inst = null;
                    for (var il = 0; il < instList.length; il++) {
                        if (!placedSet[instList[il]]) {
                            inst = instList[il];
                            break;
                        }
                    }
                    if (inst === null) {
                        continue;
                    }
                    // test1_4 §1: multi-pose execution path fix.
                    // When enablePoseBranching=true, call placeOneTopK to return multiple spatially distinct poses;
                    // when off, fall back to placeOne single pose (original behavior).
                    expandAttempts++;
                    var results;
                    if (config.enablePoseBranching && geomCtx.placeOneTopK) {
                        results = geomCtx.placeOneTopK(parent.geometry, inst, angles, config.poseK || 3, 2.0);
                    } else {
                        var singleRes = geomCtx.placeOne(parent.geometry, inst, angles);
                        results = singleRes ? [singleRes] : [];
                    }
                    if (results.length === 0) {
                        if (diagOn) {
                            geoInfeasible.push({
                                depth: parent.depth + 1,
                                parentNodeId: parent.nodeId,
                                instanceId: inst,
                                typeId: typeId,
                                netArea: areaById[inst] || 0
                            });
                        }
                        if (config.debug) {
                            console.log('  [FAIL] node ' + parent.nodeId + ' type ' + typeId + ' inst ' + inst + ' L=' + parent.L.toFixed(2) + ' W=' + parent.geometry.W.toFixed(2) + ' placed=' + parent.geometry.placed.length + ' rem=' + parent.remaining.join(','));
                        }
                        continue; // placement failed (infeasible), skip
                    }
                    // Each pose generates an independent child node (different candidate placements of the same instance, no extra instance consumed)
                    for (var poseIdx = 0; poseIdx < results.length; poseIdx++) {
                        var res = results[poseIdx];
                        var child = makeNode(parent, inst, res.geometry, res.J, parent.depth + 1);
                        var newArea = areaById[inst] || 0;
                        // test1_3 §2 labels
                        child.isLargeBranch = preserveLarge && (maxRemArea > 0) && (newArea >= maxRemArea - 1e-9);
                        // gap_fit: placement did not increase the occupied length (δL≈0)
                        child.isGapFit = (res.L1 - parent.L) < 1e-6;
                        // rigid: the piece width is close to the board width (width projection constrained), hard to embed
                        var rb = geomCtx.rotatedBounds(inst, res.placement.rot);
                        child.isRigid = (rb.height > 0.7 * binW);
                        if (config.debug && child.geometry.placed.length !== child.prefix.length) {
                            console.log('  [MISMATCH] child ' + child.nodeId + ' prefix=' + child.prefix.length + ' placed=' + child.geometry.placed.length +
                                ' parent=' + parent.nodeId + ' parentPlaced=' + parent.geometry.placed.length + ' parentPrefix=' + parent.prefix.length);
                        }
                        // test1_5: candidate diagnostic metadata (RNG-free, does not change sorting).
                        // area_secondary_term = J + w·invNorm (consistent with the beamRank secondary objective).
                        if (diagOn) {
                            var _inv = (config.areaPriority) ?
                                (child.areaInv / Math.max(1, child.depth * (child.depth - 1) / 2)) : 0;
                            var _jv = (child.lastJ == null) ? 0 : child.lastJ;
                            child._diag = {
                                instanceId: inst,
                                typeId: typeId,
                                netArea: newArea,
                                parentDepth: parent.depth,
                                poseIndex: poseIdx,
                                poseCount: results.length,
                                J: child.lastJ,
                                areaSecondaryTerm: (config.areaPriority) ? (config.areaPriorityWeight || 0) * _inv : 0,
                                largeBranch: !!child.isLargeBranch,
                                rigidBranch: !!child.isRigid,
                                gapFit: !!child.isGapFit,
                                oldLengthFeasible: (res.delta_L != null && res.delta_L < 1e-6),
                                transitions: []
                            };
                        }
                        children.push(child);
                    }
                }
            }
            if (children.length === 0) {
                if (config.debug) {
                    console.log('[STBBS] beamExpand: 0 children at parent depth ' + (beam[0] ? beam[0].depth : -1));
                }
                return [];
            }
            if (config.debug) {
                console.log('[STBBS] beamExpand depth ' + (beam[0] ? beam[0].depth + 1 : 1) + ': children=' + children.length);
            }
            var ranked = children.slice(0).sort(beamRank);
            var omega = Math.max(1, config.omega);
            var poolSize = Math.min(2 * omega, ranked.length);
            var pool = ranked.slice(0, poolSize);
            // test1_5: rank_before_pool (1-based) mapping + set of nodes entering the Top pool
            var rankById = {};
            for (var _rk = 0; _rk < ranked.length; _rk++) {
                rankById[ranked[_rk].nodeId] = _rk + 1;
            }
            var poolSet = {};
            for (var _pk = 0; _pk < pool.length; _pk++) {
                poolSet[pool[_pk].nodeId] = true;
            }

            // test1_5: candidate destination annotation (RNG-free). Called after the branch determines sampled/reserved,
            // does not change any sorting or sampling, only writes diagnostics.
            function _annotate(beam, expandAttempts, legalCount, poolCount, sampled, reservedNodeIds, geoInfeasible) {
                if (!diagOn) {
                    return;
                }
                var sampledSet = {};
                for (var _sk = 0; _sk < sampled.length; _sk++) {
                    sampledSet[sampled[_sk].nodeId] = true;
                }
                // Geometry-infeasible candidates (no child node generated)
                var _gi = geoInfeasible || [];
                for (var _g = 0; _g < _gi.length; _g++) {
                    var _ge = _gi[_g];
                    diag.appendBeamCandidate({
                        run_id: diag.runId, depth: _ge.depth, candidate_id: 'GEO_' + _ge.parentNodeId + '_' + _ge.typeId,
                        parent_id: _ge.parentNodeId, instance_id: _ge.instanceId, type_id: _ge.typeId,
                        net_area: _ge.netArea, Q_t: null, used_length: null, J: null,
                        area_secondary_term: null, rank_before_pool: null, pose_candidate_count: 0,
                        large_branch: false, rigid_branch: false, gap_fit: false,
                        entered_top_pool: false, reserved_slot: false, weighted_sampled: false,
                        survived_merge: null, final_disposition: 'GEOMETRY_INFEASIBLE',
                        disposition_stage: 'beam_expand',
                        transitions: [{ stage: 'beam_expand', disposition: 'GEOMETRY_INFEASIBLE' }]
                    });
                }
                for (var _c = 0; _c < children.length; _c++) {
                    var _ch = children[_c];
                    var _d = _ch._diag;
                    if (!_d) {
                        continue;
                    }
                    var _inPool = !!poolSet[_ch.nodeId];
                    var _reserved = reservedNodeIds && reservedNodeIds[_ch.nodeId];
                    var _selected = !!sampledSet[_ch.nodeId];
                    var _disp;
                    if (!_inPool) {
                        _disp = 'OUTSIDE_TOP_POOL';
                    } else if (_reserved) {
                        _disp = 'RESERVED';
                    } else if (_selected) {
                        _disp = 'SELECTED';
                    } else {
                        _disp = 'NOT_SAMPLED';
                    }
                    _d.oldLengthSelected = _selected && _d.oldLengthFeasible;
                    _d.transitions.push({
                        stage: 'beam_expand',
                        enteredTopPool: _inPool,
                        reservedSlot: !!_reserved,
                        weightedSampled: _selected && !_reserved,
                        finalDisposition: _disp
                    });
                    diag.appendBeamCandidate({
                        run_id: diag.runId, depth: _ch.depth,
                        candidate_id: 'C_' + _ch.nodeId,
                        parent_id: _d.parentNodeId, instance_id: _d.instanceId, type_id: _d.typeId,
                        net_area: _d.netArea,
                        Q_t: _ch.Q, used_length: _ch.L, J: _d.J,
                        area_secondary_term: _d.areaSecondaryTerm,
                        rank_before_pool: rankById[_ch.nodeId] || null,
                        pose_candidate_count: _d.poseCount,
                        large_branch: _d.largeBranch, rigid_branch: _d.rigidBranch, gap_fit: _d.gapFit,
                        entered_top_pool: _inPool, reserved_slot: !!_reserved,
                        weighted_sampled: _selected && !_reserved,
                        survived_merge: null,
                        final_disposition: _disp,
                        disposition_stage: 'beam_expand',
                        transitions: _d.transitions
                    });
                }
                // Layer summary (beam_layers.csv)
                // top_quartile: net area >= the area quartile threshold (top 25%)
                var _tqCand = 0, _tqRet = 0;
                for (var _tc = 0; _tc < children.length; _tc++) {
                    if (children[_tc]._diag && children[_tc]._diag.netArea >= _areaQuartile) _tqCand++;
                }
                for (var _tr = 0; _tr < sampled.length; _tr++) {
                    if (sampled[_tr]._diag && sampled[_tr]._diag.netArea >= _areaQuartile) _tqRet++;
                }
                var _cumArea = 0;
                for (var _ca = 0; _ca < (beam[0] ? beam[0].prefix.length : 0); _ca++) {
                    _cumArea += areaById[beam[0].prefix[_ca]] || 0;
                }
                // Stage the layer row; the trigger field is completed by runStbbs after shouldTrigger before writing
                lastLayerStats = {
                    run_id: diag.runId,
                    depth: (beam[0] ? beam[0].depth + 1 : 1),
                    parent_node_count: beam.length,
                    expand_attempt_count: expandAttempts,
                    legal_node_count: legalCount,
                    candidate_pool_count: poolCount,
                    retained_count: sampled.length,
                    top_quartile_candidate_count: _tqCand,
                    top_quartile_retained_count: _tqRet,
                    trigger_state: null,
                    actual_ancestor_count: null,
                    beam_insufficient: sampled.length < omega,
                    cumulative_placed_area: _cumArea
                };
            }

            // test1_3 §2 spatial diversity retention: reserve 1 slot each for large/rigid/gap_fit
            // A node is not allowed to occupy multiple slots repeatedly; when a category has no candidates, the quota is backfilled to high-quality nodes
            if (enableDiversity && omega >= 3) {
                var reserved = [];
                var reservedSet = {};
                // large first
                var largeBest = null;
                for (var li = 0; li < pool.length; li++) {
                    if (pool[li].isLargeBranch) {
                        if (!largeBest || beamRank(pool[li], largeBest) < 0) largeBest = pool[li];
                    }
                }
                if (largeBest) { reserved.push(largeBest); reservedSet[largeBest.nodeId] = true; }
                // rigid
                if (reserved.length < 2) {
                    var rigidBest = null;
                    for (var ri = 0; ri < pool.length; ri++) {
                        if (pool[ri].isRigid && !reservedSet[pool[ri].nodeId]) {
                            if (!rigidBest || beamRank(pool[ri], rigidBest) < 0) rigidBest = pool[ri];
                        }
                    }
                    if (rigidBest) { reserved.push(rigidBest); reservedSet[rigidBest.nodeId] = true; }
                }
                // gap_fit
                if (reserved.length < 3) {
                    var gapBest = null;
                    for (var gi = 0; gi < pool.length; gi++) {
                        if (pool[gi].isGapFit && !reservedSet[pool[gi].nodeId]) {
                            if (!gapBest || beamRank(pool[gi], gapBest) < 0) gapBest = pool[gi];
                        }
                    }
                    if (gapBest) { reserved.push(gapBest); reservedSet[gapBest.nodeId] = true; }
                }
                // Remaining slots: quality-biased sampling
                var restPool = [];
                var restWeights = [];
                for (var rj = 0; rj < pool.length; rj++) {
                    if (!reservedSet[pool[rj].nodeId]) {
                        restPool.push(pool[rj]);
                        restWeights.push(pool.length - rj);
                    }
                }
                var bRest = Math.min(omega - reserved.length, restPool.length);
                var sampled = reserved.slice(0);
                if (bRest > 0) {
                    var picked = weightedSampleWithoutReplacement(restPool, restWeights, bRest, rng);
                    for (var sp = 0; sp < picked.length; sp++) {
                        sampled.push(restPool[picked[sp]]);
                    }
                }
                var _resOut = sampled.sort(beamRank);
                _annotate(beam, expandAttempts, children.length, pool.length, _resOut, reservedSet, geoInfeasible);
                return _resOut;
            }

            // §4.3 area-aware Beam retention (when ω=2)
            if (config.preserveLargeBranch !== false && omega >= 2) {
                var largeBest2 = null;
                for (var li2 = 0; li2 < pool.length; li2++) {
                    if (pool[li2].isLargeBranch) {
                        if (!largeBest2 || beamRank(pool[li2], largeBest2) < 0) {
                            largeBest2 = pool[li2];
                        }
                    }
                }
                if (largeBest2) {
                    var sampled2 = [largeBest2];
                    var restPool2 = [];
                    var restWeights2 = [];
                    for (var rj2 = 0; rj2 < pool.length; rj2++) {
                        if (pool[rj2] === largeBest2) { continue; }
                        restPool2.push(pool[rj2]);
                        restWeights2.push(pool.length - rj2);
                    }
                    var b2 = Math.min(omega - 1, restPool2.length);
                    if (b2 > 0) {
                        var picked2 = weightedSampleWithoutReplacement(restPool2, restWeights2, b2, rng);
                        for (var s2 = 0; s2 < picked2.length; s2++) {
                            sampled2.push(restPool2[picked2[s2]]);
                        }
                    }
                    var _res2Set = {};
                    _res2Set[largeBest2.nodeId] = true;
                    var _resOut2 = sampled2.sort(beamRank);
                    _annotate(beam, expandAttempts, children.length, pool.length, _resOut2, _res2Set, geoInfeasible);
                    return _resOut2;
                }
            }

            var weights = [];
            for (var j = 0; j < pool.length; j++) {
                weights.push(pool.length - j);
            }
            var b = Math.min(omega, pool.length);
            var pickedIdx = weightedSampleWithoutReplacement(pool, weights, b, rng);
            var sampled3 = [];
            for (var s = 0; s < pickedIdx.length; s++) {
                sampled3.push(pool[pickedIdx[s]]);
            }
            var _resOut3 = sampled3.sort(beamRank);
            _annotate(beam, expandAttempts, children.length, pool.length, _resOut3, null, geoInfeasible);
            return _resOut3;
        }

        // ---- Stagnation trigger detection (§6.4 / eq. 10) ----
        function ancestorAtDepth(node, depth) {
            // ancestorNodeIds[k] = the ancestor node node_id at depth k
            // (ancestorNodeIds[0]=root (depth 0), [1]=depth 1, …, [d-1]=parent (depth d-1))
            // therefore "the ancestor at depth depth" = ancestorNodeIds[depth]
            if (depth < 0 || depth >= node.ancestorNodeIds.length) {
                return null;
            }
            return node.ancestorNodeIds[depth];
        }
        function shouldTrigger(beam, depth) {
            var s = config.s;
            var eps = config.epsilon;
            if (depth < Math.max(s, eps) || beam.length === 0) {
                return { fire: false, D: null, U: null };
            }
            var seen = {};
            var U = 0;
            for (var i = 0; i < beam.length; i++) {
                var anc = ancestorAtDepth(beam[i], depth - s);
                if (anc != null && !seen[anc]) {
                    seen[anc] = true;
                    U++;
                }
            }
            var D = U / Math.max(1, config.omega); // strictly uses the nominal width ω per eq. 10
            return { fire: (D <= config.rho), D: D, U: U };
        }

        // ---- Window re-decoding: place r pieces in window order from the checkpoint geometry ----
        function decodeWindow(checkpointGeom, window) {
            var g = geomCtx.cloneState(checkpointGeom);
            for (var i = 0; i < window.length; i++) {
                var res = geomCtx.placeOne(g, window[i], angles);
                if (!res) {
                    return null;
                }
                g = res.geometry;
            }
            return g;
        }

        // Window area (the multiset is constant, used for Q computation)
        function windowArea(window) {
            var a = 0;
            for (var i = 0; i < window.length; i++) {
                a += areaById[window[i]] || 0;
            }
            return a;
        }

        // ---- Neighborhood moves: swap(i,j) / insert(i,j) (0<=i<j<r) ----
        function applySwap(window, i, j) {
            var s = window.slice(0);
            var tmp = s[i];
            s[i] = s[j];
            s[j] = tmp;
            return s;
        }
        function applyInsert(window, i, j) {
            var s = window.slice(0);
            var item = s.splice(i, 1)[0];
            s.splice(j, 0, item);
            return s;
        }
        // Tabu attribute of a move = the inverse-operation encoding (D08: operator type + relative position)
        function moveAttr(op, i, j) {
            return op + ':' + i + ':' + j;
        }
        // Symmetric tabu (D08: the paper does not precisely define the inverse operation). The tabu attribute is the same as the forward one,
        // i.e. "the just-accepted move (op,i,j) cannot be immediately redone within l rounds", simple and reliable.
        function inverseOf(move) {
            return moveAttr(move.op, move.i, move.j);
        }

        function enumerateMoves(window) {
            var r = window.length;
            var moves = [];
            for (var i = 0; i < r; i++) {
                for (var j = i + 1; j < r; j++) {
                    moves.push({ op: 'swap', i: i, j: j });
                }
            }
            for (i = 0; i < r; i++) {
                for (j = 0; j < r; j++) {
                    if (i === j) {
                        continue;
                    }
                    moves.push({ op: 'insert', i: i, j: j });
                }
            }
            return moves;
        }

        // ---- Tabu window repair (§7.5 / algorithm 2) ----
        function tabuRepair(node, stats) {
            var t = node.depth;
            var eps = config.epsilon;
            var r = Math.min(eps, t);
            if (r <= 1) {
                return node; // window <=1 piece has no valid neighborhood (§7.4 short-circuit)
            }
            var frozen = node.prefix.slice(0, t - r);
            var originalWindow = node.prefix.slice(t - r);

            // Restore checkpoint G_(t-r): the geometry of the depth t-r node
            var cpAncestor = ancestorAtDepth(node, t - r);
            var checkpointGeom = geomCtx.checkpoints[cpAncestor];
            if (!checkpointGeom) {
                return node;
            }

            // Original window area (frozen prefix area + window area = node.areaPlaced)
            var winArea = windowArea(originalWindow);
            var frozenArea = node.areaPlaced - winArea;

            var current = originalWindow.slice(0);
            var best = originalWindow.slice(0);
            var bestGeom = node.geometry; // the original node geometry (= the original window decoded on the checkpoint)
            var bestQ = node.Q;
            var tabu = {}; // attr -> true
            var noImprove = 0;
            var k = config.noImproveLimit;
            var hardCap = k + 5; // safety cap, prevents an over-long search caused by continuous improvement
            var iter = 0;

            function qOf(geom) {
                return partialUtilization(geom, frozenArea + winArea, binW);
            }

            while (noImprove < k && iter < hardCap) {
                iter++;
                var moves = enumerateMoves(current);
                if (moves.length === 0) {
                    break; // no valid move, return early (§7.4)
                }
                var q = Math.min(moves.length, 2 * r);
                var sampledIdx = weightedSampleWithoutReplacement(
                    moves,
                    (function () { var w = []; for (var x = 0; x < moves.length; x++) w.push(1); return w; })(),
                    q, rng);
                var admissible = [];
                for (var si = 0; si < sampledIdx.length; si++) {
                    var mv = moves[sampledIdx[si]];
                    var candWindow = (mv.op === 'swap') ? applySwap(current, mv.i, mv.j) : applyInsert(current, mv.i, mv.j);
                    var candGeom = decodeWindow(checkpointGeom, candWindow);
                    if (!candGeom) {
                        continue; // any piece cannot be legally placed => eliminate this neighborhood
                    }
                    var candQ = qOf(candGeom);
                    var inv = inverseOf(mv);
                    var isTabu = !!tabu[inv];
                    // Aspiration (per the paper): tabu but strictly better than Q_best is still acceptable
                    if (!isTabu || candQ > bestQ + 1e-12) {
                        admissible.push({ q: candQ, window: candWindow, geom: candGeom, inv: inv, mv: mv });
                    }
                }
                if (admissible.length === 0) {
                    noImprove++;
                    continue;
                }
                // Pick the largest Q; on a tie take the smaller sequence string (stable)
                admissible.sort(function (a, b2) {
                    if (Math.abs(b2.q - a.q) > 1e-12) {
                        return b2.q - a.q;
                    }
                    var ka = a.window.join(','), kb = b2.window.join(',');
                    return ka < kb ? -1 : (ka > kb ? 1 : 0);
                });
                var chosen = admissible[0];
                current = chosen.window;
                tabu[chosen.inv] = true;
                // Tabu list capacity l: keep only the most recent l
                var keys = Object.keys(tabu);
                while (keys.length > config.tabuLength) {
                    delete tabu[keys.shift()];
                    keys = Object.keys(tabu);
                }
                if (chosen.q > bestQ + 1e-12) {
                    best = chosen.window.slice(0);
                    bestGeom = chosen.geom;
                    bestQ = chosen.q;
                    noImprove = 0;
                } else {
                    noImprove++;
                }
            }
            stats.tabuCalls++;
            stats.tabuIters += iter;

            // If the best is the original window => return the original node directly (dedup is left to merge)
            var sameAsOriginal = (best.join(',') === originalWindow.join(','));
            // test1_5: Tabu event (RNG-free). source = original window geometry, candidate = repaired geometry.
            // same_geometry = true when the sequence is unchanged (pose unchanged), not counted as a valid geometry repair.
            if (diagOn) {
                tabuEventSeq++;
                var _srcFp = diag._geometryFingerprint(node.geometry);
                var _candFp = sameAsOriginal ? _srcFp : diag._geometryFingerprint(bestGeom);
                var _moved = sameAsOriginal ? [] : best;
                var _movedAreas = _moved.map(function (x) { return areaById[x] || 0; });
                diag.appendTabuEvent({
                    run_id: diag.runId, stage: 'tabu_repair', event_id: tabuEventSeq,
                    operator: 'tabu_window', source_node_id: node.nodeId,
                    source_layout_fingerprint: _srcFp, candidate_layout_fingerprint: _candFp,
                    moved_instance_ids: _moved, moved_net_areas: _movedAreas,
                    old_positions: null, new_positions: null,
                    geometry_redecoded: !sameAsOriginal, geometry_valid: true,
                    same_geometry: sameAsOriginal,
                    length_before: node.L, length_after: sameAsOriginal ? node.L : bestGeom.L,
                    window: sameAsOriginal ? originalWindow : best,
                    iterations: iter,
                    // The Tabu window repair being adopted (the order within the window changed) is considered a local-optimum update of this node
                    global_best_updated: !sameAsOriginal
                });
            }
            if (sameAsOriginal) {
                return node;
            }

            // Rebuild the repaired node: frozen prefix + new window, rebuild the window checkpoints and ancestor chain (§6.2 / §7.6)
            var prefix = frozen.concat(best);
            var rem = node.remaining.slice(0);
            var newAncestors = node.ancestorNodeIds.slice(0, t - r); // frozen-prefix ancestors (including the depth t-r node)
            // Establish new node_id checkpoints for the r pieces of the window
            var g = geomCtx.cloneState(checkpointGeom);
            for (var w = 0; w < best.length; w++) {
                var res = geomCtx.placeOne(g, best[w], angles);
                if (!res) {
                    return node; // should not happen in theory (bestGeom is already verified feasible), fallback to the original node
                }
                g = res.geometry;
                var nid = nodeCounter++;
                geomCtx.checkpoints[nid] = g;
                newAncestors.push(nid);
            }
            var finalGeom = bestGeom; // use the geometry obtained from the search (consistent with the re-decoding)
            var areaPlaced = frozenArea + winArea;
            var Q = partialUtilization(finalGeom, areaPlaced, binW);
            // Recompute the area inversions of the repaired node (the prefix has changed, O(d^2) direct counting)
            var repInv = 0;
            for (var ri = 0; ri < prefix.length; ri++) {
                for (var rj = ri + 1; rj < prefix.length; rj++) {
                    if ((areaById[prefix[ri]] || 0) > (areaById[prefix[rj]] || 0)) repInv++;
                }
            }
            var id = nodeCounter++;
            geomCtx.checkpoints[id] = finalGeom;
            var repaired = {
                nodeId: id,
                depth: t,
                prefix: prefix,
                remaining: rem,
                geometry: finalGeom,
                Q: Q,
                L: finalGeom.L,
                lastJ: node.lastJ,
                areaPlaced: areaPlaced,
                areaInv: repInv,
                ancestorNodeIds: newAncestors,
                history: node.history,
                isRepaired: true
            };
            if (config.debug && (repaired.geometry.placed.length !== repaired.prefix.length)) {
                console.log('  [tabuRepair MISMATCH] node ' + node.nodeId + '(in placed=' + node.geometry.placed.length + '/prefix=' + node.prefix.length +
                    ') -> repaired ' + repaired.nodeId + '(placed=' + repaired.geometry.placed.length + '/prefix=' + repaired.prefix.length +
                    ') r=' + r + ' bestGeom.placed=' + bestGeom.placed.length + ' checkpointGeom.placed=' + checkpointGeom.placed.length);
            }
            return repaired;
        }

        // ---- Dedup + resampling re-injection (§7.6) ----
        function dedupKey(node) {
            var geo = node.geometry;
            var g = geo.placed.map(function (p, i) {
                return p.partId + '@' + Math.round(geo.placements[i].x * 1e4) + ',' + Math.round(geo.placements[i].y * 1e4) + '@' + p.rot;
            }).join(';');
            return node.prefix.join(',') + '|' + g;
        }
        function mergeBeam(beam, repaired, stats) {
            var merged = beam.concat(repaired);
            var seen = {};
            var uniq = [];
            for (var i = 0; i < merged.length; i++) {
                var k = dedupKey(merged[i]);
                if (!seen[k]) {
                    seen[k] = true;
                    uniq.push(merged[i]);
                }
            }
            var omega = Math.max(1, config.omega);
            var ranked = uniq.slice(0).sort(beamRank);
            if (config.mergePolicy === 'deterministic_top') {
                return ranked.slice(0, omega);
            }
            // Default biased_sample (algorithm 3): Top(2ω) weighted sampling without replacement to pick ω
            var poolSize = Math.min(2 * omega, ranked.length);
            var pool = ranked.slice(0, poolSize);
            var weights = [];
            for (var j = 0; j < pool.length; j++) {
                weights.push(pool.length - j);
            }
            var b = Math.min(omega, pool.length);
            var idxs = weightedSampleWithoutReplacement(pool, weights, b, rng);
            var sampled = [];
            for (var s = 0; s < idxs.length; s++) {
                sampled.push(pool[idxs[s]]);
            }
            return sampled.sort(beamRank);
        }

        // ---- Main program (§8 / paper algorithm 3) ----
        function runStbbs() {
            // ω upper-bound protection (for D instances where M≫N, ceil(M/3) is too large, so the beam width is capped to prevent expansion explosion)
            if (config.omegaCap && config.omegaCap > 0) {
                var capped = Math.min(config.omega, config.omegaCap);
                if (capped < 1) {
                    capped = 1;
                }
                config.omega = capped;
            }
            var stats = {
                seed: rng ? 'seeded' : 0,
                tabuCalls: 0,
                tabuIters: 0,
                triggerLayers: [],
                totalNodes: nodeCounter,
                geometryEval: 0,
                layerLog: [],
                elapsedMs: 0
            };
            // Time budget explicitly disabled: the timeout branch is only enabled when timeBudgetMs is a finite positive number.
            // 0 / null / undefined / Infinity are all treated as "unlimited"; the Beam fully places all N pieces.
            // algoStart is only used for logging and elapsedMs recording, and no longer affects search decisions.
            var _tLimitRaw = config.timeBudgetMs;
            var hasTimeLimit = Number.isFinite(_tLimitRaw) && _tLimitRaw > 0;
            var timeBudgetMs = hasTimeLimit ? _tLimitRaw : 0;
            var algoStart = Date.now();
            // test1_5: reset the diagnostic event sequence number on each run (guarantees event uniqueness within a run)
            tabuEventSeq = 0;
            lastLayerStats = null;
            var root = makeRoot();
            var beam = [root];

            for (var depth = 1; depth <= N; depth++) {
                // Time budget: check at the entry of each layer only when hasTimeLimit (a finite positive budget);
                // when unlimited (timeBudgetMs=0/null/Infinity) this branch is never entered, and the Beam fully places all N pieces.
                if (hasTimeLimit && (Date.now() - algoStart) > timeBudgetMs) {
                    stats.status = 'TIME_BUDGET_EXCEEDED';
                    stats.elapsedMs = Date.now() - algoStart;
                    if (config.debug) {
                        console.log('[STBBS] time budget exceeded at depth ' + depth + '/' + N + ' elapsed=' + stats.elapsedMs + 'ms (budget=' + timeBudgetMs + 'ms)');
                    }
                    break;
                }
                if (config.debug) {
                    console.log('[STBBS] --- depth ' + depth + '/' + N + ' (beam before expand=' + beam.length + ')');
                }
                beam = beamExpand(beam);
                if (beam.length === 0) {
                    stats.elapsedMs = Date.now() - algoStart;
                    if (config.debug) {
                        console.log('[STBBS] beam collapsed at depth ' + depth);
                    }
                    return {
                        status: 'NO_COMPLETE_LAYOUT',
                        stats: stats,
                        beam: beam
                    };
                }
                var trig = shouldTrigger(beam, depth);
                stats.totalNodes = nodeCounter;
                stats.layerLog.push({
                    depth: depth,
                    beamSize: beam.length,
                    D: trig.D,
                    U: trig.U,
                    fire: trig.fire
                });
                // test1_5: after completing the trigger field of the layer row, write to beam_layers.csv (RNG-free)
                if (diagOn && lastLayerStats) {
                    lastLayerStats.trigger_state = trig.fire ? 'TRIGGERED' : (trig.D != null ? 'OK_D=' + trig.D.toFixed(3) : 'NOT_CHECKED');
                    lastLayerStats.actual_ancestor_count = trig.U != null ? trig.U : null;
                    diag.appendBeamLayer(lastLayerStats);
                    lastLayerStats = null;
                }
                if (trig.fire) {
                    stats.triggerLayers.push(depth);
                    var repaired = [];
                    for (var i = 0; i < beam.length; i++) {
                        repaired.push(tabuRepair(beam[i], stats));
                    }
                    beam = mergeBeam(beam, repaired, stats);
                }
            }

            if (config.debug) {
                for (var dgi = 0; dgi < beam.length; dgi++) {
                    var nd = beam[dgi];
                    var remSum = 0;
                    for (var rr = 0; rr < nd.remaining.length; rr++) remSum += nd.remaining[rr];
                    var uniq = {};
                    var dup = 0;
                    for (var ui = 0; ui < nd.prefix.length; ui++) {
                        if (uniq[nd.prefix[ui]]) dup++;
                        uniq[nd.prefix[ui]] = true;
                    }
                    var placedIds = [];
                    for (var pi = 0; pi < nd.geometry.placed.length; pi++) placedIds.push(nd.geometry.placed[pi].partId);
                    console.log('[STBBS] final beam node ' + nd.nodeId + ': depth=' + nd.depth +
                        ' prefix=' + nd.prefix.length + ' placed=' + nd.geometry.placed.length +
                        ' remainingSum=' + remSum + ' uniq=' + Object.keys(uniq).length + ' dup=' + dup +
                        '\n    prefixIds=' + nd.prefix.join(',') +
                        '\n    placedIds=' + placedIds.join(','));
                }
            }
            var winner = null;
            for (var w = 0; w < beam.length; w++) {
                if (beam[w].depth === N) {
                    if (!winner || beamRank(beam[w], winner) < 0) {
                        winner = beam[w];
                    }
                }
            }
            // Early wrap-up on time budget (TIME_BUDGET_EXCEEDED): when there is no complete layout, take the node in the beam with
            // "the most placed pieces and the highest utilization" as the partial best layout
            if (!winner && beam.length > 0) {
                winner = beam[0];
                for (var wp = 1; wp < beam.length; wp++) {
                    if (beamRank(beam[wp], winner) < 0) {
                        winner = beam[wp];
                    }
                }
            }
            if (!winner) {
                return { status: 'NO_COMPLETE_LAYOUT', stats: stats, beam: beam };
            }
            stats.totalNodes = nodeCounter;
            stats.geometryEval = geomCtx.evalCounter.place;
            stats.elapsedMs = Date.now() - algoStart;

            // test1_5 §1: preserve the Beam completion order before LNS (beam_placement_order).
            // If LNS improves and replaces winner.prefix, result.sequence becomes the final order after LNS,
            // and cannot be used as the Beam order; so snapshot the pre-LNS winner.prefix here into stats.
            stats.beamOrder = winner.prefix.slice();
            // test1_5 §5: capture the real geometry of the Beam winner (before LNS), so source_layout uses the real placements,
            // rather than the greedy replay geometry (the two may differ: the Beam uses placeOneTopK multi-pose, the replay uses placeOne deterministic).
            stats.beamWinnerL = winner.geometry.L;
            stats.beamWinnerPlacements = winner.geometry.placed.map(function (p, i) {
                return { partId: p.partId, rot: p.rot, x: winner.geometry.placements[i].x, y: winner.geometry.placements[i].y };
            });

            // test1_3 §4: LNS large-neighborhood destroy-rebuild (executed only for complete layouts)
            var lnsResult = null;
            if (config.enableLns && winner.depth === N && (stats.status !== 'TIME_BUDGET_EXCEEDED')) {
                var runLns = (typeof require !== 'undefined') ? require('./lns.js').runLns : root.StbbsLns.runLns;
                var lnsStart = Date.now();
                lnsResult = runLns(geomCtx, { N: N, angles: angles, areaById: areaById },
                    { prefix: winner.prefix, geometry: winner.geometry },
                    { lnsBudgetMs: config.lnsBudgetMs, lnsMaxIters: config.lnsMaxIters,
                      lnsOperators: config.lnsOperators, lnsNoImproveLimit: config.lnsNoImproveLimit,
                      debug: config.debug, diag: config.diag },
                    rng);
                stats.lnsElapsedMs = Date.now() - lnsStart;
                stats.lnsIterations = lnsResult.iterations;
                stats.lnsImproved = lnsResult.improved;
                if (lnsResult.improved) {
                    winner = {
                        prefix: lnsResult.bestLayout.prefix,
                        geometry: lnsResult.bestLayout.geometry,
                        L: lnsResult.bestLayout.geometry.L,
                        Q: partialUtilization(lnsResult.bestLayout.geometry, winner.areaPlaced, binW),
                        areaPlaced: winner.areaPlaced
                    };
                    stats.status = 'OK_LNS_IMPROVED';
                }
            }

            return {
                status: (stats.status === 'TIME_BUDGET_EXCEEDED') ? 'TIME_BUDGET_EXCEEDED' : (stats.status === 'OK_LNS_IMPROVED' ? 'OK_LNS_IMPROVED' : 'OK'),
                winner: winner,
                placements: winner.geometry.placed.map(function (p, i) {
                    return {
                        partId: p.partId,
                        rotation: p.rot,
                        x: winner.geometry.placements[i].x,
                        y: winner.geometry.placements[i].y
                    };
                }),
                sequence: winner.prefix.slice(0),
                usedLength: winner.geometry.L,
                binW: binW,
                materialArea: winner.areaPlaced,
                utilizationPercent: (winner.Q != null) ? winner.Q : partialUtilization(winner.geometry, winner.areaPlaced, binW),
                stats: stats
            };
        }

        return {
            runStbbs: runStbbs,
            beamExpand: beamExpand,
            shouldTrigger: shouldTrigger,
            tabuRepair: tabuRepair,
            mergeBeam: mergeBeam,
            makeRoot: makeRoot,
            partialUtilization: partialUtilization,
            weightedSampleWithoutReplacement: weightedSampleWithoutReplacement
        };
    }

    if (typeof module !== 'undefined' && module.exports) {
        module.exports = { createRng: createRng, createSolver: createSolver, weightedSampleWithoutReplacement: weightedSampleWithoutReplacement, partialUtilization: partialUtilization };
    } else {
        root.StbbsSolver = { createRng: createRng, createSolver: createSolver };
    }

})(typeof window !== 'undefined' ? window : global);
