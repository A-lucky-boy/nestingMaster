/**
 * nfpWorker.js
 * NFP precomputation worker thread (§9.2 multi-core parallel acceleration).
 *
 * Each worker receives a jobs shard, calls calculateNfpForPair from nest.js
 * to compute the No-Fit Polygon, and returns {key: nfpPaths}. The parent process
 * merges the results of all workers.
 *
 * jobs element format (constructed by the parent, ensuring the key is identical
 * to the single-threaded version):
 *   { A, B, aId, bId, inside, Arotation, Brotation }
 *     A, B      : polygons (arrays, with .id/.children), used for NFP computation
 *     aId, bId  : ids of the two geometries (bin uses -1), used to build the cache key (matches createNfpCacheKey)
 *     inside    : true = in-board NFP
 *
 * Re-attaches global ClipperLib/GeometryUtil inside the worker (independent per thread).
 */
'use strict';

const { parentPort } = require('worker_threads');
const { calculateNfpForPair, createNfpCacheKey } = require('./nest.js');
const ClipperLib = require('./clipper.js');
const GeometryUtil = require('./geometryutil.js');

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

parentPort.on('message', function (msg) {
    if (msg.type !== 'compute') {
        return;
    }
    const jobs = msg.jobs;
    const config = msg.config;
    const results = {};
    let done = 0;
    for (let i = 0; i < jobs.length; i++) {
        const job = jobs[i];
        // pair uses polygons; key uses ids (matches the parent's precomputeNfp keyObj)
        const pair = { A: job.A, B: job.B, key: job };
        const nfp = calculateNfpForPair(pair, config);
        if (nfp && nfp.length > 0) {
            const cacheKey = createNfpCacheKey(job.aId, job.bId, job.inside, job.Arotation, job.Brotation);
            results[cacheKey] = nfp;
        }
        done++;
    }
    parentPort.postMessage({ type: 'result', results: results, done: done });
});
