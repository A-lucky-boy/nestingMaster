/**
 * Nest - core nesting engine
 * Modified for the command-line version, with all rendering features removed
 */
(function(root){
	'use strict';

	function createNfpCacheKey(A, B, inside, Arotation, Brotation){
		if(typeof A === 'object'){
			var key = A;
			return key.A + '|' + key.B + '|' + (key.inside ? 1 : 0) + '|' + key.Arotation + '|' + key.Brotation;
		}

		return A + '|' + B + '|' + (inside ? 1 : 0) + '|' + Arotation + '|' + Brotation;
	}

	function Nest(){
		var self = this;

		var parts = null;           // piece list
		var tree = null;            // piece tree structure (including holes)
		var binPolygon = null;      // sheet polygon
		var binBounds = null;       // sheet bounds
		var nfpCache = {};          // NFP cache
		var config = {
			clipperScale: 10000000,
			curveTolerance: 0.3,
			spacing: 0,
			rotations: 4,
			populationSize: 10,
			mutationRate: 10,
			useHoles: false,
			exploreConcave: false
		};

		this.working = false;
		var GA = null;
		var best = null;
		var progress = 0;
		var partAreaById = null;

		function clonePolygon(polygon){
			if(!polygon){
				return null;
			}

			var cloned = [];
			for(var i = 0; i < polygon.length; i++){
				cloned.push({x: polygon[i].x, y: polygon[i].y});
			}

			if('id' in polygon){
				cloned.id = polygon.id;
			}
			if('source' in polygon){
				cloned.source = polygon.source;
			}
			if('rotation' in polygon){
				cloned.rotation = polygon.rotation;
			}
			if('width' in polygon){
				cloned.width = polygon.width;
			}
			if('height' in polygon){
				cloned.height = polygon.height;
			}
			if(polygon.children && polygon.children.length > 0){
				cloned.children = [];
				for(var j = 0; j < polygon.children.length; j++){
					cloned.children.push(clonePolygon(polygon.children[j]));
				}
			}

			return cloned;
		}

		function clonePolygonList(list){
			var cloned = [];
			for(var i = 0; i < list.length; i++){
				cloned.push(clonePolygon(list[i]));
			}
			return cloned;
		}

		function normalizePolygonList(list){
			for(var i = 0; i < list.length; i++){
				var start = list[i][0];
				var end = list[i][list[i].length - 1];
				if(start === end || (GeometryUtil.almostEqual(start.x, end.x) && GeometryUtil.almostEqual(start.y, end.y))){
					list[i].pop();
				}

				if(GeometryUtil.polygonArea(list[i]) > 0){
					list[i].reverse();
				}

				if(list[i].children && list[i].children.length > 0){
					normalizePolygonList(list[i].children);
				}
			}
		}

		function translatePolygon(polygon, dx, dy){
			for(var i = 0; i < polygon.length; i++){
				polygon[i].x += dx;
				polygon[i].y += dy;
			}

			if(polygon.children && polygon.children.length > 0){
				for(var j = 0; j < polygon.children.length; j++){
					translatePolygon(polygon.children[j], dx, dy);
				}
			}
		}

		function getPolygonExtents(polygon){
			var xmax = polygon[0].x;
			var xmin = polygon[0].x;
			var ymax = polygon[0].y;
			var ymin = polygon[0].y;

			for(var i = 1; i < polygon.length; i++){
				if(polygon[i].x > xmax){
					xmax = polygon[i].x;
				}
				else if(polygon[i].x < xmin){
					xmin = polygon[i].x;
				}
				if(polygon[i].y > ymax){
					ymax = polygon[i].y;
				}
				else if(polygon[i].y < ymin){
					ymin = polygon[i].y;
				}
			}

			return {
				xmin: xmin,
				xmax: xmax,
				ymin: ymin,
				ymax: ymax,
				width: xmax - xmin,
				height: ymax - ymin
			};
		}

		function getPolygonNetArea(polygon){
			var area = Math.abs(GeometryUtil.polygonArea(polygon));

			if(polygon.children && polygon.children.length > 0){
				for(var i = 0; i < polygon.children.length; i++){
					area -= Math.abs(GeometryUtil.polygonArea(polygon.children[i]));
				}
			}

			return Math.max(area, 0);
		}

		function getPartAreas(partList){
			var areas = [];

			for(var i = 0; i < partList.length; i++){
				areas[partList[i].id] = getPolygonNetArea(partList[i]);
			}

			return areas;
		}

		function getPlacedBounds(placement, partList){
			var allpoints = [];

			for(var i = 0; i < placement.length; i++){
				var part = placement[i];
				var polygon = rotatePolygon(partList[part.id], part.rotation);

				for(var j = 0; j < polygon.length; j++){
					allpoints.push({
						x: polygon[j].x + part.x,
						y: polygon[j].y + part.y
					});
				}
			}

			if(allpoints.length === 0){
				return null;
			}

			return GeometryUtil.getPolygonBounds(allpoints);
		}

		function calculateMaterialUtilization(placements, partList, partAreas){
			if(!placements || placements.length === 0){
				return 0;
			}

			var usedPartArea = 0;
			var usedMaterialArea = 0;

			for(var i = 0; i < placements.length; i++){
				var placement = placements[i];
				var bounds = getPlacedBounds(placement, partList);

				if(bounds){
					usedMaterialArea += bounds.width * bounds.height;
				}

				for(var j = 0; j < placement.length; j++){
					usedPartArea += partAreas[placement[j].id] || 0;
				}
			}

			if(usedMaterialArea <= 0){
				return 0;
			}

			return usedPartArea / usedMaterialArea;
		}

		this.config = function(c){
			if(!c){
				return config;
			}

			if(c.curveTolerance && !GeometryUtil.almostEqual(parseFloat(c.curveTolerance), 0)){
				config.curveTolerance = parseFloat(c.curveTolerance);
			}

			if('spacing' in c){
				config.spacing = parseFloat(c.spacing);
			}

			if(c.rotations && parseInt(c.rotations) > 0){
				config.rotations = parseInt(c.rotations);
			}

			if(c.populationSize && parseInt(c.populationSize) > 2){
				config.populationSize = parseInt(c.populationSize);
			}

			if(c.mutationRate && parseInt(c.mutationRate) > 0){
				config.mutationRate = parseInt(c.mutationRate);
			}

			if('useHoles' in c){
				config.useHoles = !!c.useHoles;
			}

			if('exploreConcave' in c){
				config.exploreConcave = !!c.exploreConcave;
			}

			best = null;
			nfpCache = {};
			binPolygon = null;
			GA = null;

			return config;
		};

		this.setBin = function(polygon){
			binPolygon = polygon;
			return this;
		};

		this.setParts = function(partsList){
			parts = partsList;
			return this;
		};

		this.setNfpCache = function(cache){
			nfpCache = cache || {};
			return this;
		};

		// callback: progress callback, parameters are {progress, fitness, placedCount, totalCount}
		this.start = function(maxIterations, callback){
			if(!binPolygon || !parts || parts.length === 0){
				throw new Error('Sheet or parts not set');
			}

			// Build the piece tree
			tree = this.getParts(parts);
			var originalParts = clonePolygonList(tree);

			// Apply spacing offset to the piece tree
			offsetTree(tree, 0.5 * config.spacing, this.polygonOffset.bind(this));

			function offsetTree(t, offset, offsetFunction){
				for(var i = 0; i < t.length; i++){
					var offsetpaths = offsetFunction(t[i], offset);
					if(offsetpaths.length === 1){
						Array.prototype.splice.apply(t[i], [0, t[i].length].concat(offsetpaths[0]));
					}

					if(t[i].children && t[i].children.length > 0){
						offsetTree(t[i].children, -offset, offsetFunction);
					}
				}
			}

			// Clean the sheet polygon
			binPolygon = this.cleanPolygon(binPolygon);

			if(!binPolygon || binPolygon.length < 3){
				throw new Error('Sheet polygon invalid');
			}

			var originalBinPolygon = clonePolygon(binPolygon);
			var originalBinExtents = getPolygonExtents(originalBinPolygon);

			binBounds = GeometryUtil.getPolygonBounds(binPolygon);

			if(config.spacing > 0){
				var offsetBin = this.polygonOffset(binPolygon, -0.5 * config.spacing);
				if(offsetBin.length === 1){
					binPolygon = offsetBin.pop();
				}
			}

			binPolygon.id = -1;
			originalBinPolygon.id = -1;

			// Move the sheet to the origin
			var binExtents = getPolygonExtents(binPolygon);
			var xbinmax = binExtents.xmax;
			var xbinmin = binExtents.xmin;
			var ybinmax = binExtents.ymax;
			var ybinmin = binExtents.ymin;

			translatePolygon(binPolygon, -xbinmin, -ybinmin);
			translatePolygon(originalBinPolygon, -originalBinExtents.xmin, -originalBinExtents.ymin);

			binPolygon.width = xbinmax - xbinmin;
			binPolygon.height = ybinmax - ybinmin;
			originalBinPolygon.width = originalBinExtents.width;
			originalBinPolygon.height = originalBinExtents.height;

			// Record the translation from the nesting sheet coordinate system to the original sheet coordinate system
			var partOutputOffset = {
				x: xbinmin - originalBinExtents.xmin,
				y: ybinmin - originalBinExtents.ymin
			};

			// Ensure counter-clockwise direction
			if(GeometryUtil.polygonArea(binPolygon) > 0){
				binPolygon.reverse();
			}
			if(GeometryUtil.polygonArea(originalBinPolygon) > 0){
				originalBinPolygon.reverse();
			}

			// Clean the piece polygons
			normalizePolygonList(tree);
			normalizePolygonList(originalParts);
			partAreaById = getPartAreas(originalParts);

			// Initialize the genetic algorithm
			var adam = tree.slice(0);

			// Sort by area descending
			adam.sort(function(a, b){
				return Math.abs(GeometryUtil.polygonArea(b)) - Math.abs(GeometryUtil.polygonArea(a));
			});

			GA = new GeneticAlgorithm(adam, binPolygon, config);

			// Iterative optimization
			var iteration = 0;
			var lastProgress = -1;
			// Iteration limit: stop if there is no improvement within 10% of the iterations
			var checkpointInterval = Math.max(1, Math.ceil(maxIterations * 0.1));
			var lastUtilizationImprovedIteration = 0;
			var bestMaterialUtilization = 0;
			var utilizationTolerance = 1e-10;
			var utilizationHistory = [];
			var stoppedEarly = false;
			var stopReason = null;

			function buildResult(includeHistory){
				return {
					placements: best ? best.placements : null,
					fitness: best ? best.fitness : null,
					materialUtilization: best ? best.materialUtilization : 0,
					iterationsCompleted: iteration,
					maxIterations: maxIterations,
					stoppedEarly: stoppedEarly,
					stopReason: stopReason,
					binPolygon: binPolygon,
					originalBinPolygon: originalBinPolygon,
					partOutputOffset: partOutputOffset,
					parts: tree,
					originalParts: originalParts,
					utilizationHistory: includeHistory ? utilizationHistory.slice(0) : undefined
				};
			}

			while(iteration < maxIterations){
				iteration++;

				// Evaluate the population
				var allEvaluated = true;
				for(var i = 0; i < GA.population.length; i++){
					if(!GA.population[i].fitness){
						allEvaluated = false;
						this.evaluateIndividual(GA.population[i]);
						break;
					}
				}

				if(allEvaluated){
					GA.generation();
				}

				var currentMaterialUtilization = best ? best.materialUtilization : 0;
				utilizationHistory.push({
					iteration: iteration,
					materialUtilization: currentMaterialUtilization
				});

				if(currentMaterialUtilization > bestMaterialUtilization + utilizationTolerance){
					bestMaterialUtilization = currentMaterialUtilization;
					lastUtilizationImprovedIteration = iteration;
				}

				var isCheckpoint = iteration % checkpointInterval === 0 || iteration === maxIterations;

				// Compute progress
				var placedCount = 0;
				var totalCount = 0;
				if(best && best.placements){
					for(var i = 0; i < best.placements.length; i++){
						placedCount += best.placements[i].length;
					}
				}
				totalCount = tree.length;

				var currentProgress = Math.floor((iteration / maxIterations) * 100);
				if(currentProgress !== lastProgress || isCheckpoint){
					if(callback){
						callback({
							iteration: iteration,
							maxIterations: maxIterations,
							progress: currentProgress,
							fitness: best ? best.fitness : null,
							materialUtilization: currentMaterialUtilization,
							placedCount: placedCount,
							totalCount: totalCount,
							isCheckpoint: isCheckpoint,
							result: isCheckpoint ? buildResult(false) : null
						});
					}
					lastProgress = currentProgress;
				}

				if(iteration > checkpointInterval && iteration - lastUtilizationImprovedIteration >= checkpointInterval){
					stoppedEarly = true;
					stopReason = 'No material utilization improvement within 5% of max iterations.';

					if(callback){
						callback({
							iteration: iteration,
							maxIterations: maxIterations,
							progress: currentProgress,
							fitness: best ? best.fitness : null,
							materialUtilization: currentMaterialUtilization,
							placedCount: placedCount,
							totalCount: totalCount,
							isCheckpoint: true,
							stoppedEarly: true,
							stopReason: stopReason,
							result: buildResult(false)
						});
					}
					break;
				}
			}

			// Return the final result
			return buildResult(true);
		};

		this.evaluateIndividual = function(individual){
			var placelist = individual.placement;
			var rotations = individual.rotation;

			var ids = [];
			for(var i = 0; i < placelist.length; i++){
				ids.push(placelist[i].id);
				placelist[i].rotation = rotations[i];
			}

			var nfpPairs = [];
			var key;
			var newCache = {};

			for(var i = 0; i < placelist.length; i++){
				var part = placelist[i];
				key = {A: binPolygon.id, B: part.id, inside: true, Arotation: 0, Brotation: rotations[i]};
				var cacheKey = createNfpCacheKey(key);
				if(!nfpCache[cacheKey]){
					nfpPairs.push({A: binPolygon, B: part, key: key, cacheKey: cacheKey});
				}
				else{
					newCache[cacheKey] = nfpCache[cacheKey];
				}
				for(var j = 0; j < i; j++){
					var placed = placelist[j];
					key = {A: placed.id, B: part.id, inside: false, Arotation: rotations[j], Brotation: rotations[i]};
					cacheKey = createNfpCacheKey(key);
					if(!nfpCache[cacheKey]){
						nfpPairs.push({A: placed, B: part, key: key, cacheKey: cacheKey});
					}
					else{
						newCache[cacheKey] = nfpCache[cacheKey];
					}
				}
			}

			// nfpCache = newCache;

			// Compute all NFPs
			for(var i = 0; i < nfpPairs.length; i++){
				var pair = nfpPairs[i];
				var searchEdges = config.exploreConcave;
				var useHoles = config.useHoles;

				var A = rotatePolygon(pair.A, pair.key.Arotation);
				var B = rotatePolygon(pair.B, pair.key.Brotation);

				var nfp;

				if(pair.key.inside){
					if(GeometryUtil.isRectangle(A, 0.001)){
						nfp = GeometryUtil.noFitPolygonRectangle(A, B);
					}
					else{
						nfp = GeometryUtil.noFitPolygon(A, B, true, searchEdges);
					}

					if(nfp && nfp.length > 0){
						for(var j = 0; j < nfp.length; j++){
							if(GeometryUtil.polygonArea(nfp[j]) > 0){
								nfp[j].reverse();
							}
						}
					}
				}
				else{
					if(searchEdges){
						nfp = GeometryUtil.noFitPolygon(A, B, false, searchEdges);
					}
					else{
						nfp = minkowskiDifference(A, B);
					}

					if(!nfp || nfp.length === 0){
						continue;
					}

					for(var j = 0; j < nfp.length; j++){
						if(!searchEdges || j === 0){
							if(Math.abs(GeometryUtil.polygonArea(nfp[j])) < Math.abs(GeometryUtil.polygonArea(A))){
								nfp.splice(j, 1);
								j--;
							}
						}
					}

					if(nfp.length === 0){
						continue;
					}

					for(var j = 0; j < nfp.length; j++){
						if(GeometryUtil.polygonArea(nfp[j]) > 0){
							nfp[j].reverse();
						}

						if(j > 0){
							if(GeometryUtil.pointInPolygon(nfp[j][0], nfp[0])){
								if(GeometryUtil.polygonArea(nfp[j]) < 0){
									nfp[j].reverse();
								}
							}
						}
					}

					if(useHoles && A.children && A.children.length > 0){
						var Bbounds = GeometryUtil.getPolygonBounds(B);

						for(var j = 0; j < A.children.length; j++){
							var Abounds = GeometryUtil.getPolygonBounds(A.children[j]);

							if(Abounds.width > Bbounds.width && Abounds.height > Bbounds.height){
								var cnfp = GeometryUtil.noFitPolygon(A.children[j], B, true, searchEdges);
								if(cnfp && cnfp.length > 0){
									for(var k = 0; k < cnfp.length; k++){
										if(GeometryUtil.polygonArea(cnfp[k]) < 0){
											cnfp[k].reverse();
										}
										nfp.push(cnfp[k]);
									}
								}
							}
						}
					}
				}

				var nfpKey = pair.cacheKey || createNfpCacheKey(pair.key);
				nfpCache[nfpKey] = nfp;
			}

			// Place the pieces
			var worker = new PlacementWorker(binPolygon, placelist.slice(0), ids, rotations, config, nfpCache);
			var placements = worker.placePaths(placelist.slice(0));

			if(placements){
				individual.fitness = placements.fitness;
				placements.materialUtilization = calculateMaterialUtilization(placements.placements, tree, partAreaById);
				individual.materialUtilization = placements.materialUtilization;

				if(!best || placements.fitness < best.fitness){
					best = placements;
				}
			}
		};

		this.getParts = function(paths){
			var polygons = [];

			function cleanPolygonWithChildren(poly, self){
				if(!poly){
					return null;
				}

				var cleaned = self.cleanPolygon(poly);
				if(!cleaned || cleaned.length <= 2){
					return null;
				}

				if(Math.abs(GeometryUtil.polygonArea(cleaned)) <= config.curveTolerance * config.curveTolerance){
					return null;
				}

				if('source' in poly){
					cleaned.source = poly.source;
				}

				if(poly.children && poly.children.length > 0){
					cleaned.children = [];
					for(var i = 0; i < poly.children.length; i++){
						var child = cleanPolygonWithChildren(poly.children[i], self);
						if(child){
							child.parent = cleaned;
							cleaned.children.push(child);
						}
					}

					if(cleaned.children.length === 0){
						delete cleaned.children;
					}
				}

				return cleaned;
			}

			for(var i = 0; i < paths.length; i++){
				var poly = cleanPolygonWithChildren(paths[i], this);
				if(poly){
					poly.source = ('source' in poly) ? poly.source : i;
					poly.id = polygons.length;
					polygons.push(poly);
				}
			}

			return polygons;
		};

		this.polygonOffset = function(polygon, offset){
			if(!offset || offset === 0 || GeometryUtil.almostEqual(offset, 0)){
				return [polygon];
			}

			var p = this.polygonToClipper(polygon);

			var miterLimit = 2;
			var co = new ClipperLib.ClipperOffset(miterLimit, config.curveTolerance * config.clipperScale);
			co.AddPath(p, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);

			var newpaths = new ClipperLib.Paths();
			co.Execute(newpaths, offset * config.clipperScale);

			var result = [];
			for(var i = 0; i < newpaths.length; i++){
				result.push(this.clipperToPolygon(newpaths[i]));
			}

			return result;
		};

		this.cleanPolygon = function(polygon){
			var p = this.polygonToClipper(polygon);
			var simple = ClipperLib.Clipper.SimplifyPolygon(p, ClipperLib.PolyFillType.pftNonZero);

			if(!simple || simple.length === 0){
				return null;
			}

			var biggest = simple[0];
			var biggestarea = Math.abs(ClipperLib.Clipper.Area(biggest));
			for(var i = 1; i < simple.length; i++){
				var area = Math.abs(ClipperLib.Clipper.Area(simple[i]));
				if(area > biggestarea){
					biggest = simple[i];
					biggestarea = area;
				}
			}

			var clean = ClipperLib.Clipper.CleanPolygon(biggest, config.curveTolerance * config.clipperScale);

			if(!clean || clean.length === 0){
				return null;
			}

			return this.clipperToPolygon(clean);
		};

		this.polygonToClipper = function(polygon){
			var clip = [];
			for(var i = 0; i < polygon.length; i++){
				clip.push({X: polygon[i].x, Y: polygon[i].y});
			}

			ClipperLib.JS.ScaleUpPath(clip, config.clipperScale);

			return clip;
		};

		this.clipperToPolygon = function(polygon){
			var normal = [];

			for(var i = 0; i < polygon.length; i++){
				normal.push({x: polygon[i].X / config.clipperScale, y: polygon[i].Y / config.clipperScale});
			}

			return normal;
		};

		this.applyPlacement = function(placement){
			var result = [];

			for(var i = 0; i < placement.length; i++){
				var binResult = {
					bin: binPolygon,
					parts: []
				};

				for(var j = 0; j < placement[i].length; j++){
					var p = placement[i][j];
					var part = tree[p.id];

					binResult.parts.push({
						id: p.id,
						x: p.x,
						y: p.y,
						rotation: p.rotation,
						vertices: part,
						children: part.children
					});
				}

				result.push(binResult);
			}

			return result;
		};
	}

	function rotatePolygon(polygon, angle){
		var rotated = [];
		var rad = angle * Math.PI / 180;

		for(var i = 0; i < polygon.length; i++){
			var x = polygon[i].x;
			var y = polygon[i].y;
			var x1 = x * Math.cos(rad) - y * Math.sin(rad);
			var y1 = x * Math.sin(rad) + y * Math.cos(rad);
			rotated.push({x: x1, y: y1});
		}

		if(polygon.children && polygon.children.length > 0){
			rotated.children = [];
			for(var j = 0; j < polygon.children.length; j++){
				rotated.children.push(rotatePolygon(polygon.children[j], angle));
			}
		}

		return rotated;
	}

// Minkowski difference
	function minkowskiDifference(A, B){
		var Ac = [];
		for(var i = 0; i < A.length; i++){
			Ac.push({X: A[i].x, Y: A[i].y});
		}
		ClipperLib.JS.ScaleUpPath(Ac, 10000000);

		var Bc = [];
		for(var i = 0; i < B.length; i++){
			Bc.push({X: B[i].x, Y: B[i].y});
		}
		ClipperLib.JS.ScaleUpPath(Bc, 10000000);

		for(var i = 0; i < Bc.length; i++){
			Bc[i].X *= -1;
			Bc[i].Y *= -1;
		}

		var solution = ClipperLib.Clipper.MinkowskiSum(Ac, Bc, true);
		var clipperNfp = null;
		var largestArea = null;

		for(var i = 0; i < solution.length; i++){
			var n = [];
			for(var j = 0; j < solution[i].length; j++){
				n.push({x: solution[i][j].X / 10000000, y: solution[i][j].Y / 10000000});
			}
			var sarea = GeometryUtil.polygonArea(n);
			if(largestArea === null || largestArea > sarea){
				clipperNfp = n;
				largestArea = sarea;
			}
		}

		if(!clipperNfp){
			return null;
		}

		for(var i = 0; i < clipperNfp.length; i++){
			clipperNfp[i].x += B[0].x;
			clipperNfp[i].y += B[0].y;
		}

		return [clipperNfp];
	}

	function calculateNfpForPair(pair, config){
		var searchEdges = config.exploreConcave;
		var useHoles = config.useHoles;
		var A = rotatePolygon(pair.A, pair.key.Arotation);
		var B = rotatePolygon(pair.B, pair.key.Brotation);
		var nfp;

		if(pair.key.inside){
			if(GeometryUtil.isRectangle(A, 0.001)){
				nfp = GeometryUtil.noFitPolygonRectangle(A, B);
			}
			else{
				nfp = GeometryUtil.noFitPolygon(A, B, true, searchEdges);
			}

			if(nfp && nfp.length > 0){
				for(var j = 0; j < nfp.length; j++){
					if(GeometryUtil.polygonArea(nfp[j]) > 0){
						nfp[j].reverse();
					}
				}
			}
		}
		else{
			if(searchEdges){
				nfp = GeometryUtil.noFitPolygon(A, B, false, searchEdges);
			}
			else{
				nfp = minkowskiDifference(A, B);
			}

			if(!nfp || nfp.length === 0){
				return null;
			}

			for(var j = 0; j < nfp.length; j++){
				if(!searchEdges || j === 0){
					if(Math.abs(GeometryUtil.polygonArea(nfp[j])) < Math.abs(GeometryUtil.polygonArea(A))){
						nfp.splice(j, 1);
						j--;
					}
				}
			}

			if(nfp.length === 0){
				return null;
			}

			for(var j = 0; j < nfp.length; j++){
				if(GeometryUtil.polygonArea(nfp[j]) > 0){
					nfp[j].reverse();
				}

				if(j > 0){
					if(GeometryUtil.pointInPolygon(nfp[j][0], nfp[0])){
						if(GeometryUtil.polygonArea(nfp[j]) < 0){
							nfp[j].reverse();
						}
					}
				}
			}

			if(useHoles && A.children && A.children.length > 0){
				var Bbounds = GeometryUtil.getPolygonBounds(B);

				for(var j = 0; j < A.children.length; j++){
					var Abounds = GeometryUtil.getPolygonBounds(A.children[j]);

					if(Abounds.width > Bbounds.width && Abounds.height > Bbounds.height){
						var cnfp = GeometryUtil.noFitPolygon(A.children[j], B, true, searchEdges);
						if(cnfp && cnfp.length > 0){
							for(var k = 0; k < cnfp.length; k++){
								if(GeometryUtil.polygonArea(cnfp[k]) < 0){
									cnfp[k].reverse();
								}
								nfp.push(cnfp[k]);
							}
						}
					}
				}
			}
		}

		return nfp;
	}

// Genetic algorithm
	function GeneticAlgorithm(adam, bin, config){
		this.config = config || { populationSize: 10, mutationRate: 10, rotations: 4 };
		this.binBounds = GeometryUtil.getPolygonBounds(bin);

		var angles = [];
		for(var i = 0; i < adam.length; i++){
			angles.push(this.randomAngle(adam[i]));
		}

		this.population = [{placement: adam, rotation: angles}];

		while(this.population.length < config.populationSize){
			var mutant = this.mutate(this.population[0]);
			this.population.push(mutant);
		}
	}

	GeneticAlgorithm.prototype.randomAngle = function(part){
		var angleList = [];
		for(var i = 0; i < Math.max(this.config.rotations, 1); i++){
			angleList.push(i * (360 / this.config.rotations));
		}

		function shuffleArray(array){
			for(var i = array.length - 1; i > 0; i--){
				var j = Math.floor(Math.random() * (i + 1));
				var temp = array[i];
				array[i] = array[j];
				array[j] = temp;
			}
			return array;
		}

		angleList = shuffleArray(angleList);

		for(var i = 0; i < angleList.length; i++){
			var rotatedPart = GeometryUtil.rotatePolygon(part, angleList[i]);

			if(rotatedPart.width < this.binBounds.width && rotatedPart.height < this.binBounds.height){
				return angleList[i];
			}
		}

		return 0;
	}

	GeneticAlgorithm.prototype.mutate = function(individual){
		var clone = {placement: individual.placement.slice(0), rotation: individual.rotation.slice(0)};

		for(var i = 0; i < clone.placement.length; i++){
			var rand = Math.random();
			if(rand < 0.01 * this.config.mutationRate){
				var j = i + 1;
				if(j < clone.placement.length){
					var temp = clone.placement[i];
					clone.placement[i] = clone.placement[j];
					clone.placement[j] = temp;
				}
			}

			rand = Math.random();
			if(rand < 0.01 * this.config.mutationRate){
				clone.rotation[i] = this.randomAngle(clone.placement[i]);
			}
		}

		return clone;
	}

	GeneticAlgorithm.prototype.mate = function(male, female){
		var cutpoint = Math.round(Math.min(Math.max(Math.random(), 0.1), 0.9) * (male.placement.length - 1));

		var gene1 = male.placement.slice(0, cutpoint);
		var rot1 = male.rotation.slice(0, cutpoint);

		var gene2 = female.placement.slice(0, cutpoint);
		var rot2 = female.rotation.slice(0, cutpoint);

		function contains(gene, id){
			for(var i = 0; i < gene.length; i++){
				if(gene[i].id === id){
					return true;
				}
			}
			return false;
		}

		for(var i = 0; i < female.placement.length; i++){
			if(!contains(gene1, female.placement[i].id)){
				gene1.push(female.placement[i]);
				rot1.push(female.rotation[i]);
			}
		}

		for(var i = 0; i < male.placement.length; i++){
			if(!contains(gene2, male.placement[i].id)){
				gene2.push(male.placement[i]);
				rot2.push(male.rotation[i]);
			}
		}

		return [{placement: gene1, rotation: rot1}, {placement: gene2, rotation: rot2}];
	}

	GeneticAlgorithm.prototype.generation = function(){
		this.population.sort(function(a, b){
			return a.fitness - b.fitness;
		});

		var newpopulation = [this.population[0]];

		while(newpopulation.length < this.population.length){
			var male = this.randomWeightedIndividual();
			var female = this.randomWeightedIndividual(male);

			var children = this.mate(male, female);

			newpopulation.push(this.mutate(children[0]));

			if(newpopulation.length < this.population.length){
				newpopulation.push(this.mutate(children[1]));
			}
		}

		this.population = newpopulation;
	}

	GeneticAlgorithm.prototype.randomWeightedIndividual = function(exclude){
		var pop = this.population.slice(0);

		if(exclude && pop.indexOf(exclude) >= 0){
			pop.splice(pop.indexOf(exclude), 1);
		}

		var rand = Math.random();
		var lower = 0;
		var weight = 1 / pop.length;
		var upper = weight;

		for(var i = 0; i < pop.length; i++){
			if(rand > lower && rand < upper){
				return pop[i];
			}
			lower = upper;
			upper += 2 * weight * ((pop.length - i) / pop.length);
		}

		return pop[0];
	}

// PlacementWorker class
	function PlacementWorker(binPolygon, paths, ids, rotations, config, nfpCache){
		this.binPolygon = binPolygon;
		this.paths = paths;
		this.ids = ids;
		this.rotations = rotations;
		this.config = config;
		this.nfpCache = nfpCache || {};
	}

	PlacementWorker.prototype.placePaths = function(paths){
		var self = this;

		if(!self.binPolygon){
			return null;
		}

		var i, j, k, m, n, path;

		// Compute the translated bounding box
		function getTranslatedBounds(polygon, placement){
			var bounds = GeometryUtil.getPolygonBounds(polygon);
			if(!bounds){
				return null;
			}

			return {
				x: bounds.x + placement.x,
				y: bounds.y + placement.y,
				width: bounds.width,
				height: bounds.height
			};
		}

		// Bounding-box fast rejection
		function boundsOverlap(a, b){
			if(!a || !b){
				return false;
			}

			if(a.x + a.width <= b.x || b.x + b.width <= a.x){
				return false;
			}

			if(a.y + a.height <= b.y || b.y + b.height <= a.y){
				return false;
			}

			return true;
		}

		// Recursively convert a piece and its holes into Clipper paths
		// isHole = false means the outer contour, = true means a hole contour
		function appendPolygonToClipperPaths(polygon, offsetx, offsety, isHole, outPaths){
			if(!polygon || polygon.length < 3){
				return;
			}

			var clipPath = [];
			for(var idx = 0; idx < polygon.length; idx++){
				clipPath.push({
					X: polygon[idx].x + offsetx,
					Y: polygon[idx].y + offsety
				});
			}

			ClipperLib.JS.ScaleUpPath(clipPath, self.config.clipperScale);
			clipPath = ClipperLib.Clipper.CleanPolygon(clipPath, 0.0001 * self.config.clipperScale);

			if(!clipPath || clipPath.length < 3){
				return;
			}

			// Explicitly set the direction for Clipper
			// Outer contours are unified to the positive direction, hole contours to the negative direction
			var clipArea = ClipperLib.Clipper.Area(clipPath);
			if(isHole){
				if(clipArea > 0){
					clipPath.reverse();
				}
			}
			else{
				if(clipArea < 0){
					clipPath.reverse();
				}
			}

			outPaths.push(clipPath);

			if(polygon.children && polygon.children.length > 0){
				for(var c = 0; c < polygon.children.length; c++){
					appendPolygonToClipperPaths(
						polygon.children[c],
						offsetx,
						offsety,
						!isHole,
						outPaths
					);
				}
			}
		}

		// Use Clipper to do exact intersection detection on two translated pieces
		// If the intersection area is greater than a tiny threshold, it is judged as overlapping
		function polygonsOverlap(candidatePolygon, candidatePlacement, placedPolygon, placedPlacement){
			var candidateBounds = getTranslatedBounds(candidatePolygon, candidatePlacement);
			var placedBounds = getTranslatedBounds(placedPolygon, placedPlacement);

			// First do a bounding-box fast rejection
			if(!boundsOverlap(candidateBounds, placedBounds)){
				return false;
			}

			var subjectPaths = [];
			var clipPaths = [];

			appendPolygonToClipperPaths(
				candidatePolygon,
				candidatePlacement.x,
				candidatePlacement.y,
				false,
				subjectPaths
			);

			appendPolygonToClipperPaths(
				placedPolygon,
				placedPlacement.x,
				placedPlacement.y,
				false,
				clipPaths
			);

			if(subjectPaths.length === 0 || clipPaths.length === 0){
				return false;
			}

			var clipper = new ClipperLib.Clipper();
			clipper.AddPaths(subjectPaths, ClipperLib.PolyType.ptSubject, true);
			clipper.AddPaths(clipPaths, ClipperLib.PolyType.ptClip, true);

			var solution = new ClipperLib.Paths();
			if(!clipper.Execute(
				ClipperLib.ClipType.ctIntersection,
				solution,
				ClipperLib.PolyFillType.pftNonZero,
				ClipperLib.PolyFillType.pftNonZero
			)){
				return false;
			}

			// A tiny area is treated as numerical error and not judged as overlapping
			var minOverlapArea = 1e-6 * self.config.clipperScale * self.config.clipperScale;

			for(var s = 0; s < solution.length; s++){
				var interArea = Math.abs(ClipperLib.Clipper.Area(solution[s]));
				if(interArea > minOverlapArea){
					return true;
				}
			}

			return false;
		}

		// Check whether the candidate position overlaps any placed piece
		function hasOverlapWithPlaced(candidatePolygon, candidatePlacement, placed, placements){
			for(var p = 0; p < placed.length; p++){
				if(polygonsOverlap(candidatePolygon, candidatePlacement, placed[p], placements[p])){
					return true;
				}
			}
			return false;
		}

		var rotated = [];
		for(i = 0; i < paths.length; i++){
			var r = rotatePolygon(paths[i], paths[i].rotation);
			r.rotation = paths[i].rotation;
			r.source = paths[i].source;
			r.id = paths[i].id;
			rotated.push(r);
		}

		paths = rotated;

		var allplacements = [];
		var fitness = 0;
		var binarea = Math.abs(GeometryUtil.polygonArea(self.binPolygon));
		var key, nfp;

		while(paths.length > 0){
			var placed = [];
			var placements = [];
			fitness += 1;

			for(i = 0; i < paths.length; i++){
				path = paths[i];

				key = createNfpCacheKey(-1, path.id, true, 0, path.rotation);
				var binNfp = self.nfpCache[key];

				if(!binNfp || binNfp.length === 0){
					continue;
				}

				var error = false;
				for(j = 0; j < placed.length; j++){
						key = createNfpCacheKey(placed[j].id, path.id, false, placed[j].rotation, path.rotation);
						nfp = self.nfpCache[key];

					if(!nfp){
						error = true;
						break;
					}
				}

				if(error){
					continue;
				}

				var position = null;
				if(placed.length === 0){
					for(j = 0; j < binNfp.length; j++){
						for(k = 0; k < binNfp[j].length; k++){
							var firstPlacement = {
								x: binNfp[j][k].x - path[0].x,
								y: binNfp[j][k].y - path[0].y,
								id: path.id,
								rotation: path.rotation
							};

							if(position === null || firstPlacement.x < position.x){
								position = firstPlacement;
							}
						}
					}

					if(position){
						placements.push(position);
						placed.push(path);
					}
					continue;
				}

				var clipperBinNfp = [];
				for(j = 0; j < binNfp.length; j++){
					var cp = [];
					for(k = 0; k < binNfp[j].length; k++){
						cp.push({X: binNfp[j][k].x, Y: binNfp[j][k].y});
					}
					ClipperLib.JS.ScaleUpPath(cp, self.config.clipperScale);
					clipperBinNfp.push(cp);
				}

				var clipper = new ClipperLib.Clipper();
				var combinedNfp = new ClipperLib.Paths();

				for(j = 0; j < placed.length; j++){
					key = createNfpCacheKey(placed[j].id, path.id, false, placed[j].rotation, path.rotation);
					nfp = self.nfpCache[key];

					if(!nfp){
						continue;
					}

					for(k = 0; k < nfp.length; k++){
						var clone = [];
						for(m = 0; m < nfp[k].length; m++){
							clone.push({X: nfp[k][m].x + placements[j].x, Y: nfp[k][m].y + placements[j].y});
						}

						ClipperLib.JS.ScaleUpPath(clone, self.config.clipperScale);
						clone = ClipperLib.Clipper.CleanPolygon(clone, 0.0001 * self.config.clipperScale);
						var area = Math.abs(ClipperLib.Clipper.Area(clone));
						if(clone.length > 2 && area > 0.1 * self.config.clipperScale * self.config.clipperScale){
							clipper.AddPath(clone, ClipperLib.PolyType.ptSubject, true);
						}
					}
				}

				if(!clipper.Execute(ClipperLib.ClipType.ctUnion, combinedNfp, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)){
					continue;
				}

				var finalNfp = new ClipperLib.Paths();
				clipper = new ClipperLib.Clipper();

				clipper.AddPaths(combinedNfp, ClipperLib.PolyType.ptClip, true);
				clipper.AddPaths(clipperBinNfp, ClipperLib.PolyType.ptSubject, true);
				if(!clipper.Execute(ClipperLib.ClipType.ctDifference, finalNfp, ClipperLib.PolyFillType.pftNonZero, ClipperLib.PolyFillType.pftNonZero)){
					continue;
				}

				finalNfp = ClipperLib.Clipper.CleanPolygons(finalNfp, 0.0001 * self.config.clipperScale);

				for(j = 0; j < finalNfp.length; j++){
					var area = Math.abs(ClipperLib.Clipper.Area(finalNfp[j]));
					if(finalNfp[j].length < 3 || area < 0.1 * self.config.clipperScale * self.config.clipperScale){
						finalNfp.splice(j, 1);
						j--;
					}
				}

				if(!finalNfp || finalNfp.length === 0){
					continue;
				}

				var f = [];
				for(j = 0; j < finalNfp.length; j++){
					var nf = [];
					for(k = 0; k < finalNfp[j].length; k++){
						nf.push({x: finalNfp[j][k].X / self.config.clipperScale, y: finalNfp[j][k].Y / self.config.clipperScale});
					}
					f.push(nf);
				}
				finalNfp = f;

				var minwidth = null;
				var minarea = null;
				var minx = null;

				for(j = 0; j < finalNfp.length; j++){
					var nf = finalNfp[j];
					if(Math.abs(GeometryUtil.polygonArea(nf)) < 2){
						continue;
					}

					for(k = 0; k < nf.length; k++){
						var shiftvector = {
							x: nf[k].x - path[0].x,
							y: nf[k].y - path[0].y,
							id: path.id,
							rotation: path.rotation
						};

						// Added final anti-overlap check
						// If the candidate point truly area-overlaps any placed piece, skip it directly
						if(hasOverlapWithPlaced(path, shiftvector, placed, placements)){
							continue;
						}

						var allpoints = [];
						for(m = 0; m < placed.length; m++){
							for(n = 0; n < placed[m].length; n++){
								allpoints.push({x: placed[m][n].x + placements[m].x, y: placed[m][n].y + placements[m].y});
							}
						}

						for(m = 0; m < path.length; m++){
							allpoints.push({x: path[m].x + shiftvector.x, y: path[m].y + shiftvector.y});
						}

						var rectbounds = GeometryUtil.getPolygonBounds(allpoints);
						var area = rectbounds.width * 2 + rectbounds.height;

						if(minarea === null || area < minarea || (GeometryUtil.almostEqual(minarea, area) && (minx === null || shiftvector.x < minx))){
							minarea = area;
							minwidth = rectbounds.width;
							position = shiftvector;
							minx = shiftvector.x;
						}
					}
				}

				if(position){
					placed.push(path);
					placements.push(position);
				}
			}

			if(minwidth){
				fitness += minwidth / binarea;
			}

			for(i = 0; i < placed.length; i++){
				var index = paths.indexOf(placed[i]);
				if(index >= 0){
					paths.splice(index, 1);
				}
			}

			if(placements && placements.length > 0){
				allplacements.push(placements);
			}
			else{
				break;
			}
		}

		fitness += 2 * paths.length;

		return { placements: allplacements, fitness: fitness, paths: paths, area: binarea };
	}

	if(typeof module !== 'undefined' && module.exports){
		module.exports = {
			Nest: Nest,
			PlacementWorker: PlacementWorker,
			rotatePolygon: rotatePolygon,
			minkowskiDifference: minkowskiDifference,
			calculateNfpForPair: calculateNfpForPair,
			createNfpCacheKey: createNfpCacheKey
		};
	}

})(typeof window !== 'undefined' ? window : global);
