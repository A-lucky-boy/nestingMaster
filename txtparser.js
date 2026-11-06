/**
 * TxtParser - for parsing text files of the sheet and pieces
 * Supports the following two piece formats:
 * 1. Old format: each PIECE has only one outer contour
 * 2. New format: each PIECE contains one OUTER and zero or more HOLEs
 */
(function(root){
	'use strict';

	function TxtParser(){
		this.conf = {
			tolerance: 0.3  // tolerance for geometric calculations
		};
	}

	TxtParser.prototype.config = function(config){
		if(config && config.tolerance !== undefined){
			this.conf.tolerance = config.tolerance;
		}
	};

// Parse the txt content of a single piece/sheet
	TxtParser.prototype.parseShape = function(txtContent){
		var lines = txtContent.trim().split('\n');
		var vertices = [];
		var readingVertices = false;
		var numVertices = 0;

		for(var i = 0; i < lines.length; i++){
			var line = lines[i].trim();

			if(line === ''){
				continue;
			}

			// Read the number of vertices
			if(line.startsWith('NUMBER OF VERTICES')){
				if(i + 1 < lines.length){
					numVertices = parseInt(lines[i + 1].trim(), 10);
					i++;
					continue;
				}
			}

			// Start reading vertices
			if(line.startsWith('VERTICES (X,Y)')){
				readingVertices = true;
				continue;
			}

			// Read vertex coordinates
			if(readingVertices && vertices.length < numVertices){
				var coords = line.split(/\s+/).filter(function(x){ return x !== ''; });
				if(coords.length >= 2){
					vertices.push({
						x: parseFloat(coords[0]),
						y: parseFloat(coords[1])
					});
				}
			}
		}

		if(vertices.length < 3){
			throw new Error('Invalid shape: less than 3 vertices');
		}

		return vertices;
	};

// Parse a file containing multiple pieces
// New format example:
// PIECE 1
// QUANTITY
// 2
// OUTER
// NUMBER OF VERTICES
// 8
// VERTICES (X,Y)
// ...
// HOLE 1
// NUMBER OF VERTICES
// 4
// VERTICES (X,Y)
// ...
	TxtParser.prototype.parsePiecesFile = function(txtContent){
		var lines = txtContent.trim().split('\n');
		var pieces = [];
		var currentPiece = null;
		var currentContourType = null;
		var currentVertices = [];
		var currentNumVertices = 0;
		var readingVertices = false;

		function createPiece(pieceId){
			return {
				id: pieceId,
				quantity: 1,
				outer: null,
				holes: []
			};
		}

		function finalizeContour(){
			if(!currentPiece || currentVertices.length === 0){
				return;
			}

			var contourType = currentContourType;
			if(!contourType){
				contourType = currentPiece.outer ? 'HOLE' : 'OUTER';
			}

			if(contourType === 'OUTER'){
				if(currentPiece.outer !== null){
					throw new Error('Invalid piece format: duplicated OUTER contour in PIECE ' + currentPiece.id);
				}
				currentPiece.outer = currentVertices.slice(0);
			}
			else if(contourType === 'HOLE'){
				currentPiece.holes.push(currentVertices.slice(0));
			}
			else{
				throw new Error('Invalid contour type in PIECE ' + currentPiece.id);
			}

			currentVertices = [];
			currentNumVertices = 0;
			currentContourType = null;
			readingVertices = false;
		}

		function finalizePiece(){
			if(currentPiece === null){
				return;
			}

			finalizeContour();

			if(currentPiece.outer === null){
				throw new Error('Invalid piece format: missing OUTER contour in PIECE ' + currentPiece.id);
			}

			pieces.push(currentPiece);
		}

		for(var i = 0; i < lines.length; i++){
			var line = lines[i].trim();

			if(line === ''){
				continue;
			}

			if(line.startsWith('PIECE')){
				finalizePiece();
				var parts = line.split(/\s+/);
				var pieceId = parts.length >= 2 ? parseInt(parts[1], 10) : (pieces.length + 1);
				currentPiece = createPiece(pieceId);
				currentContourType = null;
				currentVertices = [];
				currentNumVertices = 0;
				readingVertices = false;
				continue;
			}

			if(currentPiece === null){
				continue;
			}

			if(line === 'QUANTITY'){
				if(i + 1 < lines.length){
					currentPiece.quantity = parseInt(lines[i + 1].trim(), 10);
					i++;
					continue;
				}
			}

			if(line === 'OUTER'){
				finalizeContour();
				currentContourType = 'OUTER';
				continue;
			}

			if(line.startsWith('HOLE')){
				finalizeContour();
				currentContourType = 'HOLE';
				continue;
			}

			if(line.startsWith('NUMBER OF VERTICES')){
				if(i + 1 < lines.length){
					currentNumVertices = parseInt(lines[i + 1].trim(), 10);
					i++;
					continue;
				}
			}

			if(line.startsWith('VERTICES (X,Y)')){
				currentVertices = [];
				readingVertices = true;
				continue;
			}

			if(readingVertices){
				var coords = line.split(/\s+/).filter(function(x){ return x !== ''; });
				if(coords.length >= 2){
					currentVertices.push({
						x: parseFloat(coords[0]),
						y: parseFloat(coords[1])
					});

					if(currentVertices.length >= currentNumVertices){
						finalizeContour();
					}
				}
			}
		}

		finalizePiece();

		return pieces;
	};

// Convert a polygon into a format suitable for the nesting algorithm
	TxtParser.prototype.preparePolygon = function(vertices){
		var polygon = vertices.slice(0);

		var first = polygon[0];
		var last = polygon[polygon.length - 1];

		// If the first and last points coincide, remove the last point
		if(polygon.length > 3 &&
			Math.abs(first.x - last.x) < this.conf.tolerance &&
			Math.abs(first.y - last.y) < this.conf.tolerance){
			polygon.pop();
		}

		// Keep the direction convention consistent with the current nesting core
		var area = this.polygonArea(polygon);
		if(area > 0){
			polygon.reverse();
		}

		return polygon;
	};

// Convert a piece (with or without holes) into the structure used by the nesting algorithm
	TxtParser.prototype.preparePiece = function(piece){
		if(!piece || !piece.outer || piece.outer.length < 3){
			throw new Error('Invalid piece: missing OUTER contour');
		}

		var outer = this.preparePolygon(piece.outer);
		outer.pieceId = piece.id;

		if(piece.holes && piece.holes.length > 0){
			outer.children = [];
			for(var i = 0; i < piece.holes.length; i++){
				if(piece.holes[i] && piece.holes[i].length >= 3){
					outer.children.push(this.preparePolygon(piece.holes[i]));
				}
			}

			if(outer.children.length === 0){
				delete outer.children;
			}
		}

		return outer;
	};

// Compute polygon area (used to determine direction)
	TxtParser.prototype.polygonArea = function(polygon){
		var area = 0;
		var n = polygon.length;

		for(var i = 0; i < n; i++){
			var j = (i + 1) % n;
			area += polygon[i].x * polygon[j].y;
			area -= polygon[j].x * polygon[i].y;
		}

		return area / 2;
	};

	root.TxtParser = TxtParser;

})(typeof window !== 'undefined' ? window : global);

// Node.js environment export
if(typeof module !== 'undefined' && module.exports){
	module.exports = global.TxtParser;
}
