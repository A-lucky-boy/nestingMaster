// Usage: node nestingGenerator.js

const fs = require('fs');
const path = require('path');

class NestingGenerator {
    constructor() {
        this.sheetVertices = [];
        this.pieces = [];
        this.scale = 2;
        this.margin = 50;
        this.sheetStrokeWidth = 0.2;
        this.pieceStrokeWidth = 0.2;

        // Sheet truncation: when drawing the SVG, cut the sheet along "rightmost piece vertex + padding",
        // removing the empty strip on the right, avoiding a second trim and keeping the display compact
        this.truncatePadRatio = 0.02;   // padding = ratio of the rightmost piece x
        this.truncatePadMin = 3;        // padding lower bound (in coordinate units), ensuring small instances also have breathing room

        // Dark low-saturation palette
        this.darkColors = [
            '#1f2937', // dark gray-blue
            '#334155', // slate blue-gray
            '#374151', // dark gray
            '#2f4858', // dark blue-gray
            '#264653', // dark teal
            '#283618', // dark olive green
            '#3d405b', // dark blue-purple gray
            '#4a4e69', // dark gray-purple
            '#2b2d42', // dark navy gray
            '#5f0f40', // dark wine red
            '#4f5d75', // dark steel blue
            '#6b705c'  // dark gray-green
        ];
    }

    parseFile(content) {
        const lines = content.split('\n');
        let currentSection = 'none';
        let currentVertices = [];
        let isReadingVertices = false;
        let currentPiece = null;

        for (let i = 0; i < lines.length; i++) {
            const line = lines[i].trim();

            if (line.includes('Sheet outline:')) {
                currentSection = 'sheet';
                continue;
            }

            if (line.includes('Pieces on sheet 1:')) {
                currentSection = 'pieces';
                continue;
            }

            if (currentSection === 'pieces' && line.startsWith('Piece')) {
                if (currentPiece && currentVertices.length > 0) {
                    currentPiece.vertices = [...currentVertices];
                    this.pieces.push(currentPiece);
                }
                currentPiece = { id: null, vertices: [] };
                currentVertices = [];
                isReadingVertices = false;
                continue;
            }

            if (line.startsWith('ID:')) {
                if (currentPiece) {
                    currentPiece.id = line.split(':')[1].trim();
                }
                continue;
            }

            if (line.includes('VERTICES (X,Y)')) {
                isReadingVertices = true;
                continue;
            }

            if (isReadingVertices) {
                if (line === '' || line.startsWith('Piece') || line.includes('Sheet')) {
                    isReadingVertices = false;
                    if (currentSection === 'sheet' && currentVertices.length > 0) {
                        this.sheetVertices = [...currentVertices];
                        currentVertices = [];
                    } else if (currentPiece) {
                        currentPiece.vertices = [...currentVertices];
                        this.pieces.push(currentPiece);
                        currentPiece = null;
                        currentVertices = [];
                    }
                    if (line.startsWith('Piece') && !line.startsWith('Pieces on sheet')) {
                        currentPiece = { id: null, vertices: [] };
                        i--;
                    }
                    continue;
                }

                const coords = line.split(/\s+/).filter(x => x !== '');
                if (coords.length >= 2) {
                    const x = parseFloat(coords[0]);
                    const y = parseFloat(coords[1]);
                    if (!isNaN(x) && !isNaN(y)) {
                        currentVertices.push({ x, y });
                    }
                }
            }
        }

        if (currentSection === 'sheet' && currentVertices.length > 0) {
            this.sheetVertices = [...currentVertices];
        } else if (currentPiece && currentVertices.length > 0) {
            currentPiece.vertices = [...currentVertices];
            this.pieces.push(currentPiece);
        }
    }

    generatePathData(vertices) {
        if (vertices.length === 0) {
            return '';
        }

        let path = `M ${vertices[0].x * this.scale},${vertices[0].y * this.scale}`;
        for (let i = 1; i < vertices.length; i++) {
            path += ` L ${vertices[i].x * this.scale},${vertices[i].y * this.scale}`;
        }
        path += ' Z';
        return path;
    }

    // Sutherland–Hodgman: keep the side of the polygon where x <= clipX (left half-plane),
    // used to cut the sheet strip along the rightmost piece vertex
    clipPolygonToMaxX(polygon, clipX) {
        if (!polygon || polygon.length < 3) {
            return polygon || [];
        }

        const out = [];
        for (let i = 0; i < polygon.length; i++) {
            const cur = polygon[i];
            const next = polygon[(i + 1) % polygon.length];
            const curIn = cur.x <= clipX;
            const nextIn = next.x <= clipX;

            if (curIn) {
                // Current point is on the kept side; output the current point
                out.push({ x: cur.x, y: cur.y });
                // Current inside, next outside: crossing the boundary, insert the intersection
                if (!nextIn) {
                    const t = (clipX - cur.x) / (next.x - cur.x);
                    out.push({ x: clipX, y: cur.y + t * (next.y - cur.y) });
                }
            } else {
                // Current outside, next inside: from outside to inside, insert the intersection first
                if (nextIn) {
                    const t = (clipX - cur.x) / (next.x - cur.x);
                    out.push({ x: clipX, y: cur.y + t * (next.y - cur.y) });
                }
            }
        }

        // Deduplicate the coincident first/last points to avoid degeneracy
        if (out.length > 1) {
            const first = out[0];
            const last = out[out.length - 1];
            if (Math.abs(first.x - last.x) < 1e-9 && Math.abs(first.y - last.y) < 1e-9) {
                out.pop();
            }
        }

        return out;
    }

    // Compute the max x of all pieces' outer contours (rightmost piece vertex), used to determine the truncation position
    computePiecesMaxX() {
        let maxX = 0;
        let hasPiece = false;
        for (let i = 0; i < this.pieces.length; i++) {
            const vertices = this.pieces[i].vertices;
            if (!vertices || vertices.length === 0) {
                continue;
            }
            for (let j = 0; j < vertices.length; j++) {
                if (vertices[j].x > maxX) {
                    maxX = vertices[j].x;
                }
            }
            hasPiece = true;
        }
        return hasPiece ? maxX : 0;
    }

    generate() {
        // 1. Determine the sheet drawing shape: truncate the strip along "rightmost piece vertex + padding"
        let drawSheet = this.sheetVertices;

        const piecesMaxX = this.computePiecesMaxX();
        if (this.pieces.length > 0 && piecesMaxX > 0) {
            const pad = Math.max(piecesMaxX * this.truncatePadRatio, this.truncatePadMin);
            const clipX = piecesMaxX + pad;

            // Only clip when the truncation position is strictly smaller than the original sheet's right end (avoid invalid/degenerate cases)
            let sheetMaxX = 0;
            this.sheetVertices.forEach(v => {
                if (v.x > sheetMaxX) sheetMaxX = v.x;
            });

            if (clipX < sheetMaxX) {
                const clipped = this.clipPolygonToMaxX(this.sheetVertices, clipX);
                if (clipped.length >= 3) {
                    drawSheet = clipped;
                }
            }
        }

        // 2. Compute the canvas size (based on the actual range of the truncated sheet, not the original strip)
        let maxX = 0;
        let maxY = 0;

        drawSheet.forEach(v => {
            if (v.x > maxX) maxX = v.x;
            if (v.y > maxY) maxY = v.y;
        });

        // Fallback: use the piece range when the sheet is empty
        if (maxX === 0 || maxY === 0) {
            this.pieces.forEach(p => {
                if (!p.vertices) return;
                p.vertices.forEach(v => {
                    if (v.x > maxX) maxX = v.x;
                    if (v.y > maxY) maxY = v.y;
                });
            });
        }

        const imgWidth = maxX * this.scale + this.margin * 2;
        const imgHeight = maxY * this.scale + this.margin * 2;

        let img = `<?xml version="1.0" encoding="UTF-8"?>\n`;
        img += `<svg width="${imgWidth}" height="${imgHeight}" xmlns="http://www.w3.org/2000/svg">\n`;

        // Keep the background light, convenient for paper and document illustrations
        img += `  <rect width="${imgWidth}" height="${imgHeight}" fill="#f8fafc"/>\n`;

        // Change the sheet color to neutral gray (drawSheet is already truncated at the rightmost piece vertex)
        const sheetPath = this.generatePathData(drawSheet);
        img += `  <path d="${sheetPath}" fill="#d1d5db" stroke="#4b5563" stroke-width="${this.sheetStrokeWidth}" transform="translate(${this.margin},${this.margin})"/>\n`;

        for (let i = 0; i < this.pieces.length; i++) {
            const piece = this.pieces[i];
            if (!piece.vertices || piece.vertices.length === 0) {
                continue;
            }

            const pathData = this.generatePathData(piece.vertices);
            const color = this.darkColors[i % this.darkColors.length];
            const opacity = 0.9;

            img += `  <path d="${pathData}" fill="${color}" stroke="#111827" stroke-width="${this.pieceStrokeWidth}" opacity="${opacity}" transform="translate(${this.margin},${this.margin})"/>\n`;
        }

        img += `</svg>`;
        return img;
    }

    save(imageContent, filename) {
        fs.writeFileSync(filename, imageContent, 'utf8');
        console.log(`SVG file saved as: ${filename}`);
    }
}

async function main() {
    try {
        const filePath = path.join(__dirname, 'nestingResult.txt');
        if (!fs.existsSync(filePath)) {
            console.error('Error: file nestingResult.txt not found');
            return;
        }

        const content = fs.readFileSync(filePath, 'utf8');
        const generator = new NestingGenerator();

        console.log('Parsing nesting result...');
        generator.parseFile(content);

        console.log(`Sheet vertex count: ${generator.sheetVertices.length}`);
        console.log(`Piece count: ${generator.pieces.length}`);

        if (generator.sheetVertices.length === 0) {
            console.warn('Warning: no sheet outline parsed');
        }

        console.log('Generating SVG...');
        const imageContent = generator.generate();
        generator.save(imageContent, 'nestingResult.svg');

        console.log('Done!');

    } catch (error) {
        console.error('Error occurred:', error.message);
        console.error(error.stack);
    }
}

// Execute only when run directly; when required, only export the class
if (require.main === module) {
    main();
}

module.exports = { NestingGenerator };
