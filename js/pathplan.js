(function() {
	const infSmall = 1e-2;
	const visbilitySmall = 1;

	var less_than = (x, y) => x + infSmall < y;
	var equal = (x, y) => Math.abs(x - y) <= infSmall;
	var more_than = (x, y) => x - infSmall > y;
	var less_or_equal = (x, y) => less_than(x, y) || equal(x, y);
	var more_or_equal = (x, y) => more_than(x, y) || equal(x, y);
	var between = (a, p, q) => more_or_equal(a, p) && less_or_equal(a, q);

	function bezier_term(t, n) {
		switch (n) {
			case 0:
				return (1 - t) ** 3;
			case 1:
				return 3 * t * ((1 - t) ** 2);
			case 2:
				return 3 * (t ** 2) * (1 - t);
			case 3:
				return t ** 3;
		}
		throw RangeError();
	}

	class Vector {
		constructor(x, y) {
			this.x = x;
			this.y = y;
		}
		get length() {
			return Math.sqrt(this.x * this.x + this.y * this.y);
		}
		toString() {
			return this.x + "," + this.y;
		}
		scale(r) {
			return new Vector(this.x * r, this.y * r);
		}
		normalize() {
			var length = Math.sqrt(this.x ** 2 + this.y ** 2);
			if(equal(length, 0)){
				throw RangeError("Zero vector cannot be normalized.");
			}
			return new Vector(this.x / length, this.y / length);
		}
		add(other) {
			return new Vector(this.x + other.x, this.y + other.y);
		}
		sub(other) {
			return new Vector(this.x - other.x, this.y - other.y);
		}
		cross(other) {
			return this.x * other.y - other.x * this.y;
		}
		dot(other) {
			return this.x * other.x + this.y * other.y;
		}
		reverse() {
			return new Vector(-this.x, -this.y);
		}
		isZero() {
			return equal(this.x, 0) && equal(this.y, 0);
		}
	}

	class Segment {
		constructor(from, to) {
			this.from = from;
			this.to = to;
		}
		toVector() {
			return new Vector(this.to.x - this.from.x, this.to.y - this.from.y);
		}
		get length() {
			return euclid_distance(this.from, this.to);
		}
	}

	function bisector_between(A, B) {
		if (A.isZero() || B.isZero()) {
			throw new RangeError("Zero vector not allowed.");
		}
		A = A.normalize();
		B = B.normalize();
		var tmpx = B.y - A.y;
		var tmpy = A.x - B.x;

		if (equal(tmpx, 0) && equal(tmpy, 0)) {
			return new Vector(A.x, A.y);
		}
		var ret = new Vector(tmpx, tmpy);
		if (less_than(ret.dot(A), 0)) {
			ret = ret.reverse();
		}
		return ret.normalize();
	}

	function euclid_distance(A, B) {
		return Math.sqrt((A.x - B.x) ** 2 + (A.y - B.y) ** 2);
	}

	class Polyline {
		constructor(points) {
			this.points = points;
		}
		pointAt(n) {
			return this.points[n];
		}
		subPolyline(a, b = this.points.length) {
			return new Polyline(this.points.slice(a, b));
		}

		get pointSum() {
			return this.points.length;
		}
		get firstPoint() {
			return this.points[0];
		}
		get lastPoint() {
			return this.points[this.points.length - 1];
		}
		get direction() {
			return new Vector(this.lastPoint.x - this.firstPoint.x,
				this.lastPoint.y - this.firstPoint.y);
		}
	}
	class BezierCurve {
		constructor(from, ctrl_1, ctrl_2, to) {
			this.points = [from, ctrl_1, ctrl_2, to];
		}
		get from() {
			return this.points[0];
		}
		get to() {
			return this.points[3];
		}
		pointAt(t) {
			// 0 <= t <= 1
			var x = 0.0;
			var y = 0.0;
			for (let i = 0; i < 4; i++) {
				let term = bezier_term(t, i);
				x += this.points[i].x * term;
				y += this.points[i].y * term;
			}
			return new Vector(x, y);
		}
		toString() {
			var result = "";
			for (let i = 1; i < 4; i++) {
				result += this.points[i].toString();
				result += ' ';
			}
			return result;
		}
	}

	class AdjListPoint extends Vector {
		constructor(x, y) {
			super(x, y);
			this.head = null;
			this.shortest = Infinity;
			this.visited = false;
		}
	}

	class AdjListEdge extends Segment {
		constructor(from, to, v) {
			super(from, to);
			this.v = v;
			this.next = null;
		}
	}

	function point_on_segment(point, segment, proper = false) {
		var v1 = point.sub(segment.from);
		var v2 = point.sub(segment.to);
		var cross = v1.cross(v2);

		if (!equal(cross, 0)) {
			return false;
		}
		var dot = v1.dot(v2);
		return proper ? less_than(dot, 0) : less_or_equal(dot, 0);
	}

	function segments_intersected(AB, CD, proper=false) {
		if(less_than(Math.max(AB.from.x, AB.to.x), Math.min(CD.from.x, CD.to.x)))
			return false;
		if(less_than(Math.max(CD.from.x, CD.to.x), Math.min(AB.from.x, AB.to.x)))
			return false;
		if(less_than(Math.max(AB.from.y, AB.to.y), Math.min(CD.from.y, CD.to.y)))
			return false;
		if(less_than(Math.max(CD.from.y, CD.to.y), Math.min(AB.from.y, AB.to.y)))
			return false;

		var ab = AB.toVector();
		var cd = CD.toVector();
		var ac = CD.from.sub(AB.from);
		var bc = CD.from.sub(AB.to);
		var ad = CD.to.sub(AB.from);

		var cross1 = ac.cross(ab) * ad.cross(ab);
		if (less_or_equal(cross1, 0)) {
			var cross2 = ac.cross(cd) * bc.cross(cd);
			if (less_or_equal(cross2, 0)) {
				if(proper === false)
					return true;
				let flag = (equal(cross1, 0) || equal(cross2, 0));
				return !flag;
			}
		}
		return false
	}

	function segments_intersection_point(AB, CD) {
		let relation = segments_intersected(AB, CD);
		if(relation === false) {
			return null;
		}
		var x1 = AB.from.x,
			x2 = AB.to.x,
			x3 = CD.from.x,
			x4 = CD.to.x;
		var y1 = AB.from.y,
			y2 = AB.to.y,
			y3 = CD.from.y,
			y4 = CD.to.y;

		var b1 = (y2 - y1) * x1 + (x1 - x2) * y1;
		var b2 = (y4 - y3) * x3 + (x3 - x4) * y3;
		var d = (x2 - x1) * (y4 - y3) - (x4 - x3) * (y2 - y1);
		var d1 = b2 * (x2 - x1) - b1 * (x4 - x3);
		var d2 = b2 * (y2 - y1) - b1 * (y4 - y3);

		return new Vector(d1 / d, d2 / d);
	}

	function point_and_polygon(point, vertex, flag = false) {
		var pointSum = vertex.length;
		var nCrossings = 0;
		var xs = [];

		for (let i = 0; i < pointSum; i++) {
			let edge = new Segment(vertex[i], vertex[(i + 1) % pointSum]);
			if (point_on_segment(point, edge) && !flag) {
				return 'on';
			}

			let x1 = edge.from.x,
				y1 = edge.from.y,
				x2 = edge.to.x,
				y2 = edge.to.y;
			let x0 = 0.0,
				y0 = point.y;

			if (equal(y1, y2))
				continue;
			if (!between(y0, Math.min(y1, y2), Math.max(y1, y2)))
				continue;

			x0 = (y0 - y1) * (x2 - x1) / (y2 - y1) + x1;

			if (more_or_equal(point.x, x0))
				continue;

			let upward = more_than(y2, y1);
			if ((equal(y0, y1) && !upward) || (equal(y0, y2) && upward))
				continue;
			nCrossings += 1;
			xs.push(x0);
		}

		if (flag) {
			return xs;
		} else {
			return (nCrossings % 2 ? 'inside' : 'outside');
		}
	}


	function polygon_trianglulation(vertex, clockwise) {
		if (clockwise === undefined)
			clockwise = is_clockwise(vertex);

		var ret = [];

		var ears = [];
		var isConvex = [];
		var next = [];
		var previous = [];
		var pointSum = vertex.length;

		function update(index) {
			isConvex[index] = is_convex_point(vertex[index],
				vertex[previous[index]],
				vertex[next[index]],
				clockwise);
			if (isConvex[index] === true) {
				let tri = [vertex[index], vertex[previous[index]], vertex[next[index]]];
				let flag = true;
				for (let j = 0; j < pointSum; j++) {
					if (j === index || j === previous[index] || j === next[index])
						continue;
					if (point_and_polygon(vertex[j], tri) !== 'outside') {
						flag = false;
						break;
					}
				}
				if (flag)
					ears.push(index);
			}
		}

		for (let i = 0; i < pointSum; i++) {
			next[i] = (i + 1) % pointSum;
			previous[i] = (i - 1 + pointSum) % pointSum;
			update(i, clockwise);
		}
		var nPointsLeft = pointSum;
		while (nPointsLeft > 2) {
			let chosenEar = ears.pop();
			ret.push([vertex[chosenEar], vertex[next[chosenEar]]], vertex[previous[chosenEar]]);
			next[previous[chosenEar]] = next[chosenEar];
			previous[next[chosenEar]] = previous[chosenEar];

			for (let adj of[next[chosenEar], previous[chosenEar]]) {
				if (!isConvex[adj]) {
					update(adj, clockwise);
				}
			}
			nPointsLeft -= 1;
		}
		return ret;
	}
	function segment_and_polygon(segment, vertex) {
		var pointSum = vertex.length;

		for(let i = 0; i < pointSum; i++){
			let p1 = vertex[i];
			let p2 = vertex[(i + 1)%pointSum];
			let p3 = vertex[(i - 1 + pointSum) % pointSum];
			let seg = new Segment(p1, p2);
			let relation = segments_intersected(seg, segment, true);
			if(relation)
				return true;

			if (point_on_segment(p1, segment, true)) {
				let v1 = p2.sub(p1);
				let v2 = p3.sub(p1);
 				let v0 = segment.toVector();
 
 				if (less_than(v0.cross(v1) * v0.cross(v2), 0)) {
 					return 'cross';
 				}
 			}
		}
		return false;
		// the following code provides correctness but very slow.
		var intersections = [];
		for (let i = 0; i < pointSum; i++) {
			let p1 = vertex[i];
			let p2 = vertex[(i + 1) % pointSum];
			let seg = new Segment(p1, p2);

			let p0 = segments_intersection_point(seg, segment);
			if (p0 !== null) {
				intersections.push(p0);
			}
		}
		intersections.push(segment.from);
		intersections.push(segment.to);

		function cmp(a, b) {
			var tmp = a.x - b.x;
			return equal(tmp, 0) ? a.y - b.y : a.x - b.x;
		}

		intersections.sort(cmp);
		let hasInside = false,
			hasOutside = false;

		for (let i = 0; i < intersections.length - 1; i++) {
			let p1 = intersections[i];
			let p2 = intersections[i + 1];
			let p0 = new Vector((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);

			let relation = point_and_polygon(p0, vertex);
			if(relation === "inside")
				return true;
		}
		return false;;
	}

	function curve_and_polygon(curve, vertex) {
		if (equal(euclid_distance(curve.from, curve.points[1]), 0)) {
			if (equal(euclid_distance(curve.to, curve.points[2]), 0)) {
				return segment_and_polygon(new Segment(curve.from, curve.to), vertex);
			}
		}

		var nDivision = 20;
		var hasInside = false,
			hasOutside = false;
		for (let i = 0; i < nDivision; i++) {
			let t0 = i / nDivision;
			let t1 = (i + 1) / nDivision;
			let p0 = curve.pointAt(t0);
			let p1 = curve.pointAt(t1);

			let result = segment_and_polygon(new Segment(p0, p1), vertex);
			if(result === true)
				return true;
		}
		return false;
	}

	function guess_tnas(polylinePath) {
		var pointSum = polylinePath.pointSum;
		var tnas = [0];

		for (let i = 1; i < pointSum; i++) {
			tnas[i] = tnas[i - 1] + euclid_distance(polylinePath.pointAt(i),
				polylinePath.pointAt(i - 1));
		}
		for (let i = 1; i < pointSum; i++) {
			tnas[i] /= tnas[pointSum - 1];
		}

		return tnas;
	}

	function route_spline(polygons, polylinePath, slopes) {
		if (slopes === undefined) {
			slopes = [];

			slopes.push(polylinePath.direction);
			slopes.push(polylinePath.direction);
		}
		slopes[0] = slopes[0].normalize();
		slopes[1] = slopes[1].normalize();

		var tnas = guess_tnas(polylinePath);

		var alphas = fit_best_curve(polylinePath, slopes, tnas);
		var original_curve = make_bezier_curve(polylinePath.firstPoint,
			polylinePath.lastPoint,
			slopes,
			alphas);
		//console.log(original_curve);
		var curve = original_curve;

		var ratio = 0.75;

		while (Math.abs(alphas[0]) > visbilitySmall && Math.abs(alphas[1]) > visbilitySmall) {
			curve = make_bezier_curve(polylinePath.firstPoint,
				polylinePath.lastPoint,
				slopes,
				alphas);

			var intersected = polygons.some(function(x) {
				if(curve.points[0].belong === 'path') {
					let relation = point_and_polygon(curve.points[0], x);
					if(relation !== 'outside')
						return false;
				}
				if(curve.points[3].belong === 'path') {
					let relation = point_and_polygon(curve.points[3], x);
					if(relation !== 'outside')
						return false;
				}
				//console.log(curve_and_polygon(curve, x))
				return curve_and_polygon(curve, x);
				//return ["inside", "cross"].indexOf(curve_and_polygon(curve, x)) > -1;
			});

			if (!intersected) {
				return [curve];
			}
			alphas[0] *= ratio;
			alphas[1] *= ratio;
		}

		if (polylinePath.pointSum == 2) {
			return [curve];
		}

		var dis = 0.0;
		var choice = 1;
		var pointSum = polylinePath.pointSum;
		for (let i = 0; i < pointSum; i++) {
			let p = original_curve.pointAt(tnas[i]);
			let tmp = euclid_distance(p, polylinePath.pointAt(i));
			if (tmp > dis) {
				dis = tmp;
				choice = i;
			}
		}

		var polylineLeft = polylinePath.subPolyline(0, choice + 1);
		var polylineRight = polylinePath.subPolyline(choice);


		var v1 = polylinePath.pointAt(choice).sub(polylinePath.pointAt(choice - 1));
		var v2 = polylinePath.pointAt(choice + 1).sub(polylinePath.pointAt(choice));
		var bisector = bisector_between(v1, v2);

		var curveLeft = route_spline(polygons, polylineLeft, [slopes[0], bisector]);
		var curveRight = route_spline(polygons, polylineRight, [bisector, slopes[1]]);

		var ret = curveLeft.concat(curveRight);
		return ret;
	}

	function make_bezier_curve(from, to, slopes, alphas = [1, -1]) {
		var ctrl_1 = from.add(slopes[0].scale(alphas[0]));
		var ctrl_2 = to.add(slopes[1].scale(alphas[1]));
		return new BezierCurve(from, ctrl_1, ctrl_2, to);
	}

	function fit_best_curve(polylinePath, slopes, tnas) {
		// make sure the slopes are UNIT VECTORS

		var pointSum = polylinePath.pointSum;

		// give an initial guess to U_i
		if (tnas == undefined) {
			tnas = guess_tnas(polylinePath);
		}

		// calculate A_i1, A_i2
		// --------- A_i1 = t_1 * B1(U_i)
		var Ai1 = [];
		var Ai2 = [];
		for (let i = 0; i < pointSum; i++) {
			Ai1[i] = slopes[0].scale(bezier_term(tnas[i], 1));
			Ai2[i] = slopes[1].scale(bezier_term(tnas[i], 2));
		}

		// parameter matrix C, constant vector X
		var c = [
			[0, 0],
			[0, 0]
		];
		var x = [0, 0];
		for (let i = 0; i < pointSum; i++) {
			c[0][0] += Ai1[i].dot(Ai1[i]);
			c[0][1] += Ai1[i].dot(Ai2[i]);
			c[1][1] += Ai2[i].dot(Ai2[i]);

			let tmp1 = polylinePath.firstPoint.scale(
				bezier_term(tnas[i], 0) + bezier_term(tnas[i], 1));
			let tmp2 = polylinePath.lastPoint.scale(
				bezier_term(tnas[i], 2), +bezier_term(tnas[i], 3));
			let tmp = polylinePath.pointAt(i).sub(tmp1.add(tmp2));
			x[0] += tmp.dot(Ai1[i]);
			x[1] += tmp.dot(Ai2[i]);
		}
		c[1][0] = c[0][1];

		//using Cramar's Law to solve the equation.
		var detC = c[0][0] * c[1][1] - c[0][1] * c[1][0];
		var detX0 = x[0] * c[1][1] - c[0][1] * x[1];
		var detX1 = c[0][0] * x[1] - c[1][0] * x[0];

		var alpha1 = 0.0;
		var alpha2 = 0.0;
		if (Math.abs(detC) > infSmall) {
			alpha1 = detX0 / detC;
			alpha2 = detX1 / detC;
		}

		if (Math.abs(detC) <= infSmall ||
			alpha1 <= -infSmall ||
			alpha2 >= infSmall) {
			let tmp = euclid_distance(polylinePath.firstPoint, polylinePath.lastPoint);
			alpha1 = tmp / 3.0;
			alpha2 = -tmp / 3.0;
		}
		return [alpha1, alpha2];
	}

	class AdjList {
		constructor() {
			this.points = [];
			this.edges = [];
		}
		addPoint(newPoint) {
			this.points.push(newPoint);
		}
		addEdge(from, to, v) {
			if (!v) {
				v = Math.sqrt((from.x - to.x) ** 2 + (from.y - to.y) ** 2);
			}
			var newEdge = new AdjListEdge(from, to, v);
			newEdge.next = from.head;
			from.head = newEdge;
			this.edges.push(newEdge);
			return newEdge;
		}
		dijkstra(from, to) {
			var nAdded = 1;
			var shortestPath = [];

			for (let point of this.points) {
				point.shortest = Infinity;
				point.visited = false;
			}
			from.shortest = 0;
			from.visited = true;
			for (let edge of this.edges) {
				if (edge.from === from) {
					edge.to.shortest = Math.min(edge.to.shortest, edge.v);
				}
			}

			while (nAdded < this.points.length) {
				let temp = Infinity;
				let choice = null;
				for (let point of this.points) {
					if(point['belong'] === 'path') {
						if(point !== from && point !== to){
							continue;
						}
					}
					if (!point.visited && point.shortest < temp) {
						temp = point.shortest;
						choice = point;
					}
				}
				if (!choice) {
					break;
				}
				choice.visited = true;
				for (let edge = choice.head; edge; edge = edge.next) {
					if(edge.to['belong'] === 'path') {
						if(edge.to !== from && edge.to !== to){
							continue;
						}
					}
					edge.to.shortest = Math.min(edge.to.shortest,
						choice.shortest + edge.v);
				}
				nAdded += 1;
			}
			if (to.shortest === Infinity) {
				return null;
			}

			var currentPos = to;
			shortestPath.push(to);

			while (currentPos != from) {
				for (let edge = currentPos.head; edge; edge = edge.next) {
					if (edge.to.shortest + edge.v === currentPos.shortest) {
						shortestPath.push(edge.to);
						currentPos = edge.to;
						break;
					}
				}
			}

			return {
				'length': to.shortest,
				'path': shortestPath.reverse(),
			}
		}
	}

	class VisibilityGraph extends AdjList {
		constructor() {
			super();
			this.pathes = [];
			this.polygons = [];
			this.endpointMap = new Map();
			this.polygonMap = new Map();
			this.pathMap = new Map();
		}
		_addPath(from, to) {
			var newPath = {
				'from': from,
				'to': to,
			}
			this.pathes.push(newPath);
			from['belong'] = 'path';
			to['belong'] = 'path';
			return newPath;
		}
		_addPolygon(points) {
			for (let point of points) {
				this.addPoint(point);
				point['belong'] = 'polygon';
			}
			this.polygons.push(points);
		}
		addPathEndpoint(coord, id) {
			if(id === undefined) {
				throw new TypeError("must have an id");
			}

			var newPoint = new Vector(coord[0], coord[1]);
			this.addPoint(newPoint);
			this.endpointMap.set(id, newPoint);
		}
		addPath(start, end, id) {
			if(id === undefined) {
				throw new TypeError("must have an id");
			}

			var p1 = this.endpointMap.get(start);
			var p2 = this.endpointMap.get(end);
			var path = this._addPath(p1, p2);

			this.pathMap.set(id, path);
		}
		addPolygonObstacle(coords, id) {
			if(id === undefined) {
				throw new TypeError("must have an id");
			}

			var points = [];
			for(let coord of coords) {
				points.push(new Vector(coord[0], coord[1]));
			}

			this._addPolygon(points);
			this.polygonMap.set(id, points);
		}
		build() {
			this.edges = [];
			for(let point of this.points) {
				point.head = undefined;
				point.through = [];
				point.in = [];

				if(point['belong'] === 'path'){
					for(let polygon of this.polygons){
						let r = point_and_polygon(point, polygon);
						if(r !== 'outside')
							point.in.push(polygon);
					}
				}
			}
			for (let i = 0; i < this.points.length; i++) {
				let p1 = this.points[i];
				for (let j = i + 1; j < this.points.length; j++) {
					let p2 = this.points[j];

					if(p1['belong'] === 'path' && p2['belong'] === 'path'){
						continue;
					}

					let segment = new Segment(p1, p2);
					let flag = this.polygons.some(function(polygon) {
						if(p1['belong'] === 'path') {
							if(p1['in'].indexOf(polygon) > -1)
								return false;
						}
						if(p2['belong'] === 'path') {
							if(p2['in'].indexOf(polygon) >-1)
								return false;
						}
						let relation = segment_and_polygon(segment, polygon);
						return relation;
					});

					if (!flag) {
						this.addEdge(p1, p2);
						this.addEdge(p2, p1);
					}
				}
			}
		}
		polylinePath(id, real = true,  epsilon = 10) {
			var path = this.pathMap.get(id);
			var from = path.from,
				to = path.to;
			var result = this.dijkstra(from, to);

			if (result !== null) {
				result = result['path'];
			} else {
				return [from, to];
			}
			if (real) {
				for (let point of result) {
					if (!point['through']) {
						point['through'] = [path];
					} else {
						point['through'].push(path);
					}
				}
			}
			for (let i = 1; i < result.length - 1; i++) {
				if (result[i]['through'].length <= 1)
					continue;
				for (let polygon of this.polygons) {
					let index = polygon.indexOf(result[i]);
					if (index > -1) {
						let p0 = result[i];
						let p1 = polygon[(index + 1) % polygon.length];
						let p2 = polygon[(index - 1 + polygon.length) % polygon.length];

						let v1 = p1.sub(p0);
						let v2 = p2.sub(p0);
						let bi = bisector_between(v1, v2);
						let tmp = p0.add(bi);

						let relation = point_and_polygon(tmp, polygon);
						bi = (relation === "outside") ? bi : bi.reverse();

						let nScale = Math.min(result[i]['through'].length - 1, 4);
						let adjust = p0.add(bi.scale(epsilon * nScale));

						result.splice(i, 1, adjust);
						break;
					}
				}
			}
			return result;
		}

		curvePath(id, real = true) {
			var polylinePath = this.polylinePath(id, real);
			var curves = route_spline(this.polygons, new Polyline(polylinePath));
			return curves;
		}
	}
	window.Pathplan = {};
	window.Pathplan['createLayout'] = function() {
		return new VisibilityGraph();
	}
})();