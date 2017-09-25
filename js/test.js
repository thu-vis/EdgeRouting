"use strict";

var pathes = [
	[
		[20, 50],
		[300, 100]
	],
	[
		[20, 60],
		[300, 90],
	],
];
var obstacles = [
	[
		[170, 70],
		[170, 140],
		[210, 140],
		[210, 70]
	],
	[
		[100, 30],
		[120, 45],
		[140, 30],
		[120, 90],
	],
	[
		[180, 60],
		[220, 105],
		[250, 70],
	],
	[
		[60, 40],
		[30, 60],
		[45, 80],
		[75, 80],
		[90, 60],
	]
];
var upperSvg = d3.select("#original");
var lowerSvg = d3.select("#routing");

function redraw() {
	for (let polygon of obstacles) {
		let pathString = "M ";
		for (let vertex of polygon) {
			pathString += (vertex[0] + "," + vertex[1] + " ");
		}
		upperSvg.append("path")
			.attr("d", pathString)
			.style("fill", "steelblue");
		lowerSvg.append("path")
			.attr("d", pathString)
			.style("fill", "steelblue");
	}

	for (let path of pathes) {
		let attrFrom = {
			"r": 5,
			"cx": path[0][0],
			"cy": path[0][1],
			"fill": "black",
		};
		let attrTo = {
			"r": 5,
			"cx": path[1][0],
			"cy": path[1][1],
			"fill": "black",
		}
		upperSvg.append("circle").attr(attrFrom);
		lowerSvg.append("circle").attr(attrFrom);
		upperSvg.append("circle").attr(attrTo);
		lowerSvg.append("circle").attr(attrTo);
	}

	let layout = WJL.createLayout();

	for (let polygon of obstacles) {
		layout.addPolygon(polygon);
	}
	for (let path of pathes) {
		layout.addPath({
			'from': [path[0][0], path[0][1]],
			'to': [path[1][0], path[1][1]],
		});
	};
	layout.build();
	//console.log(layout);
	for (let path of layout.pathes) {
		let result = layout.polylinePath(path);
		console.log("poly", result);

		let pathString = "M" + result[0].toString();
		for (let i = 1; i < result.length; i++) {
			pathString += (" L" + result[i].toString());
		}
		upperSvg.append("path")
			.attr("d", pathString)
			.style("stroke", "red")
			.style("fill", "none");

		let curves = layout.curvePath(path);
		console.log("cur", curves);
		var curveString = "M" + curves[0].from.toString();
		for (let i = 0; i < curves.length; i++) {
			curveString += (" C" + curves[i].toString());
		}

		lowerSvg.append("path")
			.attr("d", curveString)
			.style("stroke", "red")
			.style("fill", "none");
	}
}
redraw();
