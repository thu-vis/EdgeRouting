"use strict";

var pathes = [
	[
		[60, 50],
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

let layout = WJL.createLayout();

var upperSvg = d3.select("#original");
var lowerSvg = d3.select("#routing");

var dragHandler = (function(){
	var x = 0.0, y = 0.0;
	var dx = 0.0, dy = 0.0;
	var drag = function(d){
		x = d3.mouse(this)[0];
		y = d3.mouse(this)[1];
		dx += d3.event.dx;
		dy += d3.event.dy;
	};
	var dragEnd = function(d) {
		//console.log("end!", d3.event.x, d3.event.y, dx, dy);
		var result = (dx**2 + dy**2 >= 100);
		if(result) {
			var removedPathes = d3.selectAll("path");
			removedPathes.remove();
			obstacles.push([[x-dx,y-dy],[x,y-dy],[x,y],[x-dx,y]]);
			redraw();
		}
		dx = 0.0, dy = 0.0;
		x = 0.0, y = 0.0;
		return result;
	}
	return {
		"drag" : drag,
		"dragend": dragEnd,
	};
})();

var dragBehavior = d3.behavior.drag()
					 .on("drag", dragHandler["drag"])
					 .on("dragend", dragHandler["dragend"]);
upperSvg.call(dragBehavior);

function redraw() {
	//console.log(obstacles);
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

	for (let i = 0; i < obstacles.length; i++) {
		layout.addPolygonObstacle(obstacles[i], i.toString());
	}
	for (let i = 0; i < pathes.length; i++) {
		let path = pathes[i];
		layout.addPathEndpoint([path[0][0], path[0][1]], (2 * i).toString());
		layout.addPathEndpoint([path[1][0], path[1][1]], (2 * i + 1).toString());
		layout.addPath((2 * i).toString(), (2 * i + 1).toString(), i.toString());
	};
	layout.build();
	for (let i = 0; i < pathes.length; i++) {
		let result = layout.polylinePath(i.toString());
		let pathString = "M" + result[0].toString();
		for (let i = 1; i < result.length; i++) {
			pathString += (" L" + result[i].toString());
		}
		upperSvg.append("path")
			.attr("d", pathString)
			.style("stroke", "red")
			.style("fill", "none");

		let curves = layout.curvePath(i.toString());
		//console.log("cur", curves);
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
