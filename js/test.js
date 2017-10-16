"use strict";

var pathes = [
	[
		[60, 50],
		[300, 100]
	],
	[
		[20, 60],
		[300, 80],
	],
];
var obstacles_ori = [
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
	],
	[
		[165, 13],
		[183, 13],
		[183, 52],
		[165, 52],
	],
	[
		[89, 73],
		[93, 73],
		[93, 84],
		[89, 84],
	],
	[
		[95, 81],
		[105, 81],
		[105, 109],
		[95, 109],
	],
	[
		[66, 7],
		[85, 7],
		[85, 38],
		[66, 38],
	],
	[
		[269, 63],
		[273, 63],
		[273, 91],
		[269, 91],

	],
	[
		[101, 4],
		[119, 4],
		[119, 23],
		[101, 23],
	]
];
var obstacles = [];

var nDuplicates = 1;
for(let i = 0; i < nDuplicates; i++){
	obstacles = obstacles.concat(obstacles_ori);
}
var previousObstaclesLength = 0;
var previousPathesLength = 0;

var polygonId = 0;
var endPointId = 0;
var pathId = 0;

var layoutUpper = WJL.createLayout();
var layoutLower = WJL.createLayout();

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

var redrawPolygons = function(svg, layout) {
	var polygonElements = svg.selectAll("path[class=polygon]");
	var polygonUpdate = polygonElements.data(obstacles, x=>x);
	var polygonEnter = polygonUpdate.enter();
	var polygonExit = polygonUpdate.exit();

	var enterPolygonElements = polygonEnter.append("path")
		.attr("class", "polygon")
		.attr("d", function(data){
			let pathString = "M ";
			for(let vertex of data) {
				pathString += (vertex[0] + "," + vertex[1] + " ");
			}
			return pathString;
		})
		.style("fill", "steelblue");

	for(let i = previousObstaclesLength; i < obstacles.length; i++){
		layout.addPolygonObstacle(obstacles[i], i.toString());
	}
}

var redrawPathes = function(svg, layout, isCurved = true) {
	var pathElements = svg.selectAll("path[class=path]");
	var pathUpdate = pathElements.data(pathes);
	var pathEnter = pathUpdate.enter();
	var pathExit = pathUpdate.exit();

	var enterPathElements = pathEnter.append("circle")
		.attr("class", "path")
		.attr({
			"r": 5,
			"cx": function(data){
				return data[0][0];
			},
			"cy": function(data){
				return data[0][1];
			},
		})
		.style("fill", "black");
	pathEnter.append("circle")
		.attr("class", "path")
		.attr({
			"r": 5,
			"cx": function(data){
				return data[1][0];
			},
			"cy": function(data){
				return data[1][1];
			},
		})
		.style("fill", "black");

	for (let i = previousPathesLength; i < pathes.length; i++){
		let data = pathes[i];
		layout.addPathEndpoint([data[0][0], data[0][1]], endPointId.toString());
		endPointId += 1;
		layout.addPathEndpoint([data[1][0], data[1][1]], endPointId.toString());
		endPointId += 1;
		layout.addPath((endPointId - 2).toString(), (endPointId - 1).toString(), i.toString());
		data["id"] = i.toString();
	};
	layout.build();
	enterPathElements.each(function(data){
		if(isCurved) {
			let curves = layout.curvePath(data.id);
			let curveString = "M" + curves[0].from.toString();
			//console.log(curves);
			for(let i = 0; i < curves.length; i++) {
				curveString += (" C" + curves[i].toString());
			}

			svg.append("path")
				.attr("d", curveString)
				.style({
					"fill": "none",
					"stroke": "green",
				});
		}
		else{
			let points = layout.polylinePath(data.id);
			let pathString = "M" + points[0].toString();
			for(let i = 1; i < points.length; i++) {
				pathString += (" L" + points[i].toString());
			}
			svg.append("path")
				.attr("d", pathString)
				.style({
					"fill" : "none",
					"stroke": "green",
				});
		}
	});
}

var redraw = function(){
	redrawPolygons(upperSvg, layoutUpper);
	redrawPolygons(lowerSvg, layoutLower);
	redrawPathes(lowerSvg, layoutLower, true);
	redrawPathes(upperSvg, layoutUpper, false);
	previousPathesLength = pathes.length;
	previousObstaclesLength = obstacles.length;
}

redraw();