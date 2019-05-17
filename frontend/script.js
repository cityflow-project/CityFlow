/**
 * Draw Road Network
 */
id = Math.random().toString(36).substring(2, 15);

LANE_BORDER_WIDTH = 1;
LANE_BORDER_COLOR = 0x000000;
LANE_INNER_COLOR = 0x808080;
LANE_DASH = 10;
LANE_GAP = 12;
TRAFFIC_LIGHT_WIDTH = 3;
MAX_TRAFFIC_LIGHT_NUM = 100000;
ROTATE = 90;

CAR_LENGTH = 5;
CAR_WIDTH = 2;
CAR_SCALE = 5;
CAR_COLOR = 0x0000FF;
NUM_CAR_POOL = 150000;

var simulation, roadnet, steps;
var nodes = {};
var edges = {};
var logs;
var gettingLog = false;

let Application = PIXI.Application,
    Sprite = PIXI.Sprite,
    Graphics = PIXI.Graphics,
    Container = PIXI.Container,
    ParticleContainer = PIXI.particles.ParticleContainer;

var trafficLightsG = {};

var nodeCanvas;
var app, renderer, simulatorContainer, carContainer, trafficLightContainer;
var carPool;

var cnt = 0;
var frameElapsed = 0;
var totalStep;

var nodeCarNum = document.getElementById("car-num");
var nodeProgressPercentage = document.getElementById("progress-percentage");
var nodeTotalStep = document.getElementById("total-step-num");
var nodeCurrentStep = document.getElementById("current-step-num");
var nodeSelectedEntity = document.getElementById("selected-entity");

var SPEED = 3, SCALE_SPEED = 1.01;
var LEFT = 37, UP = 38, RIGHT = 39, DOWN = 40;
var MINUS = 189, EQUAL = 187, P = 80;
var LEFT_BRACKET = 219, RIGHT_BRACKET = 221; 
var ONE = 49, TWO = 50;

var keyDown = new Set();
var paused = false;
var replaySpeed = 1;

document.addEventListener('keydown', function(e) {
    if (e.keyCode == P) {
        paused = !paused;
    } else if (e.keyCode == ONE) {
        replaySpeed *= 2;
    } else if (e.keyCode == TWO && replaySpeed >= 1.9999) { // TODO: fix
        replaySpeed /= 2;
    } else if (e.keyCode == LEFT_BRACKET) {
        cnt = (cnt - 1) % totalStep;
        cnt = (cnt + totalStep) % totalStep;
        drawStep(cnt);
    } else if (e.keyCode == RIGHT_BRACKET) {
        cnt = (cnt + 1) % totalStep;
        drawStep(cnt);
    } else {
        keyDown.add(e.keyCode)
    }
});
document.addEventListener('keyup', (e) => keyDown.delete(e.keyCode));

drawRoadnet = axios.get(roadnetFile).then(function(response) {
    appendText("info", "roadnet file loaded.");
    nodeCanvas = document.getElementById("simulator-canvas");

    app = new Application({
        width: nodeCanvas.offsetWidth,
        height: nodeCanvas.offsetHeight,
        transparent: 1
    });
    nodeCanvas.appendChild(app.view);
    renderer = app.renderer;
    renderer.autoResize = true;
    simulatorContainer = new Container();

    simulation = response.data;
    roadnet = simulation.static;

    for (let i = 0, len = roadnet.nodes.length;i < len;++i) {
        node = roadnet.nodes[i];
        node.point = new Point(transCoord(node.point));
        nodes[node.id] = node;
    }

    for (let i = 0, len = roadnet.edges.length;i < len;++i) {
        edge = roadnet.edges[i];
        edge.from = nodes[edge.from];
        edge.to = nodes[edge.to];
        for (let j = 0, len = edge.points.length;j < len;++j) {
            edge.points[j] = new Point(transCoord(edge.points[j]));
        }
        edges[edge.id] = edge;
    }

    /**
     * Draw Map
     */
    trafficLightContainer = new ParticleContainer(MAX_TRAFFIC_LIGHT_NUM, {tint: true});

    var mapGraphics = new Graphics();
    for (edgeId in edges) {
        drawEdge(edges[edgeId], mapGraphics);
    }

    simulatorContainer.addChild(mapGraphics);
    let bounds = simulatorContainer.getBounds();
    simulatorContainer.pivot.set(bounds.x + bounds.width / 2, bounds.y + bounds.height / 2);
    simulatorContainer.position.set(renderer.width / 2, renderer.height / 2);
    simulatorContainer.addChild(trafficLightContainer);

    /**
     * Settings for Cars
     */
    var carG = new Graphics();
    carG.lineStyle(0);
    carG.beginFill(CAR_COLOR);
    CAR_LENGTH *= CAR_SCALE;
    CAR_WIDTH *= CAR_SCALE;
    carG.moveTo(-CAR_LENGTH, -CAR_WIDTH/2);
    carG.lineTo(-CAR_LENGTH, CAR_WIDTH/2);
    carG.lineTo(0, 0);
    carG.endFill();
    carTexture = renderer.generateTexture(carG);

    carPool = [];
    carContainer = new ParticleContainer(NUM_CAR_POOL, {rotation: true});
    simulatorContainer.addChild(carContainer);

    for (let i = 0, len = NUM_CAR_POOL;i < len;++i) {
        //var car = Sprite.fromImage("images/car.png")
        var car = new Sprite(carTexture);
        car.anchor.set(1, 0.5);
        car.scale.set(1/CAR_SCALE, 1/CAR_SCALE);
        carPool.push(car);
    }

    return true;
});

function appendText(id, text) {
    let p = document.createElement("span");
    p.innerText = text;
    document.getElementById("info").appendChild(p);
    document.getElementById("info").appendChild(document.createElement("br"));
}

var statsFile = "";
var withRange = false;
var nodeStats, nodeRange;
if (logFile == "replay/www_replay_1_4_bad.txt") {
    statsFile = "replay/www_speed_bad.txt";
    nodeStats = document.getElementById("speed");
    document.getElementById("speed-tr").classList.remove("d-none");
} else if (logFile == "replay/www_replay_1_4_good.txt") {
    statsFile = "replay/www_speed_good.txt";
    nodeStats = document.getElementById("speed");
    document.getElementById("speed-tr").classList.remove("d-none");
} else if (logFile == "replay/www_replay_volume.txt") {
    statsFile = "replay/www_volume.txt";
    nodeStats = document.getElementById("stats");
    nodeRange = document.getElementById("range");
    withRange = true;
    document.getElementById("control-box").classList.remove("d-none");
    document.getElementById("stats-name").innerText = "West-East Volume"
} else if (logFile == "replay/www_replay_ratio.txt") {
    statsFile = "replay/www_ratio.txt";
    nodeStats = document.getElementById("stats");
    nodeRange = document.getElementById("range");
    withRange = true;
    document.getElementById("control-box").classList.remove("d-none");
    document.getElementById("stats-name").innerText = "Green Ratio"
}
if (statsFile != "") {
    var stats = [];
    console.log(statsFile);
    axios.get(statsFile).then(response => {
        console.log(response.data.length);
        data = response.data.split('\n');
        for (let i = 0;i < data.length;++i) {
            tmp = data[i].split(' ');
            if (withRange)
                stats.push([parseFloat(tmp[1]), parseFloat(tmp[2])]);
            else
                stats.push([parseFloat(tmp[1])]);
        }
        return true;
    })
}

Promise
    .all([
        drawRoadnet,
        axios.get(logFile).then(response => {
            logs = response.data.split('\n');
            logs.pop();
            totalStep = logs.length;
            return true;
        })
    ])
    .then(([rnSuccess, replaySuccess]) => {
        appendText("info", "simulation start!");

        nodeCanvas.removeChild(document.getElementById("spinner"));
        renderer.resize(nodeCanvas.offsetWidth, nodeCanvas.offsetHeight);
        app.stage.addChild(simulatorContainer);

        app.ticker.add(run);
    });

function transCoord(point) {
    return [point[0], -point[1]];
}

PIXI.Graphics.prototype.drawLine = function(pointA, pointB) {
    this.moveTo(pointA.x, pointA.y);
    this.lineTo(pointB.x, pointB.y);
}

PIXI.Graphics.prototype.drawDashLine = function(pointA, pointB, dash = 16, gap = 8) {
    let direct = pointA.directTo(pointB);
    let distance = pointA.distanceTo(pointB);

    let currentPoint = pointA;
    let currentDistance = 0;
    let length;
    let finish = false;
    while (true) {
        this.moveTo(currentPoint.x, currentPoint.y);
        if (currentDistance + dash >= distance) {
            length = distance - currentDistance;
            finish = true;
        } else {
            length = dash
        }
        currentPoint = currentPoint.moveAlong(direct, length);
        this.lineTo(currentPoint.x, currentPoint.y);
        if (finish) break;
        currentDistance += length;

        if (currentDistance + gap >= distance) {
            break;
        } else {
            currentPoint = currentPoint.moveAlong(direct, gap);
            currentDistance += gap;
        }
    }
};

function drawEdge(edge, graphics) {
    let from = edge.from;
    let to = edge.to;
    let points = edge.points;

    let pointA, pointAOffset, pointB, pointBOffset;
    let prevPointBOffset = null;
    for (let i = 1;i < points.length;++i) {
        if (i == 1){
            pointA = points[0].moveAlongDirectTo(points[1], from.virtual ? 0 : from.width);
            pointAOffset = points[0].directTo(points[1]).rotate(ROTATE);
        } else {
            pointA = points[i-1];
            pointAOffset = prevPointBOffset;
        }
        if (i == points.length - 1) {
            pointB = points[i].moveAlongDirectTo(points[i-1], to.virtual ? 0 : to.width);
            pointBOffset = points[i-1].directTo(points[i]).rotate(ROTATE);
        } else {
            pointB = points[i];
            pointBOffset = points[i-1].directTo(points[i+1]).rotate(ROTATE);
        }
        prevPointBOffset = pointBOffset;

        lightG = new Graphics();
        lightG.lineStyle(TRAFFIC_LIGHT_WIDTH, 0xFFFFFF);
        lightG.drawLine(new Point(0, 0), new Point(1, 0));
        lightTexture = renderer.generateTexture(lightG);

        // Draw Traffic Lights
        if (i == points.length-1 && !to.virtual) {
            edgeTrafficLights = [];
            prevOffset = offset = 0;
            for (lane = 0;lane < edge.nLane;++lane) {
                offset += edge.laneWidths[lane];
                var light = new Sprite(lightTexture);
                light.anchor.set(0, 0.5);
                light.scale.set(offset - prevOffset, 1);
                point_ = pointB.moveAlong(pointBOffset, prevOffset);
                light.position.set(point_.x, point_.y);
                light.rotation = pointBOffset.getAngleInRadians();
                edgeTrafficLights.push(light);
                prevOffset = offset;
                trafficLightContainer.addChild(light);
            }
            trafficLightsG[edge.id] = edgeTrafficLights;
        }

        // Draw Roads
        graphics.lineStyle(LANE_BORDER_WIDTH, LANE_BORDER_COLOR, 1);
        graphics.drawLine(pointA, pointB);

        offset = 0;
        for (let lane = 0, len = edge.nLane-1;lane < len;++lane) {
            offset += edge.laneWidths[lane];
            graphics.lineStyle(LANE_BORDER_WIDTH, LANE_INNER_COLOR);
            graphics.drawDashLine(pointA.moveAlong(pointAOffset, offset), pointB.moveAlong(pointBOffset, offset), LANE_DASH, LANE_GAP);
        }
        offset += edge.laneWidths[edge.nLane-1];
        graphics.lineStyle(LANE_BORDER_WIDTH, LANE_BORDER_COLOR);
        graphics.drawLine(pointA.moveAlong(pointAOffset, offset), pointB.moveAlong(pointBOffset, offset));
    }
}

function run(delta) {
    let redraw = false;
    if (keyDown.has(LEFT)) {
        simulatorContainer.pivot.x -= SPEED / simulatorContainer.scale.x;
        redraw = true;
    }
    if (keyDown.has(UP)) {
        simulatorContainer.pivot.y -= SPEED / simulatorContainer.scale.y;
        redraw = true;
    }
    if (keyDown.has(RIGHT)) {
        simulatorContainer.pivot.x += SPEED / simulatorContainer.scale.x;
        redraw = true;
    }
    if (keyDown.has(DOWN)) {
        simulatorContainer.pivot.y += SPEED / simulatorContainer.scale.y;
        redraw = true;
    }
    if (keyDown.has(MINUS)) {
        simulatorContainer.scale.x = (1/SCALE_SPEED) * simulatorContainer.scale.x;
        simulatorContainer.scale.y = (1/SCALE_SPEED) * simulatorContainer.scale.y;
        redraw = true;
    }
    if (keyDown.has(EQUAL)) {
        simulatorContainer.scale.x = SCALE_SPEED * simulatorContainer.scale.x;
        simulatorContainer.scale.y = SCALE_SPEED * simulatorContainer.scale.y;
        redraw = true;
    }

    if (!paused || redraw) {
        drawStep(cnt);
        if (!paused) {
            frameElapsed += 1;
            if (frameElapsed >= replaySpeed) {
                cnt += 1;
                frameElapsed = 0;
                if (cnt == totalStep) cnt = 0;
            }
        }
    }
}

function _statusToColor(status) {
    switch (status) {
        case 'r':
            return 0xFF0000;
        case 'g':
            return 0x00FF00;
        default:
            return 0x808080;  
    }
}

function drawStep(step) {
    let [carLogs, tlLogs] = logs[step].split(';');

    tlLogs = tlLogs.split(',')
    carLogs = carLogs.split(',')
    
    let tlLog, tlEdge, tlStatus;
    for (let i = 0, len = tlLogs.length;i < len;++i) {
        tlLog = tlLogs[i].split(' ');
        tlEdge = tlLog[0];
        tlStatus = tlLog.slice(1);
        for (let j = 0, len = tlStatus.length;j < len;++j) {
            trafficLightsG[tlEdge][j].tint = _statusToColor(tlStatus[j]);
        }
    }

    carContainer.removeChildren();
    let carLog, position;
    for (let i = 0, len = carLogs.length;i < len;++i) {
        carLog = carLogs[i].split(' ');
        position = transCoord([parseFloat(carLog[0]), parseFloat(carLog[1])]);
        carPool[i].position.set(position[0], position[1]);
        carPool[i].rotation = 2*Math.PI - parseFloat(carLog[2]);
        carContainer.addChild(carPool[i]);
    }
    nodeCarNum.innerText = carLogs.length-1;
    nodeTotalStep.innerText = totalStep;
    nodeCurrentStep.innerText = cnt+1;
    nodeProgressPercentage.innerText = (cnt / totalStep * 100).toFixed(2) + "%";
    if (statsFile != "") {
        if (withRange) nodeRange.value = stats[step][1];
        nodeStats.innerText = stats[step][0].toFixed(2);
    }
}