/* ######################################################################
Modified by Taha Babzadeh, email : taha.babaw@gmail.com
Enhanced Ring Road Traffic Simulation with RL Control
Cleaned and optimized version for stop-and-go traffic research

Copyright (C) 2024 Martin Treiber
Enhanced for research purposes
#######################################################################*/

//#############################################################
// Core Constants and Configuration
//#############################################################

// RL Agent and Control Variables
let rlAgent;
let lastState = null;
let lastAction = 0;
let useIDM = false; // Hybrid control flag
let isRecording = false;
let recordingData = [];
let targetVehicleId = null;
let vehicleLoopCounts = new Map();
let lastPositions = new Map();

// Simulation Constants
const REFSIZE = 300;
const REFSIZE_SMARTPHONE = 200;

// Vehicle tracking for stop-and-go analysis
let veh200Distance = 0;
let lastVeh200Pos = null;

//#############################################################
// Initialization
//#############################################################

window.onload = () => {
    rlAgent = new DQNAgent(3, [-1.5, 0, 1.5]);
    rlAgent.loadModel().then(() => {
        updateModelSizeDisplay();
    });
    
    // Update model size display periodically
    setInterval(updateModelSizeDisplay, 1000);
    
    // Auto-save model every 60 seconds
    setInterval(() => {
        if (rlAgent) rlAgent.saveModel();
    }, 60000);
};

//#############################################################
// UI Settings and Debug Configuration
//#############################################################

const userCanDropObjects = true;
var showCoords = true;

//#############################################################
// Traffic Parameters
//#############################################################

var driver_varcoeff = 0.15; // Driver variability coefficient

// Override default density settings for stop-and-go research
density = 0.03;
setSlider(slider_density, slider_densityVal, 1000 * density, 0, "veh/km");

fracTruck = 0.1;
setSlider(slider_fracTruck, slider_fracTruckVal, 100 * fracTruck, 0, "%");
fracTruckToleratedMismatch = 0.02;

//#############################################################
// Scenario Setup and Graphics
//#############################################################

var scenarioString = "Ring";
console.log("Initializing Ring Road Simulation for Stop-and-Go Research");

// Canvas setup
var simDivWindow = document.getElementById("contents");
var canvas = document.getElementById("canvas");
var ctx = canvas.getContext("2d");
canvas.width = simDivWindow.clientWidth;
canvas.height = simDivWindow.clientHeight;
var aspectRatio = canvas.width / canvas.height;

console.log("Adding touch listeners...");
addTouchListeners();

//#############################################################
// Scaling and Responsive Design
//#############################################################

var isSmartphone = mqSmartphone();
var refSizePhys = isSmartphone ? REFSIZE_SMARTPHONE : REFSIZE;
var critAspectRatio = 120. / 95.;
var refSizePix = Math.min(canvas.height, canvas.width / critAspectRatio);
var scale = refSizePix / refSizePhys;

//#############################################################
// Physical Road Geometry
//#############################################################

var center_xRel = 0.5;
var center_yRel = -0.54;
var roadRadiusRel = 0.42;

// Physical geometry settings [m]
var center_xPhys = center_xRel * refSizePhys;
var center_yPhys = center_yRel * refSizePhys;
var roadRadius = roadRadiusRel * refSizePhys;
var mainroadLen = roadRadius * 2 * Math.PI;

function updateDimensions() {
    refSizePhys = isSmartphone ? REFSIZE_SMARTPHONE : REFSIZE;
    center_xPhys = center_xRel * refSizePhys;
    center_yPhys = center_yRel * refSizePhys;
    roadRadius = roadRadiusRel * refSizePhys;
    mainroadLen = roadRadius * 2 * Math.PI;
}

// Vehicle and road dimensions
var laneWidth = 8;
var nLanes_main = 1;
var car_length = 7;
var car_width = 6;
var truck_length = 15;
var truck_width = 7;

// Road trajectory functions
function trajIn_x(u) {
    return center_xPhys + roadRadius * Math.cos(u / roadRadius);
}

function trajIn_y(u) {
    return center_yPhys + roadRadius * Math.sin(u / roadRadius);
}

var trajIn = [trajIn_x, trajIn_y];

//#############################################################
// Road Network Setup
//#############################################################

var isRing = true;
var roadID = 1;
var speedInit = 20;
var mainroad = new road(roadID, mainroadLen, laneWidth, nLanes_main, trajIn,
    density, speedInit, fracTruck, isRing);

mainroad.setDriverVariation(driver_varcoeff);
network[0] = mainroad;
network[0].drawVehIDs = drawVehIDs;

// Traffic detectors for flow analysis
var detectors = [];
for (var idet = 0; idet < 4; idet++) {
    detectors[idet] = new stationaryDetector(mainroad,
        (0.125 + idet * 0.25) * mainroadLen, 10);
}

//#############################################################
// Model Initialization
//#############################################################

updateModels(); // Initialize car/truck models and lane-changing behavior

//#############################################################
// Graphics Configuration
//#############################################################

var hasChanged = true;
var drawBackground = true;
var drawRoad = true;
var userCanvasManip = false;
var drawColormap = false;
var vmin_col = 0;
var vmax_col = 100 / 3.6;

//#############################################################
// Image Loading
//#############################################################

// Background
var background = new Image();
background.src = 'figs/backgroundGrass.jpg';

// Vehicles
carImg = new Image();
carImg.src = 'figs/blackCarCropped.gif';
truckImg = new Image();
truckImg.src = 'figs/truck1Small.png';

// Traffic lights
traffLightRedImg = new Image();
traffLightRedImg.src = 'figs/trafficLightRed_affine.png';
traffLightGreenImg = new Image();
traffLightGreenImg.src = 'figs/trafficLightGreen_affine.png';

// Obstacles
obstacleImgNames = [];
obstacleImgs = [];
for (var i = 0; i < 10; i++) {
    obstacleImgs[i] = new Image();
    obstacleImgs[i].src = (i == 0)
        ? "figs/obstacleImg.png"
        : "figs/constructionVeh" + (i) + ".png";
    obstacleImgNames[i] = obstacleImgs[i].src;
}

// Road images
roadImgs1 = [];
roadImgs2 = [];

for (var i = 0; i < 4; i++) {
    roadImgs1[i] = new Image();
    roadImgs1[i].src = "figs/road" + (i + 1) + "lanesCropWith.png"
    roadImgs2[i] = new Image();
    roadImgs2[i].src = "figs/road" + (i + 1) + "lanesCropWithout.png"
}

roadImg1 = new Image();
roadImg1 = roadImgs1[nLanes_main - 1];
roadImg2 = new Image();
roadImg2 = roadImgs2[nLanes_main - 1];

//#############################################################
// Traffic Objects and Control
//#############################################################

var trafficObjs = new TrafficObjects(canvas, 2, 2, 0.40, 0.50, 3, 2);
var trafficLightControl = new TrafficLightControlEditor(trafficObjs, 0.33, 0.68);

//#############################################################
// Simulation Runtime Variables
//#############################################################

var time = 0;
var itime = 0;
var fps = 30;
var dt = timewarp / fps;

//#############################################################
// Vehicle Loop Tracking Functions
//#############################################################

function updateLoopCount(vehicleId, currentPosition) {
    const lastPos = lastPositions.get(vehicleId) || 0;
    const currentLoops = vehicleLoopCounts.get(vehicleId) || 0;

    // Detect loop completion (position reset from high to low)
    if (lastPos > mainroadLen * 0.8 && currentPosition < mainroadLen * 0.2) {
        vehicleLoopCounts.set(vehicleId, currentLoops + 1);
        console.log(`Vehicle ${vehicleId} completed loop ${currentLoops + 1}`);
    }

    lastPositions.set(vehicleId, currentPosition);
}

function getContinuousPosition(vehicleId, currentPosition) {
    const loops = vehicleLoopCounts.get(vehicleId) || 0;
    return loops * mainroadLen + currentPosition;
}

//#############################################################
// Core Simulation Update Function
//#############################################################

function updateSim() {
    // Update time
    time += dt;
    itime++;

    // Update geometry for responsive design
    isSmartphone = mqSmartphone();

    // Update vehicle models and road properties
    mainroad.updateTruckFrac(fracTruck, fracTruckToleratedMismatch);
    mainroad.updateModelsOfAllVehicles(longModelCar, longModelTruck,
        LCModelCar, LCModelTruck, LCModelMandatory);
    mainroad.updateDensity(density);
    mainroad.updateSpeedlimits(trafficObjs);

    // Core vehicle dynamics update
    mainroad.updateLastLCtimes(dt);
    mainroad.calcAccelerations();

    // RL Control Logic for Stop-and-Go Analysis
    let veh200 = mainroad.veh.find(v => v.id === 200);
    let veh222 = mainroad.veh.find(v => v.id === 222);
    let veh201 = mainroad.veh.find(v => v.id === 201);

    if (veh200 && veh222) {
        let dx = veh222.u - veh200.u;
        if (dx < 0) dx += mainroadLen;
        let gap = dx - veh222.length;
    if (veh201 && rlAgent && itime > 10) {
        applyRLControl(veh200, veh201, veh222);
        }

    }

    // Update vehicle positions and lane changes
    mainroad.changeLanes();
    mainroad.updateSpeedPositions();

    // Update UI displays
    if (veh200 && veh222) {
        let dx = veh222.u - veh200.u;
        if (dx < 0) dx += mainroadLen;
        const gap = dx - 7; // assuming 7m car length
        const laps = vehicleLoopCounts.get(200) || 0;
        updateGapAndLapUI(gap, laps);
    }

    // Update vehicle list periodically
    if (itime % 30 === 0) {
        updateVehicleList();
    }

    // Record vehicle data for analysis
    recordVehicleData();

    // Update detectors
    for (var iDet = 0; iDet < detectors.length; iDet++) {
        detectors[iDet].update(time, dt);
    }

    // Zoom back traffic objects if needed
    if (userCanDropObjects && (!isSmartphone) && (!trafficObjPicked)) {
        trafficObjs.zoomBack();
    }
}

//#############################################################
// Control Algorithm Functions
//#############################################################

function applyIDMControl(veh200, veh222) {
    const s = veh222.u - veh200.u;
    const gap = (s < 0 ? s + mainroadLen : s) - veh222.length;
    const deltaV = veh200.speed - veh222.speed;

    // IDM parameters
    const v0 = 30; // desired speed
    const T = 1.5; // time headway
    const a = 1.5; // max acceleration
    const b = 2.0; // comfortable deceleration

    const desiredGap = 2 + veh200.speed * T + (veh200.speed * deltaV) / (2 * Math.sqrt(a * b));
    const accel = a * (1 - Math.pow(veh200.speed / v0, 4) - Math.pow(desiredGap / gap, 2));

    veh200.acc = Math.max(-2, Math.min(1.5, accel));
    veh200.speed += veh200.acc * dt;
    veh200.speed = Math.max(0, veh200.speed);
}

function applyRLControl(veh200, veh201, veh222) {
    // State: [distance to vehicle behind, relative velocity with vehicle behind]
    let dx222 = veh222.u - veh200.u;
    if (dx222 < 0) dx222 += mainroadLen;
    let gapTo222 = dx222 - veh222.length;
    const s_behind = veh200.u - veh201.u;
    const rel_v_behind = veh201.speed - veh200.speed;
    const adjustedDistBehind = s_behind < 0 ? s_behind + mainroadLen : s_behind;
    const state = [adjustedDistBehind, rel_v_behind, gapTo222];

    rlAgent.act(state).then(action => {
        const accel = Math.max(-3, Math.min(2, action));
        veh200.speed += accel * dt;
        veh200.speed = Math.max(0, veh200.speed);
        veh200.acc = accel;

        // Calculate reward for RL training
        if (lastState) {
            const reward = calculateRLReward(veh200, veh222);
            rlAgent.remember(lastState, lastAction, reward, state, false);
            rlAgent.replay();
        }

        lastState = state;
        lastAction = action;
    });
}

function calculateRLReward(veh200, veh222) {
    let reward = 0;
    let collisionPenalty = 0;
    let smoothnessPenalty = 0;
    // Penalty for unsafe or unproductive gaps
    let unsafeGapPenalty = 0;
    if ((gap > 500 && gap < 850) || (gap >= -2 && gap <= 4)) {
        unsafeGapPenalty = 5.0;
    }
    reward -= unsafeGapPenalty;

    // Safety penalty for gap violations
    let dx = veh222.u - veh200.u;
    if (dx < 0) dx += mainroadLen;
    let gap = dx - 7;
    if (gap < 0) {
        collisionPenalty += 5.0; // Penalize overlap
    }

    // Smoothness penalty for harsh acceleration
    if (Math.abs(veh200.acc) > 2) {
        smoothnessPenalty = 1.0;
    }

    // Flow optimization: Analyze vehicles behind veh200
    let behindVehicles = mainroad.veh.filter(v => {
        let du = veh200.u - v.u;
        if (du < 0) du += mainroadLen;
        return du > 0 && du < 100; // vehicles within 100m behind
    });

    let speeds = behindVehicles.map(v => v.speed);
    let meanSpeed = speeds.length ? speeds.reduce((a, b) => a + b, 0) / speeds.length : 0;
    let variance = speeds.length ? speeds.reduce((sum, v) => sum + Math.pow(v - meanSpeed, 2), 0) / speeds.length : 0;
    let stddev = Math.sqrt(variance);

    // Reward: high mean speed, low speed variance, safety, smoothness
    reward = meanSpeed - stddev - 2 * collisionPenalty - smoothnessPenalty - unsafeGapPenalty;
    return reward;
}

//#############################################################
// Data Recording Functions
//#############################################################

function recordVehicleData() {
    if (isRecording && targetVehicleId !== null) {
        const v = mainroad.veh.find(v => v.id === targetVehicleId);
        if (v) {
            updateLoopCount(v.id, v.u);
            const continuousPosition = getContinuousPosition(v.id, v.u);

            recordingData.push({
                time: time.toFixed(2),
                id: v.id,
                position: continuousPosition.toFixed(2),
                rawPosition: v.u.toFixed(2),
                loops: vehicleLoopCounts.get(v.id) || 0,
                speed: v.speed.toFixed(2),
                acceleration: v.acc.toFixed(2)
            });
        }
    }
}

//#############################################################
// Drawing Function
//#############################################################

function drawSim() {
    // Handle responsive design
    var relTextsize_vmin = isSmartphone ? 0.03 : 0.02;
    var textsize = relTextsize_vmin * Math.min(canvas.width, canvas.height);

    // Update canvas dimensions if changed
    if ((canvas.width != simDivWindow.clientWidth) ||
        (canvas.height != simDivWindow.clientHeight)) {
        hasChanged = true;
        canvas.width = simDivWindow.clientWidth;
        canvas.height = simDivWindow.clientHeight;
        aspectRatio = canvas.width / canvas.height;
        refSizePix = Math.min(canvas.height, canvas.width / critAspectRatio);
        scale = refSizePix / refSizePhys;

        updateDimensions();
        trafficObjs.calcDepotPositions(canvas);
    }

    // Reset transform and draw background
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    if (drawBackground) {
        if (hasChanged || (itime <= 10) || (itime % 50 == 0) || userCanvasManip
            || (!drawRoad) || drawVehIDs) {
            ctx.drawImage(background, 0, 0, canvas.width, canvas.height);
        }
    }

    // Draw road
    var changedGeometry = userCanvasManip || hasChanged || (itime <= 1);
    mainroad.draw(roadImg1, roadImg2, changedGeometry);
    if (drawRoadIDs) { mainroad.drawRoadID(); }

    // Draw vehicles
    mainroad.drawVehicles(carImg, truckImg, obstacleImgs, vmin_col, vmax_col);

    // Draw traffic objects
    if (userCanDropObjects && (!isSmartphone)) {
        trafficObjs.draw();
    }

    // Draw UI elements
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    drawSpeedlBox();

    // Display time and detector information
    displayTime(time, textsize);
    for (var iDet = 0; iDet < detectors.length; iDet++) {
        detectors[iDet].display(textsize);
    }

    // Show coordinates if activated
    if (showCoords && mouseInside) {
        showLogicalCoords(xPixUser, yPixUser);
    }

    // Draw traffic light editor if active
    if (trafficLightControl.isActive) {
        trafficLightControl.showEditPanel();
    }

    hasChanged = false;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
}

//#############################################################
// UI Update Functions
//#############################################################

function updateModelIndicator() {
    const el = document.getElementById("modelIndicator");
    if (el) {
        el.textContent = `Controller: ${useIDM ? 'IDM' : 'RL'}`;
        el.style.color = useIDM ? 'green' : 'blue';
    }
}

function updateGapAndLapUI(gap, laps) {
    const gapEl = document.getElementById("gapInfo");
    const lapEl = document.getElementById("lapInfo");
    if (gapEl) {
        gapEl.textContent = `Gap to veh 222: ${gap.toFixed(2)} m`;
    }
    if (lapEl) {
        lapEl.textContent = `veh 200 Laps: ${laps}`;
    }
}

function updateVehicleList() {
    const container = document.getElementById("vehicleIds");
    if (!container) return;

    const ids = mainroad.veh.map(v => v.id).sort((a, b) => a - b);
    if (ids.length === 0) {
        container.innerHTML = "No vehicles";
        return;
    }

    container.innerHTML = ids.map(id =>
        `<span class="vehicle-id">${id}</span>`
    ).join('');
}

//#############################################################
// Recording Control Functions
//#############################################################

function toggleRecording() {
    const input = document.getElementById("targetVehicleId");
    let id = parseInt(input.value);

    if (isNaN(id)) {
        id = 200;
        console.log("No vehicle ID entered. Defaulting to 200.");
    }

    if (!isRecording) {
        const exists = mainroad.veh.some(v => v.id === id);
        if (!exists) {
            alert(`Vehicle ID ${id} not found.`);
            return;
        }

        targetVehicleId = id;
        recordingData = [];

        // Reset tracking data
        vehicleLoopCounts.clear();
        lastPositions.clear();

        // Initialize tracking for target vehicle
        const targetVehicle = mainroad.veh.find(v => v.id === id);
        if (targetVehicle) {
            vehicleLoopCounts.set(id, 0);
            lastPositions.set(id, targetVehicle.u);
        }

        isRecording = true;
        input.disabled = true;
        document.getElementById("downloadDiv").textContent = "Stop Recording";
        console.log(`Started recording vehicle ${id}`);
    } else {
        isRecording = false;
        input.disabled = false;
        document.getElementById("downloadDiv").textContent = "Start Recording";
        console.log(`Stopped recording. Vehicle completed ${vehicleLoopCounts.get(targetVehicleId) || 0} loops`);
        downloadCSV();
    }
}

function downloadCSV() {
    console.log("Downloading CSV with", recordingData.length, "rows");
    if (recordingData.length === 0) {
        alert("No data recorded.");
        return;
    }

    let csv = "Time,VehicleID,ContinuousPosition,RawPosition,CompletedLoops,Speed,Acceleration\n";
    recordingData.forEach(entry => {
        csv += `${entry.time},${entry.id},${entry.position},${entry.rawPosition},${entry.loops},${entry.speed},${entry.acceleration}\n`;
    });

    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
    const url = URL.createObjectURL(blob);

    const link = document.createElement('a');
    link.href = url;
    link.download = `vehicle_${targetVehicleId}_stopgo_data.csv`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

//#############################################################
// DQN Agent Class for Reinforcement Learning
//#############################################################

class DQNAgent {
    constructor(stateDim, actionSpace) {
        this.stateDim = stateDim;
        this.actionSpace = actionSpace;
        this.epsilon = 1.0;
        this.epsilonMin = 0.05;
        this.epsilonDecay = 0.995;
        this.gamma = 0.95;
        this.batchSize = 32;
        this.memory = [];
        this.model = this.buildModel();
    }

    buildModel() {
        const model = tf.sequential();
        model.add(tf.layers.dense({
            inputShape: [this.stateDim],
            units: 24,
            activation: 'relu'
        }));
        model.add(tf.layers.dense({
            units: 24,
            activation: 'relu'
        }));
        model.add(tf.layers.dense({
            units: this.actionSpace.length
        }));
        model.compile({
            loss: 'meanSquaredError',
            optimizer: 'adam'
        });
        return model;
    }

    async act(state) {
        if (Math.random() < this.epsilon) {
            return this.actionSpace[Math.floor(Math.random() * this.actionSpace.length)];
        }
        const tensor = tf.tensor([state]);
        const prediction = this.model.predict(tensor);
        const qValues = await prediction.data();
        prediction.dispose();
        tensor.dispose();
        return this.actionSpace[qValues.indexOf(Math.max(...qValues))];
    }

    remember(state, action, reward, nextState, done) {
        this.memory.push({ state, action, reward, nextState, done });
        if (this.memory.length > 2000) {
            this.memory.shift();
        }
    }

    async replay() {
        if (this.memory.length < this.batchSize) return;

        const batch = this.memory.sort(() => Math.random() - 0.5).slice(0, this.batchSize);

        const states = batch.map(d => d.state);
        const nextStates = batch.map(d => d.nextState);
        const stateTensor = tf.tensor2d(states);
        const nextStateTensor = tf.tensor2d(nextStates);

        const qValues = this.model.predict(stateTensor);
        const qNext = this.model.predict(nextStateTensor);

        const qUpdate = qValues.arraySync();

        batch.forEach((d, i) => {
            const aIndex = this.actionSpace.indexOf(d.action);
            let target = d.reward;
            if (!d.done) {
                target += this.gamma * Math.max(...qNext.arraySync()[i]);
            }
            qUpdate[i][aIndex] = target;
        });

        const qUpdateTensor = tf.tensor2d(qUpdate);
        await this.model.fit(stateTensor, qUpdateTensor, {
            epochs: 1,
            verbose: 0
        });

        // Clean up tensors
        stateTensor.dispose();
        nextStateTensor.dispose();
        qValues.dispose();
        qNext.dispose();
        qUpdateTensor.dispose();

        if (this.epsilon > this.epsilonMin) {
            this.epsilon *= this.epsilonDecay;
        }
    }

    async saveModel(name = "veh200-rl-model") {
        await this.model.save(`localstorage://${name}`);
        console.log("Model saved to localStorage");
    }

    async loadModel(name = "veh200-rl-model") {
        try {
            this.model = await tf.loadLayersModel(`localstorage://${name}`);
            this.model.compile({
                loss: 'meanSquaredError',
                optimizer: 'adam'
            });
            console.log("Model loaded from localStorage");
            this.epsilon = this.epsilonMin; // Skip exploration after load
        } catch (e) {
            console.warn("No saved model found, starting fresh");
        }
    }
}

//#############################################################
// Model Management Functions
//#############################################################

function getModelSizeInKB() {
    let totalBytes = 0;
    for (let key in localStorage) {
        if (key.startsWith("tensorflowjs_models/veh200-rl-model")) {
            const item = localStorage.getItem(key);
            totalBytes += item ? item.length * 2 : 0;
        }
    }
    return (totalBytes / 1024).toFixed(1);
}

function updateModelSizeDisplay() {
    const size = getModelSizeInKB();
    const infoEl = document.getElementById("modelSizeInfo");
    if (infoEl) {
        infoEl.textContent = `Model size: ${size} KB`;
    }
}

// Model control buttons
document.getElementById("resetModelBtn").onclick = () => {
    const keys = ["info", "model_topology", "weight_data", "weight_specs", "model_metadata"];
    keys.forEach(key => {
        localStorage.removeItem(`tensorflowjs_models/veh200-rl-model/${key}`);
    });
    updateModelSizeDisplay();
    alert("RL model reset. Reload the page to retrain.");
};

document.getElementById("saveModelBtn").onclick = () => {
    rlAgent.saveModel().then(() => {
        updateModelSizeDisplay();
        alert("Model manually saved.");
    });
};

//#############################################################
// Main Simulation Loop
//#############################################################

function main_loop() {
    updateSim();
    drawSim();
}

//#############################################################
// Animation Loop for Smooth Rendering
//#############################################################

function animationLoop() {
    drawSim();
    requestAnimationFrame(animationLoop);
}

//#############################################################
// Simulation Startup
//#############################################################

console.log("Starting Ring Road Stop-and-Go Traffic Simulation");
showInfo(); // Display initial information

// Start main simulation loop
var myRun = setInterval(main_loop, 1000 / fps);

// Start animation loop for smooth rendering
requestAnimationFrame(animationLoop);
