console.log("SCRIPT LOADED");

let simulationMode = "live";

const SIMULATION_SPEED = {
    live: 100,
    demo: 600
};

function updateEvent(message) {

    const eventDisplay =
        document.getElementById("eventDisplay");

    const eventLog =
        document.getElementById("eventLog");

    if (eventDisplay) {
        eventDisplay.textContent = message;
    }

    if (eventLog) {

        const entry =
            document.createElement("div");

        entry.textContent = message;

        eventLog.appendChild(entry);

        eventLog.scrollTop =
            eventLog.scrollHeight;
    }

    console.log("EVENT:", message);
}




const reservations = {};
function checkCollision(path, robotName) {

    for (let i = 0; i < path.length; i++) {

        const node = path[i];

        const key =
            node.row + "," +
            node.col + "," +
            i;


        if (
            reservations[key] &&
            reservations[key] !== robotName
        ) {

            return true;

        }

    }

    return false;
}

function reservePath(path, robotName) {

    for (let i = 0; i < path.length; i++) {

        const node = path[i];

        const key =
            node.row + "," +
            node.col + "," +
            i;


        reservations[key] = robotName;

    }

}

function releasePath(path, robotName) {

    for (let i = 0; i < path.length; i++) {

        const node = path[i];

        const key =
            node.row + "," +
            node.col + "," +
            i;

        if (reservations[key] === robotName) {
            delete reservations[key];
        }
    }
}





const robots = {
    "AMR-01": {
        element: document.querySelector(".robot1"),
        x: 16,
        y: 2,
        available: true,
        battery: 90
    },

    "AMR-02": {
        element: document.querySelector(".robot2"),
        x: 24,
        y: 10,
        available: true,
        battery: 75
    },

    "AMR-03": {
        element: document.querySelector(".robot3"),
        x: 12,
        y: 18,
        available: true,
        battery: 95
    }
};

const tasks = [
    {
        id: "T-001",
        location: {
            row: 5,
            col: 10
        },
        status: "Waiting"
    },

    {
        id: "T-002",
        location: {
            row: 15,
            col: 10
        },
        status: "Waiting"
    },

    {
        id: "T-003",
        location: {
            row: 5,
            col: 25
        },
        status: "Waiting"
    }
];

const packingStation = {
    row: 10,
    col: 18
};

function calculateDistance(robot, task) {

    return Math.abs(robot.x - task.location.col) +
        Math.abs(robot.y - task.location.row);

}
function calculateScore(robot, task) {

    const distance = calculateDistance(robot, task);

    const batteryPenalty =
        (100 - robot.battery) * 0.2;

    return distance + batteryPenalty;
}

function updateFleetDecision(
    taskId,
    robotName,
    reason,
    distances
) {

    document.getElementById("decisionTask").textContent =
        taskId;

    document.getElementById("decisionRobot").textContent =
        robotName;

    document.getElementById("decisionReason").textContent =
        reason;

    const comparison =
        document.getElementById("robotComparison");

    comparison.innerHTML = "";

    for (const item of distances) {

        const row =
            document.createElement("div");

        row.className = "comparison-row";

        if (item.robot === robotName) {
            row.innerHTML =
                "✓ " +
                item.robot +
                " — " +
                item.distance +
                " cells";
        } else {
            row.innerHTML =
                item.robot +
                " — " +
                item.distance +
                " cells";
        }

        comparison.appendChild(row);
    }
}

function allocateTask(task) {

    let bestRobot = null;

    let bestScore = Infinity;
    const distances = [];


    for (const [robotName, robot] of Object.entries(robots)) {

        if (!robot.available) {
            continue;
        }

        const distance =
            calculateDistance(robot, task);

        const score =
            calculateScore(robot, task);


        console.log(
            robotName,
            "distance to",
            task.id,
            "=",
            distance,
            "battery =",
            robot.battery + "%",
            "score =",
            score
        );
        distances.push({
            robot: robotName,
            distance: distance
        });


        if (score < bestScore) {
            bestScore = score;

            bestRobot = robotName;
        }
    }


    if (bestRobot !== null) {

        robots[bestRobot].available = false;

        task.status = "Assigned";

        task.assignedRobot = bestRobot;


        console.log(
            task.id,
            "assigned to",
            bestRobot
        );
        updateEvent(
            "📦 " +
            task.id +
            " assigned to " +
            bestRobot
        );
        updateFleetDecision(
            task.id,
            bestRobot,
            "Shortest distance among available AMRs",
            distances
        );
        updateRobotStatus(
            bestRobot,
            "Busy - " + task.id
        );


        setTimeout(() => {

            executeTask(
                task,
                bestRobot
            );

        }, 500);

    }

}

function updateRobotStatus(robotName, status) {

    let statusElement = null;

    if (robotName === "AMR-01") {
        statusElement = document.getElementById("status1");
    }

    if (robotName === "AMR-02") {
        statusElement = document.getElementById("status2");
    }

    if (robotName === "AMR-03") {
        statusElement = document.getElementById("status3");
    }

    if (statusElement) {
        statusElement.textContent = status;

        statusElement.className =
            "status-" +
            status.toLowerCase().replaceAll(" ", "-");
    }
}


const robot1 = document.querySelector(".robot1");
const robot2 = document.querySelector(".robot2");
const robot3 = document.querySelector(".robot3");

const cellSize = 25;

const rows = 22;
const cols = 36;


/*
    0 = free space
    1 = obstacle
*/

const grid = Array.from({ length: rows }, () =>
    Array(cols).fill(0)
);


// Create shelves as obstacles

for (let r = 4; r <= 7; r++) {
    for (let c = 4; c <= 9; c++) {
        grid[r][c] = 1;
    }
}

for (let r = 14; r <= 17; r++) {
    for (let c = 4; c <= 9; c++) {
        grid[r][c] = 1;
    }
}

for (let r = 4; r <= 7; r++) {
    for (let c = 26; c <= 31; c++) {
        grid[r][c] = 1;
    }
}

for (let r = 14; r <= 17; r++) {
    for (let c = 26; c <= 31; c++) {
        grid[r][c] = 1;
    }
}


/*
    A* PATHFINDING
*/

function heuristic(a, b) {

    return Math.abs(a.row - b.row) +
        Math.abs(a.col - b.col);

}


function getNeighbors(node) {

    const directions = [
        [-1, 0],
        [1, 0],
        [0, -1],
        [0, 1]
    ];

    const neighbors = [];

    for (const [dr, dc] of directions) {

        const row = node.row + dr;
        const col = node.col + dc;

        if (
            row >= 0 &&
            row < rows &&
            col >= 0 &&
            col < cols &&
            grid[row][col] === 0
        ) {

            neighbors.push({
                row: row,
                col: col
            });

        }

    }

    return neighbors;
}


function sameNode(a, b) {

    return a.row === b.row &&
        a.col === b.col;

}


function findPath(start, goal) {

    let openSet = [start];

    let cameFrom = {};

    let gScore = {};

    let fScore = {};

    function key(node) {
        return node.row + "," + node.col;
    }

    gScore[key(start)] = 0;

    fScore[key(start)] = heuristic(start, goal);


    while (openSet.length > 0) {

        openSet.sort(
            (a, b) =>
                fScore[key(a)] - fScore[key(b)]
        );

        let current = openSet.shift();


        if (sameNode(current, goal)) {

            let path = [current];

            while (key(current) in cameFrom) {

                current = cameFrom[key(current)];

                path.push(current);

            }

            return path.reverse();

        }


        for (const neighbor of getNeighbors(current)) {

            const neighborKey = key(neighbor);

            const currentKey = key(current);

            const tentativeG =
                gScore[currentKey] + 1;


            if (
                !(neighborKey in gScore) ||
                tentativeG < gScore[neighborKey]
            ) {

                cameFrom[neighborKey] = current;

                gScore[neighborKey] = tentativeG;

                fScore[neighborKey] =
                    tentativeG +
                    heuristic(neighbor, goal);


                if (
                    !openSet.some(
                        node => sameNode(node, neighbor)
                    )
                ) {

                    openSet.push(neighbor);

                }

            }

        }

    }

    return [];
}


/*
    MOVE ROBOT ALONG PATH
*/

function moveRobot(
    robot,
    start,
    goal,
    onComplete = null,
    robotName = null,
    wasWaiting = false
) {

    const path =
        findPath(start, goal);


    if (path.length === 0) {

        console.log(
            "No path available"
        );

        return;

    }


    /*
        Check for conflicts
    */

    if (
        robotName &&
        checkCollision(path, robotName)
    ) {

        console.log(
            "⚠️ Collision risk detected for",
            robotName
        );


        updateRobotStatus(
            robotName,
            "Waiting - congestion"
        );

        console.log(
            robotName,
            "status → Waiting - congestion"
        );
        updateEvent(
            "🟡 " +
            robotName +
            " waiting — congestion detected"
        );


        setTimeout(() => {

            console.log(
                robotName,
                "checking congestion again..."
            );

            moveRobot(
                robot,
                start,
                goal,
                onComplete,
                robotName,
                true
            );

        }, 1500);

        return;

    }


    /*
        Reserve path
    */

    if (robotName) {
        reservePath(
            path,
            robotName
        );

        updateRobotStatus(
            robotName,
            "Moving"
        );

        if (wasWaiting) {

            console.log(
                robotName,
                "congestion cleared → resuming movement"
            );

            updateEvent(
                "🔵 " +
                robotName +
                " congestion cleared — resuming movement"
            );

        }
    }


    let index = 0;


    const interval = setInterval(() => {

        if (index >= path.length) {

            clearInterval(interval);

            // Release the path after the robot has finished using it
            if (robotName) {
                releasePath(path, robotName);
            }

            if (onComplete) {
                onComplete();
            }

            return;
        }


        const node = path[index];


        robot.style.left =
            (node.col * cellSize) +
            "px";


        robot.style.top =
            (node.row * cellSize) +
            "px";

        // Update AMR's logical position
        if (robotName && robots[robotName]) {
            robots[robotName].x = node.col;
            robots[robotName].y = node.row;
        }


        index++;

    }, SIMULATION_SPEED[simulationMode]);

}

/*
    START ROBOTS
*/



function executeTask(task, robotName) {

    const robot = robots[robotName];

    const start = {
        row: robot.y,
        col: robot.x
    };

    const pickupLocation = task.location;

    console.log(
        robotName,
        "going to pickup",
        task.id
    );
    updateEvent(
        "🔵 " +
        robotName +
        " going to pickup " +
        task.id
    );

    updateRobotStatus(
        robotName,
        "Going to pickup"
    );

    updateTaskDisplay(
        task.id,
        "Going to pickup",
        robotName
    );

    moveRobot(
        robot.element,
        start,
        pickupLocation,
        () => {

            console.log(
                robotName,
                "picked up",
                task.id
            );
            updateEvent(
                "📦 " +
                robotName +
                " picked up " +
                task.id
            );

            updateRobotStatus(
                robotName,
                "Picking up"
            );

            updateTaskDisplay(
                task.id,
                "Picking up",
                robotName
            );

            setTimeout(() => {

                updateEvent(
                    "🚚 " +
                    robotName +
                    " delivering " +
                    task.id
                );

                updateRobotStatus(
                    robotName,
                    "Delivering"
                );

                updateTaskDisplay(
                    task.id,
                    "Delivering",
                    robotName
                );

                moveRobot(
                    robot.element,
                    pickupLocation,
                    packingStation,
                    () => {

                        console.log(
                            task.id,
                            "completed by",
                            robotName
                        );
                        updateEvent(
                            "✅ " +
                            task.id +
                            " completed by " +
                            robotName
                        );

                        task.status = "Completed";

                        updateTaskDisplay(
                            task.id,
                            "Completed",
                            robotName
                        );

                        updateRobotStatus(
                            robotName,
                            "Available"
                        );

                    },
                    robotName
                );

            }, SIMULATION_SPEED[simulationMode]);

        },
        robotName
    );
}

function updateTaskDisplay(
    taskId,
    status,
    robotName
) {

    const taskElements =
        document.querySelectorAll(".task");

    taskElements.forEach(taskElement => {

        const text = taskElement.textContent;

        if (text.includes(taskId)) {

            const statusElement =
                taskElement.querySelector("span");

            if (statusElement) {

                statusElement.textContent =
                    status;

                statusElement.className =
                    status.toLowerCase()
                        .replaceAll(" ", "-");
            }

            let assignment =
                taskElement.querySelector(".assignment");

            if (!assignment) {

                assignment =
                    document.createElement("p");

                assignment.className =
                    "assignment";

                taskElement.appendChild(
                    assignment
                );
            }

            assignment.textContent =
                "Assigned: " + robotName;
        }
    });
}

function startSimulation(mode) {

    simulationMode = mode;

    document.getElementById("modeDisplay").textContent =
        mode === "live"
            ? "Live Simulation"
            : "Demo Mode";

    document.getElementById("eventDisplay").textContent =
        "Simulation started";

    console.log(
        "🚀 Simulation started:",
        mode
    );

    for (const task of tasks) {
        allocateTask(task);
    }
}


document.getElementById("liveModeBtn").addEventListener("click", () => {

    resetSimulationState();

    startSimulation("live");

});


document.getElementById("demoModeBtn").addEventListener("click", () => {

    resetSimulationState();

    startSimulation("demo");

});

document.getElementById("resetBtn").addEventListener("click", () => {

    resetSimulation();

});

function resetSimulationState() {

    // Reset robots
    robots["AMR-01"].x = 16;
    robots["AMR-01"].y = 2;
    robots["AMR-01"].available = true;

    robots["AMR-02"].x = 24;
    robots["AMR-02"].y = 10;
    robots["AMR-02"].available = true;

    robots["AMR-03"].x = 12;
    robots["AMR-03"].y = 18;
    robots["AMR-03"].available = true;

    // Reset robot positions on screen
    robot1.style.left = (16 * cellSize) + "px";
    robot1.style.top = (2 * cellSize) + "px";

    robot2.style.left = (24 * cellSize) + "px";
    robot2.style.top = (10 * cellSize) + "px";

    robot3.style.left = (12 * cellSize) + "px";
    robot3.style.top = (18 * cellSize) + "px";

    // Reset tasks
    for (const task of tasks) {
        task.status = "Waiting";
        task.assignedRobot = null;
    }

    // Clear reservations
    for (const key in reservations) {
        delete reservations[key];
    }

    // Reset display
    updateRobotStatus("AMR-01", "Available");
    updateRobotStatus("AMR-02", "Available");
    updateRobotStatus("AMR-03", "Available");

    document.getElementById("eventDisplay").textContent =
        "Ready to start";

    document.getElementById("modeDisplay").textContent =
        "Not Started";
}

function resetSimulation() {

    // Reload the page to completely reset the simulation
    location.reload();

}