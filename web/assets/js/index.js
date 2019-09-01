var canvasContainer = document.getElementById("canvasContainer");
var canvasImg = document.getElementById("liveImg");

var wsDrive;
var last_x_y = { 'x': 0, 'y': 0 }
var nipple;
var tryingToConnect = false;

var serverURL = "http://raspberrybot.local";
// var serverURL = "http://localhost:8000";


function stopMotors() {
    var msg = JSON.stringify({ "x": 0, "y": 0, "min_speed": -100, "max_speed": 100 });
    sendControlData(msg);
}

function controlSignal(e) {
    var msg = null;
    e = e || window.event;
    if (e.keyCode == '32') {
        // space bar
        msg = JSON.stringify({ "x": 0, "y": 0, "min_speed": -100, "max_speed": 100 });
    }
    if (msg != null) {
        sendControlData(msg);
    }
}

function sendControlData(controlKey) {
    wsDrive.send(controlKey)
}

var fullscreenElement = document.getElementById("main");
function toggleFullscreen() {
    if (screenfull.enabled) {
        if (screenfull.enabled) {
            screenfull.toggle(fullscreenElement);
        }
        screenfull.on('error', event => {
            alert('Failed to enable fullscreen');
        });
    }
}

function setupDriveWebSockets() {
    var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";
    // Drive

    wsDrive = new WebSocket(wsProtocol + "raspberrybot.local" + "/api/operate/drive");


    wsDrive.onopen = function () {
        console.log("connection was established for Drive");
    };

    wsDrive.onmessage = function (evt) {
        var message = evt.data;
        // document.getElementById('control').value = message;
    };

    wsDrive.onerror = function (e) {
        console.log(e);
        stopMotors();
        // wsDrive.send('stop');
    };

    wsDrive.onclose = function (e) {
        console.log(e);
        if (tryingToConnect != true) {
            setTimeout(setupDriveWebSockets, 1000);
            tryingToConnect = true;
        } else {
            if (wsDrive != null && wsDrive.readyState === WebSocket.CLOSED || wsDrive.readyState === WebSocket.CLOSING) {
                tryingToConnect = false;
            }
        }
    }
    return wsDrive;
}


$(document).ready(function () {
    'use strict'

    if ("WebSocket" in window) {
        wsDrive = setupDriveWebSockets();
    } else {
        alert("WebSocket not supported");
    }

    document.onkeydown = controlSignal;

    function joyInit() {
        nipple = nipplejs.create({
            zone: document.getElementById('canvasContainer'),
            mode: 'semi',
            catchDistance: 150,
            color: 'red'
        });
        nipple.on('move', function (evt, data) {
            var x = data.position.x - nipple[0].position.x;
            var y = nipple[0].position.y - data.position.y;
            if (x != last_x_y.x || y != last_x_y.y) {
                var msg = JSON.stringify({ "x": x, "y": y, "min_speed": -100, "max_speed": 100 })
                sendControlData(msg);
                last_x_y.x, last_x_y.y = x, y;
            }
        }).on('end', function (evt, data) {
            var msg = JSON.stringify({ "x": 0, "y": 0, "min_speed": -100, "max_speed": 100 })
            sendControlData(msg);
            last_x_y.x, last_x_y.y = 0, 0;
        })
    };

    joyInit();

    function resizeCanvas() {
        canvasContainer.width = window.outerWidth;
        canvasContainer.height = window.outerHeight; // - ((20 / 100) * window.innerHeight);
        canvasImg.width = canvasContainer.width;
        canvasImg.height = canvasContainer.height;
    }
    window.addEventListener('resize', resizeCanvas, false);
    resizeCanvas();
    $('#stopMotorsButton').on('click touchstart', function () {
        stopMotors();
    });
});
