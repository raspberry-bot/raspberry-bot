var canvasContainer = document.getElementById("canvasContainer");
var canvasImg = document.getElementById("liveImg");

var wsCamera;
var wsDrive;
var last_x_y = { 'x': 0, 'y': 0 }
var nipple;
var tryingToConnect = false;

var serverURL = "http://raspberrybot.local";
// var serverURL = "http://localhost:8000";
const cameraURL = serverURL + "/api/operate/camera";


function requestImage() {
    if (wsCamera.readyState === WebSocket.OPEN) {
        request_start_time = performance.now();
        wsCamera.send(1);
    }
}

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

function setupCameraDriveWebSockets() {
    var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";
    // Camera

    wsCamera = new WebSocket(wsProtocol + "raspberrybot.local" + "/api/operate/camera");
    wsCamera.binaryType = 'arraybuffer';

    wsCamera.onopen = function () {
        console.log("connection was established for Camera");
        requestImage();
    };

    wsCamera.onmessage = function (evt) {
        var arrayBuffer = evt.data;
        var blob = new Blob([new Uint8Array(arrayBuffer)], { type: "image/jpeg" });
        var ctx = canvasImg.getContext("2d");
        var img = new Image();
        img.src = window.URL.createObjectURL(blob);
        function loadImg() {
            /// initial draw of image
            ctx.drawImage(img, 0, 0, canvasImg.width, canvasImg.height);
            /// listen to mouse move (or use jQuery on('mousemove') instead)
            // canvas.onmousemove = updateLine;
        };
        img.onload = loadImg;
    };

    wsCamera.onerror = function (e) {
        console.log(e);
        wsCamera.send(1);
    };

    wsCamera.onclose = function (e) {
        console.log(e);
        if (tryingToConnect != true) {
            setTimeout(setupCameraDriveWebSockets, 1000);
            tryingToConnect = true;
        } else {
            if (wsCamera != null && wsCamera.readyState === WebSocket.CLOSED || wsCamera.readyState === WebSocket.CLOSING) {
                tryingToConnect = false;
            }
        }
    }

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
            setTimeout(setupCameraDriveWebSockets, 1000);
            tryingToConnect = true;
        } else {
            if (wsDrive != null && wsDrive.readyState === WebSocket.CLOSED || wsDrive.readyState === WebSocket.CLOSING) {
                tryingToConnect = false;
            }
        }
    }
    return [wsCamera, wsDrive];
}


$(document).ready(function () {
    'use strict'

    if ("WebSocket" in window) {
        var websockets = setupCameraDriveWebSockets();
        wsCamera = websockets[0];
        wsDrive = websockets[1];
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
        canvasContainer.width = window.innerWidth;
        canvasContainer.height = window.innerHeight - ((20 / 100) * window.innerHeight);
        canvasImg.width = canvasContainer.width;
        canvasImg.height = canvasContainer.height;
        requestImage();
    }
    window.addEventListener('resize', resizeCanvas, false);
    resizeCanvas();    /// call the first time page is loaded


    // $('#fullscreenButton').on('click touchstart', function () {
    //     toggleFullscreen();
    // });
    $('#stopMotorsButton').on('click touchstart', function () {
        stopMotors();
    });
});
