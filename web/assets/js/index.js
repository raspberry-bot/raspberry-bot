var img = document.getElementById("liveImg");
var fpsText = document.getElementById("fps");

var target_fps = 40;

var request_start_time = performance.now();
var start_time = performance.now();
var time = 0;
var request_time = 0;
var time_smoothing = 0.3; // larger=more smoothing
var request_time_smoothing = 0.2; // larger=more smoothing
var target_time = 100 / target_fps;

var wsCamera;
var wsDrive;

var serverURL = "http://thegreenbot.local";
// var serverURL = "http://localhost:8000";
const cameraURL = serverURL + "/api/operate/camera";
const controlURL = serverURL + "/api/operate/control";

function requestImage() {
    request_start_time = performance.now();
    wsCamera.send(1);
}

var tryingToConnect = false;

function setupCameraDriveWebSockets() {
    var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";
    // Camera

    wsCamera = new WebSocket(wsProtocol + "thegreenbot.local" + "/api/operate/camera");
    wsCamera.binaryType = 'arraybuffer';

    wsCamera.onopen = function() {
        console.log("connection was established for Camera");
        start_time = performance.now();
        requestImage();
    };
    
    wsCamera.onmessage = function(evt) {
        var arrayBuffer = evt.data;
        var blob  = new Blob([new Uint8Array(arrayBuffer)], {type: "image/jpeg"});
        img.src = window.URL.createObjectURL(blob);
        var end_time = performance.now();
        var current_time = end_time - start_time;
        // smooth with moving average
        time = (time * time_smoothing) + (current_time * (1.0 - time_smoothing));
        start_time = end_time;
        var fps = Math.round(1000 / time);
        fpsText.textContent = fps;
    
        var current_request_time = performance.now() - request_start_time;
        // smooth with moving average
        request_time = (request_time * request_time_smoothing) + (current_request_time * (1.0 - request_time_smoothing));
        var timeout = Math.max(0, target_time - request_time);
    
        setTimeout(requestImage, timeout);
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

    wsDrive = new WebSocket(wsProtocol + "thegreenbot.local" + "/api/operate/drive");
    

    wsDrive.onopen = function() {
        console.log("connection was established for Drive");
    };
    
    wsDrive.onmessage = function(evt) {
        var message = evt.data;
        document.getElementById('control').value = message;
    };

    wsDrive.onerror = function (e) {
        console.log(e);
        wsDrive.send('stop');
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
var wsDrive;
var wsCamera;

function sendControlData(controlKey) {
    wsDrive.send(controlKey)
}

$(document).ready(function() {
    'use strict'

    if ("WebSocket" in window) {
        var websockets = setupCameraDriveWebSockets();
        wsCamera = websockets[0];
        wsDrive = websockets[1];
    } else {
        alert("WebSocket not supported");
    }

    document.onkeydown = controlSignal;

    function controlSignal(e) {
        var controlKey = null;
        e = e || window.event;
        if (e.keyCode == '38') {
            // up arrow
            console.log('forward arrow')
            controlKey = 'forward';
        }
        else if (e.keyCode == '40') {
            // down arrow
            console.log('reverse arrow')
            controlKey = 'reverse';
        }
        else if (e.keyCode == '37') {
        // left arrow
            console.log('left arrow')
            controlKey = 'left';
        }
        else if (e.keyCode == '39') {
        // right arrow
            console.log('right arrow')
            controlKey = 'right';
        }
        else if (e.keyCode == '32') {
        // space bar
            console.log('stop - space bar')
            controlKey = 'stop';
        }
        if (controlKey != null){
            sendControlData(controlKey);
        }
    }

    var ball   = document.querySelector('.ball');
    var plane = document.querySelector('.plane');
    var output = document.querySelector('.output');

    var maxX = plane.clientWidth  - ball.clientWidth;
    var maxY = plane.clientHeight - ball.clientHeight;

    function handleOrientation(event) {
        var x = event.beta;  // In degree in the range [-180,180]
        var y = event.gamma; // In degree in the range [-90,90]

        output.innerHTML  = "beta : " + x + "\n";
        output.innerHTML += "gamma: " + y + "\n";

        // Because we don't want to have the device upside down
        // We constrain the x value to the range [-90,90]
        if (x >  90) { x =  90};
        if (x < -90) { x = -90};

        // To make computation easier we shift the range of 
        // x and y to [0,180]
        x += 90;
        y += 90;

        // 10 is half the size of the ball
        // It center the positioning point to the center of the ball
        ball.style.top  = (maxX*x/180 - 10) + "px";
        ball.style.left = (maxY*y/180 - 10) + "px";
        sendControlData(
            JSON.stringify({
                x: x,
                y: y
            })
        )
    }

    window.addEventListener('deviceorientation', handleOrientation);

    // window.addEventListener('resize', resizeCanvas, false);
    // resizeCanvas();    /// call the first time page is loaded
    // function resizeCanvas() {
    //     canvas.width = window.innerWidth;
    //     canvas.height = window.innerHeight;
    // }
});
