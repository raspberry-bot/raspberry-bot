var canvasContainer = document.getElementById("canvasContainer");
var canvasImg = document.getElementById("liveImg");
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

var serverURL = "http://raspberrybot.local";
// var serverURL = "http://localhost:8000";
const cameraURL = serverURL + "/api/operate/camera";
const controlURL = serverURL + "/api/operate/control";


var tryingToConnect = false;

function requestImage() {
    if (wsCamera.readyState === WebSocket.OPEN) {
        request_start_time = performance.now();
        wsCamera.send(1);
    }
}

function setupCameraDriveWebSockets() {
    var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";
    // Camera

    wsCamera = new WebSocket(wsProtocol + "raspberrybot.local" + "/api/operate/camera");
    wsCamera.binaryType = 'arraybuffer';

    wsCamera.onopen = function () {
        console.log("connection was established for Camera");
        start_time = performance.now();
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

        // setTimeout(requestImage, timeout);
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

var fullscreenElement = document.getElementById("main");
function toggleFullscreen() {
    if (!document.fullscreenElement && !document.mozFullScreenElement &&
        !document.webkitFullscreenElement && !document.msFullscreenElement) {
        if (fullscreenElement.requestFullscreen) {
            fullscreenElement.requestFullscreen();
        } else if (fullscreenElement.msRequestFullscreen) {
            fullscreenElement.msRequestFullscreen();
        } else if (fullscreenElement.mozRequestFullScreen) {
            fullscreenElement.mozRequestFullScreen();
        } else if (fullscreenElement.webkitRequestFullscreen) {
            fullscreenElement.webkitRequestFullscreen(Element.ALLOW_KEYBOARD_INPUT);
        }
    } else {
        if (document.exitFullscreen) {
            document.exitFullscreen();
        } else if (document.msExitFullscreen) {
            document.msExitFullscreen();
        } else if (document.mozCancelFullScreen) {
            document.mozCancelFullScreen();
        } else if (document.webkitExitFullscreen) {
            document.webkitExitFullscreen();
        }
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

var last_x_y = { 'x': 0, 'y': 0 }
var nipple;

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

    var isMobile = true; //initiate as false
    // device detection
    if (/(android|bb\d+|meego).+mobile|avantgo|bada\/|blackberry|blazer|compal|elaine|fennec|hiptop|iemobile|ip(hone|od)|ipad|iris|kindle|Android|Silk|lge |maemo|midp|mmp|netfront|opera m(ob|in)i|palm( os)?|phone|p(ixi|re)\/|plucker|pocket|psp|series(4|6)0|symbian|treo|up\.(browser|link)|vodafone|wap|windows (ce|phone)|xda|xiino/i.test(navigator.userAgent)
        || /1207|6310|6590|3gso|4thp|50[1-6]i|770s|802s|a wa|abac|ac(er|oo|s\-)|ai(ko|rn)|al(av|ca|co)|amoi|an(ex|ny|yw)|aptu|ar(ch|go)|as(te|us)|attw|au(di|\-m|r |s )|avan|be(ck|ll|nq)|bi(lb|rd)|bl(ac|az)|br(e|v)w|bumb|bw\-(n|u)|c55\/|capi|ccwa|cdm\-|cell|chtm|cldc|cmd\-|co(mp|nd)|craw|da(it|ll|ng)|dbte|dc\-s|devi|dica|dmob|do(c|p)o|ds(12|\-d)|el(49|ai)|em(l2|ul)|er(ic|k0)|esl8|ez([4-7]0|os|wa|ze)|fetc|fly(\-|_)|g1 u|g560|gene|gf\-5|g\-mo|go(\.w|od)|gr(ad|un)|haie|hcit|hd\-(m|p|t)|hei\-|hi(pt|ta)|hp( i|ip)|hs\-c|ht(c(\-| |_|a|g|p|s|t)|tp)|hu(aw|tc)|i\-(20|go|ma)|i230|iac( |\-|\/)|ibro|idea|ig01|ikom|im1k|inno|ipaq|iris|ja(t|v)a|jbro|jemu|jigs|kddi|keji|kgt( |\/)|klon|kpt |kwc\-|kyo(c|k)|le(no|xi)|lg( g|\/(k|l|u)|50|54|\-[a-w])|libw|lynx|m1\-w|m3ga|m50\/|ma(te|ui|xo)|mc(01|21|ca)|m\-cr|me(rc|ri)|mi(o8|oa|ts)|mmef|mo(01|02|bi|de|do|t(\-| |o|v)|zz)|mt(50|p1|v )|mwbp|mywa|n10[0-2]|n20[2-3]|n30(0|2)|n50(0|2|5)|n7(0(0|1)|10)|ne((c|m)\-|on|tf|wf|wg|wt)|nok(6|i)|nzph|o2im|op(ti|wv)|oran|owg1|p800|pan(a|d|t)|pdxg|pg(13|\-([1-8]|c))|phil|pire|pl(ay|uc)|pn\-2|po(ck|rt|se)|prox|psio|pt\-g|qa\-a|qc(07|12|21|32|60|\-[2-7]|i\-)|qtek|r380|r600|raks|rim9|ro(ve|zo)|s55\/|sa(ge|ma|mm|ms|ny|va)|sc(01|h\-|oo|p\-)|sdk\/|se(c(\-|0|1)|47|mc|nd|ri)|sgh\-|shar|sie(\-|m)|sk\-0|sl(45|id)|sm(al|ar|b3|it|t5)|so(ft|ny)|sp(01|h\-|v\-|v )|sy(01|mb)|t2(18|50)|t6(00|10|18)|ta(gt|lk)|tcl\-|tdg\-|tel(i|m)|tim\-|t\-mo|to(pl|sh)|ts(70|m\-|m3|m5)|tx\-9|up(\.b|g1|si)|utst|v400|v750|veri|vi(rg|te)|vk(40|5[0-3]|\-v)|vm40|voda|vulc|vx(52|53|60|61|70|80|81|83|85|98)|w3c(\-| )|webc|whit|wi(g |nc|nw)|wmlb|wonu|x700|yas\-|your|zeto|zte\-/i.test(navigator.userAgent.substr(0, 4))) {
        isMobile = true;
    }
    console.log('Mobile device == ' + isMobile)

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
        })
    };

    joyInit();

    // if (isMobile){
    //     // launchTouchPad();
    //     launchOrientationMap();
    // }
    function resizeCanvas() {
        canvasContainer.width = window.innerWidth;
        canvasContainer.height = window.innerHeight - ((10 / 100) * window.innerHeight);
        canvasImg.width = canvasContainer.width;
        canvasImg.height = canvasContainer.height;
        requestImage();
    }
    window.addEventListener('resize', resizeCanvas, false);
    resizeCanvas();    /// call the first time page is loaded

    document.getElementById('fullscreenButton').addEventListener('click', function () {
        toggleFullscreen();
    });
});
