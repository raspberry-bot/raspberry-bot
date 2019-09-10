var canvasContainer = document.getElementById("canvasContainer");
var canvasImg = document.getElementById("liveImg");
var wsProtocol = (location.protocol === "https:") ? "wss://" : "ws://";
var wsDrive;
var last_x_y = { 'x': 0, 'y': 0 }
var nipple;
var tryingToConnect = false;

var serverURL = "http://raspberrybot.local";
var serverName = "raspberrybot.local";
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
    wsDrive = new WebSocket(wsProtocol + serverName + "/api/operate/drive");


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

function initGyroscope() {
    var width = window.innerWidth * 0.30; // 20% of the screen size
    var height = window.innerHeight * 0.30;

    var rot = { x: 0, y: 0 };
    var pos = { x: 0, y: 0, z: 0 };
    var scene = new THREE.Scene();
    // var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
    var camera = new THREE.PerspectiveCamera(75, width / height, 0.1, 1000);
    camera.position.z = 5;
    var renderer = new THREE.WebGLRenderer({ alpha: true });
    renderer.setClearColor(0xffffff, 0);
    // renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setSize(width, height);
    // document.body.appendChild(renderer.domElement);

    var gyroscopeVisual = document.getElementById("gyroscopeVisual");
    var gyroscopeData = document.getElementById("gyroscopeData");
    gyroscopeVisual.appendChild(renderer.domElement);

    // renderer.domElement.overdraw(gyroscopeContainer);

    var light = new THREE.AmbientLight(0x404040); // soft white light
    scene.add(light);

    // var loader = new STLLoader();
    // loader.load('./assets/media/base.stl', function (geometry) {
    //     var meshMaterial = material;
    //     if (geometry.hasColors) {
    //         meshMaterial = new THREE.MeshPhongMaterial({ opacity: geometry.alpha, vertexColors: THREE.VertexColors });
    //     }
    //     var mesh = new THREE.Mesh(geometry, meshMaterial);
    //     mesh.position.set(0.5, 0.2, 0);
    //     mesh.rotation.set(- Math.PI / 2, Math.PI / 2, 0);
    //     mesh.scale.set(0.3, 0.3, 0.3);
    //     mesh.castShadow = true;
    //     mesh.receiveShadow = true;
    //     scene.add(mesh);
    // });






    var geometry = new THREE.BoxGeometry(2, 0.5, 2);
    for (var i = 0; i < geometry.faces.length; i += 2) {
        var hex = Math.random() * 0xffffff;
        geometry.faces[i].color.setHex(hex);
        geometry.faces[i + 1].color.setHex(hex);
    }
    var material = new THREE.MeshBasicMaterial({ vertexColors: THREE.FaceColors, overdraw: 0.5 });
    var cube = new THREE.Mesh(geometry, material);
    scene.add(cube);
    function render() {
        requestAnimationFrame(render);
        cube.rotation.x = -rot.x;
        cube.rotation.z = -rot.y;
        cube.position.x = pos.x;
        cube.position.y = pos.y;
        cube.position.z = pos.z;
        renderer.render(scene, camera);
    }
    render();

    var gyroscopeWs = new WebSocket(wsProtocol + serverName + '/api/sensors/gyroscope');
    gyroscopeWs.addEventListener('open', function () {
        console.log('gyroscopeWs WebSocket opened!');
    });
    gyroscopeWs.addEventListener('close', function () {
        console.log('gyroscopeWs WebSocket closed.');
    });
    gyroscopeWs.addEventListener('message', function (event) {
        var json = event.data.trim();
        var data = JSON.parse(json);
        if (data != null) {
            rot.x = data.rotation.x / 180 * Math.PI;
            rot.y = -1 * data.rotation.y / 180 * Math.PI;

            gyroscopeData.innerHTML = '<pre id="gyroscopeDataText">' + JSON.stringify(data, undefined, '\t') + '</pre>';

        }
    });
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

    initGyroscope();

    function resizeCanvas() {
        canvasContainer.width = window.innerWidth;
        canvasContainer.height = window.innerHeight - ((10 / 100) * window.innerHeight);
        if (canvasImg != null) {
            canvasImg.width = canvasContainer.width;
            canvasImg.height = canvasContainer.height;
        }
    }
    window.addEventListener('resize', resizeCanvas, false);
    resizeCanvas();
    $('#stopMotorsButton').on('click touchstart', function () {
        stopMotors();
    });
});
