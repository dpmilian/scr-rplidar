// This file is required by the index.html file and will
// be executed in the renderer process for that window.
// All of the Node.js APIs are available in this process.

var camera, controls, scene, renderer;
var geometry, material, box;
var pointGeometry = new THREE.BufferGeometry();
pointGeometry.dynamic = true;
var pointMaterial = new THREE.PointsMaterial( { size: .02, color: 0xFF6700 } );
var points;

var dists = [], angles = [], qs = [];
// var updatedData, updatedView;

// ------------------------------

function rad(degrees){
    return Math.PI / 180 * degrees;
}

function deg(rad){
    return rad /Math.PI * 180;
}
function init() {
    // updatedData = window.performance.now();
    // updatedView = window.performance.now();

    camera = new THREE.PerspectiveCamera(110, window.innerWidth / window.innerHeight, 0.01, 25);
    camera.position.z = 2;
    camera.position.y = -1;
    camera.rotation.x = rad(15);

    // controls = new THREE.OrbitControls(camera);

    scene = new THREE.Scene();

    grid = new THREE.PolarGridHelper(30, 16, 30, 64, 0x00ff00, 0x0000ff);
    grid.rotation.x = Math.PI / 2;

    scene.add(grid);

    scene.fog = new THREE.FogExp2(0xff0000, 0.0128);

    // material = new THREE.MeshLambertMaterial({color: 0x0000ff, transparent: true, opacity: 0.5});

    geometry = new THREE.BoxGeometry(1, 1, 1);
    material = new THREE.MeshNormalMaterial();

    box = new THREE.Mesh(geometry, material);
    box.position.x = -4;
    box.position.z = 1;
    scene.add(box);

    renderer = new THREE.WebGLRenderer({ antialias: false, alpha: true });
    // renderer.setClearColor(scene.fog.color, 1);

    renderer.setClearColor(0x000000, 0);
    renderer.setSize(window.innerWidth, window.innerHeight);

    document.getElementById('view').appendChild(renderer.domElement);

}

// ------------------------------

function polarToCartesian(r, phi) {

    var pos = { 'x': 0, 'y': 0 , 'z': 0};

    pos.x = r * Math.cos((phi + 90) * Math.PI/180);
    pos.y = r * Math.sin((phi + 90) * Math.PI/180);

    return pos;
}

// ------------------------------

function animate() {

    setTimeout(function () {

        requestAnimationFrame(animate);

    }, 1000 / 60);

    box.rotation.x += 0.01;
    box.rotation.y += 0.02;

    updatePointCloud();

    renderer.render(scene, camera);

}

const remote = require('electron').remote;

function initClose() {
    document.getElementById("min-btn").addEventListener("click", function (e) {
        const window = remote.getCurrentWindow();
        window.minimize();
    });

    document.getElementById("max-btn").addEventListener("click", function (e) {
        const window = remote.getCurrentWindow();
        if (!window.isMaximized()) {
            window.maximize();
        } else {
            window.unmaximize();
        }
    });

    document.getElementById("close-btn").addEventListener("click", function (e) {
        const window = remote.getCurrentWindow();
        window.close();
    });

};

// ** ONLOAD EVENT FOR ACTIVATING EVENTS IS HERE!!!! *********************

window.onload = function(){

    document.getElementById("logname").addEventListener("click", function(e){
        this.value = "";
    });
    document.getElementById("logname").addEventListener("blur", function(e){
        if (this.value === "") this.value= "type log label";
    });
    document.getElementById("logname").addEventListener("keydown", function(e){
        e.stopPropagation();
    });
    document.getElementById("log").addEventListener("click", function(e){
        if (this.checked){
            var tag = document.getElementById("logname").value;
        } else CMD.stoplog(tag);
    });

}

document.onkeydown = function (evt) {
    evt = evt || window.event;

    if (evt.keyCode == 81) {
        const window = remote.getCurrentWindow();
        window.close();
    }
};

document.onreadystatechange = function () {
    if (document.readyState == "complete") {
        initClose();
    }
};

// ------------------------------

function updatePointCloud(){

    

    if (dists.length == 0) return;
    
    // else console.log("Dists length updated to " + dists.length);
    

    
    var positions = pointGeometry.attributes.position;
    if (positions === undefined){
        
        var ps = [];

        for (var ix = 0; ix < dists.length; ++ix){
            p_ix = polarToCartesian(dists[ix], angles[ix]);
            // console.log(p_ix);
            ps.push(p_ix.x, p_ix.y, p_ix.z);
        }
        pointGeometry.addAttribute('position', new THREE.Float32BufferAttribute( ps, 3 ));

        points = new THREE.Points(pointGeometry, pointMaterial);
        scene.add(points);
    } else {
        
        // scene.remove(pointsold);
        // pointsold = points.clone();
        // scene.add(pointsold);

        var ps = positions.array;

        for (var ix = 0; ix < dists.length; ++ix){
            p_ix = polarToCartesian(dists[ix], angles[ix]);
            // console.log(p_ix);
            ps[3 * ix] = p_ix.x;
            ps[3 * ix + 1] = p_ix.y;
            ps[3 * ix + 2] = p_ix.z;    
        }
    }

    dists.length = 0;
    // angles.length = 0;
    // qs.length = 0;
    pointGeometry.attributes.position.needsUpdate = true;
}

// ------------------------------

init();
animate();

