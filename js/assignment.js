var Simulation = function() {
  this.scene = new THREE.Scene();
  this.renderer = new THREE.WebGLRenderer();
  this.renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.appendChild(this.renderer.domElement);

  this.reset();
};

Simulation.prototype.render = function() {
  this.renderer.render(this.scene, this.camera);
  requestAnimationFrame(this.render.bind(this));
};

Simulation.prototype.start = function() {
  requestAnimationFrame(this.render.bind(this));

  this.simFrameCount = 0;
  this.lastSimFrameTime = performance.now();
  this.oldTime = performance.now();
  this.simulatorInterval = setInterval(this.simulate.bind(this), 10);
};

Simulation.prototype.stop = function() {
  clearInterval(this.simulatorInterval);
};

Simulation.prototype.simulate = function() {
  this.simFrameCount++;
  var newTime = performance.now();
  var elapsedMs = newTime - this.oldTime;
  if (h < 10) {
    return;
  }

  if (newTime - this.lastSimFrameTime > 1000) {
    console.log("sim frames: " + this.simFrameCount);
    this.simFrameCount = 0;
    this.lastSimFrameTime = performance.now();
  }

  var h = elapsedMs / 1000;

  for (var i = 0; i < this.boids.length; i++) {
    var sphere = this.boids[i];
    sphere.integrate(this.vertices, this.faces, this.a, h);
 }

  this.oldTime = newTime;
};

Simulation.prototype.reset = function() {
  this.scene = new THREE.Scene();
  this.vertices = [];
  this.faces = [];

  var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  camera.position.set(5, 10, 10);
  camera.up = new THREE.Vector3(0,0,1);
  camera.lookAt(new THREE.Vector3(0,0,0));
  this.camera = camera;

  // Add the outer cube
  var cubeGeometry = new THREE.BoxGeometry( 10, 10, 10 );
  var cubeMaterial = new THREE.MeshLambertMaterial(
      {
        color: 0xdddddd,
        specular: 0x009900,
        shininess: 30,
        shading: THREE.SmoothShading,
        side: THREE.DoubleSide,
        opacity: 0.3,
        transparent: true
      });
  this.cube = new THREE.Mesh(cubeGeometry, cubeMaterial);
  this.addCollidableMesh(this.cube);

  // Add the boids to the simulation
  this.boids = [];
  for (var i = 0; i < 1; i++) {
    var sphere = new Particle(this.scene);
    this.boids.push(sphere);
  }

  // Add a light
  var light = new THREE.PointLight(0xffffff, 1, 40);
  light.position.set(8, 10, 10);
  this.scene.add(light);

  this.a = new THREE.Vector3(0, 0, 0);
  //this.a = new THREE.Vector3(0, 0, -9.8);
};

Simulation.prototype.addCollidableMesh = function(mesh) {
  this.scene.add(mesh);
  var len = mesh.geometry.faces.length;
  var faceOffset = this.vertices.length;
  for (var i = 0; i < len; i++) {
    var face = mesh.geometry.faces[i];
    this.faces.push(
        new THREE.Face3(
            face.a + faceOffset,
            face.b + faceOffset,
            face.c + faceOffset,
            face.normal,
            face.color,
            face.materialIndex
        )
    );
  }
  Array.prototype.push.apply(this.vertices, mesh.geometry.vertices);
};
