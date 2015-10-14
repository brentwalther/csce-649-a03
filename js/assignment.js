var Constants = {
  NUM_BOIDS: 50,
  NUM_LEAD_BOIDS: 3,
  NUM_CUBES: 4,
  G: 0.2
};

var Simulation = function() {
  this.scene = new THREE.Scene();
  this.renderer = new THREE.WebGLRenderer();
  this.renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.querySelector('#canvasSpot').appendChild(this.renderer.domElement);

  this.reset();
};

Simulation.prototype.render = function() {
  this.renderer.render(this.scene, this.camera);
  requestAnimationFrame(this.render.bind(this));
};

Simulation.prototype.start = function() {
  requestAnimationFrame(this.render.bind(this));
  this.simulatorInterval = setInterval(this.simulate.bind(this), 10);
};

Simulation.prototype.stop = function() {
  clearInterval(this.simulatorInterval);
};

Simulation.prototype.simulate = function() {

  var accelerations = [];

  for (var i = 0; i < this.boids.length; i++) {
    accelerations.push(new THREE.Vector3());
    var a = accelerations[i];
    var boid = this.boids[i];

    for (var zz = 0; zz < this.cubes.length; zz++) {
      var cube = this.cubes[zz];
      var c = cube.position.clone();
      var e = cube.geometry.boundingSphere.radius;

      var distanceToCenter = c.sub(boid.p);
      var distanceToPlane = distanceToCenter.length() - e;
      var uHat = distanceToCenter.clone().normalize();
      var vu = boid.v.dot(uHat);
      var t = distanceToPlane / vu;

      var avoidVector = boid.v.clone().sub(uHat.clone().multiplyScalar(vu));
      var m = avoidVector.length() * t;
      if (m <= e && t > 0) {
        avoidVector.normalize();
        var am = 2 * (e - m) / (t * t);
        a.add(avoidVector.multiplyScalar(am));
      }
    }

    for (var j = 0; j < this.boids.length; j++) {
      if (j == i) continue;

      var otherBoid = this.boids[j];

      var u = otherBoid.p.clone().sub(boid.p);
      var distance = u.length();
      var uHat = u.clone().normalize();

      if (boid.isLeadBoid) {
        if (boid.isChasing == j) {
          boid.v.copy(u.clone().multiplyScalar(2));
        }
        // if (boid.lastUpdatedChaseBoid > this.clock.getElapsedTime() + 2) {
        //   boid.chaseNewBoid(this.clock);
        // }
        continue;
      }
      if (otherBoid.isLeadBoid && u.length() < 1) {
        a.add(u.negate().multiplyScalar(2));
      }

      var weight = 1;
      if (otherBoid.isLeadBoid) {
        weight = 5 * Math.pow(Math.E, -distance);
      } else {
        weight = Math.pow(Math.E, -distance / 2);
      }
      weight *= Math.max(boid.v.dot(uHat) / boid.v.length(), 0);

      if (weight <= 0) { continue; }

      // Collision avoidance
      var collisionAvoidanceAcceleration = uHat.clone()
          .multiplyScalar(1 / ((otherBoid.isLeadBoid ? 0.10 : 1) * distance))
          .multiplyScalar(-Coefficients.COLLISION_AVOIDANCE)
          .multiplyScalar(weight);
      a.add(collisionAvoidanceAcceleration);

      // Flock centering
      var flockCenteringAcceleration = uHat.clone()
          .multiplyScalar(distance)
          .multiplyScalar(Coefficients.FLOCK_CENTERING)
          .multiplyScalar(weight);
      if (otherBoid.isLeadBoid) {
        a.sub(flockCenteringAcceleration);
      } else if (boid.particleClass == otherBoid.particleClass) {
        a.add(flockCenteringAcceleration);
      }

      // Velocity matching
      var velocityMatchingAcceleration = otherBoid.v.clone().sub(boid.v)
          .multiplyScalar(Coefficients.VELOCITY_MATCHING)
          .multiplyScalar(weight);
      if (boid.particleClass == otherBoid.particleClass) {
        a.add(velocityMatchingAcceleration);
      }
    }
  }

  for (var j = 0; j < this.attractors.length; j++) {
    var attractor = this.attractors[j];
    for (var i = 0; i < this.boids.length; i++) {
      var boid = this.boids[i];
      var a = accelerations[i];
      var d = boid.p.clone().sub(attractor.position);
      var r = Math.max(d.length(), 1);
      if (r > 5) {
        a.add(d.normalize().multiplyScalar(-Constants.G * boid.mass * attractor.mass / (r / 2)));
      }
    }
  }

  var h = this.clock.getDelta();
  this.updateCamera(h);

  for (var i = 0; i < this.boids.length; i++) {
    var boid = this.boids[i];
    boid.integrate(this.vertices, this.faces, accelerations[i], h);
  }

  // Spin the cubes
  for (var zz = 0; zz < this.cubes.length; zz++) {
    var cube = this.cubes[zz];
    cube.rotateX(h * Math.random());
    cube.rotateY(h * Math.random());
    cube.rotateZ(h * Math.random());
  }
};

Simulation.prototype.reset = function() {
  this.clock = new THREE.Clock();
  this.scene = new THREE.Scene();
  this.keyboard = new THREEx.KeyboardState();
  this.vertices = [];
  this.faces = [];

  this.resetCamera(this.scene);

  // Add the outer cube
  var cubeGeometry = new THREE.BoxGeometry( 10, 10, 10 );
  var cubeMaterial = new THREE.MeshBasicMaterial({ wireframe: true });
  var wireCube = new THREE.Mesh(cubeGeometry, cubeMaterial)
  this.scene.add(wireCube);

  this.cubes = [];
  var cbs = 0;
  while (cbs < Constants.NUM_CUBES) {
    cubeGeometry = new THREE.BoxGeometry( 2, 2, 2 );
    cubeGeometry.computeBoundingSphere();
    cubeMaterial = new THREE.MeshPhongMaterial({ color: 0xdddddd, specular: 0xffffff, shininess: 30, shading: THREE.SmoothShading });
    this.cubes[cbs] = new THREE.Mesh(cubeGeometry, cubeMaterial);
    this.cubes[cbs].position.set(5 - Math.random() * 10, 5 - Math.random() * 10, 5 - Math.random() * 10);
    this.scene.add(this.cubes[cbs]);
    cbs++;
  }

  var attractorGeo = new THREE.OctahedronGeometry(0.2, 1);
  var attractorMat = new THREE.MeshBasicMaterial({ wireframe: true, color: 0x0000ff });
  this.scene.add(new THREE.Mesh(attractorGeo, attractorMat));

  this.attractors = [];
  this.attractors.push({ mass: 10, position: new THREE.Vector3() });

  // Add the boids to the simulation
  this.boids = [];
  for (var i = 0; i < Constants.NUM_BOIDS; i++) {
    var particleClass = (i < Constants.NUM_LEAD_BOIDS ? 0 : Math.floor(Math.random() * 3 + 1));
    var boid = new Particle(this.scene, particleClass);
    var positionNotOkay;
    do {
      boid.resetPosition();
      positionNotOkay = false;
      // Spin the cubes
      for (var zz = 0; zz < this.cubes.length; zz++) {
        var cube = this.cubes[zz];
        var c = cube.position.clone();
        var e = cube.geometry.boundingSphere.radius;

        var distanceOutside = c.sub(boid.p).length() - e;
        if (distanceOutside < 0) {
          positionNotOkay = true;
        }
      }
    } while (positionNotOkay);
    this.boids.push(boid);
  }

  // Add lights
  var light = new THREE.PointLight(0xffffff, 1, 40);
  light.position.set(8, 10, 10);
  this.scene.add(light);

  light = new THREE.AmbientLight( 0x404040 ); // soft white light
  this.scene.add( light );
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

Simulation.prototype.resetCamera = function(scene) {
  var camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
  camera.position.set(7, 9, 10);
  camera.up = new THREE.Vector3(0,0,1);
  camera.lookAt(new THREE.Vector3(0,0,0));
  this.camera = camera;

  var cameraPosGeom = new THREE.BoxGeometry(.1, .1, .1);
  var cameraPosMat = new THREE.MeshBasicMaterial();
  this.cameraPos = new THREE.Mesh(cameraPosGeom, cameraPosMat);
  this.cameraPos.position.set(0, 0, 0);
  scene.add(this.cameraPos);
}

Simulation.prototype.updateCamera = function(h) {
  var moveDistance = 10 * h; // 200 pixels per second
  var rotateAngle = Math.PI / 2 * h;   // pi/2 radians (90 degrees) per second
  var keyboard = this.keyboard;

  // local transformations

  // move forwards/backwards/left/rightwwa
  if (keyboard.pressed("W"))
    this.cameraPos.translateY(-moveDistance);
  if (keyboard.pressed("S"))
    this.cameraPos.translateY(moveDistance);
  if (keyboard.pressed("Q"))
    this.cameraPos.translateX(moveDistance);
  if (keyboard.pressed("E"))
    this.cameraPos.translateX(-moveDistance);

  // rotate left/right/up/down
  var rotation_matrix = new THREE.Matrix4().identity();
  if (keyboard.pressed("A"))
    this.cameraPos.rotateOnAxis(new THREE.Vector3(0,0,1), rotateAngle);
  if (keyboard.pressed("D"))
    this.cameraPos.rotateOnAxis(new THREE.Vector3(0,0,1), -rotateAngle);
  if (keyboard.pressed("R"))
    this.cameraPos.rotateOnAxis(new THREE.Vector3(1,0,0), rotateAngle);
  if (keyboard.pressed("F"))
    this.cameraPos.rotateOnAxis(new THREE.Vector3(1,0,0), -rotateAngle);

  if (keyboard.pressed("Z"))
  {
    this.cameraPos.position.set(0,0,0);
    this.cameraPos.rotation.set(0,0,0);
  }

  var relativeCameraOffset = new THREE.Vector3(0,5,0);

  var cameraOffset = relativeCameraOffset.applyMatrix4(this.cameraPos.matrixWorld);

  this.camera.position.x = cameraOffset.x;
  this.camera.position.y = cameraOffset.y;
  this.camera.position.z = cameraOffset.z;
  this.camera.lookAt(this.cameraPos.position);
}
