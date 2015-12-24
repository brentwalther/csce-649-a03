var Constants = {
  NUM_BOIDS: 1,
  NUM_LEAD_BOIDS: 0,
  NUM_CUBES: 0,
  G: 0.2
};

var Simulation = function() {
  this.scene = new THREE.Scene();
  this.renderer = new THREE.WebGLRenderer();
  this.renderer.setSize(window.innerWidth, window.innerHeight);
  document.body.querySelector('#canvasSpot').appendChild(this.renderer.domElement);

  this.integrationMethod = 'euler';
  this.shouldComputeCrossSprings = true;

  this.reset();
  THREEx.WindowResize(this.renderer, this.camera);
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

  var positions = [];
  var velocities = [];
  var N = this.boids.length;

  for (var i = 0; i < N; i++) {
    var boid = this.boids[i];
    positions.push(boid.p.clone());
    velocities.push(boid.v.clone());
  }

  var state = [];
  Array.prototype.push.apply(state, positions);
  Array.prototype.push.apply(state, velocities);


  var h = this.clock.getDelta();
  this.updateCamera(h);

  var k1 = this.computeDerivative(state);
  var stateNew = this.integrate(state, k1, h);
  var wasCollision = this.doCollisions(state, stateNew);
  if (!wasCollision && this.integrationMethod !== 'euler') {
    var k2 = this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k1, h * 0.5)));
    var k3 = this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k2, h * 0.5)));
    var k4 = this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k3, h)));

    var kSum = StateUtil.add(k1, k4);
    kSum = StateUtil.add(kSum, StateUtil.multiply(k2, 2));
    kSum = StateUtil.add(kSum, StateUtil.multiply(k3, 2));
    kSum = StateUtil.multiply(kSum, h / 6);
    stateNew = StateUtil.add(state, kSum);
    this.doCollisions(state, stateNew);
  }

  for (var i = 0; i < N; i++) {
    var boid = this.boids[i];
    var pn = stateNew[i];
    var vn = stateNew[i + N];

    boid.p.copy(pn);
    boid.mesh.position.copy(pn);
    boid.v.copy(vn);
    this.springCubeGeos[boid.g].vertices[i].copy(pn);
    this.springCubeGeos[boid.g].verticesNeedUpdate = true;
  }
};

Simulation.prototype.integrate = function(state, statePrime, h) {
  var stateNew = [];

  for (var i = 0; i < state.length; i++) {
    stateNew.push(state[i].clone().add(statePrime[i].clone().multiplyScalar(h)));
  }

  return stateNew;
}

Simulation.prototype.computeDerivative = function(state) {
  var statePrime = [];

  var N = state.length / 2;
  for (var i = 0; i < N; i++) {
    statePrime.push(state[i + N].clone());
  }
  var forces = this.getForces(state);
  for (var i = 0; i < N; i++) {
    statePrime.push(forces[i]);
  }

  return statePrime;
};

Simulation.prototype.getForces = function(state) {
  var forces = [];
  var N = state.length / 2;

  for (var i = 0; i < N; i++) {
    forces.push(new THREE.Vector3(0, 0, -9.8));
  }

  var lo = 2;
  var kij = 200;
  var dij = 8;

  for (var i = 0; i < G.Box.length; i++) {
    var vs = G.Box[i];
    var xi = state[vs[0]];
    var xj = state[vs[1]];

    var xij = xj.clone().sub(xi);
    var lij = xij.length();
    var uij = xij.clone().normalize();

    var fis = uij.clone().multiplyScalar((lij - lo) * kij);
    forces[vs[0]].add(fis);
    forces[vs[1]].sub(fis);

    var vi = state[vs[0] + N];
    var vj = state[vs[1] + N];
    var fid = uij.clone().multiplyScalar((uij.dot(vj.clone().sub(vi))) * dij);
    forces[vs[0]].add(fid);
    forces[vs[1]].sub(fid);
  }

  var loc = 3.464;
  for (var i = 0; i < G.BoxCross.length && this.shouldComputeCrossSprings; i++) {
    var vs = G.BoxCross[i];
    var xi = state[vs[0]];
    var xj = state[vs[1]];

    var xij = xj.clone().sub(xi);
    var lij = xij.length();
    var uij = xij.clone().normalize();

    var fis = uij.clone().multiplyScalar((lij - loc) * kij);
    forces[vs[0]].add(fis);
    forces[vs[1]].sub(fis);

    var vi = state[vs[0] + N];
    var vj = state[vs[1] + N];
    var fid = uij.clone().multiplyScalar((uij.dot(vj.clone().sub(vi))) * dij);
    forces[vs[0]].add(fid);
    forces[vs[1]].sub(fid);
  }

  return forces;
};

Simulation.prototype.doCollisions = function(state, stateNew) {
  var N = state.length / 2;
  var wasCollision = false;
  for (var pi = 0; pi < N; pi++) {
    var p = state[pi];
    var pn = stateNew[pi];

    var collision = undefined;
    for (var i = 0; i < this.faces.length; i++) {
      var face = this.faces[i];
      var vertex = this.vertices[face.a];
      var normal = face.normal;

      var dn = p.clone().sub(vertex).dot(normal);
      var dn1 = pn.clone().sub(vertex).dot(normal);

      if ((dn < 0) != (dn1 < 0)) {
        var f = (dn / (dn - dn1));

        var collisionPosition = p.clone().add(pn.clone().sub(p).multiplyScalar(f));

        var v1 = this.vertices[face.a];
        var v2 = this.vertices[face.b];
        var v3 = this.vertices[face.c];

        if (Triangle.isContained(collisionPosition, normal, v1, v2, v3)) {
          if (!collision || f < collision.f) {
            collision = {
                f: f,
                dn1: dn1,
                normal: normal.clone()
            };
          }
        }
      }
    }

    if (collision) {
      var vc = stateNew[pi + N].clone();
      pn = collision.pn;
      var vn = collision.normal.clone().multiplyScalar(vc.dot(collision.normal));
      var vt = vc.sub(vn);
      vt.sub(vt.clone().normalize().multiplyScalar(Math.min(Coefficients.FRICTION * vn.length(), vt.length())));

      vn.multiplyScalar(-Coefficients.RESTITUTION).add(vt);

      stateNew[pi + N].copy(vn);
      stateNew[pi].sub(collision.normal.clone().normalize().multiplyScalar(collision.dn1 * (1 + Coefficients.RESTITUTION)));
      wasCollision = true;
    }
  }
  if (wasCollision) {
    return true;
  }

  // for (var i = 0; i < G.Box.length; i++) {
  //   var vs = G.Box[i];
  //   var xi = state[vs[0]];
  //   var xj = state[vs[1]];

  //   var e2 = xi.clone().sub(xj);

  //   for (var i = 0; i < this.faces.length; i++) {
  //     var v1 = this.vertices[face.a];
  //     var v2 = this.vertices[face.b];
  //     var v3 = this.vertices[face.c];
  //     var edges = [];
  //     edges.push(v1.clone().sub(v2));
  //     edges.push(v2.clone().sub(v3));
  //     edges.push(v3.clone().sub(v1));
  //     edges.push(v1);
  //     edges.push(v2);
  //     edges.push(v3);

  //     for (var ee = 0; ee < 3; ee++) {
  //       var e1 = edges[ee];
  //       var q = xi.clone().sub(edges[ee+3]);
  //       var n = e2.clone().cross(e1).normalize();

  //       var s = q.dot(e2.clone().normalize().cross(n)) / e1.dot(e2.clone().normalize().cross(n));
  //       var t = - q.dot(e1.clone().normalize().cross(n)) / e2.dot(e1.clone().normalize().cross(n));

  //       var dn = xi.clone().sub(edges[ee+3]).dot(n);
  //       var dn1 = stateNew[vs[0]].clone().sub(edges[ee+3]).dot(n);
  //       if (s < 0 && t < 0 && ((dn < 0) != (dn1 < 0))) {
  //         var a = 1;
  //       }
  //     }
  //   }
  // }
}

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
  this.addCollidableMesh(wireCube);

  this.springCubeGeos = [];
  this.springCubeGeos.push(new THREE.BoxGeometry(2, 2, 2));
  var springCubeMat = new THREE.MeshPhongMaterial({ color: 0xdddddd, specular: 0xffffff, shininess: 30, shading: THREE.SmoothShading });
  var springCube = new THREE.Mesh(this.springCubeGeos[0], springCubeMat);
  if (document.getElementById('randomRotation').checked) {
    springCube.rotateX(1 - Math.random()*2);
    springCube.rotateY(1 - Math.random()*2);
    springCube.updateMatrix();
    springCube.geometry.applyMatrix( springCube.matrix );
    springCube.matrix.identity();
    springCube.position.set( 0, 0, 0 );
    springCube.rotation.set( 0, 0, 0 );
    springCube.scale.set( 1, 1, 1 );
  }
  this.scene.add(springCube);

  // var planeGeometry = new THREE.PlaneGeometry(10, 2);
  // var planeMaterial = new THREE.MeshBasicMaterial( {color: 0xffff00, side: THREE.DoubleSide} );
  // var plane = new THREE.Mesh( planeGeometry, planeMaterial );
  // plane.translateZ(-4);
  // plane.rotateX(3.14/2);
  // plane.updateMatrix();
  // plane.geometry.applyMatrix( plane.matrix );
  // plane.matrix.identity();
  // plane.position.set( 0, 0, 0 );
  // plane.rotation.set( 0, 0, 0 );
  // plane.scale.set( 1, 1, 1 );
  // this.addCollidableMesh(plane);

  // Add the boids to the simulation
  this.boids = [];
  for (var i = 0; i < this.springCubeGeos[0].vertices.length; i++) {
    var boid = new Particle(this.scene, 0);
    boid.g = 0;
    boid.p = this.springCubeGeos[0].vertices[i];
    boid.v = new THREE.Vector3(0, 1 - Math.random() * 2, 0);
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
  camera.position.set(7, 9, 20);
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

  var relativeCameraOffset = new THREE.Vector3(0,15,0);

  var cameraOffset = relativeCameraOffset.applyMatrix4(this.cameraPos.matrixWorld);

  this.camera.position.x = cameraOffset.x;
  this.camera.position.y = cameraOffset.y;
  this.camera.position.z = cameraOffset.z;
  this.camera.lookAt(this.cameraPos.position);
}

Simulation.prototype.setIntegrationMethod = function() {
  if (document.getElementById('useRK4').checked) {
    this.integrationMethod = 'rk4';
  } else {
    this.integrationMethod = 'euler';
  }
}

Simulation.prototype.setCrossSprings = function() {
  this.shouldComputeCrossSprings = document.getElementById('useCrossSprings').checked;
}


//---------------------
// STATE CLASS
//---------------------

var StateUtil = {
  add: function(state1, state2) {
    var state3 = []
    for (var i = 0; i < state1.length; i++) {
      state3.push(state1[i].clone().add(state2[i]));
    }
    return state3;
  },

  multiply: function(state, scalar) {
    var state2 = []
    for (var i = 0; i < state.length; i++) {
      state2.push(state[i].clone().multiplyScalar(scalar));
    }
    return state2;
  }
};

var G = {
  Box: [[0, 1],
        [0, 2],
        [0, 5],
        [1, 3],
        [1, 4],
        [2, 3],
        [2, 7],
        [3, 6],
        [4, 5],
        [4, 6],
        [5, 7],
        [6, 7]],
  BoxCross: [[0, 6],
             [1, 7],
             [2, 4],
             [3, 5]]
};
