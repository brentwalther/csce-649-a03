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

  var N = this.cloths.length;
  var h = this.clock.getDelta();
  this.updateCamera(h);

  for (var i = 0; i < N; i++) {
    /*
     * take current state S
     * calculate derivative D
     * calculate S' = S + .5h * D
     * renormalize orientation
     * calculate derivative at S' = D'
     * calculate S'' = S + h * D'
     * renormalize orientation
     */

    var cloth = this.cloths[i];
    var state = cloth.getState();

    var k1 = StateUtil.multiply(this.computeDerivative(state), h);
    var k2 = StateUtil.multiply(this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k1, 0.5))), h);
    var k3 = StateUtil.multiply(this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k2, 0.5))), h);
    var k4 = StateUtil.multiply(this.computeDerivative(StateUtil.add(state, k3)), h);

    var kSum = StateUtil.add(k1, k4);
    kSum = StateUtil.add(kSum, StateUtil.multiply(k2, 2));
    kSum = StateUtil.add(kSum, StateUtil.multiply(k3, 2));
    kSum = StateUtil.multiply(kSum, 1 / 6);
    var stateNew = StateUtil.add(state, kSum);
    // var stateNew = StateUtil.add(state, StateUtil.multiply(( this.useRK2 ? k2 : k1), h));

    this.doCollisions(cloth, state, stateNew);
    // var xn = stateNew[0];
    // var rn = stateNew[1];
    // this.normalizeMatrix4(rn);
    // var pn = stateNew[2];
    // var ln = stateNew[3];

    // cloth.x.copy(xn);
    // cloth.mesh.position.copy(xn);
    // cloth.P.copy(pn);
    // cloth.R.copy(rn);
    // cloth.L.copy(ln);
    // cloth.mesh.rotation.setFromRotationMatrix(rn);
    for (var p = 0; p < stateNew.length; p++) {
      if (p > -1 && p < 2 || p > 28 && p < 31) { continue; }
      var updates = stateNew[p];
      cloth.xs[p].copy(updates[0]);
      cloth.vs[p].copy(updates[1]);
      cloth.geometry.vertices[p].copy(updates[0]);
    }
    cloth.geometry.verticesNeedUpdate = true;
  }
};

Simulation.prototype.integrate = function(state, statePrime, h) {
  var stateNew = [];

  for (var i = 0; i < 4; i++) {
    stateNew.push(state[i].clone().add(statePrime[i].clone().multiplyScalar(h)));
  }

  return stateNew;
}

Simulation.prototype.computeDerivative = function(state) {
  /*
   * v = 1/m * p
   * I' = R * I'0 * RT
   * w = I' * L
   * compute w*
   * compute w* R
   * get all Forces
   * get all Torque
   */

  var derivative = [];
  var N = state.length;

  for (var i = 0; i < N; i++) {
    derivative.push([state[i][1], new THREE.Vector3(0, 0, -1)]);

    if (this.tryingToTear && i == this.tearLoc && this.tearSteps++ < 6) {
      derivative[i][1] = new THREE.Vector3(0, 10000, 0);
      // console.log("Tear steps: " + this.tearSteps);
    } else if (!this.tryingToTear) {
      this.tearSteps = 0;
    }
  }

  var structLen = .3333333;
  var shearLen = Math.sqrt(structLen * structLen * 2);
  var kij = 500;
  var dij = 3;
  var structuralNeighbors = [
    [1,  0],
    [-1, 0],
    [0,  1],
    [0, -1]
  ];

  var shearNeighbors = [
    [-1, -1],
    [-1,  1],
    [1,  -1],
    [1,   1]
  ];

  for (var i = 0; i < N; i++) {
    var x = state[i][0];
    var xi = i % 31;
    var xj = Math.floor(i / 31);

    // sum forces from all the neighbors
    for (var n = 0; n < 4; n++) {
      var neighbor = structuralNeighbors[n];
      var pi = xi + neighbor[0];
      var pj = xj + neighbor[1];

      if (pi >= 0 && pi < 31 && pj >= 0 && pj < 31) {
        var idx = pj * 31 + pi;
        // if (idx < i) { continue; }

        if (this.broken[i] && this.broken[i][idx]) {
          continue;
        }

        var idxer = state[idx];
        if (!idxer) {
          console.log('help!');
        }
        var xn = idxer[0];

        // compute the springy forces
        var xij = xn.clone().sub(x);
        var lij = xij.length();
        if (lij > 2) {
          console.log("Breaking link between node %d and %d are at length %f", i, idx, lij);

          if (!this.broken[i]) { this.broken[i] = []; }
          this.broken[i][idx] = true;

          if (!this.broken[idx]) { this.broken[idx] = []; }
          this.broken[idx][i] = true;


        }
        var uij = xij.clone().normalize();

        var fis = uij.clone().multiplyScalar((lij - structLen) * kij);
        derivative[i][1].add(fis);
        derivative[idx][1].sub(fis);

        var vi = state[i][1];
        var vj = state[idx][1];
        var fid = uij.clone().multiplyScalar((uij.dot(vj.clone().sub(vi))) * -dij);
        derivative[i][1].sub(fid);
        derivative[idx][1].add(fid);
      }
    }

    for (var n = 0; n < 4; n++) {
      var neighbor = shearNeighbors[n];
      var pi = xi + neighbor[0];
      var pj = xj + neighbor[1];

      if (pi >= 0 && pi < 31 && pj >= 0 && pj < 31) {
        var idx = pj * 31 + pi;
        if (idx < 0 || idx >= state.length) { console.log("E11"); }
        var idxer = state[idx];
        if (!idxer) {
          console.log('help!');
        }
        var xn = idxer[0];

        // compute the springy forces
        var xij = xn.clone().sub(x);
        var lij = xij.length();
        var uij = xij.clone().normalize();

        var fis = uij.clone().multiplyScalar((lij - shearLen) * 100);
        derivative[i][1].add(fis);
        derivative[idx][1].sub(fis);

        var vi = state[i][1];
        var vj = state[idx][1];
        var fid = uij.clone().multiplyScalar((uij.dot(vj.clone().sub(vi))) * -dij);
        derivative[i][1].sub(fid);
        derivative[idx][1].add(fid);
      }
    }
  }

  // var loc = 3.464;
  // for (var i = 0; i < G.BoxCross.length && this.shouldComputeCrossSprings; i++) {
  //   var vs = G.BoxCross[i];
  //   var xi = state[vs[0]];
  //   var xj = state[vs[1]];

  //   var xij = xj.clone().sub(xi);
  //   var lij = xij.length();
  //   var uij = xij.clone().normalize();

  //   var fis = uij.clone().multiplyScalar((lij - loc) * kij);
  //   derivative[vs[0]].add(fis);
  //   derivative[vs[1]].sub(fis);

  //   var vi = state[vs[0] + N];
  //   var vj = state[vs[1] + N];
  //   var fid = uij.clone().multiplyScalar((uij.dot(vj.clone().sub(vi))) * dij);
  //   derivative[vs[0]].add(fid);
  //   derivative[vs[1]].sub(fid);
  // }

  return derivative;
};

Simulation.prototype.doCollisions = function(body, state, stateNew) {
  return;
  var N = body.geometry.vertices.length;
  var numCollisions = 0;

  var x = state[0];
  var R = state[1].clone();
  var xn = stateNew[0];
  var Rn = stateNew[1].clone();

  for (var pi = 0; pi < N; pi++) {
    var v = body.geometry.vertices[pi].clone();
    var p = v.clone().applyMatrix4(R).add(x);
    var pn = v.clone().applyMatrix4(Rn).add(xn);

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
          numCollisions++;
          if (!collision || f < collision.f) {
            var D = this.computeDerivative(state);
            collision = {
                r: p.clone().sub(x),
                qDot: D[0].add(v.clone().applyMatrix4(D[1])),
                Iprime: state[5].clone(),
                mass: state[4],
                normal: normal.clone().normalize()
            };
          }
        }
      }
    }

    if (collision) {
      var jn = collision.qDot.clone().dot(collision.normal) * (-1 - 0.5);
      var jd = (1 / collision.mass) + collision.normal.clone().dot(collision.r.clone().cross(collision.normal).cross(collision.r.clone()).applyMatrix4(collision.Iprime));
      var j = jn / jd;

      var deltaP = collision.normal.clone().multiplyScalar(j);
      var deltaL = collision.r.clone().cross(collision.normal).multiplyScalar(j);
      stateNew[0] = state[0];
      stateNew[1] = state[1];
      stateNew[2] = state[2].add(deltaP);
      stateNew[3] = state[3].add(deltaL);
    }
  }
  if (numCollisions > 0) {
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
  // this.addCollidableMesh(wireCube);

  var cloths = [];
  cloths.push(new Cloth());
  for (var i = 0; i < cloths.length; i++) {
    this.scene.add(cloths[i].mesh);
  }

  this.cloths = cloths;
  this.broken = [];
  this.tryingToTear = false;
  this.tearSteps = 0;

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

  if (keyboard.pressed("T")) {
    if (!this.tryingToTear) {
      var y = Math.floor(Math.random() * 20 + 10);
      var x = Math.floor(Math.random() * 20 + 10);
      this.tearLoc = y * 31 + x;
    }
    this.tryingToTear = true;
  } else {
    this.tryingToTear = false;
  }
}

Simulation.prototype.setIntegrationMethod = function() {
  this.useRK2 = document.getElementById('useRK2').checked;
}

Simulation.prototype.setCrossSprings = function() {
  this.shouldComputeCrossSprings = document.getElementById('useCrossSprings').checked;
}


//---------------------
// STATE CLASS
//---------------------

var Cloth = function(i, j) {
  this.s = 10;
  this.n = 30;
  var mass = 1;

  var rotationMatrix = new THREE.Matrix4();
  rotationMatrix.makeRotationX(Math.PI / 2);
  this.geometry = new THREE.PlaneGeometry(this.s, this.s, this.n, this.n);
  this.geometry.applyMatrix(rotationMatrix);
  this.material = new THREE.MeshBasicMaterial({ color: 0xdddddd, wireframe: true, side: THREE.DoubleSide });
  this.mesh = new THREE.Mesh(this.geometry, this.material);
  // this.mesh.rotateOnAxis(new THREE.Vector3(1, 0, 0), -Math.PI / 2);
  // this.mesh.translateZ(-s / 2);

  this.xs = [];
  this.vs = [];
  for (var i = 0; i < this.geometry.vertices.length; i++) {
    this.xs.push(this.geometry.vertices[i]);
    this.vs.push(new THREE.Vector3());
  }
  this.x = new THREE.Vector3();
  // this.R = new THREE.Matrix4();
  this.v = new THREE.Vector3();//new THREE.Vector3(5 - Math.random() * 10, 5 - Math.random() * 10, 5 - Math.random() * 10);
  // this.L = (
    // document.getElementById('randomRotation').checked
    // ? new THREE.Vector3(1 - Math.random() * 2, 1 - Math.random() * 2, 1 - Math.random() * 2)
    // : new THREE.Vector3()
  // );
  this.i = i;
  this.j = j;

  // Construct the moment of intertia tensor matrix as a 4x4 with the bottom right element being
  // one. We must do it this way because THREE.Matrix4().getInverse() expects a Matrix4 object.
  // var tensorScalar = (1 / 12) * mass * (s * s + s * s);
  // this.I = new THREE.Matrix4().multiplyScalar(tensorScalar);
  // this.I.elements[15] = 1;

  this.mass = mass;
  // this.Iprime = new THREE.Matrix4().getInverse(this.I);
};

Cloth.prototype.getState = function() {
  var state = [];
  for (var i = 0; i < this.xs.length; i++) {
    state.push([this.xs[i], this.vs[i]]);
  }
  return state;
};

var StateUtil = {
  add: function(state1, state2) {
    var newState = [];
    for (var i = 0; i < state1.length; i++) {
      newState.push(state1[i].map(function (e, j) { return e.clone().add(state2[i][j]); }));
    }
    // newState.push(state1[4]);
    // newState.push(state1[5]);
    return newState;
  },

  multiply: function(state, scalar) {
    var newState = [];
    for (var i = 0; i < state.length; i++) {
      // if (i != 1) {
        newState.push(state[i].map(function (e) { return e.clone().multiplyScalar(scalar); }));
      // } else {
        // var q1 = state[i];
        // newState[i] = new THREE.Quaternion(q1.x * scalar, q1.y * scalar, q1.z * scalar, q1.w * scalar);
      // }
    }
    // newState.push(state[4]);
    // newState.push(state[5]);
    return newState;
  }
};
