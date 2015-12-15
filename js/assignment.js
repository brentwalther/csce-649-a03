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

    for (var p = 0; p < stateNew.length; p++) {
      if (p == 0 || p == 1 || p == 2 || p == 27 || p == 28 || p == 29) { continue; }
      var updates = stateNew[p];
      cloth.xs[p].copy(updates[0]);
      cloth.vs[p].copy(updates[1]);
    }
    cloth.updateMeshes();
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
  var kij = 1000;
  var dij = 10;
  var structuralNeighbors = [
    [1,  0],
    [0,  1],
  ];

  var shearNeighbors = [
    [1,  -1],
    [1,   1]
  ];

  // return derivative;

  for (var i = 0; i < N; i++) {
    var x = state[i][0];
    var xi = i % 30;
    var xj = Math.floor(i / 30);

    // sum forces from all the neighbors
    for (var n = 0; n < 2; n++) {
      var neighbor = structuralNeighbors[n];
      var pi = xi + neighbor[0];
      var pj = xj + neighbor[1];

      if (pi >= 0 && pi < 30 && pj >= 0 && pj < 30) {
        var idx = pj * 30 + pi;

        if (idx < i) { continue; }
        if (this.broken[i] && this.broken[i][idx] || this.broken[idx] && this.broken[idx][i]) {
          continue;
        }

        var idxer = state[idx];
        var xn = idxer[0];

        // compute the springy forces
        var xij = xn.clone().sub(x);
        var lij = xij.length();
        if (lij > 2) {
          console.log("Breaking link between node %d and %d are at length %f", i, idx, lij);
          this.breakLink(i, idx);
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

    for (var n = 0; n < 2; n++) {
      var neighbor = shearNeighbors[n];
      var pi = xi + neighbor[0];
      var pj = xj + neighbor[1];

      if (pi >= 0 && pi < 30 && pj >= 0 && pj < 30) {
        var idx = pj * 30 + pi;

        if (this.broken[i] && this.broken[i][idx] || this.broken[idx] && this.broken[idx][i]) {
          continue;
        }

        var idxer = state[idx];
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
}

Simulation.prototype.breakLink = function(v1, v2) {
  var cloth = this.cloths[0];

  var now = performance.now();
  // for (var f = 0; f < cloth.geometry.faces.length; f++) {
  //   var face = cloth.geometry.faces[f].clone();
  //   // find face and update vertex
  //   var hasV1 = (v1 == face.a || v1 == face.b || v1 == face.c);
  //   var hasV2 = (v2 == face.a || v2 == face.b || v2 == face.c);

  //   if (/*!*/(hasV1 && hasV2)) {
  //     console.log("Need to delete face: ");
  //     console.log(face);
  //     // newFaces.push(face);
  //   }
  // }
  var x1 = v1 % 30;
  var y1 = Math.floor(v1 / 30);
  var x2 = v2 % 30;
  var y2 = Math.floor(v2 / 30);

  if (x1 == x2) {
    var y = Math.min(y1, y2);
    cloth.meshes[y][x1][0].visible = false;
    if (y != 29 && x != 29) {
      cloth.meshes[y][x1][1].visible = false;
    }
  } else if (y1 == y2) {
    var x = Math.min(x1, x2);
    cloth.meshes[y1][x][0].visible = false;
    if (y != 29 && x != 29) {
      cloth.meshes[y1][x][1].visible = false;
    }
  } else {
    alert('idk');
  }
  console.log("Took %f msecs to scan faces.", performance.now() - now);
  // cloth.geometry.faces = newFaces;
  // cloth.geometry.elementsNeedUpdate = true;

  if (!this.broken[v1]) { this.broken[v1] = []; }
  this.broken[v1][v2] = true;

  if (!this.broken[v2]) { this.broken[v2] = []; }
  this.broken[v2][v1] = true;
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
  cloths.push(new Cloth(this.scene));

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

//---------------------
// STATE CLASS
//---------------------

var Cloth = function(scene) {
  this.s = 10;
  this.n = 30;
  var mass = 1;

  var step = this.s / this.n;

  var rotationMatrix = new THREE.Matrix4();
  rotationMatrix.makeRotationX(Math.PI / 2);

  // Create all the meshes for the cloth pieces
  var meshes = [];
  for (var y = 0; y < this.n; y++) {
    meshes[y] = [];
    for (var x = 0; x < this.n; x++) {
      meshes[y][x] = [];

      var topLeft = new THREE.Vector3(5 - (x * step), 0, 5 - (y * step));
      var right = new THREE.Vector3(5 - ((x + 1) * step), 0,  5 - (y * step));
      var bottom = new THREE.Vector3(5 - (x * step), 0, 5 - ((y + 1) * step));

      var mat = new THREE.LineBasicMaterial({ color: 0xff0000 });
      var hg = new THREE.Geometry();
      hg.vertices.push(topLeft, right);
      var vg = new THREE.Geometry();
      vg.vertices.push(topLeft, bottom);

      var horMesh = new THREE.Line(hg, mat);
      var verMesh = new THREE.Line(vg, mat);

      if ((x + 1) != this.n) {
        scene.add(horMesh);
        meshes[y][x].push(horMesh);
      }
      if ((y + 1) != this.n) {
        scene.add(verMesh);
        meshes[y][x].push(verMesh);
      }
    }
  }

  this.xs = [];
  this.vs = [];
  for (var i = 0; i < meshes.length; i++) {
    for (var j = 0; j < meshes[i].length; j++) {
      var mesh = meshes[i][j][0];
      // x will be undefined on the bottom corner
      var x = (mesh === undefined ? meshes[i][j-1][0].geometry.vertices[1] : meshes[i][j][0].geometry.vertices[0]);
      this.xs.push(x);
      this.vs.push(new THREE.Vector3());
    }
  }

  this.meshes = meshes;
  this.mass = mass;
};

Cloth.prototype.updateMeshes = function() {
  for (var i = 0; i < this.xs.length; i++) {
    var pos = this.xs[i];

    var x = i % 30;
    var y = Math.floor(i / 30);

    var mesh = this.meshes[y][x][0];
    if (mesh) {
      mesh.geometry.vertices[0].copy(pos);
      mesh.geometry.verticesNeedUpdate = true;
    }
    mesh = this.meshes[y][x][1];
    if (mesh) {
      mesh.geometry.vertices[0].copy(pos);
      mesh.geometry.verticesNeedUpdate = true;
    }
    if (y > 0) {
      mesh = this.meshes[y-1][x][ (x == 29 ? 0 : 1) ];
      if (mesh) {
        mesh.geometry.vertices[1].copy(pos);
        mesh.geometry.verticesNeedUpdate = true;
      }
    }
    if (x > 0) {
      mesh = this.meshes[y][x-1][0];
      if (mesh) {
        mesh.geometry.vertices[1].copy(pos);
        mesh.geometry.verticesNeedUpdate = true;
      }
    }
  }
}

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
      newState.push([
        state1[i][0].clone().add(state2[i][0]),
        state1[i][1].clone().add(state2[i][1])
      ]);
    }
    return newState;
  },

  multiply: function(state, scalar) {
    var newState = [];
    for (var i = 0; i < state.length; i++) {
      newState.push([state[i][0].clone().multiplyScalar(scalar), state[i][1].clone().multiplyScalar(scalar)]);
    }
    return newState;
  }
};
