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

  var state = [];
  var N = this.rigidBodies.length;

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

    var body = this.rigidBodies[i];
    state = body.getState();

    var k1 = this.computeDerivative(state);
    var k2 = this.computeDerivative(StateUtil.add(state, StateUtil.multiply(k1, h * 0.5)));
    var stateNew = StateUtil.add(state, StateUtil.multiply(( this.useRK2 ? k2 : k1), h));

    this.doCollisions(body, state, stateNew);
    var xn = stateNew[0];
    var rn = stateNew[1];
    this.normalizeMatrix4(rn);
    var pn = stateNew[2];
    var ln = stateNew[3];

    body.x.copy(xn);
    body.mesh.position.copy(xn);
    body.P.copy(pn);
    body.R.copy(rn);
    body.L.copy(ln);
    body.mesh.rotation.setFromRotationMatrix(rn);
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

  var m = state[4];
  var v = state[2].clone().multiplyScalar(1 / m);
  var R = state[1];
  var Iprime = R.clone().multiply(state[5]).multiply(R.clone().transpose());
  var w = state[3].clone().applyMatrix4(Iprime);
  var wStar = new THREE.Matrix4();
  wStar.set(
    0,    -w.z, w.y,  0,
    w.z,  0,    -w.x, 0,
    -w.y, w.x,  0,    0,
    0,    0,    0,    1
  );
  wStar.multiply(R);

  var statePrime = [
    v,
    wStar,
    new THREE.Vector3(0, 0, -5), // forces
    new THREE.Vector3(0, 0, 0),  // torque
    state[4], // mass
    state[5]  // moment of inertia inverse
  ];

  return statePrime;
};

Simulation.prototype.doCollisions = function(body, state, stateNew) {
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
  this.addCollidableMesh(wireCube);

  var rigidBodies = [];
  rigidBodies.push(new Polyhedron());
  for (var i = 0; i < rigidBodies.length; i++) {
    this.scene.add(rigidBodies[i].mesh);
  }

  this.rigidBodies = rigidBodies;

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

Simulation.prototype.normalizeMatrix4 = function(matrix) {
  for (var i = 0; i < 15; i += 4) {
    var sum = 0;
    for (var j = i; j < i + 4; j++) {
      sum += matrix.elements[j] * matrix.elements[j];
    }
    sum = Math.sqrt(sum);
    for (var j = i; j < i + 4; j++) {
      matrix.elements[j] /= sum;
    }
  }
};

Simulation.prototype.setIntegrationMethod = function() {
  this.useRK2 = document.getElementById('useRK2').checked;
}

Simulation.prototype.setCrossSprings = function() {
  this.shouldComputeCrossSprings = document.getElementById('useCrossSprings').checked;
}


//---------------------
// STATE CLASS
//---------------------

var Polyhedron = function() {
  var s = 2;
  var mass = 1;

  this.geometry = new THREE.BoxGeometry(s, s, s);
  this.material = new THREE.MeshPhongMaterial({ color: 0xdddddd, specular: 0xffffff, shininess: 30, shading: THREE.SmoothShading });
  this.mesh = new THREE.Mesh(this.geometry, this.material);

  this.x = new THREE.Vector3();
  this.R = new THREE.Matrix4();
  this.P = new THREE.Vector3(5 - Math.random() * 10, 5 - Math.random() * 10, 5 - Math.random() * 10);
  this.L = (
    document.getElementById('randomRotation').checked
    ? new THREE.Vector3(1 - Math.random() * 2, 1 - Math.random() * 2, 1 - Math.random() * 2)
    : new THREE.Vector3()
  );

  // Construct the moment of intertia tensor matrix as a 4x4 with the bottom right element being
  // one. We must do it this way because THREE.Matrix4().getInverse() expects a Matrix4 object.
  var tensorScalar = (1 / 12) * mass * (s * s + s * s);
  this.I = new THREE.Matrix4().multiplyScalar(tensorScalar);
  this.I.elements[15] = 1;

  this.mass = mass;
  this.Iprime = new THREE.Matrix4().getInverse(this.I);
};

Polyhedron.prototype.getIprime = function() {
  var mat = R.clone().multiply(this.Iprime).multiply(R.clone().transpose());
  return new THREE.Matrix4().getInverse(mat);
};

Polyhedron.prototype.getState = function() {
  return [
    this.x.clone(),
    this.R.clone(),
    this.P.clone(),
    this.L.clone(),
    this.mass,
    this.Iprime.clone()
  ];
};

var StateUtil = {
  add: function(state1, state2) {
    var state3 = [ 0, 0, 0, 0 ];
    for (var i = 0; i < 4; i++) {
      if (i != 1) {
        state3[i] = state1[i].clone().add(state2[i]);
      } else {
        var nm = new THREE.Matrix4();
        for (var b = 0; b < state1[i].elements.length; b++) {
          nm.elements[b] = state1[i].elements[b] + state2[i].elements[b];
        }
        state3[i] = nm;
        // var q1 = state1[i];
        // var q2 = state2[i];
        // state3[i] = new THREE.Quaternion(q1.x + q2.x, q1.y + q2.y, q1.z + q2.z, q1.w + q2.w);
      }
    }
    state3.push(state1[4]);
    state3.push(state1[5]);
    return state3;
  },

  multiply: function(state, scalar) {
    var state2 = [0, 0, 0, 0]
    for (var i = 0; i < 4; i++) {
      // if (i != 1) {
        state2[i] = state[i].clone().multiplyScalar(scalar);
      // } else {
        // var q1 = state[i];
        // state2[i] = new THREE.Quaternion(q1.x * scalar, q1.y * scalar, q1.z * scalar, q1.w * scalar);
      // }
    }
    state2.push(state[4]);
    state2.push(state[5]);
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
