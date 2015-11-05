var Coefficients = {
  FRICTION: 0.01,
  RESTITUTION: 0.75,
  COLLISION_AVOIDANCE: 0.5,
  FLOCK_CENTERING: 0.5,
  VELOCITY_MATCHING: 1
};

var Particle = function(scene, particleClass) {
  this.identifer = Math.random().toString(36).substring(7);

  this.radius = 0.1;
  this.isLeadBoid = (particleClass == 0);
  this.particleClass = particleClass;
  if (this.isLeadBoid) {
    this.timeLeftToUpdateChaser = -1;
  }
  this.mass = Math.random() * 5;
  this.geometry = new THREE.OctahedronGeometry(this.radius, 1);
  this.material = new THREE.MeshPhongMaterial(
      {
        color: this.getColor(particleClass),
        specular: 0xffffff,
        shininess: 30,
        shading: THREE.SmoothShading,
        side: THREE.FrontSide
      });
  this.mesh = new THREE.Mesh(this.geometry, this.material);

  this.v = new THREE.Vector3(2 - Math.random() * 4, 2 - Math.random() * 4, 2 - Math.random() * 4);

  this.resetPosition();

  scene.add(this.mesh);
};

Particle.prototype.resetPosition = function() {
  this.p = new THREE.Vector3(4 - Math.random() * 8, 4 - Math.random() * 8, 0);
}

Particle.prototype.getColor = function(particleClass) {
  switch (particleClass) {
    case 0: return 0xffffff;
    case 1: return 0xff0000;
    case 2: return 0x00ff00;
    case 3: return 0x0000ff;
  }
}

Particle.prototype.integrate = function(vertices, faces, state, h) {
  var vn = acceleration.clone().multiplyScalar(h).add(this.v);
  var pn = vn.clone().multiplyScalar(h).add(this.p);

  var timeRemaining = h;
  var count = 0;
  while (timeRemaining > 0.001 && count++ < 5) {
    var collision = null;
    var timestep = timeRemaining;

    vn = acceleration.clone().multiplyScalar(timestep).add(this.v);
    pn = vn.clone().multiplyScalar(timestep).add(this.p);

    for (var i = 0; i < faces.length; i++) {
      var face = faces[i];
      var vertex = vertices[face.a];
      var normal = face.normal;

      var dn = this.p.clone().sub(vertex).dot(normal);
      var dn1 = pn.clone().sub(vertex).dot(normal);

      if ((dn < 0) != (dn1 < 0)) {
        var f = (dn / (dn - dn1)) - 0.001;

        timestepPortion = timestep * f;
        var collisionVelocity = acceleration.clone().multiplyScalar(timestepPortion).add(this.v);
        var collisionPosition = collisionVelocity.clone().multiplyScalar(timestepPortion).add(this.p);

        var v1 = vertices[face.a];
        var v2 = vertices[face.b];
        var v3 = vertices[face.c];
        if (Triangle.isContained(collisionPosition, normal, v1, v2, v3)) {
          if (!collision || f < collision.f) {
            collision = {
                f: f,
                vn: collisionVelocity,
                pn: collisionPosition,
                normal: normal.clone()
            };
          }
        }
      }
    }

    if (collision) {
      var vc = collision.vn;
      pn = collision.pn;
      vn = collision.normal.multiplyScalar(vc.dot(collision.normal));
      var vt = vc.sub(vn);
      vt.sub(vt.clone().normalize().multiplyScalar(Math.min(Coefficients.FRICTION * vn.length(), vt.length())));

      vn.multiplyScalar(-Coefficients.RESTITUTION).add(vt);

      timestep *= f;
    }

    this.mesh.position.copy(pn);
    this.v = vn;
    this.p = pn;
    timeRemaining -= timestep;
  }
};

var State = function() {
  this.stateValues = [];
};

State.prototype.addStateValues = function(values) {
  Array.prototype.push.apply(this.stateValues, values);
};

State.prototype.multiplyScalar = function(scalar) {
  for(var i = 0; i < stateValues.length; i++) {
    stateValues[i] *= scalar;
  }
};

State.prototype.addState = function(state) {
  for(var i = 0; i < stateValues.length; i++) {
    stateValues[i] += state[i];
  }
}
