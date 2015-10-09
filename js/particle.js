var Coefficients = {
  FRICTION: 0.01,
  RESTITUTION: 0.95
};

var Particle = function(scene) {
  this.identifer = Math.random().toString(36).substring(7);
  this.geometry = new THREE.OctahedronGeometry(0.1, 1);
  this.material = new THREE.MeshPhongMaterial(
      {
        color: 0xffffff,
        specular: 0xffffff,
        shininess: 30,
        shading: THREE.SmoothShading,
        side: THREE.FrontSide
      });
  this.mesh = new THREE.Mesh(this.geometry, this.material);

  this.v = new THREE.Vector3(Math.random() * 9, Math.random() * 9, Math.random() * 9);
  this.p = new THREE.Vector3(4 - Math.random() * 8, 4 - Math.random() * 8, 0);

  scene.add(this.mesh);
};

Particle.prototype.integrate = function(vertices, faces, acceleration, h) {
  var vn = acceleration.clone().multiplyScalar(h).add(this.v);
  var pn = vn.clone().add(this.v).multiplyScalar(h/2).add(this.p);

  var timeRemaining = h;
  var count = 0;
  while (timeRemaining > 0.001 && count++ < 5) {
    var collision = null;
    var timestep = timeRemaining;

    vn = acceleration.clone().multiplyScalar(timestep).add(this.v);
    pn = vn.clone().add(this.v).multiplyScalar(timestep / 2).add(this.p);

    for (var i = 0; i < faces.length; i++) {
      var face = faces[i];
      var vertex = vertices[face.a];
      var normal = face.normal;

      var dn = this.p.clone().sub(vertex).dot(normal);
      var dn1 = pn.clone().sub(vertex).dot(normal);

      if ((dn < 0) != (dn1 < 0)) {
        var f = (dn / (dn - dn1)) - 0.001;

        timestep *= f;
        var collisionVelocity = acceleration.clone().multiplyScalar(timestep).add(this.v);
        var collisionPosition = collisionVelocity.clone().add(this.v).multiplyScalar(timestep / 2).add(this.p);

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
    }

    this.mesh.position.copy(pn);
    this.v = vn;
    this.p = pn;
    timeRemaining -= timestep;
  }
};


