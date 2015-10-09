var Triangle = {
  isContained: function(point, normal, vertex1, vertex2, vertex3) {
    var projectedVertices = [];
    var projectedPoint;

    var nx = Math.abs(normal.x);
    var ny = Math.abs(normal.y);
    var nz = Math.abs(normal.z);
    if (nx > ny && nx > nz) {
      projectedVertices.push(new THREE.Vector3(vertex1.y, vertex1.z, 0));
      projectedVertices.push(new THREE.Vector3(vertex2.y, vertex2.z, 0));
      projectedVertices.push(new THREE.Vector3(vertex3.y, vertex3.z, 0));
      projectedPoint = new THREE.Vector3(point.y, point.z, 0);
    } else if (ny > nx && ny > nz) {
      projectedVertices.push(new THREE.Vector3(vertex1.x, vertex1.z, 0));
      projectedVertices.push(new THREE.Vector3(vertex2.x, vertex2.z, 0));
      projectedVertices.push(new THREE.Vector3(vertex3.x, vertex3.z, 0));
      projectedPoint = new THREE.Vector3(point.x, point.z, 0);
    } else {
      projectedVertices.push(new THREE.Vector3(vertex1.x, vertex1.y, 0));
      projectedVertices.push(new THREE.Vector3(vertex2.x, vertex2.y, 0));
      projectedVertices.push(new THREE.Vector3(vertex3.x, vertex3.y, 0));
      projectedPoint = new THREE.Vector3(point.x, point.y, 0);
    }

    var positiveCount = 0;
    var negativeCount = 0;
    var EPSILON = 0.01;
    for (var i = 0; i < 3; i++) {
      var u = projectedVertices[(i + 1) % 3].clone().sub(projectedVertices[i]);
      var v = projectedPoint.clone().sub(projectedVertices[(i + 1) % 3]);

      var cp = new THREE.Vector3();
      cp.crossVectors(u, v);
      if (cp.z > EPSILON) {
        positiveCount++;
      }
      if (cp.z < -EPSILON) {
        negativeCount++;
      }
    }

    return positiveCount == 3 || negativeCount == 3;
  }
};
