const scene = new THREE.Scene();
const portheight = window.innerHeight * 0.75;
const camera = new THREE.PerspectiveCamera( 75, window.innerWidth / portheight, 0.1, 1000 );

const renderer = new THREE.WebGLRenderer();
renderer.setSize( window.innerWidth, portheight );
//document.body.appendChild( renderer.domElement );
document.getElementById('3js').appendChild(renderer.domElement);

// const geometry = new THREE.BoxGeometry();
// const material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
// const cube = new THREE.Mesh( geometry, material );
// scene.add( cube );

const lmat = new THREE.LineBasicMaterial( { color: 0x00aaaa } );

function PIObject(points, indices) {
  this.points = points;
  this.indices = indices;
};

function dummytriangle() {
  var points = [];
  //points.push( new THREE.Vector3( -1, 0, 0 ) );
  //points.push( new THREE.Vector3( 0, 1, 0 ) );
  //points.push( new THREE.Vector3( 1, 0, 0 ) );
  points.push(-1,0,0);
  points.push(0,1,0);
  points.push(1,0,0);

  var indices = [
	0,1,
	1,2,
	2,0
  ];
  //return {"points":points, "indices":indices};
  return new PIObject(points, indices);
}

const phi = (1 + Math.sqrt(5)) / 2;

var rotob = null;

function ico34() {
  // construct the top 3/4th of an icosahedron
  const alt = Math.atan(0.5);
  const cosalt = Math.cos(alt);
  const sinalt = Math.sin(alt);
  var points = [0,0,1]; // x,y,z triples
  var indices = [];
  // points[1,2,3,4,5] top ring
  for (var i = 0; i < 5; i++) {
	var th = i * 2 * Math.PI / 5;
	points.push(cosalt * Math.cos(th), cosalt * Math.sin(th), sinalt);
	indices.push(0,1+i, i+1,((i+1)%5)+1);
  }
  //indices.push(5,1);
  // points[6,7,8,9,10] bottom ring
  for (var i = 0; i < 5; i++) {
  	var th = (i + 0.5) * 2 * Math.PI / 5;
  	points.push(cosalt * Math.cos(th), cosalt * Math.sin(th), -sinalt);
	indices.push(i+1,i+6, i+1, ((i+4)%5)+6, i+6,((i+1)%5)+6);
  }
  //return {"points":points, "indices":indices};
  return new PIObject(points, indices);
}

function mid(a,b) {
  return [(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2];
}
function vlen(v) {
  return Math.sqrt((v[0]*v[0])+(v[1]*v[1])+(v[2]*v[2]));
}
function norm(v) {
  const r = vlen(v);
  return [v[0]/r, v[1]/r, v[2]/r];
}
function dist(a,b) {
  const dx = a[0] - b[0];
  const dy = a[1] - b[1];
  const dz = a[2] - b[2];
  return Math.sqrt((dx*dx)+(dy*dy)+(dz*dz));
}
function vsub(a,b) {
  return [
    a[0] - b[0],
    a[1] - b[1],
    a[2] - b[2]
  ];
}
function dot(a,b) {
  return (a[0]*b[0]) + (a[1]*b[1]) + (a[2]*b[2]);
}
function rad2deg(r) {
  return 180*r/Math.PI;
}

function ico34geo2v(rings) {
  rings = rings || 9;
  // construct the top 3/4th of an icosahedron, divide each edge by two
  const alt = Math.atan(0.5);
  const cosalt = Math.cos(alt);
  const sinalt = Math.sin(alt);
  var points = [0,0,1]; // x,y,z triples
  var indices = [];
  // points[1,2,3,4,5] top ring
  var topring = [];
  for (var i = 0; i < 5; i++) {
	var th = i * 2 * Math.PI / 5;
	var pt = [cosalt * Math.cos(th), cosalt * Math.sin(th), sinalt];
	var mp = norm(mid(pt, [0,0,1]));
	points = points.concat(mp);
	topring = topring.concat(pt);
	indices.push(0,1+i, i+1,((i+1)%5)+1);
  }
  if (rings == 1) {
    return new PIObject(points, indices);
  }
  // points[6..15] next ring
  for (var i = 0; i < 5; i++) {
	var pt = topring.slice(i*3,(i+1)*3);
	var ni = (i+1)%5;
	var npt = topring.slice(ni*3,(ni+1)*3);
	var mp = norm(mid(pt, npt));
	points = points.concat(pt,mp);
	indices.push(i+1,(i*2)+6, // down along ico edge
				 i+1,(i*2)+7, // down to next point
				 i+1,(((i*2)+9)%10)+6, // down to previous point
				 (i*2)+6,(i*2)+7, // horizontal 1
				 (i*2)+7,(((i*2)+2)%10)+6); // horizontal 2
  }
  if (rings == 2) {
    return new PIObject(points, indices);
  }
  var botring = [];
  for (var i = 0; i < 5; i++) {
	var th = (i+0.5) * 2 * Math.PI / 5;
	var pt = [cosalt * Math.cos(th), cosalt * Math.sin(th), -sinalt];
	botring = botring.concat(pt)
  }
  // points[16..25] next ring (midway through center band of icosahedron)
  for (var i = 0; i < 5; i++) {
	var uppt = topring.slice(i*3,(i+1)*3);
	var pt = botring.slice(i*3,(i+1)*3);
	var mp = norm(mid(pt, uppt));
	points = points.concat(mp);
	uppt = topring.slice(((i+1)%5)*3,(((i+1)%5)+1)*3);
	mp = norm(mid(pt,uppt));
	points = points.concat(mp);
	indices.push((i*2)+6, (i*2)+16, // down a
				 (i*2)+7, (i*2)+17, // down b
				 (i*2)+7, (i*2)+16,
				 (i*2)+6, (((i*2)+9)%10)+16,
				 (i*2)+16, (i*2)+17,
				 (i*2)+17, (((i*2)+2)%10)+16);
  }
  // stop here for flat bottom half ico
  if (rings == 3) {
    return new PIObject(points, indices);
  }
  // points[26..35] next ring finishing 3/4 icosahedron
  for (var i = 0; i < 5; i++) {
    var pt = botring.slice(i*3,(i+1)*3);
    var ni = (i+1)%5;
    var npt = botring.slice(ni*3,(ni+1)*3);
    var mp = norm(mid(pt, npt));
    points = points.concat(pt,mp);
  }
  for (var i = 0; i < 10; i++) {
    var ni = (i+1)%10;
    indices.push(
      16+i, 26+i,
      16+ni, 26+i,
      26+i, 26+ni
    );
  }
  return new PIObject(points, indices);
}

function feq(a,b,ep) {
  ep=ep||0.000001;
  return Math.abs(a-b)<ep;
}
function naturalSort(j,k){
  if (j<k) return -1;
  if (j>k) return 1;
  return 0;
}
PIObject.prototype.edgeLengths = function() {
  var lens = [];
  var ls = 0.0;
  for (var i = 0; i < this.indices.length; i+=2) {
	var ai = this.indices[i] * 3;
	var a = this.points.slice(ai, ai+3);
	var bi = this.indices[i+1] * 3;
	var b = this.points.slice(bi, bi+3);
	var r = dist(a,b);
	lens.push(r);
	ls += r;
  }
  lens.sort(naturalSort);
  return lens;
}

// bi directional equality (i == j)
function bidieq(ia, ib, ja, jb) {
  return ((ia == ja && ib == jb) || (ia == jb && ib == ja));
}
PIObject.prototype.hasEdge = function(ia, ib) {
  for (var i = 0; i < this.indices.length; i+=2) {
    if (bidieq(ia, ib, this.indices[i], this.indices[i+1])) {
      return true;
    }
  }
  return false;
};

PIObject.prototype.findTriangles = function() {
  // triples [[index, index, index], ...], each internally sorted by index order, no global sort order
  var triangles = [];
  var hastr = function(tr) {
    for (var tri = 0; tri < triangles.length; tri++) {
      var xtr = triangles[tri];
      if (xtr[0] == tr[0] && xtr[1] == tr[1] && xtr[2] == tr[2]) {
	return true
      }
    }
    return false;
  };
  for (var i = 0; i < this.indices.length; i+=2) {
    var ia = this.indices[i];
    var ib = this.indices[i+1];
    for (var j = 0; j < this.indices.length; j+=2) {
      if (j == i) {continue;}
      var ja = this.indices[j];
      var jb = this.indices[j+1];
      if (ia == ja && this.hasEdge(ib, jb)) {
	var ntr = [ia, ib, jb];
	ntr.sort(naturalSort);
	if (!hastr(ntr)) {
	  triangles.push(ntr);
	}
      }
    }
  }
  return triangles;
};

PIObject.prototype.triangleEdgeLengths = function(triangles) {
  // edges = [[l1, l2, l3], ...]
  var edges = [];
  for (var i = 0; i < triangles.length; i++) {
    var tr = triangles[i];
    var a = this.points.slice(tr[0]*3, (tr[0]+1)*3);
    var b = this.points.slice(tr[1]*3, (tr[1]+1)*3);
    var c = this.points.slice(tr[2]*3, (tr[2]+1)*3);
    var el = [
      dist(a,b),
      dist(b,c),
      dist(c,a)
    ];
    edges.push(el);
  }
  return edges;
};

function DeltaAccumulator() {
  this.they = {};
};
DeltaAccumulator.prototype.add = function(i, v, m) {
  var ov = this.they[i];
  if (ov == undefined) {
    this.they[i] = [v[0]*m, v[1]*m, v[2]*m];
  } else {
    for (var k=0; k<3; k++) {
      ov[k] += v[k] * m;
    }
  }
};
DeltaAccumulator.prototype.apply = function(i, points, offset) {
  var ov = this.they[i];
  if (ov) {
    for (var k=0; k<3; k++) {
      points[offset+k] += ov[k];
    }
  }
};

function stepTowardEdgeLength(ob, tr) {
  tr = tr || 0.6;
  var steps = new DeltaAccumulator();
  for (var i = 0; i < ob.indices.length; i+=2) {
	var ai = ob.indices[i];
	var ap = ai * 3;
	var a = ob.points.slice(ap, ap+3);
	var bi = ob.indices[i+1];
	var bp = bi * 3;
	var b = ob.points.slice(bp, bp+3);
	var ab = [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
	var r = vlen(ab);
	var dr = tr - r; // (dr<0) too long
	steps.add(ai, ab, 0.01 * dr);
	steps.add(bi, ab, -0.01 * dr);
  }
  for (var i = 0; i*3 < ob.points.length; i++) {
	steps.apply(i, ob.points, i*3);
  }
  edgeLengths(ob);
};

function groupTrEdgeLengths(trEdges) {
  // lllc = [[length, length, length, count], ...]
  // lengths sorted
  var lllc = [];
  for (var i = 0; i < trEdges.length; i++) {
    var lll = trEdges[i];
    lll.sort(naturalSort);
    var done = false;
    for (var x = 0; x < lllc.length; x++) {
      var lx = lllc[x];
      if (feq(lx[0], lll[0]) && feq(lx[1], lll[1]) && feq(lx[2], lll[2])) {
	done = true;
	lx[3] += 1;
      }
    }
    if (!done) {
      lllc.push([lll[0], lll[1], lll[2], 1]);
    }
  }
  return lllc;
};

function dictappend(d,k,v) {
  var l = d[k];
  if (l === undefined) {
    d[k] = [v];
  } else {
    l.push(v);
  }
};
PIObject.prototype.vertexAngles = function() {
  var angles = {};
  for (var i = 0; i < this.indices.length; i+=2) {
    var ia = this.indices[i];
    var ib = this.indices[i+1];
    var a = this.points.slice(ia*3, (ia+1)*3);
    var b = this.points.slice(ib*3, (ib+1)*3);
    // for a as radius
    var bminusa = vsub(b, a);
    var th = Math.acos(dot(a, bminusa) / (vlen(a) * vlen(bminusa)));
    dictappend(angles, ia, th);
    // with b as radius, angle off that
    var aminusb = vsub(a, b);
    th = Math.acos(dot(b, aminusb) / (vlen(b) * vlen(aminusb)));
    dictappend(angles, ib, th);
  }
  return angles;
};
// lengths and angles for e.g. 2x4 construction
PIObject.prototype.strutCuts = function() {
  // angles = [[ia, ib, th, length, sa1, sa2], ...]
  // from vertex ia towards vertex ib, angle off verticle at ia end is th
  // sa1 and sa2 are side-half-angles towards a next or previous node
  var angles = [];
  for (var i = 0; i < this.indices.length; i+=2) {
    var ia = this.indices[i];
    var ib = this.indices[i+1];
    var a = this.points.slice(ia*3, (ia+1)*3);
    var b = this.points.slice(ib*3, (ib+1)*3);
    var ablen = dist(a,b);
    // for a as radius
    var bminusa = vsub(b, a);
    var th = Math.acos(dot(a, bminusa) / (vlen(a) * vlen(bminusa)));
    angles.push([ia, ib, th, ablen]);
    // with b as radius, angle off that
    var aminusb = vsub(a, b);
    th = Math.acos(dot(b, aminusb) / (vlen(b) * vlen(aminusb)));
    angles.push([ib, ia, th, ablen]);
    // TODO: horizontal bevel angle at strut end
  }
  return angles;
};

PIObject.prototype.checkEdgeDups = function() {
  var seen = [];
  var dupcount = 0;
  for (var i = 0; i < this.indices.length; i+=2) {
    var ia = this.indices[i];
    var ib = this.indices[i+1];
    if (ib < ia) {
      var t = ia;
      ia = ib;
      ib = t;
    }
    for (var sj = 0; sj < seen.length; sj++) {
      if (seen[sj][0] == ia && seen[sj][1] == ib) {
	console.log("DUP " + ia + ", " + ib);
	dupcount++;
      }
    }
    seen.push([ia, ib]);
  }
  // console.log("dups: " + dupcount);
};

const ob = ico34geo2v(3);
ob.checkEdgeDups();
for (var i = 0; i < ob.points.length; i+=3) {
  var pt = ob.points.slice(i, i+3);
  var r = vlen(pt);
  if (!feq(r,1)) {
    console.log("ERROR expected r=1, got " + r);
  }
}
var debugdiv = document.getElementById('debug');
//console.log(JSON.stringify(ob));
var debugtext = ''
debugtext += "<tt>" + JSON.stringify(ob) + "</tt>";
//const ob = ico34();
//const ob = dummytriangle();
const lens = ob.edgeLengths();
var cv = undefined;
var count = 0;
for (var i=0; i < lens.length; i++) {
  if (feq(cv, lens[i])) {
    count++;
  } else {
    if (count>0) {
      debugtext += '<div>' + String(count) + ' struts @ ' + String(cv) + " * r</div>";
    }
    cv = lens[i];
    count = 1;
  }
}
if (count>0) {
  debugtext += '<div>' + String(count) + ' struts @ ' + String(cv) + ' * r</div>';
}
var triangles = ob.findTriangles();
debugtext += "<div>"+String(triangles.length)+" triangles</div><tt>" + JSON.stringify(triangles) + "</tt>";

var trEdges = ob.triangleEdgeLengths(triangles);
var lllc = groupTrEdgeLengths(trEdges);
for (var i = 0; i < lllc.length; i++) {
  var lx = lllc[i];
  debugtext += "<div>" + lx[3] + " triangles (" + lx[0] + ", " + lx[1] + ", " + lx[2] + ")</div>";
}

var angles = ob.vertexAngles();
debugtext += "<h4>angles off horizontal for struts out of each vertex</h4>";
for (var k in angles) {
  var ka = angles[k];
  debugtext += "<div>" + k + "["+ka.length+"]: " + (rad2deg(ka[0])-90);
  for (var i = 1; i < ka.length; i++) {
    debugtext += ", " + (rad2deg(ka[i])-90);
  }
  debugtext += "</div>";
}

var strutAngles = ob.strutCuts();
debugtext += "<h4>angles for each strut end</h4>";
for (var sa of strutAngles) {
  debugtext += "<div>" + sa[0] + " -> " + sa[1] + " " + (rad2deg(sa[2])-90) + " deg, "+sa[3]+"</div>";
}

debugdiv.innerHTML = debugtext;

const geometry = new THREE.BufferGeometry();//.setFromPoints( points );
geometry.setIndex(ob.indices);
geometry.setAttribute('position', new THREE.Float32BufferAttribute( ob.points, 3 ) );
geometry.computeBoundingSphere();
const line = new THREE.LineSegments( geometry, lmat );
scene.add(line);
rotob = line;

camera.position.z = 5;

function animate() {
  requestAnimationFrame( animate );

  if (rotob) {
	rotob.rotation.x += 0.01;
	rotob.rotation.y += 0.01;
  }
  //stepTowardEdgeLength(ob);
  geometry.attributes.position.set(ob.points);
  geometry.attributes.position.needsUpdate = true;
  //geometry.attributes.position.setXYZ(1,0,1 + ((Date.now() % 2000) / 2000),0);
  //geometry.attributes.position.needsUpdate = true;


  renderer.render( scene, camera );
}
animate();
