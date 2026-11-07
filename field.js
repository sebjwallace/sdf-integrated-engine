// field.js: caches a scene's distances in a 3D grid, then keeps a list of edits
// applied on top of that cache.
//
//   base   the scene tree, baked once. Expensive to make. When an object is
//          moved by hand, only the region around it is baked again (rebake).
//   world  base plus the edits. This is what the camera rays read.
//
// When an edit moves, only the voxels around it are rewritten, by reading base
// and re-applying the edits. The scene tree is not evaluated again.

const MAX_EDITS = 32;

// Rays read distances that may be stale outside the region an edit dirtied. An
// added shape can make the true distance shorter than the stored one, so rays
// never step further than this, and every edit dirties at least this far around
// itself. A ray therefore cannot jump over an edit it has not been told about.
const MAX_STEP = 0.2;

const EDIT_GLSL = `
uniform int uEditCount;
uniform vec4 uEditPos[${MAX_EDITS}];   // xyz position, w kind (0 sphere, 1 box)
uniform vec4 uEditSize[${MAX_EDITS}];  // xyz half-size, w smoothing
uniform vec4 uEditLook[${MAX_EDITS}];  // rgb colour, a op (0 subtract, 1 add)
uniform vec4 uEditSurf[${MAX_EDITS}];  // rough, metal, glow
uniform float uCutDepth;

float editDist(int i,vec3 p){
  vec3 q=p-uEditPos[i].xyz;
  if(uEditPos[i].w<.5)return length(q)-uEditSize[i].x;
  return sdBox(q,uEditSize[i].xyz-.02)-.02;
}

float applyEdits(float d,vec3 p){
  for(int i=0;i<uEditCount;i++){
    float e=editDist(i,p),k=uEditSize[i].w;
    if(uEditLook[i].a>.5)d=k>0.?smin(d,e,k):min(d,e);
    else d=smax(d,-e,k);
  }
  return d;
}

// Cutting deep into a solid exposes raw material rather than the colour of
// whichever painted surface happens to be nearest. The shallow layer that keeps
// its colour looks like paint at the edge of the cut.
Hit applyEditsHit(Hit m,vec3 p){
  for(int i=0;i<uEditCount;i++){
    float e=editDist(i,p),k=uEditSize[i].w;
    if(uEditLook[i].a>.5){
      Hit b=Hit(e,uEditLook[i].rgb,uEditSurf[i].x,uEditSurf[i].y,uEditSurf[i].z,-1.);
      // The shape melts over k, but colour changes over a narrower band.
      float d=k>0.?smin(m.d,e,k):min(m.d,e);
      if(k>0.)m=hBlend(m,b,k*.3);else m=hUnion(m,b);
      m.d=d;
    }else{
      if(m.d<-uCutDepth&&-e>m.d){m.col=vec3(.14,.145,.155);m.rough=.55;m.metal=.5;m.glow=0.;}
      m.d=smax(m.d,-e,k);
    }
  }
  return m;
}
`;

// Flattens edits into the arrays the shaders read.
const packEdits = (edits) => {
  const pack = {
    count: edits.length,
    pos: new Float32Array(MAX_EDITS * 4),
    size: new Float32Array(MAX_EDITS * 4),
    look: new Float32Array(MAX_EDITS * 4),
    surf: new Float32Array(MAX_EDITS * 4),
  };
  edits.forEach((e, i) => {
    pack.pos.set([...e.pos, e.kind], i * 4);
    pack.size.set([...e.size, e.k], i * 4);
    pack.look.set([...e.color, e.op], i * 4);
    pack.surf.set([e.rough, e.metal, e.glow, 0], i * 4);
  });
  return pack;
};

const setEditUniforms = (gl, u, pack) => {
  gl.uniform1i(u.uEditCount, pack.count);
  if (!pack.count) return;
  gl.uniform4fv(u['uEditPos[0]'], pack.pos);
  gl.uniform4fv(u['uEditSize[0]'], pack.size);
  gl.uniform4fv(u['uEditLook[0]'], pack.look);
  gl.uniform4fv(u['uEditSurf[0]'], pack.surf);
};

// The box an edit can affect directly, before any margin.
const editBounds = (e) => {
  const r = e.size.map((s) => s + e.k);
  return { min: e.pos.map((v, i) => v - r[i]), max: e.pos.map((v, i) => v + r[i]) };
};

const unionBounds = (a, b) => (a ? {
  min: a.min.map((v, i) => Math.min(v, b.min[i])),
  max: a.max.map((v, i) => Math.max(v, b.max[i])),
} : b);

const FIELD_HEADER = `#version 300 es
precision highp float;
precision highp sampler3D;
uniform vec3 uLo;
uniform float uVoxel,uLayer;
out vec4 o;
`;

class DistanceField {
  constructor(viewer, sceneCode, center, extent, targetVoxels) {
    const gl = (this.gl = viewer.gl);
    const size = extent.map((e) => e * 2);
    const max3D = gl.getParameter(gl.MAX_3D_TEXTURE_SIZE);
    this.voxel = Math.max(
      Math.cbrt((size[0] * size[1] * size[2]) / targetVoxels),
      ...size.map((s) => s / max3D),
    );
    this.dims = size.map((s) => Math.ceil(s / this.voxel));
    this.lo = center.map((c, i) => c - extent[i]);
    this.fieldSize = this.dims.map((d) => d * this.voxel);
    this.voxelCount = this.dims[0] * this.dims[1] * this.dims[2];

    this.base = this.makeTexture();
    this.world = this.makeTexture();
    this.fbo = gl.createFramebuffer();
    this.probe = new Float32Array(4);

    this.bakeProgram = viewer.link(`${FIELD_HEADER}${SDF_LIBRARY}${sceneCode}
void main(){
  vec3 p=uLo+vec3(gl_FragCoord.xy,uLayer+.5)*uVoxel;
  o=vec4(treeMap(p));
}`);
    this.updateProgram = viewer.link(`${FIELD_HEADER}uniform sampler3D uBase;${SDF_LIBRARY}${EDIT_GLSL}
void main(){
  vec3 p=uLo+vec3(gl_FragCoord.xy,uLayer+.5)*uVoxel;
  o=vec4(applyEdits(texelFetch(uBase,ivec3(gl_FragCoord.xy,int(uLayer)),0).r,p));
}`);
    gl.useProgram(this.updateProgram.prog);
    gl.uniform1i(this.updateProgram.uniforms.uBase, 1);
    // Hidden objects are left out of the bake, and moved ones baked where they
    // are now. The viewer sets both on every pass.
    this.viewer = viewer;
    this.region = null;

    this.layer = 0;
    this.baking = true;
    this.ready = false;
    this.bakeStart = 0;
    this.bakeMs = 0;
    this.lastUpdate = 0;
  }

  makeTexture() {
    const gl = this.gl;
    const tex = gl.createTexture();
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_3D, tex);
    gl.texStorage3D(gl.TEXTURE_3D, 1, gl.R16F, ...this.dims);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    for (const wrap of [gl.TEXTURE_WRAP_S, gl.TEXTURE_WRAP_T, gl.TEXTURE_WRAP_R]) {
      gl.texParameteri(gl.TEXTURE_3D, wrap, gl.CLAMP_TO_EDGE);
    }
    gl.activeTexture(gl.TEXTURE0);
    return tex;
  }

  beginPass(program) {
    const gl = this.gl;
    const u = program.uniforms;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
    gl.disable(gl.BLEND);
    gl.viewport(0, 0, this.dims[0], this.dims[1]);
    gl.useProgram(program.prog);
    gl.uniform3fv(u.uLo, this.lo);
    gl.uniform1f(u.uVoxel, this.voxel);
    return u;
  }

  // Bakes slices until the time budget runs out, so the page stays responsive.
  // Reading a pixel back makes the GPU finish, which gives an honest time.
  step(budgetMs) {
    const gl = this.gl;
    if (!this.bakeStart) this.bakeStart = performance.now();
    const u = this.beginPass(this.bakeProgram);
    this.viewer.setObjectUniforms(u);
    const end = this.region ? this.region.b[2] : this.dims[2];
    if (this.region) {
      const { a, b } = this.region;
      gl.enable(gl.SCISSOR_TEST);
      gl.scissor(a[0], a[1], b[0] - a[0], b[1] - a[1]);
    } else {
      gl.disable(gl.SCISSOR_TEST);
    }
    const start = performance.now();
    while (this.layer < end && performance.now() - start < budgetMs) {
      gl.framebufferTextureLayer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, this.base, 0, this.layer);
      gl.uniform1f(u.uLayer, this.layer);
      gl.drawArrays(gl.TRIANGLES, 0, 3);
      gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.FLOAT, this.probe);
      this.layer++;
    }
    gl.disable(gl.SCISSOR_TEST);
    if (this.layer >= end) {
      this.baking = false;
      if (this.region) this.rebakeMs = performance.now() - this.bakeStart;
      else this.bakeMs = performance.now() - this.bakeStart;
    }
  }

  get progress() {
    if (!this.region) return this.layer / this.dims[2];
    const { a, b } = this.region;
    return (this.layer - a[2]) / Math.max(1, b[2] - a[2]);
  }

  // Bakes the scene again inside a region only, after an object there moved.
  // Space further out keeps distances that may now be too short, which is safe,
  // since rays step no further than MAX_STEP; the margin covers that and soft
  // shadows. A region still pending is merged in.
  rebake(bounds) {
    const pad = MAX_STEP + this.voxel * 2 + 0.3;
    if (this.region) bounds = unionBounds(this.region.bounds, bounds);
    const toVoxel = (v, i, round) => Math.min(this.dims[i], Math.max(0, round((v - this.lo[i]) / this.voxel)));
    const a = bounds.min.map((v, i) => toVoxel(v - pad, i, Math.floor));
    const b = bounds.max.map((v, i) => toVoxel(v + pad, i, Math.ceil));
    if (b.some((v, i) => v <= a[i])) return;
    this.region = { bounds, a, b };
    this.layer = a[2];
    this.baking = true;
    this.ready = false;
    this.bakeStart = 0;
  }

  // The region a finished bake covered, in world units, for update().
  get bakedBounds() {
    if (!this.region) return { min: this.lo, max: this.lo.map((v, i) => v + this.fieldSize[i]) };
    const { a, b } = this.region;
    return { min: a.map((v, i) => this.lo[i] + v * this.voxel), max: b.map((v, i) => this.lo[i] + v * this.voxel) };
  }

  // Rewrites the world texture inside a region and returns the voxels touched.
  update(region, edits) {
    const gl = this.gl;
    const pad = MAX_STEP + this.voxel * 2;
    const toVoxel = (v, i, round) => Math.min(this.dims[i], Math.max(0, round((v - this.lo[i]) / this.voxel)));
    const a = region.min.map((v, i) => toVoxel(v - pad, i, Math.floor));
    const b = region.max.map((v, i) => toVoxel(v + pad, i, Math.ceil));
    const span = b.map((v, i) => v - a[i]);
    if (span.some((s) => s <= 0)) return 0;

    // Edits that cannot reach this region do not need to be evaluated in it.
    const min = a.map((v, i) => this.lo[i] + v * this.voxel);
    const max = b.map((v, i) => this.lo[i] + v * this.voxel);
    const nearby = edits.filter((e) => {
      const box = editBounds(e);
      return box.min.every((v, i) => v <= max[i]) && box.max.every((v, i) => v >= min[i]);
    });

    const u = this.beginPass(this.updateProgram);
    setEditUniforms(gl, u, packEdits(nearby));
    gl.activeTexture(gl.TEXTURE1);
    gl.bindTexture(gl.TEXTURE_3D, this.base);
    gl.activeTexture(gl.TEXTURE0);
    gl.enable(gl.SCISSOR_TEST);
    gl.scissor(a[0], a[1], span[0], span[1]);
    for (let z = a[2]; z < b[2]; z++) {
      gl.framebufferTextureLayer(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, this.world, 0, z);
      gl.uniform1f(u.uLayer, z);
      gl.drawArrays(gl.TRIANGLES, 0, 3);
    }
    gl.disable(gl.SCISSOR_TEST);
    this.lastUpdate = span[0] * span[1] * span[2];
    return this.lastUpdate;
  }

  dispose() {
    const gl = this.gl;
    gl.deleteTexture(this.base);
    gl.deleteTexture(this.world);
    gl.deleteFramebuffer(this.fbo);
    gl.deleteProgram(this.bakeProgram.prog);
    gl.deleteProgram(this.updateProgram.prog);
  }
}
