// renderer.js: raymarches a compiled SDF scene with WebGL2 and handles the
// camera. It draws a low-resolution preview while moving, then refines to an
// anti-aliased full-resolution image when the camera stops.

const VERTEX_SHADER = `#version 300 es
out vec2 uv;
void main(){
  uv=vec2((gl_VertexID<<1)&2,gl_VertexID&2);
  gl_Position=vec4(uv*2.-1.,0.,1.);
}`;

const PRESENT_SHADER = `#version 300 es
precision highp float;
uniform sampler2D tex; uniform vec2 scale;
in vec2 uv; out vec4 o;
void main(){
  vec3 c=texture(tex,uv*scale).rgb;
  float n=fract(52.9829189*fract(dot(gl_FragCoord.xy,vec2(.06711056,.00583715))));
  o=vec4(c+(n-.5)/255.,1.);
}`;

// Distance helpers shared by every scene. Generated map() code calls these.
const SDF_LIBRARY = `
struct Hit{float d;vec3 col;float rough;float metal;float glow;};

float smin(float a,float b,float k){float h=max(k-abs(a-b),0.)/k;return min(a,b)-h*h*k*.25;}
float smax(float a,float b,float k){return k>0.?-smin(-a,-b,k):max(a,b);}

Hit hUnion(Hit a,Hit b){if(a.d<b.d)return a;return b;}
Hit hBlend(Hit a,Hit b,float k){
  float h=clamp(.5+.5*(b.d-a.d)/k,0.,1.);
  return Hit(smin(a.d,b.d,k),mix(b.col,a.col,h),mix(b.rough,a.rough,h),mix(b.metal,a.metal,h),mix(b.glow,a.glow,h));
}
Hit hSubtract(Hit a,Hit b,float k){a.d=smax(a.d,-b.d,k);return a;}
Hit hIntersect(Hit a,Hit b,float k){float d=smax(a.d,b.d,k);if(b.d>a.d)a=b;a.d=d;return a;}

float sdBox(vec3 p,vec3 b){vec3 q=abs(p)-b;return length(max(q,0.))+min(max(q.x,max(q.y,q.z)),0.);}
float sdEllipsoid(vec3 p,vec3 r){float a=length(p/r),b=length(p/(r*r));return a*(a-1.)/max(b,1e-5);}
float sdCylinder(vec3 p,float r,float h){vec2 d=vec2(length(p.xz)-r,abs(p.y)-h);return min(max(d.x,d.y),0.)+length(max(d,0.));}
float sdCone(vec3 p,float r1,float r2,float h){
  vec2 q=vec2(length(p.xz),p.y),k1=vec2(r2,h),k2=vec2(r2-r1,2.*h);
  vec2 ca=vec2(q.x-min(q.x,q.y<0.?r1:r2),abs(q.y)-h);
  vec2 cb=q-k1+k2*clamp(dot(k1-q,k2)/dot(k2,k2),0.,1.);
  float s=(cb.x<0.&&ca.y<0.)?-1.:1.;
  return s*sqrt(min(dot(ca,ca),dot(cb,cb)));
}
float sdCapsule(vec3 p,vec3 a,vec3 b){vec3 pa=p-a,ba=b-a;return length(pa-ba*clamp(dot(pa,ba)/dot(ba,ba),0.,1.));}
float sdExtrude(float d,float z,float h){vec2 w=vec2(d,abs(z)-h);return min(max(w.x,w.y),0.)+length(max(w,0.));}
vec2 polarFold(vec2 p,float n){
  float s=6.2831853/n,a=mod(atan(p.y,p.x)+s*.5,s)-s*.5;
  return length(p)*vec2(cos(a),sin(a));
}
`;

const FRAGMENT_SHADER = (sceneCode, baked, editsInTree = false) => `#version 300 es
precision highp float;
precision highp sampler3D;
uniform vec2 uRes,uJitter;
uniform vec3 uEye;
uniform mat3 uCam;
uniform vec3 uCenter,uExtent;  // bounding box: everything lives inside this
uniform vec3 uStage;  // floor fade: centre x, centre z, radius
uniform vec4 uCut;    // cutaway plane: normal, offset. Space beyond it is removed.
uniform float uHQ,uPick,uMinEps,uHideBodies;
out vec4 fragColor;
${SDF_LIBRARY}
// No body is closer than this to a point inside the stage. Refreshed every frame.
uniform highp sampler3D uBodyBounds;
uniform vec3 uBoundsLo,uBoundsSize;
float bodyLowerBound(vec3 p){
  vec3 uvw=(p-uBoundsLo)/uBoundsSize;
  if(any(lessThan(uvw,vec3(0)))||any(greaterThanEqual(uvw,vec3(1))))return 0.;
  return texelFetch(uBodyBounds,ivec3(uvw*vec3(textureSize(uBodyBounds,0))),0).r;
}
${EDIT_GLSL}
${sceneCode}
float cutPlane(vec3 p){return dot(uCut.xyz,p)-uCut.w;}
float exactModel(vec3 p){return ${editsInTree ? 'treeMap(p)' : 'applyEdits(treeMap(p),p)'};}
${baked ? `
// Baked: distance comes from the cached grid. Outside the grid, add the distance
// to its edge. Material still comes from the tree, once per hit, so painted
// details stay sharper than the grid.
uniform sampler3D uField;
uniform vec3 uFieldSize;
const float STEP_LIMIT=${MAX_STEP.toFixed(3)};
float modelMap(vec3 p){
  vec3 lo=uCenter-uExtent,q=clamp(p,lo,uCenter+uExtent);
  return texture(uField,(q-lo)/uFieldSize).r+length(p-q);
}` : `
// Live: evaluate the whole scene at every step.
const float STEP_LIMIT=1e6;
float modelMap(vec3 p){return exactModel(p);}`}
// A cutaway is one operation on the distance, so it needs no rebaking.
float map(vec3 p){return max(modelMap(p),cutPlane(p));}
float exactMap(vec3 p){return max(exactModel(p),cutPlane(p));}
Hit mapFull(vec3 p){return ${editsInTree ? 'treeHit(p)' : 'applyEditsHit(treeHit(p),p)'};}

const vec3 KEY=normalize(vec3(.55,.8,.35));
const vec3 FLOOR=vec3(.03,.032,.037);
const float FOCAL=3.2;

// Rays only march where the box says something could be. Returns the range of
// t inside the box; x > y means a miss.
vec2 boundHit(vec3 ro,vec3 rd){
  vec3 a=(uCenter-uExtent-ro)/rd,b=(uCenter+uExtent-ro)/rd;
  vec3 lo=min(a,b),hi=max(a,b);
  return vec2(max(max(lo.x,max(lo.y,lo.z)),0.),min(hi.x,min(hi.y,hi.z)));
}

float march(vec3 ro,vec3 rd,int steps){
  vec2 r=boundHit(ro,rd);
  if(r.x>r.y)return -1.;
  float t=r.x,d=1.;
  for(int i=0;i<steps&&t<r.y;i++){
    d=map(ro+rd*t);
    if(d<.0004*t+.0002)return t;
    t+=min(d,STEP_LIMIT);
  }
  return d<.01&&t<r.y?t:-1.;
}

vec3 normalAt(vec3 p,float e){
  const vec2 k=vec2(1,-1);
  return normalize(k.xyy*map(p+k.xyy*e)+k.yyx*map(p+k.yyx*e)+k.yxy*map(p+k.yxy*e)+k.xxx*map(p+k.xxx*e));
}

// The grid finds the surface; final-quality frames then take the normal from the
// real scene, so reflections keep detail finer than a voxel.
vec3 exactNormal(vec3 p,float e){
  const vec2 k=vec2(1,-1);
  return normalize(k.xyy*exactMap(p+k.xyy*e)+k.yyx*exactMap(p+k.yyx*e)+k.yxy*exactMap(p+k.yxy*e)+k.xxx*exactMap(p+k.xxx*e));
}

float softShadow(vec3 ro,vec3 rd,float k,int steps){
  vec2 r=boundHit(ro,rd);
  if(r.x>r.y)return 1.;
  float res=1.,t=max(r.x,.01);
  for(int i=0;i<steps&&t<r.y;i++){
    float h=map(ro+rd*t);
    res=min(res,k*h/t);
    if(res<.002)return 0.;
    t+=clamp(h,.012,min(.3,STEP_LIMIT));
  }
  res=clamp(res,0.,1.);
  return res*res*(3.-2.*res);
}

float occlusion(vec3 p,vec3 n){
  float o=0.,w=1.;
  for(int i=1;i<=5;i++){
    float h=.015+.075*float(i);
    o+=max(h-map(p+n*h),0.)*w;
    w*=.72;
  }
  return clamp(1.-1.9*o,0.,1.);
}

// The backdrop seen directly behind the model.
vec3 backdrop(vec3 rd){
  return mix(FLOOR*1.05,vec3(.012,.014,.019),smoothstep(-.02,.55,rd.y));
}

// The studio the model reflects: a long overhead softbox and two side strips.
// blur widens each edge so that rough materials get a soft highlight.
vec3 studio(vec3 rd,float blur){
  vec3 c=mix(FLOOR*.9,vec3(.11,.125,.15),smoothstep(-.05,.9,rd.y));
  float e=.04+blur*1.3;
  if(rd.y>.02){
    vec2 q=rd.xz/rd.y;
    float s=smoothstep(2.6+e,2.6-e,abs(q.x))*smoothstep(.75+e,.75-e,abs(q.y-.1));
    c+=vec3(1.,.98,.95)*3.2*s/(1.+blur*5.);
  }
  float side=smoothstep(.17+e,.17-e,abs(rd.y-.2))*smoothstep(.45-e,.45+e,abs(rd.z));
  c+=vec3(.75,.85,1.)*.55*side/(1.+blur*5.);
  return c;
}

vec3 lightSurface(vec3 p,vec3 n,vec3 rd,Hit m,float shadow,float ao){
  float ndl=max(dot(n,KEY),0.);
  vec3 h=normalize(KEY-rd);
  float ndv=max(dot(n,-rd),0.);
  vec3 f0=mix(vec3(.05),m.col,m.metal);
  vec3 F=f0+(1.-f0)*pow(1.-ndv,5.);
  float a=max(m.rough*m.rough,.002);
  float D=a*a/pow(max(dot(n,h),0.)*max(dot(n,h),0.)*(a*a-1.)+1.,2.)/3.1416;
  vec3 diffuse=m.col*(1.-m.metal)*(vec3(1.,.97,.92)*1.7*ndl*shadow
    +vec3(.34,.38,.46)*(.5+.5*n.y)*ao
    +vec3(.13,.13,.14)*(.5-.5*n.y)*ao);
  vec3 spec=F*min(D,40.)*.2*ndl*shadow;
  return diffuse+spec+m.col*m.glow;
}

// One bounce, used for mirror-like paint, chrome and the polished floor.
vec3 reflected(vec3 ro,vec3 rd,float blur){
  float t=march(ro,rd,48);
  if(t<0.)return studio(rd,blur);
  vec3 p=ro+rd*t;
  Hit m=mapFull(p);
  vec3 n=normalAt(p,max(.002,uMinEps));
  vec3 c=lightSurface(p,n,rd,m,1.,.75);
  vec3 F=mix(vec3(.05),m.col,m.metal);
  return c+F*studio(reflect(rd,n),max(m.rough,.3));
}

vec3 shadeObject(vec3 ro,vec3 rd,float t){
  vec3 p=ro+rd*t;
  vec3 n=uHQ>.5?exactNormal(p,max(.0006,.0004*t)):normalAt(p,max(uMinEps,.0004*t));
  Hit m=mapFull(p);
  vec3 q=p+n*.003;
  float ao=occlusion(p,n);
  float sh=max(dot(n,KEY),0.)>0.?softShadow(q,KEY,14.,uHQ>.5?40:12):0.;
  vec3 c=lightSurface(p,n,rd,m,sh,ao);

  vec3 r=reflect(rd,n);
  float ndv=max(dot(n,-rd),0.);
  vec3 f0=mix(vec3(.05),m.col,m.metal);
  vec3 F=f0+(max(vec3(1.-m.rough),f0)-f0)*pow(1.-ndv,5.);
  vec3 refl=(uHQ>.5&&m.rough<.35)?reflected(q,r,m.rough):studio(r,m.rough);
  float specAO=clamp(ao*1.3+.1,0.,1.);
  return c+F*refl*specAO;
}

vec3 shadeFloor(vec3 ro,vec3 rd,float t){
  vec3 p=ro+rd*t;
  float dist=length(p.xz-uStage.xy),R=uStage.z;
  vec3 n=vec3(0,1,0);
  float sh=softShadow(p+n*.002,KEY,9.,uHQ>.5?40:12),ao=1.;
  // Contact shading only reaches about half a unit, so skip it far from the model.
  if(length(max(abs(p-uCenter)-uExtent,0.))<.6){
    float h=map(p);
    ao=mix(.08,1.,smoothstep(0.,.42,h));
    ao=min(ao,occlusion(p,n));
  }
  vec3 c=FLOOR*(.28+.72*sh)*ao*1.25;

  float ndv=max(-rd.y,0.);
  float F=.03+.45*pow(1.-ndv,4.);
  vec3 r=reflect(rd,n);
  if(uHQ>.5&&boundHit(p,r).x<=boundHit(p,r).y){
    float rt=march(p+n*.002,r,40);
    if(rt>0.){
      vec3 rp=p+r*rt;
      Hit m=mapFull(rp);
      vec3 rn=normalAt(rp,max(.003,uMinEps));
      c+=F*lightSurface(rp,rn,r,m,1.,.7)*exp(-rt*1.1);
    }
  }
  return mix(c,backdrop(rd),smoothstep(R*1.6,R*5.,dist));
}

// The cut face, drawn like an engineering section: flat, hatched, tinted by the
// part being cut, and outlined where the solid ends or one part meets another.
vec3 shadeCut(vec3 p,vec3 rd,float t,Hit m){
  vec3 n=uCut.xyz;
  p-=n*cutPlane(p);
  vec3 u=normalize(cross(n,abs(n.x)>.5?vec3(0,1,0):vec3(1,0,0))),v=cross(n,u);
  float w=.0012*t;

  float spacing=uStage.z*.014;
  float g=(dot(p,u)+dot(p,v))*.7071/spacing;
  float footprint=(2.*t/(FOCAL*uRes.y))/(spacing*max(abs(dot(rd,n)),.15));
  float line=1.-smoothstep(.13-footprint,.13+footprint,abs(fract(g)-.5));
  line=mix(line,.26,smoothstep(.35,.9,footprint));

  vec3 tint=m.col*.8+vec3(.06);
  vec3 c=mix(tint,tint*.4,line)*(.82+.18*max(dot(n,KEY),0.));

  float seam=(length(mapFull(p+u*w).col-m.col)>.04||length(mapFull(p+v*w).col-m.col)>.04)?1.:0.;
  float edge=max(seam,1.-smoothstep(w,w*1.6,-m.d));
  // Dark lines on pale parts and pale lines on dark ones, so the outline always reads.
  vec3 ink=dot(tint,vec3(.3,.6,.1))>.3?vec3(.012,.014,.02):vec3(1.,.94,.78);
  return mix(c,ink,edge);
}

vec3 shadeHit(vec3 ro,vec3 rd,float t){
  vec3 p=ro+rd*t;
  float plane=cutPlane(p);
  // Only points on the plane can be on the cut face, which keeps this free
  // whenever nothing is being cut.
  if(plane>-(.002+.0006*t)){
    Hit m=mapFull(p);
    if(m.d<plane+.0012*t)return shadeCut(p,rd,t,m);
  }
  return shadeObject(ro,rd,t);
}

vec3 aces(vec3 x){return clamp(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14),0.,1.);}

void main(){
  vec2 ndc=(2.*(gl_FragCoord.xy+uJitter)-uRes)/uRes.y;
  vec3 rd=normalize(uCam*vec3(ndc,FOCAL));
  float tFloor=rd.y<0.?-uEye.y/rd.y:-1.;
  float t=march(uEye,rd,uHQ>.5?128:48);
  if(uPick>.5){fragColor=vec4(t);return;}
  vec3 c;
  if(t>0.&&(tFloor<0.||t<tFloor))c=shadeHit(uEye,rd,t);
  else if(tFloor>0.)c=shadeFloor(uEye,rd,tFloor);
  else c=backdrop(rd);
  // Soft vignette, tone mapping, gamma.
  c*=1.-.28*dot(ndc*.55,ndc*.55);
  fragColor=vec4(pow(aces(c),vec3(1./2.2)),1.);
}`;

const NO_EDITS = packEdits([]);

// Fly keys by physical position (so they work on any keyboard layout):
// [right, up, forward].
const FLY_KEYS = {
  KeyW: [0, 0, 1], KeyS: [0, 0, -1], KeyA: [-1, 0, 0], KeyD: [1, 0, 0],
  KeyE: [0, 1, 0], KeyQ: [0, -1, 0],
  ShiftLeft: [0, 0, 0], ShiftRight: [0, 0, 0],  // held for speed
  Space: [0, 0, 0],  // held to turn the head with the pointer, see look()
};

// The shapes the edit tools place. Half-sizes are in world units.
const TOOLS = {
  hole: { kind: 0, op: 0, size: [0.28, 0.28, 0.28], k: 0.03 },
  cut: { kind: 1, op: 0, size: [0.3, 0.22, 0.3], k: 0.015 },
  blob: { kind: 0, op: 1, size: [0.22, 0.22, 0.22], k: 0.1, paint: ['#d9a441', 0.3, 0.1] },
};

class Viewer {
  constructor(canvas, onError, onChange = () => {}) {
    this.canvas = canvas;
    this.onError = onError;
    this.onChange = onChange;
    this.mode = 'baked';
    this.gridVoxels = 16e6;
    this.tool = 'orbit';
    this.edits = [];
    this.editPack = packEdits([]);
    this.frameMs = 16;
    this.paused = matchMedia('(prefers-reduced-motion: reduce)').matches;
    const gl = (this.gl = canvas.getContext('webgl2', { alpha: false, antialias: false, depth: false, powerPreference: 'high-performance' }));
    if (!gl) throw new Error('This browser does not support WebGL2.');

    this.floatTarget = !!gl.getExtension('EXT_color_buffer_float');
    gl.bindVertexArray(gl.createVertexArray());
    this.present = this.link(PRESENT_SHADER);
    this.fbo = gl.createFramebuffer();
    this.tex = gl.createTexture();
    if (this.floatTarget) {
      // A single pixel for asking "what is under the pointer?"
      this.pickTex = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, this.pickTex);
      gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGBA16F, 1, 1);
      this.pickFbo = gl.createFramebuffer();
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.pickFbo);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.pickTex, 0);
    } else {
      this.mode = 'live';
    }
    gl.disable(gl.DEPTH_TEST);

    // Camera: spherical coordinates around a target. `goal` is where the
    // pointer wants the camera; `cam` eases towards it.
    this.cam = { theta: 30, phi: 15, dist: 6, target: [0, 0.5, 0] };
    this.goal = { ...this.cam };
    this.previewScale = 0.6;
    // Render resolution as a fraction of the screen's, or 0 for automatic:
    // lower while moving to hold the frame rate, full when still.
    this.quality = 0;
    this.strips = 1;
    this.sample = 0;
    this.strip = 0;
    this.maxSamples = 16;

    this.bindInput();
    canvas.addEventListener('webglcontextlost', (e) => {
      e.preventDefault();
      this.onError('The GPU reset the WebGL context. This scene may be too heavy for this machine. Reload to try again.');
    });
    new ResizeObserver(() => this.resize()).observe(canvas);
    this.resize();
    requestAnimationFrame((t) => this.frame(t));
  }

  link(fragmentSource) {
    const gl = this.gl;
    const compile = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const prog = gl.createProgram();
    gl.attachShader(prog, compile(gl.VERTEX_SHADER, VERTEX_SHADER));
    gl.attachShader(prog, compile(gl.FRAGMENT_SHADER, fragmentSource));
    gl.linkProgram(prog);
    if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(prog));
    const uniforms = {};
    const count = gl.getProgramParameter(prog, gl.ACTIVE_UNIFORMS);
    for (let i = 0; i < count; i++) {
      const { name } = gl.getActiveUniform(prog, i);
      uniforms[name] = gl.getUniformLocation(prog, name);
    }
    return { prog, uniforms };
  }

  setScene(scene) {
    // A physics scene is a fixed world plus bodies, which the simulation moves.
    this.sim = null;
    this.bodyGrab = null;
    let tree;
    if (scene.physics) {
      const spec = scene.physics();
      this.sim = new PhysicsWorld({ world: spec.world.d });
      for (const { shape, ...options } of spec.bodies) this.sim.add(shape, options);
      tree = SDF.withBodies(spec.world, this.sim.bodies.map((b) => ({ shape: b.shape, radius: b.info.renderRadius })));
    } else {
      tree = scene.build();
    }
    this.toolScale = scene.toolScale || 1;
    const compiled = SDF.compile(tree);
    this.sceneCode = compiled.code;
    this.animation = scene.animate ? scene : null;
    this.clock = 0;
    this.jointIndex = new Map(compiled.joints.map((name, i) => [name, i]));
    this.jointR = new Float32Array(compiled.joints.length * 9);
    this.jointT = new Float32Array(compiled.joints.length * 3);
    this.cut = null;
    const program = this.link(FRAGMENT_SHADER(this.sceneCode, false, !!this.sim));
    this.gl.useProgram(program.prog);
    this.gl.uniform1i(program.uniforms.uBodyBounds, 2);
    if (this.live) this.gl.deleteProgram(this.live.prog);
    this.live = program;
    this.disposeField();
    if (this.baked) this.gl.deleteProgram(this.baked.prog);
    this.baked = null;
    this.edits = [];
    this.editPack = packEdits([]);
    this.dirty = null;
    this.hover = null;
    this.grab = null;
    this.setTool('orbit');
    this.bound = { center: scene.center, extent: scene.extent, radius: Math.hypot(...scene.extent) };
    // Rays march inside this box. An animated scene moves it along with the model.
    this.rayBox = { center: scene.center, extent: scene.movingExtent || scene.extent };
    this.applyPose();
    this.applySimulation();
    this.cut = { axis: scene.cutAxis || 'z', amount: 0, flip: false };
    this.updateCutPlane();
    this.home = { ...scene.camera, target: scene.center };
    Object.assign(this.cam, this.home);
    Object.assign(this.goal, this.home);
    this.restart(false);
  }

  restart(moving = true) {
    this.moving = moving;
    this.sample = 0;
    this.strip = 0;
  }

  // ---- renderer modes ---------------------------------------------------------

  get liveOnly() { return !!(this.animation || this.sim); }
  get usingField() { return !this.liveOnly && this.mode === 'baked' && !!this.field && this.field.ready; }
  get active() { return this.usingField ? this.baked : this.live; }

  setMode(mode) {
    if (this.liveOnly || (mode === 'baked' && !this.floatTarget)) return;
    this.mode = mode;
    this.restart(this.moving);
    this.onChange();
  }

  setGrid(voxels) {
    this.gridVoxels = voxels;
    this.disposeField();
    this.restart(this.moving);
    this.onChange();
  }

  disposeField() {
    if (this.field) this.field.dispose();
    this.field = null;
  }

  startBake() {
    try {
      if (!this.baked) {
        this.baked = this.link(FRAGMENT_SHADER(this.sceneCode, true));
        this.gl.useProgram(this.baked.prog);
        this.gl.uniform1i(this.baked.uniforms.uField, 1);
      }
      this.field = new DistanceField(this, this.sceneCode, this.bound.center, this.bound.extent, this.gridVoxels);
    } catch (e) {
      this.mode = 'live';
      this.onError(`Could not build the baked renderer, so it is using Live mode.\n${e.message}`);
    }
    this.onChange();
  }

  // Asks the scene for this moment's joints and turns them into uniform data.
  applyPose() {
    if (!this.animation) return;
    const center = this.animation.animate(this.clock, (name, { rotate = [0, 0, 0], move = [0, 0, 0] }) => {
      const i = this.jointIndex.get(name);
      if (i === undefined) return;
      this.jointR.set(SDF.rotationRows(...rotate), i * 9);
      this.jointT.set(move, i * 3);
    });
    if (center) this.rayBox.center = center;
    if (this.cut) this.updateCutPlane();
  }

  // Hands the simulation's transforms to the shader's joints.
  applySimulation() {
    if (!this.sim) return;
    this.sim.bodies.forEach((b, i) => {
      // Rows of R, as GLSL columns, give the inverse that shape-space needs.
      this.jointR.set(b.R, i * 9);
      this.jointT.set(b.renderTranslation(), i * 3);
    });
    this.updateBodyBounds();
  }

  // A coarse grid over the stage. Each cell holds a lower bound on the distance
  // to any body, or BOUND_FAR when none is within reach. Rays read this first.
  updateBodyBounds() {
    const gl = this.gl;
    const { center, extent } = this.bound;
    const dims = [40, 20, 40];
    if (!this.boundsTex) {
      this.boundsTex = gl.createTexture();
      gl.activeTexture(gl.TEXTURE2);
      gl.bindTexture(gl.TEXTURE_3D, this.boundsTex);
      gl.texStorage3D(gl.TEXTURE_3D, 1, gl.R32F, ...dims);
      gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_3D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.activeTexture(gl.TEXTURE0);
      this.boundsData = new Float32Array(dims[0] * dims[1] * dims[2]);
    }
    const FAR = 1.2;
    const lo = center.map((c, i) => c - extent[i]);
    const size = extent.map((e) => e * 2);
    const cell = size.map((s, i) => s / dims[i]);
    const slack = Math.hypot(...cell) / 2;
    const data = this.boundsData;
    data.fill(FAR);
    for (const b of this.sim.bodies) {
      const t = b.renderTranslation();
      const r = b.info.renderRadius;
      const reach = FAR + r + slack;
      const from = t.map((v, i) => Math.max(0, Math.floor((v - reach - lo[i]) / cell[i])));
      const to = t.map((v, i) => Math.min(dims[i] - 1, Math.floor((v + reach - lo[i]) / cell[i])));
      for (let k = from[2]; k <= to[2]; k++) {
        const z = lo[2] + (k + 0.5) * cell[2] - t[2];
        for (let j = from[1]; j <= to[1]; j++) {
          const y = lo[1] + (j + 0.5) * cell[1] - t[1];
          const row = (k * dims[1] + j) * dims[0];
          for (let i = from[0]; i <= to[0]; i++) {
            const x = lo[0] + (i + 0.5) * cell[0] - t[0];
            const d = Math.sqrt(x * x + y * y + z * z) - r - slack;
            if (d < data[row + i]) data[row + i] = d;
          }
        }
      }
    }
    gl.activeTexture(gl.TEXTURE2);
    gl.bindTexture(gl.TEXTURE_3D, this.boundsTex);
    gl.texSubImage3D(gl.TEXTURE_3D, 0, 0, 0, 0, ...dims, gl.RED, gl.FLOAT, data);
    gl.activeTexture(gl.TEXTURE0);
  }

  dropAgain() {
    if (!this.sim) return;
    this.sim.reset();
    this.applySimulation();
    this.restart(true);
  }

  setPaused(paused) {
    this.paused = paused;
    this.onChange();
  }

  // Bakes a little each frame, then keeps the world texture in step with edits.
  stepField() {
    if (this.liveOnly) return;
    if (this.mode === 'baked' && !this.field && this.sample >= 1) this.startBake();
    const field = this.field;
    if (!field) return;
    if (field.baking) {
      field.step(12);
      if (!field.baking) {
        field.update({ min: field.lo, max: field.lo.map((v, i) => v + field.fieldSize[i]) }, this.edits);
        field.ready = true;
        this.dirty = null;
        this.restart(this.moving);
      }
      this.onChange();
    } else if (this.dirty) {
      field.update(this.dirty, this.edits);
      this.dirty = null;
    }
  }

  // ---- cutaway ----------------------------------------------------------------

  // amount runs from 0 (whole model) to 1 (all removed) across the scene's box.
  setCut(change) {
    Object.assign(this.cut, change);
    this.updateCutPlane();
    this.activeUntil = performance.now() + 150;
    this.onChange();
  }

  updateCutPlane() {
    const { axis, amount, flip } = this.cut;
    if (amount <= 0) {
      this.cutPlane = [0, 0, 0, 1e6]; // every point is far inside: no cut
      return;
    }
    const i = 'xyz'.indexOf(axis);
    // The ray box follows an animated model, so its cut travels with it.
    const lo = this.rayBox.center[i] - this.rayBox.extent[i];
    const hi = this.rayBox.center[i] + this.rayBox.extent[i];
    const sign = flip ? -1 : 1;
    const position = flip ? lo + amount * (hi - lo) : hi - amount * (hi - lo);
    const normal = [0, 0, 0];
    normal[i] = sign;
    this.cutPlane = [...normal, sign * position];
  }

  // ---- edits ------------------------------------------------------------------

  changed(region) {
    this.dirty = unionBounds(this.dirty, region);
    this.editPack = packEdits(this.edits);
    this.activeUntil = performance.now() + 150;
    if (this.sim) this.sim.setEdits(this.edits);
    this.onChange();
  }

  setTool(tool) {
    if (this.animation) tool = 'orbit';
    this.tool = tool;
    this.canvas.classList.toggle('placing', tool !== 'orbit');
    this.onChange();
  }

  addEdit(tool, pos) {
    if (this.edits.length >= MAX_EDITS) {
      this.onError(`This version supports up to ${MAX_EDITS} edits. Remove one to add another.`);
      return null;
    }
    const { paint, ...shape } = TOOLS[tool];
    const look = SDF.material(...(paint || ['#808080']));
    const edit = { ...shape, size: shape.size.map((s) => s * this.toolScale), pos: [...pos], ...look };
    this.edits.push(edit);
    this.changed(editBounds(edit));
    return edit;
  }

  moveEdit(edit, pos) {
    const before = editBounds(edit);
    edit.pos = pos;
    this.changed(unionBounds(before, editBounds(edit)));
  }

  resizeEdit(edit, factor) {
    const before = editBounds(edit);
    const scale = Math.min(1.5 / Math.max(...edit.size), Math.max(0.04 / Math.min(...edit.size), factor));
    edit.size = edit.size.map((s) => s * scale);
    this.changed(unionBounds(before, editBounds(edit)));
  }

  removeEdit(edit) {
    this.edits = this.edits.filter((e) => e !== edit);
    if (this.hover === edit) this.hover = null;
    this.changed(editBounds(edit));
  }

  clearEdits() {
    const region = this.edits.map(editBounds).reduce(unionBounds, null);
    this.edits = [];
    this.hover = null;
    if (region) this.changed(region);
  }

  pointerRay(x, y) {
    const { clientWidth: w, clientHeight: h } = this.canvas;
    const { eye, forward, right, up } = this.basis(this.cam);
    const u = (2 * x - w) / h;
    const v = (h - 2 * y) / h;
    const d = right.map((r, i) => r * u + up[i] * v + forward[i] * 3.2);
    const len = Math.hypot(...d);
    return { eye, forward, dir: d.map((c) => c / len) };
  }

  // Asks the GPU where the pointer's ray hits the model.
  pickSurface(x, y, includeBodies = false) {
    if (!this.pickFbo) return null;
    this.pickBodies = includeBodies;
    const gl = this.gl;
    const { width, height, clientWidth, clientHeight } = this.canvas;
    const px = (x * width) / clientWidth;
    const py = height - (y * height) / clientHeight;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.pickFbo);
    gl.disable(gl.BLEND);
    gl.disable(gl.SCISSOR_TEST);
    gl.viewport(0, 0, 1, 1);
    this.draw(true, width, height, [px - 0.5, py - 0.5], true);
    const out = new Float32Array(4);
    gl.readPixels(0, 0, 1, 1, gl.RGBA, gl.FLOAT, out);
    if (!(out[0] > 0)) return null;
    const { eye, dir } = this.pointerRay(x, y);
    return eye.map((v, i) => v + dir[i] * out[0]);
  }

  // Nearest edit whose bounding sphere the pointer's ray passes through.
  editAt(x, y) {
    const { eye, dir } = this.pointerRay(x, y);
    let best = null;
    let bestT = Infinity;
    for (const e of this.edits) {
      const r = e.kind === 0 ? e.size[0] : Math.hypot(...e.size) * 0.8;
      const c = e.pos.map((v, i) => v - eye[i]);
      const along = c[0] * dir[0] + c[1] * dir[1] + c[2] * dir[2];
      const gap2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2] - along * along;
      if (along > 0 && gap2 < r * r && along < bestT) { best = e; bestT = along; }
    }
    return best;
  }

  // The point under the pointer on a camera-facing plane through `pos`.
  onDragPlane(x, y, pos) {
    const { eye, forward, dir } = this.pointerRay(x, y);
    const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const t = dot(pos.map((v, i) => v - eye[i]), forward) / dot(dir, forward);
    return eye.map((v, i) => v + dir[i] * t);
  }

  grabEdit(edit, x, y) {
    const hit = this.pickSurface(x, y) || this.onDragPlane(x, y, edit.pos);
    this.grab = { edit, offset: edit.pos.map((v, i) => v - hit[i]) };
  }

  dragEdit(x, y) {
    // Follow the model's surface; off the model, slide on a camera-facing plane.
    const { edit, offset } = this.grab;
    const hit = this.pickSurface(x, y);
    this.moveEdit(edit, hit ? hit.map((v, i) => v + offset[i]) : this.onDragPlane(x, y, edit.pos));
  }

  setQuality(scale) {
    this.quality = scale;
    this.restart(false);
  }

  // The resolution being drawn at, as a fraction of the screen's.
  get renderScale() {
    if (this.quality) return this.quality;
    return this.moving ? this.previewScale : 1;
  }

  describe() {
    const f = this.field;
    if (this.sim) {
      const { bodies, stats } = this.sim;
      const asleep = bodies.length - this.sim.awakeCount;
      return `${bodies.length} bodies, ${asleep} asleep · physics ${stats.stepMs.toFixed(1)} ms/step, ${stats.contacts} contacts · ${(this.renderScale * 100).toFixed(0)}% resolution · ${this.frameMs.toFixed(0)} ms/frame`;
    }
    if (this.animation) {
      return this.paused
        ? 'Paused · refining to full quality'
        : `Animated with ${this.jointIndex.size} joints · Live · ${(this.renderScale * 100).toFixed(0)}% resolution · ${this.frameMs.toFixed(0)} ms/frame`;
    }
    if (this.mode === 'live') return `Live: evaluating the whole scene at every ray step · ${this.frameMs.toFixed(0)} ms/frame`;
    if (!f) return 'Compiling shaders…';
    if (f.baking) return `Baking ${Math.round(f.progress * 100)}% · showing Live until it finishes`;
    const mb = Math.round((f.voxelCount * 2 * 2) / 1e6);
    const parts = [
      `Grid ${f.dims.join('×')}`,
      `${(f.voxelCount / 1e6).toFixed(1)}M voxels, ${(f.voxel * 100).toFixed(1)} cm`,
      `${mb} MB`,
      `baked in ${(f.bakeMs / 1000).toFixed(1)} s`,
      `${this.frameMs.toFixed(0)} ms/frame`,
    ];
    if (this.edits.length) {
      parts.push(`${this.edits.length} edit${this.edits.length > 1 ? 's' : ''}, last update ${(f.lastUpdate / 1e6).toFixed(2)}M voxels`);
    }
    return parts.join(' · ');
  }

  resize() {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(this.canvas.clientWidth * dpr));
    const h = Math.max(1, Math.round(this.canvas.clientHeight * dpr));
    if (w === this.canvas.width && h === this.canvas.height && this.texReady) return;
    this.canvas.width = w;
    this.canvas.height = h;
    const gl = this.gl;
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    if (this.floatTarget) gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA16F, w, h, 0, gl.RGBA, gl.HALF_FLOAT, null);
    else gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, null);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.tex, 0);
    this.texReady = true;
    this.restart(true);
  }

  // ---- input ---------------------------------------------------------------

  bindInput() {
    const c = this.canvas;
    const pointers = new Map();
    let pinch = 0;
    c.addEventListener('pointerdown', (e) => {
      c.setPointerCapture(e.pointerId);
      pointers.set(e.pointerId, [e.clientX, e.clientY]);
      c.classList.add('dragging');
      pinch = 0;
      this.grab = null;
      if (pointers.size > 1 || e.button !== 0 || e.shiftKey || this.keys.has('Space')) return;
      if (this.tool !== 'orbit') {
        const hit = this.pickSurface(e.clientX, e.clientY);
        const edit = hit && this.addEdit(this.tool, hit);
        if (edit) {
          this.setTool('orbit');
          this.grabEdit(edit, e.clientX, e.clientY);
        }
      } else {
        const edit = this.editAt(e.clientX, e.clientY);
        if (edit) {
          this.grabEdit(edit, e.clientX, e.clientY);
        } else if (this.sim) {
          // Pick a body up. It is pulled towards the pointer by a spring.
          const hit = this.pickSurface(e.clientX, e.clientY, true);
          const body = hit && this.sim.bodyAt(...hit);
          if (body) {
            this.bodyGrab = { point: hit };
            this.sim.grabBody(body, hit);
          }
        }
      }
    });
    c.addEventListener('pointermove', (e) => {
      const last = pointers.get(e.pointerId);
      // Space held: the pointer turns the head, pressed or not, so a trackpad
      // can look around without clicking.
      if (this.keys.has('Space') && !this.grab && !this.bodyGrab && pointers.size <= 1) {
        if (last) { last[0] = e.clientX; last[1] = e.clientY; }
        this.look(e.movementX, e.movementY);
        return;
      }
      if (!last) {
        this.hover = this.editAt(e.clientX, e.clientY);
        c.classList.toggle('over-edit', !!this.hover);
        return;
      }
      const dx = e.clientX - last[0];
      const dy = e.clientY - last[1];
      last[0] = e.clientX;
      last[1] = e.clientY;
      if (this.bodyGrab && pointers.size === 1) {
        this.bodyGrab.point = this.onDragPlane(e.clientX, e.clientY, this.bodyGrab.point);
        this.sim.moveGrab(this.bodyGrab.point);
      } else if (this.grab && pointers.size === 1) {
        this.dragEdit(e.clientX, e.clientY);
      } else if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const gap = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) this.zoom(pinch / gap);
        pinch = gap;
      } else if (e.shiftKey || e.buttons === 2 || e.buttons === 4) {
        this.pan(dx, dy);
      } else {
        this.goal.theta -= dx * 0.35;
        // Looking up from below the orbit point is allowed, but orbiting will
        // not take the camera any further below it.
        this.goal.phi = Math.min(88, Math.max(Math.min(1.5, this.goal.phi), this.goal.phi + dy * 0.3));
      }
    });
    const release = (e) => {
      pointers.delete(e.pointerId);
      pinch = 0;
      this.grab = null;
      if (this.bodyGrab) this.sim.releaseGrab();
      this.bodyGrab = null;
      if (!pointers.size) c.classList.remove('dragging');
    };
    c.addEventListener('pointerup', release);
    c.addEventListener('pointercancel', release);
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const unit = e.deltaMode === 1 ? 16 : 1;
      const factor = Math.exp(e.deltaY * unit * (e.ctrlKey ? 0.01 : 0.0015));
      if (this.hover) this.resizeEdit(this.hover, 1 / factor);
      else this.zoom(factor);
    }, { passive: false });
    c.addEventListener('dblclick', () => { if (!this.hover) Object.assign(this.goal, this.home); });
    // Fly keys, as in Unity and Unreal: WASD moves, Q and E go down and up,
    // Shift goes faster. Held keys are applied each frame in fly().
    this.keys = new Set();
    const typing = (e) => e.target.closest && e.target.closest('input:not([type=range]), select, textarea, [contenteditable]');
    addEventListener('keydown', (e) => {
      if (typing(e)) return;
      if ((e.key === 'Delete' || e.key === 'Backspace') && this.hover) this.removeEdit(this.hover);
      if (e.key === 'Escape') this.setTool('orbit');
      if (FLY_KEYS[e.code] && !e.metaKey && !e.ctrlKey && !e.altKey) {
        this.keys.add(e.code);
        e.preventDefault();
        c.classList.toggle('looking', this.keys.has('Space'));
      }
    });
    addEventListener('keyup', (e) => {
      // Stops a focused button from being pressed by the space bar.
      if (e.code === 'Space' && this.keys.has('Space')) e.preventDefault();
      this.keys.delete(e.code);
      c.classList.toggle('looking', this.keys.has('Space'));
    });
    // A key released while the window is not focused never sends keyup.
    addEventListener('blur', () => { this.keys.clear(); c.classList.remove('looking'); });
  }

  // Turns the camera's head: it stays where it is and its orbit point swings
  // round in front of it. Applied to the eased camera too, so the eye does not
  // drift while easing catches up.
  look(dx, dy) {
    for (const cam of [this.cam, this.goal]) {
      const { eye } = this.basis(cam);
      cam.theta -= dx * 0.2;
      cam.phi = Math.min(88, Math.max(-85, cam.phi + dy * 0.2));
      const { forward } = this.basis(cam);
      cam.target = eye.map((v, i) => v + forward[i] * cam.dist);
    }
    this.restart(true);
  }

  // Moves the camera and its orbit point together, so orbiting afterwards turns
  // around whatever is now in front of the camera.
  fly(dt, fast) {
    let [x, y, z] = [0, 0, 0];
    for (const code of this.keys) {
      const [kx, ky, kz] = FLY_KEYS[code];
      x += kx; y += ky; z += kz;
    }
    if (!x && !y && !z) return;
    const { forward, right } = this.basis(this.goal);
    // Speed scales with the scene, so small and large models feel the same.
    const speed = this.bound.radius * 0.6 * (fast ? 3 : 1) * (Math.min(dt, 100) / 1000);
    this.goal.target = this.goal.target.map((v, i) => v + (forward[i] * z + right[i] * x + (i === 1 ? y : 0)) * speed);
  }

  zoom(factor) {
    this.goal.dist = Math.min(this.bound.radius * 8, Math.max(this.bound.radius * 0.25, this.goal.dist * factor));
  }

  pan(dx, dy) {
    const { right, up } = this.basis(this.goal);
    const k = (this.goal.dist * 2) / (3.2 * this.canvas.clientHeight);
    this.goal.target = this.goal.target.map((v, i) => v - right[i] * dx * k + up[i] * dy * k);
  }

  basis(cam) {
    const th = (cam.theta * Math.PI) / 180;
    const ph = (cam.phi * Math.PI) / 180;
    const dir = [Math.cos(ph) * Math.sin(th), Math.sin(ph), Math.cos(ph) * Math.cos(th)];
    const eye = cam.target.map((v, i) => v + dir[i] * cam.dist);
    const forward = dir.map((v) => -v);
    const right = [Math.cos(th), 0, -Math.sin(th)];
    const up = [
      right[1] * forward[2] - right[2] * forward[1],
      right[2] * forward[0] - right[0] * forward[2],
      right[0] * forward[1] - right[1] * forward[0],
    ];
    return { eye, forward, right, up };
  }

  // ---- render loop ------------------------------------------------------------

  ease() {
    const { cam, goal } = this;
    let delta = 0;
    const step = (v, g) => { delta = Math.max(delta, Math.abs(g - v)); return v + (g - v) * 0.3; };
    cam.theta = step(cam.theta, goal.theta);
    cam.phi = step(cam.phi, goal.phi);
    cam.dist = step(cam.dist, goal.dist);
    cam.target = cam.target.map((v, i) => step(v, goal.target[i]));
    if (delta < 0.002) {
      Object.assign(cam, { ...goal, target: [...goal.target] });
      return false;
    }
    return true;
  }

  draw(hq, width, height, jitter, pick = false) {
    const gl = this.gl;
    const { eye, forward, right, up } = this.basis(this.cam);
    const program = this.active;
    const u = program.uniforms;
    gl.useProgram(program.prog);
    const voxel = this.usingField ? this.field.voxel : 0;
    gl.uniform1f(u.uMinEps, Math.max(0.0006, voxel));
    gl.uniform1f(u.uCutDepth, Math.max(0.012, voxel * 1.5));
    gl.uniform1f(u.uPick, pick ? 1 : 0);
    gl.uniform1f(u.uHideBodies, pick && !this.pickBodies ? 1 : 0);
    gl.uniform4fv(u.uCut, this.cutPlane);
    // Picking looks at the unedited model, so a hole can glide along the surface
    // without falling into itself.
    setEditUniforms(gl, u, pick ? NO_EDITS : this.editPack);
    if (this.usingField) {
      gl.uniform3fv(u.uFieldSize, this.field.fieldSize);
      gl.activeTexture(gl.TEXTURE1);
      gl.bindTexture(gl.TEXTURE_3D, pick ? this.field.base : this.field.world);
      gl.activeTexture(gl.TEXTURE0);
    }
    gl.uniform2f(u.uRes, width, height);
    gl.uniform2f(u.uJitter, jitter[0], jitter[1]);
    gl.uniform3fv(u.uEye, eye);
    gl.uniformMatrix3fv(u.uCam, false, [...right, ...up, ...forward]);
    gl.uniform3fv(u.uCenter, this.rayBox.center);
    gl.uniform3fv(u.uExtent, this.rayBox.extent);
    gl.uniform3f(u.uStage, this.bound.center[0], this.bound.center[2], this.bound.radius);
    gl.uniform3f(u.uBoundsLo, ...this.bound.center.map((c, i) => c - this.bound.extent[i]));
    gl.uniform3fv(u.uBoundsSize, this.bound.extent.map((e) => e * 2));
    if (this.jointIndex.size) {
      gl.uniformMatrix3fv(u['uJointR[0]'], false, this.jointR);
      gl.uniform3fv(u['uJointT[0]'], this.jointT);
    }
    gl.uniform1f(u.uHQ, hq ? 1 : 0);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }

  show(sx, sy) {
    const gl = this.gl;
    gl.disable(gl.BLEND);
    gl.disable(gl.SCISSOR_TEST);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.canvas.width, this.canvas.height);
    gl.useProgram(this.present.prog);
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.uniform2f(this.present.uniforms.scale, sx, sy);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }

  frame(now) {
    requestAnimationFrame((t) => this.frame(t));
    const dt = now - (this.lastFrame || now);
    this.lastFrame = now;
    if (!this.live) return;

    const gl = this.gl;
    const { width, height } = this.canvas;
    this.stepField();
    const animating = this.animation && !this.paused;
    if (animating) {
      this.clock += Math.min(dt, 100) / 1000;
      this.applyPose();
    }
    const simulating = this.sim && !this.paused && this.sim.awakeCount > 0;
    if (simulating) {
      this.sim.advance(Math.min(dt, 50) / 1000);
      this.applySimulation();
    }
    if (this.keys.size) this.fly(dt, this.keys.has('ShiftLeft') || this.keys.has('ShiftRight'));
    if (this.ease() || animating || simulating || now < (this.activeUntil || 0)) this.restart(true);
    else if (this.moving) this.restart(false);
    if (this.moving) this.frameMs += (dt - this.frameMs) * 0.1;
    if (now - (this.lastStatus || 0) > 250) { this.lastStatus = now; this.onChange(); }

    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);

    // A fixed quality draws the same image moving or still, so nothing pops when
    // the camera stops; only anti-aliasing is added while still.
    const fixed = this.quality > 0;
    if (this.moving) {
      // Hold preview frame rate by trading resolution.
      if (this.drewLast && !fixed) {
        if (dt > 30) this.previewScale = Math.max(0.2, this.previewScale * 0.88);
        else if (dt < 19) this.previewScale = Math.min(1, this.previewScale * 1.04);
      }
      const scale = this.renderScale;
      const w = Math.max(1, Math.round(width * scale));
      const h = Math.max(1, Math.round(height * scale));
      gl.disable(gl.BLEND);
      gl.disable(gl.SCISSOR_TEST);
      gl.viewport(0, 0, w, h);
      // Animated scenes are the final image, so keep shadows and reflections on.
      this.draw(!!this.animation || fixed, w, h, [0, 0]);
      this.show(w / width, h / height);
      this.drewLast = true;
      return;
    }

    if (this.sample >= this.maxSamples) { this.drewLast = false; return; }
    // While baking, one sample is enough to look at. Leave the GPU to the bake.
    if (this.field && this.field.baking && this.sample >= 1) return;

    // Refine: each sample is split into strips across several frames on slower
    // GPUs, so a heavy scene cannot stall the browser.
    if (this.strip === 0 && this.drewLast && this.sample > 0) {
      if (dt > 50) this.strips = Math.min(16, this.strips * 2);
      else if (dt < 20 && this.strips > 1) this.strips = Math.ceil(this.strips / 2);
    }
    const n = this.sample;
    const halton = (i, b) => { let f = 1, r = 0; while (i > 0) { f /= b; r += f * (i % b); i = Math.floor(i / b); } return r; };
    const jitter = n === 0 ? [0, 0] : [halton(n, 2) - 0.5, halton(n, 3) - 0.5];
    const scale = this.renderScale;
    const w = Math.max(1, Math.round(width * scale));
    const h = Math.max(1, Math.round(height * scale));
    gl.viewport(0, 0, w, h);
    gl.enable(gl.SCISSOR_TEST);
    const y0 = Math.floor((this.strip * h) / this.strips);
    const y1 = Math.floor(((this.strip + 1) * h) / this.strips);
    gl.scissor(0, y0, w, y1 - y0);
    if (n === 0) gl.disable(gl.BLEND);
    else {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.CONSTANT_ALPHA, gl.ONE_MINUS_CONSTANT_ALPHA);
      gl.blendColor(0, 0, 0, 1 / (n + 1));
    }
    this.draw(true, w, h, jitter);
    this.drewLast = true;

    this.strip++;
    if (this.strip >= this.strips) {
      this.strip = 0;
      this.sample++;
      this.show(w / width, h / height);
    }
  }
}
