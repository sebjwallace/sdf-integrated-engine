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

const FRAGMENT_SHADER = (sceneCode) => `#version 300 es
precision highp float;
uniform vec2 uRes,uJitter;
uniform vec3 uEye;
uniform mat3 uCam;
uniform vec3 uCenter,uExtent;  // bounding box: everything lives inside this
uniform float uHQ;
out vec4 fragColor;
${SDF_LIBRARY}
${sceneCode}

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
    t+=d;
  }
  return d<.01&&t<r.y?t:-1.;
}

vec3 normalAt(vec3 p,float e){
  const vec2 k=vec2(1,-1);
  return normalize(k.xyy*map(p+k.xyy*e)+k.yyx*map(p+k.yyx*e)+k.yxy*map(p+k.yxy*e)+k.xxx*map(p+k.xxx*e));
}

float softShadow(vec3 ro,vec3 rd,float k,int steps){
  vec2 r=boundHit(ro,rd);
  if(r.x>r.y)return 1.;
  float res=1.,t=max(r.x,.01);
  for(int i=0;i<steps&&t<r.y;i++){
    float h=map(ro+rd*t);
    res=min(res,k*h/t);
    if(res<.002)return 0.;
    t+=clamp(h,.012,.3);
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
  vec3 n=normalAt(p,.002);
  vec3 c=lightSurface(p,n,rd,m,1.,.75);
  vec3 F=mix(vec3(.05),m.col,m.metal);
  return c+F*studio(reflect(rd,n),max(m.rough,.3));
}

vec3 shadeObject(vec3 ro,vec3 rd,float t){
  vec3 p=ro+rd*t;
  vec3 n=normalAt(p,max(.0006,.0004*t));
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
  float dist=length(p.xz-uCenter.xz),R=length(uExtent);
  vec3 n=vec3(0,1,0);
  float sh=1.,ao=1.;
  if(dist<R*2.5){
    sh=softShadow(p+n*.002,KEY,9.,uHQ>.5?40:12);
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
      vec3 rn=normalAt(rp,.003);
      c+=F*lightSurface(rp,rn,r,m,1.,.7)*exp(-rt*1.1);
    }
  }
  return mix(c,backdrop(rd),smoothstep(R*1.6,R*5.,dist));
}

vec3 aces(vec3 x){return clamp(x*(2.51*x+.03)/(x*(2.43*x+.59)+.14),0.,1.);}

void main(){
  vec2 ndc=(2.*(gl_FragCoord.xy+uJitter)-uRes)/uRes.y;
  vec3 rd=normalize(uCam*vec3(ndc,FOCAL));
  float tFloor=rd.y<0.?-uEye.y/rd.y:-1.;
  float t=march(uEye,rd,uHQ>.5?128:48);
  vec3 c;
  if(t>0.&&(tFloor<0.||t<tFloor))c=shadeObject(uEye,rd,t);
  else if(tFloor>0.)c=shadeFloor(uEye,rd,tFloor);
  else c=backdrop(rd);
  // Soft vignette, tone mapping, gamma.
  c*=1.-.28*dot(ndc*.55,ndc*.55);
  fragColor=vec4(pow(aces(c),vec3(1./2.2)),1.);
}`;

class Viewer {
  constructor(canvas, onError) {
    this.canvas = canvas;
    this.onError = onError;
    const gl = (this.gl = canvas.getContext('webgl2', { alpha: false, antialias: false, depth: false, powerPreference: 'high-performance' }));
    if (!gl) throw new Error('This browser does not support WebGL2.');

    this.floatTarget = !!gl.getExtension('EXT_color_buffer_float');
    gl.bindVertexArray(gl.createVertexArray());
    this.present = this.link(PRESENT_SHADER);
    this.fbo = gl.createFramebuffer();
    this.tex = gl.createTexture();
    gl.disable(gl.DEPTH_TEST);

    // Camera: spherical coordinates around a target. `goal` is where the
    // pointer wants the camera; `cam` eases towards it.
    this.cam = { theta: 30, phi: 15, dist: 6, target: [0, 0.5, 0] };
    this.goal = { ...this.cam };
    this.previewScale = 0.6;
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
    const shape = scene.build();
    const program = this.link(FRAGMENT_SHADER(SDF.compile(shape)));
    if (this.scene) this.gl.deleteProgram(this.scene.prog);
    this.scene = program;
    this.bound = { center: scene.center, extent: scene.extent, radius: Math.hypot(...scene.extent) };
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
    });
    c.addEventListener('pointermove', (e) => {
      const last = pointers.get(e.pointerId);
      if (!last) return;
      const dx = e.clientX - last[0];
      const dy = e.clientY - last[1];
      last[0] = e.clientX;
      last[1] = e.clientY;
      if (pointers.size === 2) {
        const [a, b] = [...pointers.values()];
        const gap = Math.hypot(a[0] - b[0], a[1] - b[1]);
        if (pinch) this.zoom(pinch / gap);
        pinch = gap;
      } else if (e.shiftKey || e.buttons === 2 || e.buttons === 4) {
        this.pan(dx, dy);
      } else {
        this.goal.theta -= dx * 0.35;
        this.goal.phi = Math.min(88, Math.max(1.5, this.goal.phi + dy * 0.3));
      }
    });
    const release = (e) => {
      pointers.delete(e.pointerId);
      pinch = 0;
      if (!pointers.size) c.classList.remove('dragging');
    };
    c.addEventListener('pointerup', release);
    c.addEventListener('pointercancel', release);
    c.addEventListener('contextmenu', (e) => e.preventDefault());
    c.addEventListener('wheel', (e) => {
      e.preventDefault();
      const unit = e.deltaMode === 1 ? 16 : 1;
      this.zoom(Math.exp(e.deltaY * unit * (e.ctrlKey ? 0.01 : 0.0015)));
    }, { passive: false });
    c.addEventListener('dblclick', () => Object.assign(this.goal, this.home));
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

  draw(hq, width, height, jitter) {
    const gl = this.gl;
    const { eye, forward, right, up } = this.basis(this.cam);
    const u = this.scene.uniforms;
    gl.useProgram(this.scene.prog);
    gl.uniform2f(u.uRes, width, height);
    gl.uniform2f(u.uJitter, jitter[0], jitter[1]);
    gl.uniform3fv(u.uEye, eye);
    gl.uniformMatrix3fv(u.uCam, false, [...right, ...up, ...forward]);
    gl.uniform3fv(u.uCenter, this.bound.center);
    gl.uniform3fv(u.uExtent, this.bound.extent);
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
    if (!this.scene) return;

    const gl = this.gl;
    const { width, height } = this.canvas;
    if (this.ease()) this.restart(true);
    else if (this.moving) this.restart(false);

    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);

    if (this.moving) {
      // Hold preview frame rate by trading resolution.
      if (this.drewLast) {
        if (dt > 30) this.previewScale = Math.max(0.2, this.previewScale * 0.88);
        else if (dt < 19) this.previewScale = Math.min(1, this.previewScale * 1.04);
      }
      const w = Math.max(1, Math.round(width * this.previewScale));
      const h = Math.max(1, Math.round(height * this.previewScale));
      gl.disable(gl.BLEND);
      gl.disable(gl.SCISSOR_TEST);
      gl.viewport(0, 0, w, h);
      this.draw(false, w, h, [0, 0]);
      this.show(w / width, h / height);
      this.drewLast = true;
      return;
    }

    if (this.sample >= this.maxSamples) { this.drewLast = false; return; }

    // Refine: each sample is split into strips across several frames on slower
    // GPUs, so a heavy scene cannot stall the browser.
    if (this.strip === 0 && this.drewLast && this.sample > 0) {
      if (dt > 50) this.strips = Math.min(16, this.strips * 2);
      else if (dt < 20 && this.strips > 1) this.strips = Math.ceil(this.strips / 2);
    }
    const n = this.sample;
    const halton = (i, b) => { let f = 1, r = 0; while (i > 0) { f /= b; r += f * (i % b); i = Math.floor(i / b); } return r; };
    const jitter = n === 0 ? [0, 0] : [halton(n, 2) - 0.5, halton(n, 3) - 0.5];
    gl.viewport(0, 0, width, height);
    gl.enable(gl.SCISSOR_TEST);
    const y0 = Math.floor((this.strip * height) / this.strips);
    const y1 = Math.floor(((this.strip + 1) * height) / this.strips);
    gl.scissor(0, y0, width, y1 - y0);
    if (n === 0) gl.disable(gl.BLEND);
    else {
      gl.enable(gl.BLEND);
      gl.blendFunc(gl.CONSTANT_ALPHA, gl.ONE_MINUS_CONSTANT_ALPHA);
      gl.blendColor(0, 0, 0, 1 / (n + 1));
    }
    this.draw(true, width, height, jitter);
    this.drewLast = true;

    this.strip++;
    if (this.strip >= this.strips) {
      this.strip = 0;
      this.sample++;
      this.show(1, 1);
    }
  }
}
