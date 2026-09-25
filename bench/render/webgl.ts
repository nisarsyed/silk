import {DrawScene, camera, colorRgb} from './geometry.js';
import {Overlay,paletteValues} from './overlay.js';
import {OverlayGpu} from './overlay_webgl.js';

const vertex=`#version 300 es
precision highp float;
layout(location=0) in vec2 point;
layout(location=1) in vec4 pose;
uniform vec4 camera;
void main() {
  vec2 world=pose.xy+vec2(pose.z*point.x-pose.w*point.y,pose.w*point.x+pose.z*point.y);
  gl_Position=vec4(world*camera.xy+camera.zw,0.0,1.0);
}`;
const fragment=`#version 300 es
precision highp float;
uniform vec3 color;
out vec4 pixel;
void main() { pixel=vec4(color,1.0); }
`;

/** One instanced draw per contiguous geometry/color range preserves source
 * ordering. VAOs and GPU storage are fixed at setup. Only changing transforms
 * upload in steady state (16 bytes/instance); frozen scenes upload once.
 * No getParameter/getError/readPixels or completion waits occur in draw().
 */
export class WebglCandidate {
  readonly gl: WebGL2RenderingContext;
  private readonly program: WebGLProgram;
  private readonly buffers: WebGLBuffer[]=[];
  private readonly arrays: WebGLVertexArrayObject[]=[];
  private readonly transforms: WebGLBuffer;
  private readonly color: WebGLUniformLocation|null;
  private readonly colors?:WebGLBuffer;
  private readonly overlayGpu?:OverlayGpu;
  private overlayRevision=0;
  private revision=0;
  private disposed=false;
  uploadBytes=0;
  readonly gpuBytes: number;
  drawCalls: number;
  constructor(readonly canvas: HTMLCanvasElement|OffscreenCanvas, readonly scene: DrawScene,readonly overlay?:Overlay) {
    const gl=canvas.getContext('webgl2',{antialias:true,depth:false,stencil:false,preserveDrawingBuffer:false}) as WebGL2RenderingContext|null;
    if (!gl) throw new Error('WebGL 2 unavailable');
    this.gl=gl;
    const program=gl.createProgram();
    if (!program) throw new Error('Cannot allocate WebGL program');
    this.program=program;
    try {
      const bodyVertex=overlay?vertex.replace('uniform vec4 camera;',
        'uniform vec4 camera;\nlayout(location=2) in float code;\nuniform vec3 palette[16];\nflat out vec3 shade;')
        .replace('void main() {','void main() { shade=palette[int(code)];'):vertex;
      const bodyFragment=overlay?fragment.replace('uniform vec3 color;','flat in vec3 shade;').replace('vec4(color,1.0)','vec4(shade,1.0)'):fragment;
      for (const [type,source] of [[gl.VERTEX_SHADER,bodyVertex],[gl.FRAGMENT_SHADER,bodyFragment]] as const) {
        const shader=gl.createShader(type);
        if (!shader) throw new Error('Cannot allocate shader');
        gl.shaderSource(shader,source); gl.compileShader(shader); gl.attachShader(program,shader); gl.deleteShader(shader);
      }
      gl.linkProgram(program);
      if (!gl.getProgramParameter(program,gl.LINK_STATUS)) throw new Error(`Shader link failed: ${gl.getProgramInfoLog(program)}`);
      gl.useProgram(program);
      const view=camera(scene.rect,canvas.width,canvas.height), location=gl.getUniformLocation(program,'camera');
      const color=gl.getUniformLocation(program,'color');
      if (!location || (!overlay&&!color)) throw new Error('Missing shader uniforms');
      this.color=color;
      gl.uniform4f(location,2*view.scale/canvas.width,2*view.scale/canvas.height,
        2*view.x/canvas.width-1,1-2*view.y/canvas.height);
      if(overlay){
        const palette=gl.getUniformLocation(program,'palette[0]');if(!palette)throw new Error('Body palette missing');
        gl.uniform3fv(palette,paletteValues);this.colors=this.buffer();gl.bufferData(gl.ARRAY_BUFFER,overlay.bodyColors.byteLength,gl.DYNAMIC_DRAW);
      }
      this.transforms=this.buffer();
      gl.bufferData(gl.ARRAY_BUFFER,scene.poses.byteLength,gl.DYNAMIC_DRAW);
      const meshes=scene.meshes.map(mesh=>{
        const buffer=this.buffer(); gl.bufferData(gl.ARRAY_BUFFER,mesh.triangles,gl.STATIC_DRAW); return buffer;
      });
      for (const batch of scene.batches) {
        const vao=gl.createVertexArray(); if (!vao) throw new Error('Cannot allocate vertex array');
        this.arrays.push(vao); gl.bindVertexArray(vao);
        gl.bindBuffer(gl.ARRAY_BUFFER,meshes[batch.mesh]); gl.enableVertexAttribArray(0);
        gl.vertexAttribPointer(0,2,gl.FLOAT,false,8,0);
        gl.bindBuffer(gl.ARRAY_BUFFER,this.transforms); gl.enableVertexAttribArray(1);
        gl.vertexAttribPointer(1,4,gl.FLOAT,false,16,batch.first*16); gl.vertexAttribDivisor(1,1);
        if(this.colors){gl.bindBuffer(gl.ARRAY_BUFFER,this.colors);gl.enableVertexAttribArray(2);gl.vertexAttribPointer(2,1,gl.FLOAT,false,4,batch.first*4);gl.vertexAttribDivisor(2,1);}
      }
      gl.bindVertexArray(null); gl.disable(gl.DEPTH_TEST); gl.disable(gl.BLEND); gl.disable(gl.CULL_FACE);
      gl.clearColor(16/255,24/255,32/255,1); gl.viewport(0,0,canvas.width,canvas.height);
      if (gl.getError()!==gl.NO_ERROR) throw new Error('WebGL setup failed');
      if(overlay)this.overlayGpu=new OverlayGpu(gl,overlay,canvas.width,canvas.height);
      this.gpuBytes=(overlay?.bodyColors.byteLength??0)+(this.overlayGpu?.bytes??0)+scene.poses.byteLength+scene.meshes.reduce((n,m)=>n+m.triangles.byteLength,0);
      this.drawCalls=scene.batches.length;
    } catch (error) { this.dispose(); throw error; }
  }
  private buffer(): WebGLBuffer {
    const buffer=this.gl.createBuffer(); if (!buffer) throw new Error('Cannot allocate GPU buffer');
    this.buffers.push(buffer); this.gl.bindBuffer(this.gl.ARRAY_BUFFER,buffer); return buffer;
  }
  draw(): void {
    if (this.disposed) throw new Error('Disposed WebGL candidate');
    const gl=this.gl, scene=this.scene;
    this.uploadBytes=0;
    if (this.revision!==scene.revision) {
      gl.bindBuffer(gl.ARRAY_BUFFER,this.transforms); gl.bufferSubData(gl.ARRAY_BUFFER,0,scene.poses);
      this.uploadBytes=scene.poses.byteLength; this.revision=scene.revision;
    }
    if(this.overlay&&this.colors&&this.overlayRevision!==this.overlay.revision){
      gl.bindBuffer(gl.ARRAY_BUFFER,this.colors);gl.bufferSubData(gl.ARRAY_BUFFER,0,this.overlay.bodyColors);
      this.uploadBytes+=this.overlay.bodyColors.byteLength;this.overlayRevision=this.overlay.revision;
    }
    this.drawCalls=scene.batches.length;
    gl.clear(gl.COLOR_BUFFER_BIT); gl.useProgram(this.program);
    for (let i=0; i<scene.batches.length; ++i) {
      const b=scene.batches[i], c=colorRgb[b.color];
      if(this.color)gl.uniform3f(this.color,c[0],c[1],c[2]); gl.bindVertexArray(this.arrays[i]);
      gl.drawArraysInstanced(gl.TRIANGLES,0,scene.meshes[b.mesh].triangles.length/2,b.count);
    }
    if(this.overlayGpu){this.overlayGpu.draw();this.uploadBytes+=this.overlayGpu.uploadBytes;this.drawCalls+=this.overlayGpu.drawCalls;}
  }
  dispose(): void {
    if (!this.disposed) {
      for (const vao of this.arrays) this.gl.deleteVertexArray(vao);
      for (const buffer of this.buffers) this.gl.deleteBuffer(buffer);
      this.overlayGpu?.dispose();this.gl.deleteProgram(this.program); this.disposed=true;
    }
  }
}
