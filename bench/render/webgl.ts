import {DrawScene, camera, colorRgb} from './geometry.js';

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
  private readonly color: WebGLUniformLocation;
  private revision=0;
  private disposed=false;
  uploadBytes=0;
  readonly gpuBytes: number;
  readonly drawCalls: number;
  constructor(readonly canvas: HTMLCanvasElement, readonly scene: DrawScene) {
    const gl=canvas.getContext('webgl2',{antialias:true,depth:false,stencil:false,preserveDrawingBuffer:false});
    if (!gl) throw new Error('WebGL 2 unavailable');
    this.gl=gl;
    const program=gl.createProgram();
    if (!program) throw new Error('Cannot allocate WebGL program');
    this.program=program;
    try {
      for (const [type,source] of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]] as const) {
        const shader=gl.createShader(type);
        if (!shader) throw new Error('Cannot allocate shader');
        gl.shaderSource(shader,source); gl.compileShader(shader); gl.attachShader(program,shader); gl.deleteShader(shader);
      }
      gl.linkProgram(program);
      if (!gl.getProgramParameter(program,gl.LINK_STATUS)) throw new Error(`Shader link failed: ${gl.getProgramInfoLog(program)}`);
      gl.useProgram(program);
      const view=camera(scene.rect,canvas.width,canvas.height), location=gl.getUniformLocation(program,'camera');
      const color=gl.getUniformLocation(program,'color');
      if (!location || !color) throw new Error('Missing shader uniforms');
      this.color=color;
      gl.uniform4f(location,2*view.scale/canvas.width,2*view.scale/canvas.height,
        2*view.x/canvas.width-1,1-2*view.y/canvas.height);
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
      }
      gl.bindVertexArray(null); gl.disable(gl.DEPTH_TEST); gl.disable(gl.BLEND); gl.disable(gl.CULL_FACE);
      gl.clearColor(16/255,24/255,32/255,1); gl.viewport(0,0,canvas.width,canvas.height);
      if (gl.getError()!==gl.NO_ERROR) throw new Error('WebGL setup failed');
      this.gpuBytes=scene.poses.byteLength+scene.meshes.reduce((n,m)=>n+m.triangles.byteLength,0);
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
    gl.clear(gl.COLOR_BUFFER_BIT); gl.useProgram(this.program);
    for (let i=0; i<scene.batches.length; ++i) {
      const b=scene.batches[i], c=colorRgb[b.color];
      gl.uniform3f(this.color,c[0],c[1],c[2]); gl.bindVertexArray(this.arrays[i]);
      gl.drawArraysInstanced(gl.TRIANGLES,0,scene.meshes[b.mesh].triangles.length/2,b.count);
    }
  }
  dispose(): void {
    if (!this.disposed) {
      for (const vao of this.arrays) this.gl.deleteVertexArray(vao);
      for (const buffer of this.buffers) this.gl.deleteBuffer(buffer);
      this.gl.deleteProgram(this.program); this.disposed=true;
    }
  }
}
