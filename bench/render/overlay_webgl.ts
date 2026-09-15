import {Overlay,paletteValues,markerTriangles} from './overlay.js';
const vertex=`#version 300 es
precision highp float;
layout(location=0) in vec2 unit;
layout(location=1) in vec4 item;
layout(location=2) in float code;
uniform vec2 scale;
uniform bool marker;
uniform vec3 palette[16];
flat out vec3 shade;
void main() {
  vec2 delta=item.zw-item.xy;
  vec2 normal=dot(delta,delta)>0.0?vec2(-delta.y,delta.x)*inversesqrt(dot(delta,delta)):vec2(0.0);
  vec2 pixel=marker?item.xy+3.0*unit:item.xy+unit.x*delta+unit.y*normal;
  gl_Position=vec4(pixel*scale+vec2(-1.0,1.0),0.0,1.0);
  shade=palette[int(code)];
}`;
const fragment=`#version 300 es
precision highp float;
flat in vec3 shade;
out vec4 pixel;
void main(){pixel=vec4(shade,1.0);}`;

/** One instanced quad per one-pixel line, one cached 32-gon per marker.
 * Buffers and VAOs are fixed; uploads cover only active prefixes, without
 * constructing subarray views. GPU completion is not measured here.
 */
export class OverlayGpu {
  private readonly program:WebGLProgram;
  private readonly buffers:WebGLBuffer[]=[];
  private readonly vaos:WebGLVertexArrayObject[]=[];
  private readonly lineBuffer:WebGLBuffer;
  private readonly markerBuffer:WebGLBuffer;
  private readonly mode:WebGLUniformLocation;
  private revision=0;
  readonly bytes:number;
  uploadBytes=0;drawCalls=0;
  constructor(private readonly gl:WebGL2RenderingContext,private readonly overlay:Overlay,width:number,height:number){
    const program=gl.createProgram();if(!program)throw new Error('Overlay program allocation failed');this.program=program;
    try{
      for(const [type,source]of [[gl.VERTEX_SHADER,vertex],[gl.FRAGMENT_SHADER,fragment]]as const){
        const shader=gl.createShader(type);if(!shader)throw new Error('Overlay shader allocation failed');
        gl.shaderSource(shader,source);gl.compileShader(shader);gl.attachShader(program,shader);gl.deleteShader(shader);
      }
      gl.linkProgram(program);if(!gl.getProgramParameter(program,gl.LINK_STATUS))throw new Error(`Overlay shader failed: ${gl.getProgramInfoLog(program)}`);
      gl.useProgram(program);const scale=gl.getUniformLocation(program,'scale'),palette=gl.getUniformLocation(program,'palette[0]'),mode=gl.getUniformLocation(program,'marker');
      if(!scale||!palette||!mode)throw new Error('Overlay uniforms missing');this.mode=mode;
      gl.uniform2f(scale,2/width,-2/height);gl.uniform3fv(palette,paletteValues);
      const quad=new Float32Array([0,-.5,1,-.5,1,.5,0,-.5,1,.5,0,.5]);
      this.lineBuffer=this.setup(quad,overlay.lines.byteLength,5,4);
      this.markerBuffer=this.setup(markerTriangles,overlay.markers.byteLength,3,2);
      this.bytes=quad.byteLength+markerTriangles.byteLength+overlay.lines.byteLength+overlay.markers.byteLength;
      if(gl.getError()!==gl.NO_ERROR)throw new Error('Overlay GPU setup failed');
    }catch(error){this.dispose();throw error;}
  }
  private buffer():WebGLBuffer{
    const buffer=this.gl.createBuffer();if(!buffer)throw new Error('Overlay buffer allocation failed');
    this.buffers.push(buffer);this.gl.bindBuffer(this.gl.ARRAY_BUFFER,buffer);return buffer;
  }
  private setup(mesh:Float32Array,bytes:number,stride:number,components:number):WebGLBuffer{
    const gl=this.gl,vao=gl.createVertexArray();if(!vao)throw new Error('Overlay VAO allocation failed');
    this.vaos.push(vao);gl.bindVertexArray(vao);this.buffer();gl.bufferData(gl.ARRAY_BUFFER,mesh,gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);gl.vertexAttribPointer(0,2,gl.FLOAT,false,8,0);
    const buffer=this.buffer();gl.bufferData(gl.ARRAY_BUFFER,bytes,gl.DYNAMIC_DRAW);
    gl.enableVertexAttribArray(1);gl.vertexAttribPointer(1,components,gl.FLOAT,false,stride*4,0);gl.vertexAttribDivisor(1,1);
    gl.enableVertexAttribArray(2);gl.vertexAttribPointer(2,1,gl.FLOAT,false,stride*4,(stride-1)*4);gl.vertexAttribDivisor(2,1);
    gl.bindVertexArray(null);return buffer;
  }
  draw():void{
    const gl=this.gl,o=this.overlay;this.uploadBytes=0;this.drawCalls=0;
    if(this.revision!==o.revision){
      if(o.lineCount){gl.bindBuffer(gl.ARRAY_BUFFER,this.lineBuffer);gl.bufferSubData(gl.ARRAY_BUFFER,0,o.lines,0,o.lineCount*5);}
      if(o.markerCount){gl.bindBuffer(gl.ARRAY_BUFFER,this.markerBuffer);gl.bufferSubData(gl.ARRAY_BUFFER,0,o.markers,0,o.markerCount*3);}
      this.uploadBytes=o.lineCount*20+o.markerCount*12;this.revision=o.revision;
    }
    gl.useProgram(this.program);
    if(o.lineCount){gl.uniform1i(this.mode,0);gl.bindVertexArray(this.vaos[0]);gl.drawArraysInstanced(gl.TRIANGLES,0,6,o.lineCount);++this.drawCalls;}
    if(o.markerCount){gl.uniform1i(this.mode,1);gl.bindVertexArray(this.vaos[1]);gl.drawArraysInstanced(gl.TRIANGLES,0,markerTriangles.length/2,o.markerCount);++this.drawCalls;}
  }
  dispose():void{for(const vao of this.vaos)this.gl.deleteVertexArray(vao);for(const buffer of this.buffers)this.gl.deleteBuffer(buffer);this.gl.deleteProgram(this.program);}
}
