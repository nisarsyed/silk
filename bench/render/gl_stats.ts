type BufferDataCall=(target:number,data:number|AllowSharedBufferSource|null,usage:number,offset?:number,length?:number)=>void;
type BufferSubDataCall=(target:number,destination:number,data:AllowSharedBufferSource,offset?:number,length?:number)=>void;
const names=Object.freeze(['bufferData','bufferSubData','drawArrays','drawElements','drawArraysInstanced',
  'drawElementsInstanced','drawRangeElements'] as const);
const owners=new WeakSet<WebGL2RenderingContext>();

/** Host buffer bytes requested through WebGL, not physical bus traffic.
 * WebGL 2 offsets/lengths are elements, except DataView uses bytes. Zero length
 * means the remaining view. No typed-array views or argument arrays are made.
 */
export function bufferSourceBytes(data:AllowSharedBufferSource,offset=0,length=0):number|null {
  if(!Number.isSafeInteger(offset)||offset<0||!Number.isSafeInteger(length)||length<0)return null;
  const bytes=data?.byteLength;
  if(!Number.isSafeInteger(bytes)||bytes<0)return null;
  if(!ArrayBuffer.isView(data))return offset===0&&length===0?bytes:null;
  const unit='BYTES_PER_ELEMENT' in data?Number(data.BYTES_PER_ELEMENT):1;
  if(!Number.isSafeInteger(unit)||unit<1||bytes%unit)return null;
  const elements=bytes/unit,count=length===0?elements-offset:length;
  return offset<=elements&&count<=elements-offset?count*unit:null;
}

/** Shared developer-only call observer for WebGL and raylib's WebGL context.
 * Install before warm-up. Seven fixed wrappers count calls during explicit
 * draw sections, without GL queries, argument arrays or per-call allocation.
 * Counts describe submitted API calls, not driver-internal draws/allocations.
 * Both GL candidates incur the same observer per API call; its CPU overhead
 * remains inside measured draw/critical-path time. No engine or pin changes.
 */
export class GlStats {
  uploadBytes=0;drawCalls=0;bufferDataCalls=0;bufferSubDataCalls=0;valid=true;
  private collecting=false;private disposed=false;
  private sections=0;private totalUploadBytes=0;private totalDrawCalls=0;
  private readonly descriptors:(PropertyDescriptor|undefined)[];
  constructor(private readonly gl:WebGL2RenderingContext){
    if(owners.has(gl))throw new Error('GL context already has an observer');
    this.descriptors=names.map(name=>Object.getOwnPropertyDescriptor(gl,name));
    const data=gl.bufferData.bind(gl) as BufferDataCall,sub=gl.bufferSubData.bind(gl) as BufferSubDataCall;
    const arrays=gl.drawArrays.bind(gl),elements=gl.drawElements.bind(gl),
      arraysInstanced=gl.drawArraysInstanced.bind(gl),elementsInstanced=gl.drawElementsInstanced.bind(gl),
      range=gl.drawRangeElements.bind(gl),self=this;
    try{
      gl.bufferData=function(target:number,source:number|AllowSharedBufferSource|null,usage:number,offset?:number,length?:number){
        if(arguments.length<3)throw new TypeError('bufferData requires three arguments');
        if(self.collecting){++self.bufferDataCalls;
          if(typeof source!=='number'&&source!==null)self.bytes(bufferSourceBytes(source,offset,length));}
        if(arguments.length===3)data(target,source,usage);
        else if(arguments.length===4)data(target,source,usage,offset);
        else data(target,source,usage,offset,length);
      } as typeof gl.bufferData;
      gl.bufferSubData=function(target:number,destination:number,source:AllowSharedBufferSource,offset?:number,length?:number){
        if(arguments.length<3)throw new TypeError('bufferSubData requires three arguments');
        if(self.collecting){++self.bufferSubDataCalls;self.bytes(bufferSourceBytes(source,offset,length));}
        if(arguments.length===3)sub(target,destination,source);
        else if(arguments.length===4)sub(target,destination,source,offset);
        else sub(target,destination,source,offset,length);
      } as typeof gl.bufferSubData;
      gl.drawArrays=function(mode,first,count){if(arguments.length<3)throw new TypeError('drawArrays requires three arguments');
        if(self.collecting)++self.drawCalls;arrays(mode,first,count);};
      gl.drawElements=function(mode,count,type,offset){if(arguments.length<4)throw new TypeError('drawElements requires four arguments');
        if(self.collecting)++self.drawCalls;elements(mode,count,type,offset);};
      gl.drawArraysInstanced=function(mode,first,count,instances){if(arguments.length<4)throw new TypeError('drawArraysInstanced requires four arguments');
        if(self.collecting)++self.drawCalls;arraysInstanced(mode,first,count,instances);};
      gl.drawElementsInstanced=function(mode,count,type,offset,instances){if(arguments.length<5)throw new TypeError('drawElementsInstanced requires five arguments');
        if(self.collecting)++self.drawCalls;elementsInstanced(mode,count,type,offset,instances);};
      gl.drawRangeElements=function(mode,start,end,count,type,offset){if(arguments.length<6)throw new TypeError('drawRangeElements requires six arguments');
        if(self.collecting)++self.drawCalls;range(mode,start,end,count,type,offset);};
      owners.add(gl);
    }catch(error){this.restore();throw error;}
  }
  private bytes(bytes:number|null):void {
    if(bytes===null||!Number.isSafeInteger(this.uploadBytes+bytes)){this.valid=false;return;}
    this.uploadBytes+=bytes;
  }
  begin():void {
    if(this.disposed||this.collecting)throw new Error('Invalid GL observer lifetime');
    this.uploadBytes=0;this.drawCalls=0;this.bufferDataCalls=0;this.bufferSubDataCalls=0;this.collecting=true;
  }
  end():void {
    if(this.disposed||!this.collecting)throw new Error('No GL observation in progress');
    this.collecting=false;
    if(!Number.isSafeInteger(this.sections+1)||!Number.isSafeInteger(this.totalUploadBytes+this.uploadBytes)||
      !Number.isSafeInteger(this.totalDrawCalls+this.drawCalls))this.valid=false;
    if(this.valid){++this.sections;this.totalUploadBytes+=this.uploadBytes;this.totalDrawCalls+=this.drawCalls;}
  }
  resetTotals():void {
    if(this.disposed||this.collecting||!this.valid)throw new Error('Cannot reset GL observation');
    this.sections=0;this.totalUploadBytes=0;this.totalDrawCalls=0;
  }
  report(){return {valid:this.valid,sections:this.sections,totalUploadBytes:this.totalUploadBytes,totalDrawCalls:this.totalDrawCalls,
    methods:names,scope:'submitted core WebGL draws and host bufferData/bufferSubData bytes during renderer.draw',
    excludes:['uniforms','textures','framebuffer writes','driver-internal traffic','extension multi-draw'],
    overhead:'same wrappers per GL call for WebGL and raylib; included in measured CPU draw time'};}
  private restore():void {
    for(let i=0;i<names.length;++i){const descriptor=this.descriptors[i];
      if(descriptor)Object.defineProperty(this.gl,names[i],descriptor);else Reflect.deleteProperty(this.gl,names[i]);}
  }
  dispose():void {if(!this.disposed){this.collecting=false;this.restore();owners.delete(this.gl);this.disposed=true;}}
}
