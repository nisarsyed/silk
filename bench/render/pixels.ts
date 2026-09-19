import {DrawScene,camera} from './geometry.js';
import {Overlay,markerVertices} from './overlay.js';

// The AA allowance is measured from actual primitive boundaries, including
// subpixel gaps hidden by Canvas coverage. The radius remains exactly one
// drawing-buffer pixel. Overlay command/count tests separately cover thin
// lines, whose whole width necessarily lies inside this edge band.
function segment(mask:Uint8Array,width:number,height:number,ax:number,ay:number,bx:number,by:number):void {
  const dx=bx-ax,dy=by-ay,lengthSq=dx*dx+dy*dy;
  for(let y=Math.max(0,Math.floor(Math.min(ay,by)-1));y<=Math.min(height-1,Math.ceil(Math.max(ay,by)+1));++y)
    for(let x=Math.max(0,Math.floor(Math.min(ax,bx)-1));x<=Math.min(width-1,Math.ceil(Math.max(ax,bx)+1));++x){
      const t=lengthSq===0?0:Math.max(0,Math.min(1,((x+.5-ax)*dx+(y+.5-ay)*dy)/lengthSq));
      const ex=x+.5-ax-t*dx,ey=y+.5-ay-t*dy;
      if(ex*ex+ey*ey<=1)mask[y*width+x]=1;
    }
}
export function edgeMask(scene:DrawScene,width:number,height:number,overlay?:Overlay):Uint8Array {
  const mask=new Uint8Array(width*height),view=camera(scene.rect,width,height),p=scene.poses;
  for(const batch of scene.batches){
    const v=scene.meshes[batch.mesh].vertices;
    for(let i=batch.first;i<batch.first+batch.count;++i){
      const at=i*4;
      for(let a=0;a<v.length;a+=2){
        const b=(a+2)%v.length;
        segment(mask,width,height,
          view.x+view.scale*(p[at]+p[at+2]*v[a]-p[at+3]*v[a+1]),view.y-view.scale*(p[at+1]+p[at+3]*v[a]+p[at+2]*v[a+1]),
          view.x+view.scale*(p[at]+p[at+2]*v[b]-p[at+3]*v[b+1]),view.y-view.scale*(p[at+1]+p[at+3]*v[b]+p[at+2]*v[b+1]));
      }
    }
  }
  if(overlay){
    const l=overlay.lines,m=overlay.markers;
    for(let i=0;i<overlay.lineCount;++i){
      const at=i*5,ax=l[at],ay=l[at+1],bx=l[at+2],by=l[at+3];
      const dx=bx-ax,dy=by-ay,len=Math.hypot(dx,dy),nx=len?-dy*.5/len:0,ny=len?dx*.5/len:0;
      segment(mask,width,height,ax-nx,ay-ny,bx-nx,by-ny);segment(mask,width,height,bx-nx,by-ny,bx+nx,by+ny);
      segment(mask,width,height,bx+nx,by+ny,ax+nx,ay+ny);segment(mask,width,height,ax+nx,ay+ny,ax-nx,ay-ny);
    }
    for(let i=0;i<overlay.markerCount;++i)for(let v=0;v<markerVertices.length;v+=2){
      const next=(v+2)%markerVertices.length;
      segment(mask,width,height,m[i*3]+3*markerVertices[v],m[i*3+1]+3*markerVertices[v+1],
        m[i*3]+3*markerVertices[next],m[i*3+1]+3*markerVertices[next+1]);
    }
  }
  return mask;
}
export function compare(reference:ArrayLike<number>,candidate:ArrayLike<number>,width:number,height:number,edges:Uint8Array){
  let different=0,interiorMismatch=0;const examples:{x:number;y:number;reference:number[];candidate:number[]}[]=[];
  for(let y=0;y<height;++y)for(let x=0;x<width;++x){
    const at=4*(y*width+x);
    if(reference[at]===candidate[at]&&reference[at+1]===candidate[at+1]&&reference[at+2]===candidate[at+2])continue;
    ++different;
    if(!edges[y*width+x]){
      ++interiorMismatch;
      if(examples.length<8)examples.push({x,y,reference:Array.from({length:4},(_,i)=>reference[at+i]),candidate:Array.from({length:4},(_,i)=>candidate[at+i])});
    }
  }
  return {different,interiorMismatch,examples};
}
