import {DrawScene,camera} from './geometry.js';

// The contract permits edge AA differences within one buffer pixel. Derive
// that band analytically from polygon boundaries: neighboring reference pixels
// cannot expose edges that Canvas coverage has merged across a subpixel gap.
export function edgeMask(scene:DrawScene,width:number,height:number) {
  const mask=new Uint8Array(width*height), view=camera(scene.rect,width,height), p=scene.poses;
  for (const batch of scene.batches) {
    const v=scene.meshes[batch.mesh].vertices;
    for (let i=batch.first; i<batch.first+batch.count; ++i) {
      const at=i*4;
      for (let a=0; a<v.length; a+=2) {
        const b=(a+2)%v.length;
        const ax=view.x+view.scale*(p[at]+p[at+2]*v[a]-p[at+3]*v[a+1]);
        const ay=view.y-view.scale*(p[at+1]+p[at+3]*v[a]+p[at+2]*v[a+1]);
        const bx=view.x+view.scale*(p[at]+p[at+2]*v[b]-p[at+3]*v[b+1]);
        const by=view.y-view.scale*(p[at+1]+p[at+3]*v[b]+p[at+2]*v[b+1]);
        const dx=bx-ax,dy=by-ay,lengthSq=dx*dx+dy*dy;
        for (let y=Math.max(0,Math.floor(Math.min(ay,by)-1));y<=Math.min(height-1,Math.ceil(Math.max(ay,by)+1));++y)
          for (let x=Math.max(0,Math.floor(Math.min(ax,bx)-1));x<=Math.min(width-1,Math.ceil(Math.max(ax,bx)+1));++x) {
            const t=lengthSq===0?0:Math.max(0,Math.min(1,((x+0.5-ax)*dx+(y+0.5-ay)*dy)/lengthSq));
            const ex=x+0.5-ax-t*dx,ey=y+0.5-ay-t*dy;
            if(ex*ex+ey*ey<=1) mask[y*width+x]=1;
          }
      }
    }
  }
  return mask;
}
export function compare(reference:ArrayLike<number>,candidate:ArrayLike<number>,width:number,height:number,edges:Uint8Array) {
  let different=0,interiorMismatch=0; const examples:{x:number;y:number;reference:number[];candidate:number[]}[]=[];
  for (let y=0;y<height;++y) for (let x=0;x<width;++x) {
    const at=4*(y*width+x);
    if (reference[at]===candidate[at] && reference[at+1]===candidate[at+1] && reference[at+2]===candidate[at+2]) continue;
    ++different;
    if (!edges[y*width+x]) {
      ++interiorMismatch;
      if(examples.length<8) examples.push({x,y,reference:Array.from({length:4},(_,i)=>reference[at+i]),candidate:Array.from({length:4},(_,i)=>candidate[at+i])});
    }
  }
  return {different,interiorMismatch,examples};
}
