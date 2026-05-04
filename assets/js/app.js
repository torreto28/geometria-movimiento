const canvas = document.getElementById('billar');
const ctx    = canvas.getContext('2d');
const wrap   = document.getElementById('tw');
let W,H,PAD;

function resize(){
  const r=wrap.getBoundingClientRect();
  canvas.width=r.width; canvas.height=r.height;
  W=canvas.width; H=canvas.height;
  PAD=Math.round(Math.min(W,H)*0.07);
}
resize();
window.addEventListener('resize',resize);

const dot  =(a,b)=>a.x*b.x+a.y*b.y;
const mag  =v=>Math.sqrt(v.x*v.x+v.y*v.y);
const norm =v=>{const m=mag(v);return m<1e-9?{x:0,y:0}:{x:v.x/m,y:v.y/m};};
const dist2=(a,b)=>Math.sqrt((b.x-a.x)**2+(b.y-a.y)**2);

function refl(v,n){const d=dot(v,n);return{x:v.x-2*d*n.x,y:v.y-2*d*n.y};}

function angN(v,n){const c=Math.abs(dot(norm(v),n));return Math.acos(Math.max(-1,Math.min(1,c)))*180/Math.PI;}

function symPt(P,w){const p=projW(P,w);return{x:2*p.x-P.x,y:2*p.y-P.y};}
function projW(P,w){return w.h?{x:P.x,y:w.p}:{x:w.p,y:P.y};}
function dW(P,w){return w.h?Math.abs(P.y-w.p):Math.abs(P.x-w.p);} 

function segX(p1,p2,p3,p4){
  const d1={x:p2.x-p1.x,y:p2.y-p1.y},d2={x:p4.x-p3.x,y:p4.y-p3.y};
  const cr=d1.x*d2.y-d1.y*d2.x;
  if(Math.abs(cr)<1e-6)return{t:'NINGUNO'};
  const t=((p3.x-p1.x)*d2.y-(p3.y-p1.y)*d2.x)/cr;
  const u=((p3.x-p1.x)*d1.y-(p3.y-p1.y)*d1.x)/cr;
  return(t>=-0.01&&t<=1.01&&u>=-0.01&&u<=1.01)?{t:'PUNTO',pt:{x:p1.x+t*d1.x,y:p1.y+t*d1.y}}:{t:'NINGUNO'};
}

const FRIC=0.988,STOP=0.1,MAXV=18,REST=0.93,BR=13;
const PAL=['#f5ede0','#ff5c5c','#5aabff','#3ddc84','#ffe066','#d084ff','#ff9f40','#34d399'];
let balls=[],walls=[],ovs=[],ilog=[];
let nB=0,nBB=0,aSum=0,aCnt=0;
let shTr=true,shN=true,shS=true,shL=true;
let drag=false,dSt=null,cue=null,hintOn=true;

function bwalls(){
  walls=[
    {id:'top',   h:true, p:PAD,   nx:0, ny:1 },
    {id:'bottom',h:true, p:H-PAD, nx:0, ny:-1},
    {id:'left',  h:false,p:PAD,   nx:1, ny:0 },
    {id:'right', h:false,p:W-PAD, nx:-1,ny:0 },
  ];
}

function mkBall(x,y,vx,vy,idx,isCue=false){
  return{x,y,vel:{x:vx,y:vy},r:BR,color:PAL[idx%PAL.length],
         id:idx,isCue,trail:[],pp:{x,y},ia:999};
}

function initB(){
  const cx=PAD+(W-PAD*2)*0.28, cy=H/2;
  balls=[
    mkBall(cx,cy,0,0,0,true),
    mkBall(PAD+(W-PAD*2)*0.62,PAD+(H-PAD*2)*0.38,0,0,1),
    mkBall(PAD+(W-PAD*2)*0.68,PAD+(H-PAD*2)*0.55,0,0,2),
    mkBall(PAD+(W-PAD*2)*0.60,PAD+(H-PAD*2)*0.68,0,0,3),
  ];
  cue=balls[0];
}

function step(){
  bwalls();
  for(const b of balls){
    if(mag(b.vel)<STOP){b.vel={x:0,y:0};continue;}
    b.pp={x:b.x,y:b.y};
    b.x+=b.vel.x; b.y+=b.vel.y;
    for(const w of walls){
      const d=dW(b,w);
      if(d<b.r){
        const iA=angN(b.vel,w);
        const wn={x:w.nx,y:w.ny};
        b.vel=refl(b.vel,wn);
        b.vel.x*=REST; b.vel.y*=REST;
        const ov=b.r-d+0.5; b.x+=w.nx*ov; b.y+=w.ny*ov;
        const rA=angN(b.vel,wn);
        const iP=projW(b,w), Ps=symPt(b.pp,w);
        const lal=calcLAL(b,iP,w);
        ovs.push({b,w,iP,Ps,iA,rA,bp:{x:b.x,y:b.y},pp:{...b.pp},lal,age:0});
        if(ovs.length>10)ovs.shift();
        onHit(b,w,iP,iA,rA,d,b.pp,Ps,lal);
        nB++; aSum+=iA; aCnt++;
        b.ia=0;
        addLog(`🎱 Bola ${b.id+1} | α=${iA.toFixed(1)}° β=${rA.toFixed(1)}° [${w.id}]`);
        hideHint();
      }
    }
    for(let i=0;i<balls.length;i++){
      const o=balls[i];if(o===b)continue;
      const d=dist2(b,o),md=b.r+o.r;
      if(d<md&&d>0.01){
        const n=norm({x:o.x-b.x,y:o.y-b.y});
        const v1n=dot(b.vel,n),v2n=dot(o.vel,n);
        if(v1n-v2n>0){
          b.vel.x+=(v2n-v1n)*n.x;b.vel.y+=(v2n-v1n)*n.y;
          o.vel.x+=(v1n-v2n)*n.x;o.vel.y+=(v1n-v2n)*n.y;
        }
        const ov2=(md-d)/2;
        b.x-=n.x*ov2;b.y-=n.y*ov2;o.x+=n.x*ov2;o.y+=n.y*ov2;
        if(i>balls.indexOf(b)){nBB++;addLog(`💥 Bola ${b.id+1} ↔ Bola ${o.id+1}`,true);} 
      }
    }
    const sp=mag(b.vel);
    if(sp>MAXV){b.vel.x=(b.vel.x/sp)*MAXV;b.vel.y=(b.vel.y/sp)*MAXV}
    b.vel.x*=FRIC;b.vel.y*=FRIC;
    if(shTr){b.trail.push({x:b.x,y:b.y});if(b.trail.length>150)b.trail.shift()}
    b.ia++;
  }
  ovs.forEach(o=>o.age++);
  ovs=ovs.filter(o=>o.age<320);
  updStats();
}

function calcLAL(b,iP,w){
  const A=b.pp,P=iP,C=projW(A,w);
  const od=norm(b.vel);
  const B={x:P.x+od.x*70,y:P.y+od.y*70};
  const D=projW(B,w);
  const AP=dist2(A,P),PC=dist2(P,C),PB=dist2(P,B),PD=dist2(P,D);
  const valid=Math.abs(PC-PD)<3.5;
  const ix=segX(A,P,B,P);
  return{A,B,C,D,P,AP,PC,PB,PD,valid,it:ix.t};
}

const CONCEPTS=[
  {title:'① Ley de Reflexión',col:'#ffe066',body:`El ángulo de <strong style="color:#ffe066">entrada (α)</strong> siempre es igual al de <strong style="color:#3ddc84">salida (β)</strong>, medidos desde la <em>normal</em> de la banda.<br><br>Esta fórmula opera en <strong>Unity</strong> y <strong>Godot</strong>. <em>Berrío et al. (2017)</em>`,form:`v' = v − 2(v · n̂)n̂`,val:(ov)=>`α=${ov.iA.toFixed(1)}°  β=${ov.rA.toFixed(1)}°   Δ=${Math.abs(ov.iA-ov.rA).toFixed(2)}°`},
  {title:'② Isometría — Punto P*',col:'#d084ff',body:`La reflexión es una <strong style="color:#d084ff">transformación isométrica</strong>: el punto P* está al otro lado de la banda, exactamente a la misma distancia que la bola.<br><br>La <strong>mediatriz</strong> de PP* coincide con la banda. <em>Berrío et al. (2017)</em>`,form:`P* = 2·proy(P, banda) − P`,val:(ov)=>`d(P,banda) = ${dW(ov.pp,ov.w).toFixed(1)}px = d(P*,banda)`},
  {title:'③ Congruencia L-A-L',col:'#ff6b6b',body:`Los triángulos de <strong style="color:#ff6b6b">incidencia</strong> y <strong style="color:#3ddc84">reflexión</strong> son <em>congruentes</em> por el criterio Lado-Ángulo-Lado.<br><br><strong>Sabharwal et al. (2013)</strong> clasifican la intersección entre los triángulos: se tocan en exactamente un <strong>PUNTO</strong>.`,form:`PC = PD,   ∠P,   AP ≅ PB`,val:(ov)=>`Intersección (Sabharwal): ${ov.lal.it}   LAL=${ov.lal.valid?'✓':'~'}`},
];
let cIdx=0;

function onHit(b,w,iP,iA,rA,d,pp,Ps,lal){
  const ov={iA,rA,pp,w,lal,Ps};
  const c=CONCEPTS[cIdx%CONCEPTS.length]; cIdx++;

  document.getElementById('vI').textContent=iA.toFixed(1)+'°';
  document.getElementById('vR').textContent=rA.toFixed(1)+'°';
  const df=Math.abs(iA-rA);
  const eq=document.getElementById('eqs');
  if(df<1.8){eq.textContent=`✓  α ≈ β  (Δ=${df.toFixed(2)}°)  — Ley de reflexión verificada`;eq.className='yes'}
  else{eq.textContent=`Δ = ${df.toFixed(1)}°`;eq.className=''}

  document.getElementById('cb').innerHTML=`
    <div class="ct" style="color:${c.col}">${c.title}</div>
    <div class="cy">${c.body}</div>
    <div class="cf">${c.form}</div>
    <div class="cv"><span class="cok">✓</span><span class="cvv">${c.val(ov)}</span></div>`;

  const tp=document.getElementById('tp');
  tp.style.borderColor=c.col;
  document.getElementById('tpT').style.color=c.col;
  document.getElementById('tpT').textContent=c.title;
  document.getElementById('tpB').innerHTML=`<span class="tf">${c.form}</span>`;
  tp.classList.add('show');
  clearTimeout(tp._t);
  tp._t=setTimeout(()=>tp.classList.remove('show'),2800);

  ilog.push({t:Date.now(),ball:b.id+1,banda:w.id,
             alpha:iA.toFixed(2),beta:rA.toFixed(2),
             dP:dW(pp,w).toFixed(2),dPs:dW(Ps,w).toFixed(2),
             lal:lal.valid?'SI':'NO',inter:lal.it});
}

function hideHint(){
  if(!hintOn)return; hintOn=false;
  const h=document.getElementById('hint');
  h.classList.add('hidden');
  setTimeout(()=>h.style.display='none',600);
}

function drawTable(){
  ctx.fillStyle='#090909';ctx.fillRect(0,0,W,H);
  ctx.fillStyle='#5c3310';ctx.fillRect(0,0,W,H);
  const ri=PAD*0.28;
  ctx.fillStyle='#7a4518';ctx.fillRect(ri,ri,W-ri*2,H-ri*2);
  const g=ctx.createLinearGradient(PAD,PAD,W-PAD,H-PAD);
  g.addColorStop(0,'#0a4829');g.addColorStop(0.5,'#0d5c35');g.addColorStop(1,'#0a4829');
  ctx.fillStyle=g;ctx.fillRect(PAD,PAD,W-PAD*2,H-PAD*2);
  ctx.strokeStyle='rgba(255,255,255,0.05)';ctx.lineWidth=1;ctx.setLineDash([5,10]);
  ctx.beginPath();ctx.moveTo(W/2,PAD);ctx.lineTo(W/2,H-PAD);ctx.stroke();ctx.setLineDash([]);
  ctx.beginPath();ctx.arc(W/2,H/2,4,0,Math.PI*2);ctx.fillStyle='rgba(255,255,255,0.1)';ctx.fill();
  const pk=[{x:PAD,y:PAD},{x:W/2,y:PAD-3},{x:W-PAD,y:PAD},{x:PAD,y:H-PAD},{x:W/2,y:H-PAD+3},{x:W-PAD,y:H-PAD}];
  for(const p of pk){ctx.beginPath();ctx.arc(p.x,p.y,PAD*0.46,0,Math.PI*2);ctx.fillStyle='#000';ctx.fill();ctx.strokeStyle='rgba(240,192,64,0.3)';ctx.lineWidth=1.5;ctx.stroke();}
  ctx.strokeStyle='rgba(240,192,64,0.15)';ctx.lineWidth=1;ctx.strokeRect(PAD,PAD,W-PAD*2,H-PAD*2);
}

function drawBall(b){
  if(shTr&&b.trail.length>1){for(let i=1;i<b.trail.length;i++){ctx.globalAlpha=(i/b.trail.length)*0.5;ctx.beginPath();ctx.moveTo(b.trail[i-1].x,b.trail[i-1].y);ctx.lineTo(b.trail[i].x,b.trail[i].y);ctx.strokeStyle=b.color;ctx.lineWidth=2;ctx.stroke();}ctx.globalAlpha=1}
  ctx.beginPath();ctx.arc(b.x+3,b.y+3,b.r,0,Math.PI*2);ctx.fillStyle='rgba(0,0,0,0.45)';ctx.fill();
  const gr=ctx.createRadialGradient(b.x-b.r*.32,b.y-b.r*.32,1,b.x,b.y,b.r);gr.addColorStop(0,'rgba(255,255,255,0.9)');gr.addColorStop(0.28,b.color);gr.addColorStop(1,'rgba(0,0,0,0.6)');ctx.beginPath();ctx.arc(b.x,b.y,b.r,0,Math.PI*2);ctx.fillStyle=gr;ctx.fill();
  if(b.ia<22){const a=1-b.ia/22;ctx.beginPath();ctx.arc(b.x,b.y,b.r+9+b.ia*0.9,0,Math.PI*2);ctx.strokeStyle=`rgba(255,255,255,${a*0.55})`;ctx.lineWidth=2.5;ctx.stroke();}
  if(!b.isCue){ctx.fillStyle='rgba(0,0,0,0.65)';ctx.font='bold 9px Inter,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(b.id+1,b.x,b.y);} 
}

function drawNorm(ov){if(!shN)return;const {w,iP:P,iA,rA,b,pp}=ov;const fade=Math.max(0,1-ov.age/300);if(fade<0.04)return;ctx.save();ctx.globalAlpha=fade;const n={x:w.nx,y:w.ny},NL=85;ctx.strokeStyle='rgba(255,255,255,0.72)';ctx.lineWidth=2;ctx.setLineDash([7,5]);ctx.beginPath();ctx.moveTo(P.x-n.x*NL,P.y-n.y*NL);ctx.lineTo(P.x+n.x*NL,P.y+n.y*NL);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='rgba(255,255,255,0.55)';ctx.font='bold 12px Inter,sans-serif';ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText('NORMAL',P.x+n.x*(NL+16),P.y+n.y*(NL+16));const iDir=norm({x:P.x-pp.x,y:P.y-pp.y}),IL=95;ctx.strokeStyle='rgba(255,224,102,0.9)';ctx.fillStyle='rgba(255,224,102,0.9)';ctx.lineWidth=3.5;bigArrow(ctx,P.x,P.y,P.x-iDir.x*IL,P.y-iDir.y*IL);const oDir=norm(b.vel);ctx.strokeStyle='rgba(61,220,132,0.9)';ctx.fillStyle='rgba(61,220,132,0.9)';ctx.lineWidth=3.5;bigArrow(ctx,P.x,P.y,P.x+oDir.x*IL,P.y+oDir.y*IL);const aN=Math.atan2(n.y,n.x),aI=Math.atan2(-iDir.y,-iDir.x),aO=Math.atan2(oDir.y,oDir.x);const arcR=52;ctx.strokeStyle='rgba(255,224,102,0.85)';ctx.lineWidth=3.5;ctx.beginPath();ctx.arc(P.x,P.y,arcR,Math.min(aI,aN),Math.max(aI,aN));ctx.stroke();ctx.strokeStyle='rgba(61,220,132,0.85)';ctx.beginPath();ctx.arc(P.x,P.y,arcR,Math.min(aO,aN),Math.max(aO,aN));ctx.stroke();ctx.textAlign='center';ctx.textBaseline='middle';const lR=arcR+32;const mA=(aI+aN)/2,mB=(aO+aN)/2;ctx.font='bold 20px Inter,sans-serif';ctx.fillStyle='#ffe066';ctx.fillText(`α=${iA.toFixed(0)}°`,P.x+Math.cos(mA)*lR,P.y+Math.sin(mA)*lR);ctx.fillStyle='#3ddc84';ctx.fillText(`β=${rA.toFixed(0)}°`,P.x+Math.cos(mB)*lR,P.y+Math.sin(mB)*lR);ctx.beginPath();ctx.arc(P.x,P.y,7,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle='rgba(255,224,102,0.9)';ctx.lineWidth=2;ctx.stroke();ctx.restore();}

function drawSym(ov){if(!shS)return;const {w,b,Ps,iP}=ov;const fade=Math.max(0,1-ov.age/300);if(fade<0.04)return;ctx.save();ctx.globalAlpha=fade;ctx.strokeStyle='rgba(208,132,255,0.75)';ctx.lineWidth=2;ctx.setLineDash([4,5]);ctx.beginPath();ctx.moveTo(b.x,b.y);ctx.lineTo(Ps.x,Ps.y);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.arc(Ps.x,Ps.y,8,0,Math.PI*2);ctx.fillStyle='rgba(208,132,255,0.9)';ctx.fill();ctx.strokeStyle='#d084ff';ctx.lineWidth=2;ctx.stroke();ctx.fillStyle='#d084ff';ctx.font='bold 17px Inter,sans-serif';ctx.textAlign='left';ctx.textBaseline='middle';ctx.fillText('P*',Ps.x+12,Ps.y-7);ctx.font='11px Inter,sans-serif';ctx.fillStyle='rgba(208,132,255,0.68)';ctx.fillText('punto simétrico',Ps.x+12,Ps.y+9);const mid={x:(b.x+Ps.x)/2,y:(b.y+Ps.y)/2};const tx=w.h?1:0,ty=w.h?0:1,mL=40;ctx.strokeStyle='rgba(208,132,255,0.65)';ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(mid.x-tx*mL,mid.y-ty*mL);ctx.lineTo(mid.x+tx*mL,mid.y+ty*mL);ctx.stroke();const sq=8;ctx.strokeStyle='rgba(208,132,255,0.8)';ctx.lineWidth=1.5;ctx.strokeRect(mid.x-sq/2,mid.y-sq/2,sq,sq);const dP=dW(b,w).toFixed(0),dPs=dW(Ps,w).toFixed(0);ctx.font='bold 13px Inter,sans-serif';ctx.fillStyle='rgba(208,132,255,0.88)';ctx.textAlign='center';ctx.fillText(`d=${dP}px`,(b.x+mid.x)/2,(b.y+mid.y)/2-12);ctx.fillText(`d=${dPs}px`,(Ps.x+mid.x)/2,(Ps.y+mid.y)/2+14);ctx.restore();}

function drawLAL(ov){if(!shL)return;const {lal}=ov;if(!lal)return;const {A,B,C,D,P,AP,PC,PB,PD,valid,it}=lal;const fade=Math.max(0,1-ov.age/300);if(fade<0.04)return;ctx.save();ctx.globalAlpha=fade;ctx.beginPath();ctx.moveTo(A.x,A.y);ctx.lineTo(P.x,P.y);ctx.lineTo(C.x,C.y);ctx.closePath();ctx.fillStyle='rgba(255,77,77,0.2)';ctx.fill();ctx.strokeStyle='rgba(255,110,110,0.75)';ctx.lineWidth=2;ctx.setLineDash([5,4]);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.moveTo(B.x,B.y);ctx.lineTo(P.x,P.y);ctx.lineTo(D.x,D.y);ctx.closePath();ctx.fillStyle='rgba(61,220,132,0.2)';ctx.fill();ctx.strokeStyle='rgba(80,220,140,0.75)';ctx.lineWidth=2;ctx.setLineDash([5,4]);ctx.stroke();ctx.setLineDash([]);const vtx=(x,y,l,col,ox=0,oy=-16)=>{ctx.font='bold 14px Inter,sans-serif';ctx.fillStyle=col;ctx.textAlign='center';ctx.textBaseline='middle';ctx.fillText(l,x+ox,y+oy)};vtx(A.x,A.y,'A','rgba(255,150,150,0.9)');vtx(C.x,C.y,'C','rgba(255,150,150,0.9)',0,16);vtx(B.x,B.y,'B','rgba(100,230,160,0.9)');vtx(D.x,D.y,'D','rgba(100,230,160,0.9)',0,16);ctx.font='12px Inter,sans-serif';ctx.textAlign='center';ctx.fillStyle='rgba(255,190,190,0.85)';midLbl(ctx,A,P,`|AP|=${AP.toFixed(0)}px`,-12);ctx.fillStyle='rgba(150,230,180,0.85)';midLbl(ctx,B,P,`|PB|=${PB.toFixed(0)}px`,14);ctx.fillStyle='rgba(255,255,255,0.6)';ctx.font='11px Inter,sans-serif';midLbl(ctx,P,C,`PC=${PC.toFixed(0)}`,-10);midLbl(ctx,P,D,`PD=${PD.toFixed(0)}`,10);ctx.beginPath();ctx.arc(P.x,P.y,8,0,Math.PI*2);ctx.fillStyle='#fff';ctx.fill();ctx.strokeStyle=valid?'#3ddc84':'#ff4d4d';ctx.lineWidth=3;ctx.stroke();ctx.font='bold 12px Inter,sans-serif';ctx.fillStyle='#fff';ctx.textAlign='left';ctx.textBaseline='middle';ctx.fillText('P',P.x+10,P.y-10);ctx.font='bold 16px Inter,sans-serif';ctx.fillStyle=valid?'#3ddc84':'#ff6666';ctx.fillText(valid?'△ L-A-L ≅ ✓':'△ L-A-L',P.x+18,P.y+4);ctx.font='11px Inter,sans-serif';ctx.fillStyle='rgba(255,255,255,0.5)';ctx.fillText(`Sabharwal: ${it}`,P.x+18,P.y+20);ctx.restore();}

function midLbl(ctx,a,b,txt,dy=0){ctx.fillText(txt,(a.x+b.x)/2,(a.y+b.y)/2+dy)}

function bigArrow(ctx,x1,y1,x2,y2){const dx=x2-x1,dy=y2-y1,ang=Math.atan2(dy,dx);ctx.beginPath();ctx.moveTo(x1,y1);ctx.lineTo(x2,y2);ctx.stroke();const hs=14;ctx.beginPath();ctx.moveTo(x2,y2);ctx.lineTo(x2-hs*Math.cos(ang-0.42),y2-hs*Math.sin(ang-0.42));ctx.lineTo(x2-hs*Math.cos(ang+0.42),y2-hs*Math.sin(ang+0.42));ctx.closePath();ctx.fill()}

function drawCue(){if(!drag||!dSt||!cue)return;const dx=cue.x-dSt.x,dy=cue.y-dSt.y,len=Math.sqrt(dx*dx+dy*dy);if(len<3)return;const d=norm({x:dx,y:dy}),mLen=Math.min(len,210);ctx.save();ctx.strokeStyle='rgba(245,237,224,0.55)';ctx.lineWidth=2;ctx.setLineDash([8,5]);ctx.beginPath();ctx.moveTo(cue.x,cue.y);ctx.lineTo(cue.x+d.x*mLen,cue.y+d.y*mLen);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='rgba(240,192,64,0.85)';ctx.strokeStyle='rgba(240,192,64,0.85)';ctx.lineWidth=2.5;bigArrow(ctx,cue.x+d.x*(mLen-22),cue.y+d.y*(mLen-22),cue.x+d.x*mLen,cue.y+d.y*mLen);bwalls();let tMin=Infinity,hw=null;for(const w of walls){let t=Infinity;if(w.h){if(Math.abs(d.y)>0.001){t=(d.y>0?(w.p-cue.y-BR):(w.p-cue.y+BR))/d.y;}}else{if(Math.abs(d.x)>0.001){t=(d.x>0?(w.p-cue.x-BR):(w.p-cue.x+BR))/d.x;}}if(t>5&&t<tMin){tMin=t;hw=w;}}if(hw&&tMin<700){const hx=cue.x+d.x*tMin,hy=cue.y+d.y*tMin;ctx.strokeStyle='rgba(240,192,64,0.28)';ctx.lineWidth=1.5;ctx.setLineDash([4,7]);ctx.beginPath();ctx.moveTo(cue.x+d.x*mLen,cue.y+d.y*mLen);ctx.lineTo(hx,hy);ctx.stroke();ctx.setLineDash([]);ctx.beginPath();ctx.arc(hx,hy,6,0,Math.PI*2);ctx.fillStyle='rgba(240,192,64,0.55)';ctx.fill();ctx.strokeStyle='rgba(240,192,64,0.8)';ctx.lineWidth=1.5;const s=5;ctx.beginPath();ctx.moveTo(hx-s,hy-s);ctx.lineTo(hx+s,hy+s);ctx.stroke();ctx.beginPath();ctx.moveTo(hx+s,hy-s);ctx.lineTo(hx-s,hy+s);ctx.stroke();}
const pct=Math.min(100,Math.round(len/210*100));document.getElementById('fm').style.display='block';document.getElementById('fm').textContent=`FUERZA: ${pct}%`;ctx.restore()}

function frame(){ctx.clearRect(0,0,W,H);drawTable();for(const ov of ovs){drawLAL(ov);drawSym(ov);drawNorm(ov);}for(const b of balls)drawBall(b);drawCue();step();requestAnimationFrame(frame)}

function gPos(e){const r=canvas.getBoundingClientRect();const cl=e.touches?e.touches[0]:e;return{x:(cl.clientX-r.left)*(canvas.width/r.width),y:(cl.clientY-r.top)*(canvas.height/r.height)}}
function onDn(e){e.preventDefault();const p=gPos(e);if(!cue)return;if(dist2(p,cue)<cue.r+22&&mag(cue.vel)<0.3){drag=true;dSt=p}}function onMv(e){if(!drag)return;e.preventDefault();dSt=gPos(e)}function onUp(e){if(!drag)return;e.preventDefault();drag=false;document.getElementById('fm').style.display='none';if(!dSt||!cue)return;const dx=cue.x-dSt.x,dy=cue.y-dSt.y,len=Math.sqrt(dx*dx+dy*dy);if(len<6)return;const d=norm({x:dx,y:dy}),f=Math.min(len/210,1)*16;cue.vel={x:d.x*f,y:d.y*f};cue.trail=[];dSt=null}
canvas.addEventListener('mousedown',onDn);canvas.addEventListener('mousemove',onMv);canvas.addEventListener('mouseup',onUp);canvas.addEventListener('touchstart',onDn,{passive:false});canvas.addEventListener('touchmove',onMv,{passive:false});canvas.addEventListener('touchend',onUp,{passive:false});

let ll=['<div class="le">— Simulación lista —</div>'],lt=0;function addLog(m,c=false){const n=Date.now();if(!c&&n-lt<200)return;lt=n;ll.unshift(`<div class="le${c?' c':''}">${m}</div>`);if(ll.length>60)ll.pop();document.getElementById('lb').innerHTML=ll.join('')}function updStats(){document.getElementById('sB').textContent=nB;document.getElementById('sBa').textContent=balls.length;document.getElementById('sC').textContent=nBB;document.getElementById('sA').textContent=aCnt?(aSum/aCnt).toFixed(1)+'°':'—'}

function tog(id,f){document.getElementById(id).classList.toggle('on',f);return!f}
document.getElementById('bN').addEventListener('click',function(){shN=tog('bN',shN)});
document.getElementById('bS').addEventListener('click',function(){shS=tog('bS',shS)});
document.getElementById('bL').addEventListener('click',function(){shL=tog('bL',shL)});
document.getElementById('bT').addEventListener('click',function(){shTr=tog('bT',shTr);if(!shTr)balls.forEach(b=>b.trail=[])});
document.getElementById('bA').addEventListener('click',()=>{if(balls.length>=9)return;const idx=balls.length;balls.push(mkBall(PAD+20+Math.random()*(W-PAD*2-40),PAD+20+Math.random()*(H-PAD*2-40),0,0,idx));updStats()});
document.getElementById('bR').addEventListener('click',()=>{const i=balls.findIndex(b=>!b.isCue);if(i>=0){balls.splice(i,1);updStats()}});
document.getElementById('bRst').addEventListener('click',()=>{initB();ovs=[];ilog=[];nB=0;nBB=0;aSum=0;aCnt=0;cIdx=0;ll=['<div class="le">— Reiniciado —</div>'];document.getElementById('lb').innerHTML=ll[0];document.getElementById('vI').textContent='—';document.getElementById('vR').textContent='—';document.getElementById('eqs').textContent='— Dispara para ver la ley de reflexión —';document.getElementById('eqs').className='';document.getElementById('cb').innerHTML=`
    <div class="ct" style="color:var(--gold)">Ley de Reflexión</div>
    <div class="cy">Cuando una bola choca con una banda, el <strong>ángulo de entrada (α)</strong> siempre es igual al <strong>ángulo de salida (β)</strong>, medidos desde la <em>normal</em>.<br><br>Es la misma fórmula que usa <strong>Unity</strong> y <strong>Godot</strong>. <em>Berrío et al. (2017)</em></div>
    <div class="cf">v' = v − 2(v · n̂)n̂</div>
    <div class="cv"><span class="cok">✓</span><span class="cvv">Esperando primer rebote…</span></div>`;updStats();const h=document.getElementById('hint');h.style.display='';h.classList.remove('hidden');hintOn=true});

document.getElementById('bEx').addEventListener('click',()=>{if(!ilog.length){alert('Dispara primero.');return;}const h=['t','bola','banda','alpha','beta','dP','dPstar','LAL','interseccion'];const r=ilog.map(x=>Object.values(x).join(','));const a=Object.assign(document.createElement('a'),{href:URL.createObjectURL(new Blob([[h.join(','),...r].join('\n')],{type:'text/csv'})),download:`billar_datos_${Date.now()}.csv`});a.click()});

initB();
frame();
