import Fastify from 'fastify';
import { randomUUID, createHash } from 'node:crypto';

type Joint={min:number;max:number;maxVelocity:number}; type Robot={id:string;name:string;dof:number;joints:Joint[];reach:number}; type Pose={x:number;y:number;z:number;rx:number;ry:number;rz:number}; type Waypoint=Pose&{speed:number;blend:number;process?:Record<string,number>};
const app=Fastify({logger:true});const now=()=>new Date().toISOString();
const robots:Robot[]=[{id:'robot-01',name:'六轴焊接机器人 A',dof:6,reach:1450,joints:[-170,170,-100,135,-180,180].map((_,i)=>({min:[-170,-100,-150,-180,-120,-360][i],max:[170,100,150,180,120,360][i],maxVelocity:[180,160,170,360,360,500][i]}))}];
const cells:any[]=[{id:'cell-01',name:'车身侧围焊装单元',robotId:'robot-01',tools:[{id:'gun-01',mass:12,tcp:{x:0,y:0,z:220}}],obstacles:[{id:'fence',type:'box',center:{x:0,y:0,z:700},size:{x:1800,y:1800,z:1600}},{id:'fixture',type:'box',center:{x:500,y:0,z:300},size:{x:420,y:360,z:300}}]}];
const paths:any[]=[];const validations:any[]=[];const plans:any[]=[];const executions:any[]=[];const audit:any[]=[];const log=(action:string,payload:any)=>audit.push({id:randomUUID(),time:now(),action,payload});
function distance(a:Pose,b:Pose){return Math.hypot(a.x-b.x,a.y-b.y,a.z-b.z)}
function insideBox(p:Pose,b:any){return Math.abs(p.x-b.center.x)<=b.size.x/2&&Math.abs(p.y-b.center.y)<=b.size.y/2&&Math.abs(p.z-b.center.z)<=b.size.z/2}
function validatePath(path:any){const robot=robots.find(x=>x.id===path.robotId);const cell=cells.find(x=>x.id===path.cellId);const errors:any[]=[];let length=0,maxSpeed=0;for(let i=1;i<path.waypoints.length;i++){const prev=path.waypoints[i-1],cur=path.waypoints[i];length+=distance(prev,cur);maxSpeed=Math.max(maxSpeed,Number(cur.speed||0));for(const ob of cell?.obstacles||[]){if(insideBox(cur,ob)||insideBox({x:(prev.x+cur.x)/2,y:(prev.y+cur.y)/2,z:(prev.z+cur.z)/2,rx:0,ry:0,rz:0},ob))errors.push({type:'collision',obstacleId:ob.id,segment:i-1,clearance:0});}}
for(const p of path.waypoints){const reach=Math.hypot(p.x,p.y,p.z);if(reach>Number(robot?.reach||0))errors.push({type:'unreachable',waypoint:p});if(Math.abs(p.rx)>180||Math.abs(p.ry)>180||Math.abs(p.rz)>360)errors.push({type:'orientation-limit',waypoint:p});}
if(maxSpeed>220)errors.push({type:'speed-limit',maxSpeed});return{valid:errors.length===0,errors,length:Number(length.toFixed(2)),maxSpeed,estimatedSeconds:Number((length/Math.max(1,maxSpeed||100)).toFixed(2)),singularityRisk:path.waypoints.filter((p:any)=>Math.abs(p.z)<80).length};}
app.get('/health',async()=>({ok:true,service:'industrial-robot-trajectory-validation',time:now()}));
app.get('/api/cells',async()=>cells);app.get('/api/robots',async()=>robots);
app.post('/api/paths',async(req:any,res)=>{const b=req.body as any;if(!b.cellId||!b.robotId||!Array.isArray(b.waypoints)||b.waypoints.length<2)return res.code(400).send({error:'cellId、robotId 和至少两个 waypoints 必填'});const p={id:randomUUID(),version:1,name:b.name||'未命名轨迹',cellId:b.cellId,robotId:b.robotId,toolId:b.toolId||null,waypoints:b.waypoints,process:b.process||{},status:'draft',createdAt:now()};paths.push(p);return p;});
app.post('/api/paths/:id/validate',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const result=validatePath(p);const v={id:randomUUID(),pathId:p.id,createdAt:now(),...result};validations.push(v);p.lastValidationId=v.id;p.status=result.valid?'validated':'blocked';log('path.validate',v);return v;});
app.post('/api/paths/:id/plan',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const v=validations.filter(x=>x.pathId===p.id).at(-1);if(!v?.valid)return res.code(409).send({error:'path must pass collision validation'});const b=req.body as any;const plan={id:randomUUID(),pathId:p.id,controller:b.controller||'RAPID',posture:b.posture||'elbow-up',blendPolicy:b.blendPolicy||'smooth',samplingMs:Number(b.samplingMs||20),segments:p.waypoints.slice(1).map((w:any,i:number)=>({index:i+1,from:p.waypoints[i],to:w,acceleration:Number((Math.min(2.5,(w.speed||100)/80)).toFixed(2))})),status:'planned',createdAt:now()};plans.push(plan);log('trajectory.plan',plan);return plan;});
app.post('/api/paths/:id/optimize',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const candidates=[.75,1,1.2].map(scale=>{const copy={...p,waypoints:p.waypoints.map((w:any)=>({...w,speed:Math.min(220,(w.speed||100)*scale),blend:Math.min(100,(w.blend||10)+scale*5)}))};return{scale,validation:validatePath(copy),waypoints:copy.waypoints}}).filter(x=>x.validation.valid).sort((a,b)=>a.validation.estimatedSeconds-b.validation.estimatedSeconds);return{pathId:p.id,candidates,best:candidates[0]||null};});
app.post('/api/executions',async(req:any,res)=>{const b=req.body as any;const p=paths.find(x=>x.id===b.pathId);if(!p)return res.code(404).send({error:'path not found'});const v=validations.filter(x=>x.pathId===p.id).at(-1);if(!v?.valid)return res.code(409).send({error:'execution blocked by validation'});const e:any={id:randomUUID(),pathId:p.id,mode:b.mode||'dry-run',startedAt:now(),status:'running',progress:0,telemetry:[],safety:{eStop:false,protectiveStop:false}};executions.push(e);for(let i=0;i<p.waypoints.length;i++){e.progress=Math.round((i+1)/p.waypoints.length*100);e.telemetry.push({time:now(),waypoint:i,joints:p.waypoints[i],tcpSpeed:p.waypoints[i].speed});}e.status='completed';e.finishedAt=now();log('execution.complete',e);return e;});
app.post('/api/executions/:id/stop',async(req:any,res)=>{const e=executions.find(x=>x.id===(req.params as any).id);if(!e)return res.code(404).send({error:'execution not found'});e.status='protective-stopped';e.safety.protectiveStop=true;e.stoppedAt=now();return e;});
app.get('/api/paths/:id/compare',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const versions=paths.filter(x=>x.name===p.name).map(x=>({id:x.id,version:x.version,validation:validations.filter(v=>v.pathId===x.id).at(-1)}));return{pathId:p.id,versions};});
app.get('/api/audit',async()=>audit.slice(-100));

// Advanced operations: calibration, workpiece frames, dynamic obstacles and safety release gates.
const calibrations:any[]=[{id:'cal-01',robotId:'robot-01',toolId:'gun-01',tcpErrorMm:0.42,angleErrorDeg:.18,status:'valid',validUntil:new Date(Date.now()+30*86400000).toISOString()}];
const dynamicObstacles:any[]=[]; const frames:any[]=[{id:'frame-body',name:'车身基准坐标系',origin:{x:0,y:0,z:0},rotation:{rx:0,ry:0,rz:0},status:'verified'}]; const safetyReleases:any[]=[];
function safetyGate(path:any){const v=validations.filter(x=>x.pathId===path.id).at(-1);const cell=cells.find(x=>x.id===path.cellId);const calibration=calibrations.find(x=>x.robotId===path.robotId&&x.toolId===path.toolId);const activeDynamic=dynamicObstacles.filter(x=>x.status==='active'&&x.cellId===path.cellId);const failures:string[]=[];if(calibration && Date.parse(calibration.validUntil)<=Date.now())calibration.status='expired';if(!v?.valid)failures.push('collision-validation');if(!calibration||calibration.status!=='valid')failures.push('tcp-calibration');if(activeDynamic.length)failures.push('dynamic-obstacle-present');if(!cell)failures.push('cell-missing');return{released:failures.length===0,failures,checkedAt:now(),calibrationId:calibration?.id||null};}
app.get('/api/calibrations',async()=>calibrations);
app.post('/api/calibrations',async(req:any,res)=>{const b=req.body as any;if(!b.robotId||!b.toolId||b.tcpErrorMm==null)return res.code(400).send({error:'robotId、toolId、tcpErrorMm 必填'});const c={id:randomUUID(),robotId:b.robotId,toolId:b.toolId,tcpErrorMm:Number(b.tcpErrorMm),angleErrorDeg:Number(b.angleErrorDeg||0),status:Number(b.tcpErrorMm)<=1&&Number(b.angleErrorDeg||0)<=.5?'valid':'invalid',validUntil:new Date(Date.now()+Number(b.validDays||30)*86400000).toISOString(),createdAt:now()};calibrations.push(c);return c;});
app.post('/api/frames',async(req:any,res)=>{const b=req.body as any;if(!b.name)return res.code(400).send({error:'name 必填'});const f={id:randomUUID(),name:b.name,origin:b.origin||{x:0,y:0,z:0},rotation:b.rotation||{rx:0,ry:0,rz:0},status:'pending-verification',createdAt:now()};frames.push(f);return f;});
app.post('/api/dynamic-obstacles',async(req:any,res)=>{const b=req.body as any;if(!b.cellId||!b.center||!b.size)return res.code(400).send({error:'cellId、center、size 必填'});const o={id:randomUUID(),cellId:b.cellId,center:b.center,size:b.size,source:b.source||'safety-scanner',status:'active',reportedAt:now()};dynamicObstacles.push(o);return o;});
app.post('/api/dynamic-obstacles/:id/clear',async(req:any,res)=>{const o=dynamicObstacles.find(x=>x.id===(req.params as any).id);if(!o)return res.code(404).send({error:'dynamic obstacle not found'});o.status='cleared';o.clearedAt=now();return o;});
app.post('/api/paths/:id/release',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const gate=safetyGate(p);const release={id:randomUUID(),pathId:p.id,gate,status:gate.released?'released':'blocked',operator:(req.body as any)?.operator||'cell-supervisor',createdAt:now()};safetyReleases.push(release);if(gate.released)p.status='released';log('path.release',release);return release;});
app.post('/api/executions/:id/telemetry',async(req:any,res)=>{const e=executions.find(x=>x.id===(req.params as any).id);if(!e)return res.code(404).send({error:'execution not found'});const b=req.body as any;if(b.jointErrorDeg!=null&&Number(b.jointErrorDeg)>2){e.status='protective-stopped';e.safety.protectiveStop=true;e.stopReason='joint tracking error';}if(b.tcpForceN!=null&&Number(b.tcpForceN)>180){e.status='protective-stopped';e.safety.protectiveStop=true;e.stopReason='TCP force threshold exceeded';}e.telemetry.push({time:now(),...b});return e;});
app.get('/api/safety/releases',async()=>safetyReleases);app.get('/api/dynamic-obstacles',async()=>dynamicObstacles);

// 坐标系（frame）验证与轨迹转换：名义点/实测点 -> Kabsch 求刚体变换、残差与版本；
// 路径可关联已验证 frame，转换预览只生成派生点，绝不修改原始 waypoints。
type Vec3={x:number;y:number;z:number}; type Mat3=[[number,number,number],[number,number,number],[number,number,number]];
const matI=():Mat3=>[[1,0,0],[0,1,0],[0,0,1]];
const matT=(m:Mat3):Mat3=>[[m[0][0],m[1][0],m[2][0]],[m[0][1],m[1][1],m[2][1]],[m[0][2],m[1][2],m[2][2]]];
const matMul=(a:Mat3,b:Mat3):Mat3=>[0,1,2].map(i=>[0,1,2].map(j=>a[i][0]*b[0][j]+a[i][1]*b[1][j]+a[i][2]*b[2][j])) as Mat3;
const matVec=(m:Mat3,v:number[]):Vec3=>({x:m[0][0]*v[0]+m[0][1]*v[1]+m[0][2]*v[2],y:m[1][0]*v[0]+m[1][1]*v[1]+m[1][2]*v[2],z:m[2][0]*v[0]+m[2][1]*v[1]+m[2][2]*v[2]});
const det3=(m:Mat3)=>m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])-m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])+m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
const cross=(a:number[],b:number[]):number[]=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
// 实对称 3x3 矩阵的特征分解（Jacobi 迭代），特征值降序；特征向量为 V 的列
function jacobiSym3(m:Mat3){const a:number[][]=m.map(r=>r.slice());const V:number[][]=[[1,0,0],[0,1,0],[0,0,1]];
for(let sweep=0;sweep<50;sweep++){for(const[p,q]of[[0,1],[0,2],[1,2]] as const){if(Math.abs(a[p][q])<1e-15)continue;const theta=(a[q][q]-a[p][p])/(2*a[p][q]);const t=theta>=0?1/(theta+Math.sqrt(theta*theta+1)):-1/(-theta+Math.sqrt(theta*theta+1));const c=1/Math.sqrt(1+t*t),s=c*t;
const app=a[p][p],aqq=a[q][q],apq=a[p][q];a[p][p]=app-t*apq;a[q][q]=aqq+t*apq;a[p][q]=0;a[q][p]=0;
for(let k=0;k<3;k++){if(k===p||k===q)continue;const akp=a[k][p],akq=a[k][q];a[k][p]=c*akp-s*akq;a[p][k]=a[k][p];a[k][q]=s*akp+c*akq;a[q][k]=a[k][q];}
for(let k=0;k<3;k++){const vkp=V[k][p],vkq=V[k][q];V[k][p]=c*vkp-s*vkq;V[k][q]=s*vkp+c*vkq;}}}
const d0=[a[0][0],a[1][1],a[2][2]];const idx=[0,1,2].sort((x,y)=>d0[y]-d0[x]);const Vout:Mat3=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<3;i++)for(let k=0;k<3;k++)Vout[k][i]=V[k][idx[i]];return{d:idx.map(i=>d0[i]),V:Vout};}
// H 的 SVD：U 列 = H V列 / σ，d>=0 降序；rank<2 时用向量叉积补齐退化方向
function svdH(H:Mat3){const{d,V}=jacobiSym3(matMul(matT(H),H));const w=d.map(x=>Math.sqrt(Math.max(0,x)));const U:Mat3=[[0,0,0],[0,0,0],[0,0,0]];
for(let k=0;k<3;k++){const vk=[V[0][k],V[1][k],V[2][k]];const h=matVec(H,vk);const inv=w[k]>1e-12?1/w[k]:0;U[0][k]=h.x*inv;U[1][k]=h.y*inv;U[2][k]=h.z*inv;}
if(w[1]<1e-9){if(w[0]<1e-9){U[0][1]=1;U[1][1]=0;U[2][1]=0;U[0][2]=0;U[1][2]=1;U[2][2]=0;}else{const e1=[U[0][0],U[1][0],U[2][0]];const tmp=Math.abs(e1[2])<0.9?[0,0,1]:[1,0,0];const e2=cross(e1,tmp),n=Math.hypot(e2[0],e2[1],e2[2]);const u2=e2.map(x=>x/n);const u3=cross(e1,u2);for(let i=0;i<3;i++){U[i][1]=u2[i];U[i][2]=u3[i];}}}
return{w,U,V};}
// Kabsch：P=名义点（body 系），Q=实测点（焊装单元系），求 Q=R P+t
function kabsch(rawP:number[][],rawQ:number[][]){const n=rawP.length;const c=(a:number[][])=>{const m=a[0].map((_,k)=>a.reduce((s,r)=>s+r[k],0)/n);return{arr:a.map(r=>r.map((x,k)=>x-m[k])),m};};const{arr:P,m:cp}=c(rawP) as {arr:number[][];m:number[]};const{arr:Q,m:cq}=c(rawQ) as {arr:number[][];m:number[]};
let H:Mat3=[[0,0,0],[0,0,0],[0,0,0]];for(let i=0;i<n;i++)for(let a=0;a<3;a++)for(let b=0;b<3;b++)H[a][b]+=P[i][a]*Q[i][b];
const{w,U,V}=svdH(H);let R=matMul(V,matT(U));if(det3(R)<0){V[0][2]=-V[0][2];V[1][2]=-V[1][2];V[2][2]=-V[2][2];R=matMul(V,matT(U));}
const negCp=matVec(R,[-cp[0],-cp[1],-cp[2]]);return{R,t:{x:cq[0]+negCp.x,y:cq[1]+negCp.y,z:cq[2]+negCp.z},w};}
// 旋转矩阵 <-> ZYX 内旋欧拉角（单位：度，航空顺序 yaw/pitch/roll）
function rotToEuler(m:Mat3){const pitch=Math.asin(Math.min(1,Math.max(-1,-m[2][0])));const deg=180/Math.PI;if(Math.abs(m[2][0])>0.999999)return{rx:0,ry:pitch*deg,rz:Math.atan2(-m[0][1],m[1][1])*deg};return{rx:Math.atan2(m[2][1],m[2][2])*deg,ry:pitch*deg,rz:Math.atan2(m[1][0],m[0][0])*deg};}
function eulerToRot(r:Vec3):Mat3{const d=Math.PI/180,{x:rx,y:ry,z:rz}=r,cz=Math.cos(rz*d),sz=Math.sin(rz*d),cy=Math.cos(ry*d),sy=Math.sin(ry*d),cx=Math.cos(rx*d),sx=Math.sin(rx*d);
return[[cz*cy,cz*sy*sx-sz*cx,cz*sy*cx+sz*sx],[sz*cy,sz*sy*sx+cz*cx,sz*sy*cx-cz*sx],[-sy,cy*sx,cy*cx]];}
const r3=(x:number)=>Math.round(x*1000)/1000; const r6=(x:number)=>Math.round(x*1e6)/1e6;
function parsePoint(p:any):number[]|null{if(Array.isArray(p)&&p.length>=3&&p.slice(0,3).every(Number.isFinite))return p.slice(0,3).map(Number);if(p&&Number.isFinite(+p.x)&&Number.isFinite(+p.y)&&Number.isFinite(+p.z))return[+p.x,+p.y,+p.z];return null;}
function transformPoint(R:Mat3,p:number[],t:Vec3){const q=matVec(R,p);return{x:q.x+t.x,y:q.y+t.y,z:q.z+t.z};}
function transformWaypoint(R:Mat3,t:Vec3,w:any){const p=transformPoint(R,[+w.x||0,+w.y||0,+w.z||0],t);const Rw=matMul(R,eulerToRot({x:+w.rx||0,y:+w.ry||0,z:+w.rz||0}));const e=rotToEuler(Rw);return{...w,x:r3(p.x),y:r3(p.y),z:r3(p.z),rx:r3(e.rx),ry:r3(e.ry),rz:r3(e.rz)};}
const roundPoint=(q:Vec3)=>({x:r3(q.x),y:r3(q.y),z:r3(q.z)});

const frameValidations:any[]=[];
// 预置 frame-body 补一条单位变换的初始验证版本，保证既有登记数据可直接用于关联与预览
(function seedFrameValidation(){const f=frames.find((x:any)=>x.id==='frame-body');if(!f)return;const v={id:'fv-frame-body-seed',frameId:f.id,version:1,passed:true,pointCount:0,toleranceMm:0,translation:{x:0,y:0,z:0},rotationEuler:{rx:0,ry:0,rz:0},rotationMatrix:matI(),residualMaxMm:0,residualRmsMm:0,residuals:[],singularValues:[null,null,null],signature:null,note:'seed identity verification for pre-registered frame',createdAt:f.createdAt||now()};frameValidations.push(v);f.verificationIds=[v.id];f.latestVerificationId=v.id;f.status='verified';f.verifiedAt=v.createdAt;})();
const verificationBrief=(v:any)=>({id:v.id,frameId:v.frameId,version:v.version,passed:v.passed,pointCount:v.pointCount,toleranceMm:v.toleranceMm,translation:v.translation,rotationEuler:v.rotationEuler,residualMaxMm:v.residualMaxMm,residualRmsMm:v.residualRmsMm,createdAt:v.createdAt,note:v.note});
const verificationDetail=(v:any)=>({...verificationBrief(v),rotationMatrix:v.rotationMatrix,residuals:v.residuals,singularValues:v.singularValues,signature:v.signature});
const frameBrief=(f:any)=>{const v=frameValidations.find(x=>x.id===f.latestVerificationId);return{id:f.id,name:f.name,origin:f.origin,rotation:f.rotation,status:f.status,createdAt:f.createdAt,verifiedAt:f.verifiedAt||null,latestVerificationId:f.latestVerificationId||null,latestVerification:v?verificationBrief(v):null,verificationCount:(f.verificationIds||[]).length};};

app.post('/api/frames/:id/verifications',async(req:any,res)=>{const f=frames.find(x=>x.id===(req.params as any).id);if(!f)return res.code(404).send({error:'frame not found'});
const b=req.body as any;if(!Array.isArray(b.nominalPoints)||!Array.isArray(b.measuredPoints))return res.code(400).send({error:'nominalPoints 和 measuredPoints 必填（数组）'});if(b.nominalPoints.length!==b.measuredPoints.length)return res.code(400).send({error:'名义点与实测点数量必须一致'});
const nominal=b.nominalPoints.map((p:any)=>parsePoint(p)),measured=b.measuredPoints.map((p:any)=>parsePoint(p));if(nominal.some((p:number[]|null)=>!p)||measured.some((p:number[]|null)=>!p))return res.code(400).send({error:'每个点需包含有限数值 x/y/z（或长度为 3 的数组）'});
if(nominal.length<3)return res.code(400).send({error:'points-insufficient',message:'至少需要 3 个不共线的名义/实测点对',received:nominal.length,required:3});
const toleranceMm=b.toleranceMm==null?2:Number(b.toleranceMm);if(!Number.isFinite(toleranceMm)||toleranceMm<=0)return res.code(400).send({error:'toleranceMm 必须为正数'});
const signature=createHash('sha1').update(JSON.stringify({n:nominal,m:measured,t:r6(toleranceMm)})).digest('hex');
const dup=frameValidations.find(x=>x.frameId===f.id&&x.signature===signature);if(dup){log('frame.verify.replay',{frameId:f.id,verificationId:dup.id});return{...verificationDetail(dup),idempotent:true};}
const k=kabsch(nominal as number[][],measured as number[][]);
if(k.w[1]<1e-9)return res.code(400).send({error:'points-degenerate',message:'点集共线或全部重合，无法唯一确定旋转',singularValues:k.w.map(r6)});
const residuals=(nominal as number[][]).map((p,i)=>{const q=transformPoint(k.R,p,k.t);const m=measured[i]!;return{index:i,errorMm:r3(Math.hypot(q.x-m[0],q.y-m[1],q.z-m[2])),predicted:roundPoint(q),measured:{x:r3(m[0]),y:r3(m[1]),z:r3(m[2])}};});
const maxErr=Math.max(...residuals.map(x=>x.errorMm)),rms=r3(Math.sqrt(residuals.reduce((s,x)=>s+x.errorMm*x.errorMm,0)/residuals.length));
const version=(f.verificationIds||[]).length+1;const passed=maxErr<=toleranceMm;
const v={id:randomUUID(),frameId:f.id,version,passed,pointCount:nominal.length,toleranceMm,translation:{x:r3(k.t.x),y:r3(k.t.y),z:r3(k.t.z)},rotationEuler:rotToEuler(k.R),rotationMatrix:k.R.map(r=>r.map(r6)),residualMaxMm:maxErr,residualRmsMm:rms,residuals,singularValues:k.w.map(r6),signature,createdAt:now()};
frameValidations.push(v);f.verificationIds=[...(f.verificationIds||[]),v.id];f.latestVerificationId=v.id;f.status=passed?'verified':'failed';if(passed)f.verifiedAt=v.createdAt;log('frame.verify',verificationBrief(v));
return verificationDetail(v);});
app.get('/api/frames/:id/verifications',async(req:any,res)=>{const f=frames.find(x=>x.id===(req.params as any).id);if(!f)return res.code(404).send({error:'frame not found'});return{frameId:f.id,verifications:(f.verificationIds||[]).map((id:string)=>verificationBrief(frameValidations.find(x=>x.id===id)!)).filter(Boolean)};});
app.get('/api/frames/:id/verifications/:vid',async(req:any,res)=>{const v=frameValidations.find(x=>x.id===(req.params as any).vid&&x.frameId===(req.params as any).id);if(!v)return res.code(404).send({error:'verification not found'});return verificationDetail(v);});
app.get('/api/frames',async()=>frames.map(frameBrief));
app.get('/api/frames/:id',async(req:any,res)=>{const f=frames.find(x=>x.id===(req.params as any).id);if(!f)return res.code(404).send({error:'frame not found'});return{...frameBrief(f),verifications:(f.verificationIds||[]).map((id:string)=>verificationBrief(frameValidations.find(x=>x.id===id)!)).filter(Boolean)};});

function resolveFrameVerification(frameId:string,verificationId?:string){const f=frames.find(x=>x.id===frameId);if(!f)return{error:{code:404,body:{error:'frame not found'}}};
const v=verificationId?frameValidations.find(x=>x.id===verificationId&&x.frameId===f.id):frameValidations.find(x=>x.id===f.latestVerificationId);
if(verificationId&&!v)return{error:{code:404,body:{error:'verification not found for frame'}}};if(!v)return{error:{code:409,body:{error:'frame has no verification record'}}};if(!v.passed)return{error:{code:409,body:{error:'frame verification failed','verification':verificationBrief(v)}}};return{f,v};}
app.post('/api/paths/:id/frame',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const b=req.body as any;if(!b.frameId)return res.code(400).send({error:'frameId 必填'});
const r=resolveFrameVerification(b.frameId,b.verificationId);if(r.error)return res.code(r.error.code).send(r.error.body);const f=r.f!,v=r.v!;
if(p.frameRef&&p.frameRef.frameId===f.id&&p.frameRef.verificationId===v.id)return{...p.frameRef,frame:frameBrief(f),idempotent:true,history:p.frameRefHistory||[]};
const ref={pathId:p.id,frameId:f.id,verificationId:v.id,translation:v.translation,rotationEuler:v.rotationEuler,attachedAt:now()};p.frameRef=ref;(p.frameRefHistory=p.frameRefHistory||[]).push(ref);log('path.frame.attach',{pathId:p.id,frameId:f.id,verificationId:v.id});
return{...ref,frame:frameBrief(f),idempotent:false};});
app.post('/api/paths/:id/transform-preview',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});const b=req.body as any;
const useFrameId=b?.frameId??p.frameRef?.frameId;if(!useFrameId)return res.code(409).send({error:'path 未关联 frame，请先 POST /api/paths/:id/frame 或在 body 中指定 frameId'});
const useVersion=b?.verificationId??p.frameRef?.verificationId;
const r=resolveFrameVerification(useFrameId,useVersion);if(r.error)return res.code(r.error.code).send(r.error.body);const f=r.f!,v=r.v!;
const useLatest=b?.useLatest===true;const effV=useLatest?frameValidations.find((x:any)=>x.id===f.latestVerificationId)!:v;if(useLatest&&!effV.passed)return res.code(409).send({error:'latest frame verification failed','verification':verificationBrief(effV)});
const R=effV.rotationMatrix as Mat3,t=effV.translation as Vec3;
const transformed=p.waypoints.map((w:any,i:number)=>{const out=transformWaypoint(R,t,w);return{index:i,source:{x:+w.x||0,y:+w.y||0,z:+w.z||0,rx:+w.rx||0,ry:+w.ry||0,rz:+w.rz||0},transformed:{x:out.x,y:out.y,z:out.z,rx:out.rx,ry:out.ry,rz:out.rz},translationDelta:{x:r3(out.x-(+w.x||0)),y:r3(out.y-(+w.y||0)),z:r3(out.z-(+w.z||0))},speed:out.speed,blend:out.blend,process:out.process};});
const newerPassed=!!f.verificationIds?.some((id:string)=>{const x=frameValidations.find(y=>y.id===id);return x&&x.passed&&x.id!==effV.id&&Date.parse(x.createdAt)>=Date.parse(effV.createdAt);});
return{pathId:p.id,frameId:f.id,verificationId:effV.id,verificationVersion:effV.version,pinned:!!p.frameRef&&p.frameRef.verificationId===effV.id,updateAvailable:newerPassed,originalWaypointsUnchanged:true,waypointCount:transformed.length,transformedWaypoints:transformed,generatedAt:now()};});
app.get('/api/paths/:id/frame',async(req:any,res)=>{const p=paths.find(x=>x.id===(req.params as any).id);if(!p)return res.code(404).send({error:'path not found'});if(!p.frameRef)return{pathId:p.id,frameRef:null,historyCount:(p.frameRefHistory||[]).length};
const f=frames.find((x:any)=>x.id===p.frameRef.frameId);const v=frameValidations.find(x=>x.id===p.frameRef.verificationId);
const newerPassed=!!f&&(f.verificationIds||[]).some((id:string)=>{const x=frameValidations.find(y=>y.id===id);return x&&x.passed&&x.id!==p.frameRef.verificationId&&Date.parse(x.createdAt)>=Date.parse(v?.createdAt||0);});
return{...p.frameRef,frameExists:!!f,frame:f?frameBrief(f):null,verificationStatus:v?v.passed?'passed':'failed':'missing',verification:v?verificationBrief(v):null,updateAvailable:newerPassed,history:p.frameRefHistory};});

app.listen({port:Number(process.env.PORT||6303),host:'127.0.0.1'}).catch(err=>{app.log.error(err);process.exit(1)});
