var yf=Object.defineProperty;var Mf=(n,t,e)=>t in n?yf(n,t,{enumerable:!0,configurable:!0,writable:!0,value:e}):n[t]=e;var Wc=(n,t,e)=>Mf(n,typeof t!="symbol"?t+"":t,e);(function(){const t=document.createElement("link").relList;if(t&&t.supports&&t.supports("modulepreload"))return;for(const s of document.querySelectorAll('link[rel="modulepreload"]'))i(s);new MutationObserver(s=>{for(const r of s)if(r.type==="childList")for(const a of r.addedNodes)a.tagName==="LINK"&&a.rel==="modulepreload"&&i(a)}).observe(document,{childList:!0,subtree:!0});function e(s){const r={};return s.integrity&&(r.integrity=s.integrity),s.referrerPolicy&&(r.referrerPolicy=s.referrerPolicy),s.crossOrigin==="use-credentials"?r.credentials="include":s.crossOrigin==="anonymous"?r.credentials="omit":r.credentials="same-origin",r}function i(s){if(s.ep)return;s.ep=!0;const r=e(s);fetch(s.href,r)}})();class Sf extends Error{constructor(t,e=0,i=null){super(t),this.name="ApiError",this.status=e,this.payload=i}}async function yt(n,t){const e=await fetch(n,{method:t===void 0?"GET":"POST",headers:t===void 0?{}:{"Content-Type":"application/json"},body:t===void 0?void 0:JSON.stringify(t),cache:t===void 0?"no-store":"default"}),s=(e.headers.get("content-type")??"").includes("application/json")?await e.json():await e.text();if(!e.ok||(s==null?void 0:s.ok)===!1){const r=(s==null?void 0:s.error)||(s==null?void 0:s.message)||`HTTP ${e.status}`;throw new Sf(r,e.status,s)}return s}function Ef(){return yt("/api/status")}function Tf(){return yt("/api/robot_state")}function nl(n){return yt(`/api/links?arm=${encodeURIComponent(n)}`)}function wf(n){return yt(`/api/gripper?arm=${encodeURIComponent(n)}`)}function Bh(){return yt("/api/vision_target")}function Af(){return yt("/api/vision_status")}function Cf(){return yt("/api/mujoco_camera/status")}function Rf(){return yt("/api/right_wrist_camera/status")}function Pf(){return yt("/api/left_wrist_camera/status")}function zh(n){return yt("/api/mujoco/move_object",n)}function Lf(n){return yt("/api/mujoco/calibration_fixtures",n)}function Nf(n){return yt("/api/mujoco/rebuild_scene",n)}function ec(){return yt("/api/mujoco/grasp_target")}function nc(){return yt("/api/mujoco/visual_grasp_target")}function Df(){return yt("/api/mujoco/camera_calibration/sample",{})}function If(n){return yt("/api/mujoco/camera_calibration/analyze",{samples:n})}function Uf(n,t){return yt("/api/mujoco/camera_calibration/fit",{samples:n,base_extrinsic:t})}function ic(){return yt("/api/mujoco/scene")}function kf(n={}){return yt("/api/mujoco/left_side_drop_rotor",n)}function Ff(){return yt("/api/camera_extrinsic")}function il(n){return yt(`/api/joint_config?arm=${encodeURIComponent(n)}`)}function Of(){return yt("/api/kinematics")}function Bf(){return yt("/api/ompl_config")}function Vh(n){return yt("/api/planning_presets",n)}function zf(n){return yt("/api/planning_presets/delete",n)}function Vf(n){return yt(`/api/benchmark/options?arm=${encodeURIComponent(n)}`)}function Hf(n){return yt(`/api/benchmark/progress?arm=${encodeURIComponent(n)}`)}function Gf(n){return yt("/api/benchmark/generate",n)}function Wf(n){return yt("/api/benchmark/run",n)}function $f(n){return yt("/api/benchmark/export_csv",n)}function jf(){return yt("/api/platform_obstacle")}function Xf(){return yt("/api/manual_collision_boxes")}function qf(){return yt("/api/workspace_bounds")}function Yf(n){return yt("/api/workspace_bounds",n)}function Kf(){return yt("/api/vlm_config")}function Jf(n){return yt("/api/vlm_config",n)}function Zf(){return yt("/api/poses_lib")}function Qf(n){return yt("/api/poses_lib",n)}function tp(n){return yt("/api/poses_lib/delete",n)}function ep(n){return yt("/api/manual_collision_boxes",n)}function np(n){return yt("/api/manual_collision_boxes/apply",n)}function ip(){return yt("/api/manual_collision_boxes/clear",{})}function sp(){return yt("/api/points")}function rp(){return yt("/api/presets")}function ap(n=200){return yt(`/api/logs?n=${n}`)}/**
 * @license
 * Copyright 2010-2025 Three.js Authors
 * SPDX-License-Identifier: MIT
 */const sc="174",ks={ROTATE:0,DOLLY:1,PAN:2},Is={ROTATE:0,PAN:1,DOLLY_PAN:2,DOLLY_ROTATE:3},op=0,$c=1,lp=2,Hh=1,Gh=2,ri=3,ui=0,bn=1,Tn=2,Ri=0,Fs=1,jc=2,Xc=3,qc=4,cp=5,qi=100,up=101,hp=102,dp=103,fp=104,pp=200,mp=201,gp=202,_p=203,sl=204,rl=205,vp=206,xp=207,bp=208,yp=209,Mp=210,Sp=211,Ep=212,Tp=213,wp=214,al=0,ol=1,ll=2,Ws=3,cl=4,ul=5,hl=6,dl=7,Ya=0,Ap=1,Cp=2,Pi=0,Rp=1,Pp=2,Lp=3,Wh=4,Np=5,Dp=6,Ip=7,Yc="attached",Up="detached",$h=300,$s=301,js=302,fl=303,pl=304,Ka=306,Ki=1e3,zn=1001,ml=1002,An=1003,kp=1004,Hr=1005,wn=1006,mo=1007,oi=1008,hi=1009,jh=1010,Xh=1011,Mr=1012,rc=1013,ts=1014,jn=1015,Rr=1016,ac=1017,oc=1018,Xs=1020,qh=35902,Yh=1021,Kh=1022,Pn=1023,Jh=1024,Zh=1025,Os=1026,qs=1027,Qh=1028,lc=1029,td=1030,cc=1031,uc=1033,Aa=33776,Ca=33777,Ra=33778,Pa=33779,gl=35840,_l=35841,vl=35842,xl=35843,bl=36196,yl=37492,Ml=37496,Sl=37808,El=37809,Tl=37810,wl=37811,Al=37812,Cl=37813,Rl=37814,Pl=37815,Ll=37816,Nl=37817,Dl=37818,Il=37819,Ul=37820,kl=37821,La=36492,Fl=36494,Ol=36495,ed=36283,Bl=36284,zl=36285,Vl=36286,ka=2300,Hl=2301,go=2302,Kc=2400,Jc=2401,Zc=2402,Fp=2500,Op=3200,Bp=3201,Ja=0,zp=1,Ai="",ze="srgb",Ys="srgb-linear",Fa="linear",Ce="srgb",_s=7680,Qc=519,Vp=512,Hp=513,Gp=514,nd=515,Wp=516,$p=517,jp=518,Xp=519,tu=35044,eu="300 es",li=2e3,Oa=2001;class rs{addEventListener(t,e){this._listeners===void 0&&(this._listeners={});const i=this._listeners;i[t]===void 0&&(i[t]=[]),i[t].indexOf(e)===-1&&i[t].push(e)}hasEventListener(t,e){const i=this._listeners;return i===void 0?!1:i[t]!==void 0&&i[t].indexOf(e)!==-1}removeEventListener(t,e){const i=this._listeners;if(i===void 0)return;const s=i[t];if(s!==void 0){const r=s.indexOf(e);r!==-1&&s.splice(r,1)}}dispatchEvent(t){const e=this._listeners;if(e===void 0)return;const i=e[t.type];if(i!==void 0){t.target=this;const s=i.slice(0);for(let r=0,a=s.length;r<a;r++)s[r].call(this,t);t.target=null}}}const sn=["00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f","10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f","20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f","30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f","40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f","50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f","60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f","70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f","80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f","90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f","a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af","b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf","c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf","d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df","e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef","f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"];let nu=1234567;const Bs=Math.PI/180,Ks=180/Math.PI;function Ui(){const n=Math.random()*4294967295|0,t=Math.random()*4294967295|0,e=Math.random()*4294967295|0,i=Math.random()*4294967295|0;return(sn[n&255]+sn[n>>8&255]+sn[n>>16&255]+sn[n>>24&255]+"-"+sn[t&255]+sn[t>>8&255]+"-"+sn[t>>16&15|64]+sn[t>>24&255]+"-"+sn[e&63|128]+sn[e>>8&255]+"-"+sn[e>>16&255]+sn[e>>24&255]+sn[i&255]+sn[i>>8&255]+sn[i>>16&255]+sn[i>>24&255]).toLowerCase()}function he(n,t,e){return Math.max(t,Math.min(e,n))}function hc(n,t){return(n%t+t)%t}function qp(n,t,e,i,s){return i+(n-t)*(s-i)/(e-t)}function Yp(n,t,e){return n!==t?(e-n)/(t-n):0}function xr(n,t,e){return(1-e)*n+e*t}function Kp(n,t,e,i){return xr(n,t,1-Math.exp(-e*i))}function Jp(n,t=1){return t-Math.abs(hc(n,t*2)-t)}function Zp(n,t,e){return n<=t?0:n>=e?1:(n=(n-t)/(e-t),n*n*(3-2*n))}function Qp(n,t,e){return n<=t?0:n>=e?1:(n=(n-t)/(e-t),n*n*n*(n*(n*6-15)+10))}function tm(n,t){return n+Math.floor(Math.random()*(t-n+1))}function em(n,t){return n+Math.random()*(t-n)}function nm(n){return n*(.5-Math.random())}function im(n){n!==void 0&&(nu=n);let t=nu+=1831565813;return t=Math.imul(t^t>>>15,t|1),t^=t+Math.imul(t^t>>>7,t|61),((t^t>>>14)>>>0)/4294967296}function sm(n){return n*Bs}function rm(n){return n*Ks}function am(n){return(n&n-1)===0&&n!==0}function om(n){return Math.pow(2,Math.ceil(Math.log(n)/Math.LN2))}function lm(n){return Math.pow(2,Math.floor(Math.log(n)/Math.LN2))}function cm(n,t,e,i,s){const r=Math.cos,a=Math.sin,o=r(e/2),l=a(e/2),c=r((t+i)/2),u=a((t+i)/2),h=r((t-i)/2),d=a((t-i)/2),p=r((i-t)/2),g=a((i-t)/2);switch(s){case"XYX":n.set(o*u,l*h,l*d,o*c);break;case"YZY":n.set(l*d,o*u,l*h,o*c);break;case"ZXZ":n.set(l*h,l*d,o*u,o*c);break;case"XZX":n.set(o*u,l*g,l*p,o*c);break;case"YXY":n.set(l*p,o*u,l*g,o*c);break;case"ZYZ":n.set(l*g,l*p,o*u,o*c);break;default:console.warn("THREE.MathUtils: .setQuaternionFromProperEuler() encountered an unknown order: "+s)}}function Ns(n,t){switch(t.constructor){case Float32Array:return n;case Uint32Array:return n/4294967295;case Uint16Array:return n/65535;case Uint8Array:return n/255;case Int32Array:return Math.max(n/2147483647,-1);case Int16Array:return Math.max(n/32767,-1);case Int8Array:return Math.max(n/127,-1);default:throw new Error("Invalid component type.")}}function un(n,t){switch(t.constructor){case Float32Array:return n;case Uint32Array:return Math.round(n*4294967295);case Uint16Array:return Math.round(n*65535);case Uint8Array:return Math.round(n*255);case Int32Array:return Math.round(n*2147483647);case Int16Array:return Math.round(n*32767);case Int8Array:return Math.round(n*127);default:throw new Error("Invalid component type.")}}const Ds={DEG2RAD:Bs,RAD2DEG:Ks,generateUUID:Ui,clamp:he,euclideanModulo:hc,mapLinear:qp,inverseLerp:Yp,lerp:xr,damp:Kp,pingpong:Jp,smoothstep:Zp,smootherstep:Qp,randInt:tm,randFloat:em,randFloatSpread:nm,seededRandom:im,degToRad:sm,radToDeg:rm,isPowerOfTwo:am,ceilPowerOfTwo:om,floorPowerOfTwo:lm,setQuaternionFromProperEuler:cm,normalize:un,denormalize:Ns};class Yt{constructor(t=0,e=0){Yt.prototype.isVector2=!0,this.x=t,this.y=e}get width(){return this.x}set width(t){this.x=t}get height(){return this.y}set height(t){this.y=t}set(t,e){return this.x=t,this.y=e,this}setScalar(t){return this.x=t,this.y=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y)}copy(t){return this.x=t.x,this.y=t.y,this}add(t){return this.x+=t.x,this.y+=t.y,this}addScalar(t){return this.x+=t,this.y+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this}subScalar(t){return this.x-=t,this.y-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this}multiply(t){return this.x*=t.x,this.y*=t.y,this}multiplyScalar(t){return this.x*=t,this.y*=t,this}divide(t){return this.x/=t.x,this.y/=t.y,this}divideScalar(t){return this.multiplyScalar(1/t)}applyMatrix3(t){const e=this.x,i=this.y,s=t.elements;return this.x=s[0]*e+s[3]*i+s[6],this.y=s[1]*e+s[4]*i+s[7],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this}clamp(t,e){return this.x=he(this.x,t.x,e.x),this.y=he(this.y,t.y,e.y),this}clampScalar(t,e){return this.x=he(this.x,t,e),this.y=he(this.y,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(he(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this}negate(){return this.x=-this.x,this.y=-this.y,this}dot(t){return this.x*t.x+this.y*t.y}cross(t){return this.x*t.y-this.y*t.x}lengthSq(){return this.x*this.x+this.y*this.y}length(){return Math.sqrt(this.x*this.x+this.y*this.y)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)}normalize(){return this.divideScalar(this.length()||1)}angle(){return Math.atan2(-this.y,-this.x)+Math.PI}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(he(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y;return e*e+i*i}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this}equals(t){return t.x===this.x&&t.y===this.y}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this}rotateAround(t,e){const i=Math.cos(e),s=Math.sin(e),r=this.x-t.x,a=this.y-t.y;return this.x=r*i-a*s+t.x,this.y=r*s+a*i+t.y,this}random(){return this.x=Math.random(),this.y=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y}}class le{constructor(t,e,i,s,r,a,o,l,c){le.prototype.isMatrix3=!0,this.elements=[1,0,0,0,1,0,0,0,1],t!==void 0&&this.set(t,e,i,s,r,a,o,l,c)}set(t,e,i,s,r,a,o,l,c){const u=this.elements;return u[0]=t,u[1]=s,u[2]=o,u[3]=e,u[4]=r,u[5]=l,u[6]=i,u[7]=a,u[8]=c,this}identity(){return this.set(1,0,0,0,1,0,0,0,1),this}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],this}extractBasis(t,e,i){return t.setFromMatrix3Column(this,0),e.setFromMatrix3Column(this,1),i.setFromMatrix3Column(this,2),this}setFromMatrix4(t){const e=t.elements;return this.set(e[0],e[4],e[8],e[1],e[5],e[9],e[2],e[6],e[10]),this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,s=e.elements,r=this.elements,a=i[0],o=i[3],l=i[6],c=i[1],u=i[4],h=i[7],d=i[2],p=i[5],g=i[8],_=s[0],m=s[3],f=s[6],P=s[1],L=s[4],w=s[7],N=s[2],F=s[5],U=s[8];return r[0]=a*_+o*P+l*N,r[3]=a*m+o*L+l*F,r[6]=a*f+o*w+l*U,r[1]=c*_+u*P+h*N,r[4]=c*m+u*L+h*F,r[7]=c*f+u*w+h*U,r[2]=d*_+p*P+g*N,r[5]=d*m+p*L+g*F,r[8]=d*f+p*w+g*U,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[3]*=t,e[6]*=t,e[1]*=t,e[4]*=t,e[7]*=t,e[2]*=t,e[5]*=t,e[8]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],a=t[4],o=t[5],l=t[6],c=t[7],u=t[8];return e*a*u-e*o*c-i*r*u+i*o*l+s*r*c-s*a*l}invert(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],a=t[4],o=t[5],l=t[6],c=t[7],u=t[8],h=u*a-o*c,d=o*l-u*r,p=c*r-a*l,g=e*h+i*d+s*p;if(g===0)return this.set(0,0,0,0,0,0,0,0,0);const _=1/g;return t[0]=h*_,t[1]=(s*c-u*i)*_,t[2]=(o*i-s*a)*_,t[3]=d*_,t[4]=(u*e-s*l)*_,t[5]=(s*r-o*e)*_,t[6]=p*_,t[7]=(i*l-c*e)*_,t[8]=(a*e-i*r)*_,this}transpose(){let t;const e=this.elements;return t=e[1],e[1]=e[3],e[3]=t,t=e[2],e[2]=e[6],e[6]=t,t=e[5],e[5]=e[7],e[7]=t,this}getNormalMatrix(t){return this.setFromMatrix4(t).invert().transpose()}transposeIntoArray(t){const e=this.elements;return t[0]=e[0],t[1]=e[3],t[2]=e[6],t[3]=e[1],t[4]=e[4],t[5]=e[7],t[6]=e[2],t[7]=e[5],t[8]=e[8],this}setUvTransform(t,e,i,s,r,a,o){const l=Math.cos(r),c=Math.sin(r);return this.set(i*l,i*c,-i*(l*a+c*o)+a+t,-s*c,s*l,-s*(-c*a+l*o)+o+e,0,0,1),this}scale(t,e){return this.premultiply(_o.makeScale(t,e)),this}rotate(t){return this.premultiply(_o.makeRotation(-t)),this}translate(t,e){return this.premultiply(_o.makeTranslation(t,e)),this}makeTranslation(t,e){return t.isVector2?this.set(1,0,t.x,0,1,t.y,0,0,1):this.set(1,0,t,0,1,e,0,0,1),this}makeRotation(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,i,e,0,0,0,1),this}makeScale(t,e){return this.set(t,0,0,0,e,0,0,0,1),this}equals(t){const e=this.elements,i=t.elements;for(let s=0;s<9;s++)if(e[s]!==i[s])return!1;return!0}fromArray(t,e=0){for(let i=0;i<9;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t}clone(){return new this.constructor().fromArray(this.elements)}}const _o=new le;function id(n){for(let t=n.length-1;t>=0;--t)if(n[t]>=65535)return!0;return!1}function Sr(n){return document.createElementNS("http://www.w3.org/1999/xhtml",n)}function um(){const n=Sr("canvas");return n.style.display="block",n}const iu={};function ji(n){n in iu||(iu[n]=!0,console.warn(n))}function hm(n,t,e){return new Promise(function(i,s){function r(){switch(n.clientWaitSync(t,n.SYNC_FLUSH_COMMANDS_BIT,0)){case n.WAIT_FAILED:s();break;case n.TIMEOUT_EXPIRED:setTimeout(r,e);break;default:i()}}setTimeout(r,e)})}function dm(n){const t=n.elements;t[2]=.5*t[2]+.5*t[3],t[6]=.5*t[6]+.5*t[7],t[10]=.5*t[10]+.5*t[11],t[14]=.5*t[14]+.5*t[15]}function fm(n){const t=n.elements;t[11]===-1?(t[10]=-t[10]-1,t[14]=-t[14]):(t[10]=-t[10],t[14]=-t[14]+1)}const su=new le().set(.4123908,.3575843,.1804808,.212639,.7151687,.0721923,.0193308,.1191948,.9505322),ru=new le().set(3.2409699,-1.5373832,-.4986108,-.9692436,1.8759675,.0415551,.0556301,-.203977,1.0569715);function pm(){const n={enabled:!0,workingColorSpace:Ys,spaces:{},convert:function(s,r,a){return this.enabled===!1||r===a||!r||!a||(this.spaces[r].transfer===Ce&&(s.r=ci(s.r),s.g=ci(s.g),s.b=ci(s.b)),this.spaces[r].primaries!==this.spaces[a].primaries&&(s.applyMatrix3(this.spaces[r].toXYZ),s.applyMatrix3(this.spaces[a].fromXYZ)),this.spaces[a].transfer===Ce&&(s.r=zs(s.r),s.g=zs(s.g),s.b=zs(s.b))),s},fromWorkingColorSpace:function(s,r){return this.convert(s,this.workingColorSpace,r)},toWorkingColorSpace:function(s,r){return this.convert(s,r,this.workingColorSpace)},getPrimaries:function(s){return this.spaces[s].primaries},getTransfer:function(s){return s===Ai?Fa:this.spaces[s].transfer},getLuminanceCoefficients:function(s,r=this.workingColorSpace){return s.fromArray(this.spaces[r].luminanceCoefficients)},define:function(s){Object.assign(this.spaces,s)},_getMatrix:function(s,r,a){return s.copy(this.spaces[r].toXYZ).multiply(this.spaces[a].fromXYZ)},_getDrawingBufferColorSpace:function(s){return this.spaces[s].outputColorSpaceConfig.drawingBufferColorSpace},_getUnpackColorSpace:function(s=this.workingColorSpace){return this.spaces[s].workingColorSpaceConfig.unpackColorSpace}},t=[.64,.33,.3,.6,.15,.06],e=[.2126,.7152,.0722],i=[.3127,.329];return n.define({[Ys]:{primaries:t,whitePoint:i,transfer:Fa,toXYZ:su,fromXYZ:ru,luminanceCoefficients:e,workingColorSpaceConfig:{unpackColorSpace:ze},outputColorSpaceConfig:{drawingBufferColorSpace:ze}},[ze]:{primaries:t,whitePoint:i,transfer:Ce,toXYZ:su,fromXYZ:ru,luminanceCoefficients:e,outputColorSpaceConfig:{drawingBufferColorSpace:ze}}}),n}const _e=pm();function ci(n){return n<.04045?n*.0773993808:Math.pow(n*.9478672986+.0521327014,2.4)}function zs(n){return n<.0031308?n*12.92:1.055*Math.pow(n,.41666)-.055}let vs;class mm{static getDataURL(t){if(/^data:/i.test(t.src)||typeof HTMLCanvasElement>"u")return t.src;let e;if(t instanceof HTMLCanvasElement)e=t;else{vs===void 0&&(vs=Sr("canvas")),vs.width=t.width,vs.height=t.height;const i=vs.getContext("2d");t instanceof ImageData?i.putImageData(t,0,0):i.drawImage(t,0,0,t.width,t.height),e=vs}return e.toDataURL("image/png")}static sRGBToLinear(t){if(typeof HTMLImageElement<"u"&&t instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&t instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&t instanceof ImageBitmap){const e=Sr("canvas");e.width=t.width,e.height=t.height;const i=e.getContext("2d");i.drawImage(t,0,0,t.width,t.height);const s=i.getImageData(0,0,t.width,t.height),r=s.data;for(let a=0;a<r.length;a++)r[a]=ci(r[a]/255)*255;return i.putImageData(s,0,0),e}else if(t.data){const e=t.data.slice(0);for(let i=0;i<e.length;i++)e instanceof Uint8Array||e instanceof Uint8ClampedArray?e[i]=Math.floor(ci(e[i]/255)*255):e[i]=ci(e[i]);return{data:e,width:t.width,height:t.height}}else return console.warn("THREE.ImageUtils.sRGBToLinear(): Unsupported image type. No color space conversion applied."),t}}let gm=0;class dc{constructor(t=null){this.isSource=!0,Object.defineProperty(this,"id",{value:gm++}),this.uuid=Ui(),this.data=t,this.dataReady=!0,this.version=0}set needsUpdate(t){t===!0&&this.version++}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.images[this.uuid]!==void 0)return t.images[this.uuid];const i={uuid:this.uuid,url:""},s=this.data;if(s!==null){let r;if(Array.isArray(s)){r=[];for(let a=0,o=s.length;a<o;a++)s[a].isDataTexture?r.push(vo(s[a].image)):r.push(vo(s[a]))}else r=vo(s);i.url=r}return e||(t.images[this.uuid]=i),i}}function vo(n){return typeof HTMLImageElement<"u"&&n instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&n instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&n instanceof ImageBitmap?mm.getDataURL(n):n.data?{data:Array.from(n.data),width:n.width,height:n.height,type:n.data.constructor.name}:(console.warn("THREE.Texture: Unable to serialize Texture."),{})}let _m=0;class ln extends rs{constructor(t=ln.DEFAULT_IMAGE,e=ln.DEFAULT_MAPPING,i=zn,s=zn,r=wn,a=oi,o=Pn,l=hi,c=ln.DEFAULT_ANISOTROPY,u=Ai){super(),this.isTexture=!0,Object.defineProperty(this,"id",{value:_m++}),this.uuid=Ui(),this.name="",this.source=new dc(t),this.mipmaps=[],this.mapping=e,this.channel=0,this.wrapS=i,this.wrapT=s,this.magFilter=r,this.minFilter=a,this.anisotropy=c,this.format=o,this.internalFormat=null,this.type=l,this.offset=new Yt(0,0),this.repeat=new Yt(1,1),this.center=new Yt(0,0),this.rotation=0,this.matrixAutoUpdate=!0,this.matrix=new le,this.generateMipmaps=!0,this.premultiplyAlpha=!1,this.flipY=!0,this.unpackAlignment=4,this.colorSpace=u,this.userData={},this.version=0,this.onUpdate=null,this.renderTarget=null,this.isRenderTargetTexture=!1,this.pmremVersion=0}get image(){return this.source.data}set image(t=null){this.source.data=t}updateMatrix(){this.matrix.setUvTransform(this.offset.x,this.offset.y,this.repeat.x,this.repeat.y,this.rotation,this.center.x,this.center.y)}clone(){return new this.constructor().copy(this)}copy(t){return this.name=t.name,this.source=t.source,this.mipmaps=t.mipmaps.slice(0),this.mapping=t.mapping,this.channel=t.channel,this.wrapS=t.wrapS,this.wrapT=t.wrapT,this.magFilter=t.magFilter,this.minFilter=t.minFilter,this.anisotropy=t.anisotropy,this.format=t.format,this.internalFormat=t.internalFormat,this.type=t.type,this.offset.copy(t.offset),this.repeat.copy(t.repeat),this.center.copy(t.center),this.rotation=t.rotation,this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrix.copy(t.matrix),this.generateMipmaps=t.generateMipmaps,this.premultiplyAlpha=t.premultiplyAlpha,this.flipY=t.flipY,this.unpackAlignment=t.unpackAlignment,this.colorSpace=t.colorSpace,this.renderTarget=t.renderTarget,this.isRenderTargetTexture=t.isRenderTargetTexture,this.userData=JSON.parse(JSON.stringify(t.userData)),this.needsUpdate=!0,this}toJSON(t){const e=t===void 0||typeof t=="string";if(!e&&t.textures[this.uuid]!==void 0)return t.textures[this.uuid];const i={metadata:{version:4.6,type:"Texture",generator:"Texture.toJSON"},uuid:this.uuid,name:this.name,image:this.source.toJSON(t).uuid,mapping:this.mapping,channel:this.channel,repeat:[this.repeat.x,this.repeat.y],offset:[this.offset.x,this.offset.y],center:[this.center.x,this.center.y],rotation:this.rotation,wrap:[this.wrapS,this.wrapT],format:this.format,internalFormat:this.internalFormat,type:this.type,colorSpace:this.colorSpace,minFilter:this.minFilter,magFilter:this.magFilter,anisotropy:this.anisotropy,flipY:this.flipY,generateMipmaps:this.generateMipmaps,premultiplyAlpha:this.premultiplyAlpha,unpackAlignment:this.unpackAlignment};return Object.keys(this.userData).length>0&&(i.userData=this.userData),e||(t.textures[this.uuid]=i),i}dispose(){this.dispatchEvent({type:"dispose"})}transformUv(t){if(this.mapping!==$h)return t;if(t.applyMatrix3(this.matrix),t.x<0||t.x>1)switch(this.wrapS){case Ki:t.x=t.x-Math.floor(t.x);break;case zn:t.x=t.x<0?0:1;break;case ml:Math.abs(Math.floor(t.x)%2)===1?t.x=Math.ceil(t.x)-t.x:t.x=t.x-Math.floor(t.x);break}if(t.y<0||t.y>1)switch(this.wrapT){case Ki:t.y=t.y-Math.floor(t.y);break;case zn:t.y=t.y<0?0:1;break;case ml:Math.abs(Math.floor(t.y)%2)===1?t.y=Math.ceil(t.y)-t.y:t.y=t.y-Math.floor(t.y);break}return this.flipY&&(t.y=1-t.y),t}set needsUpdate(t){t===!0&&(this.version++,this.source.needsUpdate=!0)}set needsPMREMUpdate(t){t===!0&&this.pmremVersion++}}ln.DEFAULT_IMAGE=null;ln.DEFAULT_MAPPING=$h;ln.DEFAULT_ANISOTROPY=1;class Ee{constructor(t=0,e=0,i=0,s=1){Ee.prototype.isVector4=!0,this.x=t,this.y=e,this.z=i,this.w=s}get width(){return this.z}set width(t){this.z=t}get height(){return this.w}set height(t){this.w=t}set(t,e,i,s){return this.x=t,this.y=e,this.z=i,this.w=s,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this.w=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setW(t){return this.w=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;case 3:this.w=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;case 3:return this.w;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z,this.w)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this.w=t.w!==void 0?t.w:1,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this.w+=t.w,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this.w+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this.w=t.w+e.w,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this.w+=t.w*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this.w-=t.w,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this.w-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this.w=t.w-e.w,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this.w*=t.w,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this.w*=t,this}applyMatrix4(t){const e=this.x,i=this.y,s=this.z,r=this.w,a=t.elements;return this.x=a[0]*e+a[4]*i+a[8]*s+a[12]*r,this.y=a[1]*e+a[5]*i+a[9]*s+a[13]*r,this.z=a[2]*e+a[6]*i+a[10]*s+a[14]*r,this.w=a[3]*e+a[7]*i+a[11]*s+a[15]*r,this}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this.w/=t.w,this}divideScalar(t){return this.multiplyScalar(1/t)}setAxisAngleFromQuaternion(t){this.w=2*Math.acos(t.w);const e=Math.sqrt(1-t.w*t.w);return e<1e-4?(this.x=1,this.y=0,this.z=0):(this.x=t.x/e,this.y=t.y/e,this.z=t.z/e),this}setAxisAngleFromRotationMatrix(t){let e,i,s,r;const l=t.elements,c=l[0],u=l[4],h=l[8],d=l[1],p=l[5],g=l[9],_=l[2],m=l[6],f=l[10];if(Math.abs(u-d)<.01&&Math.abs(h-_)<.01&&Math.abs(g-m)<.01){if(Math.abs(u+d)<.1&&Math.abs(h+_)<.1&&Math.abs(g+m)<.1&&Math.abs(c+p+f-3)<.1)return this.set(1,0,0,0),this;e=Math.PI;const L=(c+1)/2,w=(p+1)/2,N=(f+1)/2,F=(u+d)/4,U=(h+_)/4,k=(g+m)/4;return L>w&&L>N?L<.01?(i=0,s=.707106781,r=.707106781):(i=Math.sqrt(L),s=F/i,r=U/i):w>N?w<.01?(i=.707106781,s=0,r=.707106781):(s=Math.sqrt(w),i=F/s,r=k/s):N<.01?(i=.707106781,s=.707106781,r=0):(r=Math.sqrt(N),i=U/r,s=k/r),this.set(i,s,r,e),this}let P=Math.sqrt((m-g)*(m-g)+(h-_)*(h-_)+(d-u)*(d-u));return Math.abs(P)<.001&&(P=1),this.x=(m-g)/P,this.y=(h-_)/P,this.z=(d-u)/P,this.w=Math.acos((c+p+f-1)/2),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this.w=e[15],this}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this.w=Math.min(this.w,t.w),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this.w=Math.max(this.w,t.w),this}clamp(t,e){return this.x=he(this.x,t.x,e.x),this.y=he(this.y,t.y,e.y),this.z=he(this.z,t.z,e.z),this.w=he(this.w,t.w,e.w),this}clampScalar(t,e){return this.x=he(this.x,t,e),this.y=he(this.y,t,e),this.z=he(this.z,t,e),this.w=he(this.w,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(he(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this.w=Math.floor(this.w),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this.w=Math.ceil(this.w),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this.w=Math.round(this.w),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this.w=Math.trunc(this.w),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this.w=-this.w,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z+this.w*t.w}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z+this.w*this.w)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)+Math.abs(this.w)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this.w+=(t.w-this.w)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this.w=t.w+(e.w-t.w)*i,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z&&t.w===this.w}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this.w=t[e+3],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t[e+3]=this.w,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this.w=t.getW(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this.w=Math.random(),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z,yield this.w}}class vm extends rs{constructor(t=1,e=1,i={}){super(),this.isRenderTarget=!0,this.width=t,this.height=e,this.depth=1,this.scissor=new Ee(0,0,t,e),this.scissorTest=!1,this.viewport=new Ee(0,0,t,e);const s={width:t,height:e,depth:1};i=Object.assign({generateMipmaps:!1,internalFormat:null,minFilter:wn,depthBuffer:!0,stencilBuffer:!1,resolveDepthBuffer:!0,resolveStencilBuffer:!0,depthTexture:null,samples:0,count:1},i);const r=new ln(s,i.mapping,i.wrapS,i.wrapT,i.magFilter,i.minFilter,i.format,i.type,i.anisotropy,i.colorSpace);r.flipY=!1,r.generateMipmaps=i.generateMipmaps,r.internalFormat=i.internalFormat,this.textures=[];const a=i.count;for(let o=0;o<a;o++)this.textures[o]=r.clone(),this.textures[o].isRenderTargetTexture=!0,this.textures[o].renderTarget=this;this.depthBuffer=i.depthBuffer,this.stencilBuffer=i.stencilBuffer,this.resolveDepthBuffer=i.resolveDepthBuffer,this.resolveStencilBuffer=i.resolveStencilBuffer,this._depthTexture=null,this.depthTexture=i.depthTexture,this.samples=i.samples}get texture(){return this.textures[0]}set texture(t){this.textures[0]=t}set depthTexture(t){this._depthTexture!==null&&(this._depthTexture.renderTarget=null),t!==null&&(t.renderTarget=this),this._depthTexture=t}get depthTexture(){return this._depthTexture}setSize(t,e,i=1){if(this.width!==t||this.height!==e||this.depth!==i){this.width=t,this.height=e,this.depth=i;for(let s=0,r=this.textures.length;s<r;s++)this.textures[s].image.width=t,this.textures[s].image.height=e,this.textures[s].image.depth=i;this.dispose()}this.viewport.set(0,0,t,e),this.scissor.set(0,0,t,e)}clone(){return new this.constructor().copy(this)}copy(t){this.width=t.width,this.height=t.height,this.depth=t.depth,this.scissor.copy(t.scissor),this.scissorTest=t.scissorTest,this.viewport.copy(t.viewport),this.textures.length=0;for(let e=0,i=t.textures.length;e<i;e++){this.textures[e]=t.textures[e].clone(),this.textures[e].isRenderTargetTexture=!0,this.textures[e].renderTarget=this;const s=Object.assign({},t.textures[e].image);this.textures[e].source=new dc(s)}return this.depthBuffer=t.depthBuffer,this.stencilBuffer=t.stencilBuffer,this.resolveDepthBuffer=t.resolveDepthBuffer,this.resolveStencilBuffer=t.resolveStencilBuffer,t.depthTexture!==null&&(this.depthTexture=t.depthTexture.clone()),this.samples=t.samples,this}dispose(){this.dispatchEvent({type:"dispose"})}}class es extends vm{constructor(t=1,e=1,i={}){super(t,e,i),this.isWebGLRenderTarget=!0}}class sd extends ln{constructor(t=null,e=1,i=1,s=1){super(null),this.isDataArrayTexture=!0,this.image={data:t,width:e,height:i,depth:s},this.magFilter=An,this.minFilter=An,this.wrapR=zn,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1,this.layerUpdates=new Set}addLayerUpdate(t){this.layerUpdates.add(t)}clearLayerUpdates(){this.layerUpdates.clear()}}class xm extends ln{constructor(t=null,e=1,i=1,s=1){super(null),this.isData3DTexture=!0,this.image={data:t,width:e,height:i,depth:s},this.magFilter=An,this.minFilter=An,this.wrapR=zn,this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}class Nn{constructor(t=0,e=0,i=0,s=1){this.isQuaternion=!0,this._x=t,this._y=e,this._z=i,this._w=s}static slerpFlat(t,e,i,s,r,a,o){let l=i[s+0],c=i[s+1],u=i[s+2],h=i[s+3];const d=r[a+0],p=r[a+1],g=r[a+2],_=r[a+3];if(o===0){t[e+0]=l,t[e+1]=c,t[e+2]=u,t[e+3]=h;return}if(o===1){t[e+0]=d,t[e+1]=p,t[e+2]=g,t[e+3]=_;return}if(h!==_||l!==d||c!==p||u!==g){let m=1-o;const f=l*d+c*p+u*g+h*_,P=f>=0?1:-1,L=1-f*f;if(L>Number.EPSILON){const N=Math.sqrt(L),F=Math.atan2(N,f*P);m=Math.sin(m*F)/N,o=Math.sin(o*F)/N}const w=o*P;if(l=l*m+d*w,c=c*m+p*w,u=u*m+g*w,h=h*m+_*w,m===1-o){const N=1/Math.sqrt(l*l+c*c+u*u+h*h);l*=N,c*=N,u*=N,h*=N}}t[e]=l,t[e+1]=c,t[e+2]=u,t[e+3]=h}static multiplyQuaternionsFlat(t,e,i,s,r,a){const o=i[s],l=i[s+1],c=i[s+2],u=i[s+3],h=r[a],d=r[a+1],p=r[a+2],g=r[a+3];return t[e]=o*g+u*h+l*p-c*d,t[e+1]=l*g+u*d+c*h-o*p,t[e+2]=c*g+u*p+o*d-l*h,t[e+3]=u*g-o*h-l*d-c*p,t}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get w(){return this._w}set w(t){this._w=t,this._onChangeCallback()}set(t,e,i,s){return this._x=t,this._y=e,this._z=i,this._w=s,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._w)}copy(t){return this._x=t.x,this._y=t.y,this._z=t.z,this._w=t.w,this._onChangeCallback(),this}setFromEuler(t,e=!0){const i=t._x,s=t._y,r=t._z,a=t._order,o=Math.cos,l=Math.sin,c=o(i/2),u=o(s/2),h=o(r/2),d=l(i/2),p=l(s/2),g=l(r/2);switch(a){case"XYZ":this._x=d*u*h+c*p*g,this._y=c*p*h-d*u*g,this._z=c*u*g+d*p*h,this._w=c*u*h-d*p*g;break;case"YXZ":this._x=d*u*h+c*p*g,this._y=c*p*h-d*u*g,this._z=c*u*g-d*p*h,this._w=c*u*h+d*p*g;break;case"ZXY":this._x=d*u*h-c*p*g,this._y=c*p*h+d*u*g,this._z=c*u*g+d*p*h,this._w=c*u*h-d*p*g;break;case"ZYX":this._x=d*u*h-c*p*g,this._y=c*p*h+d*u*g,this._z=c*u*g-d*p*h,this._w=c*u*h+d*p*g;break;case"YZX":this._x=d*u*h+c*p*g,this._y=c*p*h+d*u*g,this._z=c*u*g-d*p*h,this._w=c*u*h-d*p*g;break;case"XZY":this._x=d*u*h-c*p*g,this._y=c*p*h-d*u*g,this._z=c*u*g+d*p*h,this._w=c*u*h+d*p*g;break;default:console.warn("THREE.Quaternion: .setFromEuler() encountered an unknown order: "+a)}return e===!0&&this._onChangeCallback(),this}setFromAxisAngle(t,e){const i=e/2,s=Math.sin(i);return this._x=t.x*s,this._y=t.y*s,this._z=t.z*s,this._w=Math.cos(i),this._onChangeCallback(),this}setFromRotationMatrix(t){const e=t.elements,i=e[0],s=e[4],r=e[8],a=e[1],o=e[5],l=e[9],c=e[2],u=e[6],h=e[10],d=i+o+h;if(d>0){const p=.5/Math.sqrt(d+1);this._w=.25/p,this._x=(u-l)*p,this._y=(r-c)*p,this._z=(a-s)*p}else if(i>o&&i>h){const p=2*Math.sqrt(1+i-o-h);this._w=(u-l)/p,this._x=.25*p,this._y=(s+a)/p,this._z=(r+c)/p}else if(o>h){const p=2*Math.sqrt(1+o-i-h);this._w=(r-c)/p,this._x=(s+a)/p,this._y=.25*p,this._z=(l+u)/p}else{const p=2*Math.sqrt(1+h-i-o);this._w=(a-s)/p,this._x=(r+c)/p,this._y=(l+u)/p,this._z=.25*p}return this._onChangeCallback(),this}setFromUnitVectors(t,e){let i=t.dot(e)+1;return i<Number.EPSILON?(i=0,Math.abs(t.x)>Math.abs(t.z)?(this._x=-t.y,this._y=t.x,this._z=0,this._w=i):(this._x=0,this._y=-t.z,this._z=t.y,this._w=i)):(this._x=t.y*e.z-t.z*e.y,this._y=t.z*e.x-t.x*e.z,this._z=t.x*e.y-t.y*e.x,this._w=i),this.normalize()}angleTo(t){return 2*Math.acos(Math.abs(he(this.dot(t),-1,1)))}rotateTowards(t,e){const i=this.angleTo(t);if(i===0)return this;const s=Math.min(1,e/i);return this.slerp(t,s),this}identity(){return this.set(0,0,0,1)}invert(){return this.conjugate()}conjugate(){return this._x*=-1,this._y*=-1,this._z*=-1,this._onChangeCallback(),this}dot(t){return this._x*t._x+this._y*t._y+this._z*t._z+this._w*t._w}lengthSq(){return this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w}length(){return Math.sqrt(this._x*this._x+this._y*this._y+this._z*this._z+this._w*this._w)}normalize(){let t=this.length();return t===0?(this._x=0,this._y=0,this._z=0,this._w=1):(t=1/t,this._x=this._x*t,this._y=this._y*t,this._z=this._z*t,this._w=this._w*t),this._onChangeCallback(),this}multiply(t){return this.multiplyQuaternions(this,t)}premultiply(t){return this.multiplyQuaternions(t,this)}multiplyQuaternions(t,e){const i=t._x,s=t._y,r=t._z,a=t._w,o=e._x,l=e._y,c=e._z,u=e._w;return this._x=i*u+a*o+s*c-r*l,this._y=s*u+a*l+r*o-i*c,this._z=r*u+a*c+i*l-s*o,this._w=a*u-i*o-s*l-r*c,this._onChangeCallback(),this}slerp(t,e){if(e===0)return this;if(e===1)return this.copy(t);const i=this._x,s=this._y,r=this._z,a=this._w;let o=a*t._w+i*t._x+s*t._y+r*t._z;if(o<0?(this._w=-t._w,this._x=-t._x,this._y=-t._y,this._z=-t._z,o=-o):this.copy(t),o>=1)return this._w=a,this._x=i,this._y=s,this._z=r,this;const l=1-o*o;if(l<=Number.EPSILON){const p=1-e;return this._w=p*a+e*this._w,this._x=p*i+e*this._x,this._y=p*s+e*this._y,this._z=p*r+e*this._z,this.normalize(),this}const c=Math.sqrt(l),u=Math.atan2(c,o),h=Math.sin((1-e)*u)/c,d=Math.sin(e*u)/c;return this._w=a*h+this._w*d,this._x=i*h+this._x*d,this._y=s*h+this._y*d,this._z=r*h+this._z*d,this._onChangeCallback(),this}slerpQuaternions(t,e,i){return this.copy(t).slerp(e,i)}random(){const t=2*Math.PI*Math.random(),e=2*Math.PI*Math.random(),i=Math.random(),s=Math.sqrt(1-i),r=Math.sqrt(i);return this.set(s*Math.sin(t),s*Math.cos(t),r*Math.sin(e),r*Math.cos(e))}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._w===this._w}fromArray(t,e=0){return this._x=t[e],this._y=t[e+1],this._z=t[e+2],this._w=t[e+3],this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._w,t}fromBufferAttribute(t,e){return this._x=t.getX(e),this._y=t.getY(e),this._z=t.getZ(e),this._w=t.getW(e),this._onChangeCallback(),this}toJSON(){return this.toArray()}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._w}}class O{constructor(t=0,e=0,i=0){O.prototype.isVector3=!0,this.x=t,this.y=e,this.z=i}set(t,e,i){return i===void 0&&(i=this.z),this.x=t,this.y=e,this.z=i,this}setScalar(t){return this.x=t,this.y=t,this.z=t,this}setX(t){return this.x=t,this}setY(t){return this.y=t,this}setZ(t){return this.z=t,this}setComponent(t,e){switch(t){case 0:this.x=e;break;case 1:this.y=e;break;case 2:this.z=e;break;default:throw new Error("index is out of range: "+t)}return this}getComponent(t){switch(t){case 0:return this.x;case 1:return this.y;case 2:return this.z;default:throw new Error("index is out of range: "+t)}}clone(){return new this.constructor(this.x,this.y,this.z)}copy(t){return this.x=t.x,this.y=t.y,this.z=t.z,this}add(t){return this.x+=t.x,this.y+=t.y,this.z+=t.z,this}addScalar(t){return this.x+=t,this.y+=t,this.z+=t,this}addVectors(t,e){return this.x=t.x+e.x,this.y=t.y+e.y,this.z=t.z+e.z,this}addScaledVector(t,e){return this.x+=t.x*e,this.y+=t.y*e,this.z+=t.z*e,this}sub(t){return this.x-=t.x,this.y-=t.y,this.z-=t.z,this}subScalar(t){return this.x-=t,this.y-=t,this.z-=t,this}subVectors(t,e){return this.x=t.x-e.x,this.y=t.y-e.y,this.z=t.z-e.z,this}multiply(t){return this.x*=t.x,this.y*=t.y,this.z*=t.z,this}multiplyScalar(t){return this.x*=t,this.y*=t,this.z*=t,this}multiplyVectors(t,e){return this.x=t.x*e.x,this.y=t.y*e.y,this.z=t.z*e.z,this}applyEuler(t){return this.applyQuaternion(au.setFromEuler(t))}applyAxisAngle(t,e){return this.applyQuaternion(au.setFromAxisAngle(t,e))}applyMatrix3(t){const e=this.x,i=this.y,s=this.z,r=t.elements;return this.x=r[0]*e+r[3]*i+r[6]*s,this.y=r[1]*e+r[4]*i+r[7]*s,this.z=r[2]*e+r[5]*i+r[8]*s,this}applyNormalMatrix(t){return this.applyMatrix3(t).normalize()}applyMatrix4(t){const e=this.x,i=this.y,s=this.z,r=t.elements,a=1/(r[3]*e+r[7]*i+r[11]*s+r[15]);return this.x=(r[0]*e+r[4]*i+r[8]*s+r[12])*a,this.y=(r[1]*e+r[5]*i+r[9]*s+r[13])*a,this.z=(r[2]*e+r[6]*i+r[10]*s+r[14])*a,this}applyQuaternion(t){const e=this.x,i=this.y,s=this.z,r=t.x,a=t.y,o=t.z,l=t.w,c=2*(a*s-o*i),u=2*(o*e-r*s),h=2*(r*i-a*e);return this.x=e+l*c+a*h-o*u,this.y=i+l*u+o*c-r*h,this.z=s+l*h+r*u-a*c,this}project(t){return this.applyMatrix4(t.matrixWorldInverse).applyMatrix4(t.projectionMatrix)}unproject(t){return this.applyMatrix4(t.projectionMatrixInverse).applyMatrix4(t.matrixWorld)}transformDirection(t){const e=this.x,i=this.y,s=this.z,r=t.elements;return this.x=r[0]*e+r[4]*i+r[8]*s,this.y=r[1]*e+r[5]*i+r[9]*s,this.z=r[2]*e+r[6]*i+r[10]*s,this.normalize()}divide(t){return this.x/=t.x,this.y/=t.y,this.z/=t.z,this}divideScalar(t){return this.multiplyScalar(1/t)}min(t){return this.x=Math.min(this.x,t.x),this.y=Math.min(this.y,t.y),this.z=Math.min(this.z,t.z),this}max(t){return this.x=Math.max(this.x,t.x),this.y=Math.max(this.y,t.y),this.z=Math.max(this.z,t.z),this}clamp(t,e){return this.x=he(this.x,t.x,e.x),this.y=he(this.y,t.y,e.y),this.z=he(this.z,t.z,e.z),this}clampScalar(t,e){return this.x=he(this.x,t,e),this.y=he(this.y,t,e),this.z=he(this.z,t,e),this}clampLength(t,e){const i=this.length();return this.divideScalar(i||1).multiplyScalar(he(i,t,e))}floor(){return this.x=Math.floor(this.x),this.y=Math.floor(this.y),this.z=Math.floor(this.z),this}ceil(){return this.x=Math.ceil(this.x),this.y=Math.ceil(this.y),this.z=Math.ceil(this.z),this}round(){return this.x=Math.round(this.x),this.y=Math.round(this.y),this.z=Math.round(this.z),this}roundToZero(){return this.x=Math.trunc(this.x),this.y=Math.trunc(this.y),this.z=Math.trunc(this.z),this}negate(){return this.x=-this.x,this.y=-this.y,this.z=-this.z,this}dot(t){return this.x*t.x+this.y*t.y+this.z*t.z}lengthSq(){return this.x*this.x+this.y*this.y+this.z*this.z}length(){return Math.sqrt(this.x*this.x+this.y*this.y+this.z*this.z)}manhattanLength(){return Math.abs(this.x)+Math.abs(this.y)+Math.abs(this.z)}normalize(){return this.divideScalar(this.length()||1)}setLength(t){return this.normalize().multiplyScalar(t)}lerp(t,e){return this.x+=(t.x-this.x)*e,this.y+=(t.y-this.y)*e,this.z+=(t.z-this.z)*e,this}lerpVectors(t,e,i){return this.x=t.x+(e.x-t.x)*i,this.y=t.y+(e.y-t.y)*i,this.z=t.z+(e.z-t.z)*i,this}cross(t){return this.crossVectors(this,t)}crossVectors(t,e){const i=t.x,s=t.y,r=t.z,a=e.x,o=e.y,l=e.z;return this.x=s*l-r*o,this.y=r*a-i*l,this.z=i*o-s*a,this}projectOnVector(t){const e=t.lengthSq();if(e===0)return this.set(0,0,0);const i=t.dot(this)/e;return this.copy(t).multiplyScalar(i)}projectOnPlane(t){return xo.copy(this).projectOnVector(t),this.sub(xo)}reflect(t){return this.sub(xo.copy(t).multiplyScalar(2*this.dot(t)))}angleTo(t){const e=Math.sqrt(this.lengthSq()*t.lengthSq());if(e===0)return Math.PI/2;const i=this.dot(t)/e;return Math.acos(he(i,-1,1))}distanceTo(t){return Math.sqrt(this.distanceToSquared(t))}distanceToSquared(t){const e=this.x-t.x,i=this.y-t.y,s=this.z-t.z;return e*e+i*i+s*s}manhattanDistanceTo(t){return Math.abs(this.x-t.x)+Math.abs(this.y-t.y)+Math.abs(this.z-t.z)}setFromSpherical(t){return this.setFromSphericalCoords(t.radius,t.phi,t.theta)}setFromSphericalCoords(t,e,i){const s=Math.sin(e)*t;return this.x=s*Math.sin(i),this.y=Math.cos(e)*t,this.z=s*Math.cos(i),this}setFromCylindrical(t){return this.setFromCylindricalCoords(t.radius,t.theta,t.y)}setFromCylindricalCoords(t,e,i){return this.x=t*Math.sin(e),this.y=i,this.z=t*Math.cos(e),this}setFromMatrixPosition(t){const e=t.elements;return this.x=e[12],this.y=e[13],this.z=e[14],this}setFromMatrixScale(t){const e=this.setFromMatrixColumn(t,0).length(),i=this.setFromMatrixColumn(t,1).length(),s=this.setFromMatrixColumn(t,2).length();return this.x=e,this.y=i,this.z=s,this}setFromMatrixColumn(t,e){return this.fromArray(t.elements,e*4)}setFromMatrix3Column(t,e){return this.fromArray(t.elements,e*3)}setFromEuler(t){return this.x=t._x,this.y=t._y,this.z=t._z,this}setFromColor(t){return this.x=t.r,this.y=t.g,this.z=t.b,this}equals(t){return t.x===this.x&&t.y===this.y&&t.z===this.z}fromArray(t,e=0){return this.x=t[e],this.y=t[e+1],this.z=t[e+2],this}toArray(t=[],e=0){return t[e]=this.x,t[e+1]=this.y,t[e+2]=this.z,t}fromBufferAttribute(t,e){return this.x=t.getX(e),this.y=t.getY(e),this.z=t.getZ(e),this}random(){return this.x=Math.random(),this.y=Math.random(),this.z=Math.random(),this}randomDirection(){const t=Math.random()*Math.PI*2,e=Math.random()*2-1,i=Math.sqrt(1-e*e);return this.x=i*Math.cos(t),this.y=e,this.z=i*Math.sin(t),this}*[Symbol.iterator](){yield this.x,yield this.y,yield this.z}}const xo=new O,au=new Nn;class Li{constructor(t=new O(1/0,1/0,1/0),e=new O(-1/0,-1/0,-1/0)){this.isBox3=!0,this.min=t,this.max=e}set(t,e){return this.min.copy(t),this.max.copy(e),this}setFromArray(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e+=3)this.expandByPoint(In.fromArray(t,e));return this}setFromBufferAttribute(t){this.makeEmpty();for(let e=0,i=t.count;e<i;e++)this.expandByPoint(In.fromBufferAttribute(t,e));return this}setFromPoints(t){this.makeEmpty();for(let e=0,i=t.length;e<i;e++)this.expandByPoint(t[e]);return this}setFromCenterAndSize(t,e){const i=In.copy(e).multiplyScalar(.5);return this.min.copy(t).sub(i),this.max.copy(t).add(i),this}setFromObject(t,e=!1){return this.makeEmpty(),this.expandByObject(t,e)}clone(){return new this.constructor().copy(this)}copy(t){return this.min.copy(t.min),this.max.copy(t.max),this}makeEmpty(){return this.min.x=this.min.y=this.min.z=1/0,this.max.x=this.max.y=this.max.z=-1/0,this}isEmpty(){return this.max.x<this.min.x||this.max.y<this.min.y||this.max.z<this.min.z}getCenter(t){return this.isEmpty()?t.set(0,0,0):t.addVectors(this.min,this.max).multiplyScalar(.5)}getSize(t){return this.isEmpty()?t.set(0,0,0):t.subVectors(this.max,this.min)}expandByPoint(t){return this.min.min(t),this.max.max(t),this}expandByVector(t){return this.min.sub(t),this.max.add(t),this}expandByScalar(t){return this.min.addScalar(-t),this.max.addScalar(t),this}expandByObject(t,e=!1){t.updateWorldMatrix(!1,!1);const i=t.geometry;if(i!==void 0){const r=i.getAttribute("position");if(e===!0&&r!==void 0&&t.isInstancedMesh!==!0)for(let a=0,o=r.count;a<o;a++)t.isMesh===!0?t.getVertexPosition(a,In):In.fromBufferAttribute(r,a),In.applyMatrix4(t.matrixWorld),this.expandByPoint(In);else t.boundingBox!==void 0?(t.boundingBox===null&&t.computeBoundingBox(),Gr.copy(t.boundingBox)):(i.boundingBox===null&&i.computeBoundingBox(),Gr.copy(i.boundingBox)),Gr.applyMatrix4(t.matrixWorld),this.union(Gr)}const s=t.children;for(let r=0,a=s.length;r<a;r++)this.expandByObject(s[r],e);return this}containsPoint(t){return t.x>=this.min.x&&t.x<=this.max.x&&t.y>=this.min.y&&t.y<=this.max.y&&t.z>=this.min.z&&t.z<=this.max.z}containsBox(t){return this.min.x<=t.min.x&&t.max.x<=this.max.x&&this.min.y<=t.min.y&&t.max.y<=this.max.y&&this.min.z<=t.min.z&&t.max.z<=this.max.z}getParameter(t,e){return e.set((t.x-this.min.x)/(this.max.x-this.min.x),(t.y-this.min.y)/(this.max.y-this.min.y),(t.z-this.min.z)/(this.max.z-this.min.z))}intersectsBox(t){return t.max.x>=this.min.x&&t.min.x<=this.max.x&&t.max.y>=this.min.y&&t.min.y<=this.max.y&&t.max.z>=this.min.z&&t.min.z<=this.max.z}intersectsSphere(t){return this.clampPoint(t.center,In),In.distanceToSquared(t.center)<=t.radius*t.radius}intersectsPlane(t){let e,i;return t.normal.x>0?(e=t.normal.x*this.min.x,i=t.normal.x*this.max.x):(e=t.normal.x*this.max.x,i=t.normal.x*this.min.x),t.normal.y>0?(e+=t.normal.y*this.min.y,i+=t.normal.y*this.max.y):(e+=t.normal.y*this.max.y,i+=t.normal.y*this.min.y),t.normal.z>0?(e+=t.normal.z*this.min.z,i+=t.normal.z*this.max.z):(e+=t.normal.z*this.max.z,i+=t.normal.z*this.min.z),e<=-t.constant&&i>=-t.constant}intersectsTriangle(t){if(this.isEmpty())return!1;this.getCenter(ur),Wr.subVectors(this.max,ur),xs.subVectors(t.a,ur),bs.subVectors(t.b,ur),ys.subVectors(t.c,ur),pi.subVectors(bs,xs),mi.subVectors(ys,bs),zi.subVectors(xs,ys);let e=[0,-pi.z,pi.y,0,-mi.z,mi.y,0,-zi.z,zi.y,pi.z,0,-pi.x,mi.z,0,-mi.x,zi.z,0,-zi.x,-pi.y,pi.x,0,-mi.y,mi.x,0,-zi.y,zi.x,0];return!bo(e,xs,bs,ys,Wr)||(e=[1,0,0,0,1,0,0,0,1],!bo(e,xs,bs,ys,Wr))?!1:($r.crossVectors(pi,mi),e=[$r.x,$r.y,$r.z],bo(e,xs,bs,ys,Wr))}clampPoint(t,e){return e.copy(t).clamp(this.min,this.max)}distanceToPoint(t){return this.clampPoint(t,In).distanceTo(t)}getBoundingSphere(t){return this.isEmpty()?t.makeEmpty():(this.getCenter(t.center),t.radius=this.getSize(In).length()*.5),t}intersect(t){return this.min.max(t.min),this.max.min(t.max),this.isEmpty()&&this.makeEmpty(),this}union(t){return this.min.min(t.min),this.max.max(t.max),this}applyMatrix4(t){return this.isEmpty()?this:(Qn[0].set(this.min.x,this.min.y,this.min.z).applyMatrix4(t),Qn[1].set(this.min.x,this.min.y,this.max.z).applyMatrix4(t),Qn[2].set(this.min.x,this.max.y,this.min.z).applyMatrix4(t),Qn[3].set(this.min.x,this.max.y,this.max.z).applyMatrix4(t),Qn[4].set(this.max.x,this.min.y,this.min.z).applyMatrix4(t),Qn[5].set(this.max.x,this.min.y,this.max.z).applyMatrix4(t),Qn[6].set(this.max.x,this.max.y,this.min.z).applyMatrix4(t),Qn[7].set(this.max.x,this.max.y,this.max.z).applyMatrix4(t),this.setFromPoints(Qn),this)}translate(t){return this.min.add(t),this.max.add(t),this}equals(t){return t.min.equals(this.min)&&t.max.equals(this.max)}}const Qn=[new O,new O,new O,new O,new O,new O,new O,new O],In=new O,Gr=new Li,xs=new O,bs=new O,ys=new O,pi=new O,mi=new O,zi=new O,ur=new O,Wr=new O,$r=new O,Vi=new O;function bo(n,t,e,i,s){for(let r=0,a=n.length-3;r<=a;r+=3){Vi.fromArray(n,r);const o=s.x*Math.abs(Vi.x)+s.y*Math.abs(Vi.y)+s.z*Math.abs(Vi.z),l=t.dot(Vi),c=e.dot(Vi),u=i.dot(Vi);if(Math.max(-Math.max(l,c,u),Math.min(l,c,u))>o)return!1}return!0}const bm=new Li,hr=new O,yo=new O;class as{constructor(t=new O,e=-1){this.isSphere=!0,this.center=t,this.radius=e}set(t,e){return this.center.copy(t),this.radius=e,this}setFromPoints(t,e){const i=this.center;e!==void 0?i.copy(e):bm.setFromPoints(t).getCenter(i);let s=0;for(let r=0,a=t.length;r<a;r++)s=Math.max(s,i.distanceToSquared(t[r]));return this.radius=Math.sqrt(s),this}copy(t){return this.center.copy(t.center),this.radius=t.radius,this}isEmpty(){return this.radius<0}makeEmpty(){return this.center.set(0,0,0),this.radius=-1,this}containsPoint(t){return t.distanceToSquared(this.center)<=this.radius*this.radius}distanceToPoint(t){return t.distanceTo(this.center)-this.radius}intersectsSphere(t){const e=this.radius+t.radius;return t.center.distanceToSquared(this.center)<=e*e}intersectsBox(t){return t.intersectsSphere(this)}intersectsPlane(t){return Math.abs(t.distanceToPoint(this.center))<=this.radius}clampPoint(t,e){const i=this.center.distanceToSquared(t);return e.copy(t),i>this.radius*this.radius&&(e.sub(this.center).normalize(),e.multiplyScalar(this.radius).add(this.center)),e}getBoundingBox(t){return this.isEmpty()?(t.makeEmpty(),t):(t.set(this.center,this.center),t.expandByScalar(this.radius),t)}applyMatrix4(t){return this.center.applyMatrix4(t),this.radius=this.radius*t.getMaxScaleOnAxis(),this}translate(t){return this.center.add(t),this}expandByPoint(t){if(this.isEmpty())return this.center.copy(t),this.radius=0,this;hr.subVectors(t,this.center);const e=hr.lengthSq();if(e>this.radius*this.radius){const i=Math.sqrt(e),s=(i-this.radius)*.5;this.center.addScaledVector(hr,s/i),this.radius+=s}return this}union(t){return t.isEmpty()?this:this.isEmpty()?(this.copy(t),this):(this.center.equals(t.center)===!0?this.radius=Math.max(this.radius,t.radius):(yo.subVectors(t.center,this.center).setLength(t.radius),this.expandByPoint(hr.copy(t.center).add(yo)),this.expandByPoint(hr.copy(t.center).sub(yo))),this)}equals(t){return t.center.equals(this.center)&&t.radius===this.radius}clone(){return new this.constructor().copy(this)}}const ti=new O,Mo=new O,jr=new O,gi=new O,So=new O,Xr=new O,Eo=new O;class Pr{constructor(t=new O,e=new O(0,0,-1)){this.origin=t,this.direction=e}set(t,e){return this.origin.copy(t),this.direction.copy(e),this}copy(t){return this.origin.copy(t.origin),this.direction.copy(t.direction),this}at(t,e){return e.copy(this.origin).addScaledVector(this.direction,t)}lookAt(t){return this.direction.copy(t).sub(this.origin).normalize(),this}recast(t){return this.origin.copy(this.at(t,ti)),this}closestPointToPoint(t,e){e.subVectors(t,this.origin);const i=e.dot(this.direction);return i<0?e.copy(this.origin):e.copy(this.origin).addScaledVector(this.direction,i)}distanceToPoint(t){return Math.sqrt(this.distanceSqToPoint(t))}distanceSqToPoint(t){const e=ti.subVectors(t,this.origin).dot(this.direction);return e<0?this.origin.distanceToSquared(t):(ti.copy(this.origin).addScaledVector(this.direction,e),ti.distanceToSquared(t))}distanceSqToSegment(t,e,i,s){Mo.copy(t).add(e).multiplyScalar(.5),jr.copy(e).sub(t).normalize(),gi.copy(this.origin).sub(Mo);const r=t.distanceTo(e)*.5,a=-this.direction.dot(jr),o=gi.dot(this.direction),l=-gi.dot(jr),c=gi.lengthSq(),u=Math.abs(1-a*a);let h,d,p,g;if(u>0)if(h=a*l-o,d=a*o-l,g=r*u,h>=0)if(d>=-g)if(d<=g){const _=1/u;h*=_,d*=_,p=h*(h+a*d+2*o)+d*(a*h+d+2*l)+c}else d=r,h=Math.max(0,-(a*d+o)),p=-h*h+d*(d+2*l)+c;else d=-r,h=Math.max(0,-(a*d+o)),p=-h*h+d*(d+2*l)+c;else d<=-g?(h=Math.max(0,-(-a*r+o)),d=h>0?-r:Math.min(Math.max(-r,-l),r),p=-h*h+d*(d+2*l)+c):d<=g?(h=0,d=Math.min(Math.max(-r,-l),r),p=d*(d+2*l)+c):(h=Math.max(0,-(a*r+o)),d=h>0?r:Math.min(Math.max(-r,-l),r),p=-h*h+d*(d+2*l)+c);else d=a>0?-r:r,h=Math.max(0,-(a*d+o)),p=-h*h+d*(d+2*l)+c;return i&&i.copy(this.origin).addScaledVector(this.direction,h),s&&s.copy(Mo).addScaledVector(jr,d),p}intersectSphere(t,e){ti.subVectors(t.center,this.origin);const i=ti.dot(this.direction),s=ti.dot(ti)-i*i,r=t.radius*t.radius;if(s>r)return null;const a=Math.sqrt(r-s),o=i-a,l=i+a;return l<0?null:o<0?this.at(l,e):this.at(o,e)}intersectsSphere(t){return this.distanceSqToPoint(t.center)<=t.radius*t.radius}distanceToPlane(t){const e=t.normal.dot(this.direction);if(e===0)return t.distanceToPoint(this.origin)===0?0:null;const i=-(this.origin.dot(t.normal)+t.constant)/e;return i>=0?i:null}intersectPlane(t,e){const i=this.distanceToPlane(t);return i===null?null:this.at(i,e)}intersectsPlane(t){const e=t.distanceToPoint(this.origin);return e===0||t.normal.dot(this.direction)*e<0}intersectBox(t,e){let i,s,r,a,o,l;const c=1/this.direction.x,u=1/this.direction.y,h=1/this.direction.z,d=this.origin;return c>=0?(i=(t.min.x-d.x)*c,s=(t.max.x-d.x)*c):(i=(t.max.x-d.x)*c,s=(t.min.x-d.x)*c),u>=0?(r=(t.min.y-d.y)*u,a=(t.max.y-d.y)*u):(r=(t.max.y-d.y)*u,a=(t.min.y-d.y)*u),i>a||r>s||((r>i||isNaN(i))&&(i=r),(a<s||isNaN(s))&&(s=a),h>=0?(o=(t.min.z-d.z)*h,l=(t.max.z-d.z)*h):(o=(t.max.z-d.z)*h,l=(t.min.z-d.z)*h),i>l||o>s)||((o>i||i!==i)&&(i=o),(l<s||s!==s)&&(s=l),s<0)?null:this.at(i>=0?i:s,e)}intersectsBox(t){return this.intersectBox(t,ti)!==null}intersectTriangle(t,e,i,s,r){So.subVectors(e,t),Xr.subVectors(i,t),Eo.crossVectors(So,Xr);let a=this.direction.dot(Eo),o;if(a>0){if(s)return null;o=1}else if(a<0)o=-1,a=-a;else return null;gi.subVectors(this.origin,t);const l=o*this.direction.dot(Xr.crossVectors(gi,Xr));if(l<0)return null;const c=o*this.direction.dot(So.cross(gi));if(c<0||l+c>a)return null;const u=-o*gi.dot(Eo);return u<0?null:this.at(u/a,r)}applyMatrix4(t){return this.origin.applyMatrix4(t),this.direction.transformDirection(t),this}equals(t){return t.origin.equals(this.origin)&&t.direction.equals(this.direction)}clone(){return new this.constructor().copy(this)}}class Kt{constructor(t,e,i,s,r,a,o,l,c,u,h,d,p,g,_,m){Kt.prototype.isMatrix4=!0,this.elements=[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1],t!==void 0&&this.set(t,e,i,s,r,a,o,l,c,u,h,d,p,g,_,m)}set(t,e,i,s,r,a,o,l,c,u,h,d,p,g,_,m){const f=this.elements;return f[0]=t,f[4]=e,f[8]=i,f[12]=s,f[1]=r,f[5]=a,f[9]=o,f[13]=l,f[2]=c,f[6]=u,f[10]=h,f[14]=d,f[3]=p,f[7]=g,f[11]=_,f[15]=m,this}identity(){return this.set(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1),this}clone(){return new Kt().fromArray(this.elements)}copy(t){const e=this.elements,i=t.elements;return e[0]=i[0],e[1]=i[1],e[2]=i[2],e[3]=i[3],e[4]=i[4],e[5]=i[5],e[6]=i[6],e[7]=i[7],e[8]=i[8],e[9]=i[9],e[10]=i[10],e[11]=i[11],e[12]=i[12],e[13]=i[13],e[14]=i[14],e[15]=i[15],this}copyPosition(t){const e=this.elements,i=t.elements;return e[12]=i[12],e[13]=i[13],e[14]=i[14],this}setFromMatrix3(t){const e=t.elements;return this.set(e[0],e[3],e[6],0,e[1],e[4],e[7],0,e[2],e[5],e[8],0,0,0,0,1),this}extractBasis(t,e,i){return t.setFromMatrixColumn(this,0),e.setFromMatrixColumn(this,1),i.setFromMatrixColumn(this,2),this}makeBasis(t,e,i){return this.set(t.x,e.x,i.x,0,t.y,e.y,i.y,0,t.z,e.z,i.z,0,0,0,0,1),this}extractRotation(t){const e=this.elements,i=t.elements,s=1/Ms.setFromMatrixColumn(t,0).length(),r=1/Ms.setFromMatrixColumn(t,1).length(),a=1/Ms.setFromMatrixColumn(t,2).length();return e[0]=i[0]*s,e[1]=i[1]*s,e[2]=i[2]*s,e[3]=0,e[4]=i[4]*r,e[5]=i[5]*r,e[6]=i[6]*r,e[7]=0,e[8]=i[8]*a,e[9]=i[9]*a,e[10]=i[10]*a,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromEuler(t){const e=this.elements,i=t.x,s=t.y,r=t.z,a=Math.cos(i),o=Math.sin(i),l=Math.cos(s),c=Math.sin(s),u=Math.cos(r),h=Math.sin(r);if(t.order==="XYZ"){const d=a*u,p=a*h,g=o*u,_=o*h;e[0]=l*u,e[4]=-l*h,e[8]=c,e[1]=p+g*c,e[5]=d-_*c,e[9]=-o*l,e[2]=_-d*c,e[6]=g+p*c,e[10]=a*l}else if(t.order==="YXZ"){const d=l*u,p=l*h,g=c*u,_=c*h;e[0]=d+_*o,e[4]=g*o-p,e[8]=a*c,e[1]=a*h,e[5]=a*u,e[9]=-o,e[2]=p*o-g,e[6]=_+d*o,e[10]=a*l}else if(t.order==="ZXY"){const d=l*u,p=l*h,g=c*u,_=c*h;e[0]=d-_*o,e[4]=-a*h,e[8]=g+p*o,e[1]=p+g*o,e[5]=a*u,e[9]=_-d*o,e[2]=-a*c,e[6]=o,e[10]=a*l}else if(t.order==="ZYX"){const d=a*u,p=a*h,g=o*u,_=o*h;e[0]=l*u,e[4]=g*c-p,e[8]=d*c+_,e[1]=l*h,e[5]=_*c+d,e[9]=p*c-g,e[2]=-c,e[6]=o*l,e[10]=a*l}else if(t.order==="YZX"){const d=a*l,p=a*c,g=o*l,_=o*c;e[0]=l*u,e[4]=_-d*h,e[8]=g*h+p,e[1]=h,e[5]=a*u,e[9]=-o*u,e[2]=-c*u,e[6]=p*h+g,e[10]=d-_*h}else if(t.order==="XZY"){const d=a*l,p=a*c,g=o*l,_=o*c;e[0]=l*u,e[4]=-h,e[8]=c*u,e[1]=d*h+_,e[5]=a*u,e[9]=p*h-g,e[2]=g*h-p,e[6]=o*u,e[10]=_*h+d}return e[3]=0,e[7]=0,e[11]=0,e[12]=0,e[13]=0,e[14]=0,e[15]=1,this}makeRotationFromQuaternion(t){return this.compose(ym,t,Mm)}lookAt(t,e,i){const s=this.elements;return Sn.subVectors(t,e),Sn.lengthSq()===0&&(Sn.z=1),Sn.normalize(),_i.crossVectors(i,Sn),_i.lengthSq()===0&&(Math.abs(i.z)===1?Sn.x+=1e-4:Sn.z+=1e-4,Sn.normalize(),_i.crossVectors(i,Sn)),_i.normalize(),qr.crossVectors(Sn,_i),s[0]=_i.x,s[4]=qr.x,s[8]=Sn.x,s[1]=_i.y,s[5]=qr.y,s[9]=Sn.y,s[2]=_i.z,s[6]=qr.z,s[10]=Sn.z,this}multiply(t){return this.multiplyMatrices(this,t)}premultiply(t){return this.multiplyMatrices(t,this)}multiplyMatrices(t,e){const i=t.elements,s=e.elements,r=this.elements,a=i[0],o=i[4],l=i[8],c=i[12],u=i[1],h=i[5],d=i[9],p=i[13],g=i[2],_=i[6],m=i[10],f=i[14],P=i[3],L=i[7],w=i[11],N=i[15],F=s[0],U=s[4],k=s[8],y=s[12],M=s[1],I=s[5],V=s[9],W=s[13],Z=s[2],et=s[6],z=s[10],it=s[14],j=s[3],ut=s[7],mt=s[11],gt=s[15];return r[0]=a*F+o*M+l*Z+c*j,r[4]=a*U+o*I+l*et+c*ut,r[8]=a*k+o*V+l*z+c*mt,r[12]=a*y+o*W+l*it+c*gt,r[1]=u*F+h*M+d*Z+p*j,r[5]=u*U+h*I+d*et+p*ut,r[9]=u*k+h*V+d*z+p*mt,r[13]=u*y+h*W+d*it+p*gt,r[2]=g*F+_*M+m*Z+f*j,r[6]=g*U+_*I+m*et+f*ut,r[10]=g*k+_*V+m*z+f*mt,r[14]=g*y+_*W+m*it+f*gt,r[3]=P*F+L*M+w*Z+N*j,r[7]=P*U+L*I+w*et+N*ut,r[11]=P*k+L*V+w*z+N*mt,r[15]=P*y+L*W+w*it+N*gt,this}multiplyScalar(t){const e=this.elements;return e[0]*=t,e[4]*=t,e[8]*=t,e[12]*=t,e[1]*=t,e[5]*=t,e[9]*=t,e[13]*=t,e[2]*=t,e[6]*=t,e[10]*=t,e[14]*=t,e[3]*=t,e[7]*=t,e[11]*=t,e[15]*=t,this}determinant(){const t=this.elements,e=t[0],i=t[4],s=t[8],r=t[12],a=t[1],o=t[5],l=t[9],c=t[13],u=t[2],h=t[6],d=t[10],p=t[14],g=t[3],_=t[7],m=t[11],f=t[15];return g*(+r*l*h-s*c*h-r*o*d+i*c*d+s*o*p-i*l*p)+_*(+e*l*p-e*c*d+r*a*d-s*a*p+s*c*u-r*l*u)+m*(+e*c*h-e*o*p-r*a*h+i*a*p+r*o*u-i*c*u)+f*(-s*o*u-e*l*h+e*o*d+s*a*h-i*a*d+i*l*u)}transpose(){const t=this.elements;let e;return e=t[1],t[1]=t[4],t[4]=e,e=t[2],t[2]=t[8],t[8]=e,e=t[6],t[6]=t[9],t[9]=e,e=t[3],t[3]=t[12],t[12]=e,e=t[7],t[7]=t[13],t[13]=e,e=t[11],t[11]=t[14],t[14]=e,this}setPosition(t,e,i){const s=this.elements;return t.isVector3?(s[12]=t.x,s[13]=t.y,s[14]=t.z):(s[12]=t,s[13]=e,s[14]=i),this}invert(){const t=this.elements,e=t[0],i=t[1],s=t[2],r=t[3],a=t[4],o=t[5],l=t[6],c=t[7],u=t[8],h=t[9],d=t[10],p=t[11],g=t[12],_=t[13],m=t[14],f=t[15],P=h*m*c-_*d*c+_*l*p-o*m*p-h*l*f+o*d*f,L=g*d*c-u*m*c-g*l*p+a*m*p+u*l*f-a*d*f,w=u*_*c-g*h*c+g*o*p-a*_*p-u*o*f+a*h*f,N=g*h*l-u*_*l-g*o*d+a*_*d+u*o*m-a*h*m,F=e*P+i*L+s*w+r*N;if(F===0)return this.set(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0);const U=1/F;return t[0]=P*U,t[1]=(_*d*r-h*m*r-_*s*p+i*m*p+h*s*f-i*d*f)*U,t[2]=(o*m*r-_*l*r+_*s*c-i*m*c-o*s*f+i*l*f)*U,t[3]=(h*l*r-o*d*r-h*s*c+i*d*c+o*s*p-i*l*p)*U,t[4]=L*U,t[5]=(u*m*r-g*d*r+g*s*p-e*m*p-u*s*f+e*d*f)*U,t[6]=(g*l*r-a*m*r-g*s*c+e*m*c+a*s*f-e*l*f)*U,t[7]=(a*d*r-u*l*r+u*s*c-e*d*c-a*s*p+e*l*p)*U,t[8]=w*U,t[9]=(g*h*r-u*_*r-g*i*p+e*_*p+u*i*f-e*h*f)*U,t[10]=(a*_*r-g*o*r+g*i*c-e*_*c-a*i*f+e*o*f)*U,t[11]=(u*o*r-a*h*r-u*i*c+e*h*c+a*i*p-e*o*p)*U,t[12]=N*U,t[13]=(u*_*s-g*h*s+g*i*d-e*_*d-u*i*m+e*h*m)*U,t[14]=(g*o*s-a*_*s-g*i*l+e*_*l+a*i*m-e*o*m)*U,t[15]=(a*h*s-u*o*s+u*i*l-e*h*l-a*i*d+e*o*d)*U,this}scale(t){const e=this.elements,i=t.x,s=t.y,r=t.z;return e[0]*=i,e[4]*=s,e[8]*=r,e[1]*=i,e[5]*=s,e[9]*=r,e[2]*=i,e[6]*=s,e[10]*=r,e[3]*=i,e[7]*=s,e[11]*=r,this}getMaxScaleOnAxis(){const t=this.elements,e=t[0]*t[0]+t[1]*t[1]+t[2]*t[2],i=t[4]*t[4]+t[5]*t[5]+t[6]*t[6],s=t[8]*t[8]+t[9]*t[9]+t[10]*t[10];return Math.sqrt(Math.max(e,i,s))}makeTranslation(t,e,i){return t.isVector3?this.set(1,0,0,t.x,0,1,0,t.y,0,0,1,t.z,0,0,0,1):this.set(1,0,0,t,0,1,0,e,0,0,1,i,0,0,0,1),this}makeRotationX(t){const e=Math.cos(t),i=Math.sin(t);return this.set(1,0,0,0,0,e,-i,0,0,i,e,0,0,0,0,1),this}makeRotationY(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,0,i,0,0,1,0,0,-i,0,e,0,0,0,0,1),this}makeRotationZ(t){const e=Math.cos(t),i=Math.sin(t);return this.set(e,-i,0,0,i,e,0,0,0,0,1,0,0,0,0,1),this}makeRotationAxis(t,e){const i=Math.cos(e),s=Math.sin(e),r=1-i,a=t.x,o=t.y,l=t.z,c=r*a,u=r*o;return this.set(c*a+i,c*o-s*l,c*l+s*o,0,c*o+s*l,u*o+i,u*l-s*a,0,c*l-s*o,u*l+s*a,r*l*l+i,0,0,0,0,1),this}makeScale(t,e,i){return this.set(t,0,0,0,0,e,0,0,0,0,i,0,0,0,0,1),this}makeShear(t,e,i,s,r,a){return this.set(1,i,r,0,t,1,a,0,e,s,1,0,0,0,0,1),this}compose(t,e,i){const s=this.elements,r=e._x,a=e._y,o=e._z,l=e._w,c=r+r,u=a+a,h=o+o,d=r*c,p=r*u,g=r*h,_=a*u,m=a*h,f=o*h,P=l*c,L=l*u,w=l*h,N=i.x,F=i.y,U=i.z;return s[0]=(1-(_+f))*N,s[1]=(p+w)*N,s[2]=(g-L)*N,s[3]=0,s[4]=(p-w)*F,s[5]=(1-(d+f))*F,s[6]=(m+P)*F,s[7]=0,s[8]=(g+L)*U,s[9]=(m-P)*U,s[10]=(1-(d+_))*U,s[11]=0,s[12]=t.x,s[13]=t.y,s[14]=t.z,s[15]=1,this}decompose(t,e,i){const s=this.elements;let r=Ms.set(s[0],s[1],s[2]).length();const a=Ms.set(s[4],s[5],s[6]).length(),o=Ms.set(s[8],s[9],s[10]).length();this.determinant()<0&&(r=-r),t.x=s[12],t.y=s[13],t.z=s[14],Un.copy(this);const c=1/r,u=1/a,h=1/o;return Un.elements[0]*=c,Un.elements[1]*=c,Un.elements[2]*=c,Un.elements[4]*=u,Un.elements[5]*=u,Un.elements[6]*=u,Un.elements[8]*=h,Un.elements[9]*=h,Un.elements[10]*=h,e.setFromRotationMatrix(Un),i.x=r,i.y=a,i.z=o,this}makePerspective(t,e,i,s,r,a,o=li){const l=this.elements,c=2*r/(e-t),u=2*r/(i-s),h=(e+t)/(e-t),d=(i+s)/(i-s);let p,g;if(o===li)p=-(a+r)/(a-r),g=-2*a*r/(a-r);else if(o===Oa)p=-a/(a-r),g=-a*r/(a-r);else throw new Error("THREE.Matrix4.makePerspective(): Invalid coordinate system: "+o);return l[0]=c,l[4]=0,l[8]=h,l[12]=0,l[1]=0,l[5]=u,l[9]=d,l[13]=0,l[2]=0,l[6]=0,l[10]=p,l[14]=g,l[3]=0,l[7]=0,l[11]=-1,l[15]=0,this}makeOrthographic(t,e,i,s,r,a,o=li){const l=this.elements,c=1/(e-t),u=1/(i-s),h=1/(a-r),d=(e+t)*c,p=(i+s)*u;let g,_;if(o===li)g=(a+r)*h,_=-2*h;else if(o===Oa)g=r*h,_=-1*h;else throw new Error("THREE.Matrix4.makeOrthographic(): Invalid coordinate system: "+o);return l[0]=2*c,l[4]=0,l[8]=0,l[12]=-d,l[1]=0,l[5]=2*u,l[9]=0,l[13]=-p,l[2]=0,l[6]=0,l[10]=_,l[14]=-g,l[3]=0,l[7]=0,l[11]=0,l[15]=1,this}equals(t){const e=this.elements,i=t.elements;for(let s=0;s<16;s++)if(e[s]!==i[s])return!1;return!0}fromArray(t,e=0){for(let i=0;i<16;i++)this.elements[i]=t[i+e];return this}toArray(t=[],e=0){const i=this.elements;return t[e]=i[0],t[e+1]=i[1],t[e+2]=i[2],t[e+3]=i[3],t[e+4]=i[4],t[e+5]=i[5],t[e+6]=i[6],t[e+7]=i[7],t[e+8]=i[8],t[e+9]=i[9],t[e+10]=i[10],t[e+11]=i[11],t[e+12]=i[12],t[e+13]=i[13],t[e+14]=i[14],t[e+15]=i[15],t}}const Ms=new O,Un=new Kt,ym=new O(0,0,0),Mm=new O(1,1,1),_i=new O,qr=new O,Sn=new O,ou=new Kt,lu=new Nn;class yn{constructor(t=0,e=0,i=0,s=yn.DEFAULT_ORDER){this.isEuler=!0,this._x=t,this._y=e,this._z=i,this._order=s}get x(){return this._x}set x(t){this._x=t,this._onChangeCallback()}get y(){return this._y}set y(t){this._y=t,this._onChangeCallback()}get z(){return this._z}set z(t){this._z=t,this._onChangeCallback()}get order(){return this._order}set order(t){this._order=t,this._onChangeCallback()}set(t,e,i,s=this._order){return this._x=t,this._y=e,this._z=i,this._order=s,this._onChangeCallback(),this}clone(){return new this.constructor(this._x,this._y,this._z,this._order)}copy(t){return this._x=t._x,this._y=t._y,this._z=t._z,this._order=t._order,this._onChangeCallback(),this}setFromRotationMatrix(t,e=this._order,i=!0){const s=t.elements,r=s[0],a=s[4],o=s[8],l=s[1],c=s[5],u=s[9],h=s[2],d=s[6],p=s[10];switch(e){case"XYZ":this._y=Math.asin(he(o,-1,1)),Math.abs(o)<.9999999?(this._x=Math.atan2(-u,p),this._z=Math.atan2(-a,r)):(this._x=Math.atan2(d,c),this._z=0);break;case"YXZ":this._x=Math.asin(-he(u,-1,1)),Math.abs(u)<.9999999?(this._y=Math.atan2(o,p),this._z=Math.atan2(l,c)):(this._y=Math.atan2(-h,r),this._z=0);break;case"ZXY":this._x=Math.asin(he(d,-1,1)),Math.abs(d)<.9999999?(this._y=Math.atan2(-h,p),this._z=Math.atan2(-a,c)):(this._y=0,this._z=Math.atan2(l,r));break;case"ZYX":this._y=Math.asin(-he(h,-1,1)),Math.abs(h)<.9999999?(this._x=Math.atan2(d,p),this._z=Math.atan2(l,r)):(this._x=0,this._z=Math.atan2(-a,c));break;case"YZX":this._z=Math.asin(he(l,-1,1)),Math.abs(l)<.9999999?(this._x=Math.atan2(-u,c),this._y=Math.atan2(-h,r)):(this._x=0,this._y=Math.atan2(o,p));break;case"XZY":this._z=Math.asin(-he(a,-1,1)),Math.abs(a)<.9999999?(this._x=Math.atan2(d,c),this._y=Math.atan2(o,r)):(this._x=Math.atan2(-u,p),this._y=0);break;default:console.warn("THREE.Euler: .setFromRotationMatrix() encountered an unknown order: "+e)}return this._order=e,i===!0&&this._onChangeCallback(),this}setFromQuaternion(t,e,i){return ou.makeRotationFromQuaternion(t),this.setFromRotationMatrix(ou,e,i)}setFromVector3(t,e=this._order){return this.set(t.x,t.y,t.z,e)}reorder(t){return lu.setFromEuler(this),this.setFromQuaternion(lu,t)}equals(t){return t._x===this._x&&t._y===this._y&&t._z===this._z&&t._order===this._order}fromArray(t){return this._x=t[0],this._y=t[1],this._z=t[2],t[3]!==void 0&&(this._order=t[3]),this._onChangeCallback(),this}toArray(t=[],e=0){return t[e]=this._x,t[e+1]=this._y,t[e+2]=this._z,t[e+3]=this._order,t}_onChange(t){return this._onChangeCallback=t,this}_onChangeCallback(){}*[Symbol.iterator](){yield this._x,yield this._y,yield this._z,yield this._order}}yn.DEFAULT_ORDER="XYZ";class rd{constructor(){this.mask=1}set(t){this.mask=(1<<t|0)>>>0}enable(t){this.mask|=1<<t|0}enableAll(){this.mask=-1}toggle(t){this.mask^=1<<t|0}disable(t){this.mask&=~(1<<t|0)}disableAll(){this.mask=0}test(t){return(this.mask&t.mask)!==0}isEnabled(t){return(this.mask&(1<<t|0))!==0}}let Sm=0;const cu=new O,Ss=new Nn,ei=new Kt,Yr=new O,dr=new O,Em=new O,Tm=new Nn,uu=new O(1,0,0),hu=new O(0,1,0),du=new O(0,0,1),fu={type:"added"},wm={type:"removed"},Es={type:"childadded",child:null},To={type:"childremoved",child:null};class Ve extends rs{constructor(){super(),this.isObject3D=!0,Object.defineProperty(this,"id",{value:Sm++}),this.uuid=Ui(),this.name="",this.type="Object3D",this.parent=null,this.children=[],this.up=Ve.DEFAULT_UP.clone();const t=new O,e=new yn,i=new Nn,s=new O(1,1,1);function r(){i.setFromEuler(e,!1)}function a(){e.setFromQuaternion(i,void 0,!1)}e._onChange(r),i._onChange(a),Object.defineProperties(this,{position:{configurable:!0,enumerable:!0,value:t},rotation:{configurable:!0,enumerable:!0,value:e},quaternion:{configurable:!0,enumerable:!0,value:i},scale:{configurable:!0,enumerable:!0,value:s},modelViewMatrix:{value:new Kt},normalMatrix:{value:new le}}),this.matrix=new Kt,this.matrixWorld=new Kt,this.matrixAutoUpdate=Ve.DEFAULT_MATRIX_AUTO_UPDATE,this.matrixWorldAutoUpdate=Ve.DEFAULT_MATRIX_WORLD_AUTO_UPDATE,this.matrixWorldNeedsUpdate=!1,this.layers=new rd,this.visible=!0,this.castShadow=!1,this.receiveShadow=!1,this.frustumCulled=!0,this.renderOrder=0,this.animations=[],this.userData={}}onBeforeShadow(){}onAfterShadow(){}onBeforeRender(){}onAfterRender(){}applyMatrix4(t){this.matrixAutoUpdate&&this.updateMatrix(),this.matrix.premultiply(t),this.matrix.decompose(this.position,this.quaternion,this.scale)}applyQuaternion(t){return this.quaternion.premultiply(t),this}setRotationFromAxisAngle(t,e){this.quaternion.setFromAxisAngle(t,e)}setRotationFromEuler(t){this.quaternion.setFromEuler(t,!0)}setRotationFromMatrix(t){this.quaternion.setFromRotationMatrix(t)}setRotationFromQuaternion(t){this.quaternion.copy(t)}rotateOnAxis(t,e){return Ss.setFromAxisAngle(t,e),this.quaternion.multiply(Ss),this}rotateOnWorldAxis(t,e){return Ss.setFromAxisAngle(t,e),this.quaternion.premultiply(Ss),this}rotateX(t){return this.rotateOnAxis(uu,t)}rotateY(t){return this.rotateOnAxis(hu,t)}rotateZ(t){return this.rotateOnAxis(du,t)}translateOnAxis(t,e){return cu.copy(t).applyQuaternion(this.quaternion),this.position.add(cu.multiplyScalar(e)),this}translateX(t){return this.translateOnAxis(uu,t)}translateY(t){return this.translateOnAxis(hu,t)}translateZ(t){return this.translateOnAxis(du,t)}localToWorld(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(this.matrixWorld)}worldToLocal(t){return this.updateWorldMatrix(!0,!1),t.applyMatrix4(ei.copy(this.matrixWorld).invert())}lookAt(t,e,i){t.isVector3?Yr.copy(t):Yr.set(t,e,i);const s=this.parent;this.updateWorldMatrix(!0,!1),dr.setFromMatrixPosition(this.matrixWorld),this.isCamera||this.isLight?ei.lookAt(dr,Yr,this.up):ei.lookAt(Yr,dr,this.up),this.quaternion.setFromRotationMatrix(ei),s&&(ei.extractRotation(s.matrixWorld),Ss.setFromRotationMatrix(ei),this.quaternion.premultiply(Ss.invert()))}add(t){if(arguments.length>1){for(let e=0;e<arguments.length;e++)this.add(arguments[e]);return this}return t===this?(console.error("THREE.Object3D.add: object can't be added as a child of itself.",t),this):(t&&t.isObject3D?(t.removeFromParent(),t.parent=this,this.children.push(t),t.dispatchEvent(fu),Es.child=t,this.dispatchEvent(Es),Es.child=null):console.error("THREE.Object3D.add: object not an instance of THREE.Object3D.",t),this)}remove(t){if(arguments.length>1){for(let i=0;i<arguments.length;i++)this.remove(arguments[i]);return this}const e=this.children.indexOf(t);return e!==-1&&(t.parent=null,this.children.splice(e,1),t.dispatchEvent(wm),To.child=t,this.dispatchEvent(To),To.child=null),this}removeFromParent(){const t=this.parent;return t!==null&&t.remove(this),this}clear(){return this.remove(...this.children)}attach(t){return this.updateWorldMatrix(!0,!1),ei.copy(this.matrixWorld).invert(),t.parent!==null&&(t.parent.updateWorldMatrix(!0,!1),ei.multiply(t.parent.matrixWorld)),t.applyMatrix4(ei),t.removeFromParent(),t.parent=this,this.children.push(t),t.updateWorldMatrix(!1,!0),t.dispatchEvent(fu),Es.child=t,this.dispatchEvent(Es),Es.child=null,this}getObjectById(t){return this.getObjectByProperty("id",t)}getObjectByName(t){return this.getObjectByProperty("name",t)}getObjectByProperty(t,e){if(this[t]===e)return this;for(let i=0,s=this.children.length;i<s;i++){const a=this.children[i].getObjectByProperty(t,e);if(a!==void 0)return a}}getObjectsByProperty(t,e,i=[]){this[t]===e&&i.push(this);const s=this.children;for(let r=0,a=s.length;r<a;r++)s[r].getObjectsByProperty(t,e,i);return i}getWorldPosition(t){return this.updateWorldMatrix(!0,!1),t.setFromMatrixPosition(this.matrixWorld)}getWorldQuaternion(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(dr,t,Em),t}getWorldScale(t){return this.updateWorldMatrix(!0,!1),this.matrixWorld.decompose(dr,Tm,t),t}getWorldDirection(t){this.updateWorldMatrix(!0,!1);const e=this.matrixWorld.elements;return t.set(e[8],e[9],e[10]).normalize()}raycast(){}traverse(t){t(this);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].traverse(t)}traverseVisible(t){if(this.visible===!1)return;t(this);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].traverseVisible(t)}traverseAncestors(t){const e=this.parent;e!==null&&(t(e),e.traverseAncestors(t))}updateMatrix(){this.matrix.compose(this.position,this.quaternion,this.scale),this.matrixWorldNeedsUpdate=!0}updateMatrixWorld(t){this.matrixAutoUpdate&&this.updateMatrix(),(this.matrixWorldNeedsUpdate||t)&&(this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),this.matrixWorldNeedsUpdate=!1,t=!0);const e=this.children;for(let i=0,s=e.length;i<s;i++)e[i].updateMatrixWorld(t)}updateWorldMatrix(t,e){const i=this.parent;if(t===!0&&i!==null&&i.updateWorldMatrix(!0,!1),this.matrixAutoUpdate&&this.updateMatrix(),this.matrixWorldAutoUpdate===!0&&(this.parent===null?this.matrixWorld.copy(this.matrix):this.matrixWorld.multiplyMatrices(this.parent.matrixWorld,this.matrix)),e===!0){const s=this.children;for(let r=0,a=s.length;r<a;r++)s[r].updateWorldMatrix(!1,!0)}}toJSON(t){const e=t===void 0||typeof t=="string",i={};e&&(t={geometries:{},materials:{},textures:{},images:{},shapes:{},skeletons:{},animations:{},nodes:{}},i.metadata={version:4.6,type:"Object",generator:"Object3D.toJSON"});const s={};s.uuid=this.uuid,s.type=this.type,this.name!==""&&(s.name=this.name),this.castShadow===!0&&(s.castShadow=!0),this.receiveShadow===!0&&(s.receiveShadow=!0),this.visible===!1&&(s.visible=!1),this.frustumCulled===!1&&(s.frustumCulled=!1),this.renderOrder!==0&&(s.renderOrder=this.renderOrder),Object.keys(this.userData).length>0&&(s.userData=this.userData),s.layers=this.layers.mask,s.matrix=this.matrix.toArray(),s.up=this.up.toArray(),this.matrixAutoUpdate===!1&&(s.matrixAutoUpdate=!1),this.isInstancedMesh&&(s.type="InstancedMesh",s.count=this.count,s.instanceMatrix=this.instanceMatrix.toJSON(),this.instanceColor!==null&&(s.instanceColor=this.instanceColor.toJSON())),this.isBatchedMesh&&(s.type="BatchedMesh",s.perObjectFrustumCulled=this.perObjectFrustumCulled,s.sortObjects=this.sortObjects,s.drawRanges=this._drawRanges,s.reservedRanges=this._reservedRanges,s.visibility=this._visibility,s.active=this._active,s.bounds=this._bounds.map(o=>({boxInitialized:o.boxInitialized,boxMin:o.box.min.toArray(),boxMax:o.box.max.toArray(),sphereInitialized:o.sphereInitialized,sphereRadius:o.sphere.radius,sphereCenter:o.sphere.center.toArray()})),s.maxInstanceCount=this._maxInstanceCount,s.maxVertexCount=this._maxVertexCount,s.maxIndexCount=this._maxIndexCount,s.geometryInitialized=this._geometryInitialized,s.geometryCount=this._geometryCount,s.matricesTexture=this._matricesTexture.toJSON(t),this._colorsTexture!==null&&(s.colorsTexture=this._colorsTexture.toJSON(t)),this.boundingSphere!==null&&(s.boundingSphere={center:s.boundingSphere.center.toArray(),radius:s.boundingSphere.radius}),this.boundingBox!==null&&(s.boundingBox={min:s.boundingBox.min.toArray(),max:s.boundingBox.max.toArray()}));function r(o,l){return o[l.uuid]===void 0&&(o[l.uuid]=l.toJSON(t)),l.uuid}if(this.isScene)this.background&&(this.background.isColor?s.background=this.background.toJSON():this.background.isTexture&&(s.background=this.background.toJSON(t).uuid)),this.environment&&this.environment.isTexture&&this.environment.isRenderTargetTexture!==!0&&(s.environment=this.environment.toJSON(t).uuid);else if(this.isMesh||this.isLine||this.isPoints){s.geometry=r(t.geometries,this.geometry);const o=this.geometry.parameters;if(o!==void 0&&o.shapes!==void 0){const l=o.shapes;if(Array.isArray(l))for(let c=0,u=l.length;c<u;c++){const h=l[c];r(t.shapes,h)}else r(t.shapes,l)}}if(this.isSkinnedMesh&&(s.bindMode=this.bindMode,s.bindMatrix=this.bindMatrix.toArray(),this.skeleton!==void 0&&(r(t.skeletons,this.skeleton),s.skeleton=this.skeleton.uuid)),this.material!==void 0)if(Array.isArray(this.material)){const o=[];for(let l=0,c=this.material.length;l<c;l++)o.push(r(t.materials,this.material[l]));s.material=o}else s.material=r(t.materials,this.material);if(this.children.length>0){s.children=[];for(let o=0;o<this.children.length;o++)s.children.push(this.children[o].toJSON(t).object)}if(this.animations.length>0){s.animations=[];for(let o=0;o<this.animations.length;o++){const l=this.animations[o];s.animations.push(r(t.animations,l))}}if(e){const o=a(t.geometries),l=a(t.materials),c=a(t.textures),u=a(t.images),h=a(t.shapes),d=a(t.skeletons),p=a(t.animations),g=a(t.nodes);o.length>0&&(i.geometries=o),l.length>0&&(i.materials=l),c.length>0&&(i.textures=c),u.length>0&&(i.images=u),h.length>0&&(i.shapes=h),d.length>0&&(i.skeletons=d),p.length>0&&(i.animations=p),g.length>0&&(i.nodes=g)}return i.object=s,i;function a(o){const l=[];for(const c in o){const u=o[c];delete u.metadata,l.push(u)}return l}}clone(t){return new this.constructor().copy(this,t)}copy(t,e=!0){if(this.name=t.name,this.up.copy(t.up),this.position.copy(t.position),this.rotation.order=t.rotation.order,this.quaternion.copy(t.quaternion),this.scale.copy(t.scale),this.matrix.copy(t.matrix),this.matrixWorld.copy(t.matrixWorld),this.matrixAutoUpdate=t.matrixAutoUpdate,this.matrixWorldAutoUpdate=t.matrixWorldAutoUpdate,this.matrixWorldNeedsUpdate=t.matrixWorldNeedsUpdate,this.layers.mask=t.layers.mask,this.visible=t.visible,this.castShadow=t.castShadow,this.receiveShadow=t.receiveShadow,this.frustumCulled=t.frustumCulled,this.renderOrder=t.renderOrder,this.animations=t.animations.slice(),this.userData=JSON.parse(JSON.stringify(t.userData)),e===!0)for(let i=0;i<t.children.length;i++){const s=t.children[i];this.add(s.clone())}return this}}Ve.DEFAULT_UP=new O(0,1,0);Ve.DEFAULT_MATRIX_AUTO_UPDATE=!0;Ve.DEFAULT_MATRIX_WORLD_AUTO_UPDATE=!0;const kn=new O,ni=new O,wo=new O,ii=new O,Ts=new O,ws=new O,pu=new O,Ao=new O,Co=new O,Ro=new O,Po=new Ee,Lo=new Ee,No=new Ee;class Rn{constructor(t=new O,e=new O,i=new O){this.a=t,this.b=e,this.c=i}static getNormal(t,e,i,s){s.subVectors(i,e),kn.subVectors(t,e),s.cross(kn);const r=s.lengthSq();return r>0?s.multiplyScalar(1/Math.sqrt(r)):s.set(0,0,0)}static getBarycoord(t,e,i,s,r){kn.subVectors(s,e),ni.subVectors(i,e),wo.subVectors(t,e);const a=kn.dot(kn),o=kn.dot(ni),l=kn.dot(wo),c=ni.dot(ni),u=ni.dot(wo),h=a*c-o*o;if(h===0)return r.set(0,0,0),null;const d=1/h,p=(c*l-o*u)*d,g=(a*u-o*l)*d;return r.set(1-p-g,g,p)}static containsPoint(t,e,i,s){return this.getBarycoord(t,e,i,s,ii)===null?!1:ii.x>=0&&ii.y>=0&&ii.x+ii.y<=1}static getInterpolation(t,e,i,s,r,a,o,l){return this.getBarycoord(t,e,i,s,ii)===null?(l.x=0,l.y=0,"z"in l&&(l.z=0),"w"in l&&(l.w=0),null):(l.setScalar(0),l.addScaledVector(r,ii.x),l.addScaledVector(a,ii.y),l.addScaledVector(o,ii.z),l)}static getInterpolatedAttribute(t,e,i,s,r,a){return Po.setScalar(0),Lo.setScalar(0),No.setScalar(0),Po.fromBufferAttribute(t,e),Lo.fromBufferAttribute(t,i),No.fromBufferAttribute(t,s),a.setScalar(0),a.addScaledVector(Po,r.x),a.addScaledVector(Lo,r.y),a.addScaledVector(No,r.z),a}static isFrontFacing(t,e,i,s){return kn.subVectors(i,e),ni.subVectors(t,e),kn.cross(ni).dot(s)<0}set(t,e,i){return this.a.copy(t),this.b.copy(e),this.c.copy(i),this}setFromPointsAndIndices(t,e,i,s){return this.a.copy(t[e]),this.b.copy(t[i]),this.c.copy(t[s]),this}setFromAttributeAndIndices(t,e,i,s){return this.a.fromBufferAttribute(t,e),this.b.fromBufferAttribute(t,i),this.c.fromBufferAttribute(t,s),this}clone(){return new this.constructor().copy(this)}copy(t){return this.a.copy(t.a),this.b.copy(t.b),this.c.copy(t.c),this}getArea(){return kn.subVectors(this.c,this.b),ni.subVectors(this.a,this.b),kn.cross(ni).length()*.5}getMidpoint(t){return t.addVectors(this.a,this.b).add(this.c).multiplyScalar(1/3)}getNormal(t){return Rn.getNormal(this.a,this.b,this.c,t)}getPlane(t){return t.setFromCoplanarPoints(this.a,this.b,this.c)}getBarycoord(t,e){return Rn.getBarycoord(t,this.a,this.b,this.c,e)}getInterpolation(t,e,i,s,r){return Rn.getInterpolation(t,this.a,this.b,this.c,e,i,s,r)}containsPoint(t){return Rn.containsPoint(t,this.a,this.b,this.c)}isFrontFacing(t){return Rn.isFrontFacing(this.a,this.b,this.c,t)}intersectsBox(t){return t.intersectsTriangle(this)}closestPointToPoint(t,e){const i=this.a,s=this.b,r=this.c;let a,o;Ts.subVectors(s,i),ws.subVectors(r,i),Ao.subVectors(t,i);const l=Ts.dot(Ao),c=ws.dot(Ao);if(l<=0&&c<=0)return e.copy(i);Co.subVectors(t,s);const u=Ts.dot(Co),h=ws.dot(Co);if(u>=0&&h<=u)return e.copy(s);const d=l*h-u*c;if(d<=0&&l>=0&&u<=0)return a=l/(l-u),e.copy(i).addScaledVector(Ts,a);Ro.subVectors(t,r);const p=Ts.dot(Ro),g=ws.dot(Ro);if(g>=0&&p<=g)return e.copy(r);const _=p*c-l*g;if(_<=0&&c>=0&&g<=0)return o=c/(c-g),e.copy(i).addScaledVector(ws,o);const m=u*g-p*h;if(m<=0&&h-u>=0&&p-g>=0)return pu.subVectors(r,s),o=(h-u)/(h-u+(p-g)),e.copy(s).addScaledVector(pu,o);const f=1/(m+_+d);return a=_*f,o=d*f,e.copy(i).addScaledVector(Ts,a).addScaledVector(ws,o)}equals(t){return t.a.equals(this.a)&&t.b.equals(this.b)&&t.c.equals(this.c)}}const ad={aliceblue:15792383,antiquewhite:16444375,aqua:65535,aquamarine:8388564,azure:15794175,beige:16119260,bisque:16770244,black:0,blanchedalmond:16772045,blue:255,blueviolet:9055202,brown:10824234,burlywood:14596231,cadetblue:6266528,chartreuse:8388352,chocolate:13789470,coral:16744272,cornflowerblue:6591981,cornsilk:16775388,crimson:14423100,cyan:65535,darkblue:139,darkcyan:35723,darkgoldenrod:12092939,darkgray:11119017,darkgreen:25600,darkgrey:11119017,darkkhaki:12433259,darkmagenta:9109643,darkolivegreen:5597999,darkorange:16747520,darkorchid:10040012,darkred:9109504,darksalmon:15308410,darkseagreen:9419919,darkslateblue:4734347,darkslategray:3100495,darkslategrey:3100495,darkturquoise:52945,darkviolet:9699539,deeppink:16716947,deepskyblue:49151,dimgray:6908265,dimgrey:6908265,dodgerblue:2003199,firebrick:11674146,floralwhite:16775920,forestgreen:2263842,fuchsia:16711935,gainsboro:14474460,ghostwhite:16316671,gold:16766720,goldenrod:14329120,gray:8421504,green:32768,greenyellow:11403055,grey:8421504,honeydew:15794160,hotpink:16738740,indianred:13458524,indigo:4915330,ivory:16777200,khaki:15787660,lavender:15132410,lavenderblush:16773365,lawngreen:8190976,lemonchiffon:16775885,lightblue:11393254,lightcoral:15761536,lightcyan:14745599,lightgoldenrodyellow:16448210,lightgray:13882323,lightgreen:9498256,lightgrey:13882323,lightpink:16758465,lightsalmon:16752762,lightseagreen:2142890,lightskyblue:8900346,lightslategray:7833753,lightslategrey:7833753,lightsteelblue:11584734,lightyellow:16777184,lime:65280,limegreen:3329330,linen:16445670,magenta:16711935,maroon:8388608,mediumaquamarine:6737322,mediumblue:205,mediumorchid:12211667,mediumpurple:9662683,mediumseagreen:3978097,mediumslateblue:8087790,mediumspringgreen:64154,mediumturquoise:4772300,mediumvioletred:13047173,midnightblue:1644912,mintcream:16121850,mistyrose:16770273,moccasin:16770229,navajowhite:16768685,navy:128,oldlace:16643558,olive:8421376,olivedrab:7048739,orange:16753920,orangered:16729344,orchid:14315734,palegoldenrod:15657130,palegreen:10025880,paleturquoise:11529966,palevioletred:14381203,papayawhip:16773077,peachpuff:16767673,peru:13468991,pink:16761035,plum:14524637,powderblue:11591910,purple:8388736,rebeccapurple:6697881,red:16711680,rosybrown:12357519,royalblue:4286945,saddlebrown:9127187,salmon:16416882,sandybrown:16032864,seagreen:3050327,seashell:16774638,sienna:10506797,silver:12632256,skyblue:8900331,slateblue:6970061,slategray:7372944,slategrey:7372944,snow:16775930,springgreen:65407,steelblue:4620980,tan:13808780,teal:32896,thistle:14204888,tomato:16737095,turquoise:4251856,violet:15631086,wheat:16113331,white:16777215,whitesmoke:16119285,yellow:16776960,yellowgreen:10145074},vi={h:0,s:0,l:0},Kr={h:0,s:0,l:0};function Do(n,t,e){return e<0&&(e+=1),e>1&&(e-=1),e<1/6?n+(t-n)*6*e:e<1/2?t:e<2/3?n+(t-n)*6*(2/3-e):n}class ne{constructor(t,e,i){return this.isColor=!0,this.r=1,this.g=1,this.b=1,this.set(t,e,i)}set(t,e,i){if(e===void 0&&i===void 0){const s=t;s&&s.isColor?this.copy(s):typeof s=="number"?this.setHex(s):typeof s=="string"&&this.setStyle(s)}else this.setRGB(t,e,i);return this}setScalar(t){return this.r=t,this.g=t,this.b=t,this}setHex(t,e=ze){return t=Math.floor(t),this.r=(t>>16&255)/255,this.g=(t>>8&255)/255,this.b=(t&255)/255,_e.toWorkingColorSpace(this,e),this}setRGB(t,e,i,s=_e.workingColorSpace){return this.r=t,this.g=e,this.b=i,_e.toWorkingColorSpace(this,s),this}setHSL(t,e,i,s=_e.workingColorSpace){if(t=hc(t,1),e=he(e,0,1),i=he(i,0,1),e===0)this.r=this.g=this.b=i;else{const r=i<=.5?i*(1+e):i+e-i*e,a=2*i-r;this.r=Do(a,r,t+1/3),this.g=Do(a,r,t),this.b=Do(a,r,t-1/3)}return _e.toWorkingColorSpace(this,s),this}setStyle(t,e=ze){function i(r){r!==void 0&&parseFloat(r)<1&&console.warn("THREE.Color: Alpha component of "+t+" will be ignored.")}let s;if(s=/^(\w+)\(([^\)]*)\)/.exec(t)){let r;const a=s[1],o=s[2];switch(a){case"rgb":case"rgba":if(r=/^\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setRGB(Math.min(255,parseInt(r[1],10))/255,Math.min(255,parseInt(r[2],10))/255,Math.min(255,parseInt(r[3],10))/255,e);if(r=/^\s*(\d+)\%\s*,\s*(\d+)\%\s*,\s*(\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setRGB(Math.min(100,parseInt(r[1],10))/100,Math.min(100,parseInt(r[2],10))/100,Math.min(100,parseInt(r[3],10))/100,e);break;case"hsl":case"hsla":if(r=/^\s*(\d*\.?\d+)\s*,\s*(\d*\.?\d+)\%\s*,\s*(\d*\.?\d+)\%\s*(?:,\s*(\d*\.?\d+)\s*)?$/.exec(o))return i(r[4]),this.setHSL(parseFloat(r[1])/360,parseFloat(r[2])/100,parseFloat(r[3])/100,e);break;default:console.warn("THREE.Color: Unknown color model "+t)}}else if(s=/^\#([A-Fa-f\d]+)$/.exec(t)){const r=s[1],a=r.length;if(a===3)return this.setRGB(parseInt(r.charAt(0),16)/15,parseInt(r.charAt(1),16)/15,parseInt(r.charAt(2),16)/15,e);if(a===6)return this.setHex(parseInt(r,16),e);console.warn("THREE.Color: Invalid hex color "+t)}else if(t&&t.length>0)return this.setColorName(t,e);return this}setColorName(t,e=ze){const i=ad[t.toLowerCase()];return i!==void 0?this.setHex(i,e):console.warn("THREE.Color: Unknown color "+t),this}clone(){return new this.constructor(this.r,this.g,this.b)}copy(t){return this.r=t.r,this.g=t.g,this.b=t.b,this}copySRGBToLinear(t){return this.r=ci(t.r),this.g=ci(t.g),this.b=ci(t.b),this}copyLinearToSRGB(t){return this.r=zs(t.r),this.g=zs(t.g),this.b=zs(t.b),this}convertSRGBToLinear(){return this.copySRGBToLinear(this),this}convertLinearToSRGB(){return this.copyLinearToSRGB(this),this}getHex(t=ze){return _e.fromWorkingColorSpace(rn.copy(this),t),Math.round(he(rn.r*255,0,255))*65536+Math.round(he(rn.g*255,0,255))*256+Math.round(he(rn.b*255,0,255))}getHexString(t=ze){return("000000"+this.getHex(t).toString(16)).slice(-6)}getHSL(t,e=_e.workingColorSpace){_e.fromWorkingColorSpace(rn.copy(this),e);const i=rn.r,s=rn.g,r=rn.b,a=Math.max(i,s,r),o=Math.min(i,s,r);let l,c;const u=(o+a)/2;if(o===a)l=0,c=0;else{const h=a-o;switch(c=u<=.5?h/(a+o):h/(2-a-o),a){case i:l=(s-r)/h+(s<r?6:0);break;case s:l=(r-i)/h+2;break;case r:l=(i-s)/h+4;break}l/=6}return t.h=l,t.s=c,t.l=u,t}getRGB(t,e=_e.workingColorSpace){return _e.fromWorkingColorSpace(rn.copy(this),e),t.r=rn.r,t.g=rn.g,t.b=rn.b,t}getStyle(t=ze){_e.fromWorkingColorSpace(rn.copy(this),t);const e=rn.r,i=rn.g,s=rn.b;return t!==ze?`color(${t} ${e.toFixed(3)} ${i.toFixed(3)} ${s.toFixed(3)})`:`rgb(${Math.round(e*255)},${Math.round(i*255)},${Math.round(s*255)})`}offsetHSL(t,e,i){return this.getHSL(vi),this.setHSL(vi.h+t,vi.s+e,vi.l+i)}add(t){return this.r+=t.r,this.g+=t.g,this.b+=t.b,this}addColors(t,e){return this.r=t.r+e.r,this.g=t.g+e.g,this.b=t.b+e.b,this}addScalar(t){return this.r+=t,this.g+=t,this.b+=t,this}sub(t){return this.r=Math.max(0,this.r-t.r),this.g=Math.max(0,this.g-t.g),this.b=Math.max(0,this.b-t.b),this}multiply(t){return this.r*=t.r,this.g*=t.g,this.b*=t.b,this}multiplyScalar(t){return this.r*=t,this.g*=t,this.b*=t,this}lerp(t,e){return this.r+=(t.r-this.r)*e,this.g+=(t.g-this.g)*e,this.b+=(t.b-this.b)*e,this}lerpColors(t,e,i){return this.r=t.r+(e.r-t.r)*i,this.g=t.g+(e.g-t.g)*i,this.b=t.b+(e.b-t.b)*i,this}lerpHSL(t,e){this.getHSL(vi),t.getHSL(Kr);const i=xr(vi.h,Kr.h,e),s=xr(vi.s,Kr.s,e),r=xr(vi.l,Kr.l,e);return this.setHSL(i,s,r),this}setFromVector3(t){return this.r=t.x,this.g=t.y,this.b=t.z,this}applyMatrix3(t){const e=this.r,i=this.g,s=this.b,r=t.elements;return this.r=r[0]*e+r[3]*i+r[6]*s,this.g=r[1]*e+r[4]*i+r[7]*s,this.b=r[2]*e+r[5]*i+r[8]*s,this}equals(t){return t.r===this.r&&t.g===this.g&&t.b===this.b}fromArray(t,e=0){return this.r=t[e],this.g=t[e+1],this.b=t[e+2],this}toArray(t=[],e=0){return t[e]=this.r,t[e+1]=this.g,t[e+2]=this.b,t}fromBufferAttribute(t,e){return this.r=t.getX(e),this.g=t.getY(e),this.b=t.getZ(e),this}toJSON(){return this.getHex()}*[Symbol.iterator](){yield this.r,yield this.g,yield this.b}}const rn=new ne;ne.NAMES=ad;let Am=0;class di extends rs{constructor(){super(),this.isMaterial=!0,Object.defineProperty(this,"id",{value:Am++}),this.uuid=Ui(),this.name="",this.type="Material",this.blending=Fs,this.side=ui,this.vertexColors=!1,this.opacity=1,this.transparent=!1,this.alphaHash=!1,this.blendSrc=sl,this.blendDst=rl,this.blendEquation=qi,this.blendSrcAlpha=null,this.blendDstAlpha=null,this.blendEquationAlpha=null,this.blendColor=new ne(0,0,0),this.blendAlpha=0,this.depthFunc=Ws,this.depthTest=!0,this.depthWrite=!0,this.stencilWriteMask=255,this.stencilFunc=Qc,this.stencilRef=0,this.stencilFuncMask=255,this.stencilFail=_s,this.stencilZFail=_s,this.stencilZPass=_s,this.stencilWrite=!1,this.clippingPlanes=null,this.clipIntersection=!1,this.clipShadows=!1,this.shadowSide=null,this.colorWrite=!0,this.precision=null,this.polygonOffset=!1,this.polygonOffsetFactor=0,this.polygonOffsetUnits=0,this.dithering=!1,this.alphaToCoverage=!1,this.premultipliedAlpha=!1,this.forceSinglePass=!1,this.visible=!0,this.toneMapped=!0,this.userData={},this.version=0,this._alphaTest=0}get alphaTest(){return this._alphaTest}set alphaTest(t){this._alphaTest>0!=t>0&&this.version++,this._alphaTest=t}onBeforeRender(){}onBeforeCompile(){}customProgramCacheKey(){return this.onBeforeCompile.toString()}setValues(t){if(t!==void 0)for(const e in t){const i=t[e];if(i===void 0){console.warn(`THREE.Material: parameter '${e}' has value of undefined.`);continue}const s=this[e];if(s===void 0){console.warn(`THREE.Material: '${e}' is not a property of THREE.${this.type}.`);continue}s&&s.isColor?s.set(i):s&&s.isVector3&&i&&i.isVector3?s.copy(i):this[e]=i}}toJSON(t){const e=t===void 0||typeof t=="string";e&&(t={textures:{},images:{}});const i={metadata:{version:4.6,type:"Material",generator:"Material.toJSON"}};i.uuid=this.uuid,i.type=this.type,this.name!==""&&(i.name=this.name),this.color&&this.color.isColor&&(i.color=this.color.getHex()),this.roughness!==void 0&&(i.roughness=this.roughness),this.metalness!==void 0&&(i.metalness=this.metalness),this.sheen!==void 0&&(i.sheen=this.sheen),this.sheenColor&&this.sheenColor.isColor&&(i.sheenColor=this.sheenColor.getHex()),this.sheenRoughness!==void 0&&(i.sheenRoughness=this.sheenRoughness),this.emissive&&this.emissive.isColor&&(i.emissive=this.emissive.getHex()),this.emissiveIntensity!==void 0&&this.emissiveIntensity!==1&&(i.emissiveIntensity=this.emissiveIntensity),this.specular&&this.specular.isColor&&(i.specular=this.specular.getHex()),this.specularIntensity!==void 0&&(i.specularIntensity=this.specularIntensity),this.specularColor&&this.specularColor.isColor&&(i.specularColor=this.specularColor.getHex()),this.shininess!==void 0&&(i.shininess=this.shininess),this.clearcoat!==void 0&&(i.clearcoat=this.clearcoat),this.clearcoatRoughness!==void 0&&(i.clearcoatRoughness=this.clearcoatRoughness),this.clearcoatMap&&this.clearcoatMap.isTexture&&(i.clearcoatMap=this.clearcoatMap.toJSON(t).uuid),this.clearcoatRoughnessMap&&this.clearcoatRoughnessMap.isTexture&&(i.clearcoatRoughnessMap=this.clearcoatRoughnessMap.toJSON(t).uuid),this.clearcoatNormalMap&&this.clearcoatNormalMap.isTexture&&(i.clearcoatNormalMap=this.clearcoatNormalMap.toJSON(t).uuid,i.clearcoatNormalScale=this.clearcoatNormalScale.toArray()),this.dispersion!==void 0&&(i.dispersion=this.dispersion),this.iridescence!==void 0&&(i.iridescence=this.iridescence),this.iridescenceIOR!==void 0&&(i.iridescenceIOR=this.iridescenceIOR),this.iridescenceThicknessRange!==void 0&&(i.iridescenceThicknessRange=this.iridescenceThicknessRange),this.iridescenceMap&&this.iridescenceMap.isTexture&&(i.iridescenceMap=this.iridescenceMap.toJSON(t).uuid),this.iridescenceThicknessMap&&this.iridescenceThicknessMap.isTexture&&(i.iridescenceThicknessMap=this.iridescenceThicknessMap.toJSON(t).uuid),this.anisotropy!==void 0&&(i.anisotropy=this.anisotropy),this.anisotropyRotation!==void 0&&(i.anisotropyRotation=this.anisotropyRotation),this.anisotropyMap&&this.anisotropyMap.isTexture&&(i.anisotropyMap=this.anisotropyMap.toJSON(t).uuid),this.map&&this.map.isTexture&&(i.map=this.map.toJSON(t).uuid),this.matcap&&this.matcap.isTexture&&(i.matcap=this.matcap.toJSON(t).uuid),this.alphaMap&&this.alphaMap.isTexture&&(i.alphaMap=this.alphaMap.toJSON(t).uuid),this.lightMap&&this.lightMap.isTexture&&(i.lightMap=this.lightMap.toJSON(t).uuid,i.lightMapIntensity=this.lightMapIntensity),this.aoMap&&this.aoMap.isTexture&&(i.aoMap=this.aoMap.toJSON(t).uuid,i.aoMapIntensity=this.aoMapIntensity),this.bumpMap&&this.bumpMap.isTexture&&(i.bumpMap=this.bumpMap.toJSON(t).uuid,i.bumpScale=this.bumpScale),this.normalMap&&this.normalMap.isTexture&&(i.normalMap=this.normalMap.toJSON(t).uuid,i.normalMapType=this.normalMapType,i.normalScale=this.normalScale.toArray()),this.displacementMap&&this.displacementMap.isTexture&&(i.displacementMap=this.displacementMap.toJSON(t).uuid,i.displacementScale=this.displacementScale,i.displacementBias=this.displacementBias),this.roughnessMap&&this.roughnessMap.isTexture&&(i.roughnessMap=this.roughnessMap.toJSON(t).uuid),this.metalnessMap&&this.metalnessMap.isTexture&&(i.metalnessMap=this.metalnessMap.toJSON(t).uuid),this.emissiveMap&&this.emissiveMap.isTexture&&(i.emissiveMap=this.emissiveMap.toJSON(t).uuid),this.specularMap&&this.specularMap.isTexture&&(i.specularMap=this.specularMap.toJSON(t).uuid),this.specularIntensityMap&&this.specularIntensityMap.isTexture&&(i.specularIntensityMap=this.specularIntensityMap.toJSON(t).uuid),this.specularColorMap&&this.specularColorMap.isTexture&&(i.specularColorMap=this.specularColorMap.toJSON(t).uuid),this.envMap&&this.envMap.isTexture&&(i.envMap=this.envMap.toJSON(t).uuid,this.combine!==void 0&&(i.combine=this.combine)),this.envMapRotation!==void 0&&(i.envMapRotation=this.envMapRotation.toArray()),this.envMapIntensity!==void 0&&(i.envMapIntensity=this.envMapIntensity),this.reflectivity!==void 0&&(i.reflectivity=this.reflectivity),this.refractionRatio!==void 0&&(i.refractionRatio=this.refractionRatio),this.gradientMap&&this.gradientMap.isTexture&&(i.gradientMap=this.gradientMap.toJSON(t).uuid),this.transmission!==void 0&&(i.transmission=this.transmission),this.transmissionMap&&this.transmissionMap.isTexture&&(i.transmissionMap=this.transmissionMap.toJSON(t).uuid),this.thickness!==void 0&&(i.thickness=this.thickness),this.thicknessMap&&this.thicknessMap.isTexture&&(i.thicknessMap=this.thicknessMap.toJSON(t).uuid),this.attenuationDistance!==void 0&&this.attenuationDistance!==1/0&&(i.attenuationDistance=this.attenuationDistance),this.attenuationColor!==void 0&&(i.attenuationColor=this.attenuationColor.getHex()),this.size!==void 0&&(i.size=this.size),this.shadowSide!==null&&(i.shadowSide=this.shadowSide),this.sizeAttenuation!==void 0&&(i.sizeAttenuation=this.sizeAttenuation),this.blending!==Fs&&(i.blending=this.blending),this.side!==ui&&(i.side=this.side),this.vertexColors===!0&&(i.vertexColors=!0),this.opacity<1&&(i.opacity=this.opacity),this.transparent===!0&&(i.transparent=!0),this.blendSrc!==sl&&(i.blendSrc=this.blendSrc),this.blendDst!==rl&&(i.blendDst=this.blendDst),this.blendEquation!==qi&&(i.blendEquation=this.blendEquation),this.blendSrcAlpha!==null&&(i.blendSrcAlpha=this.blendSrcAlpha),this.blendDstAlpha!==null&&(i.blendDstAlpha=this.blendDstAlpha),this.blendEquationAlpha!==null&&(i.blendEquationAlpha=this.blendEquationAlpha),this.blendColor&&this.blendColor.isColor&&(i.blendColor=this.blendColor.getHex()),this.blendAlpha!==0&&(i.blendAlpha=this.blendAlpha),this.depthFunc!==Ws&&(i.depthFunc=this.depthFunc),this.depthTest===!1&&(i.depthTest=this.depthTest),this.depthWrite===!1&&(i.depthWrite=this.depthWrite),this.colorWrite===!1&&(i.colorWrite=this.colorWrite),this.stencilWriteMask!==255&&(i.stencilWriteMask=this.stencilWriteMask),this.stencilFunc!==Qc&&(i.stencilFunc=this.stencilFunc),this.stencilRef!==0&&(i.stencilRef=this.stencilRef),this.stencilFuncMask!==255&&(i.stencilFuncMask=this.stencilFuncMask),this.stencilFail!==_s&&(i.stencilFail=this.stencilFail),this.stencilZFail!==_s&&(i.stencilZFail=this.stencilZFail),this.stencilZPass!==_s&&(i.stencilZPass=this.stencilZPass),this.stencilWrite===!0&&(i.stencilWrite=this.stencilWrite),this.rotation!==void 0&&this.rotation!==0&&(i.rotation=this.rotation),this.polygonOffset===!0&&(i.polygonOffset=!0),this.polygonOffsetFactor!==0&&(i.polygonOffsetFactor=this.polygonOffsetFactor),this.polygonOffsetUnits!==0&&(i.polygonOffsetUnits=this.polygonOffsetUnits),this.linewidth!==void 0&&this.linewidth!==1&&(i.linewidth=this.linewidth),this.dashSize!==void 0&&(i.dashSize=this.dashSize),this.gapSize!==void 0&&(i.gapSize=this.gapSize),this.scale!==void 0&&(i.scale=this.scale),this.dithering===!0&&(i.dithering=!0),this.alphaTest>0&&(i.alphaTest=this.alphaTest),this.alphaHash===!0&&(i.alphaHash=!0),this.alphaToCoverage===!0&&(i.alphaToCoverage=!0),this.premultipliedAlpha===!0&&(i.premultipliedAlpha=!0),this.forceSinglePass===!0&&(i.forceSinglePass=!0),this.wireframe===!0&&(i.wireframe=!0),this.wireframeLinewidth>1&&(i.wireframeLinewidth=this.wireframeLinewidth),this.wireframeLinecap!=="round"&&(i.wireframeLinecap=this.wireframeLinecap),this.wireframeLinejoin!=="round"&&(i.wireframeLinejoin=this.wireframeLinejoin),this.flatShading===!0&&(i.flatShading=!0),this.visible===!1&&(i.visible=!1),this.toneMapped===!1&&(i.toneMapped=!1),this.fog===!1&&(i.fog=!1),Object.keys(this.userData).length>0&&(i.userData=this.userData);function s(r){const a=[];for(const o in r){const l=r[o];delete l.metadata,a.push(l)}return a}if(e){const r=s(t.textures),a=s(t.images);r.length>0&&(i.textures=r),a.length>0&&(i.images=a)}return i}clone(){return new this.constructor().copy(this)}copy(t){this.name=t.name,this.blending=t.blending,this.side=t.side,this.vertexColors=t.vertexColors,this.opacity=t.opacity,this.transparent=t.transparent,this.blendSrc=t.blendSrc,this.blendDst=t.blendDst,this.blendEquation=t.blendEquation,this.blendSrcAlpha=t.blendSrcAlpha,this.blendDstAlpha=t.blendDstAlpha,this.blendEquationAlpha=t.blendEquationAlpha,this.blendColor.copy(t.blendColor),this.blendAlpha=t.blendAlpha,this.depthFunc=t.depthFunc,this.depthTest=t.depthTest,this.depthWrite=t.depthWrite,this.stencilWriteMask=t.stencilWriteMask,this.stencilFunc=t.stencilFunc,this.stencilRef=t.stencilRef,this.stencilFuncMask=t.stencilFuncMask,this.stencilFail=t.stencilFail,this.stencilZFail=t.stencilZFail,this.stencilZPass=t.stencilZPass,this.stencilWrite=t.stencilWrite;const e=t.clippingPlanes;let i=null;if(e!==null){const s=e.length;i=new Array(s);for(let r=0;r!==s;++r)i[r]=e[r].clone()}return this.clippingPlanes=i,this.clipIntersection=t.clipIntersection,this.clipShadows=t.clipShadows,this.shadowSide=t.shadowSide,this.colorWrite=t.colorWrite,this.precision=t.precision,this.polygonOffset=t.polygonOffset,this.polygonOffsetFactor=t.polygonOffsetFactor,this.polygonOffsetUnits=t.polygonOffsetUnits,this.dithering=t.dithering,this.alphaTest=t.alphaTest,this.alphaHash=t.alphaHash,this.alphaToCoverage=t.alphaToCoverage,this.premultipliedAlpha=t.premultipliedAlpha,this.forceSinglePass=t.forceSinglePass,this.visible=t.visible,this.toneMapped=t.toneMapped,this.userData=JSON.parse(JSON.stringify(t.userData)),this}dispose(){this.dispatchEvent({type:"dispose"})}set needsUpdate(t){t===!0&&this.version++}onBuild(){console.warn("Material: onBuild() has been removed.")}}class Er extends di{constructor(t){super(),this.isMeshBasicMaterial=!0,this.type="MeshBasicMaterial",this.color=new ne(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yn,this.combine=Ya,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.fog=t.fog,this}}const We=new O,Jr=new Yt;let Cm=0;class dn{constructor(t,e,i=!1){if(Array.isArray(t))throw new TypeError("THREE.BufferAttribute: array should be a Typed Array.");this.isBufferAttribute=!0,Object.defineProperty(this,"id",{value:Cm++}),this.name="",this.array=t,this.itemSize=e,this.count=t!==void 0?t.length/e:0,this.normalized=i,this.usage=tu,this.updateRanges=[],this.gpuType=jn,this.version=0}onUploadCallback(){}set needsUpdate(t){t===!0&&this.version++}setUsage(t){return this.usage=t,this}addUpdateRange(t,e){this.updateRanges.push({start:t,count:e})}clearUpdateRanges(){this.updateRanges.length=0}copy(t){return this.name=t.name,this.array=new t.array.constructor(t.array),this.itemSize=t.itemSize,this.count=t.count,this.normalized=t.normalized,this.usage=t.usage,this.gpuType=t.gpuType,this}copyAt(t,e,i){t*=this.itemSize,i*=e.itemSize;for(let s=0,r=this.itemSize;s<r;s++)this.array[t+s]=e.array[i+s];return this}copyArray(t){return this.array.set(t),this}applyMatrix3(t){if(this.itemSize===2)for(let e=0,i=this.count;e<i;e++)Jr.fromBufferAttribute(this,e),Jr.applyMatrix3(t),this.setXY(e,Jr.x,Jr.y);else if(this.itemSize===3)for(let e=0,i=this.count;e<i;e++)We.fromBufferAttribute(this,e),We.applyMatrix3(t),this.setXYZ(e,We.x,We.y,We.z);return this}applyMatrix4(t){for(let e=0,i=this.count;e<i;e++)We.fromBufferAttribute(this,e),We.applyMatrix4(t),this.setXYZ(e,We.x,We.y,We.z);return this}applyNormalMatrix(t){for(let e=0,i=this.count;e<i;e++)We.fromBufferAttribute(this,e),We.applyNormalMatrix(t),this.setXYZ(e,We.x,We.y,We.z);return this}transformDirection(t){for(let e=0,i=this.count;e<i;e++)We.fromBufferAttribute(this,e),We.transformDirection(t),this.setXYZ(e,We.x,We.y,We.z);return this}set(t,e=0){return this.array.set(t,e),this}getComponent(t,e){let i=this.array[t*this.itemSize+e];return this.normalized&&(i=Ns(i,this.array)),i}setComponent(t,e,i){return this.normalized&&(i=un(i,this.array)),this.array[t*this.itemSize+e]=i,this}getX(t){let e=this.array[t*this.itemSize];return this.normalized&&(e=Ns(e,this.array)),e}setX(t,e){return this.normalized&&(e=un(e,this.array)),this.array[t*this.itemSize]=e,this}getY(t){let e=this.array[t*this.itemSize+1];return this.normalized&&(e=Ns(e,this.array)),e}setY(t,e){return this.normalized&&(e=un(e,this.array)),this.array[t*this.itemSize+1]=e,this}getZ(t){let e=this.array[t*this.itemSize+2];return this.normalized&&(e=Ns(e,this.array)),e}setZ(t,e){return this.normalized&&(e=un(e,this.array)),this.array[t*this.itemSize+2]=e,this}getW(t){let e=this.array[t*this.itemSize+3];return this.normalized&&(e=Ns(e,this.array)),e}setW(t,e){return this.normalized&&(e=un(e,this.array)),this.array[t*this.itemSize+3]=e,this}setXY(t,e,i){return t*=this.itemSize,this.normalized&&(e=un(e,this.array),i=un(i,this.array)),this.array[t+0]=e,this.array[t+1]=i,this}setXYZ(t,e,i,s){return t*=this.itemSize,this.normalized&&(e=un(e,this.array),i=un(i,this.array),s=un(s,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=s,this}setXYZW(t,e,i,s,r){return t*=this.itemSize,this.normalized&&(e=un(e,this.array),i=un(i,this.array),s=un(s,this.array),r=un(r,this.array)),this.array[t+0]=e,this.array[t+1]=i,this.array[t+2]=s,this.array[t+3]=r,this}onUpload(t){return this.onUploadCallback=t,this}clone(){return new this.constructor(this.array,this.itemSize).copy(this)}toJSON(){const t={itemSize:this.itemSize,type:this.array.constructor.name,array:Array.from(this.array),normalized:this.normalized};return this.name!==""&&(t.name=this.name),this.usage!==tu&&(t.usage=this.usage),t}}class od extends dn{constructor(t,e,i){super(new Uint16Array(t),e,i)}}class ld extends dn{constructor(t,e,i){super(new Uint32Array(t),e,i)}}class Ae extends dn{constructor(t,e,i){super(new Float32Array(t),e,i)}}let Rm=0;const Cn=new Kt,Io=new Ve,As=new O,En=new Li,fr=new Li,Ze=new O;class $e extends rs{constructor(){super(),this.isBufferGeometry=!0,Object.defineProperty(this,"id",{value:Rm++}),this.uuid=Ui(),this.name="",this.type="BufferGeometry",this.index=null,this.indirect=null,this.attributes={},this.morphAttributes={},this.morphTargetsRelative=!1,this.groups=[],this.boundingBox=null,this.boundingSphere=null,this.drawRange={start:0,count:1/0},this.userData={}}getIndex(){return this.index}setIndex(t){return Array.isArray(t)?this.index=new(id(t)?ld:od)(t,1):this.index=t,this}setIndirect(t){return this.indirect=t,this}getIndirect(){return this.indirect}getAttribute(t){return this.attributes[t]}setAttribute(t,e){return this.attributes[t]=e,this}deleteAttribute(t){return delete this.attributes[t],this}hasAttribute(t){return this.attributes[t]!==void 0}addGroup(t,e,i=0){this.groups.push({start:t,count:e,materialIndex:i})}clearGroups(){this.groups=[]}setDrawRange(t,e){this.drawRange.start=t,this.drawRange.count=e}applyMatrix4(t){const e=this.attributes.position;e!==void 0&&(e.applyMatrix4(t),e.needsUpdate=!0);const i=this.attributes.normal;if(i!==void 0){const r=new le().getNormalMatrix(t);i.applyNormalMatrix(r),i.needsUpdate=!0}const s=this.attributes.tangent;return s!==void 0&&(s.transformDirection(t),s.needsUpdate=!0),this.boundingBox!==null&&this.computeBoundingBox(),this.boundingSphere!==null&&this.computeBoundingSphere(),this}applyQuaternion(t){return Cn.makeRotationFromQuaternion(t),this.applyMatrix4(Cn),this}rotateX(t){return Cn.makeRotationX(t),this.applyMatrix4(Cn),this}rotateY(t){return Cn.makeRotationY(t),this.applyMatrix4(Cn),this}rotateZ(t){return Cn.makeRotationZ(t),this.applyMatrix4(Cn),this}translate(t,e,i){return Cn.makeTranslation(t,e,i),this.applyMatrix4(Cn),this}scale(t,e,i){return Cn.makeScale(t,e,i),this.applyMatrix4(Cn),this}lookAt(t){return Io.lookAt(t),Io.updateMatrix(),this.applyMatrix4(Io.matrix),this}center(){return this.computeBoundingBox(),this.boundingBox.getCenter(As).negate(),this.translate(As.x,As.y,As.z),this}setFromPoints(t){const e=this.getAttribute("position");if(e===void 0){const i=[];for(let s=0,r=t.length;s<r;s++){const a=t[s];i.push(a.x,a.y,a.z||0)}this.setAttribute("position",new Ae(i,3))}else{const i=Math.min(t.length,e.count);for(let s=0;s<i;s++){const r=t[s];e.setXYZ(s,r.x,r.y,r.z||0)}t.length>e.count&&console.warn("THREE.BufferGeometry: Buffer size too small for points data. Use .dispose() and create a new geometry."),e.needsUpdate=!0}return this}computeBoundingBox(){this.boundingBox===null&&(this.boundingBox=new Li);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingBox(): GLBufferAttribute requires a manual bounding box.",this),this.boundingBox.set(new O(-1/0,-1/0,-1/0),new O(1/0,1/0,1/0));return}if(t!==void 0){if(this.boundingBox.setFromBufferAttribute(t),e)for(let i=0,s=e.length;i<s;i++){const r=e[i];En.setFromBufferAttribute(r),this.morphTargetsRelative?(Ze.addVectors(this.boundingBox.min,En.min),this.boundingBox.expandByPoint(Ze),Ze.addVectors(this.boundingBox.max,En.max),this.boundingBox.expandByPoint(Ze)):(this.boundingBox.expandByPoint(En.min),this.boundingBox.expandByPoint(En.max))}}else this.boundingBox.makeEmpty();(isNaN(this.boundingBox.min.x)||isNaN(this.boundingBox.min.y)||isNaN(this.boundingBox.min.z))&&console.error('THREE.BufferGeometry.computeBoundingBox(): Computed min/max have NaN values. The "position" attribute is likely to have NaN values.',this)}computeBoundingSphere(){this.boundingSphere===null&&(this.boundingSphere=new as);const t=this.attributes.position,e=this.morphAttributes.position;if(t&&t.isGLBufferAttribute){console.error("THREE.BufferGeometry.computeBoundingSphere(): GLBufferAttribute requires a manual bounding sphere.",this),this.boundingSphere.set(new O,1/0);return}if(t){const i=this.boundingSphere.center;if(En.setFromBufferAttribute(t),e)for(let r=0,a=e.length;r<a;r++){const o=e[r];fr.setFromBufferAttribute(o),this.morphTargetsRelative?(Ze.addVectors(En.min,fr.min),En.expandByPoint(Ze),Ze.addVectors(En.max,fr.max),En.expandByPoint(Ze)):(En.expandByPoint(fr.min),En.expandByPoint(fr.max))}En.getCenter(i);let s=0;for(let r=0,a=t.count;r<a;r++)Ze.fromBufferAttribute(t,r),s=Math.max(s,i.distanceToSquared(Ze));if(e)for(let r=0,a=e.length;r<a;r++){const o=e[r],l=this.morphTargetsRelative;for(let c=0,u=o.count;c<u;c++)Ze.fromBufferAttribute(o,c),l&&(As.fromBufferAttribute(t,c),Ze.add(As)),s=Math.max(s,i.distanceToSquared(Ze))}this.boundingSphere.radius=Math.sqrt(s),isNaN(this.boundingSphere.radius)&&console.error('THREE.BufferGeometry.computeBoundingSphere(): Computed radius is NaN. The "position" attribute is likely to have NaN values.',this)}}computeTangents(){const t=this.index,e=this.attributes;if(t===null||e.position===void 0||e.normal===void 0||e.uv===void 0){console.error("THREE.BufferGeometry: .computeTangents() failed. Missing required attributes (index, position, normal or uv)");return}const i=e.position,s=e.normal,r=e.uv;this.hasAttribute("tangent")===!1&&this.setAttribute("tangent",new dn(new Float32Array(4*i.count),4));const a=this.getAttribute("tangent"),o=[],l=[];for(let k=0;k<i.count;k++)o[k]=new O,l[k]=new O;const c=new O,u=new O,h=new O,d=new Yt,p=new Yt,g=new Yt,_=new O,m=new O;function f(k,y,M){c.fromBufferAttribute(i,k),u.fromBufferAttribute(i,y),h.fromBufferAttribute(i,M),d.fromBufferAttribute(r,k),p.fromBufferAttribute(r,y),g.fromBufferAttribute(r,M),u.sub(c),h.sub(c),p.sub(d),g.sub(d);const I=1/(p.x*g.y-g.x*p.y);isFinite(I)&&(_.copy(u).multiplyScalar(g.y).addScaledVector(h,-p.y).multiplyScalar(I),m.copy(h).multiplyScalar(p.x).addScaledVector(u,-g.x).multiplyScalar(I),o[k].add(_),o[y].add(_),o[M].add(_),l[k].add(m),l[y].add(m),l[M].add(m))}let P=this.groups;P.length===0&&(P=[{start:0,count:t.count}]);for(let k=0,y=P.length;k<y;++k){const M=P[k],I=M.start,V=M.count;for(let W=I,Z=I+V;W<Z;W+=3)f(t.getX(W+0),t.getX(W+1),t.getX(W+2))}const L=new O,w=new O,N=new O,F=new O;function U(k){N.fromBufferAttribute(s,k),F.copy(N);const y=o[k];L.copy(y),L.sub(N.multiplyScalar(N.dot(y))).normalize(),w.crossVectors(F,y);const I=w.dot(l[k])<0?-1:1;a.setXYZW(k,L.x,L.y,L.z,I)}for(let k=0,y=P.length;k<y;++k){const M=P[k],I=M.start,V=M.count;for(let W=I,Z=I+V;W<Z;W+=3)U(t.getX(W+0)),U(t.getX(W+1)),U(t.getX(W+2))}}computeVertexNormals(){const t=this.index,e=this.getAttribute("position");if(e!==void 0){let i=this.getAttribute("normal");if(i===void 0)i=new dn(new Float32Array(e.count*3),3),this.setAttribute("normal",i);else for(let d=0,p=i.count;d<p;d++)i.setXYZ(d,0,0,0);const s=new O,r=new O,a=new O,o=new O,l=new O,c=new O,u=new O,h=new O;if(t)for(let d=0,p=t.count;d<p;d+=3){const g=t.getX(d+0),_=t.getX(d+1),m=t.getX(d+2);s.fromBufferAttribute(e,g),r.fromBufferAttribute(e,_),a.fromBufferAttribute(e,m),u.subVectors(a,r),h.subVectors(s,r),u.cross(h),o.fromBufferAttribute(i,g),l.fromBufferAttribute(i,_),c.fromBufferAttribute(i,m),o.add(u),l.add(u),c.add(u),i.setXYZ(g,o.x,o.y,o.z),i.setXYZ(_,l.x,l.y,l.z),i.setXYZ(m,c.x,c.y,c.z)}else for(let d=0,p=e.count;d<p;d+=3)s.fromBufferAttribute(e,d+0),r.fromBufferAttribute(e,d+1),a.fromBufferAttribute(e,d+2),u.subVectors(a,r),h.subVectors(s,r),u.cross(h),i.setXYZ(d+0,u.x,u.y,u.z),i.setXYZ(d+1,u.x,u.y,u.z),i.setXYZ(d+2,u.x,u.y,u.z);this.normalizeNormals(),i.needsUpdate=!0}}normalizeNormals(){const t=this.attributes.normal;for(let e=0,i=t.count;e<i;e++)Ze.fromBufferAttribute(t,e),Ze.normalize(),t.setXYZ(e,Ze.x,Ze.y,Ze.z)}toNonIndexed(){function t(o,l){const c=o.array,u=o.itemSize,h=o.normalized,d=new c.constructor(l.length*u);let p=0,g=0;for(let _=0,m=l.length;_<m;_++){o.isInterleavedBufferAttribute?p=l[_]*o.data.stride+o.offset:p=l[_]*u;for(let f=0;f<u;f++)d[g++]=c[p++]}return new dn(d,u,h)}if(this.index===null)return console.warn("THREE.BufferGeometry.toNonIndexed(): BufferGeometry is already non-indexed."),this;const e=new $e,i=this.index.array,s=this.attributes;for(const o in s){const l=s[o],c=t(l,i);e.setAttribute(o,c)}const r=this.morphAttributes;for(const o in r){const l=[],c=r[o];for(let u=0,h=c.length;u<h;u++){const d=c[u],p=t(d,i);l.push(p)}e.morphAttributes[o]=l}e.morphTargetsRelative=this.morphTargetsRelative;const a=this.groups;for(let o=0,l=a.length;o<l;o++){const c=a[o];e.addGroup(c.start,c.count,c.materialIndex)}return e}toJSON(){const t={metadata:{version:4.6,type:"BufferGeometry",generator:"BufferGeometry.toJSON"}};if(t.uuid=this.uuid,t.type=this.type,this.name!==""&&(t.name=this.name),Object.keys(this.userData).length>0&&(t.userData=this.userData),this.parameters!==void 0){const l=this.parameters;for(const c in l)l[c]!==void 0&&(t[c]=l[c]);return t}t.data={attributes:{}};const e=this.index;e!==null&&(t.data.index={type:e.array.constructor.name,array:Array.prototype.slice.call(e.array)});const i=this.attributes;for(const l in i){const c=i[l];t.data.attributes[l]=c.toJSON(t.data)}const s={};let r=!1;for(const l in this.morphAttributes){const c=this.morphAttributes[l],u=[];for(let h=0,d=c.length;h<d;h++){const p=c[h];u.push(p.toJSON(t.data))}u.length>0&&(s[l]=u,r=!0)}r&&(t.data.morphAttributes=s,t.data.morphTargetsRelative=this.morphTargetsRelative);const a=this.groups;a.length>0&&(t.data.groups=JSON.parse(JSON.stringify(a)));const o=this.boundingSphere;return o!==null&&(t.data.boundingSphere={center:o.center.toArray(),radius:o.radius}),t}clone(){return new this.constructor().copy(this)}copy(t){this.index=null,this.attributes={},this.morphAttributes={},this.groups=[],this.boundingBox=null,this.boundingSphere=null;const e={};this.name=t.name;const i=t.index;i!==null&&this.setIndex(i.clone(e));const s=t.attributes;for(const c in s){const u=s[c];this.setAttribute(c,u.clone(e))}const r=t.morphAttributes;for(const c in r){const u=[],h=r[c];for(let d=0,p=h.length;d<p;d++)u.push(h[d].clone(e));this.morphAttributes[c]=u}this.morphTargetsRelative=t.morphTargetsRelative;const a=t.groups;for(let c=0,u=a.length;c<u;c++){const h=a[c];this.addGroup(h.start,h.count,h.materialIndex)}const o=t.boundingBox;o!==null&&(this.boundingBox=o.clone());const l=t.boundingSphere;return l!==null&&(this.boundingSphere=l.clone()),this.drawRange.start=t.drawRange.start,this.drawRange.count=t.drawRange.count,this.userData=t.userData,this}dispose(){this.dispatchEvent({type:"dispose"})}}const mu=new Kt,Hi=new Pr,Zr=new as,gu=new O,Qr=new O,ta=new O,ea=new O,Uo=new O,na=new O,_u=new O,ia=new O;class Re extends Ve{constructor(t=new $e,e=new Er){super(),this.isMesh=!0,this.type="Mesh",this.geometry=t,this.material=e,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),t.morphTargetInfluences!==void 0&&(this.morphTargetInfluences=t.morphTargetInfluences.slice()),t.morphTargetDictionary!==void 0&&(this.morphTargetDictionary=Object.assign({},t.morphTargetDictionary)),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,a=s.length;r<a;r++){const o=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=r}}}}getVertexPosition(t,e){const i=this.geometry,s=i.attributes.position,r=i.morphAttributes.position,a=i.morphTargetsRelative;e.fromBufferAttribute(s,t);const o=this.morphTargetInfluences;if(r&&o){na.set(0,0,0);for(let l=0,c=r.length;l<c;l++){const u=o[l],h=r[l];u!==0&&(Uo.fromBufferAttribute(h,t),a?na.addScaledVector(Uo,u):na.addScaledVector(Uo.sub(e),u))}e.add(na)}return e}raycast(t,e){const i=this.geometry,s=this.material,r=this.matrixWorld;s!==void 0&&(i.boundingSphere===null&&i.computeBoundingSphere(),Zr.copy(i.boundingSphere),Zr.applyMatrix4(r),Hi.copy(t.ray).recast(t.near),!(Zr.containsPoint(Hi.origin)===!1&&(Hi.intersectSphere(Zr,gu)===null||Hi.origin.distanceToSquared(gu)>(t.far-t.near)**2))&&(mu.copy(r).invert(),Hi.copy(t.ray).applyMatrix4(mu),!(i.boundingBox!==null&&Hi.intersectsBox(i.boundingBox)===!1)&&this._computeIntersections(t,e,Hi)))}_computeIntersections(t,e,i){let s;const r=this.geometry,a=this.material,o=r.index,l=r.attributes.position,c=r.attributes.uv,u=r.attributes.uv1,h=r.attributes.normal,d=r.groups,p=r.drawRange;if(o!==null)if(Array.isArray(a))for(let g=0,_=d.length;g<_;g++){const m=d[g],f=a[m.materialIndex],P=Math.max(m.start,p.start),L=Math.min(o.count,Math.min(m.start+m.count,p.start+p.count));for(let w=P,N=L;w<N;w+=3){const F=o.getX(w),U=o.getX(w+1),k=o.getX(w+2);s=sa(this,f,t,i,c,u,h,F,U,k),s&&(s.faceIndex=Math.floor(w/3),s.face.materialIndex=m.materialIndex,e.push(s))}}else{const g=Math.max(0,p.start),_=Math.min(o.count,p.start+p.count);for(let m=g,f=_;m<f;m+=3){const P=o.getX(m),L=o.getX(m+1),w=o.getX(m+2);s=sa(this,a,t,i,c,u,h,P,L,w),s&&(s.faceIndex=Math.floor(m/3),e.push(s))}}else if(l!==void 0)if(Array.isArray(a))for(let g=0,_=d.length;g<_;g++){const m=d[g],f=a[m.materialIndex],P=Math.max(m.start,p.start),L=Math.min(l.count,Math.min(m.start+m.count,p.start+p.count));for(let w=P,N=L;w<N;w+=3){const F=w,U=w+1,k=w+2;s=sa(this,f,t,i,c,u,h,F,U,k),s&&(s.faceIndex=Math.floor(w/3),s.face.materialIndex=m.materialIndex,e.push(s))}}else{const g=Math.max(0,p.start),_=Math.min(l.count,p.start+p.count);for(let m=g,f=_;m<f;m+=3){const P=m,L=m+1,w=m+2;s=sa(this,a,t,i,c,u,h,P,L,w),s&&(s.faceIndex=Math.floor(m/3),e.push(s))}}}}function Pm(n,t,e,i,s,r,a,o){let l;if(t.side===bn?l=i.intersectTriangle(a,r,s,!0,o):l=i.intersectTriangle(s,r,a,t.side===ui,o),l===null)return null;ia.copy(o),ia.applyMatrix4(n.matrixWorld);const c=e.ray.origin.distanceTo(ia);return c<e.near||c>e.far?null:{distance:c,point:ia.clone(),object:n}}function sa(n,t,e,i,s,r,a,o,l,c){n.getVertexPosition(o,Qr),n.getVertexPosition(l,ta),n.getVertexPosition(c,ea);const u=Pm(n,t,e,i,Qr,ta,ea,_u);if(u){const h=new O;Rn.getBarycoord(_u,Qr,ta,ea,h),s&&(u.uv=Rn.getInterpolatedAttribute(s,o,l,c,h,new Yt)),r&&(u.uv1=Rn.getInterpolatedAttribute(r,o,l,c,h,new Yt)),a&&(u.normal=Rn.getInterpolatedAttribute(a,o,l,c,h,new O),u.normal.dot(i.direction)>0&&u.normal.multiplyScalar(-1));const d={a:o,b:l,c,normal:new O,materialIndex:0};Rn.getNormal(Qr,ta,ea,d.normal),u.face=d,u.barycoord=h}return u}class Bn extends $e{constructor(t=1,e=1,i=1,s=1,r=1,a=1){super(),this.type="BoxGeometry",this.parameters={width:t,height:e,depth:i,widthSegments:s,heightSegments:r,depthSegments:a};const o=this;s=Math.floor(s),r=Math.floor(r),a=Math.floor(a);const l=[],c=[],u=[],h=[];let d=0,p=0;g("z","y","x",-1,-1,i,e,t,a,r,0),g("z","y","x",1,-1,i,e,-t,a,r,1),g("x","z","y",1,1,t,i,e,s,a,2),g("x","z","y",1,-1,t,i,-e,s,a,3),g("x","y","z",1,-1,t,e,i,s,r,4),g("x","y","z",-1,-1,t,e,-i,s,r,5),this.setIndex(l),this.setAttribute("position",new Ae(c,3)),this.setAttribute("normal",new Ae(u,3)),this.setAttribute("uv",new Ae(h,2));function g(_,m,f,P,L,w,N,F,U,k,y){const M=w/U,I=N/k,V=w/2,W=N/2,Z=F/2,et=U+1,z=k+1;let it=0,j=0;const ut=new O;for(let mt=0;mt<z;mt++){const gt=mt*I-W;for(let Lt=0;Lt<et;Lt++){const At=Lt*M-V;ut[_]=At*P,ut[m]=gt*L,ut[f]=Z,c.push(ut.x,ut.y,ut.z),ut[_]=0,ut[m]=0,ut[f]=F>0?1:-1,u.push(ut.x,ut.y,ut.z),h.push(Lt/U),h.push(1-mt/k),it+=1}}for(let mt=0;mt<k;mt++)for(let gt=0;gt<U;gt++){const Lt=d+gt+et*mt,At=d+gt+et*(mt+1),X=d+(gt+1)+et*(mt+1),Q=d+(gt+1)+et*mt;l.push(Lt,At,Q),l.push(At,X,Q),j+=6}o.addGroup(p,j,y),p+=j,d+=it}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Bn(t.width,t.height,t.depth,t.widthSegments,t.heightSegments,t.depthSegments)}}function Js(n){const t={};for(const e in n){t[e]={};for(const i in n[e]){const s=n[e][i];s&&(s.isColor||s.isMatrix3||s.isMatrix4||s.isVector2||s.isVector3||s.isVector4||s.isTexture||s.isQuaternion)?s.isRenderTargetTexture?(console.warn("UniformsUtils: Textures of render targets cannot be cloned via cloneUniforms() or mergeUniforms()."),t[e][i]=null):t[e][i]=s.clone():Array.isArray(s)?t[e][i]=s.slice():t[e][i]=s}}return t}function hn(n){const t={};for(let e=0;e<n.length;e++){const i=Js(n[e]);for(const s in i)t[s]=i[s]}return t}function Lm(n){const t=[];for(let e=0;e<n.length;e++)t.push(n[e].clone());return t}function cd(n){const t=n.getRenderTarget();return t===null?n.outputColorSpace:t.isXRRenderTarget===!0?t.texture.colorSpace:_e.workingColorSpace}const Nm={clone:Js,merge:hn};var Dm=`void main() {
	gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}`,Im=`void main() {
	gl_FragColor = vec4( 1.0, 0.0, 0.0, 1.0 );
}`;class Ni extends di{constructor(t){super(),this.isShaderMaterial=!0,this.type="ShaderMaterial",this.defines={},this.uniforms={},this.uniformsGroups=[],this.vertexShader=Dm,this.fragmentShader=Im,this.linewidth=1,this.wireframe=!1,this.wireframeLinewidth=1,this.fog=!1,this.lights=!1,this.clipping=!1,this.forceSinglePass=!0,this.extensions={clipCullDistance:!1,multiDraw:!1},this.defaultAttributeValues={color:[1,1,1],uv:[0,0],uv1:[0,0]},this.index0AttributeName=void 0,this.uniformsNeedUpdate=!1,this.glslVersion=null,t!==void 0&&this.setValues(t)}copy(t){return super.copy(t),this.fragmentShader=t.fragmentShader,this.vertexShader=t.vertexShader,this.uniforms=Js(t.uniforms),this.uniformsGroups=Lm(t.uniformsGroups),this.defines=Object.assign({},t.defines),this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.fog=t.fog,this.lights=t.lights,this.clipping=t.clipping,this.extensions=Object.assign({},t.extensions),this.glslVersion=t.glslVersion,this}toJSON(t){const e=super.toJSON(t);e.glslVersion=this.glslVersion,e.uniforms={};for(const s in this.uniforms){const a=this.uniforms[s].value;a&&a.isTexture?e.uniforms[s]={type:"t",value:a.toJSON(t).uuid}:a&&a.isColor?e.uniforms[s]={type:"c",value:a.getHex()}:a&&a.isVector2?e.uniforms[s]={type:"v2",value:a.toArray()}:a&&a.isVector3?e.uniforms[s]={type:"v3",value:a.toArray()}:a&&a.isVector4?e.uniforms[s]={type:"v4",value:a.toArray()}:a&&a.isMatrix3?e.uniforms[s]={type:"m3",value:a.toArray()}:a&&a.isMatrix4?e.uniforms[s]={type:"m4",value:a.toArray()}:e.uniforms[s]={value:a}}Object.keys(this.defines).length>0&&(e.defines=this.defines),e.vertexShader=this.vertexShader,e.fragmentShader=this.fragmentShader,e.lights=this.lights,e.clipping=this.clipping;const i={};for(const s in this.extensions)this.extensions[s]===!0&&(i[s]=!0);return Object.keys(i).length>0&&(e.extensions=i),e}}let ud=class extends Ve{constructor(){super(),this.isCamera=!0,this.type="Camera",this.matrixWorldInverse=new Kt,this.projectionMatrix=new Kt,this.projectionMatrixInverse=new Kt,this.coordinateSystem=li}copy(t,e){return super.copy(t,e),this.matrixWorldInverse.copy(t.matrixWorldInverse),this.projectionMatrix.copy(t.projectionMatrix),this.projectionMatrixInverse.copy(t.projectionMatrixInverse),this.coordinateSystem=t.coordinateSystem,this}getWorldDirection(t){return super.getWorldDirection(t).negate()}updateMatrixWorld(t){super.updateMatrixWorld(t),this.matrixWorldInverse.copy(this.matrixWorld).invert()}updateWorldMatrix(t,e){super.updateWorldMatrix(t,e),this.matrixWorldInverse.copy(this.matrixWorld).invert()}clone(){return new this.constructor().copy(this)}};const xi=new O,vu=new Yt,xu=new Yt;class an extends ud{constructor(t=50,e=1,i=.1,s=2e3){super(),this.isPerspectiveCamera=!0,this.type="PerspectiveCamera",this.fov=t,this.zoom=1,this.near=i,this.far=s,this.focus=10,this.aspect=e,this.view=null,this.filmGauge=35,this.filmOffset=0,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.fov=t.fov,this.zoom=t.zoom,this.near=t.near,this.far=t.far,this.focus=t.focus,this.aspect=t.aspect,this.view=t.view===null?null:Object.assign({},t.view),this.filmGauge=t.filmGauge,this.filmOffset=t.filmOffset,this}setFocalLength(t){const e=.5*this.getFilmHeight()/t;this.fov=Ks*2*Math.atan(e),this.updateProjectionMatrix()}getFocalLength(){const t=Math.tan(Bs*.5*this.fov);return .5*this.getFilmHeight()/t}getEffectiveFOV(){return Ks*2*Math.atan(Math.tan(Bs*.5*this.fov)/this.zoom)}getFilmWidth(){return this.filmGauge*Math.min(this.aspect,1)}getFilmHeight(){return this.filmGauge/Math.max(this.aspect,1)}getViewBounds(t,e,i){xi.set(-1,-1,.5).applyMatrix4(this.projectionMatrixInverse),e.set(xi.x,xi.y).multiplyScalar(-t/xi.z),xi.set(1,1,.5).applyMatrix4(this.projectionMatrixInverse),i.set(xi.x,xi.y).multiplyScalar(-t/xi.z)}getViewSize(t,e){return this.getViewBounds(t,vu,xu),e.subVectors(xu,vu)}setViewOffset(t,e,i,s,r,a){this.aspect=t/e,this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=s,this.view.width=r,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=this.near;let e=t*Math.tan(Bs*.5*this.fov)/this.zoom,i=2*e,s=this.aspect*i,r=-.5*s;const a=this.view;if(this.view!==null&&this.view.enabled){const l=a.fullWidth,c=a.fullHeight;r+=a.offsetX*s/l,e-=a.offsetY*i/c,s*=a.width/l,i*=a.height/c}const o=this.filmOffset;o!==0&&(r+=t*o/this.getFilmWidth()),this.projectionMatrix.makePerspective(r,r+s,e,e-i,t,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.fov=this.fov,e.object.zoom=this.zoom,e.object.near=this.near,e.object.far=this.far,e.object.focus=this.focus,e.object.aspect=this.aspect,this.view!==null&&(e.object.view=Object.assign({},this.view)),e.object.filmGauge=this.filmGauge,e.object.filmOffset=this.filmOffset,e}}const Cs=-90,Rs=1;class Um extends Ve{constructor(t,e,i){super(),this.type="CubeCamera",this.renderTarget=i,this.coordinateSystem=null,this.activeMipmapLevel=0;const s=new an(Cs,Rs,t,e);s.layers=this.layers,this.add(s);const r=new an(Cs,Rs,t,e);r.layers=this.layers,this.add(r);const a=new an(Cs,Rs,t,e);a.layers=this.layers,this.add(a);const o=new an(Cs,Rs,t,e);o.layers=this.layers,this.add(o);const l=new an(Cs,Rs,t,e);l.layers=this.layers,this.add(l);const c=new an(Cs,Rs,t,e);c.layers=this.layers,this.add(c)}updateCoordinateSystem(){const t=this.coordinateSystem,e=this.children.concat(),[i,s,r,a,o,l]=e;for(const c of e)this.remove(c);if(t===li)i.up.set(0,1,0),i.lookAt(1,0,0),s.up.set(0,1,0),s.lookAt(-1,0,0),r.up.set(0,0,-1),r.lookAt(0,1,0),a.up.set(0,0,1),a.lookAt(0,-1,0),o.up.set(0,1,0),o.lookAt(0,0,1),l.up.set(0,1,0),l.lookAt(0,0,-1);else if(t===Oa)i.up.set(0,-1,0),i.lookAt(-1,0,0),s.up.set(0,-1,0),s.lookAt(1,0,0),r.up.set(0,0,1),r.lookAt(0,1,0),a.up.set(0,0,-1),a.lookAt(0,-1,0),o.up.set(0,-1,0),o.lookAt(0,0,1),l.up.set(0,-1,0),l.lookAt(0,0,-1);else throw new Error("THREE.CubeCamera.updateCoordinateSystem(): Invalid coordinate system: "+t);for(const c of e)this.add(c),c.updateMatrixWorld()}update(t,e){this.parent===null&&this.updateMatrixWorld();const{renderTarget:i,activeMipmapLevel:s}=this;this.coordinateSystem!==t.coordinateSystem&&(this.coordinateSystem=t.coordinateSystem,this.updateCoordinateSystem());const[r,a,o,l,c,u]=this.children,h=t.getRenderTarget(),d=t.getActiveCubeFace(),p=t.getActiveMipmapLevel(),g=t.xr.enabled;t.xr.enabled=!1;const _=i.texture.generateMipmaps;i.texture.generateMipmaps=!1,t.setRenderTarget(i,0,s),t.render(e,r),t.setRenderTarget(i,1,s),t.render(e,a),t.setRenderTarget(i,2,s),t.render(e,o),t.setRenderTarget(i,3,s),t.render(e,l),t.setRenderTarget(i,4,s),t.render(e,c),i.texture.generateMipmaps=_,t.setRenderTarget(i,5,s),t.render(e,u),t.setRenderTarget(h,d,p),t.xr.enabled=g,i.texture.needsPMREMUpdate=!0}}class hd extends ln{constructor(t,e,i,s,r,a,o,l,c,u){t=t!==void 0?t:[],e=e!==void 0?e:$s,super(t,e,i,s,r,a,o,l,c,u),this.isCubeTexture=!0,this.flipY=!1}get images(){return this.image}set images(t){this.image=t}}class km extends es{constructor(t=1,e={}){super(t,t,e),this.isWebGLCubeRenderTarget=!0;const i={width:t,height:t,depth:1},s=[i,i,i,i,i,i];this.texture=new hd(s,e.mapping,e.wrapS,e.wrapT,e.magFilter,e.minFilter,e.format,e.type,e.anisotropy,e.colorSpace),this.texture.isRenderTargetTexture=!0,this.texture.generateMipmaps=e.generateMipmaps!==void 0?e.generateMipmaps:!1,this.texture.minFilter=e.minFilter!==void 0?e.minFilter:wn}fromEquirectangularTexture(t,e){this.texture.type=e.type,this.texture.colorSpace=e.colorSpace,this.texture.generateMipmaps=e.generateMipmaps,this.texture.minFilter=e.minFilter,this.texture.magFilter=e.magFilter;const i={uniforms:{tEquirect:{value:null}},vertexShader:`

				varying vec3 vWorldDirection;

				vec3 transformDirection( in vec3 dir, in mat4 matrix ) {

					return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );

				}

				void main() {

					vWorldDirection = transformDirection( position, modelMatrix );

					#include <begin_vertex>
					#include <project_vertex>

				}
			`,fragmentShader:`

				uniform sampler2D tEquirect;

				varying vec3 vWorldDirection;

				#include <common>

				void main() {

					vec3 direction = normalize( vWorldDirection );

					vec2 sampleUV = equirectUv( direction );

					gl_FragColor = texture2D( tEquirect, sampleUV );

				}
			`},s=new Bn(5,5,5),r=new Ni({name:"CubemapFromEquirect",uniforms:Js(i.uniforms),vertexShader:i.vertexShader,fragmentShader:i.fragmentShader,side:bn,blending:Ri});r.uniforms.tEquirect.value=e;const a=new Re(s,r),o=e.minFilter;return e.minFilter===oi&&(e.minFilter=wn),new Um(1,10,this).update(t,a),e.minFilter=o,a.geometry.dispose(),a.material.dispose(),this}clear(t,e,i,s){const r=t.getRenderTarget();for(let a=0;a<6;a++)t.setRenderTarget(this,a),t.clear(e,i,s);t.setRenderTarget(r)}}class He extends Ve{constructor(){super(),this.isGroup=!0,this.type="Group"}}const Fm={type:"move"};class ko{constructor(){this._targetRay=null,this._grip=null,this._hand=null}getHandSpace(){return this._hand===null&&(this._hand=new He,this._hand.matrixAutoUpdate=!1,this._hand.visible=!1,this._hand.joints={},this._hand.inputState={pinching:!1}),this._hand}getTargetRaySpace(){return this._targetRay===null&&(this._targetRay=new He,this._targetRay.matrixAutoUpdate=!1,this._targetRay.visible=!1,this._targetRay.hasLinearVelocity=!1,this._targetRay.linearVelocity=new O,this._targetRay.hasAngularVelocity=!1,this._targetRay.angularVelocity=new O),this._targetRay}getGripSpace(){return this._grip===null&&(this._grip=new He,this._grip.matrixAutoUpdate=!1,this._grip.visible=!1,this._grip.hasLinearVelocity=!1,this._grip.linearVelocity=new O,this._grip.hasAngularVelocity=!1,this._grip.angularVelocity=new O),this._grip}dispatchEvent(t){return this._targetRay!==null&&this._targetRay.dispatchEvent(t),this._grip!==null&&this._grip.dispatchEvent(t),this._hand!==null&&this._hand.dispatchEvent(t),this}connect(t){if(t&&t.hand){const e=this._hand;if(e)for(const i of t.hand.values())this._getHandJoint(e,i)}return this.dispatchEvent({type:"connected",data:t}),this}disconnect(t){return this.dispatchEvent({type:"disconnected",data:t}),this._targetRay!==null&&(this._targetRay.visible=!1),this._grip!==null&&(this._grip.visible=!1),this._hand!==null&&(this._hand.visible=!1),this}update(t,e,i){let s=null,r=null,a=null;const o=this._targetRay,l=this._grip,c=this._hand;if(t&&e.session.visibilityState!=="visible-blurred"){if(c&&t.hand){a=!0;for(const _ of t.hand.values()){const m=e.getJointPose(_,i),f=this._getHandJoint(c,_);m!==null&&(f.matrix.fromArray(m.transform.matrix),f.matrix.decompose(f.position,f.rotation,f.scale),f.matrixWorldNeedsUpdate=!0,f.jointRadius=m.radius),f.visible=m!==null}const u=c.joints["index-finger-tip"],h=c.joints["thumb-tip"],d=u.position.distanceTo(h.position),p=.02,g=.005;c.inputState.pinching&&d>p+g?(c.inputState.pinching=!1,this.dispatchEvent({type:"pinchend",handedness:t.handedness,target:this})):!c.inputState.pinching&&d<=p-g&&(c.inputState.pinching=!0,this.dispatchEvent({type:"pinchstart",handedness:t.handedness,target:this}))}else l!==null&&t.gripSpace&&(r=e.getPose(t.gripSpace,i),r!==null&&(l.matrix.fromArray(r.transform.matrix),l.matrix.decompose(l.position,l.rotation,l.scale),l.matrixWorldNeedsUpdate=!0,r.linearVelocity?(l.hasLinearVelocity=!0,l.linearVelocity.copy(r.linearVelocity)):l.hasLinearVelocity=!1,r.angularVelocity?(l.hasAngularVelocity=!0,l.angularVelocity.copy(r.angularVelocity)):l.hasAngularVelocity=!1));o!==null&&(s=e.getPose(t.targetRaySpace,i),s===null&&r!==null&&(s=r),s!==null&&(o.matrix.fromArray(s.transform.matrix),o.matrix.decompose(o.position,o.rotation,o.scale),o.matrixWorldNeedsUpdate=!0,s.linearVelocity?(o.hasLinearVelocity=!0,o.linearVelocity.copy(s.linearVelocity)):o.hasLinearVelocity=!1,s.angularVelocity?(o.hasAngularVelocity=!0,o.angularVelocity.copy(s.angularVelocity)):o.hasAngularVelocity=!1,this.dispatchEvent(Fm)))}return o!==null&&(o.visible=s!==null),l!==null&&(l.visible=r!==null),c!==null&&(c.visible=a!==null),this}_getHandJoint(t,e){if(t.joints[e.jointName]===void 0){const i=new He;i.matrixAutoUpdate=!1,i.visible=!1,t.joints[e.jointName]=i,t.add(i)}return t.joints[e.jointName]}}class dd extends Ve{constructor(){super(),this.isScene=!0,this.type="Scene",this.background=null,this.environment=null,this.fog=null,this.backgroundBlurriness=0,this.backgroundIntensity=1,this.backgroundRotation=new yn,this.environmentIntensity=1,this.environmentRotation=new yn,this.overrideMaterial=null,typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}copy(t,e){return super.copy(t,e),t.background!==null&&(this.background=t.background.clone()),t.environment!==null&&(this.environment=t.environment.clone()),t.fog!==null&&(this.fog=t.fog.clone()),this.backgroundBlurriness=t.backgroundBlurriness,this.backgroundIntensity=t.backgroundIntensity,this.backgroundRotation.copy(t.backgroundRotation),this.environmentIntensity=t.environmentIntensity,this.environmentRotation.copy(t.environmentRotation),t.overrideMaterial!==null&&(this.overrideMaterial=t.overrideMaterial.clone()),this.matrixAutoUpdate=t.matrixAutoUpdate,this}toJSON(t){const e=super.toJSON(t);return this.fog!==null&&(e.object.fog=this.fog.toJSON()),this.backgroundBlurriness>0&&(e.object.backgroundBlurriness=this.backgroundBlurriness),this.backgroundIntensity!==1&&(e.object.backgroundIntensity=this.backgroundIntensity),e.object.backgroundRotation=this.backgroundRotation.toArray(),this.environmentIntensity!==1&&(e.object.environmentIntensity=this.environmentIntensity),e.object.environmentRotation=this.environmentRotation.toArray(),e}}const bu=new O,yu=new Ee,Mu=new Ee,Om=new O,Su=new Kt,ra=new O,Fo=new as,Eu=new Kt,Oo=new Pr;class Bm extends Re{constructor(t,e){super(t,e),this.isSkinnedMesh=!0,this.type="SkinnedMesh",this.bindMode=Yc,this.bindMatrix=new Kt,this.bindMatrixInverse=new Kt,this.boundingBox=null,this.boundingSphere=null}computeBoundingBox(){const t=this.geometry;this.boundingBox===null&&(this.boundingBox=new Li),this.boundingBox.makeEmpty();const e=t.getAttribute("position");for(let i=0;i<e.count;i++)this.getVertexPosition(i,ra),this.boundingBox.expandByPoint(ra)}computeBoundingSphere(){const t=this.geometry;this.boundingSphere===null&&(this.boundingSphere=new as),this.boundingSphere.makeEmpty();const e=t.getAttribute("position");for(let i=0;i<e.count;i++)this.getVertexPosition(i,ra),this.boundingSphere.expandByPoint(ra)}copy(t,e){return super.copy(t,e),this.bindMode=t.bindMode,this.bindMatrix.copy(t.bindMatrix),this.bindMatrixInverse.copy(t.bindMatrixInverse),this.skeleton=t.skeleton,t.boundingBox!==null&&(this.boundingBox=t.boundingBox.clone()),t.boundingSphere!==null&&(this.boundingSphere=t.boundingSphere.clone()),this}raycast(t,e){const i=this.material,s=this.matrixWorld;i!==void 0&&(this.boundingSphere===null&&this.computeBoundingSphere(),Fo.copy(this.boundingSphere),Fo.applyMatrix4(s),t.ray.intersectsSphere(Fo)!==!1&&(Eu.copy(s).invert(),Oo.copy(t.ray).applyMatrix4(Eu),!(this.boundingBox!==null&&Oo.intersectsBox(this.boundingBox)===!1)&&this._computeIntersections(t,e,Oo)))}getVertexPosition(t,e){return super.getVertexPosition(t,e),this.applyBoneTransform(t,e),e}bind(t,e){this.skeleton=t,e===void 0&&(this.updateMatrixWorld(!0),this.skeleton.calculateInverses(),e=this.matrixWorld),this.bindMatrix.copy(e),this.bindMatrixInverse.copy(e).invert()}pose(){this.skeleton.pose()}normalizeSkinWeights(){const t=new Ee,e=this.geometry.attributes.skinWeight;for(let i=0,s=e.count;i<s;i++){t.fromBufferAttribute(e,i);const r=1/t.manhattanLength();r!==1/0?t.multiplyScalar(r):t.set(1,0,0,0),e.setXYZW(i,t.x,t.y,t.z,t.w)}}updateMatrixWorld(t){super.updateMatrixWorld(t),this.bindMode===Yc?this.bindMatrixInverse.copy(this.matrixWorld).invert():this.bindMode===Up?this.bindMatrixInverse.copy(this.bindMatrix).invert():console.warn("THREE.SkinnedMesh: Unrecognized bindMode: "+this.bindMode)}applyBoneTransform(t,e){const i=this.skeleton,s=this.geometry;yu.fromBufferAttribute(s.attributes.skinIndex,t),Mu.fromBufferAttribute(s.attributes.skinWeight,t),bu.copy(e).applyMatrix4(this.bindMatrix),e.set(0,0,0);for(let r=0;r<4;r++){const a=Mu.getComponent(r);if(a!==0){const o=yu.getComponent(r);Su.multiplyMatrices(i.bones[o].matrixWorld,i.boneInverses[o]),e.addScaledVector(Om.copy(bu).applyMatrix4(Su),a)}}return e.applyMatrix4(this.bindMatrixInverse)}}class fd extends Ve{constructor(){super(),this.isBone=!0,this.type="Bone"}}class pd extends ln{constructor(t=null,e=1,i=1,s,r,a,o,l,c=An,u=An,h,d){super(null,a,o,l,c,u,s,r,h,d),this.isDataTexture=!0,this.image={data:t,width:e,height:i},this.generateMipmaps=!1,this.flipY=!1,this.unpackAlignment=1}}const Tu=new Kt,zm=new Kt;class fc{constructor(t=[],e=[]){this.uuid=Ui(),this.bones=t.slice(0),this.boneInverses=e,this.boneMatrices=null,this.boneTexture=null,this.init()}init(){const t=this.bones,e=this.boneInverses;if(this.boneMatrices=new Float32Array(t.length*16),e.length===0)this.calculateInverses();else if(t.length!==e.length){console.warn("THREE.Skeleton: Number of inverse bone matrices does not match amount of bones."),this.boneInverses=[];for(let i=0,s=this.bones.length;i<s;i++)this.boneInverses.push(new Kt)}}calculateInverses(){this.boneInverses.length=0;for(let t=0,e=this.bones.length;t<e;t++){const i=new Kt;this.bones[t]&&i.copy(this.bones[t].matrixWorld).invert(),this.boneInverses.push(i)}}pose(){for(let t=0,e=this.bones.length;t<e;t++){const i=this.bones[t];i&&i.matrixWorld.copy(this.boneInverses[t]).invert()}for(let t=0,e=this.bones.length;t<e;t++){const i=this.bones[t];i&&(i.parent&&i.parent.isBone?(i.matrix.copy(i.parent.matrixWorld).invert(),i.matrix.multiply(i.matrixWorld)):i.matrix.copy(i.matrixWorld),i.matrix.decompose(i.position,i.quaternion,i.scale))}}update(){const t=this.bones,e=this.boneInverses,i=this.boneMatrices,s=this.boneTexture;for(let r=0,a=t.length;r<a;r++){const o=t[r]?t[r].matrixWorld:zm;Tu.multiplyMatrices(o,e[r]),Tu.toArray(i,r*16)}s!==null&&(s.needsUpdate=!0)}clone(){return new fc(this.bones,this.boneInverses)}computeBoneTexture(){let t=Math.sqrt(this.bones.length*4);t=Math.ceil(t/4)*4,t=Math.max(t,4);const e=new Float32Array(t*t*4);e.set(this.boneMatrices);const i=new pd(e,t,t,Pn,jn);return i.needsUpdate=!0,this.boneMatrices=e,this.boneTexture=i,this}getBoneByName(t){for(let e=0,i=this.bones.length;e<i;e++){const s=this.bones[e];if(s.name===t)return s}}dispose(){this.boneTexture!==null&&(this.boneTexture.dispose(),this.boneTexture=null)}fromJSON(t,e){this.uuid=t.uuid;for(let i=0,s=t.bones.length;i<s;i++){const r=t.bones[i];let a=e[r];a===void 0&&(console.warn("THREE.Skeleton: No bone found with UUID:",r),a=new fd),this.bones.push(a),this.boneInverses.push(new Kt().fromArray(t.boneInverses[i]))}return this.init(),this}toJSON(){const t={metadata:{version:4.6,type:"Skeleton",generator:"Skeleton.toJSON"},bones:[],boneInverses:[]};t.uuid=this.uuid;const e=this.bones,i=this.boneInverses;for(let s=0,r=e.length;s<r;s++){const a=e[s];t.bones.push(a.uuid);const o=i[s];t.boneInverses.push(o.toArray())}return t}}const Bo=new O,Vm=new O,Hm=new le;class wi{constructor(t=new O(1,0,0),e=0){this.isPlane=!0,this.normal=t,this.constant=e}set(t,e){return this.normal.copy(t),this.constant=e,this}setComponents(t,e,i,s){return this.normal.set(t,e,i),this.constant=s,this}setFromNormalAndCoplanarPoint(t,e){return this.normal.copy(t),this.constant=-e.dot(this.normal),this}setFromCoplanarPoints(t,e,i){const s=Bo.subVectors(i,e).cross(Vm.subVectors(t,e)).normalize();return this.setFromNormalAndCoplanarPoint(s,t),this}copy(t){return this.normal.copy(t.normal),this.constant=t.constant,this}normalize(){const t=1/this.normal.length();return this.normal.multiplyScalar(t),this.constant*=t,this}negate(){return this.constant*=-1,this.normal.negate(),this}distanceToPoint(t){return this.normal.dot(t)+this.constant}distanceToSphere(t){return this.distanceToPoint(t.center)-t.radius}projectPoint(t,e){return e.copy(t).addScaledVector(this.normal,-this.distanceToPoint(t))}intersectLine(t,e){const i=t.delta(Bo),s=this.normal.dot(i);if(s===0)return this.distanceToPoint(t.start)===0?e.copy(t.start):null;const r=-(t.start.dot(this.normal)+this.constant)/s;return r<0||r>1?null:e.copy(t.start).addScaledVector(i,r)}intersectsLine(t){const e=this.distanceToPoint(t.start),i=this.distanceToPoint(t.end);return e<0&&i>0||i<0&&e>0}intersectsBox(t){return t.intersectsPlane(this)}intersectsSphere(t){return t.intersectsPlane(this)}coplanarPoint(t){return t.copy(this.normal).multiplyScalar(-this.constant)}applyMatrix4(t,e){const i=e||Hm.getNormalMatrix(t),s=this.coplanarPoint(Bo).applyMatrix4(t),r=this.normal.applyMatrix3(i).normalize();return this.constant=-s.dot(r),this}translate(t){return this.constant-=t.dot(this.normal),this}equals(t){return t.normal.equals(this.normal)&&t.constant===this.constant}clone(){return new this.constructor().copy(this)}}const Gi=new as,aa=new O;class pc{constructor(t=new wi,e=new wi,i=new wi,s=new wi,r=new wi,a=new wi){this.planes=[t,e,i,s,r,a]}set(t,e,i,s,r,a){const o=this.planes;return o[0].copy(t),o[1].copy(e),o[2].copy(i),o[3].copy(s),o[4].copy(r),o[5].copy(a),this}copy(t){const e=this.planes;for(let i=0;i<6;i++)e[i].copy(t.planes[i]);return this}setFromProjectionMatrix(t,e=li){const i=this.planes,s=t.elements,r=s[0],a=s[1],o=s[2],l=s[3],c=s[4],u=s[5],h=s[6],d=s[7],p=s[8],g=s[9],_=s[10],m=s[11],f=s[12],P=s[13],L=s[14],w=s[15];if(i[0].setComponents(l-r,d-c,m-p,w-f).normalize(),i[1].setComponents(l+r,d+c,m+p,w+f).normalize(),i[2].setComponents(l+a,d+u,m+g,w+P).normalize(),i[3].setComponents(l-a,d-u,m-g,w-P).normalize(),i[4].setComponents(l-o,d-h,m-_,w-L).normalize(),e===li)i[5].setComponents(l+o,d+h,m+_,w+L).normalize();else if(e===Oa)i[5].setComponents(o,h,_,L).normalize();else throw new Error("THREE.Frustum.setFromProjectionMatrix(): Invalid coordinate system: "+e);return this}intersectsObject(t){if(t.boundingSphere!==void 0)t.boundingSphere===null&&t.computeBoundingSphere(),Gi.copy(t.boundingSphere).applyMatrix4(t.matrixWorld);else{const e=t.geometry;e.boundingSphere===null&&e.computeBoundingSphere(),Gi.copy(e.boundingSphere).applyMatrix4(t.matrixWorld)}return this.intersectsSphere(Gi)}intersectsSprite(t){return Gi.center.set(0,0,0),Gi.radius=.7071067811865476,Gi.applyMatrix4(t.matrixWorld),this.intersectsSphere(Gi)}intersectsSphere(t){const e=this.planes,i=t.center,s=-t.radius;for(let r=0;r<6;r++)if(e[r].distanceToPoint(i)<s)return!1;return!0}intersectsBox(t){const e=this.planes;for(let i=0;i<6;i++){const s=e[i];if(aa.x=s.normal.x>0?t.max.x:t.min.x,aa.y=s.normal.y>0?t.max.y:t.min.y,aa.z=s.normal.z>0?t.max.z:t.min.z,s.distanceToPoint(aa)<0)return!1}return!0}containsPoint(t){const e=this.planes;for(let i=0;i<6;i++)if(e[i].distanceToPoint(t)<0)return!1;return!0}clone(){return new this.constructor().copy(this)}}class Ci extends di{constructor(t){super(),this.isLineBasicMaterial=!0,this.type="LineBasicMaterial",this.color=new ne(16777215),this.map=null,this.linewidth=1,this.linecap="round",this.linejoin="round",this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.linewidth=t.linewidth,this.linecap=t.linecap,this.linejoin=t.linejoin,this.fog=t.fog,this}}const Ba=new O,za=new O,wu=new Kt,pr=new Pr,oa=new as,zo=new O,Au=new O;class mc extends Ve{constructor(t=new $e,e=new Ci){super(),this.isLine=!0,this.type="Line",this.geometry=t,this.material=e,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,i=[0];for(let s=1,r=e.count;s<r;s++)Ba.fromBufferAttribute(e,s-1),za.fromBufferAttribute(e,s),i[s]=i[s-1],i[s]+=Ba.distanceTo(za);t.setAttribute("lineDistance",new Ae(i,1))}else console.warn("THREE.Line.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}raycast(t,e){const i=this.geometry,s=this.matrixWorld,r=t.params.Line.threshold,a=i.drawRange;if(i.boundingSphere===null&&i.computeBoundingSphere(),oa.copy(i.boundingSphere),oa.applyMatrix4(s),oa.radius+=r,t.ray.intersectsSphere(oa)===!1)return;wu.copy(s).invert(),pr.copy(t.ray).applyMatrix4(wu);const o=r/((this.scale.x+this.scale.y+this.scale.z)/3),l=o*o,c=this.isLineSegments?2:1,u=i.index,d=i.attributes.position;if(u!==null){const p=Math.max(0,a.start),g=Math.min(u.count,a.start+a.count);for(let _=p,m=g-1;_<m;_+=c){const f=u.getX(_),P=u.getX(_+1),L=la(this,t,pr,l,f,P,_);L&&e.push(L)}if(this.isLineLoop){const _=u.getX(g-1),m=u.getX(p),f=la(this,t,pr,l,_,m,g-1);f&&e.push(f)}}else{const p=Math.max(0,a.start),g=Math.min(d.count,a.start+a.count);for(let _=p,m=g-1;_<m;_+=c){const f=la(this,t,pr,l,_,_+1,_);f&&e.push(f)}if(this.isLineLoop){const _=la(this,t,pr,l,g-1,p,g-1);_&&e.push(_)}}}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,a=s.length;r<a;r++){const o=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=r}}}}}function la(n,t,e,i,s,r,a){const o=n.geometry.attributes.position;if(Ba.fromBufferAttribute(o,s),za.fromBufferAttribute(o,r),e.distanceSqToSegment(Ba,za,zo,Au)>i)return;zo.applyMatrix4(n.matrixWorld);const c=t.ray.origin.distanceTo(zo);if(!(c<t.near||c>t.far))return{distance:c,point:Au.clone().applyMatrix4(n.matrixWorld),index:a,face:null,faceIndex:null,barycoord:null,object:n}}const Cu=new O,Ru=new O;class _r extends mc{constructor(t,e){super(t,e),this.isLineSegments=!0,this.type="LineSegments"}computeLineDistances(){const t=this.geometry;if(t.index===null){const e=t.attributes.position,i=[];for(let s=0,r=e.count;s<r;s+=2)Cu.fromBufferAttribute(e,s),Ru.fromBufferAttribute(e,s+1),i[s]=s===0?0:i[s-1],i[s+1]=i[s]+Cu.distanceTo(Ru);t.setAttribute("lineDistance",new Ae(i,1))}else console.warn("THREE.LineSegments.computeLineDistances(): Computation only possible with non-indexed BufferGeometry.");return this}}class md extends di{constructor(t){super(),this.isPointsMaterial=!0,this.type="PointsMaterial",this.color=new ne(16777215),this.map=null,this.alphaMap=null,this.size=1,this.sizeAttenuation=!0,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.alphaMap=t.alphaMap,this.size=t.size,this.sizeAttenuation=t.sizeAttenuation,this.fog=t.fog,this}}const Pu=new Kt,Gl=new Pr,ca=new as,ua=new O;class Gm extends Ve{constructor(t=new $e,e=new md){super(),this.isPoints=!0,this.type="Points",this.geometry=t,this.material=e,this.morphTargetDictionary=void 0,this.morphTargetInfluences=void 0,this.updateMorphTargets()}copy(t,e){return super.copy(t,e),this.material=Array.isArray(t.material)?t.material.slice():t.material,this.geometry=t.geometry,this}raycast(t,e){const i=this.geometry,s=this.matrixWorld,r=t.params.Points.threshold,a=i.drawRange;if(i.boundingSphere===null&&i.computeBoundingSphere(),ca.copy(i.boundingSphere),ca.applyMatrix4(s),ca.radius+=r,t.ray.intersectsSphere(ca)===!1)return;Pu.copy(s).invert(),Gl.copy(t.ray).applyMatrix4(Pu);const o=r/((this.scale.x+this.scale.y+this.scale.z)/3),l=o*o,c=i.index,h=i.attributes.position;if(c!==null){const d=Math.max(0,a.start),p=Math.min(c.count,a.start+a.count);for(let g=d,_=p;g<_;g++){const m=c.getX(g);ua.fromBufferAttribute(h,m),Lu(ua,m,l,s,t,e,this)}}else{const d=Math.max(0,a.start),p=Math.min(h.count,a.start+a.count);for(let g=d,_=p;g<_;g++)ua.fromBufferAttribute(h,g),Lu(ua,g,l,s,t,e,this)}}updateMorphTargets(){const e=this.geometry.morphAttributes,i=Object.keys(e);if(i.length>0){const s=e[i[0]];if(s!==void 0){this.morphTargetInfluences=[],this.morphTargetDictionary={};for(let r=0,a=s.length;r<a;r++){const o=s[r].name||String(r);this.morphTargetInfluences.push(0),this.morphTargetDictionary[o]=r}}}}}function Lu(n,t,e,i,s,r,a){const o=Gl.distanceSqToPoint(n);if(o<e){const l=new O;Gl.closestPointToPoint(n,l),l.applyMatrix4(i);const c=s.ray.origin.distanceTo(l);if(c<s.near||c>s.far)return;r.push({distance:c,distanceToRay:Math.sqrt(o),point:l,index:t,face:null,faceIndex:null,barycoord:null,object:a})}}class gd extends ln{constructor(t,e,i,s,r,a,o,l,c,u=Os){if(u!==Os&&u!==qs)throw new Error("DepthTexture format must be either THREE.DepthFormat or THREE.DepthStencilFormat");i===void 0&&u===Os&&(i=ts),i===void 0&&u===qs&&(i=Xs),super(null,s,r,a,o,l,u,i,c),this.isDepthTexture=!0,this.image={width:t,height:e},this.magFilter=o!==void 0?o:An,this.minFilter=l!==void 0?l:An,this.flipY=!1,this.generateMipmaps=!1,this.compareFunction=null}copy(t){return super.copy(t),this.source=new dc(Object.assign({},t.image)),this.compareFunction=t.compareFunction,this}toJSON(t){const e=super.toJSON(t);return this.compareFunction!==null&&(e.compareFunction=this.compareFunction),e}}class Lr extends $e{constructor(t=1,e=1,i=1,s=32,r=1,a=!1,o=0,l=Math.PI*2){super(),this.type="CylinderGeometry",this.parameters={radiusTop:t,radiusBottom:e,height:i,radialSegments:s,heightSegments:r,openEnded:a,thetaStart:o,thetaLength:l};const c=this;s=Math.floor(s),r=Math.floor(r);const u=[],h=[],d=[],p=[];let g=0;const _=[],m=i/2;let f=0;P(),a===!1&&(t>0&&L(!0),e>0&&L(!1)),this.setIndex(u),this.setAttribute("position",new Ae(h,3)),this.setAttribute("normal",new Ae(d,3)),this.setAttribute("uv",new Ae(p,2));function P(){const w=new O,N=new O;let F=0;const U=(e-t)/i;for(let k=0;k<=r;k++){const y=[],M=k/r,I=M*(e-t)+t;for(let V=0;V<=s;V++){const W=V/s,Z=W*l+o,et=Math.sin(Z),z=Math.cos(Z);N.x=I*et,N.y=-M*i+m,N.z=I*z,h.push(N.x,N.y,N.z),w.set(et,U,z).normalize(),d.push(w.x,w.y,w.z),p.push(W,1-M),y.push(g++)}_.push(y)}for(let k=0;k<s;k++)for(let y=0;y<r;y++){const M=_[y][k],I=_[y+1][k],V=_[y+1][k+1],W=_[y][k+1];(t>0||y!==0)&&(u.push(M,I,W),F+=3),(e>0||y!==r-1)&&(u.push(I,V,W),F+=3)}c.addGroup(f,F,0),f+=F}function L(w){const N=g,F=new Yt,U=new O;let k=0;const y=w===!0?t:e,M=w===!0?1:-1;for(let V=1;V<=s;V++)h.push(0,m*M,0),d.push(0,M,0),p.push(.5,.5),g++;const I=g;for(let V=0;V<=s;V++){const Z=V/s*l+o,et=Math.cos(Z),z=Math.sin(Z);U.x=y*z,U.y=m*M,U.z=y*et,h.push(U.x,U.y,U.z),d.push(0,M,0),F.x=et*.5+.5,F.y=z*.5*M+.5,p.push(F.x,F.y),g++}for(let V=0;V<s;V++){const W=N+V,Z=I+V;w===!0?u.push(Z,Z+1,W):u.push(Z+1,Z,W),k+=3}c.addGroup(f,k,w===!0?1:2),f+=k}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Lr(t.radiusTop,t.radiusBottom,t.height,t.radialSegments,t.heightSegments,t.openEnded,t.thetaStart,t.thetaLength)}}const ha=new O,da=new O,Vo=new O,fa=new Rn;class Nu extends $e{constructor(t=null,e=1){if(super(),this.type="EdgesGeometry",this.parameters={geometry:t,thresholdAngle:e},t!==null){const s=Math.pow(10,4),r=Math.cos(Bs*e),a=t.getIndex(),o=t.getAttribute("position"),l=a?a.count:o.count,c=[0,0,0],u=["a","b","c"],h=new Array(3),d={},p=[];for(let g=0;g<l;g+=3){a?(c[0]=a.getX(g),c[1]=a.getX(g+1),c[2]=a.getX(g+2)):(c[0]=g,c[1]=g+1,c[2]=g+2);const{a:_,b:m,c:f}=fa;if(_.fromBufferAttribute(o,c[0]),m.fromBufferAttribute(o,c[1]),f.fromBufferAttribute(o,c[2]),fa.getNormal(Vo),h[0]=`${Math.round(_.x*s)},${Math.round(_.y*s)},${Math.round(_.z*s)}`,h[1]=`${Math.round(m.x*s)},${Math.round(m.y*s)},${Math.round(m.z*s)}`,h[2]=`${Math.round(f.x*s)},${Math.round(f.y*s)},${Math.round(f.z*s)}`,!(h[0]===h[1]||h[1]===h[2]||h[2]===h[0]))for(let P=0;P<3;P++){const L=(P+1)%3,w=h[P],N=h[L],F=fa[u[P]],U=fa[u[L]],k=`${w}_${N}`,y=`${N}_${w}`;y in d&&d[y]?(Vo.dot(d[y].normal)<=r&&(p.push(F.x,F.y,F.z),p.push(U.x,U.y,U.z)),d[y]=null):k in d||(d[k]={index0:c[P],index1:c[L],normal:Vo.clone()})}}for(const g in d)if(d[g]){const{index0:_,index1:m}=d[g];ha.fromBufferAttribute(o,_),da.fromBufferAttribute(o,m),p.push(ha.x,ha.y,ha.z),p.push(da.x,da.y,da.z)}this.setAttribute("position",new Ae(p,3))}}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}}class Nr extends $e{constructor(t=1,e=1,i=1,s=1){super(),this.type="PlaneGeometry",this.parameters={width:t,height:e,widthSegments:i,heightSegments:s};const r=t/2,a=e/2,o=Math.floor(i),l=Math.floor(s),c=o+1,u=l+1,h=t/o,d=e/l,p=[],g=[],_=[],m=[];for(let f=0;f<u;f++){const P=f*d-a;for(let L=0;L<c;L++){const w=L*h-r;g.push(w,-P,0),_.push(0,0,1),m.push(L/o),m.push(1-f/l)}}for(let f=0;f<l;f++)for(let P=0;P<o;P++){const L=P+c*f,w=P+c*(f+1),N=P+1+c*(f+1),F=P+1+c*f;p.push(L,w,F),p.push(w,N,F)}this.setIndex(p),this.setAttribute("position",new Ae(g,3)),this.setAttribute("normal",new Ae(_,3)),this.setAttribute("uv",new Ae(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Nr(t.width,t.height,t.widthSegments,t.heightSegments)}}class gc extends $e{constructor(t=.5,e=1,i=32,s=1,r=0,a=Math.PI*2){super(),this.type="RingGeometry",this.parameters={innerRadius:t,outerRadius:e,thetaSegments:i,phiSegments:s,thetaStart:r,thetaLength:a},i=Math.max(3,i),s=Math.max(1,s);const o=[],l=[],c=[],u=[];let h=t;const d=(e-t)/s,p=new O,g=new Yt;for(let _=0;_<=s;_++){for(let m=0;m<=i;m++){const f=r+m/i*a;p.x=h*Math.cos(f),p.y=h*Math.sin(f),l.push(p.x,p.y,p.z),c.push(0,0,1),g.x=(p.x/e+1)/2,g.y=(p.y/e+1)/2,u.push(g.x,g.y)}h+=d}for(let _=0;_<s;_++){const m=_*(i+1);for(let f=0;f<i;f++){const P=f+m,L=P,w=P+i+1,N=P+i+2,F=P+1;o.push(L,w,F),o.push(w,N,F)}}this.setIndex(o),this.setAttribute("position",new Ae(l,3)),this.setAttribute("normal",new Ae(c,3)),this.setAttribute("uv",new Ae(u,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new gc(t.innerRadius,t.outerRadius,t.thetaSegments,t.phiSegments,t.thetaStart,t.thetaLength)}}class Tr extends $e{constructor(t=1,e=32,i=16,s=0,r=Math.PI*2,a=0,o=Math.PI){super(),this.type="SphereGeometry",this.parameters={radius:t,widthSegments:e,heightSegments:i,phiStart:s,phiLength:r,thetaStart:a,thetaLength:o},e=Math.max(3,Math.floor(e)),i=Math.max(2,Math.floor(i));const l=Math.min(a+o,Math.PI);let c=0;const u=[],h=new O,d=new O,p=[],g=[],_=[],m=[];for(let f=0;f<=i;f++){const P=[],L=f/i;let w=0;f===0&&a===0?w=.5/e:f===i&&l===Math.PI&&(w=-.5/e);for(let N=0;N<=e;N++){const F=N/e;h.x=-t*Math.cos(s+F*r)*Math.sin(a+L*o),h.y=t*Math.cos(a+L*o),h.z=t*Math.sin(s+F*r)*Math.sin(a+L*o),g.push(h.x,h.y,h.z),d.copy(h).normalize(),_.push(d.x,d.y,d.z),m.push(F+w,1-L),P.push(c++)}u.push(P)}for(let f=0;f<i;f++)for(let P=0;P<e;P++){const L=u[f][P+1],w=u[f][P],N=u[f+1][P],F=u[f+1][P+1];(f!==0||a>0)&&p.push(L,w,F),(f!==i-1||l<Math.PI)&&p.push(w,N,F)}this.setIndex(p),this.setAttribute("position",new Ae(g,3)),this.setAttribute("normal",new Ae(_,3)),this.setAttribute("uv",new Ae(m,2))}copy(t){return super.copy(t),this.parameters=Object.assign({},t.parameters),this}static fromJSON(t){return new Tr(t.radius,t.widthSegments,t.heightSegments,t.phiStart,t.phiLength,t.thetaStart,t.thetaLength)}}class Fn extends di{constructor(t){super(),this.isMeshStandardMaterial=!0,this.type="MeshStandardMaterial",this.defines={STANDARD:""},this.color=new ne(16777215),this.roughness=1,this.metalness=0,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new ne(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Ja,this.normalScale=new Yt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.roughnessMap=null,this.metalnessMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yn,this.envMapIntensity=1,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.defines={STANDARD:""},this.color.copy(t.color),this.roughness=t.roughness,this.metalness=t.metalness,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.roughnessMap=t.roughnessMap,this.metalnessMap=t.metalnessMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.envMapIntensity=t.envMapIntensity,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class Wm extends Fn{constructor(t){super(),this.isMeshPhysicalMaterial=!0,this.defines={STANDARD:"",PHYSICAL:""},this.type="MeshPhysicalMaterial",this.anisotropyRotation=0,this.anisotropyMap=null,this.clearcoatMap=null,this.clearcoatRoughness=0,this.clearcoatRoughnessMap=null,this.clearcoatNormalScale=new Yt(1,1),this.clearcoatNormalMap=null,this.ior=1.5,Object.defineProperty(this,"reflectivity",{get:function(){return he(2.5*(this.ior-1)/(this.ior+1),0,1)},set:function(e){this.ior=(1+.4*e)/(1-.4*e)}}),this.iridescenceMap=null,this.iridescenceIOR=1.3,this.iridescenceThicknessRange=[100,400],this.iridescenceThicknessMap=null,this.sheenColor=new ne(0),this.sheenColorMap=null,this.sheenRoughness=1,this.sheenRoughnessMap=null,this.transmissionMap=null,this.thickness=0,this.thicknessMap=null,this.attenuationDistance=1/0,this.attenuationColor=new ne(1,1,1),this.specularIntensity=1,this.specularIntensityMap=null,this.specularColor=new ne(1,1,1),this.specularColorMap=null,this._anisotropy=0,this._clearcoat=0,this._dispersion=0,this._iridescence=0,this._sheen=0,this._transmission=0,this.setValues(t)}get anisotropy(){return this._anisotropy}set anisotropy(t){this._anisotropy>0!=t>0&&this.version++,this._anisotropy=t}get clearcoat(){return this._clearcoat}set clearcoat(t){this._clearcoat>0!=t>0&&this.version++,this._clearcoat=t}get iridescence(){return this._iridescence}set iridescence(t){this._iridescence>0!=t>0&&this.version++,this._iridescence=t}get dispersion(){return this._dispersion}set dispersion(t){this._dispersion>0!=t>0&&this.version++,this._dispersion=t}get sheen(){return this._sheen}set sheen(t){this._sheen>0!=t>0&&this.version++,this._sheen=t}get transmission(){return this._transmission}set transmission(t){this._transmission>0!=t>0&&this.version++,this._transmission=t}copy(t){return super.copy(t),this.defines={STANDARD:"",PHYSICAL:""},this.anisotropy=t.anisotropy,this.anisotropyRotation=t.anisotropyRotation,this.anisotropyMap=t.anisotropyMap,this.clearcoat=t.clearcoat,this.clearcoatMap=t.clearcoatMap,this.clearcoatRoughness=t.clearcoatRoughness,this.clearcoatRoughnessMap=t.clearcoatRoughnessMap,this.clearcoatNormalMap=t.clearcoatNormalMap,this.clearcoatNormalScale.copy(t.clearcoatNormalScale),this.dispersion=t.dispersion,this.ior=t.ior,this.iridescence=t.iridescence,this.iridescenceMap=t.iridescenceMap,this.iridescenceIOR=t.iridescenceIOR,this.iridescenceThicknessRange=[...t.iridescenceThicknessRange],this.iridescenceThicknessMap=t.iridescenceThicknessMap,this.sheen=t.sheen,this.sheenColor.copy(t.sheenColor),this.sheenColorMap=t.sheenColorMap,this.sheenRoughness=t.sheenRoughness,this.sheenRoughnessMap=t.sheenRoughnessMap,this.transmission=t.transmission,this.transmissionMap=t.transmissionMap,this.thickness=t.thickness,this.thicknessMap=t.thicknessMap,this.attenuationDistance=t.attenuationDistance,this.attenuationColor.copy(t.attenuationColor),this.specularIntensity=t.specularIntensity,this.specularIntensityMap=t.specularIntensityMap,this.specularColor.copy(t.specularColor),this.specularColorMap=t.specularColorMap,this}}class br extends di{constructor(t){super(),this.isMeshPhongMaterial=!0,this.type="MeshPhongMaterial",this.color=new ne(16777215),this.specular=new ne(1118481),this.shininess=30,this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new ne(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Ja,this.normalScale=new Yt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yn,this.combine=Ya,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.specular.copy(t.specular),this.shininess=t.shininess,this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class $m extends di{constructor(t){super(),this.isMeshLambertMaterial=!0,this.type="MeshLambertMaterial",this.color=new ne(16777215),this.map=null,this.lightMap=null,this.lightMapIntensity=1,this.aoMap=null,this.aoMapIntensity=1,this.emissive=new ne(0),this.emissiveIntensity=1,this.emissiveMap=null,this.bumpMap=null,this.bumpScale=1,this.normalMap=null,this.normalMapType=Ja,this.normalScale=new Yt(1,1),this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.specularMap=null,this.alphaMap=null,this.envMap=null,this.envMapRotation=new yn,this.combine=Ya,this.reflectivity=1,this.refractionRatio=.98,this.wireframe=!1,this.wireframeLinewidth=1,this.wireframeLinecap="round",this.wireframeLinejoin="round",this.flatShading=!1,this.fog=!0,this.setValues(t)}copy(t){return super.copy(t),this.color.copy(t.color),this.map=t.map,this.lightMap=t.lightMap,this.lightMapIntensity=t.lightMapIntensity,this.aoMap=t.aoMap,this.aoMapIntensity=t.aoMapIntensity,this.emissive.copy(t.emissive),this.emissiveMap=t.emissiveMap,this.emissiveIntensity=t.emissiveIntensity,this.bumpMap=t.bumpMap,this.bumpScale=t.bumpScale,this.normalMap=t.normalMap,this.normalMapType=t.normalMapType,this.normalScale.copy(t.normalScale),this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.specularMap=t.specularMap,this.alphaMap=t.alphaMap,this.envMap=t.envMap,this.envMapRotation.copy(t.envMapRotation),this.combine=t.combine,this.reflectivity=t.reflectivity,this.refractionRatio=t.refractionRatio,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this.wireframeLinecap=t.wireframeLinecap,this.wireframeLinejoin=t.wireframeLinejoin,this.flatShading=t.flatShading,this.fog=t.fog,this}}class jm extends di{constructor(t){super(),this.isMeshDepthMaterial=!0,this.type="MeshDepthMaterial",this.depthPacking=Op,this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.wireframe=!1,this.wireframeLinewidth=1,this.setValues(t)}copy(t){return super.copy(t),this.depthPacking=t.depthPacking,this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this.wireframe=t.wireframe,this.wireframeLinewidth=t.wireframeLinewidth,this}}class Xm extends di{constructor(t){super(),this.isMeshDistanceMaterial=!0,this.type="MeshDistanceMaterial",this.map=null,this.alphaMap=null,this.displacementMap=null,this.displacementScale=1,this.displacementBias=0,this.setValues(t)}copy(t){return super.copy(t),this.map=t.map,this.alphaMap=t.alphaMap,this.displacementMap=t.displacementMap,this.displacementScale=t.displacementScale,this.displacementBias=t.displacementBias,this}}function pa(n,t,e){return!n||!e&&n.constructor===t?n:typeof t.BYTES_PER_ELEMENT=="number"?new t(n):Array.prototype.slice.call(n)}function qm(n){return ArrayBuffer.isView(n)&&!(n instanceof DataView)}function Ym(n){function t(s,r){return n[s]-n[r]}const e=n.length,i=new Array(e);for(let s=0;s!==e;++s)i[s]=s;return i.sort(t),i}function Du(n,t,e){const i=n.length,s=new n.constructor(i);for(let r=0,a=0;a!==i;++r){const o=e[r]*t;for(let l=0;l!==t;++l)s[a++]=n[o+l]}return s}function _d(n,t,e,i){let s=1,r=n[0];for(;r!==void 0&&r[i]===void 0;)r=n[s++];if(r===void 0)return;let a=r[i];if(a!==void 0)if(Array.isArray(a))do a=r[i],a!==void 0&&(t.push(r.time),e.push(...a)),r=n[s++];while(r!==void 0);else if(a.toArray!==void 0)do a=r[i],a!==void 0&&(t.push(r.time),a.toArray(e,e.length)),r=n[s++];while(r!==void 0);else do a=r[i],a!==void 0&&(t.push(r.time),e.push(a)),r=n[s++];while(r!==void 0)}class Za{constructor(t,e,i,s){this.parameterPositions=t,this._cachedIndex=0,this.resultBuffer=s!==void 0?s:new e.constructor(i),this.sampleValues=e,this.valueSize=i,this.settings=null,this.DefaultSettings_={}}evaluate(t){const e=this.parameterPositions;let i=this._cachedIndex,s=e[i],r=e[i-1];n:{t:{let a;e:{i:if(!(t<s)){for(let o=i+2;;){if(s===void 0){if(t<r)break i;return i=e.length,this._cachedIndex=i,this.copySampleValue_(i-1)}if(i===o)break;if(r=s,s=e[++i],t<s)break t}a=e.length;break e}if(!(t>=r)){const o=e[1];t<o&&(i=2,r=o);for(let l=i-2;;){if(r===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(i===l)break;if(s=r,r=e[--i-1],t>=r)break t}a=i,i=0;break e}break n}for(;i<a;){const o=i+a>>>1;t<e[o]?a=o:i=o+1}if(s=e[i],r=e[i-1],r===void 0)return this._cachedIndex=0,this.copySampleValue_(0);if(s===void 0)return i=e.length,this._cachedIndex=i,this.copySampleValue_(i-1)}this._cachedIndex=i,this.intervalChanged_(i,r,s)}return this.interpolate_(i,r,t,s)}getSettings_(){return this.settings||this.DefaultSettings_}copySampleValue_(t){const e=this.resultBuffer,i=this.sampleValues,s=this.valueSize,r=t*s;for(let a=0;a!==s;++a)e[a]=i[r+a];return e}interpolate_(){throw new Error("call to abstract method")}intervalChanged_(){}}class Km extends Za{constructor(t,e,i,s){super(t,e,i,s),this._weightPrev=-0,this._offsetPrev=-0,this._weightNext=-0,this._offsetNext=-0,this.DefaultSettings_={endingStart:Kc,endingEnd:Kc}}intervalChanged_(t,e,i){const s=this.parameterPositions;let r=t-2,a=t+1,o=s[r],l=s[a];if(o===void 0)switch(this.getSettings_().endingStart){case Jc:r=t,o=2*e-i;break;case Zc:r=s.length-2,o=e+s[r]-s[r+1];break;default:r=t,o=i}if(l===void 0)switch(this.getSettings_().endingEnd){case Jc:a=t,l=2*i-e;break;case Zc:a=1,l=i+s[1]-s[0];break;default:a=t-1,l=e}const c=(i-e)*.5,u=this.valueSize;this._weightPrev=c/(e-o),this._weightNext=c/(l-i),this._offsetPrev=r*u,this._offsetNext=a*u}interpolate_(t,e,i,s){const r=this.resultBuffer,a=this.sampleValues,o=this.valueSize,l=t*o,c=l-o,u=this._offsetPrev,h=this._offsetNext,d=this._weightPrev,p=this._weightNext,g=(i-e)/(s-e),_=g*g,m=_*g,f=-d*m+2*d*_-d*g,P=(1+d)*m+(-1.5-2*d)*_+(-.5+d)*g+1,L=(-1-p)*m+(1.5+p)*_+.5*g,w=p*m-p*_;for(let N=0;N!==o;++N)r[N]=f*a[u+N]+P*a[c+N]+L*a[l+N]+w*a[h+N];return r}}class Jm extends Za{constructor(t,e,i,s){super(t,e,i,s)}interpolate_(t,e,i,s){const r=this.resultBuffer,a=this.sampleValues,o=this.valueSize,l=t*o,c=l-o,u=(i-e)/(s-e),h=1-u;for(let d=0;d!==o;++d)r[d]=a[c+d]*h+a[l+d]*u;return r}}class Zm extends Za{constructor(t,e,i,s){super(t,e,i,s)}interpolate_(t){return this.copySampleValue_(t-1)}}class Yn{constructor(t,e,i,s){if(t===void 0)throw new Error("THREE.KeyframeTrack: track name is undefined");if(e===void 0||e.length===0)throw new Error("THREE.KeyframeTrack: no keyframes in track named "+t);this.name=t,this.times=pa(e,this.TimeBufferType),this.values=pa(i,this.ValueBufferType),this.setInterpolation(s||this.DefaultInterpolation)}static toJSON(t){const e=t.constructor;let i;if(e.toJSON!==this.toJSON)i=e.toJSON(t);else{i={name:t.name,times:pa(t.times,Array),values:pa(t.values,Array)};const s=t.getInterpolation();s!==t.DefaultInterpolation&&(i.interpolation=s)}return i.type=t.ValueTypeName,i}InterpolantFactoryMethodDiscrete(t){return new Zm(this.times,this.values,this.getValueSize(),t)}InterpolantFactoryMethodLinear(t){return new Jm(this.times,this.values,this.getValueSize(),t)}InterpolantFactoryMethodSmooth(t){return new Km(this.times,this.values,this.getValueSize(),t)}setInterpolation(t){let e;switch(t){case ka:e=this.InterpolantFactoryMethodDiscrete;break;case Hl:e=this.InterpolantFactoryMethodLinear;break;case go:e=this.InterpolantFactoryMethodSmooth;break}if(e===void 0){const i="unsupported interpolation for "+this.ValueTypeName+" keyframe track named "+this.name;if(this.createInterpolant===void 0)if(t!==this.DefaultInterpolation)this.setInterpolation(this.DefaultInterpolation);else throw new Error(i);return console.warn("THREE.KeyframeTrack:",i),this}return this.createInterpolant=e,this}getInterpolation(){switch(this.createInterpolant){case this.InterpolantFactoryMethodDiscrete:return ka;case this.InterpolantFactoryMethodLinear:return Hl;case this.InterpolantFactoryMethodSmooth:return go}}getValueSize(){return this.values.length/this.times.length}shift(t){if(t!==0){const e=this.times;for(let i=0,s=e.length;i!==s;++i)e[i]+=t}return this}scale(t){if(t!==1){const e=this.times;for(let i=0,s=e.length;i!==s;++i)e[i]*=t}return this}trim(t,e){const i=this.times,s=i.length;let r=0,a=s-1;for(;r!==s&&i[r]<t;)++r;for(;a!==-1&&i[a]>e;)--a;if(++a,r!==0||a!==s){r>=a&&(a=Math.max(a,1),r=a-1);const o=this.getValueSize();this.times=i.slice(r,a),this.values=this.values.slice(r*o,a*o)}return this}validate(){let t=!0;const e=this.getValueSize();e-Math.floor(e)!==0&&(console.error("THREE.KeyframeTrack: Invalid value size in track.",this),t=!1);const i=this.times,s=this.values,r=i.length;r===0&&(console.error("THREE.KeyframeTrack: Track is empty.",this),t=!1);let a=null;for(let o=0;o!==r;o++){const l=i[o];if(typeof l=="number"&&isNaN(l)){console.error("THREE.KeyframeTrack: Time is not a valid number.",this,o,l),t=!1;break}if(a!==null&&a>l){console.error("THREE.KeyframeTrack: Out of order keys.",this,o,l,a),t=!1;break}a=l}if(s!==void 0&&qm(s))for(let o=0,l=s.length;o!==l;++o){const c=s[o];if(isNaN(c)){console.error("THREE.KeyframeTrack: Value is not a valid number.",this,o,c),t=!1;break}}return t}optimize(){const t=this.times.slice(),e=this.values.slice(),i=this.getValueSize(),s=this.getInterpolation()===go,r=t.length-1;let a=1;for(let o=1;o<r;++o){let l=!1;const c=t[o],u=t[o+1];if(c!==u&&(o!==1||c!==t[0]))if(s)l=!0;else{const h=o*i,d=h-i,p=h+i;for(let g=0;g!==i;++g){const _=e[h+g];if(_!==e[d+g]||_!==e[p+g]){l=!0;break}}}if(l){if(o!==a){t[a]=t[o];const h=o*i,d=a*i;for(let p=0;p!==i;++p)e[d+p]=e[h+p]}++a}}if(r>0){t[a]=t[r];for(let o=r*i,l=a*i,c=0;c!==i;++c)e[l+c]=e[o+c];++a}return a!==t.length?(this.times=t.slice(0,a),this.values=e.slice(0,a*i)):(this.times=t,this.values=e),this}clone(){const t=this.times.slice(),e=this.values.slice(),i=this.constructor,s=new i(this.name,t,e);return s.createInterpolant=this.createInterpolant,s}}Yn.prototype.TimeBufferType=Float32Array;Yn.prototype.ValueBufferType=Float32Array;Yn.prototype.DefaultInterpolation=Hl;class nr extends Yn{constructor(t,e,i){super(t,e,i)}}nr.prototype.ValueTypeName="bool";nr.prototype.ValueBufferType=Array;nr.prototype.DefaultInterpolation=ka;nr.prototype.InterpolantFactoryMethodLinear=void 0;nr.prototype.InterpolantFactoryMethodSmooth=void 0;class vd extends Yn{}vd.prototype.ValueTypeName="color";class Va extends Yn{}Va.prototype.ValueTypeName="number";class Qm extends Za{constructor(t,e,i,s){super(t,e,i,s)}interpolate_(t,e,i,s){const r=this.resultBuffer,a=this.sampleValues,o=this.valueSize,l=(i-e)/(s-e);let c=t*o;for(let u=c+o;c!==u;c+=4)Nn.slerpFlat(r,0,a,c-o,a,c,l);return r}}class Dr extends Yn{InterpolantFactoryMethodLinear(t){return new Qm(this.times,this.values,this.getValueSize(),t)}}Dr.prototype.ValueTypeName="quaternion";Dr.prototype.InterpolantFactoryMethodSmooth=void 0;class ir extends Yn{constructor(t,e,i){super(t,e,i)}}ir.prototype.ValueTypeName="string";ir.prototype.ValueBufferType=Array;ir.prototype.DefaultInterpolation=ka;ir.prototype.InterpolantFactoryMethodLinear=void 0;ir.prototype.InterpolantFactoryMethodSmooth=void 0;class Zs extends Yn{}Zs.prototype.ValueTypeName="vector";class Iu{constructor(t="",e=-1,i=[],s=Fp){this.name=t,this.tracks=i,this.duration=e,this.blendMode=s,this.uuid=Ui(),this.duration<0&&this.resetDuration()}static parse(t){const e=[],i=t.tracks,s=1/(t.fps||1);for(let a=0,o=i.length;a!==o;++a)e.push(eg(i[a]).scale(s));const r=new this(t.name,t.duration,e,t.blendMode);return r.uuid=t.uuid,r}static toJSON(t){const e=[],i=t.tracks,s={name:t.name,duration:t.duration,tracks:e,uuid:t.uuid,blendMode:t.blendMode};for(let r=0,a=i.length;r!==a;++r)e.push(Yn.toJSON(i[r]));return s}static CreateFromMorphTargetSequence(t,e,i,s){const r=e.length,a=[];for(let o=0;o<r;o++){let l=[],c=[];l.push((o+r-1)%r,o,(o+1)%r),c.push(0,1,0);const u=Ym(l);l=Du(l,1,u),c=Du(c,1,u),!s&&l[0]===0&&(l.push(r),c.push(c[0])),a.push(new Va(".morphTargetInfluences["+e[o].name+"]",l,c).scale(1/i))}return new this(t,-1,a)}static findByName(t,e){let i=t;if(!Array.isArray(t)){const s=t;i=s.geometry&&s.geometry.animations||s.animations}for(let s=0;s<i.length;s++)if(i[s].name===e)return i[s];return null}static CreateClipsFromMorphTargetSequences(t,e,i){const s={},r=/^([\w-]*?)([\d]+)$/;for(let o=0,l=t.length;o<l;o++){const c=t[o],u=c.name.match(r);if(u&&u.length>1){const h=u[1];let d=s[h];d||(s[h]=d=[]),d.push(c)}}const a=[];for(const o in s)a.push(this.CreateFromMorphTargetSequence(o,s[o],e,i));return a}static parseAnimation(t,e){if(!t)return console.error("THREE.AnimationClip: No animation in JSONLoader data."),null;const i=function(h,d,p,g,_){if(p.length!==0){const m=[],f=[];_d(p,m,f,g),m.length!==0&&_.push(new h(d,m,f))}},s=[],r=t.name||"default",a=t.fps||30,o=t.blendMode;let l=t.length||-1;const c=t.hierarchy||[];for(let h=0;h<c.length;h++){const d=c[h].keys;if(!(!d||d.length===0))if(d[0].morphTargets){const p={};let g;for(g=0;g<d.length;g++)if(d[g].morphTargets)for(let _=0;_<d[g].morphTargets.length;_++)p[d[g].morphTargets[_]]=-1;for(const _ in p){const m=[],f=[];for(let P=0;P!==d[g].morphTargets.length;++P){const L=d[g];m.push(L.time),f.push(L.morphTarget===_?1:0)}s.push(new Va(".morphTargetInfluence["+_+"]",m,f))}l=p.length*a}else{const p=".bones["+e[h].name+"]";i(Zs,p+".position",d,"pos",s),i(Dr,p+".quaternion",d,"rot",s),i(Zs,p+".scale",d,"scl",s)}}return s.length===0?null:new this(r,l,s,o)}resetDuration(){const t=this.tracks;let e=0;for(let i=0,s=t.length;i!==s;++i){const r=this.tracks[i];e=Math.max(e,r.times[r.times.length-1])}return this.duration=e,this}trim(){for(let t=0;t<this.tracks.length;t++)this.tracks[t].trim(0,this.duration);return this}validate(){let t=!0;for(let e=0;e<this.tracks.length;e++)t=t&&this.tracks[e].validate();return t}optimize(){for(let t=0;t<this.tracks.length;t++)this.tracks[t].optimize();return this}clone(){const t=[];for(let e=0;e<this.tracks.length;e++)t.push(this.tracks[e].clone());return new this.constructor(this.name,this.duration,t,this.blendMode)}toJSON(){return this.constructor.toJSON(this)}}function tg(n){switch(n.toLowerCase()){case"scalar":case"double":case"float":case"number":case"integer":return Va;case"vector":case"vector2":case"vector3":case"vector4":return Zs;case"color":return vd;case"quaternion":return Dr;case"bool":case"boolean":return nr;case"string":return ir}throw new Error("THREE.KeyframeTrack: Unsupported typeName: "+n)}function eg(n){if(n.type===void 0)throw new Error("THREE.KeyframeTrack: track type undefined, can not parse");const t=tg(n.type);if(n.times===void 0){const e=[],i=[];_d(n.keys,e,i,"value"),n.times=e,n.values=i}return t.parse!==void 0?t.parse(n):new t(n.name,n.times,n.values,n.interpolation)}const Ha={enabled:!1,files:{},add:function(n,t){this.enabled!==!1&&(this.files[n]=t)},get:function(n){if(this.enabled!==!1)return this.files[n]},remove:function(n){delete this.files[n]},clear:function(){this.files={}}};class xd{constructor(t,e,i){const s=this;let r=!1,a=0,o=0,l;const c=[];this.onStart=void 0,this.onLoad=t,this.onProgress=e,this.onError=i,this.itemStart=function(u){o++,r===!1&&s.onStart!==void 0&&s.onStart(u,a,o),r=!0},this.itemEnd=function(u){a++,s.onProgress!==void 0&&s.onProgress(u,a,o),a===o&&(r=!1,s.onLoad!==void 0&&s.onLoad())},this.itemError=function(u){s.onError!==void 0&&s.onError(u)},this.resolveURL=function(u){return l?l(u):u},this.setURLModifier=function(u){return l=u,this},this.addHandler=function(u,h){return c.push(u,h),this},this.removeHandler=function(u){const h=c.indexOf(u);return h!==-1&&c.splice(h,2),this},this.getHandler=function(u){for(let h=0,d=c.length;h<d;h+=2){const p=c[h],g=c[h+1];if(p.global&&(p.lastIndex=0),p.test(u))return g}return null}}}const bd=new xd;class Di{constructor(t){this.manager=t!==void 0?t:bd,this.crossOrigin="anonymous",this.withCredentials=!1,this.path="",this.resourcePath="",this.requestHeader={}}load(){}loadAsync(t,e){const i=this;return new Promise(function(s,r){i.load(t,s,e,r)})}parse(){}setCrossOrigin(t){return this.crossOrigin=t,this}setWithCredentials(t){return this.withCredentials=t,this}setPath(t){return this.path=t,this}setResourcePath(t){return this.resourcePath=t,this}setRequestHeader(t){return this.requestHeader=t,this}}Di.DEFAULT_MATERIAL_NAME="__DEFAULT";const si={};class ng extends Error{constructor(t,e){super(t),this.response=e}}class _c extends Di{constructor(t){super(t)}load(t,e,i,s){t===void 0&&(t=""),this.path!==void 0&&(t=this.path+t),t=this.manager.resolveURL(t);const r=Ha.get(t);if(r!==void 0)return this.manager.itemStart(t),setTimeout(()=>{e&&e(r),this.manager.itemEnd(t)},0),r;if(si[t]!==void 0){si[t].push({onLoad:e,onProgress:i,onError:s});return}si[t]=[],si[t].push({onLoad:e,onProgress:i,onError:s});const a=new Request(t,{headers:new Headers(this.requestHeader),credentials:this.withCredentials?"include":"same-origin"}),o=this.mimeType,l=this.responseType;fetch(a).then(c=>{if(c.status===200||c.status===0){if(c.status===0&&console.warn("THREE.FileLoader: HTTP Status 0 received."),typeof ReadableStream>"u"||c.body===void 0||c.body.getReader===void 0)return c;const u=si[t],h=c.body.getReader(),d=c.headers.get("X-File-Size")||c.headers.get("Content-Length"),p=d?parseInt(d):0,g=p!==0;let _=0;const m=new ReadableStream({start(f){P();function P(){h.read().then(({done:L,value:w})=>{if(L)f.close();else{_+=w.byteLength;const N=new ProgressEvent("progress",{lengthComputable:g,loaded:_,total:p});for(let F=0,U=u.length;F<U;F++){const k=u[F];k.onProgress&&k.onProgress(N)}f.enqueue(w),P()}},L=>{f.error(L)})}}});return new Response(m)}else throw new ng(`fetch for "${c.url}" responded with ${c.status}: ${c.statusText}`,c)}).then(c=>{switch(l){case"arraybuffer":return c.arrayBuffer();case"blob":return c.blob();case"document":return c.text().then(u=>new DOMParser().parseFromString(u,o));case"json":return c.json();default:if(o===void 0)return c.text();{const h=/charset="?([^;"\s]*)"?/i.exec(o),d=h&&h[1]?h[1].toLowerCase():void 0,p=new TextDecoder(d);return c.arrayBuffer().then(g=>p.decode(g))}}}).then(c=>{Ha.add(t,c);const u=si[t];delete si[t];for(let h=0,d=u.length;h<d;h++){const p=u[h];p.onLoad&&p.onLoad(c)}}).catch(c=>{const u=si[t];if(u===void 0)throw this.manager.itemError(t),c;delete si[t];for(let h=0,d=u.length;h<d;h++){const p=u[h];p.onError&&p.onError(c)}this.manager.itemError(t)}).finally(()=>{this.manager.itemEnd(t)}),this.manager.itemStart(t)}setResponseType(t){return this.responseType=t,this}setMimeType(t){return this.mimeType=t,this}}class ig extends Di{constructor(t){super(t)}load(t,e,i,s){this.path!==void 0&&(t=this.path+t),t=this.manager.resolveURL(t);const r=this,a=Ha.get(t);if(a!==void 0)return r.manager.itemStart(t),setTimeout(function(){e&&e(a),r.manager.itemEnd(t)},0),a;const o=Sr("img");function l(){u(),Ha.add(t,this),e&&e(this),r.manager.itemEnd(t)}function c(h){u(),s&&s(h),r.manager.itemError(t),r.manager.itemEnd(t)}function u(){o.removeEventListener("load",l,!1),o.removeEventListener("error",c,!1)}return o.addEventListener("load",l,!1),o.addEventListener("error",c,!1),t.slice(0,5)!=="data:"&&this.crossOrigin!==void 0&&(o.crossOrigin=this.crossOrigin),r.manager.itemStart(t),o.src=t,o}}class sg extends Di{constructor(t){super(t)}load(t,e,i,s){const r=this,a=new pd,o=new _c(this.manager);return o.setResponseType("arraybuffer"),o.setRequestHeader(this.requestHeader),o.setPath(this.path),o.setWithCredentials(r.withCredentials),o.load(t,function(l){let c;try{c=r.parse(l)}catch(u){if(s!==void 0)s(u);else{console.error(u);return}}c.image!==void 0?a.image=c.image:c.data!==void 0&&(a.image.width=c.width,a.image.height=c.height,a.image.data=c.data),a.wrapS=c.wrapS!==void 0?c.wrapS:zn,a.wrapT=c.wrapT!==void 0?c.wrapT:zn,a.magFilter=c.magFilter!==void 0?c.magFilter:wn,a.minFilter=c.minFilter!==void 0?c.minFilter:wn,a.anisotropy=c.anisotropy!==void 0?c.anisotropy:1,c.colorSpace!==void 0&&(a.colorSpace=c.colorSpace),c.flipY!==void 0&&(a.flipY=c.flipY),c.format!==void 0&&(a.format=c.format),c.type!==void 0&&(a.type=c.type),c.mipmaps!==void 0&&(a.mipmaps=c.mipmaps,a.minFilter=oi),c.mipmapCount===1&&(a.minFilter=wn),c.generateMipmaps!==void 0&&(a.generateMipmaps=c.generateMipmaps),a.needsUpdate=!0,e&&e(a,c)},i,s),a}}class yd extends Di{constructor(t){super(t)}load(t,e,i,s){const r=new ln,a=new ig(this.manager);return a.setCrossOrigin(this.crossOrigin),a.setPath(this.path),a.load(t,function(o){r.image=o,r.needsUpdate=!0,e!==void 0&&e(r)},i,s),r}}class Qa extends Ve{constructor(t,e=1){super(),this.isLight=!0,this.type="Light",this.color=new ne(t),this.intensity=e}dispose(){}copy(t,e){return super.copy(t,e),this.color.copy(t.color),this.intensity=t.intensity,this}toJSON(t){const e=super.toJSON(t);return e.object.color=this.color.getHex(),e.object.intensity=this.intensity,this.groundColor!==void 0&&(e.object.groundColor=this.groundColor.getHex()),this.distance!==void 0&&(e.object.distance=this.distance),this.angle!==void 0&&(e.object.angle=this.angle),this.decay!==void 0&&(e.object.decay=this.decay),this.penumbra!==void 0&&(e.object.penumbra=this.penumbra),this.shadow!==void 0&&(e.object.shadow=this.shadow.toJSON()),this.target!==void 0&&(e.object.target=this.target.uuid),e}}const Ho=new Kt,Uu=new O,ku=new O;class vc{constructor(t){this.camera=t,this.intensity=1,this.bias=0,this.normalBias=0,this.radius=1,this.blurSamples=8,this.mapSize=new Yt(512,512),this.map=null,this.mapPass=null,this.matrix=new Kt,this.autoUpdate=!0,this.needsUpdate=!1,this._frustum=new pc,this._frameExtents=new Yt(1,1),this._viewportCount=1,this._viewports=[new Ee(0,0,1,1)]}getViewportCount(){return this._viewportCount}getFrustum(){return this._frustum}updateMatrices(t){const e=this.camera,i=this.matrix;Uu.setFromMatrixPosition(t.matrixWorld),e.position.copy(Uu),ku.setFromMatrixPosition(t.target.matrixWorld),e.lookAt(ku),e.updateMatrixWorld(),Ho.multiplyMatrices(e.projectionMatrix,e.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Ho),i.set(.5,0,0,.5,0,.5,0,.5,0,0,.5,.5,0,0,0,1),i.multiply(Ho)}getViewport(t){return this._viewports[t]}getFrameExtents(){return this._frameExtents}dispose(){this.map&&this.map.dispose(),this.mapPass&&this.mapPass.dispose()}copy(t){return this.camera=t.camera.clone(),this.intensity=t.intensity,this.bias=t.bias,this.radius=t.radius,this.mapSize.copy(t.mapSize),this}clone(){return new this.constructor().copy(this)}toJSON(){const t={};return this.intensity!==1&&(t.intensity=this.intensity),this.bias!==0&&(t.bias=this.bias),this.normalBias!==0&&(t.normalBias=this.normalBias),this.radius!==1&&(t.radius=this.radius),(this.mapSize.x!==512||this.mapSize.y!==512)&&(t.mapSize=this.mapSize.toArray()),t.camera=this.camera.toJSON(!1).object,delete t.camera.matrix,t}}class rg extends vc{constructor(){super(new an(50,1,.5,500)),this.isSpotLightShadow=!0,this.focus=1}updateMatrices(t){const e=this.camera,i=Ks*2*t.angle*this.focus,s=this.mapSize.width/this.mapSize.height,r=t.distance||e.far;(i!==e.fov||s!==e.aspect||r!==e.far)&&(e.fov=i,e.aspect=s,e.far=r,e.updateProjectionMatrix()),super.updateMatrices(t)}copy(t){return super.copy(t),this.focus=t.focus,this}}class ag extends Qa{constructor(t,e,i=0,s=Math.PI/3,r=0,a=2){super(t,e),this.isSpotLight=!0,this.type="SpotLight",this.position.copy(Ve.DEFAULT_UP),this.updateMatrix(),this.target=new Ve,this.distance=i,this.angle=s,this.penumbra=r,this.decay=a,this.map=null,this.shadow=new rg}get power(){return this.intensity*Math.PI}set power(t){this.intensity=t/Math.PI}dispose(){this.shadow.dispose()}copy(t,e){return super.copy(t,e),this.distance=t.distance,this.angle=t.angle,this.penumbra=t.penumbra,this.decay=t.decay,this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}const Fu=new Kt,mr=new O,Go=new O;class og extends vc{constructor(){super(new an(90,1,.5,500)),this.isPointLightShadow=!0,this._frameExtents=new Yt(4,2),this._viewportCount=6,this._viewports=[new Ee(2,1,1,1),new Ee(0,1,1,1),new Ee(3,1,1,1),new Ee(1,1,1,1),new Ee(3,0,1,1),new Ee(1,0,1,1)],this._cubeDirections=[new O(1,0,0),new O(-1,0,0),new O(0,0,1),new O(0,0,-1),new O(0,1,0),new O(0,-1,0)],this._cubeUps=[new O(0,1,0),new O(0,1,0),new O(0,1,0),new O(0,1,0),new O(0,0,1),new O(0,0,-1)]}updateMatrices(t,e=0){const i=this.camera,s=this.matrix,r=t.distance||i.far;r!==i.far&&(i.far=r,i.updateProjectionMatrix()),mr.setFromMatrixPosition(t.matrixWorld),i.position.copy(mr),Go.copy(i.position),Go.add(this._cubeDirections[e]),i.up.copy(this._cubeUps[e]),i.lookAt(Go),i.updateMatrixWorld(),s.makeTranslation(-mr.x,-mr.y,-mr.z),Fu.multiplyMatrices(i.projectionMatrix,i.matrixWorldInverse),this._frustum.setFromProjectionMatrix(Fu)}}class lg extends Qa{constructor(t,e,i=0,s=2){super(t,e),this.isPointLight=!0,this.type="PointLight",this.distance=i,this.decay=s,this.shadow=new og}get power(){return this.intensity*4*Math.PI}set power(t){this.intensity=t/(4*Math.PI)}dispose(){this.shadow.dispose()}copy(t,e){return super.copy(t,e),this.distance=t.distance,this.decay=t.decay,this.shadow=t.shadow.clone(),this}}class xc extends ud{constructor(t=-1,e=1,i=1,s=-1,r=.1,a=2e3){super(),this.isOrthographicCamera=!0,this.type="OrthographicCamera",this.zoom=1,this.view=null,this.left=t,this.right=e,this.top=i,this.bottom=s,this.near=r,this.far=a,this.updateProjectionMatrix()}copy(t,e){return super.copy(t,e),this.left=t.left,this.right=t.right,this.top=t.top,this.bottom=t.bottom,this.near=t.near,this.far=t.far,this.zoom=t.zoom,this.view=t.view===null?null:Object.assign({},t.view),this}setViewOffset(t,e,i,s,r,a){this.view===null&&(this.view={enabled:!0,fullWidth:1,fullHeight:1,offsetX:0,offsetY:0,width:1,height:1}),this.view.enabled=!0,this.view.fullWidth=t,this.view.fullHeight=e,this.view.offsetX=i,this.view.offsetY=s,this.view.width=r,this.view.height=a,this.updateProjectionMatrix()}clearViewOffset(){this.view!==null&&(this.view.enabled=!1),this.updateProjectionMatrix()}updateProjectionMatrix(){const t=(this.right-this.left)/(2*this.zoom),e=(this.top-this.bottom)/(2*this.zoom),i=(this.right+this.left)/2,s=(this.top+this.bottom)/2;let r=i-t,a=i+t,o=s+e,l=s-e;if(this.view!==null&&this.view.enabled){const c=(this.right-this.left)/this.view.fullWidth/this.zoom,u=(this.top-this.bottom)/this.view.fullHeight/this.zoom;r+=c*this.view.offsetX,a=r+c*this.view.width,o-=u*this.view.offsetY,l=o-u*this.view.height}this.projectionMatrix.makeOrthographic(r,a,o,l,this.near,this.far,this.coordinateSystem),this.projectionMatrixInverse.copy(this.projectionMatrix).invert()}toJSON(t){const e=super.toJSON(t);return e.object.zoom=this.zoom,e.object.left=this.left,e.object.right=this.right,e.object.top=this.top,e.object.bottom=this.bottom,e.object.near=this.near,e.object.far=this.far,this.view!==null&&(e.object.view=Object.assign({},this.view)),e}}class cg extends vc{constructor(){super(new xc(-5,5,5,-5,.5,500)),this.isDirectionalLightShadow=!0}}class Na extends Qa{constructor(t,e){super(t,e),this.isDirectionalLight=!0,this.type="DirectionalLight",this.position.copy(Ve.DEFAULT_UP),this.updateMatrix(),this.target=new Ve,this.shadow=new cg}dispose(){this.shadow.dispose()}copy(t){return super.copy(t),this.target=t.target.clone(),this.shadow=t.shadow.clone(),this}}class Md extends Qa{constructor(t,e){super(t,e),this.isAmbientLight=!0,this.type="AmbientLight"}}class Sd{static decodeText(t){if(console.warn("THREE.LoaderUtils: decodeText() has been deprecated with r165 and will be removed with r175. Use TextDecoder instead."),typeof TextDecoder<"u")return new TextDecoder().decode(t);let e="";for(let i=0,s=t.length;i<s;i++)e+=String.fromCharCode(t[i]);try{return decodeURIComponent(escape(e))}catch{return e}}static extractUrlBase(t){const e=t.lastIndexOf("/");return e===-1?"./":t.slice(0,e+1)}static resolveURL(t,e){return typeof t!="string"||t===""?"":(/^https?:\/\//i.test(e)&&/^\//.test(t)&&(e=e.replace(/(^https?:\/\/[^\/]+).*/i,"$1")),/^(https?:)?\/\//i.test(t)||/^data:.*,.*$/i.test(t)||/^blob:.*$/i.test(t)?t:e+t)}}class ug extends an{constructor(t=[]){super(),this.isArrayCamera=!0,this.cameras=t,this.index=0}}class Ou{constructor(t=1,e=0,i=0){this.radius=t,this.phi=e,this.theta=i}set(t,e,i){return this.radius=t,this.phi=e,this.theta=i,this}copy(t){return this.radius=t.radius,this.phi=t.phi,this.theta=t.theta,this}makeSafe(){return this.phi=he(this.phi,1e-6,Math.PI-1e-6),this}setFromVector3(t){return this.setFromCartesianCoords(t.x,t.y,t.z)}setFromCartesianCoords(t,e,i){return this.radius=Math.sqrt(t*t+e*e+i*i),this.radius===0?(this.theta=0,this.phi=0):(this.theta=Math.atan2(t,i),this.phi=Math.acos(he(e/this.radius,-1,1))),this}clone(){return new this.constructor().copy(this)}}const Bu=new O;let ma,Wo;class ga extends Ve{constructor(t=new O(0,0,1),e=new O(0,0,0),i=1,s=16776960,r=i*.2,a=r*.2){super(),this.type="ArrowHelper",ma===void 0&&(ma=new $e,ma.setAttribute("position",new Ae([0,0,0,0,1,0],3)),Wo=new Lr(0,.5,1,5,1),Wo.translate(0,-.5,0)),this.position.copy(e),this.line=new mc(ma,new Ci({color:s,toneMapped:!1})),this.line.matrixAutoUpdate=!1,this.add(this.line),this.cone=new Re(Wo,new Er({color:s,toneMapped:!1})),this.cone.matrixAutoUpdate=!1,this.add(this.cone),this.setDirection(t),this.setLength(i,r,a)}setDirection(t){if(t.y>.99999)this.quaternion.set(0,0,0,1);else if(t.y<-.99999)this.quaternion.set(1,0,0,0);else{Bu.set(t.z,0,-t.x).normalize();const e=Math.acos(t.y);this.quaternion.setFromAxisAngle(Bu,e)}}setLength(t,e=t*.2,i=e*.2){this.line.scale.set(1,Math.max(1e-4,t-e),1),this.line.updateMatrix(),this.cone.scale.set(i,e,i),this.cone.position.y=t,this.cone.updateMatrix()}setColor(t){this.line.material.color.set(t),this.cone.material.color.set(t)}copy(t){return super.copy(t,!1),this.line.copy(t.line),this.cone.copy(t.cone),this}dispose(){this.line.geometry.dispose(),this.line.material.dispose(),this.cone.geometry.dispose(),this.cone.material.dispose()}}class hg extends rs{constructor(t,e=null){super(),this.object=t,this.domElement=e,this.enabled=!0,this.state=-1,this.keys={},this.mouseButtons={LEFT:null,MIDDLE:null,RIGHT:null},this.touches={ONE:null,TWO:null}}connect(){}disconnect(){}dispose(){}update(){}}function zu(n,t,e,i){const s=dg(i);switch(e){case Yh:return n*t;case Jh:return n*t;case Zh:return n*t*2;case Qh:return n*t/s.components*s.byteLength;case lc:return n*t/s.components*s.byteLength;case td:return n*t*2/s.components*s.byteLength;case cc:return n*t*2/s.components*s.byteLength;case Kh:return n*t*3/s.components*s.byteLength;case Pn:return n*t*4/s.components*s.byteLength;case uc:return n*t*4/s.components*s.byteLength;case Aa:case Ca:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*8;case Ra:case Pa:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case _l:case xl:return Math.max(n,16)*Math.max(t,8)/4;case gl:case vl:return Math.max(n,8)*Math.max(t,8)/2;case bl:case yl:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*8;case Ml:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case Sl:return Math.floor((n+3)/4)*Math.floor((t+3)/4)*16;case El:return Math.floor((n+4)/5)*Math.floor((t+3)/4)*16;case Tl:return Math.floor((n+4)/5)*Math.floor((t+4)/5)*16;case wl:return Math.floor((n+5)/6)*Math.floor((t+4)/5)*16;case Al:return Math.floor((n+5)/6)*Math.floor((t+5)/6)*16;case Cl:return Math.floor((n+7)/8)*Math.floor((t+4)/5)*16;case Rl:return Math.floor((n+7)/8)*Math.floor((t+5)/6)*16;case Pl:return Math.floor((n+7)/8)*Math.floor((t+7)/8)*16;case Ll:return Math.floor((n+9)/10)*Math.floor((t+4)/5)*16;case Nl:return Math.floor((n+9)/10)*Math.floor((t+5)/6)*16;case Dl:return Math.floor((n+9)/10)*Math.floor((t+7)/8)*16;case Il:return Math.floor((n+9)/10)*Math.floor((t+9)/10)*16;case Ul:return Math.floor((n+11)/12)*Math.floor((t+9)/10)*16;case kl:return Math.floor((n+11)/12)*Math.floor((t+11)/12)*16;case La:case Fl:case Ol:return Math.ceil(n/4)*Math.ceil(t/4)*16;case ed:case Bl:return Math.ceil(n/4)*Math.ceil(t/4)*8;case zl:case Vl:return Math.ceil(n/4)*Math.ceil(t/4)*16}throw new Error(`Unable to determine texture byte length for ${e} format.`)}function dg(n){switch(n){case hi:case jh:return{byteLength:1,components:1};case Mr:case Xh:case Rr:return{byteLength:2,components:1};case ac:case oc:return{byteLength:2,components:4};case ts:case rc:case jn:return{byteLength:4,components:1};case qh:return{byteLength:4,components:3}}throw new Error(`Unknown texture type ${n}.`)}typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("register",{detail:{revision:sc}}));typeof window<"u"&&(window.__THREE__?console.warn("WARNING: Multiple instances of Three.js being imported."):window.__THREE__=sc);/**
 * @license
 * Copyright 2010-2025 Three.js Authors
 * SPDX-License-Identifier: MIT
 */function Ed(){let n=null,t=!1,e=null,i=null;function s(r,a){e(r,a),i=n.requestAnimationFrame(s)}return{start:function(){t!==!0&&e!==null&&(i=n.requestAnimationFrame(s),t=!0)},stop:function(){n.cancelAnimationFrame(i),t=!1},setAnimationLoop:function(r){e=r},setContext:function(r){n=r}}}function fg(n){const t=new WeakMap;function e(o,l){const c=o.array,u=o.usage,h=c.byteLength,d=n.createBuffer();n.bindBuffer(l,d),n.bufferData(l,c,u),o.onUploadCallback();let p;if(c instanceof Float32Array)p=n.FLOAT;else if(c instanceof Uint16Array)o.isFloat16BufferAttribute?p=n.HALF_FLOAT:p=n.UNSIGNED_SHORT;else if(c instanceof Int16Array)p=n.SHORT;else if(c instanceof Uint32Array)p=n.UNSIGNED_INT;else if(c instanceof Int32Array)p=n.INT;else if(c instanceof Int8Array)p=n.BYTE;else if(c instanceof Uint8Array)p=n.UNSIGNED_BYTE;else if(c instanceof Uint8ClampedArray)p=n.UNSIGNED_BYTE;else throw new Error("THREE.WebGLAttributes: Unsupported buffer data format: "+c);return{buffer:d,type:p,bytesPerElement:c.BYTES_PER_ELEMENT,version:o.version,size:h}}function i(o,l,c){const u=l.array,h=l.updateRanges;if(n.bindBuffer(c,o),h.length===0)n.bufferSubData(c,0,u);else{h.sort((p,g)=>p.start-g.start);let d=0;for(let p=1;p<h.length;p++){const g=h[d],_=h[p];_.start<=g.start+g.count+1?g.count=Math.max(g.count,_.start+_.count-g.start):(++d,h[d]=_)}h.length=d+1;for(let p=0,g=h.length;p<g;p++){const _=h[p];n.bufferSubData(c,_.start*u.BYTES_PER_ELEMENT,u,_.start,_.count)}l.clearUpdateRanges()}l.onUploadCallback()}function s(o){return o.isInterleavedBufferAttribute&&(o=o.data),t.get(o)}function r(o){o.isInterleavedBufferAttribute&&(o=o.data);const l=t.get(o);l&&(n.deleteBuffer(l.buffer),t.delete(o))}function a(o,l){if(o.isInterleavedBufferAttribute&&(o=o.data),o.isGLBufferAttribute){const u=t.get(o);(!u||u.version<o.version)&&t.set(o,{buffer:o.buffer,type:o.type,bytesPerElement:o.elementSize,version:o.version});return}const c=t.get(o);if(c===void 0)t.set(o,e(o,l));else if(c.version<o.version){if(c.size!==o.array.byteLength)throw new Error("THREE.WebGLAttributes: The size of the buffer attribute's array buffer does not match the original size. Resizing buffer attributes is not supported.");i(c.buffer,o,l),c.version=o.version}}return{get:s,remove:r,update:a}}var pg=`#ifdef USE_ALPHAHASH
	if ( diffuseColor.a < getAlphaHashThreshold( vPosition ) ) discard;
#endif`,mg=`#ifdef USE_ALPHAHASH
	const float ALPHA_HASH_SCALE = 0.05;
	float hash2D( vec2 value ) {
		return fract( 1.0e4 * sin( 17.0 * value.x + 0.1 * value.y ) * ( 0.1 + abs( sin( 13.0 * value.y + value.x ) ) ) );
	}
	float hash3D( vec3 value ) {
		return hash2D( vec2( hash2D( value.xy ), value.z ) );
	}
	float getAlphaHashThreshold( vec3 position ) {
		float maxDeriv = max(
			length( dFdx( position.xyz ) ),
			length( dFdy( position.xyz ) )
		);
		float pixScale = 1.0 / ( ALPHA_HASH_SCALE * maxDeriv );
		vec2 pixScales = vec2(
			exp2( floor( log2( pixScale ) ) ),
			exp2( ceil( log2( pixScale ) ) )
		);
		vec2 alpha = vec2(
			hash3D( floor( pixScales.x * position.xyz ) ),
			hash3D( floor( pixScales.y * position.xyz ) )
		);
		float lerpFactor = fract( log2( pixScale ) );
		float x = ( 1.0 - lerpFactor ) * alpha.x + lerpFactor * alpha.y;
		float a = min( lerpFactor, 1.0 - lerpFactor );
		vec3 cases = vec3(
			x * x / ( 2.0 * a * ( 1.0 - a ) ),
			( x - 0.5 * a ) / ( 1.0 - a ),
			1.0 - ( ( 1.0 - x ) * ( 1.0 - x ) / ( 2.0 * a * ( 1.0 - a ) ) )
		);
		float threshold = ( x < ( 1.0 - a ) )
			? ( ( x < a ) ? cases.x : cases.y )
			: cases.z;
		return clamp( threshold , 1.0e-6, 1.0 );
	}
#endif`,gg=`#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, vAlphaMapUv ).g;
#endif`,_g=`#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,vg=`#ifdef USE_ALPHATEST
	#ifdef ALPHA_TO_COVERAGE
	diffuseColor.a = smoothstep( alphaTest, alphaTest + fwidth( diffuseColor.a ), diffuseColor.a );
	if ( diffuseColor.a == 0.0 ) discard;
	#else
	if ( diffuseColor.a < alphaTest ) discard;
	#endif
#endif`,xg=`#ifdef USE_ALPHATEST
	uniform float alphaTest;
#endif`,bg=`#ifdef USE_AOMAP
	float ambientOcclusion = ( texture2D( aoMap, vAoMapUv ).r - 1.0 ) * aoMapIntensity + 1.0;
	reflectedLight.indirectDiffuse *= ambientOcclusion;
	#if defined( USE_CLEARCOAT ) 
		clearcoatSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_SHEEN ) 
		sheenSpecularIndirect *= ambientOcclusion;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD )
		float dotNV = saturate( dot( geometryNormal, geometryViewDir ) );
		reflectedLight.indirectSpecular *= computeSpecularOcclusion( dotNV, ambientOcclusion, material.roughness );
	#endif
#endif`,yg=`#ifdef USE_AOMAP
	uniform sampler2D aoMap;
	uniform float aoMapIntensity;
#endif`,Mg=`#ifdef USE_BATCHING
	#if ! defined( GL_ANGLE_multi_draw )
	#define gl_DrawID _gl_DrawID
	uniform int _gl_DrawID;
	#endif
	uniform highp sampler2D batchingTexture;
	uniform highp usampler2D batchingIdTexture;
	mat4 getBatchingMatrix( const in float i ) {
		int size = textureSize( batchingTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( batchingTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( batchingTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( batchingTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( batchingTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
	float getIndirectIndex( const in int i ) {
		int size = textureSize( batchingIdTexture, 0 ).x;
		int x = i % size;
		int y = i / size;
		return float( texelFetch( batchingIdTexture, ivec2( x, y ), 0 ).r );
	}
#endif
#ifdef USE_BATCHING_COLOR
	uniform sampler2D batchingColorTexture;
	vec3 getBatchingColor( const in float i ) {
		int size = textureSize( batchingColorTexture, 0 ).x;
		int j = int( i );
		int x = j % size;
		int y = j / size;
		return texelFetch( batchingColorTexture, ivec2( x, y ), 0 ).rgb;
	}
#endif`,Sg=`#ifdef USE_BATCHING
	mat4 batchingMatrix = getBatchingMatrix( getIndirectIndex( gl_DrawID ) );
#endif`,Eg=`vec3 transformed = vec3( position );
#ifdef USE_ALPHAHASH
	vPosition = vec3( position );
#endif`,Tg=`vec3 objectNormal = vec3( normal );
#ifdef USE_TANGENT
	vec3 objectTangent = vec3( tangent.xyz );
#endif`,wg=`float G_BlinnPhong_Implicit( ) {
	return 0.25;
}
float D_BlinnPhong( const in float shininess, const in float dotNH ) {
	return RECIPROCAL_PI * ( shininess * 0.5 + 1.0 ) * pow( dotNH, shininess );
}
vec3 BRDF_BlinnPhong( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in vec3 specularColor, const in float shininess ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( specularColor, 1.0, dotVH );
	float G = G_BlinnPhong_Implicit( );
	float D = D_BlinnPhong( shininess, dotNH );
	return F * ( G * D );
} // validated`,Ag=`#ifdef USE_IRIDESCENCE
	const mat3 XYZ_TO_REC709 = mat3(
		 3.2404542, -0.9692660,  0.0556434,
		-1.5371385,  1.8760108, -0.2040259,
		-0.4985314,  0.0415560,  1.0572252
	);
	vec3 Fresnel0ToIor( vec3 fresnel0 ) {
		vec3 sqrtF0 = sqrt( fresnel0 );
		return ( vec3( 1.0 ) + sqrtF0 ) / ( vec3( 1.0 ) - sqrtF0 );
	}
	vec3 IorToFresnel0( vec3 transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - vec3( incidentIor ) ) / ( transmittedIor + vec3( incidentIor ) ) );
	}
	float IorToFresnel0( float transmittedIor, float incidentIor ) {
		return pow2( ( transmittedIor - incidentIor ) / ( transmittedIor + incidentIor ));
	}
	vec3 evalSensitivity( float OPD, vec3 shift ) {
		float phase = 2.0 * PI * OPD * 1.0e-9;
		vec3 val = vec3( 5.4856e-13, 4.4201e-13, 5.2481e-13 );
		vec3 pos = vec3( 1.6810e+06, 1.7953e+06, 2.2084e+06 );
		vec3 var = vec3( 4.3278e+09, 9.3046e+09, 6.6121e+09 );
		vec3 xyz = val * sqrt( 2.0 * PI * var ) * cos( pos * phase + shift ) * exp( - pow2( phase ) * var );
		xyz.x += 9.7470e-14 * sqrt( 2.0 * PI * 4.5282e+09 ) * cos( 2.2399e+06 * phase + shift[ 0 ] ) * exp( - 4.5282e+09 * pow2( phase ) );
		xyz /= 1.0685e-7;
		vec3 rgb = XYZ_TO_REC709 * xyz;
		return rgb;
	}
	vec3 evalIridescence( float outsideIOR, float eta2, float cosTheta1, float thinFilmThickness, vec3 baseF0 ) {
		vec3 I;
		float iridescenceIOR = mix( outsideIOR, eta2, smoothstep( 0.0, 0.03, thinFilmThickness ) );
		float sinTheta2Sq = pow2( outsideIOR / iridescenceIOR ) * ( 1.0 - pow2( cosTheta1 ) );
		float cosTheta2Sq = 1.0 - sinTheta2Sq;
		if ( cosTheta2Sq < 0.0 ) {
			return vec3( 1.0 );
		}
		float cosTheta2 = sqrt( cosTheta2Sq );
		float R0 = IorToFresnel0( iridescenceIOR, outsideIOR );
		float R12 = F_Schlick( R0, 1.0, cosTheta1 );
		float T121 = 1.0 - R12;
		float phi12 = 0.0;
		if ( iridescenceIOR < outsideIOR ) phi12 = PI;
		float phi21 = PI - phi12;
		vec3 baseIOR = Fresnel0ToIor( clamp( baseF0, 0.0, 0.9999 ) );		vec3 R1 = IorToFresnel0( baseIOR, iridescenceIOR );
		vec3 R23 = F_Schlick( R1, 1.0, cosTheta2 );
		vec3 phi23 = vec3( 0.0 );
		if ( baseIOR[ 0 ] < iridescenceIOR ) phi23[ 0 ] = PI;
		if ( baseIOR[ 1 ] < iridescenceIOR ) phi23[ 1 ] = PI;
		if ( baseIOR[ 2 ] < iridescenceIOR ) phi23[ 2 ] = PI;
		float OPD = 2.0 * iridescenceIOR * thinFilmThickness * cosTheta2;
		vec3 phi = vec3( phi21 ) + phi23;
		vec3 R123 = clamp( R12 * R23, 1e-5, 0.9999 );
		vec3 r123 = sqrt( R123 );
		vec3 Rs = pow2( T121 ) * R23 / ( vec3( 1.0 ) - R123 );
		vec3 C0 = R12 + Rs;
		I = C0;
		vec3 Cm = Rs - T121;
		for ( int m = 1; m <= 2; ++ m ) {
			Cm *= r123;
			vec3 Sm = 2.0 * evalSensitivity( float( m ) * OPD, float( m ) * phi );
			I += Cm * Sm;
		}
		return max( I, vec3( 0.0 ) );
	}
#endif`,Cg=`#ifdef USE_BUMPMAP
	uniform sampler2D bumpMap;
	uniform float bumpScale;
	vec2 dHdxy_fwd() {
		vec2 dSTdx = dFdx( vBumpMapUv );
		vec2 dSTdy = dFdy( vBumpMapUv );
		float Hll = bumpScale * texture2D( bumpMap, vBumpMapUv ).x;
		float dBx = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdx ).x - Hll;
		float dBy = bumpScale * texture2D( bumpMap, vBumpMapUv + dSTdy ).x - Hll;
		return vec2( dBx, dBy );
	}
	vec3 perturbNormalArb( vec3 surf_pos, vec3 surf_norm, vec2 dHdxy, float faceDirection ) {
		vec3 vSigmaX = normalize( dFdx( surf_pos.xyz ) );
		vec3 vSigmaY = normalize( dFdy( surf_pos.xyz ) );
		vec3 vN = surf_norm;
		vec3 R1 = cross( vSigmaY, vN );
		vec3 R2 = cross( vN, vSigmaX );
		float fDet = dot( vSigmaX, R1 ) * faceDirection;
		vec3 vGrad = sign( fDet ) * ( dHdxy.x * R1 + dHdxy.y * R2 );
		return normalize( abs( fDet ) * surf_norm - vGrad );
	}
#endif`,Rg=`#if NUM_CLIPPING_PLANES > 0
	vec4 plane;
	#ifdef ALPHA_TO_COVERAGE
		float distanceToPlane, distanceGradient;
		float clipOpacity = 1.0;
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
			distanceGradient = fwidth( distanceToPlane ) / 2.0;
			clipOpacity *= smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			if ( clipOpacity == 0.0 ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			float unionClipOpacity = 1.0;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				distanceToPlane = - dot( vClipPosition, plane.xyz ) + plane.w;
				distanceGradient = fwidth( distanceToPlane ) / 2.0;
				unionClipOpacity *= 1.0 - smoothstep( - distanceGradient, distanceGradient, distanceToPlane );
			}
			#pragma unroll_loop_end
			clipOpacity *= 1.0 - unionClipOpacity;
		#endif
		diffuseColor.a *= clipOpacity;
		if ( diffuseColor.a == 0.0 ) discard;
	#else
		#pragma unroll_loop_start
		for ( int i = 0; i < UNION_CLIPPING_PLANES; i ++ ) {
			plane = clippingPlanes[ i ];
			if ( dot( vClipPosition, plane.xyz ) > plane.w ) discard;
		}
		#pragma unroll_loop_end
		#if UNION_CLIPPING_PLANES < NUM_CLIPPING_PLANES
			bool clipped = true;
			#pragma unroll_loop_start
			for ( int i = UNION_CLIPPING_PLANES; i < NUM_CLIPPING_PLANES; i ++ ) {
				plane = clippingPlanes[ i ];
				clipped = ( dot( vClipPosition, plane.xyz ) > plane.w ) && clipped;
			}
			#pragma unroll_loop_end
			if ( clipped ) discard;
		#endif
	#endif
#endif`,Pg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
	uniform vec4 clippingPlanes[ NUM_CLIPPING_PLANES ];
#endif`,Lg=`#if NUM_CLIPPING_PLANES > 0
	varying vec3 vClipPosition;
#endif`,Ng=`#if NUM_CLIPPING_PLANES > 0
	vClipPosition = - mvPosition.xyz;
#endif`,Dg=`#if defined( USE_COLOR_ALPHA )
	diffuseColor *= vColor;
#elif defined( USE_COLOR )
	diffuseColor.rgb *= vColor;
#endif`,Ig=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR )
	varying vec3 vColor;
#endif`,Ug=`#if defined( USE_COLOR_ALPHA )
	varying vec4 vColor;
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	varying vec3 vColor;
#endif`,kg=`#if defined( USE_COLOR_ALPHA )
	vColor = vec4( 1.0 );
#elif defined( USE_COLOR ) || defined( USE_INSTANCING_COLOR ) || defined( USE_BATCHING_COLOR )
	vColor = vec3( 1.0 );
#endif
#ifdef USE_COLOR
	vColor *= color;
#endif
#ifdef USE_INSTANCING_COLOR
	vColor.xyz *= instanceColor.xyz;
#endif
#ifdef USE_BATCHING_COLOR
	vec3 batchingColor = getBatchingColor( getIndirectIndex( gl_DrawID ) );
	vColor.xyz *= batchingColor.xyz;
#endif`,Fg=`#define PI 3.141592653589793
#define PI2 6.283185307179586
#define PI_HALF 1.5707963267948966
#define RECIPROCAL_PI 0.3183098861837907
#define RECIPROCAL_PI2 0.15915494309189535
#define EPSILON 1e-6
#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
#define whiteComplement( a ) ( 1.0 - saturate( a ) )
float pow2( const in float x ) { return x*x; }
vec3 pow2( const in vec3 x ) { return x*x; }
float pow3( const in float x ) { return x*x*x; }
float pow4( const in float x ) { float x2 = x*x; return x2*x2; }
float max3( const in vec3 v ) { return max( max( v.x, v.y ), v.z ); }
float average( const in vec3 v ) { return dot( v, vec3( 0.3333333 ) ); }
highp float rand( const in vec2 uv ) {
	const highp float a = 12.9898, b = 78.233, c = 43758.5453;
	highp float dt = dot( uv.xy, vec2( a,b ) ), sn = mod( dt, PI );
	return fract( sin( sn ) * c );
}
#ifdef HIGH_PRECISION
	float precisionSafeLength( vec3 v ) { return length( v ); }
#else
	float precisionSafeLength( vec3 v ) {
		float maxComponent = max3( abs( v ) );
		return length( v / maxComponent ) * maxComponent;
	}
#endif
struct IncidentLight {
	vec3 color;
	vec3 direction;
	bool visible;
};
struct ReflectedLight {
	vec3 directDiffuse;
	vec3 directSpecular;
	vec3 indirectDiffuse;
	vec3 indirectSpecular;
};
#ifdef USE_ALPHAHASH
	varying vec3 vPosition;
#endif
vec3 transformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( matrix * vec4( dir, 0.0 ) ).xyz );
}
vec3 inverseTransformDirection( in vec3 dir, in mat4 matrix ) {
	return normalize( ( vec4( dir, 0.0 ) * matrix ).xyz );
}
mat3 transposeMat3( const in mat3 m ) {
	mat3 tmp;
	tmp[ 0 ] = vec3( m[ 0 ].x, m[ 1 ].x, m[ 2 ].x );
	tmp[ 1 ] = vec3( m[ 0 ].y, m[ 1 ].y, m[ 2 ].y );
	tmp[ 2 ] = vec3( m[ 0 ].z, m[ 1 ].z, m[ 2 ].z );
	return tmp;
}
bool isPerspectiveMatrix( mat4 m ) {
	return m[ 2 ][ 3 ] == - 1.0;
}
vec2 equirectUv( in vec3 dir ) {
	float u = atan( dir.z, dir.x ) * RECIPROCAL_PI2 + 0.5;
	float v = asin( clamp( dir.y, - 1.0, 1.0 ) ) * RECIPROCAL_PI + 0.5;
	return vec2( u, v );
}
vec3 BRDF_Lambert( const in vec3 diffuseColor ) {
	return RECIPROCAL_PI * diffuseColor;
}
vec3 F_Schlick( const in vec3 f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
}
float F_Schlick( const in float f0, const in float f90, const in float dotVH ) {
	float fresnel = exp2( ( - 5.55473 * dotVH - 6.98316 ) * dotVH );
	return f0 * ( 1.0 - fresnel ) + ( f90 * fresnel );
} // validated`,Og=`#ifdef ENVMAP_TYPE_CUBE_UV
	#define cubeUV_minMipLevel 4.0
	#define cubeUV_minTileSize 16.0
	float getFace( vec3 direction ) {
		vec3 absDirection = abs( direction );
		float face = - 1.0;
		if ( absDirection.x > absDirection.z ) {
			if ( absDirection.x > absDirection.y )
				face = direction.x > 0.0 ? 0.0 : 3.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		} else {
			if ( absDirection.z > absDirection.y )
				face = direction.z > 0.0 ? 2.0 : 5.0;
			else
				face = direction.y > 0.0 ? 1.0 : 4.0;
		}
		return face;
	}
	vec2 getUV( vec3 direction, float face ) {
		vec2 uv;
		if ( face == 0.0 ) {
			uv = vec2( direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 1.0 ) {
			uv = vec2( - direction.x, - direction.z ) / abs( direction.y );
		} else if ( face == 2.0 ) {
			uv = vec2( - direction.x, direction.y ) / abs( direction.z );
		} else if ( face == 3.0 ) {
			uv = vec2( - direction.z, direction.y ) / abs( direction.x );
		} else if ( face == 4.0 ) {
			uv = vec2( - direction.x, direction.z ) / abs( direction.y );
		} else {
			uv = vec2( direction.x, direction.y ) / abs( direction.z );
		}
		return 0.5 * ( uv + 1.0 );
	}
	vec3 bilinearCubeUV( sampler2D envMap, vec3 direction, float mipInt ) {
		float face = getFace( direction );
		float filterInt = max( cubeUV_minMipLevel - mipInt, 0.0 );
		mipInt = max( mipInt, cubeUV_minMipLevel );
		float faceSize = exp2( mipInt );
		highp vec2 uv = getUV( direction, face ) * ( faceSize - 2.0 ) + 1.0;
		if ( face > 2.0 ) {
			uv.y += faceSize;
			face -= 3.0;
		}
		uv.x += face * faceSize;
		uv.x += filterInt * 3.0 * cubeUV_minTileSize;
		uv.y += 4.0 * ( exp2( CUBEUV_MAX_MIP ) - faceSize );
		uv.x *= CUBEUV_TEXEL_WIDTH;
		uv.y *= CUBEUV_TEXEL_HEIGHT;
		#ifdef texture2DGradEXT
			return texture2DGradEXT( envMap, uv, vec2( 0.0 ), vec2( 0.0 ) ).rgb;
		#else
			return texture2D( envMap, uv ).rgb;
		#endif
	}
	#define cubeUV_r0 1.0
	#define cubeUV_m0 - 2.0
	#define cubeUV_r1 0.8
	#define cubeUV_m1 - 1.0
	#define cubeUV_r4 0.4
	#define cubeUV_m4 2.0
	#define cubeUV_r5 0.305
	#define cubeUV_m5 3.0
	#define cubeUV_r6 0.21
	#define cubeUV_m6 4.0
	float roughnessToMip( float roughness ) {
		float mip = 0.0;
		if ( roughness >= cubeUV_r1 ) {
			mip = ( cubeUV_r0 - roughness ) * ( cubeUV_m1 - cubeUV_m0 ) / ( cubeUV_r0 - cubeUV_r1 ) + cubeUV_m0;
		} else if ( roughness >= cubeUV_r4 ) {
			mip = ( cubeUV_r1 - roughness ) * ( cubeUV_m4 - cubeUV_m1 ) / ( cubeUV_r1 - cubeUV_r4 ) + cubeUV_m1;
		} else if ( roughness >= cubeUV_r5 ) {
			mip = ( cubeUV_r4 - roughness ) * ( cubeUV_m5 - cubeUV_m4 ) / ( cubeUV_r4 - cubeUV_r5 ) + cubeUV_m4;
		} else if ( roughness >= cubeUV_r6 ) {
			mip = ( cubeUV_r5 - roughness ) * ( cubeUV_m6 - cubeUV_m5 ) / ( cubeUV_r5 - cubeUV_r6 ) + cubeUV_m5;
		} else {
			mip = - 2.0 * log2( 1.16 * roughness );		}
		return mip;
	}
	vec4 textureCubeUV( sampler2D envMap, vec3 sampleDir, float roughness ) {
		float mip = clamp( roughnessToMip( roughness ), cubeUV_m0, CUBEUV_MAX_MIP );
		float mipF = fract( mip );
		float mipInt = floor( mip );
		vec3 color0 = bilinearCubeUV( envMap, sampleDir, mipInt );
		if ( mipF == 0.0 ) {
			return vec4( color0, 1.0 );
		} else {
			vec3 color1 = bilinearCubeUV( envMap, sampleDir, mipInt + 1.0 );
			return vec4( mix( color0, color1, mipF ), 1.0 );
		}
	}
#endif`,Bg=`vec3 transformedNormal = objectNormal;
#ifdef USE_TANGENT
	vec3 transformedTangent = objectTangent;
#endif
#ifdef USE_BATCHING
	mat3 bm = mat3( batchingMatrix );
	transformedNormal /= vec3( dot( bm[ 0 ], bm[ 0 ] ), dot( bm[ 1 ], bm[ 1 ] ), dot( bm[ 2 ], bm[ 2 ] ) );
	transformedNormal = bm * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = bm * transformedTangent;
	#endif
#endif
#ifdef USE_INSTANCING
	mat3 im = mat3( instanceMatrix );
	transformedNormal /= vec3( dot( im[ 0 ], im[ 0 ] ), dot( im[ 1 ], im[ 1 ] ), dot( im[ 2 ], im[ 2 ] ) );
	transformedNormal = im * transformedNormal;
	#ifdef USE_TANGENT
		transformedTangent = im * transformedTangent;
	#endif
#endif
transformedNormal = normalMatrix * transformedNormal;
#ifdef FLIP_SIDED
	transformedNormal = - transformedNormal;
#endif
#ifdef USE_TANGENT
	transformedTangent = ( modelViewMatrix * vec4( transformedTangent, 0.0 ) ).xyz;
	#ifdef FLIP_SIDED
		transformedTangent = - transformedTangent;
	#endif
#endif`,zg=`#ifdef USE_DISPLACEMENTMAP
	uniform sampler2D displacementMap;
	uniform float displacementScale;
	uniform float displacementBias;
#endif`,Vg=`#ifdef USE_DISPLACEMENTMAP
	transformed += normalize( objectNormal ) * ( texture2D( displacementMap, vDisplacementMapUv ).x * displacementScale + displacementBias );
#endif`,Hg=`#ifdef USE_EMISSIVEMAP
	vec4 emissiveColor = texture2D( emissiveMap, vEmissiveMapUv );
	#ifdef DECODE_VIDEO_TEXTURE_EMISSIVE
		emissiveColor = sRGBTransferEOTF( emissiveColor );
	#endif
	totalEmissiveRadiance *= emissiveColor.rgb;
#endif`,Gg=`#ifdef USE_EMISSIVEMAP
	uniform sampler2D emissiveMap;
#endif`,Wg="gl_FragColor = linearToOutputTexel( gl_FragColor );",$g=`vec4 LinearTransferOETF( in vec4 value ) {
	return value;
}
vec4 sRGBTransferEOTF( in vec4 value ) {
	return vec4( mix( pow( value.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), value.rgb * 0.0773993808, vec3( lessThanEqual( value.rgb, vec3( 0.04045 ) ) ) ), value.a );
}
vec4 sRGBTransferOETF( in vec4 value ) {
	return vec4( mix( pow( value.rgb, vec3( 0.41666 ) ) * 1.055 - vec3( 0.055 ), value.rgb * 12.92, vec3( lessThanEqual( value.rgb, vec3( 0.0031308 ) ) ) ), value.a );
}`,jg=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vec3 cameraToFrag;
		if ( isOrthographic ) {
			cameraToFrag = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToFrag = normalize( vWorldPosition - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vec3 reflectVec = reflect( cameraToFrag, worldNormal );
		#else
			vec3 reflectVec = refract( cameraToFrag, worldNormal, refractionRatio );
		#endif
	#else
		vec3 reflectVec = vReflect;
	#endif
	#ifdef ENVMAP_TYPE_CUBE
		vec4 envColor = textureCube( envMap, envMapRotation * vec3( flipEnvMap * reflectVec.x, reflectVec.yz ) );
	#else
		vec4 envColor = vec4( 0.0 );
	#endif
	#ifdef ENVMAP_BLENDING_MULTIPLY
		outgoingLight = mix( outgoingLight, outgoingLight * envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_MIX )
		outgoingLight = mix( outgoingLight, envColor.xyz, specularStrength * reflectivity );
	#elif defined( ENVMAP_BLENDING_ADD )
		outgoingLight += envColor.xyz * specularStrength * reflectivity;
	#endif
#endif`,Xg=`#ifdef USE_ENVMAP
	uniform float envMapIntensity;
	uniform float flipEnvMap;
	uniform mat3 envMapRotation;
	#ifdef ENVMAP_TYPE_CUBE
		uniform samplerCube envMap;
	#else
		uniform sampler2D envMap;
	#endif
	
#endif`,qg=`#ifdef USE_ENVMAP
	uniform float reflectivity;
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		varying vec3 vWorldPosition;
		uniform float refractionRatio;
	#else
		varying vec3 vReflect;
	#endif
#endif`,Yg=`#ifdef USE_ENVMAP
	#if defined( USE_BUMPMAP ) || defined( USE_NORMALMAP ) || defined( PHONG ) || defined( LAMBERT )
		#define ENV_WORLDPOS
	#endif
	#ifdef ENV_WORLDPOS
		
		varying vec3 vWorldPosition;
	#else
		varying vec3 vReflect;
		uniform float refractionRatio;
	#endif
#endif`,Kg=`#ifdef USE_ENVMAP
	#ifdef ENV_WORLDPOS
		vWorldPosition = worldPosition.xyz;
	#else
		vec3 cameraToVertex;
		if ( isOrthographic ) {
			cameraToVertex = normalize( vec3( - viewMatrix[ 0 ][ 2 ], - viewMatrix[ 1 ][ 2 ], - viewMatrix[ 2 ][ 2 ] ) );
		} else {
			cameraToVertex = normalize( worldPosition.xyz - cameraPosition );
		}
		vec3 worldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
		#ifdef ENVMAP_MODE_REFLECTION
			vReflect = reflect( cameraToVertex, worldNormal );
		#else
			vReflect = refract( cameraToVertex, worldNormal, refractionRatio );
		#endif
	#endif
#endif`,Jg=`#ifdef USE_FOG
	vFogDepth = - mvPosition.z;
#endif`,Zg=`#ifdef USE_FOG
	varying float vFogDepth;
#endif`,Qg=`#ifdef USE_FOG
	#ifdef FOG_EXP2
		float fogFactor = 1.0 - exp( - fogDensity * fogDensity * vFogDepth * vFogDepth );
	#else
		float fogFactor = smoothstep( fogNear, fogFar, vFogDepth );
	#endif
	gl_FragColor.rgb = mix( gl_FragColor.rgb, fogColor, fogFactor );
#endif`,t_=`#ifdef USE_FOG
	uniform vec3 fogColor;
	varying float vFogDepth;
	#ifdef FOG_EXP2
		uniform float fogDensity;
	#else
		uniform float fogNear;
		uniform float fogFar;
	#endif
#endif`,e_=`#ifdef USE_GRADIENTMAP
	uniform sampler2D gradientMap;
#endif
vec3 getGradientIrradiance( vec3 normal, vec3 lightDirection ) {
	float dotNL = dot( normal, lightDirection );
	vec2 coord = vec2( dotNL * 0.5 + 0.5, 0.0 );
	#ifdef USE_GRADIENTMAP
		return vec3( texture2D( gradientMap, coord ).r );
	#else
		vec2 fw = fwidth( coord ) * 0.5;
		return mix( vec3( 0.7 ), vec3( 1.0 ), smoothstep( 0.7 - fw.x, 0.7 + fw.x, coord.x ) );
	#endif
}`,n_=`#ifdef USE_LIGHTMAP
	uniform sampler2D lightMap;
	uniform float lightMapIntensity;
#endif`,i_=`LambertMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularStrength = specularStrength;`,s_=`varying vec3 vViewPosition;
struct LambertMaterial {
	vec3 diffuseColor;
	float specularStrength;
};
void RE_Direct_Lambert( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Lambert( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in LambertMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Lambert
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Lambert`,r_=`uniform bool receiveShadow;
uniform vec3 ambientLightColor;
#if defined( USE_LIGHT_PROBES )
	uniform vec3 lightProbe[ 9 ];
#endif
vec3 shGetIrradianceAt( in vec3 normal, in vec3 shCoefficients[ 9 ] ) {
	float x = normal.x, y = normal.y, z = normal.z;
	vec3 result = shCoefficients[ 0 ] * 0.886227;
	result += shCoefficients[ 1 ] * 2.0 * 0.511664 * y;
	result += shCoefficients[ 2 ] * 2.0 * 0.511664 * z;
	result += shCoefficients[ 3 ] * 2.0 * 0.511664 * x;
	result += shCoefficients[ 4 ] * 2.0 * 0.429043 * x * y;
	result += shCoefficients[ 5 ] * 2.0 * 0.429043 * y * z;
	result += shCoefficients[ 6 ] * ( 0.743125 * z * z - 0.247708 );
	result += shCoefficients[ 7 ] * 2.0 * 0.429043 * x * z;
	result += shCoefficients[ 8 ] * 0.429043 * ( x * x - y * y );
	return result;
}
vec3 getLightProbeIrradiance( const in vec3 lightProbe[ 9 ], const in vec3 normal ) {
	vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
	vec3 irradiance = shGetIrradianceAt( worldNormal, lightProbe );
	return irradiance;
}
vec3 getAmbientLightIrradiance( const in vec3 ambientLightColor ) {
	vec3 irradiance = ambientLightColor;
	return irradiance;
}
float getDistanceAttenuation( const in float lightDistance, const in float cutoffDistance, const in float decayExponent ) {
	float distanceFalloff = 1.0 / max( pow( lightDistance, decayExponent ), 0.01 );
	if ( cutoffDistance > 0.0 ) {
		distanceFalloff *= pow2( saturate( 1.0 - pow4( lightDistance / cutoffDistance ) ) );
	}
	return distanceFalloff;
}
float getSpotAttenuation( const in float coneCosine, const in float penumbraCosine, const in float angleCosine ) {
	return smoothstep( coneCosine, penumbraCosine, angleCosine );
}
#if NUM_DIR_LIGHTS > 0
	struct DirectionalLight {
		vec3 direction;
		vec3 color;
	};
	uniform DirectionalLight directionalLights[ NUM_DIR_LIGHTS ];
	void getDirectionalLightInfo( const in DirectionalLight directionalLight, out IncidentLight light ) {
		light.color = directionalLight.color;
		light.direction = directionalLight.direction;
		light.visible = true;
	}
#endif
#if NUM_POINT_LIGHTS > 0
	struct PointLight {
		vec3 position;
		vec3 color;
		float distance;
		float decay;
	};
	uniform PointLight pointLights[ NUM_POINT_LIGHTS ];
	void getPointLightInfo( const in PointLight pointLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = pointLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float lightDistance = length( lVector );
		light.color = pointLight.color;
		light.color *= getDistanceAttenuation( lightDistance, pointLight.distance, pointLight.decay );
		light.visible = ( light.color != vec3( 0.0 ) );
	}
#endif
#if NUM_SPOT_LIGHTS > 0
	struct SpotLight {
		vec3 position;
		vec3 direction;
		vec3 color;
		float distance;
		float decay;
		float coneCos;
		float penumbraCos;
	};
	uniform SpotLight spotLights[ NUM_SPOT_LIGHTS ];
	void getSpotLightInfo( const in SpotLight spotLight, const in vec3 geometryPosition, out IncidentLight light ) {
		vec3 lVector = spotLight.position - geometryPosition;
		light.direction = normalize( lVector );
		float angleCos = dot( light.direction, spotLight.direction );
		float spotAttenuation = getSpotAttenuation( spotLight.coneCos, spotLight.penumbraCos, angleCos );
		if ( spotAttenuation > 0.0 ) {
			float lightDistance = length( lVector );
			light.color = spotLight.color * spotAttenuation;
			light.color *= getDistanceAttenuation( lightDistance, spotLight.distance, spotLight.decay );
			light.visible = ( light.color != vec3( 0.0 ) );
		} else {
			light.color = vec3( 0.0 );
			light.visible = false;
		}
	}
#endif
#if NUM_RECT_AREA_LIGHTS > 0
	struct RectAreaLight {
		vec3 color;
		vec3 position;
		vec3 halfWidth;
		vec3 halfHeight;
	};
	uniform sampler2D ltc_1;	uniform sampler2D ltc_2;
	uniform RectAreaLight rectAreaLights[ NUM_RECT_AREA_LIGHTS ];
#endif
#if NUM_HEMI_LIGHTS > 0
	struct HemisphereLight {
		vec3 direction;
		vec3 skyColor;
		vec3 groundColor;
	};
	uniform HemisphereLight hemisphereLights[ NUM_HEMI_LIGHTS ];
	vec3 getHemisphereLightIrradiance( const in HemisphereLight hemiLight, const in vec3 normal ) {
		float dotNL = dot( normal, hemiLight.direction );
		float hemiDiffuseWeight = 0.5 * dotNL + 0.5;
		vec3 irradiance = mix( hemiLight.groundColor, hemiLight.skyColor, hemiDiffuseWeight );
		return irradiance;
	}
#endif`,a_=`#ifdef USE_ENVMAP
	vec3 getIBLIrradiance( const in vec3 normal ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 worldNormal = inverseTransformDirection( normal, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * worldNormal, 1.0 );
			return PI * envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	vec3 getIBLRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness ) {
		#ifdef ENVMAP_TYPE_CUBE_UV
			vec3 reflectVec = reflect( - viewDir, normal );
			reflectVec = normalize( mix( reflectVec, normal, roughness * roughness) );
			reflectVec = inverseTransformDirection( reflectVec, viewMatrix );
			vec4 envMapColor = textureCubeUV( envMap, envMapRotation * reflectVec, roughness );
			return envMapColor.rgb * envMapIntensity;
		#else
			return vec3( 0.0 );
		#endif
	}
	#ifdef USE_ANISOTROPY
		vec3 getIBLAnisotropyRadiance( const in vec3 viewDir, const in vec3 normal, const in float roughness, const in vec3 bitangent, const in float anisotropy ) {
			#ifdef ENVMAP_TYPE_CUBE_UV
				vec3 bentNormal = cross( bitangent, viewDir );
				bentNormal = normalize( cross( bentNormal, bitangent ) );
				bentNormal = normalize( mix( bentNormal, normal, pow2( pow2( 1.0 - anisotropy * ( 1.0 - roughness ) ) ) ) );
				return getIBLRadiance( viewDir, bentNormal, roughness );
			#else
				return vec3( 0.0 );
			#endif
		}
	#endif
#endif`,o_=`ToonMaterial material;
material.diffuseColor = diffuseColor.rgb;`,l_=`varying vec3 vViewPosition;
struct ToonMaterial {
	vec3 diffuseColor;
};
void RE_Direct_Toon( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	vec3 irradiance = getGradientIrradiance( geometryNormal, directLight.direction ) * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Toon( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in ToonMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_Toon
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Toon`,c_=`BlinnPhongMaterial material;
material.diffuseColor = diffuseColor.rgb;
material.specularColor = specular;
material.specularShininess = shininess;
material.specularStrength = specularStrength;`,u_=`varying vec3 vViewPosition;
struct BlinnPhongMaterial {
	vec3 diffuseColor;
	vec3 specularColor;
	float specularShininess;
	float specularStrength;
};
void RE_Direct_BlinnPhong( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
	reflectedLight.directSpecular += irradiance * BRDF_BlinnPhong( directLight.direction, geometryViewDir, geometryNormal, material.specularColor, material.specularShininess ) * material.specularStrength;
}
void RE_IndirectDiffuse_BlinnPhong( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in BlinnPhongMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
#define RE_Direct				RE_Direct_BlinnPhong
#define RE_IndirectDiffuse		RE_IndirectDiffuse_BlinnPhong`,h_=`PhysicalMaterial material;
material.diffuseColor = diffuseColor.rgb * ( 1.0 - metalnessFactor );
vec3 dxy = max( abs( dFdx( nonPerturbedNormal ) ), abs( dFdy( nonPerturbedNormal ) ) );
float geometryRoughness = max( max( dxy.x, dxy.y ), dxy.z );
material.roughness = max( roughnessFactor, 0.0525 );material.roughness += geometryRoughness;
material.roughness = min( material.roughness, 1.0 );
#ifdef IOR
	material.ior = ior;
	#ifdef USE_SPECULAR
		float specularIntensityFactor = specularIntensity;
		vec3 specularColorFactor = specularColor;
		#ifdef USE_SPECULAR_COLORMAP
			specularColorFactor *= texture2D( specularColorMap, vSpecularColorMapUv ).rgb;
		#endif
		#ifdef USE_SPECULAR_INTENSITYMAP
			specularIntensityFactor *= texture2D( specularIntensityMap, vSpecularIntensityMapUv ).a;
		#endif
		material.specularF90 = mix( specularIntensityFactor, 1.0, metalnessFactor );
	#else
		float specularIntensityFactor = 1.0;
		vec3 specularColorFactor = vec3( 1.0 );
		material.specularF90 = 1.0;
	#endif
	material.specularColor = mix( min( pow2( ( material.ior - 1.0 ) / ( material.ior + 1.0 ) ) * specularColorFactor, vec3( 1.0 ) ) * specularIntensityFactor, diffuseColor.rgb, metalnessFactor );
#else
	material.specularColor = mix( vec3( 0.04 ), diffuseColor.rgb, metalnessFactor );
	material.specularF90 = 1.0;
#endif
#ifdef USE_CLEARCOAT
	material.clearcoat = clearcoat;
	material.clearcoatRoughness = clearcoatRoughness;
	material.clearcoatF0 = vec3( 0.04 );
	material.clearcoatF90 = 1.0;
	#ifdef USE_CLEARCOATMAP
		material.clearcoat *= texture2D( clearcoatMap, vClearcoatMapUv ).x;
	#endif
	#ifdef USE_CLEARCOAT_ROUGHNESSMAP
		material.clearcoatRoughness *= texture2D( clearcoatRoughnessMap, vClearcoatRoughnessMapUv ).y;
	#endif
	material.clearcoat = saturate( material.clearcoat );	material.clearcoatRoughness = max( material.clearcoatRoughness, 0.0525 );
	material.clearcoatRoughness += geometryRoughness;
	material.clearcoatRoughness = min( material.clearcoatRoughness, 1.0 );
#endif
#ifdef USE_DISPERSION
	material.dispersion = dispersion;
#endif
#ifdef USE_IRIDESCENCE
	material.iridescence = iridescence;
	material.iridescenceIOR = iridescenceIOR;
	#ifdef USE_IRIDESCENCEMAP
		material.iridescence *= texture2D( iridescenceMap, vIridescenceMapUv ).r;
	#endif
	#ifdef USE_IRIDESCENCE_THICKNESSMAP
		material.iridescenceThickness = (iridescenceThicknessMaximum - iridescenceThicknessMinimum) * texture2D( iridescenceThicknessMap, vIridescenceThicknessMapUv ).g + iridescenceThicknessMinimum;
	#else
		material.iridescenceThickness = iridescenceThicknessMaximum;
	#endif
#endif
#ifdef USE_SHEEN
	material.sheenColor = sheenColor;
	#ifdef USE_SHEEN_COLORMAP
		material.sheenColor *= texture2D( sheenColorMap, vSheenColorMapUv ).rgb;
	#endif
	material.sheenRoughness = clamp( sheenRoughness, 0.07, 1.0 );
	#ifdef USE_SHEEN_ROUGHNESSMAP
		material.sheenRoughness *= texture2D( sheenRoughnessMap, vSheenRoughnessMapUv ).a;
	#endif
#endif
#ifdef USE_ANISOTROPY
	#ifdef USE_ANISOTROPYMAP
		mat2 anisotropyMat = mat2( anisotropyVector.x, anisotropyVector.y, - anisotropyVector.y, anisotropyVector.x );
		vec3 anisotropyPolar = texture2D( anisotropyMap, vAnisotropyMapUv ).rgb;
		vec2 anisotropyV = anisotropyMat * normalize( 2.0 * anisotropyPolar.rg - vec2( 1.0 ) ) * anisotropyPolar.b;
	#else
		vec2 anisotropyV = anisotropyVector;
	#endif
	material.anisotropy = length( anisotropyV );
	if( material.anisotropy == 0.0 ) {
		anisotropyV = vec2( 1.0, 0.0 );
	} else {
		anisotropyV /= material.anisotropy;
		material.anisotropy = saturate( material.anisotropy );
	}
	material.alphaT = mix( pow2( material.roughness ), 1.0, pow2( material.anisotropy ) );
	material.anisotropyT = tbn[ 0 ] * anisotropyV.x + tbn[ 1 ] * anisotropyV.y;
	material.anisotropyB = tbn[ 1 ] * anisotropyV.x - tbn[ 0 ] * anisotropyV.y;
#endif`,d_=`struct PhysicalMaterial {
	vec3 diffuseColor;
	float roughness;
	vec3 specularColor;
	float specularF90;
	float dispersion;
	#ifdef USE_CLEARCOAT
		float clearcoat;
		float clearcoatRoughness;
		vec3 clearcoatF0;
		float clearcoatF90;
	#endif
	#ifdef USE_IRIDESCENCE
		float iridescence;
		float iridescenceIOR;
		float iridescenceThickness;
		vec3 iridescenceFresnel;
		vec3 iridescenceF0;
	#endif
	#ifdef USE_SHEEN
		vec3 sheenColor;
		float sheenRoughness;
	#endif
	#ifdef IOR
		float ior;
	#endif
	#ifdef USE_TRANSMISSION
		float transmission;
		float transmissionAlpha;
		float thickness;
		float attenuationDistance;
		vec3 attenuationColor;
	#endif
	#ifdef USE_ANISOTROPY
		float anisotropy;
		float alphaT;
		vec3 anisotropyT;
		vec3 anisotropyB;
	#endif
};
vec3 clearcoatSpecularDirect = vec3( 0.0 );
vec3 clearcoatSpecularIndirect = vec3( 0.0 );
vec3 sheenSpecularDirect = vec3( 0.0 );
vec3 sheenSpecularIndirect = vec3(0.0 );
vec3 Schlick_to_F0( const in vec3 f, const in float f90, const in float dotVH ) {
    float x = clamp( 1.0 - dotVH, 0.0, 1.0 );
    float x2 = x * x;
    float x5 = clamp( x * x2 * x2, 0.0, 0.9999 );
    return ( f - vec3( f90 ) * x5 ) / ( 1.0 - x5 );
}
float V_GGX_SmithCorrelated( const in float alpha, const in float dotNL, const in float dotNV ) {
	float a2 = pow2( alpha );
	float gv = dotNL * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNV ) );
	float gl = dotNV * sqrt( a2 + ( 1.0 - a2 ) * pow2( dotNL ) );
	return 0.5 / max( gv + gl, EPSILON );
}
float D_GGX( const in float alpha, const in float dotNH ) {
	float a2 = pow2( alpha );
	float denom = pow2( dotNH ) * ( a2 - 1.0 ) + 1.0;
	return RECIPROCAL_PI * a2 / pow2( denom );
}
#ifdef USE_ANISOTROPY
	float V_GGX_SmithCorrelated_Anisotropic( const in float alphaT, const in float alphaB, const in float dotTV, const in float dotBV, const in float dotTL, const in float dotBL, const in float dotNV, const in float dotNL ) {
		float gv = dotNL * length( vec3( alphaT * dotTV, alphaB * dotBV, dotNV ) );
		float gl = dotNV * length( vec3( alphaT * dotTL, alphaB * dotBL, dotNL ) );
		float v = 0.5 / ( gv + gl );
		return saturate(v);
	}
	float D_GGX_Anisotropic( const in float alphaT, const in float alphaB, const in float dotNH, const in float dotTH, const in float dotBH ) {
		float a2 = alphaT * alphaB;
		highp vec3 v = vec3( alphaB * dotTH, alphaT * dotBH, a2 * dotNH );
		highp float v2 = dot( v, v );
		float w2 = a2 / v2;
		return RECIPROCAL_PI * a2 * pow2 ( w2 );
	}
#endif
#ifdef USE_CLEARCOAT
	vec3 BRDF_GGX_Clearcoat( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material) {
		vec3 f0 = material.clearcoatF0;
		float f90 = material.clearcoatF90;
		float roughness = material.clearcoatRoughness;
		float alpha = pow2( roughness );
		vec3 halfDir = normalize( lightDir + viewDir );
		float dotNL = saturate( dot( normal, lightDir ) );
		float dotNV = saturate( dot( normal, viewDir ) );
		float dotNH = saturate( dot( normal, halfDir ) );
		float dotVH = saturate( dot( viewDir, halfDir ) );
		vec3 F = F_Schlick( f0, f90, dotVH );
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
		return F * ( V * D );
	}
#endif
vec3 BRDF_GGX( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, const in PhysicalMaterial material ) {
	vec3 f0 = material.specularColor;
	float f90 = material.specularF90;
	float roughness = material.roughness;
	float alpha = pow2( roughness );
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float dotVH = saturate( dot( viewDir, halfDir ) );
	vec3 F = F_Schlick( f0, f90, dotVH );
	#ifdef USE_IRIDESCENCE
		F = mix( F, material.iridescenceFresnel, material.iridescence );
	#endif
	#ifdef USE_ANISOTROPY
		float dotTL = dot( material.anisotropyT, lightDir );
		float dotTV = dot( material.anisotropyT, viewDir );
		float dotTH = dot( material.anisotropyT, halfDir );
		float dotBL = dot( material.anisotropyB, lightDir );
		float dotBV = dot( material.anisotropyB, viewDir );
		float dotBH = dot( material.anisotropyB, halfDir );
		float V = V_GGX_SmithCorrelated_Anisotropic( material.alphaT, alpha, dotTV, dotBV, dotTL, dotBL, dotNV, dotNL );
		float D = D_GGX_Anisotropic( material.alphaT, alpha, dotNH, dotTH, dotBH );
	#else
		float V = V_GGX_SmithCorrelated( alpha, dotNL, dotNV );
		float D = D_GGX( alpha, dotNH );
	#endif
	return F * ( V * D );
}
vec2 LTC_Uv( const in vec3 N, const in vec3 V, const in float roughness ) {
	const float LUT_SIZE = 64.0;
	const float LUT_SCALE = ( LUT_SIZE - 1.0 ) / LUT_SIZE;
	const float LUT_BIAS = 0.5 / LUT_SIZE;
	float dotNV = saturate( dot( N, V ) );
	vec2 uv = vec2( roughness, sqrt( 1.0 - dotNV ) );
	uv = uv * LUT_SCALE + LUT_BIAS;
	return uv;
}
float LTC_ClippedSphereFormFactor( const in vec3 f ) {
	float l = length( f );
	return max( ( l * l + f.z ) / ( l + 1.0 ), 0.0 );
}
vec3 LTC_EdgeVectorFormFactor( const in vec3 v1, const in vec3 v2 ) {
	float x = dot( v1, v2 );
	float y = abs( x );
	float a = 0.8543985 + ( 0.4965155 + 0.0145206 * y ) * y;
	float b = 3.4175940 + ( 4.1616724 + y ) * y;
	float v = a / b;
	float theta_sintheta = ( x > 0.0 ) ? v : 0.5 * inversesqrt( max( 1.0 - x * x, 1e-7 ) ) - v;
	return cross( v1, v2 ) * theta_sintheta;
}
vec3 LTC_Evaluate( const in vec3 N, const in vec3 V, const in vec3 P, const in mat3 mInv, const in vec3 rectCoords[ 4 ] ) {
	vec3 v1 = rectCoords[ 1 ] - rectCoords[ 0 ];
	vec3 v2 = rectCoords[ 3 ] - rectCoords[ 0 ];
	vec3 lightNormal = cross( v1, v2 );
	if( dot( lightNormal, P - rectCoords[ 0 ] ) < 0.0 ) return vec3( 0.0 );
	vec3 T1, T2;
	T1 = normalize( V - N * dot( V, N ) );
	T2 = - cross( N, T1 );
	mat3 mat = mInv * transposeMat3( mat3( T1, T2, N ) );
	vec3 coords[ 4 ];
	coords[ 0 ] = mat * ( rectCoords[ 0 ] - P );
	coords[ 1 ] = mat * ( rectCoords[ 1 ] - P );
	coords[ 2 ] = mat * ( rectCoords[ 2 ] - P );
	coords[ 3 ] = mat * ( rectCoords[ 3 ] - P );
	coords[ 0 ] = normalize( coords[ 0 ] );
	coords[ 1 ] = normalize( coords[ 1 ] );
	coords[ 2 ] = normalize( coords[ 2 ] );
	coords[ 3 ] = normalize( coords[ 3 ] );
	vec3 vectorFormFactor = vec3( 0.0 );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 0 ], coords[ 1 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 1 ], coords[ 2 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 2 ], coords[ 3 ] );
	vectorFormFactor += LTC_EdgeVectorFormFactor( coords[ 3 ], coords[ 0 ] );
	float result = LTC_ClippedSphereFormFactor( vectorFormFactor );
	return vec3( result );
}
#if defined( USE_SHEEN )
float D_Charlie( float roughness, float dotNH ) {
	float alpha = pow2( roughness );
	float invAlpha = 1.0 / alpha;
	float cos2h = dotNH * dotNH;
	float sin2h = max( 1.0 - cos2h, 0.0078125 );
	return ( 2.0 + invAlpha ) * pow( sin2h, invAlpha * 0.5 ) / ( 2.0 * PI );
}
float V_Neubelt( float dotNV, float dotNL ) {
	return saturate( 1.0 / ( 4.0 * ( dotNL + dotNV - dotNL * dotNV ) ) );
}
vec3 BRDF_Sheen( const in vec3 lightDir, const in vec3 viewDir, const in vec3 normal, vec3 sheenColor, const in float sheenRoughness ) {
	vec3 halfDir = normalize( lightDir + viewDir );
	float dotNL = saturate( dot( normal, lightDir ) );
	float dotNV = saturate( dot( normal, viewDir ) );
	float dotNH = saturate( dot( normal, halfDir ) );
	float D = D_Charlie( sheenRoughness, dotNH );
	float V = V_Neubelt( dotNV, dotNL );
	return sheenColor * ( D * V );
}
#endif
float IBLSheenBRDF( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	float r2 = roughness * roughness;
	float a = roughness < 0.25 ? -339.2 * r2 + 161.4 * roughness - 25.9 : -8.48 * r2 + 14.3 * roughness - 9.95;
	float b = roughness < 0.25 ? 44.0 * r2 - 23.7 * roughness + 3.26 : 1.97 * r2 - 3.27 * roughness + 0.72;
	float DG = exp( a * dotNV + b ) + ( roughness < 0.25 ? 0.0 : 0.1 * ( roughness - 0.25 ) );
	return saturate( DG * RECIPROCAL_PI );
}
vec2 DFGApprox( const in vec3 normal, const in vec3 viewDir, const in float roughness ) {
	float dotNV = saturate( dot( normal, viewDir ) );
	const vec4 c0 = vec4( - 1, - 0.0275, - 0.572, 0.022 );
	const vec4 c1 = vec4( 1, 0.0425, 1.04, - 0.04 );
	vec4 r = roughness * c0 + c1;
	float a004 = min( r.x * r.x, exp2( - 9.28 * dotNV ) ) * r.x + r.y;
	vec2 fab = vec2( - 1.04, 1.04 ) * a004 + r.zw;
	return fab;
}
vec3 EnvironmentBRDF( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness ) {
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	return specularColor * fab.x + specularF90 * fab.y;
}
#ifdef USE_IRIDESCENCE
void computeMultiscatteringIridescence( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float iridescence, const in vec3 iridescenceF0, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#else
void computeMultiscattering( const in vec3 normal, const in vec3 viewDir, const in vec3 specularColor, const in float specularF90, const in float roughness, inout vec3 singleScatter, inout vec3 multiScatter ) {
#endif
	vec2 fab = DFGApprox( normal, viewDir, roughness );
	#ifdef USE_IRIDESCENCE
		vec3 Fr = mix( specularColor, iridescenceF0, iridescence );
	#else
		vec3 Fr = specularColor;
	#endif
	vec3 FssEss = Fr * fab.x + specularF90 * fab.y;
	float Ess = fab.x + fab.y;
	float Ems = 1.0 - Ess;
	vec3 Favg = Fr + ( 1.0 - Fr ) * 0.047619;	vec3 Fms = FssEss * Favg / ( 1.0 - Ems * Favg );
	singleScatter += FssEss;
	multiScatter += Fms * Ems;
}
#if NUM_RECT_AREA_LIGHTS > 0
	void RE_Direct_RectArea_Physical( const in RectAreaLight rectAreaLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
		vec3 normal = geometryNormal;
		vec3 viewDir = geometryViewDir;
		vec3 position = geometryPosition;
		vec3 lightPos = rectAreaLight.position;
		vec3 halfWidth = rectAreaLight.halfWidth;
		vec3 halfHeight = rectAreaLight.halfHeight;
		vec3 lightColor = rectAreaLight.color;
		float roughness = material.roughness;
		vec3 rectCoords[ 4 ];
		rectCoords[ 0 ] = lightPos + halfWidth - halfHeight;		rectCoords[ 1 ] = lightPos - halfWidth - halfHeight;
		rectCoords[ 2 ] = lightPos - halfWidth + halfHeight;
		rectCoords[ 3 ] = lightPos + halfWidth + halfHeight;
		vec2 uv = LTC_Uv( normal, viewDir, roughness );
		vec4 t1 = texture2D( ltc_1, uv );
		vec4 t2 = texture2D( ltc_2, uv );
		mat3 mInv = mat3(
			vec3( t1.x, 0, t1.y ),
			vec3(    0, 1,    0 ),
			vec3( t1.z, 0, t1.w )
		);
		vec3 fresnel = ( material.specularColor * t2.x + ( vec3( 1.0 ) - material.specularColor ) * t2.y );
		reflectedLight.directSpecular += lightColor * fresnel * LTC_Evaluate( normal, viewDir, position, mInv, rectCoords );
		reflectedLight.directDiffuse += lightColor * material.diffuseColor * LTC_Evaluate( normal, viewDir, position, mat3( 1.0 ), rectCoords );
	}
#endif
void RE_Direct_Physical( const in IncidentLight directLight, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	float dotNL = saturate( dot( geometryNormal, directLight.direction ) );
	vec3 irradiance = dotNL * directLight.color;
	#ifdef USE_CLEARCOAT
		float dotNLcc = saturate( dot( geometryClearcoatNormal, directLight.direction ) );
		vec3 ccIrradiance = dotNLcc * directLight.color;
		clearcoatSpecularDirect += ccIrradiance * BRDF_GGX_Clearcoat( directLight.direction, geometryViewDir, geometryClearcoatNormal, material );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularDirect += irradiance * BRDF_Sheen( directLight.direction, geometryViewDir, geometryNormal, material.sheenColor, material.sheenRoughness );
	#endif
	reflectedLight.directSpecular += irradiance * BRDF_GGX( directLight.direction, geometryViewDir, geometryNormal, material );
	reflectedLight.directDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectDiffuse_Physical( const in vec3 irradiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight ) {
	reflectedLight.indirectDiffuse += irradiance * BRDF_Lambert( material.diffuseColor );
}
void RE_IndirectSpecular_Physical( const in vec3 radiance, const in vec3 irradiance, const in vec3 clearcoatRadiance, const in vec3 geometryPosition, const in vec3 geometryNormal, const in vec3 geometryViewDir, const in vec3 geometryClearcoatNormal, const in PhysicalMaterial material, inout ReflectedLight reflectedLight) {
	#ifdef USE_CLEARCOAT
		clearcoatSpecularIndirect += clearcoatRadiance * EnvironmentBRDF( geometryClearcoatNormal, geometryViewDir, material.clearcoatF0, material.clearcoatF90, material.clearcoatRoughness );
	#endif
	#ifdef USE_SHEEN
		sheenSpecularIndirect += irradiance * material.sheenColor * IBLSheenBRDF( geometryNormal, geometryViewDir, material.sheenRoughness );
	#endif
	vec3 singleScattering = vec3( 0.0 );
	vec3 multiScattering = vec3( 0.0 );
	vec3 cosineWeightedIrradiance = irradiance * RECIPROCAL_PI;
	#ifdef USE_IRIDESCENCE
		computeMultiscatteringIridescence( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.iridescence, material.iridescenceFresnel, material.roughness, singleScattering, multiScattering );
	#else
		computeMultiscattering( geometryNormal, geometryViewDir, material.specularColor, material.specularF90, material.roughness, singleScattering, multiScattering );
	#endif
	vec3 totalScattering = singleScattering + multiScattering;
	vec3 diffuse = material.diffuseColor * ( 1.0 - max( max( totalScattering.r, totalScattering.g ), totalScattering.b ) );
	reflectedLight.indirectSpecular += radiance * singleScattering;
	reflectedLight.indirectSpecular += multiScattering * cosineWeightedIrradiance;
	reflectedLight.indirectDiffuse += diffuse * cosineWeightedIrradiance;
}
#define RE_Direct				RE_Direct_Physical
#define RE_Direct_RectArea		RE_Direct_RectArea_Physical
#define RE_IndirectDiffuse		RE_IndirectDiffuse_Physical
#define RE_IndirectSpecular		RE_IndirectSpecular_Physical
float computeSpecularOcclusion( const in float dotNV, const in float ambientOcclusion, const in float roughness ) {
	return saturate( pow( dotNV + ambientOcclusion, exp2( - 16.0 * roughness - 1.0 ) ) - 1.0 + ambientOcclusion );
}`,f_=`
vec3 geometryPosition = - vViewPosition;
vec3 geometryNormal = normal;
vec3 geometryViewDir = ( isOrthographic ) ? vec3( 0, 0, 1 ) : normalize( vViewPosition );
vec3 geometryClearcoatNormal = vec3( 0.0 );
#ifdef USE_CLEARCOAT
	geometryClearcoatNormal = clearcoatNormal;
#endif
#ifdef USE_IRIDESCENCE
	float dotNVi = saturate( dot( normal, geometryViewDir ) );
	if ( material.iridescenceThickness == 0.0 ) {
		material.iridescence = 0.0;
	} else {
		material.iridescence = saturate( material.iridescence );
	}
	if ( material.iridescence > 0.0 ) {
		material.iridescenceFresnel = evalIridescence( 1.0, material.iridescenceIOR, dotNVi, material.iridescenceThickness, material.specularColor );
		material.iridescenceF0 = Schlick_to_F0( material.iridescenceFresnel, 1.0, dotNVi );
	}
#endif
IncidentLight directLight;
#if ( NUM_POINT_LIGHTS > 0 ) && defined( RE_Direct )
	PointLight pointLight;
	#if defined( USE_SHADOWMAP ) && NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHTS; i ++ ) {
		pointLight = pointLights[ i ];
		getPointLightInfo( pointLight, geometryPosition, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_POINT_LIGHT_SHADOWS )
		pointLightShadow = pointLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getPointShadow( pointShadowMap[ i ], pointLightShadow.shadowMapSize, pointLightShadow.shadowIntensity, pointLightShadow.shadowBias, pointLightShadow.shadowRadius, vPointShadowCoord[ i ], pointLightShadow.shadowCameraNear, pointLightShadow.shadowCameraFar ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_SPOT_LIGHTS > 0 ) && defined( RE_Direct )
	SpotLight spotLight;
	vec4 spotColor;
	vec3 spotLightCoord;
	bool inSpotLightMap;
	#if defined( USE_SHADOWMAP ) && NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHTS; i ++ ) {
		spotLight = spotLights[ i ];
		getSpotLightInfo( spotLight, geometryPosition, directLight );
		#if ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#define SPOT_LIGHT_MAP_INDEX UNROLLED_LOOP_INDEX
		#elif ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		#define SPOT_LIGHT_MAP_INDEX NUM_SPOT_LIGHT_MAPS
		#else
		#define SPOT_LIGHT_MAP_INDEX ( UNROLLED_LOOP_INDEX - NUM_SPOT_LIGHT_SHADOWS + NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS )
		#endif
		#if ( SPOT_LIGHT_MAP_INDEX < NUM_SPOT_LIGHT_MAPS )
			spotLightCoord = vSpotLightCoord[ i ].xyz / vSpotLightCoord[ i ].w;
			inSpotLightMap = all( lessThan( abs( spotLightCoord * 2. - 1. ), vec3( 1.0 ) ) );
			spotColor = texture2D( spotLightMap[ SPOT_LIGHT_MAP_INDEX ], spotLightCoord.xy );
			directLight.color = inSpotLightMap ? directLight.color * spotColor.rgb : directLight.color;
		#endif
		#undef SPOT_LIGHT_MAP_INDEX
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
		spotLightShadow = spotLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( spotShadowMap[ i ], spotLightShadow.shadowMapSize, spotLightShadow.shadowIntensity, spotLightShadow.shadowBias, spotLightShadow.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_DIR_LIGHTS > 0 ) && defined( RE_Direct )
	DirectionalLight directionalLight;
	#if defined( USE_SHADOWMAP ) && NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLightShadow;
	#endif
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHTS; i ++ ) {
		directionalLight = directionalLights[ i ];
		getDirectionalLightInfo( directionalLight, directLight );
		#if defined( USE_SHADOWMAP ) && ( UNROLLED_LOOP_INDEX < NUM_DIR_LIGHT_SHADOWS )
		directionalLightShadow = directionalLightShadows[ i ];
		directLight.color *= ( directLight.visible && receiveShadow ) ? getShadow( directionalShadowMap[ i ], directionalLightShadow.shadowMapSize, directionalLightShadow.shadowIntensity, directionalLightShadow.shadowBias, directionalLightShadow.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
		#endif
		RE_Direct( directLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if ( NUM_RECT_AREA_LIGHTS > 0 ) && defined( RE_Direct_RectArea )
	RectAreaLight rectAreaLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_RECT_AREA_LIGHTS; i ++ ) {
		rectAreaLight = rectAreaLights[ i ];
		RE_Direct_RectArea( rectAreaLight, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
	}
	#pragma unroll_loop_end
#endif
#if defined( RE_IndirectDiffuse )
	vec3 iblIrradiance = vec3( 0.0 );
	vec3 irradiance = getAmbientLightIrradiance( ambientLightColor );
	#if defined( USE_LIGHT_PROBES )
		irradiance += getLightProbeIrradiance( lightProbe, geometryNormal );
	#endif
	#if ( NUM_HEMI_LIGHTS > 0 )
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_HEMI_LIGHTS; i ++ ) {
			irradiance += getHemisphereLightIrradiance( hemisphereLights[ i ], geometryNormal );
		}
		#pragma unroll_loop_end
	#endif
#endif
#if defined( RE_IndirectSpecular )
	vec3 radiance = vec3( 0.0 );
	vec3 clearcoatRadiance = vec3( 0.0 );
#endif`,p_=`#if defined( RE_IndirectDiffuse )
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		vec3 lightMapIrradiance = lightMapTexel.rgb * lightMapIntensity;
		irradiance += lightMapIrradiance;
	#endif
	#if defined( USE_ENVMAP ) && defined( STANDARD ) && defined( ENVMAP_TYPE_CUBE_UV )
		iblIrradiance += getIBLIrradiance( geometryNormal );
	#endif
#endif
#if defined( USE_ENVMAP ) && defined( RE_IndirectSpecular )
	#ifdef USE_ANISOTROPY
		radiance += getIBLAnisotropyRadiance( geometryViewDir, geometryNormal, material.roughness, material.anisotropyB, material.anisotropy );
	#else
		radiance += getIBLRadiance( geometryViewDir, geometryNormal, material.roughness );
	#endif
	#ifdef USE_CLEARCOAT
		clearcoatRadiance += getIBLRadiance( geometryViewDir, geometryClearcoatNormal, material.clearcoatRoughness );
	#endif
#endif`,m_=`#if defined( RE_IndirectDiffuse )
	RE_IndirectDiffuse( irradiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif
#if defined( RE_IndirectSpecular )
	RE_IndirectSpecular( radiance, iblIrradiance, clearcoatRadiance, geometryPosition, geometryNormal, geometryViewDir, geometryClearcoatNormal, material, reflectedLight );
#endif`,g_=`#if defined( USE_LOGDEPTHBUF )
	gl_FragDepth = vIsPerspective == 0.0 ? gl_FragCoord.z : log2( vFragDepth ) * logDepthBufFC * 0.5;
#endif`,__=`#if defined( USE_LOGDEPTHBUF )
	uniform float logDepthBufFC;
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,v_=`#ifdef USE_LOGDEPTHBUF
	varying float vFragDepth;
	varying float vIsPerspective;
#endif`,x_=`#ifdef USE_LOGDEPTHBUF
	vFragDepth = 1.0 + gl_Position.w;
	vIsPerspective = float( isPerspectiveMatrix( projectionMatrix ) );
#endif`,b_=`#ifdef USE_MAP
	vec4 sampledDiffuseColor = texture2D( map, vMapUv );
	#ifdef DECODE_VIDEO_TEXTURE
		sampledDiffuseColor = sRGBTransferEOTF( sampledDiffuseColor );
	#endif
	diffuseColor *= sampledDiffuseColor;
#endif`,y_=`#ifdef USE_MAP
	uniform sampler2D map;
#endif`,M_=`#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
	#if defined( USE_POINTS_UV )
		vec2 uv = vUv;
	#else
		vec2 uv = ( uvTransform * vec3( gl_PointCoord.x, 1.0 - gl_PointCoord.y, 1 ) ).xy;
	#endif
#endif
#ifdef USE_MAP
	diffuseColor *= texture2D( map, uv );
#endif
#ifdef USE_ALPHAMAP
	diffuseColor.a *= texture2D( alphaMap, uv ).g;
#endif`,S_=`#if defined( USE_POINTS_UV )
	varying vec2 vUv;
#else
	#if defined( USE_MAP ) || defined( USE_ALPHAMAP )
		uniform mat3 uvTransform;
	#endif
#endif
#ifdef USE_MAP
	uniform sampler2D map;
#endif
#ifdef USE_ALPHAMAP
	uniform sampler2D alphaMap;
#endif`,E_=`float metalnessFactor = metalness;
#ifdef USE_METALNESSMAP
	vec4 texelMetalness = texture2D( metalnessMap, vMetalnessMapUv );
	metalnessFactor *= texelMetalness.b;
#endif`,T_=`#ifdef USE_METALNESSMAP
	uniform sampler2D metalnessMap;
#endif`,w_=`#ifdef USE_INSTANCING_MORPH
	float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	float morphTargetBaseInfluence = texelFetch( morphTexture, ivec2( 0, gl_InstanceID ), 0 ).r;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		morphTargetInfluences[i] =  texelFetch( morphTexture, ivec2( i + 1, gl_InstanceID ), 0 ).r;
	}
#endif`,A_=`#if defined( USE_MORPHCOLORS )
	vColor *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		#if defined( USE_COLOR_ALPHA )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ) * morphTargetInfluences[ i ];
		#elif defined( USE_COLOR )
			if ( morphTargetInfluences[ i ] != 0.0 ) vColor += getMorph( gl_VertexID, i, 2 ).rgb * morphTargetInfluences[ i ];
		#endif
	}
#endif`,C_=`#ifdef USE_MORPHNORMALS
	objectNormal *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) objectNormal += getMorph( gl_VertexID, i, 1 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,R_=`#ifdef USE_MORPHTARGETS
	#ifndef USE_INSTANCING_MORPH
		uniform float morphTargetBaseInfluence;
		uniform float morphTargetInfluences[ MORPHTARGETS_COUNT ];
	#endif
	uniform sampler2DArray morphTargetsTexture;
	uniform ivec2 morphTargetsTextureSize;
	vec4 getMorph( const in int vertexIndex, const in int morphTargetIndex, const in int offset ) {
		int texelIndex = vertexIndex * MORPHTARGETS_TEXTURE_STRIDE + offset;
		int y = texelIndex / morphTargetsTextureSize.x;
		int x = texelIndex - y * morphTargetsTextureSize.x;
		ivec3 morphUV = ivec3( x, y, morphTargetIndex );
		return texelFetch( morphTargetsTexture, morphUV, 0 );
	}
#endif`,P_=`#ifdef USE_MORPHTARGETS
	transformed *= morphTargetBaseInfluence;
	for ( int i = 0; i < MORPHTARGETS_COUNT; i ++ ) {
		if ( morphTargetInfluences[ i ] != 0.0 ) transformed += getMorph( gl_VertexID, i, 0 ).xyz * morphTargetInfluences[ i ];
	}
#endif`,L_=`float faceDirection = gl_FrontFacing ? 1.0 : - 1.0;
#ifdef FLAT_SHADED
	vec3 fdx = dFdx( vViewPosition );
	vec3 fdy = dFdy( vViewPosition );
	vec3 normal = normalize( cross( fdx, fdy ) );
#else
	vec3 normal = normalize( vNormal );
	#ifdef DOUBLE_SIDED
		normal *= faceDirection;
	#endif
#endif
#if defined( USE_NORMALMAP_TANGENTSPACE ) || defined( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY )
	#ifdef USE_TANGENT
		mat3 tbn = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn = getTangentFrame( - vViewPosition, normal,
		#if defined( USE_NORMALMAP )
			vNormalMapUv
		#elif defined( USE_CLEARCOAT_NORMALMAP )
			vClearcoatNormalMapUv
		#else
			vUv
		#endif
		);
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn[0] *= faceDirection;
		tbn[1] *= faceDirection;
	#endif
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	#ifdef USE_TANGENT
		mat3 tbn2 = mat3( normalize( vTangent ), normalize( vBitangent ), normal );
	#else
		mat3 tbn2 = getTangentFrame( - vViewPosition, normal, vClearcoatNormalMapUv );
	#endif
	#if defined( DOUBLE_SIDED ) && ! defined( FLAT_SHADED )
		tbn2[0] *= faceDirection;
		tbn2[1] *= faceDirection;
	#endif
#endif
vec3 nonPerturbedNormal = normal;`,N_=`#ifdef USE_NORMALMAP_OBJECTSPACE
	normal = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	#ifdef FLIP_SIDED
		normal = - normal;
	#endif
	#ifdef DOUBLE_SIDED
		normal = normal * faceDirection;
	#endif
	normal = normalize( normalMatrix * normal );
#elif defined( USE_NORMALMAP_TANGENTSPACE )
	vec3 mapN = texture2D( normalMap, vNormalMapUv ).xyz * 2.0 - 1.0;
	mapN.xy *= normalScale;
	normal = normalize( tbn * mapN );
#elif defined( USE_BUMPMAP )
	normal = perturbNormalArb( - vViewPosition, normal, dHdxy_fwd(), faceDirection );
#endif`,D_=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,I_=`#ifndef FLAT_SHADED
	varying vec3 vNormal;
	#ifdef USE_TANGENT
		varying vec3 vTangent;
		varying vec3 vBitangent;
	#endif
#endif`,U_=`#ifndef FLAT_SHADED
	vNormal = normalize( transformedNormal );
	#ifdef USE_TANGENT
		vTangent = normalize( transformedTangent );
		vBitangent = normalize( cross( vNormal, vTangent ) * tangent.w );
	#endif
#endif`,k_=`#ifdef USE_NORMALMAP
	uniform sampler2D normalMap;
	uniform vec2 normalScale;
#endif
#ifdef USE_NORMALMAP_OBJECTSPACE
	uniform mat3 normalMatrix;
#endif
#if ! defined ( USE_TANGENT ) && ( defined ( USE_NORMALMAP_TANGENTSPACE ) || defined ( USE_CLEARCOAT_NORMALMAP ) || defined( USE_ANISOTROPY ) )
	mat3 getTangentFrame( vec3 eye_pos, vec3 surf_norm, vec2 uv ) {
		vec3 q0 = dFdx( eye_pos.xyz );
		vec3 q1 = dFdy( eye_pos.xyz );
		vec2 st0 = dFdx( uv.st );
		vec2 st1 = dFdy( uv.st );
		vec3 N = surf_norm;
		vec3 q1perp = cross( q1, N );
		vec3 q0perp = cross( N, q0 );
		vec3 T = q1perp * st0.x + q0perp * st1.x;
		vec3 B = q1perp * st0.y + q0perp * st1.y;
		float det = max( dot( T, T ), dot( B, B ) );
		float scale = ( det == 0.0 ) ? 0.0 : inversesqrt( det );
		return mat3( T * scale, B * scale, N );
	}
#endif`,F_=`#ifdef USE_CLEARCOAT
	vec3 clearcoatNormal = nonPerturbedNormal;
#endif`,O_=`#ifdef USE_CLEARCOAT_NORMALMAP
	vec3 clearcoatMapN = texture2D( clearcoatNormalMap, vClearcoatNormalMapUv ).xyz * 2.0 - 1.0;
	clearcoatMapN.xy *= clearcoatNormalScale;
	clearcoatNormal = normalize( tbn2 * clearcoatMapN );
#endif`,B_=`#ifdef USE_CLEARCOATMAP
	uniform sampler2D clearcoatMap;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform sampler2D clearcoatNormalMap;
	uniform vec2 clearcoatNormalScale;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform sampler2D clearcoatRoughnessMap;
#endif`,z_=`#ifdef USE_IRIDESCENCEMAP
	uniform sampler2D iridescenceMap;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform sampler2D iridescenceThicknessMap;
#endif`,V_=`#ifdef OPAQUE
diffuseColor.a = 1.0;
#endif
#ifdef USE_TRANSMISSION
diffuseColor.a *= material.transmissionAlpha;
#endif
gl_FragColor = vec4( outgoingLight, diffuseColor.a );`,H_=`vec3 packNormalToRGB( const in vec3 normal ) {
	return normalize( normal ) * 0.5 + 0.5;
}
vec3 unpackRGBToNormal( const in vec3 rgb ) {
	return 2.0 * rgb.xyz - 1.0;
}
const float PackUpscale = 256. / 255.;const float UnpackDownscale = 255. / 256.;const float ShiftRight8 = 1. / 256.;
const float Inv255 = 1. / 255.;
const vec4 PackFactors = vec4( 1.0, 256.0, 256.0 * 256.0, 256.0 * 256.0 * 256.0 );
const vec2 UnpackFactors2 = vec2( UnpackDownscale, 1.0 / PackFactors.g );
const vec3 UnpackFactors3 = vec3( UnpackDownscale / PackFactors.rg, 1.0 / PackFactors.b );
const vec4 UnpackFactors4 = vec4( UnpackDownscale / PackFactors.rgb, 1.0 / PackFactors.a );
vec4 packDepthToRGBA( const in float v ) {
	if( v <= 0.0 )
		return vec4( 0., 0., 0., 0. );
	if( v >= 1.0 )
		return vec4( 1., 1., 1., 1. );
	float vuf;
	float af = modf( v * PackFactors.a, vuf );
	float bf = modf( vuf * ShiftRight8, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec4( vuf * Inv255, gf * PackUpscale, bf * PackUpscale, af );
}
vec3 packDepthToRGB( const in float v ) {
	if( v <= 0.0 )
		return vec3( 0., 0., 0. );
	if( v >= 1.0 )
		return vec3( 1., 1., 1. );
	float vuf;
	float bf = modf( v * PackFactors.b, vuf );
	float gf = modf( vuf * ShiftRight8, vuf );
	return vec3( vuf * Inv255, gf * PackUpscale, bf );
}
vec2 packDepthToRG( const in float v ) {
	if( v <= 0.0 )
		return vec2( 0., 0. );
	if( v >= 1.0 )
		return vec2( 1., 1. );
	float vuf;
	float gf = modf( v * 256., vuf );
	return vec2( vuf * Inv255, gf );
}
float unpackRGBAToDepth( const in vec4 v ) {
	return dot( v, UnpackFactors4 );
}
float unpackRGBToDepth( const in vec3 v ) {
	return dot( v, UnpackFactors3 );
}
float unpackRGToDepth( const in vec2 v ) {
	return v.r * UnpackFactors2.r + v.g * UnpackFactors2.g;
}
vec4 pack2HalfToRGBA( const in vec2 v ) {
	vec4 r = vec4( v.x, fract( v.x * 255.0 ), v.y, fract( v.y * 255.0 ) );
	return vec4( r.x - r.y / 255.0, r.y, r.z - r.w / 255.0, r.w );
}
vec2 unpackRGBATo2Half( const in vec4 v ) {
	return vec2( v.x + ( v.y / 255.0 ), v.z + ( v.w / 255.0 ) );
}
float viewZToOrthographicDepth( const in float viewZ, const in float near, const in float far ) {
	return ( viewZ + near ) / ( near - far );
}
float orthographicDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return depth * ( near - far ) - near;
}
float viewZToPerspectiveDepth( const in float viewZ, const in float near, const in float far ) {
	return ( ( near + viewZ ) * far ) / ( ( far - near ) * viewZ );
}
float perspectiveDepthToViewZ( const in float depth, const in float near, const in float far ) {
	return ( near * far ) / ( ( far - near ) * depth - far );
}`,G_=`#ifdef PREMULTIPLIED_ALPHA
	gl_FragColor.rgb *= gl_FragColor.a;
#endif`,W_=`vec4 mvPosition = vec4( transformed, 1.0 );
#ifdef USE_BATCHING
	mvPosition = batchingMatrix * mvPosition;
#endif
#ifdef USE_INSTANCING
	mvPosition = instanceMatrix * mvPosition;
#endif
mvPosition = modelViewMatrix * mvPosition;
gl_Position = projectionMatrix * mvPosition;`,$_=`#ifdef DITHERING
	gl_FragColor.rgb = dithering( gl_FragColor.rgb );
#endif`,j_=`#ifdef DITHERING
	vec3 dithering( vec3 color ) {
		float grid_position = rand( gl_FragCoord.xy );
		vec3 dither_shift_RGB = vec3( 0.25 / 255.0, -0.25 / 255.0, 0.25 / 255.0 );
		dither_shift_RGB = mix( 2.0 * dither_shift_RGB, -2.0 * dither_shift_RGB, grid_position );
		return color + dither_shift_RGB;
	}
#endif`,X_=`float roughnessFactor = roughness;
#ifdef USE_ROUGHNESSMAP
	vec4 texelRoughness = texture2D( roughnessMap, vRoughnessMapUv );
	roughnessFactor *= texelRoughness.g;
#endif`,q_=`#ifdef USE_ROUGHNESSMAP
	uniform sampler2D roughnessMap;
#endif`,Y_=`#if NUM_SPOT_LIGHT_COORDS > 0
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#if NUM_SPOT_LIGHT_MAPS > 0
	uniform sampler2D spotLightMap[ NUM_SPOT_LIGHT_MAPS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform sampler2D directionalShadowMap[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		uniform sampler2D spotShadowMap[ NUM_SPOT_LIGHT_SHADOWS ];
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform sampler2D pointShadowMap[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
	float texture2DCompare( sampler2D depths, vec2 uv, float compare ) {
		return step( compare, unpackRGBAToDepth( texture2D( depths, uv ) ) );
	}
	vec2 texture2DDistribution( sampler2D shadow, vec2 uv ) {
		return unpackRGBATo2Half( texture2D( shadow, uv ) );
	}
	float VSMShadow (sampler2D shadow, vec2 uv, float compare ){
		float occlusion = 1.0;
		vec2 distribution = texture2DDistribution( shadow, uv );
		float hard_shadow = step( compare , distribution.x );
		if (hard_shadow != 1.0 ) {
			float distance = compare - distribution.x ;
			float variance = max( 0.00000, distribution.y * distribution.y );
			float softness_probability = variance / (variance + distance * distance );			softness_probability = clamp( ( softness_probability - 0.3 ) / ( 0.95 - 0.3 ), 0.0, 1.0 );			occlusion = clamp( max( hard_shadow, softness_probability ), 0.0, 1.0 );
		}
		return occlusion;
	}
	float getShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord ) {
		float shadow = 1.0;
		shadowCoord.xyz /= shadowCoord.w;
		shadowCoord.z += shadowBias;
		bool inFrustum = shadowCoord.x >= 0.0 && shadowCoord.x <= 1.0 && shadowCoord.y >= 0.0 && shadowCoord.y <= 1.0;
		bool frustumTest = inFrustum && shadowCoord.z <= 1.0;
		if ( frustumTest ) {
		#if defined( SHADOWMAP_TYPE_PCF )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx0 = - texelSize.x * shadowRadius;
			float dy0 = - texelSize.y * shadowRadius;
			float dx1 = + texelSize.x * shadowRadius;
			float dy1 = + texelSize.y * shadowRadius;
			float dx2 = dx0 / 2.0;
			float dy2 = dy0 / 2.0;
			float dx3 = dx1 / 2.0;
			float dy3 = dy1 / 2.0;
			shadow = (
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy2 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx2, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx3, dy3 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( 0.0, dy1 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, shadowCoord.xy + vec2( dx1, dy1 ), shadowCoord.z )
			) * ( 1.0 / 17.0 );
		#elif defined( SHADOWMAP_TYPE_PCF_SOFT )
			vec2 texelSize = vec2( 1.0 ) / shadowMapSize;
			float dx = texelSize.x;
			float dy = texelSize.y;
			vec2 uv = shadowCoord.xy;
			vec2 f = fract( uv * shadowMapSize + 0.5 );
			uv -= f * texelSize;
			shadow = (
				texture2DCompare( shadowMap, uv, shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( dx, 0.0 ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + vec2( 0.0, dy ), shadowCoord.z ) +
				texture2DCompare( shadowMap, uv + texelSize, shadowCoord.z ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, 0.0 ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 0.0 ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( -dx, dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, dy ), shadowCoord.z ),
					 f.x ) +
				mix( texture2DCompare( shadowMap, uv + vec2( 0.0, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( 0.0, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( texture2DCompare( shadowMap, uv + vec2( dx, -dy ), shadowCoord.z ),
					 texture2DCompare( shadowMap, uv + vec2( dx, 2.0 * dy ), shadowCoord.z ),
					 f.y ) +
				mix( mix( texture2DCompare( shadowMap, uv + vec2( -dx, -dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, -dy ), shadowCoord.z ),
						  f.x ),
					 mix( texture2DCompare( shadowMap, uv + vec2( -dx, 2.0 * dy ), shadowCoord.z ),
						  texture2DCompare( shadowMap, uv + vec2( 2.0 * dx, 2.0 * dy ), shadowCoord.z ),
						  f.x ),
					 f.y )
			) * ( 1.0 / 9.0 );
		#elif defined( SHADOWMAP_TYPE_VSM )
			shadow = VSMShadow( shadowMap, shadowCoord.xy, shadowCoord.z );
		#else
			shadow = texture2DCompare( shadowMap, shadowCoord.xy, shadowCoord.z );
		#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
	vec2 cubeToUV( vec3 v, float texelSizeY ) {
		vec3 absV = abs( v );
		float scaleToCube = 1.0 / max( absV.x, max( absV.y, absV.z ) );
		absV *= scaleToCube;
		v *= scaleToCube * ( 1.0 - 2.0 * texelSizeY );
		vec2 planar = v.xy;
		float almostATexel = 1.5 * texelSizeY;
		float almostOne = 1.0 - almostATexel;
		if ( absV.z >= almostOne ) {
			if ( v.z > 0.0 )
				planar.x = 4.0 - v.x;
		} else if ( absV.x >= almostOne ) {
			float signX = sign( v.x );
			planar.x = v.z * signX + 2.0 * signX;
		} else if ( absV.y >= almostOne ) {
			float signY = sign( v.y );
			planar.x = v.x + 2.0 * signY + 2.0;
			planar.y = v.z * signY - 2.0;
		}
		return vec2( 0.125, 0.25 ) * planar + vec2( 0.375, 0.75 );
	}
	float getPointShadow( sampler2D shadowMap, vec2 shadowMapSize, float shadowIntensity, float shadowBias, float shadowRadius, vec4 shadowCoord, float shadowCameraNear, float shadowCameraFar ) {
		float shadow = 1.0;
		vec3 lightToPosition = shadowCoord.xyz;
		
		float lightToPositionLength = length( lightToPosition );
		if ( lightToPositionLength - shadowCameraFar <= 0.0 && lightToPositionLength - shadowCameraNear >= 0.0 ) {
			float dp = ( lightToPositionLength - shadowCameraNear ) / ( shadowCameraFar - shadowCameraNear );			dp += shadowBias;
			vec3 bd3D = normalize( lightToPosition );
			vec2 texelSize = vec2( 1.0 ) / ( shadowMapSize * vec2( 4.0, 2.0 ) );
			#if defined( SHADOWMAP_TYPE_PCF ) || defined( SHADOWMAP_TYPE_PCF_SOFT ) || defined( SHADOWMAP_TYPE_VSM )
				vec2 offset = vec2( - 1, 1 ) * shadowRadius * texelSize.y;
				shadow = (
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yyx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxy, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.xxx, texelSize.y ), dp ) +
					texture2DCompare( shadowMap, cubeToUV( bd3D + offset.yxx, texelSize.y ), dp )
				) * ( 1.0 / 9.0 );
			#else
				shadow = texture2DCompare( shadowMap, cubeToUV( bd3D, texelSize.y ), dp );
			#endif
		}
		return mix( 1.0, shadow, shadowIntensity );
	}
#endif`,K_=`#if NUM_SPOT_LIGHT_COORDS > 0
	uniform mat4 spotLightMatrix[ NUM_SPOT_LIGHT_COORDS ];
	varying vec4 vSpotLightCoord[ NUM_SPOT_LIGHT_COORDS ];
#endif
#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
		uniform mat4 directionalShadowMatrix[ NUM_DIR_LIGHT_SHADOWS ];
		varying vec4 vDirectionalShadowCoord[ NUM_DIR_LIGHT_SHADOWS ];
		struct DirectionalLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform DirectionalLightShadow directionalLightShadows[ NUM_DIR_LIGHT_SHADOWS ];
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
		struct SpotLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
		};
		uniform SpotLightShadow spotLightShadows[ NUM_SPOT_LIGHT_SHADOWS ];
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		uniform mat4 pointShadowMatrix[ NUM_POINT_LIGHT_SHADOWS ];
		varying vec4 vPointShadowCoord[ NUM_POINT_LIGHT_SHADOWS ];
		struct PointLightShadow {
			float shadowIntensity;
			float shadowBias;
			float shadowNormalBias;
			float shadowRadius;
			vec2 shadowMapSize;
			float shadowCameraNear;
			float shadowCameraFar;
		};
		uniform PointLightShadow pointLightShadows[ NUM_POINT_LIGHT_SHADOWS ];
	#endif
#endif`,J_=`#if ( defined( USE_SHADOWMAP ) && ( NUM_DIR_LIGHT_SHADOWS > 0 || NUM_POINT_LIGHT_SHADOWS > 0 ) ) || ( NUM_SPOT_LIGHT_COORDS > 0 )
	vec3 shadowWorldNormal = inverseTransformDirection( transformedNormal, viewMatrix );
	vec4 shadowWorldPosition;
#endif
#if defined( USE_SHADOWMAP )
	#if NUM_DIR_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * directionalLightShadows[ i ].shadowNormalBias, 0 );
			vDirectionalShadowCoord[ i ] = directionalShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
		#pragma unroll_loop_start
		for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
			shadowWorldPosition = worldPosition + vec4( shadowWorldNormal * pointLightShadows[ i ].shadowNormalBias, 0 );
			vPointShadowCoord[ i ] = pointShadowMatrix[ i ] * shadowWorldPosition;
		}
		#pragma unroll_loop_end
	#endif
#endif
#if NUM_SPOT_LIGHT_COORDS > 0
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_COORDS; i ++ ) {
		shadowWorldPosition = worldPosition;
		#if ( defined( USE_SHADOWMAP ) && UNROLLED_LOOP_INDEX < NUM_SPOT_LIGHT_SHADOWS )
			shadowWorldPosition.xyz += shadowWorldNormal * spotLightShadows[ i ].shadowNormalBias;
		#endif
		vSpotLightCoord[ i ] = spotLightMatrix[ i ] * shadowWorldPosition;
	}
	#pragma unroll_loop_end
#endif`,Z_=`float getShadowMask() {
	float shadow = 1.0;
	#ifdef USE_SHADOWMAP
	#if NUM_DIR_LIGHT_SHADOWS > 0
	DirectionalLightShadow directionalLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_DIR_LIGHT_SHADOWS; i ++ ) {
		directionalLight = directionalLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( directionalShadowMap[ i ], directionalLight.shadowMapSize, directionalLight.shadowIntensity, directionalLight.shadowBias, directionalLight.shadowRadius, vDirectionalShadowCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_SPOT_LIGHT_SHADOWS > 0
	SpotLightShadow spotLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_SPOT_LIGHT_SHADOWS; i ++ ) {
		spotLight = spotLightShadows[ i ];
		shadow *= receiveShadow ? getShadow( spotShadowMap[ i ], spotLight.shadowMapSize, spotLight.shadowIntensity, spotLight.shadowBias, spotLight.shadowRadius, vSpotLightCoord[ i ] ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#if NUM_POINT_LIGHT_SHADOWS > 0
	PointLightShadow pointLight;
	#pragma unroll_loop_start
	for ( int i = 0; i < NUM_POINT_LIGHT_SHADOWS; i ++ ) {
		pointLight = pointLightShadows[ i ];
		shadow *= receiveShadow ? getPointShadow( pointShadowMap[ i ], pointLight.shadowMapSize, pointLight.shadowIntensity, pointLight.shadowBias, pointLight.shadowRadius, vPointShadowCoord[ i ], pointLight.shadowCameraNear, pointLight.shadowCameraFar ) : 1.0;
	}
	#pragma unroll_loop_end
	#endif
	#endif
	return shadow;
}`,Q_=`#ifdef USE_SKINNING
	mat4 boneMatX = getBoneMatrix( skinIndex.x );
	mat4 boneMatY = getBoneMatrix( skinIndex.y );
	mat4 boneMatZ = getBoneMatrix( skinIndex.z );
	mat4 boneMatW = getBoneMatrix( skinIndex.w );
#endif`,t0=`#ifdef USE_SKINNING
	uniform mat4 bindMatrix;
	uniform mat4 bindMatrixInverse;
	uniform highp sampler2D boneTexture;
	mat4 getBoneMatrix( const in float i ) {
		int size = textureSize( boneTexture, 0 ).x;
		int j = int( i ) * 4;
		int x = j % size;
		int y = j / size;
		vec4 v1 = texelFetch( boneTexture, ivec2( x, y ), 0 );
		vec4 v2 = texelFetch( boneTexture, ivec2( x + 1, y ), 0 );
		vec4 v3 = texelFetch( boneTexture, ivec2( x + 2, y ), 0 );
		vec4 v4 = texelFetch( boneTexture, ivec2( x + 3, y ), 0 );
		return mat4( v1, v2, v3, v4 );
	}
#endif`,e0=`#ifdef USE_SKINNING
	vec4 skinVertex = bindMatrix * vec4( transformed, 1.0 );
	vec4 skinned = vec4( 0.0 );
	skinned += boneMatX * skinVertex * skinWeight.x;
	skinned += boneMatY * skinVertex * skinWeight.y;
	skinned += boneMatZ * skinVertex * skinWeight.z;
	skinned += boneMatW * skinVertex * skinWeight.w;
	transformed = ( bindMatrixInverse * skinned ).xyz;
#endif`,n0=`#ifdef USE_SKINNING
	mat4 skinMatrix = mat4( 0.0 );
	skinMatrix += skinWeight.x * boneMatX;
	skinMatrix += skinWeight.y * boneMatY;
	skinMatrix += skinWeight.z * boneMatZ;
	skinMatrix += skinWeight.w * boneMatW;
	skinMatrix = bindMatrixInverse * skinMatrix * bindMatrix;
	objectNormal = vec4( skinMatrix * vec4( objectNormal, 0.0 ) ).xyz;
	#ifdef USE_TANGENT
		objectTangent = vec4( skinMatrix * vec4( objectTangent, 0.0 ) ).xyz;
	#endif
#endif`,i0=`float specularStrength;
#ifdef USE_SPECULARMAP
	vec4 texelSpecular = texture2D( specularMap, vSpecularMapUv );
	specularStrength = texelSpecular.r;
#else
	specularStrength = 1.0;
#endif`,s0=`#ifdef USE_SPECULARMAP
	uniform sampler2D specularMap;
#endif`,r0=`#if defined( TONE_MAPPING )
	gl_FragColor.rgb = toneMapping( gl_FragColor.rgb );
#endif`,a0=`#ifndef saturate
#define saturate( a ) clamp( a, 0.0, 1.0 )
#endif
uniform float toneMappingExposure;
vec3 LinearToneMapping( vec3 color ) {
	return saturate( toneMappingExposure * color );
}
vec3 ReinhardToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	return saturate( color / ( vec3( 1.0 ) + color ) );
}
vec3 CineonToneMapping( vec3 color ) {
	color *= toneMappingExposure;
	color = max( vec3( 0.0 ), color - 0.004 );
	return pow( ( color * ( 6.2 * color + 0.5 ) ) / ( color * ( 6.2 * color + 1.7 ) + 0.06 ), vec3( 2.2 ) );
}
vec3 RRTAndODTFit( vec3 v ) {
	vec3 a = v * ( v + 0.0245786 ) - 0.000090537;
	vec3 b = v * ( 0.983729 * v + 0.4329510 ) + 0.238081;
	return a / b;
}
vec3 ACESFilmicToneMapping( vec3 color ) {
	const mat3 ACESInputMat = mat3(
		vec3( 0.59719, 0.07600, 0.02840 ),		vec3( 0.35458, 0.90834, 0.13383 ),
		vec3( 0.04823, 0.01566, 0.83777 )
	);
	const mat3 ACESOutputMat = mat3(
		vec3(  1.60475, -0.10208, -0.00327 ),		vec3( -0.53108,  1.10813, -0.07276 ),
		vec3( -0.07367, -0.00605,  1.07602 )
	);
	color *= toneMappingExposure / 0.6;
	color = ACESInputMat * color;
	color = RRTAndODTFit( color );
	color = ACESOutputMat * color;
	return saturate( color );
}
const mat3 LINEAR_REC2020_TO_LINEAR_SRGB = mat3(
	vec3( 1.6605, - 0.1246, - 0.0182 ),
	vec3( - 0.5876, 1.1329, - 0.1006 ),
	vec3( - 0.0728, - 0.0083, 1.1187 )
);
const mat3 LINEAR_SRGB_TO_LINEAR_REC2020 = mat3(
	vec3( 0.6274, 0.0691, 0.0164 ),
	vec3( 0.3293, 0.9195, 0.0880 ),
	vec3( 0.0433, 0.0113, 0.8956 )
);
vec3 agxDefaultContrastApprox( vec3 x ) {
	vec3 x2 = x * x;
	vec3 x4 = x2 * x2;
	return + 15.5 * x4 * x2
		- 40.14 * x4 * x
		+ 31.96 * x4
		- 6.868 * x2 * x
		+ 0.4298 * x2
		+ 0.1191 * x
		- 0.00232;
}
vec3 AgXToneMapping( vec3 color ) {
	const mat3 AgXInsetMatrix = mat3(
		vec3( 0.856627153315983, 0.137318972929847, 0.11189821299995 ),
		vec3( 0.0951212405381588, 0.761241990602591, 0.0767994186031903 ),
		vec3( 0.0482516061458583, 0.101439036467562, 0.811302368396859 )
	);
	const mat3 AgXOutsetMatrix = mat3(
		vec3( 1.1271005818144368, - 0.1413297634984383, - 0.14132976349843826 ),
		vec3( - 0.11060664309660323, 1.157823702216272, - 0.11060664309660294 ),
		vec3( - 0.016493938717834573, - 0.016493938717834257, 1.2519364065950405 )
	);
	const float AgxMinEv = - 12.47393;	const float AgxMaxEv = 4.026069;
	color *= toneMappingExposure;
	color = LINEAR_SRGB_TO_LINEAR_REC2020 * color;
	color = AgXInsetMatrix * color;
	color = max( color, 1e-10 );	color = log2( color );
	color = ( color - AgxMinEv ) / ( AgxMaxEv - AgxMinEv );
	color = clamp( color, 0.0, 1.0 );
	color = agxDefaultContrastApprox( color );
	color = AgXOutsetMatrix * color;
	color = pow( max( vec3( 0.0 ), color ), vec3( 2.2 ) );
	color = LINEAR_REC2020_TO_LINEAR_SRGB * color;
	color = clamp( color, 0.0, 1.0 );
	return color;
}
vec3 NeutralToneMapping( vec3 color ) {
	const float StartCompression = 0.8 - 0.04;
	const float Desaturation = 0.15;
	color *= toneMappingExposure;
	float x = min( color.r, min( color.g, color.b ) );
	float offset = x < 0.08 ? x - 6.25 * x * x : 0.04;
	color -= offset;
	float peak = max( color.r, max( color.g, color.b ) );
	if ( peak < StartCompression ) return color;
	float d = 1. - StartCompression;
	float newPeak = 1. - d * d / ( peak + d - StartCompression );
	color *= newPeak / peak;
	float g = 1. - 1. / ( Desaturation * ( peak - newPeak ) + 1. );
	return mix( color, vec3( newPeak ), g );
}
vec3 CustomToneMapping( vec3 color ) { return color; }`,o0=`#ifdef USE_TRANSMISSION
	material.transmission = transmission;
	material.transmissionAlpha = 1.0;
	material.thickness = thickness;
	material.attenuationDistance = attenuationDistance;
	material.attenuationColor = attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		material.transmission *= texture2D( transmissionMap, vTransmissionMapUv ).r;
	#endif
	#ifdef USE_THICKNESSMAP
		material.thickness *= texture2D( thicknessMap, vThicknessMapUv ).g;
	#endif
	vec3 pos = vWorldPosition;
	vec3 v = normalize( cameraPosition - pos );
	vec3 n = inverseTransformDirection( normal, viewMatrix );
	vec4 transmitted = getIBLVolumeRefraction(
		n, v, material.roughness, material.diffuseColor, material.specularColor, material.specularF90,
		pos, modelMatrix, viewMatrix, projectionMatrix, material.dispersion, material.ior, material.thickness,
		material.attenuationColor, material.attenuationDistance );
	material.transmissionAlpha = mix( material.transmissionAlpha, transmitted.a, material.transmission );
	totalDiffuse = mix( totalDiffuse, transmitted.rgb, material.transmission );
#endif`,l0=`#ifdef USE_TRANSMISSION
	uniform float transmission;
	uniform float thickness;
	uniform float attenuationDistance;
	uniform vec3 attenuationColor;
	#ifdef USE_TRANSMISSIONMAP
		uniform sampler2D transmissionMap;
	#endif
	#ifdef USE_THICKNESSMAP
		uniform sampler2D thicknessMap;
	#endif
	uniform vec2 transmissionSamplerSize;
	uniform sampler2D transmissionSamplerMap;
	uniform mat4 modelMatrix;
	uniform mat4 projectionMatrix;
	varying vec3 vWorldPosition;
	float w0( float a ) {
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - a + 3.0 ) - 3.0 ) + 1.0 );
	}
	float w1( float a ) {
		return ( 1.0 / 6.0 ) * ( a *  a * ( 3.0 * a - 6.0 ) + 4.0 );
	}
	float w2( float a ){
		return ( 1.0 / 6.0 ) * ( a * ( a * ( - 3.0 * a + 3.0 ) + 3.0 ) + 1.0 );
	}
	float w3( float a ) {
		return ( 1.0 / 6.0 ) * ( a * a * a );
	}
	float g0( float a ) {
		return w0( a ) + w1( a );
	}
	float g1( float a ) {
		return w2( a ) + w3( a );
	}
	float h0( float a ) {
		return - 1.0 + w1( a ) / ( w0( a ) + w1( a ) );
	}
	float h1( float a ) {
		return 1.0 + w3( a ) / ( w2( a ) + w3( a ) );
	}
	vec4 bicubic( sampler2D tex, vec2 uv, vec4 texelSize, float lod ) {
		uv = uv * texelSize.zw + 0.5;
		vec2 iuv = floor( uv );
		vec2 fuv = fract( uv );
		float g0x = g0( fuv.x );
		float g1x = g1( fuv.x );
		float h0x = h0( fuv.x );
		float h1x = h1( fuv.x );
		float h0y = h0( fuv.y );
		float h1y = h1( fuv.y );
		vec2 p0 = ( vec2( iuv.x + h0x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p1 = ( vec2( iuv.x + h1x, iuv.y + h0y ) - 0.5 ) * texelSize.xy;
		vec2 p2 = ( vec2( iuv.x + h0x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		vec2 p3 = ( vec2( iuv.x + h1x, iuv.y + h1y ) - 0.5 ) * texelSize.xy;
		return g0( fuv.y ) * ( g0x * textureLod( tex, p0, lod ) + g1x * textureLod( tex, p1, lod ) ) +
			g1( fuv.y ) * ( g0x * textureLod( tex, p2, lod ) + g1x * textureLod( tex, p3, lod ) );
	}
	vec4 textureBicubic( sampler2D sampler, vec2 uv, float lod ) {
		vec2 fLodSize = vec2( textureSize( sampler, int( lod ) ) );
		vec2 cLodSize = vec2( textureSize( sampler, int( lod + 1.0 ) ) );
		vec2 fLodSizeInv = 1.0 / fLodSize;
		vec2 cLodSizeInv = 1.0 / cLodSize;
		vec4 fSample = bicubic( sampler, uv, vec4( fLodSizeInv, fLodSize ), floor( lod ) );
		vec4 cSample = bicubic( sampler, uv, vec4( cLodSizeInv, cLodSize ), ceil( lod ) );
		return mix( fSample, cSample, fract( lod ) );
	}
	vec3 getVolumeTransmissionRay( const in vec3 n, const in vec3 v, const in float thickness, const in float ior, const in mat4 modelMatrix ) {
		vec3 refractionVector = refract( - v, normalize( n ), 1.0 / ior );
		vec3 modelScale;
		modelScale.x = length( vec3( modelMatrix[ 0 ].xyz ) );
		modelScale.y = length( vec3( modelMatrix[ 1 ].xyz ) );
		modelScale.z = length( vec3( modelMatrix[ 2 ].xyz ) );
		return normalize( refractionVector ) * thickness * modelScale;
	}
	float applyIorToRoughness( const in float roughness, const in float ior ) {
		return roughness * clamp( ior * 2.0 - 2.0, 0.0, 1.0 );
	}
	vec4 getTransmissionSample( const in vec2 fragCoord, const in float roughness, const in float ior ) {
		float lod = log2( transmissionSamplerSize.x ) * applyIorToRoughness( roughness, ior );
		return textureBicubic( transmissionSamplerMap, fragCoord.xy, lod );
	}
	vec3 volumeAttenuation( const in float transmissionDistance, const in vec3 attenuationColor, const in float attenuationDistance ) {
		if ( isinf( attenuationDistance ) ) {
			return vec3( 1.0 );
		} else {
			vec3 attenuationCoefficient = -log( attenuationColor ) / attenuationDistance;
			vec3 transmittance = exp( - attenuationCoefficient * transmissionDistance );			return transmittance;
		}
	}
	vec4 getIBLVolumeRefraction( const in vec3 n, const in vec3 v, const in float roughness, const in vec3 diffuseColor,
		const in vec3 specularColor, const in float specularF90, const in vec3 position, const in mat4 modelMatrix,
		const in mat4 viewMatrix, const in mat4 projMatrix, const in float dispersion, const in float ior, const in float thickness,
		const in vec3 attenuationColor, const in float attenuationDistance ) {
		vec4 transmittedLight;
		vec3 transmittance;
		#ifdef USE_DISPERSION
			float halfSpread = ( ior - 1.0 ) * 0.025 * dispersion;
			vec3 iors = vec3( ior - halfSpread, ior, ior + halfSpread );
			for ( int i = 0; i < 3; i ++ ) {
				vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, iors[ i ], modelMatrix );
				vec3 refractedRayExit = position + transmissionRay;
				vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
				vec2 refractionCoords = ndcPos.xy / ndcPos.w;
				refractionCoords += 1.0;
				refractionCoords /= 2.0;
				vec4 transmissionSample = getTransmissionSample( refractionCoords, roughness, iors[ i ] );
				transmittedLight[ i ] = transmissionSample[ i ];
				transmittedLight.a += transmissionSample.a;
				transmittance[ i ] = diffuseColor[ i ] * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance )[ i ];
			}
			transmittedLight.a /= 3.0;
		#else
			vec3 transmissionRay = getVolumeTransmissionRay( n, v, thickness, ior, modelMatrix );
			vec3 refractedRayExit = position + transmissionRay;
			vec4 ndcPos = projMatrix * viewMatrix * vec4( refractedRayExit, 1.0 );
			vec2 refractionCoords = ndcPos.xy / ndcPos.w;
			refractionCoords += 1.0;
			refractionCoords /= 2.0;
			transmittedLight = getTransmissionSample( refractionCoords, roughness, ior );
			transmittance = diffuseColor * volumeAttenuation( length( transmissionRay ), attenuationColor, attenuationDistance );
		#endif
		vec3 attenuatedColor = transmittance * transmittedLight.rgb;
		vec3 F = EnvironmentBRDF( n, v, specularColor, specularF90, roughness );
		float transmittanceFactor = ( transmittance.r + transmittance.g + transmittance.b ) / 3.0;
		return vec4( ( 1.0 - F ) * attenuatedColor, 1.0 - ( 1.0 - transmittedLight.a ) * transmittanceFactor );
	}
#endif`,c0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_SPECULARMAP
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,u0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	varying vec2 vUv;
#endif
#ifdef USE_MAP
	uniform mat3 mapTransform;
	varying vec2 vMapUv;
#endif
#ifdef USE_ALPHAMAP
	uniform mat3 alphaMapTransform;
	varying vec2 vAlphaMapUv;
#endif
#ifdef USE_LIGHTMAP
	uniform mat3 lightMapTransform;
	varying vec2 vLightMapUv;
#endif
#ifdef USE_AOMAP
	uniform mat3 aoMapTransform;
	varying vec2 vAoMapUv;
#endif
#ifdef USE_BUMPMAP
	uniform mat3 bumpMapTransform;
	varying vec2 vBumpMapUv;
#endif
#ifdef USE_NORMALMAP
	uniform mat3 normalMapTransform;
	varying vec2 vNormalMapUv;
#endif
#ifdef USE_DISPLACEMENTMAP
	uniform mat3 displacementMapTransform;
	varying vec2 vDisplacementMapUv;
#endif
#ifdef USE_EMISSIVEMAP
	uniform mat3 emissiveMapTransform;
	varying vec2 vEmissiveMapUv;
#endif
#ifdef USE_METALNESSMAP
	uniform mat3 metalnessMapTransform;
	varying vec2 vMetalnessMapUv;
#endif
#ifdef USE_ROUGHNESSMAP
	uniform mat3 roughnessMapTransform;
	varying vec2 vRoughnessMapUv;
#endif
#ifdef USE_ANISOTROPYMAP
	uniform mat3 anisotropyMapTransform;
	varying vec2 vAnisotropyMapUv;
#endif
#ifdef USE_CLEARCOATMAP
	uniform mat3 clearcoatMapTransform;
	varying vec2 vClearcoatMapUv;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	uniform mat3 clearcoatNormalMapTransform;
	varying vec2 vClearcoatNormalMapUv;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	uniform mat3 clearcoatRoughnessMapTransform;
	varying vec2 vClearcoatRoughnessMapUv;
#endif
#ifdef USE_SHEEN_COLORMAP
	uniform mat3 sheenColorMapTransform;
	varying vec2 vSheenColorMapUv;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	uniform mat3 sheenRoughnessMapTransform;
	varying vec2 vSheenRoughnessMapUv;
#endif
#ifdef USE_IRIDESCENCEMAP
	uniform mat3 iridescenceMapTransform;
	varying vec2 vIridescenceMapUv;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	uniform mat3 iridescenceThicknessMapTransform;
	varying vec2 vIridescenceThicknessMapUv;
#endif
#ifdef USE_SPECULARMAP
	uniform mat3 specularMapTransform;
	varying vec2 vSpecularMapUv;
#endif
#ifdef USE_SPECULAR_COLORMAP
	uniform mat3 specularColorMapTransform;
	varying vec2 vSpecularColorMapUv;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	uniform mat3 specularIntensityMapTransform;
	varying vec2 vSpecularIntensityMapUv;
#endif
#ifdef USE_TRANSMISSIONMAP
	uniform mat3 transmissionMapTransform;
	varying vec2 vTransmissionMapUv;
#endif
#ifdef USE_THICKNESSMAP
	uniform mat3 thicknessMapTransform;
	varying vec2 vThicknessMapUv;
#endif`,h0=`#if defined( USE_UV ) || defined( USE_ANISOTROPY )
	vUv = vec3( uv, 1 ).xy;
#endif
#ifdef USE_MAP
	vMapUv = ( mapTransform * vec3( MAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ALPHAMAP
	vAlphaMapUv = ( alphaMapTransform * vec3( ALPHAMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_LIGHTMAP
	vLightMapUv = ( lightMapTransform * vec3( LIGHTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_AOMAP
	vAoMapUv = ( aoMapTransform * vec3( AOMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_BUMPMAP
	vBumpMapUv = ( bumpMapTransform * vec3( BUMPMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_NORMALMAP
	vNormalMapUv = ( normalMapTransform * vec3( NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_DISPLACEMENTMAP
	vDisplacementMapUv = ( displacementMapTransform * vec3( DISPLACEMENTMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_EMISSIVEMAP
	vEmissiveMapUv = ( emissiveMapTransform * vec3( EMISSIVEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_METALNESSMAP
	vMetalnessMapUv = ( metalnessMapTransform * vec3( METALNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ROUGHNESSMAP
	vRoughnessMapUv = ( roughnessMapTransform * vec3( ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_ANISOTROPYMAP
	vAnisotropyMapUv = ( anisotropyMapTransform * vec3( ANISOTROPYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOATMAP
	vClearcoatMapUv = ( clearcoatMapTransform * vec3( CLEARCOATMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_NORMALMAP
	vClearcoatNormalMapUv = ( clearcoatNormalMapTransform * vec3( CLEARCOAT_NORMALMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_CLEARCOAT_ROUGHNESSMAP
	vClearcoatRoughnessMapUv = ( clearcoatRoughnessMapTransform * vec3( CLEARCOAT_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCEMAP
	vIridescenceMapUv = ( iridescenceMapTransform * vec3( IRIDESCENCEMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_IRIDESCENCE_THICKNESSMAP
	vIridescenceThicknessMapUv = ( iridescenceThicknessMapTransform * vec3( IRIDESCENCE_THICKNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_COLORMAP
	vSheenColorMapUv = ( sheenColorMapTransform * vec3( SHEEN_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SHEEN_ROUGHNESSMAP
	vSheenRoughnessMapUv = ( sheenRoughnessMapTransform * vec3( SHEEN_ROUGHNESSMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULARMAP
	vSpecularMapUv = ( specularMapTransform * vec3( SPECULARMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_COLORMAP
	vSpecularColorMapUv = ( specularColorMapTransform * vec3( SPECULAR_COLORMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_SPECULAR_INTENSITYMAP
	vSpecularIntensityMapUv = ( specularIntensityMapTransform * vec3( SPECULAR_INTENSITYMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_TRANSMISSIONMAP
	vTransmissionMapUv = ( transmissionMapTransform * vec3( TRANSMISSIONMAP_UV, 1 ) ).xy;
#endif
#ifdef USE_THICKNESSMAP
	vThicknessMapUv = ( thicknessMapTransform * vec3( THICKNESSMAP_UV, 1 ) ).xy;
#endif`,d0=`#if defined( USE_ENVMAP ) || defined( DISTANCE ) || defined ( USE_SHADOWMAP ) || defined ( USE_TRANSMISSION ) || NUM_SPOT_LIGHT_COORDS > 0
	vec4 worldPosition = vec4( transformed, 1.0 );
	#ifdef USE_BATCHING
		worldPosition = batchingMatrix * worldPosition;
	#endif
	#ifdef USE_INSTANCING
		worldPosition = instanceMatrix * worldPosition;
	#endif
	worldPosition = modelMatrix * worldPosition;
#endif`;const f0=`varying vec2 vUv;
uniform mat3 uvTransform;
void main() {
	vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	gl_Position = vec4( position.xy, 1.0, 1.0 );
}`,p0=`uniform sampler2D t2D;
uniform float backgroundIntensity;
varying vec2 vUv;
void main() {
	vec4 texColor = texture2D( t2D, vUv );
	#ifdef DECODE_VIDEO_TEXTURE
		texColor = vec4( mix( pow( texColor.rgb * 0.9478672986 + vec3( 0.0521327014 ), vec3( 2.4 ) ), texColor.rgb * 0.0773993808, vec3( lessThanEqual( texColor.rgb, vec3( 0.04045 ) ) ) ), texColor.w );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,m0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,g0=`#ifdef ENVMAP_TYPE_CUBE
	uniform samplerCube envMap;
#elif defined( ENVMAP_TYPE_CUBE_UV )
	uniform sampler2D envMap;
#endif
uniform float flipEnvMap;
uniform float backgroundBlurriness;
uniform float backgroundIntensity;
uniform mat3 backgroundRotation;
varying vec3 vWorldDirection;
#include <cube_uv_reflection_fragment>
void main() {
	#ifdef ENVMAP_TYPE_CUBE
		vec4 texColor = textureCube( envMap, backgroundRotation * vec3( flipEnvMap * vWorldDirection.x, vWorldDirection.yz ) );
	#elif defined( ENVMAP_TYPE_CUBE_UV )
		vec4 texColor = textureCubeUV( envMap, backgroundRotation * vWorldDirection, backgroundBlurriness );
	#else
		vec4 texColor = vec4( 0.0, 0.0, 0.0, 1.0 );
	#endif
	texColor.rgb *= backgroundIntensity;
	gl_FragColor = texColor;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,_0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
	gl_Position.z = gl_Position.w;
}`,v0=`uniform samplerCube tCube;
uniform float tFlip;
uniform float opacity;
varying vec3 vWorldDirection;
void main() {
	vec4 texColor = textureCube( tCube, vec3( tFlip * vWorldDirection.x, vWorldDirection.yz ) );
	gl_FragColor = texColor;
	gl_FragColor.a *= opacity;
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,x0=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
varying vec2 vHighPrecisionZW;
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vHighPrecisionZW = gl_Position.zw;
}`,b0=`#if DEPTH_PACKING == 3200
	uniform float opacity;
#endif
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
varying vec2 vHighPrecisionZW;
void main() {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#if DEPTH_PACKING == 3200
		diffuseColor.a = opacity;
	#endif
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <logdepthbuf_fragment>
	float fragCoordZ = 0.5 * vHighPrecisionZW[0] / vHighPrecisionZW[1] + 0.5;
	#if DEPTH_PACKING == 3200
		gl_FragColor = vec4( vec3( 1.0 - fragCoordZ ), opacity );
	#elif DEPTH_PACKING == 3201
		gl_FragColor = packDepthToRGBA( fragCoordZ );
	#elif DEPTH_PACKING == 3202
		gl_FragColor = vec4( packDepthToRGB( fragCoordZ ), 1.0 );
	#elif DEPTH_PACKING == 3203
		gl_FragColor = vec4( packDepthToRG( fragCoordZ ), 0.0, 1.0 );
	#endif
}`,y0=`#define DISTANCE
varying vec3 vWorldPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <skinbase_vertex>
	#include <morphinstance_vertex>
	#ifdef USE_DISPLACEMENTMAP
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <worldpos_vertex>
	#include <clipping_planes_vertex>
	vWorldPosition = worldPosition.xyz;
}`,M0=`#define DISTANCE
uniform vec3 referencePosition;
uniform float nearDistance;
uniform float farDistance;
varying vec3 vWorldPosition;
#include <common>
#include <packing>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <clipping_planes_pars_fragment>
void main () {
	vec4 diffuseColor = vec4( 1.0 );
	#include <clipping_planes_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	float dist = length( vWorldPosition - referencePosition );
	dist = ( dist - nearDistance ) / ( farDistance - nearDistance );
	dist = saturate( dist );
	gl_FragColor = packDepthToRGBA( dist );
}`,S0=`varying vec3 vWorldDirection;
#include <common>
void main() {
	vWorldDirection = transformDirection( position, modelMatrix );
	#include <begin_vertex>
	#include <project_vertex>
}`,E0=`uniform sampler2D tEquirect;
varying vec3 vWorldDirection;
#include <common>
void main() {
	vec3 direction = normalize( vWorldDirection );
	vec2 sampleUV = equirectUv( direction );
	gl_FragColor = texture2D( tEquirect, sampleUV );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
}`,T0=`uniform float scale;
attribute float lineDistance;
varying float vLineDistance;
#include <common>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	vLineDistance = scale * lineDistance;
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,w0=`uniform vec3 diffuse;
uniform float opacity;
uniform float dashSize;
uniform float totalSize;
varying float vLineDistance;
#include <common>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	if ( mod( vLineDistance, totalSize ) > dashSize ) {
		discard;
	}
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,A0=`#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#if defined ( USE_ENVMAP ) || defined ( USE_SKINNING )
		#include <beginnormal_vertex>
		#include <morphnormal_vertex>
		#include <skinbase_vertex>
		#include <skinnormal_vertex>
		#include <defaultnormal_vertex>
	#endif
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <fog_vertex>
}`,C0=`uniform vec3 diffuse;
uniform float opacity;
#ifndef FLAT_SHADED
	varying vec3 vNormal;
#endif
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	#ifdef USE_LIGHTMAP
		vec4 lightMapTexel = texture2D( lightMap, vLightMapUv );
		reflectedLight.indirectDiffuse += lightMapTexel.rgb * lightMapIntensity * RECIPROCAL_PI;
	#else
		reflectedLight.indirectDiffuse += vec3( 1.0 );
	#endif
	#include <aomap_fragment>
	reflectedLight.indirectDiffuse *= diffuseColor.rgb;
	vec3 outgoingLight = reflectedLight.indirectDiffuse;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,R0=`#define LAMBERT
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,P0=`#define LAMBERT
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_lambert_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_lambert_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,L0=`#define MATCAP
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <color_pars_vertex>
#include <displacementmap_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
	vViewPosition = - mvPosition.xyz;
}`,N0=`#define MATCAP
uniform vec3 diffuse;
uniform float opacity;
uniform sampler2D matcap;
varying vec3 vViewPosition;
#include <common>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	vec3 viewDir = normalize( vViewPosition );
	vec3 x = normalize( vec3( viewDir.z, 0.0, - viewDir.x ) );
	vec3 y = cross( viewDir, x );
	vec2 uv = vec2( dot( x, normal ), dot( y, normal ) ) * 0.495 + 0.5;
	#ifdef USE_MATCAP
		vec4 matcapColor = texture2D( matcap, uv );
	#else
		vec4 matcapColor = vec4( vec3( mix( 0.2, 0.8, uv.y ) ), 1.0 );
	#endif
	vec3 outgoingLight = diffuseColor.rgb * matcapColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,D0=`#define NORMAL
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	vViewPosition = - mvPosition.xyz;
#endif
}`,I0=`#define NORMAL
uniform float opacity;
#if defined( FLAT_SHADED ) || defined( USE_BUMPMAP ) || defined( USE_NORMALMAP_TANGENTSPACE )
	varying vec3 vViewPosition;
#endif
#include <packing>
#include <uv_pars_fragment>
#include <normal_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( 0.0, 0.0, 0.0, opacity );
	#include <clipping_planes_fragment>
	#include <logdepthbuf_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	gl_FragColor = vec4( packNormalToRGB( normal ), diffuseColor.a );
	#ifdef OPAQUE
		gl_FragColor.a = 1.0;
	#endif
}`,U0=`#define PHONG
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <envmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <envmap_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,k0=`#define PHONG
uniform vec3 diffuse;
uniform vec3 emissive;
uniform vec3 specular;
uniform float shininess;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_phong_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <specularmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <specularmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_phong_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + reflectedLight.directSpecular + reflectedLight.indirectSpecular + totalEmissiveRadiance;
	#include <envmap_fragment>
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,F0=`#define STANDARD
varying vec3 vViewPosition;
#ifdef USE_TRANSMISSION
	varying vec3 vWorldPosition;
#endif
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
#ifdef USE_TRANSMISSION
	vWorldPosition = worldPosition.xyz;
#endif
}`,O0=`#define STANDARD
#ifdef PHYSICAL
	#define IOR
	#define USE_SPECULAR
#endif
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float roughness;
uniform float metalness;
uniform float opacity;
#ifdef IOR
	uniform float ior;
#endif
#ifdef USE_SPECULAR
	uniform float specularIntensity;
	uniform vec3 specularColor;
	#ifdef USE_SPECULAR_COLORMAP
		uniform sampler2D specularColorMap;
	#endif
	#ifdef USE_SPECULAR_INTENSITYMAP
		uniform sampler2D specularIntensityMap;
	#endif
#endif
#ifdef USE_CLEARCOAT
	uniform float clearcoat;
	uniform float clearcoatRoughness;
#endif
#ifdef USE_DISPERSION
	uniform float dispersion;
#endif
#ifdef USE_IRIDESCENCE
	uniform float iridescence;
	uniform float iridescenceIOR;
	uniform float iridescenceThicknessMinimum;
	uniform float iridescenceThicknessMaximum;
#endif
#ifdef USE_SHEEN
	uniform vec3 sheenColor;
	uniform float sheenRoughness;
	#ifdef USE_SHEEN_COLORMAP
		uniform sampler2D sheenColorMap;
	#endif
	#ifdef USE_SHEEN_ROUGHNESSMAP
		uniform sampler2D sheenRoughnessMap;
	#endif
#endif
#ifdef USE_ANISOTROPY
	uniform vec2 anisotropyVector;
	#ifdef USE_ANISOTROPYMAP
		uniform sampler2D anisotropyMap;
	#endif
#endif
varying vec3 vViewPosition;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <iridescence_fragment>
#include <cube_uv_reflection_fragment>
#include <envmap_common_pars_fragment>
#include <envmap_physical_pars_fragment>
#include <fog_pars_fragment>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_physical_pars_fragment>
#include <transmission_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <clearcoat_pars_fragment>
#include <iridescence_pars_fragment>
#include <roughnessmap_pars_fragment>
#include <metalnessmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <roughnessmap_fragment>
	#include <metalnessmap_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <clearcoat_normal_fragment_begin>
	#include <clearcoat_normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_physical_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 totalDiffuse = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse;
	vec3 totalSpecular = reflectedLight.directSpecular + reflectedLight.indirectSpecular;
	#include <transmission_fragment>
	vec3 outgoingLight = totalDiffuse + totalSpecular + totalEmissiveRadiance;
	#ifdef USE_SHEEN
		float sheenEnergyComp = 1.0 - 0.157 * max3( material.sheenColor );
		outgoingLight = outgoingLight * sheenEnergyComp + sheenSpecularDirect + sheenSpecularIndirect;
	#endif
	#ifdef USE_CLEARCOAT
		float dotNVcc = saturate( dot( geometryClearcoatNormal, geometryViewDir ) );
		vec3 Fcc = F_Schlick( material.clearcoatF0, material.clearcoatF90, dotNVcc );
		outgoingLight = outgoingLight * ( 1.0 - material.clearcoat * Fcc ) + ( clearcoatSpecularDirect + clearcoatSpecularIndirect ) * material.clearcoat;
	#endif
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,B0=`#define TOON
varying vec3 vViewPosition;
#include <common>
#include <batching_pars_vertex>
#include <uv_pars_vertex>
#include <displacementmap_pars_vertex>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <normal_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <shadowmap_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <normal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <displacementmap_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	vViewPosition = - mvPosition.xyz;
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,z0=`#define TOON
uniform vec3 diffuse;
uniform vec3 emissive;
uniform float opacity;
#include <common>
#include <packing>
#include <dithering_pars_fragment>
#include <color_pars_fragment>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <aomap_pars_fragment>
#include <lightmap_pars_fragment>
#include <emissivemap_pars_fragment>
#include <gradientmap_pars_fragment>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <normal_pars_fragment>
#include <lights_toon_pars_fragment>
#include <shadowmap_pars_fragment>
#include <bumpmap_pars_fragment>
#include <normalmap_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	ReflectedLight reflectedLight = ReflectedLight( vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ), vec3( 0.0 ) );
	vec3 totalEmissiveRadiance = emissive;
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <color_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	#include <normal_fragment_begin>
	#include <normal_fragment_maps>
	#include <emissivemap_fragment>
	#include <lights_toon_fragment>
	#include <lights_fragment_begin>
	#include <lights_fragment_maps>
	#include <lights_fragment_end>
	#include <aomap_fragment>
	vec3 outgoingLight = reflectedLight.directDiffuse + reflectedLight.indirectDiffuse + totalEmissiveRadiance;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
	#include <dithering_fragment>
}`,V0=`uniform float size;
uniform float scale;
#include <common>
#include <color_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
#ifdef USE_POINTS_UV
	varying vec2 vUv;
	uniform mat3 uvTransform;
#endif
void main() {
	#ifdef USE_POINTS_UV
		vUv = ( uvTransform * vec3( uv, 1 ) ).xy;
	#endif
	#include <color_vertex>
	#include <morphinstance_vertex>
	#include <morphcolor_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <project_vertex>
	gl_PointSize = size;
	#ifdef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) gl_PointSize *= ( scale / - mvPosition.z );
	#endif
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <worldpos_vertex>
	#include <fog_vertex>
}`,H0=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <color_pars_fragment>
#include <map_particle_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_particle_fragment>
	#include <color_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
	#include <premultiplied_alpha_fragment>
}`,G0=`#include <common>
#include <batching_pars_vertex>
#include <fog_pars_vertex>
#include <morphtarget_pars_vertex>
#include <skinning_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <shadowmap_pars_vertex>
void main() {
	#include <batching_vertex>
	#include <beginnormal_vertex>
	#include <morphinstance_vertex>
	#include <morphnormal_vertex>
	#include <skinbase_vertex>
	#include <skinnormal_vertex>
	#include <defaultnormal_vertex>
	#include <begin_vertex>
	#include <morphtarget_vertex>
	#include <skinning_vertex>
	#include <project_vertex>
	#include <logdepthbuf_vertex>
	#include <worldpos_vertex>
	#include <shadowmap_vertex>
	#include <fog_vertex>
}`,W0=`uniform vec3 color;
uniform float opacity;
#include <common>
#include <packing>
#include <fog_pars_fragment>
#include <bsdfs>
#include <lights_pars_begin>
#include <logdepthbuf_pars_fragment>
#include <shadowmap_pars_fragment>
#include <shadowmask_pars_fragment>
void main() {
	#include <logdepthbuf_fragment>
	gl_FragColor = vec4( color, opacity * ( 1.0 - getShadowMask() ) );
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,$0=`uniform float rotation;
uniform vec2 center;
#include <common>
#include <uv_pars_vertex>
#include <fog_pars_vertex>
#include <logdepthbuf_pars_vertex>
#include <clipping_planes_pars_vertex>
void main() {
	#include <uv_vertex>
	vec4 mvPosition = modelViewMatrix[ 3 ];
	vec2 scale = vec2( length( modelMatrix[ 0 ].xyz ), length( modelMatrix[ 1 ].xyz ) );
	#ifndef USE_SIZEATTENUATION
		bool isPerspective = isPerspectiveMatrix( projectionMatrix );
		if ( isPerspective ) scale *= - mvPosition.z;
	#endif
	vec2 alignedPosition = ( position.xy - ( center - vec2( 0.5 ) ) ) * scale;
	vec2 rotatedPosition;
	rotatedPosition.x = cos( rotation ) * alignedPosition.x - sin( rotation ) * alignedPosition.y;
	rotatedPosition.y = sin( rotation ) * alignedPosition.x + cos( rotation ) * alignedPosition.y;
	mvPosition.xy += rotatedPosition;
	gl_Position = projectionMatrix * mvPosition;
	#include <logdepthbuf_vertex>
	#include <clipping_planes_vertex>
	#include <fog_vertex>
}`,j0=`uniform vec3 diffuse;
uniform float opacity;
#include <common>
#include <uv_pars_fragment>
#include <map_pars_fragment>
#include <alphamap_pars_fragment>
#include <alphatest_pars_fragment>
#include <alphahash_pars_fragment>
#include <fog_pars_fragment>
#include <logdepthbuf_pars_fragment>
#include <clipping_planes_pars_fragment>
void main() {
	vec4 diffuseColor = vec4( diffuse, opacity );
	#include <clipping_planes_fragment>
	vec3 outgoingLight = vec3( 0.0 );
	#include <logdepthbuf_fragment>
	#include <map_fragment>
	#include <alphamap_fragment>
	#include <alphatest_fragment>
	#include <alphahash_fragment>
	outgoingLight = diffuseColor.rgb;
	#include <opaque_fragment>
	#include <tonemapping_fragment>
	#include <colorspace_fragment>
	#include <fog_fragment>
}`,ue={alphahash_fragment:pg,alphahash_pars_fragment:mg,alphamap_fragment:gg,alphamap_pars_fragment:_g,alphatest_fragment:vg,alphatest_pars_fragment:xg,aomap_fragment:bg,aomap_pars_fragment:yg,batching_pars_vertex:Mg,batching_vertex:Sg,begin_vertex:Eg,beginnormal_vertex:Tg,bsdfs:wg,iridescence_fragment:Ag,bumpmap_pars_fragment:Cg,clipping_planes_fragment:Rg,clipping_planes_pars_fragment:Pg,clipping_planes_pars_vertex:Lg,clipping_planes_vertex:Ng,color_fragment:Dg,color_pars_fragment:Ig,color_pars_vertex:Ug,color_vertex:kg,common:Fg,cube_uv_reflection_fragment:Og,defaultnormal_vertex:Bg,displacementmap_pars_vertex:zg,displacementmap_vertex:Vg,emissivemap_fragment:Hg,emissivemap_pars_fragment:Gg,colorspace_fragment:Wg,colorspace_pars_fragment:$g,envmap_fragment:jg,envmap_common_pars_fragment:Xg,envmap_pars_fragment:qg,envmap_pars_vertex:Yg,envmap_physical_pars_fragment:a_,envmap_vertex:Kg,fog_vertex:Jg,fog_pars_vertex:Zg,fog_fragment:Qg,fog_pars_fragment:t_,gradientmap_pars_fragment:e_,lightmap_pars_fragment:n_,lights_lambert_fragment:i_,lights_lambert_pars_fragment:s_,lights_pars_begin:r_,lights_toon_fragment:o_,lights_toon_pars_fragment:l_,lights_phong_fragment:c_,lights_phong_pars_fragment:u_,lights_physical_fragment:h_,lights_physical_pars_fragment:d_,lights_fragment_begin:f_,lights_fragment_maps:p_,lights_fragment_end:m_,logdepthbuf_fragment:g_,logdepthbuf_pars_fragment:__,logdepthbuf_pars_vertex:v_,logdepthbuf_vertex:x_,map_fragment:b_,map_pars_fragment:y_,map_particle_fragment:M_,map_particle_pars_fragment:S_,metalnessmap_fragment:E_,metalnessmap_pars_fragment:T_,morphinstance_vertex:w_,morphcolor_vertex:A_,morphnormal_vertex:C_,morphtarget_pars_vertex:R_,morphtarget_vertex:P_,normal_fragment_begin:L_,normal_fragment_maps:N_,normal_pars_fragment:D_,normal_pars_vertex:I_,normal_vertex:U_,normalmap_pars_fragment:k_,clearcoat_normal_fragment_begin:F_,clearcoat_normal_fragment_maps:O_,clearcoat_pars_fragment:B_,iridescence_pars_fragment:z_,opaque_fragment:V_,packing:H_,premultiplied_alpha_fragment:G_,project_vertex:W_,dithering_fragment:$_,dithering_pars_fragment:j_,roughnessmap_fragment:X_,roughnessmap_pars_fragment:q_,shadowmap_pars_fragment:Y_,shadowmap_pars_vertex:K_,shadowmap_vertex:J_,shadowmask_pars_fragment:Z_,skinbase_vertex:Q_,skinning_pars_vertex:t0,skinning_vertex:e0,skinnormal_vertex:n0,specularmap_fragment:i0,specularmap_pars_fragment:s0,tonemapping_fragment:r0,tonemapping_pars_fragment:a0,transmission_fragment:o0,transmission_pars_fragment:l0,uv_pars_fragment:c0,uv_pars_vertex:u0,uv_vertex:h0,worldpos_vertex:d0,background_vert:f0,background_frag:p0,backgroundCube_vert:m0,backgroundCube_frag:g0,cube_vert:_0,cube_frag:v0,depth_vert:x0,depth_frag:b0,distanceRGBA_vert:y0,distanceRGBA_frag:M0,equirect_vert:S0,equirect_frag:E0,linedashed_vert:T0,linedashed_frag:w0,meshbasic_vert:A0,meshbasic_frag:C0,meshlambert_vert:R0,meshlambert_frag:P0,meshmatcap_vert:L0,meshmatcap_frag:N0,meshnormal_vert:D0,meshnormal_frag:I0,meshphong_vert:U0,meshphong_frag:k0,meshphysical_vert:F0,meshphysical_frag:O0,meshtoon_vert:B0,meshtoon_frag:z0,points_vert:V0,points_frag:H0,shadow_vert:G0,shadow_frag:W0,sprite_vert:$0,sprite_frag:j0},bt={common:{diffuse:{value:new ne(16777215)},opacity:{value:1},map:{value:null},mapTransform:{value:new le},alphaMap:{value:null},alphaMapTransform:{value:new le},alphaTest:{value:0}},specularmap:{specularMap:{value:null},specularMapTransform:{value:new le}},envmap:{envMap:{value:null},envMapRotation:{value:new le},flipEnvMap:{value:-1},reflectivity:{value:1},ior:{value:1.5},refractionRatio:{value:.98}},aomap:{aoMap:{value:null},aoMapIntensity:{value:1},aoMapTransform:{value:new le}},lightmap:{lightMap:{value:null},lightMapIntensity:{value:1},lightMapTransform:{value:new le}},bumpmap:{bumpMap:{value:null},bumpMapTransform:{value:new le},bumpScale:{value:1}},normalmap:{normalMap:{value:null},normalMapTransform:{value:new le},normalScale:{value:new Yt(1,1)}},displacementmap:{displacementMap:{value:null},displacementMapTransform:{value:new le},displacementScale:{value:1},displacementBias:{value:0}},emissivemap:{emissiveMap:{value:null},emissiveMapTransform:{value:new le}},metalnessmap:{metalnessMap:{value:null},metalnessMapTransform:{value:new le}},roughnessmap:{roughnessMap:{value:null},roughnessMapTransform:{value:new le}},gradientmap:{gradientMap:{value:null}},fog:{fogDensity:{value:25e-5},fogNear:{value:1},fogFar:{value:2e3},fogColor:{value:new ne(16777215)}},lights:{ambientLightColor:{value:[]},lightProbe:{value:[]},directionalLights:{value:[],properties:{direction:{},color:{}}},directionalLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},directionalShadowMap:{value:[]},directionalShadowMatrix:{value:[]},spotLights:{value:[],properties:{color:{},position:{},direction:{},distance:{},coneCos:{},penumbraCos:{},decay:{}}},spotLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{}}},spotLightMap:{value:[]},spotShadowMap:{value:[]},spotLightMatrix:{value:[]},pointLights:{value:[],properties:{color:{},position:{},decay:{},distance:{}}},pointLightShadows:{value:[],properties:{shadowIntensity:1,shadowBias:{},shadowNormalBias:{},shadowRadius:{},shadowMapSize:{},shadowCameraNear:{},shadowCameraFar:{}}},pointShadowMap:{value:[]},pointShadowMatrix:{value:[]},hemisphereLights:{value:[],properties:{direction:{},skyColor:{},groundColor:{}}},rectAreaLights:{value:[],properties:{color:{},position:{},width:{},height:{}}},ltc_1:{value:null},ltc_2:{value:null}},points:{diffuse:{value:new ne(16777215)},opacity:{value:1},size:{value:1},scale:{value:1},map:{value:null},alphaMap:{value:null},alphaMapTransform:{value:new le},alphaTest:{value:0},uvTransform:{value:new le}},sprite:{diffuse:{value:new ne(16777215)},opacity:{value:1},center:{value:new Yt(.5,.5)},rotation:{value:0},map:{value:null},mapTransform:{value:new le},alphaMap:{value:null},alphaMapTransform:{value:new le},alphaTest:{value:0}}},Gn={basic:{uniforms:hn([bt.common,bt.specularmap,bt.envmap,bt.aomap,bt.lightmap,bt.fog]),vertexShader:ue.meshbasic_vert,fragmentShader:ue.meshbasic_frag},lambert:{uniforms:hn([bt.common,bt.specularmap,bt.envmap,bt.aomap,bt.lightmap,bt.emissivemap,bt.bumpmap,bt.normalmap,bt.displacementmap,bt.fog,bt.lights,{emissive:{value:new ne(0)}}]),vertexShader:ue.meshlambert_vert,fragmentShader:ue.meshlambert_frag},phong:{uniforms:hn([bt.common,bt.specularmap,bt.envmap,bt.aomap,bt.lightmap,bt.emissivemap,bt.bumpmap,bt.normalmap,bt.displacementmap,bt.fog,bt.lights,{emissive:{value:new ne(0)},specular:{value:new ne(1118481)},shininess:{value:30}}]),vertexShader:ue.meshphong_vert,fragmentShader:ue.meshphong_frag},standard:{uniforms:hn([bt.common,bt.envmap,bt.aomap,bt.lightmap,bt.emissivemap,bt.bumpmap,bt.normalmap,bt.displacementmap,bt.roughnessmap,bt.metalnessmap,bt.fog,bt.lights,{emissive:{value:new ne(0)},roughness:{value:1},metalness:{value:0},envMapIntensity:{value:1}}]),vertexShader:ue.meshphysical_vert,fragmentShader:ue.meshphysical_frag},toon:{uniforms:hn([bt.common,bt.aomap,bt.lightmap,bt.emissivemap,bt.bumpmap,bt.normalmap,bt.displacementmap,bt.gradientmap,bt.fog,bt.lights,{emissive:{value:new ne(0)}}]),vertexShader:ue.meshtoon_vert,fragmentShader:ue.meshtoon_frag},matcap:{uniforms:hn([bt.common,bt.bumpmap,bt.normalmap,bt.displacementmap,bt.fog,{matcap:{value:null}}]),vertexShader:ue.meshmatcap_vert,fragmentShader:ue.meshmatcap_frag},points:{uniforms:hn([bt.points,bt.fog]),vertexShader:ue.points_vert,fragmentShader:ue.points_frag},dashed:{uniforms:hn([bt.common,bt.fog,{scale:{value:1},dashSize:{value:1},totalSize:{value:2}}]),vertexShader:ue.linedashed_vert,fragmentShader:ue.linedashed_frag},depth:{uniforms:hn([bt.common,bt.displacementmap]),vertexShader:ue.depth_vert,fragmentShader:ue.depth_frag},normal:{uniforms:hn([bt.common,bt.bumpmap,bt.normalmap,bt.displacementmap,{opacity:{value:1}}]),vertexShader:ue.meshnormal_vert,fragmentShader:ue.meshnormal_frag},sprite:{uniforms:hn([bt.sprite,bt.fog]),vertexShader:ue.sprite_vert,fragmentShader:ue.sprite_frag},background:{uniforms:{uvTransform:{value:new le},t2D:{value:null},backgroundIntensity:{value:1}},vertexShader:ue.background_vert,fragmentShader:ue.background_frag},backgroundCube:{uniforms:{envMap:{value:null},flipEnvMap:{value:-1},backgroundBlurriness:{value:0},backgroundIntensity:{value:1},backgroundRotation:{value:new le}},vertexShader:ue.backgroundCube_vert,fragmentShader:ue.backgroundCube_frag},cube:{uniforms:{tCube:{value:null},tFlip:{value:-1},opacity:{value:1}},vertexShader:ue.cube_vert,fragmentShader:ue.cube_frag},equirect:{uniforms:{tEquirect:{value:null}},vertexShader:ue.equirect_vert,fragmentShader:ue.equirect_frag},distanceRGBA:{uniforms:hn([bt.common,bt.displacementmap,{referencePosition:{value:new O},nearDistance:{value:1},farDistance:{value:1e3}}]),vertexShader:ue.distanceRGBA_vert,fragmentShader:ue.distanceRGBA_frag},shadow:{uniforms:hn([bt.lights,bt.fog,{color:{value:new ne(0)},opacity:{value:1}}]),vertexShader:ue.shadow_vert,fragmentShader:ue.shadow_frag}};Gn.physical={uniforms:hn([Gn.standard.uniforms,{clearcoat:{value:0},clearcoatMap:{value:null},clearcoatMapTransform:{value:new le},clearcoatNormalMap:{value:null},clearcoatNormalMapTransform:{value:new le},clearcoatNormalScale:{value:new Yt(1,1)},clearcoatRoughness:{value:0},clearcoatRoughnessMap:{value:null},clearcoatRoughnessMapTransform:{value:new le},dispersion:{value:0},iridescence:{value:0},iridescenceMap:{value:null},iridescenceMapTransform:{value:new le},iridescenceIOR:{value:1.3},iridescenceThicknessMinimum:{value:100},iridescenceThicknessMaximum:{value:400},iridescenceThicknessMap:{value:null},iridescenceThicknessMapTransform:{value:new le},sheen:{value:0},sheenColor:{value:new ne(0)},sheenColorMap:{value:null},sheenColorMapTransform:{value:new le},sheenRoughness:{value:1},sheenRoughnessMap:{value:null},sheenRoughnessMapTransform:{value:new le},transmission:{value:0},transmissionMap:{value:null},transmissionMapTransform:{value:new le},transmissionSamplerSize:{value:new Yt},transmissionSamplerMap:{value:null},thickness:{value:0},thicknessMap:{value:null},thicknessMapTransform:{value:new le},attenuationDistance:{value:0},attenuationColor:{value:new ne(0)},specularColor:{value:new ne(1,1,1)},specularColorMap:{value:null},specularColorMapTransform:{value:new le},specularIntensity:{value:1},specularIntensityMap:{value:null},specularIntensityMapTransform:{value:new le},anisotropyVector:{value:new Yt},anisotropyMap:{value:null},anisotropyMapTransform:{value:new le}}]),vertexShader:ue.meshphysical_vert,fragmentShader:ue.meshphysical_frag};const _a={r:0,b:0,g:0},Wi=new yn,X0=new Kt;function q0(n,t,e,i,s,r,a){const o=new ne(0);let l=r===!0?0:1,c,u,h=null,d=0,p=null;function g(L){let w=L.isScene===!0?L.background:null;return w&&w.isTexture&&(w=(L.backgroundBlurriness>0?e:t).get(w)),w}function _(L){let w=!1;const N=g(L);N===null?f(o,l):N&&N.isColor&&(f(N,1),w=!0);const F=n.xr.getEnvironmentBlendMode();F==="additive"?i.buffers.color.setClear(0,0,0,1,a):F==="alpha-blend"&&i.buffers.color.setClear(0,0,0,0,a),(n.autoClear||w)&&(i.buffers.depth.setTest(!0),i.buffers.depth.setMask(!0),i.buffers.color.setMask(!0),n.clear(n.autoClearColor,n.autoClearDepth,n.autoClearStencil))}function m(L,w){const N=g(w);N&&(N.isCubeTexture||N.mapping===Ka)?(u===void 0&&(u=new Re(new Bn(1,1,1),new Ni({name:"BackgroundCubeMaterial",uniforms:Js(Gn.backgroundCube.uniforms),vertexShader:Gn.backgroundCube.vertexShader,fragmentShader:Gn.backgroundCube.fragmentShader,side:bn,depthTest:!1,depthWrite:!1,fog:!1})),u.geometry.deleteAttribute("normal"),u.geometry.deleteAttribute("uv"),u.onBeforeRender=function(F,U,k){this.matrixWorld.copyPosition(k.matrixWorld)},Object.defineProperty(u.material,"envMap",{get:function(){return this.uniforms.envMap.value}}),s.update(u)),Wi.copy(w.backgroundRotation),Wi.x*=-1,Wi.y*=-1,Wi.z*=-1,N.isCubeTexture&&N.isRenderTargetTexture===!1&&(Wi.y*=-1,Wi.z*=-1),u.material.uniforms.envMap.value=N,u.material.uniforms.flipEnvMap.value=N.isCubeTexture&&N.isRenderTargetTexture===!1?-1:1,u.material.uniforms.backgroundBlurriness.value=w.backgroundBlurriness,u.material.uniforms.backgroundIntensity.value=w.backgroundIntensity,u.material.uniforms.backgroundRotation.value.setFromMatrix4(X0.makeRotationFromEuler(Wi)),u.material.toneMapped=_e.getTransfer(N.colorSpace)!==Ce,(h!==N||d!==N.version||p!==n.toneMapping)&&(u.material.needsUpdate=!0,h=N,d=N.version,p=n.toneMapping),u.layers.enableAll(),L.unshift(u,u.geometry,u.material,0,0,null)):N&&N.isTexture&&(c===void 0&&(c=new Re(new Nr(2,2),new Ni({name:"BackgroundMaterial",uniforms:Js(Gn.background.uniforms),vertexShader:Gn.background.vertexShader,fragmentShader:Gn.background.fragmentShader,side:ui,depthTest:!1,depthWrite:!1,fog:!1})),c.geometry.deleteAttribute("normal"),Object.defineProperty(c.material,"map",{get:function(){return this.uniforms.t2D.value}}),s.update(c)),c.material.uniforms.t2D.value=N,c.material.uniforms.backgroundIntensity.value=w.backgroundIntensity,c.material.toneMapped=_e.getTransfer(N.colorSpace)!==Ce,N.matrixAutoUpdate===!0&&N.updateMatrix(),c.material.uniforms.uvTransform.value.copy(N.matrix),(h!==N||d!==N.version||p!==n.toneMapping)&&(c.material.needsUpdate=!0,h=N,d=N.version,p=n.toneMapping),c.layers.enableAll(),L.unshift(c,c.geometry,c.material,0,0,null))}function f(L,w){L.getRGB(_a,cd(n)),i.buffers.color.setClear(_a.r,_a.g,_a.b,w,a)}function P(){u!==void 0&&(u.geometry.dispose(),u.material.dispose(),u=void 0),c!==void 0&&(c.geometry.dispose(),c.material.dispose(),c=void 0)}return{getClearColor:function(){return o},setClearColor:function(L,w=1){o.set(L),l=w,f(o,l)},getClearAlpha:function(){return l},setClearAlpha:function(L){l=L,f(o,l)},render:_,addToRenderList:m,dispose:P}}function Y0(n,t){const e=n.getParameter(n.MAX_VERTEX_ATTRIBS),i={},s=d(null);let r=s,a=!1;function o(M,I,V,W,Z){let et=!1;const z=h(W,V,I);r!==z&&(r=z,c(r.object)),et=p(M,W,V,Z),et&&g(M,W,V,Z),Z!==null&&t.update(Z,n.ELEMENT_ARRAY_BUFFER),(et||a)&&(a=!1,w(M,I,V,W),Z!==null&&n.bindBuffer(n.ELEMENT_ARRAY_BUFFER,t.get(Z).buffer))}function l(){return n.createVertexArray()}function c(M){return n.bindVertexArray(M)}function u(M){return n.deleteVertexArray(M)}function h(M,I,V){const W=V.wireframe===!0;let Z=i[M.id];Z===void 0&&(Z={},i[M.id]=Z);let et=Z[I.id];et===void 0&&(et={},Z[I.id]=et);let z=et[W];return z===void 0&&(z=d(l()),et[W]=z),z}function d(M){const I=[],V=[],W=[];for(let Z=0;Z<e;Z++)I[Z]=0,V[Z]=0,W[Z]=0;return{geometry:null,program:null,wireframe:!1,newAttributes:I,enabledAttributes:V,attributeDivisors:W,object:M,attributes:{},index:null}}function p(M,I,V,W){const Z=r.attributes,et=I.attributes;let z=0;const it=V.getAttributes();for(const j in it)if(it[j].location>=0){const mt=Z[j];let gt=et[j];if(gt===void 0&&(j==="instanceMatrix"&&M.instanceMatrix&&(gt=M.instanceMatrix),j==="instanceColor"&&M.instanceColor&&(gt=M.instanceColor)),mt===void 0||mt.attribute!==gt||gt&&mt.data!==gt.data)return!0;z++}return r.attributesNum!==z||r.index!==W}function g(M,I,V,W){const Z={},et=I.attributes;let z=0;const it=V.getAttributes();for(const j in it)if(it[j].location>=0){let mt=et[j];mt===void 0&&(j==="instanceMatrix"&&M.instanceMatrix&&(mt=M.instanceMatrix),j==="instanceColor"&&M.instanceColor&&(mt=M.instanceColor));const gt={};gt.attribute=mt,mt&&mt.data&&(gt.data=mt.data),Z[j]=gt,z++}r.attributes=Z,r.attributesNum=z,r.index=W}function _(){const M=r.newAttributes;for(let I=0,V=M.length;I<V;I++)M[I]=0}function m(M){f(M,0)}function f(M,I){const V=r.newAttributes,W=r.enabledAttributes,Z=r.attributeDivisors;V[M]=1,W[M]===0&&(n.enableVertexAttribArray(M),W[M]=1),Z[M]!==I&&(n.vertexAttribDivisor(M,I),Z[M]=I)}function P(){const M=r.newAttributes,I=r.enabledAttributes;for(let V=0,W=I.length;V<W;V++)I[V]!==M[V]&&(n.disableVertexAttribArray(V),I[V]=0)}function L(M,I,V,W,Z,et,z){z===!0?n.vertexAttribIPointer(M,I,V,Z,et):n.vertexAttribPointer(M,I,V,W,Z,et)}function w(M,I,V,W){_();const Z=W.attributes,et=V.getAttributes(),z=I.defaultAttributeValues;for(const it in et){const j=et[it];if(j.location>=0){let ut=Z[it];if(ut===void 0&&(it==="instanceMatrix"&&M.instanceMatrix&&(ut=M.instanceMatrix),it==="instanceColor"&&M.instanceColor&&(ut=M.instanceColor)),ut!==void 0){const mt=ut.normalized,gt=ut.itemSize,Lt=t.get(ut);if(Lt===void 0)continue;const At=Lt.buffer,X=Lt.type,Q=Lt.bytesPerElement,nt=X===n.INT||X===n.UNSIGNED_INT||ut.gpuType===rc;if(ut.isInterleavedBufferAttribute){const st=ut.data,vt=st.stride,se=ut.offset;if(st.isInstancedInterleavedBuffer){for(let Wt=0;Wt<j.locationSize;Wt++)f(j.location+Wt,st.meshPerAttribute);M.isInstancedMesh!==!0&&W._maxInstanceCount===void 0&&(W._maxInstanceCount=st.meshPerAttribute*st.count)}else for(let Wt=0;Wt<j.locationSize;Wt++)m(j.location+Wt);n.bindBuffer(n.ARRAY_BUFFER,At);for(let Wt=0;Wt<j.locationSize;Wt++)L(j.location+Wt,gt/j.locationSize,X,mt,vt*Q,(se+gt/j.locationSize*Wt)*Q,nt)}else{if(ut.isInstancedBufferAttribute){for(let st=0;st<j.locationSize;st++)f(j.location+st,ut.meshPerAttribute);M.isInstancedMesh!==!0&&W._maxInstanceCount===void 0&&(W._maxInstanceCount=ut.meshPerAttribute*ut.count)}else for(let st=0;st<j.locationSize;st++)m(j.location+st);n.bindBuffer(n.ARRAY_BUFFER,At);for(let st=0;st<j.locationSize;st++)L(j.location+st,gt/j.locationSize,X,mt,gt*Q,gt/j.locationSize*st*Q,nt)}}else if(z!==void 0){const mt=z[it];if(mt!==void 0)switch(mt.length){case 2:n.vertexAttrib2fv(j.location,mt);break;case 3:n.vertexAttrib3fv(j.location,mt);break;case 4:n.vertexAttrib4fv(j.location,mt);break;default:n.vertexAttrib1fv(j.location,mt)}}}}P()}function N(){k();for(const M in i){const I=i[M];for(const V in I){const W=I[V];for(const Z in W)u(W[Z].object),delete W[Z];delete I[V]}delete i[M]}}function F(M){if(i[M.id]===void 0)return;const I=i[M.id];for(const V in I){const W=I[V];for(const Z in W)u(W[Z].object),delete W[Z];delete I[V]}delete i[M.id]}function U(M){for(const I in i){const V=i[I];if(V[M.id]===void 0)continue;const W=V[M.id];for(const Z in W)u(W[Z].object),delete W[Z];delete V[M.id]}}function k(){y(),a=!0,r!==s&&(r=s,c(r.object))}function y(){s.geometry=null,s.program=null,s.wireframe=!1}return{setup:o,reset:k,resetDefaultState:y,dispose:N,releaseStatesOfGeometry:F,releaseStatesOfProgram:U,initAttributes:_,enableAttribute:m,disableUnusedAttributes:P}}function K0(n,t,e){let i;function s(c){i=c}function r(c,u){n.drawArrays(i,c,u),e.update(u,i,1)}function a(c,u,h){h!==0&&(n.drawArraysInstanced(i,c,u,h),e.update(u,i,h))}function o(c,u,h){if(h===0)return;t.get("WEBGL_multi_draw").multiDrawArraysWEBGL(i,c,0,u,0,h);let p=0;for(let g=0;g<h;g++)p+=u[g];e.update(p,i,1)}function l(c,u,h,d){if(h===0)return;const p=t.get("WEBGL_multi_draw");if(p===null)for(let g=0;g<c.length;g++)a(c[g],u[g],d[g]);else{p.multiDrawArraysInstancedWEBGL(i,c,0,u,0,d,0,h);let g=0;for(let _=0;_<h;_++)g+=u[_]*d[_];e.update(g,i,1)}}this.setMode=s,this.render=r,this.renderInstances=a,this.renderMultiDraw=o,this.renderMultiDrawInstances=l}function J0(n,t,e,i){let s;function r(){if(s!==void 0)return s;if(t.has("EXT_texture_filter_anisotropic")===!0){const U=t.get("EXT_texture_filter_anisotropic");s=n.getParameter(U.MAX_TEXTURE_MAX_ANISOTROPY_EXT)}else s=0;return s}function a(U){return!(U!==Pn&&i.convert(U)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_FORMAT))}function o(U){const k=U===Rr&&(t.has("EXT_color_buffer_half_float")||t.has("EXT_color_buffer_float"));return!(U!==hi&&i.convert(U)!==n.getParameter(n.IMPLEMENTATION_COLOR_READ_TYPE)&&U!==jn&&!k)}function l(U){if(U==="highp"){if(n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.HIGH_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.HIGH_FLOAT).precision>0)return"highp";U="mediump"}return U==="mediump"&&n.getShaderPrecisionFormat(n.VERTEX_SHADER,n.MEDIUM_FLOAT).precision>0&&n.getShaderPrecisionFormat(n.FRAGMENT_SHADER,n.MEDIUM_FLOAT).precision>0?"mediump":"lowp"}let c=e.precision!==void 0?e.precision:"highp";const u=l(c);u!==c&&(console.warn("THREE.WebGLRenderer:",c,"not supported, using",u,"instead."),c=u);const h=e.logarithmicDepthBuffer===!0,d=e.reverseDepthBuffer===!0&&t.has("EXT_clip_control"),p=n.getParameter(n.MAX_TEXTURE_IMAGE_UNITS),g=n.getParameter(n.MAX_VERTEX_TEXTURE_IMAGE_UNITS),_=n.getParameter(n.MAX_TEXTURE_SIZE),m=n.getParameter(n.MAX_CUBE_MAP_TEXTURE_SIZE),f=n.getParameter(n.MAX_VERTEX_ATTRIBS),P=n.getParameter(n.MAX_VERTEX_UNIFORM_VECTORS),L=n.getParameter(n.MAX_VARYING_VECTORS),w=n.getParameter(n.MAX_FRAGMENT_UNIFORM_VECTORS),N=g>0,F=n.getParameter(n.MAX_SAMPLES);return{isWebGL2:!0,getMaxAnisotropy:r,getMaxPrecision:l,textureFormatReadable:a,textureTypeReadable:o,precision:c,logarithmicDepthBuffer:h,reverseDepthBuffer:d,maxTextures:p,maxVertexTextures:g,maxTextureSize:_,maxCubemapSize:m,maxAttributes:f,maxVertexUniforms:P,maxVaryings:L,maxFragmentUniforms:w,vertexTextures:N,maxSamples:F}}function Z0(n){const t=this;let e=null,i=0,s=!1,r=!1;const a=new wi,o=new le,l={value:null,needsUpdate:!1};this.uniform=l,this.numPlanes=0,this.numIntersection=0,this.init=function(h,d){const p=h.length!==0||d||i!==0||s;return s=d,i=h.length,p},this.beginShadows=function(){r=!0,u(null)},this.endShadows=function(){r=!1},this.setGlobalState=function(h,d){e=u(h,d,0)},this.setState=function(h,d,p){const g=h.clippingPlanes,_=h.clipIntersection,m=h.clipShadows,f=n.get(h);if(!s||g===null||g.length===0||r&&!m)r?u(null):c();else{const P=r?0:i,L=P*4;let w=f.clippingState||null;l.value=w,w=u(g,d,L,p);for(let N=0;N!==L;++N)w[N]=e[N];f.clippingState=w,this.numIntersection=_?this.numPlanes:0,this.numPlanes+=P}};function c(){l.value!==e&&(l.value=e,l.needsUpdate=i>0),t.numPlanes=i,t.numIntersection=0}function u(h,d,p,g){const _=h!==null?h.length:0;let m=null;if(_!==0){if(m=l.value,g!==!0||m===null){const f=p+_*4,P=d.matrixWorldInverse;o.getNormalMatrix(P),(m===null||m.length<f)&&(m=new Float32Array(f));for(let L=0,w=p;L!==_;++L,w+=4)a.copy(h[L]).applyMatrix4(P,o),a.normal.toArray(m,w),m[w+3]=a.constant}l.value=m,l.needsUpdate=!0}return t.numPlanes=_,t.numIntersection=0,m}}function Q0(n){let t=new WeakMap;function e(a,o){return o===fl?a.mapping=$s:o===pl&&(a.mapping=js),a}function i(a){if(a&&a.isTexture){const o=a.mapping;if(o===fl||o===pl)if(t.has(a)){const l=t.get(a).texture;return e(l,a.mapping)}else{const l=a.image;if(l&&l.height>0){const c=new km(l.height);return c.fromEquirectangularTexture(n,a),t.set(a,c),a.addEventListener("dispose",s),e(c.texture,a.mapping)}else return null}}return a}function s(a){const o=a.target;o.removeEventListener("dispose",s);const l=t.get(o);l!==void 0&&(t.delete(o),l.dispose())}function r(){t=new WeakMap}return{get:i,dispose:r}}const Us=4,Vu=[.125,.215,.35,.446,.526,.582],Yi=20,$o=new xc,Hu=new ne;let jo=null,Xo=0,qo=0,Yo=!1;const Xi=(1+Math.sqrt(5))/2,Ps=1/Xi,Gu=[new O(-Xi,Ps,0),new O(Xi,Ps,0),new O(-Ps,0,Xi),new O(Ps,0,Xi),new O(0,Xi,-Ps),new O(0,Xi,Ps),new O(-1,1,-1),new O(1,1,-1),new O(-1,1,1),new O(1,1,1)],tv=new O;class Wu{constructor(t){this._renderer=t,this._pingPongRenderTarget=null,this._lodMax=0,this._cubeSize=0,this._lodPlanes=[],this._sizeLods=[],this._sigmas=[],this._blurMaterial=null,this._cubemapMaterial=null,this._equirectMaterial=null,this._compileMaterial(this._blurMaterial)}fromScene(t,e=0,i=.1,s=100,r={}){const{size:a=256,position:o=tv}=r;jo=this._renderer.getRenderTarget(),Xo=this._renderer.getActiveCubeFace(),qo=this._renderer.getActiveMipmapLevel(),Yo=this._renderer.xr.enabled,this._renderer.xr.enabled=!1,this._setSize(a);const l=this._allocateTargets();return l.depthBuffer=!0,this._sceneToCubeUV(t,i,s,l,o),e>0&&this._blur(l,0,0,e),this._applyPMREM(l),this._cleanup(l),l}fromEquirectangular(t,e=null){return this._fromTexture(t,e)}fromCubemap(t,e=null){return this._fromTexture(t,e)}compileCubemapShader(){this._cubemapMaterial===null&&(this._cubemapMaterial=Xu(),this._compileMaterial(this._cubemapMaterial))}compileEquirectangularShader(){this._equirectMaterial===null&&(this._equirectMaterial=ju(),this._compileMaterial(this._equirectMaterial))}dispose(){this._dispose(),this._cubemapMaterial!==null&&this._cubemapMaterial.dispose(),this._equirectMaterial!==null&&this._equirectMaterial.dispose()}_setSize(t){this._lodMax=Math.floor(Math.log2(t)),this._cubeSize=Math.pow(2,this._lodMax)}_dispose(){this._blurMaterial!==null&&this._blurMaterial.dispose(),this._pingPongRenderTarget!==null&&this._pingPongRenderTarget.dispose();for(let t=0;t<this._lodPlanes.length;t++)this._lodPlanes[t].dispose()}_cleanup(t){this._renderer.setRenderTarget(jo,Xo,qo),this._renderer.xr.enabled=Yo,t.scissorTest=!1,va(t,0,0,t.width,t.height)}_fromTexture(t,e){t.mapping===$s||t.mapping===js?this._setSize(t.image.length===0?16:t.image[0].width||t.image[0].image.width):this._setSize(t.image.width/4),jo=this._renderer.getRenderTarget(),Xo=this._renderer.getActiveCubeFace(),qo=this._renderer.getActiveMipmapLevel(),Yo=this._renderer.xr.enabled,this._renderer.xr.enabled=!1;const i=e||this._allocateTargets();return this._textureToCubeUV(t,i),this._applyPMREM(i),this._cleanup(i),i}_allocateTargets(){const t=3*Math.max(this._cubeSize,112),e=4*this._cubeSize,i={magFilter:wn,minFilter:wn,generateMipmaps:!1,type:Rr,format:Pn,colorSpace:Ys,depthBuffer:!1},s=$u(t,e,i);if(this._pingPongRenderTarget===null||this._pingPongRenderTarget.width!==t||this._pingPongRenderTarget.height!==e){this._pingPongRenderTarget!==null&&this._dispose(),this._pingPongRenderTarget=$u(t,e,i);const{_lodMax:r}=this;({sizeLods:this._sizeLods,lodPlanes:this._lodPlanes,sigmas:this._sigmas}=ev(r)),this._blurMaterial=nv(r,t,e)}return s}_compileMaterial(t){const e=new Re(this._lodPlanes[0],t);this._renderer.compile(e,$o)}_sceneToCubeUV(t,e,i,s,r){const l=new an(90,1,e,i),c=[1,-1,1,1,1,1],u=[1,1,1,-1,-1,-1],h=this._renderer,d=h.autoClear,p=h.toneMapping;h.getClearColor(Hu),h.toneMapping=Pi,h.autoClear=!1;const g=new Er({name:"PMREM.Background",side:bn,depthWrite:!1,depthTest:!1}),_=new Re(new Bn,g);let m=!1;const f=t.background;f?f.isColor&&(g.color.copy(f),t.background=null,m=!0):(g.color.copy(Hu),m=!0);for(let P=0;P<6;P++){const L=P%3;L===0?(l.up.set(0,c[P],0),l.position.set(r.x,r.y,r.z),l.lookAt(r.x+u[P],r.y,r.z)):L===1?(l.up.set(0,0,c[P]),l.position.set(r.x,r.y,r.z),l.lookAt(r.x,r.y+u[P],r.z)):(l.up.set(0,c[P],0),l.position.set(r.x,r.y,r.z),l.lookAt(r.x,r.y,r.z+u[P]));const w=this._cubeSize;va(s,L*w,P>2?w:0,w,w),h.setRenderTarget(s),m&&h.render(_,l),h.render(t,l)}_.geometry.dispose(),_.material.dispose(),h.toneMapping=p,h.autoClear=d,t.background=f}_textureToCubeUV(t,e){const i=this._renderer,s=t.mapping===$s||t.mapping===js;s?(this._cubemapMaterial===null&&(this._cubemapMaterial=Xu()),this._cubemapMaterial.uniforms.flipEnvMap.value=t.isRenderTargetTexture===!1?-1:1):this._equirectMaterial===null&&(this._equirectMaterial=ju());const r=s?this._cubemapMaterial:this._equirectMaterial,a=new Re(this._lodPlanes[0],r),o=r.uniforms;o.envMap.value=t;const l=this._cubeSize;va(e,0,0,3*l,2*l),i.setRenderTarget(e),i.render(a,$o)}_applyPMREM(t){const e=this._renderer,i=e.autoClear;e.autoClear=!1;const s=this._lodPlanes.length;for(let r=1;r<s;r++){const a=Math.sqrt(this._sigmas[r]*this._sigmas[r]-this._sigmas[r-1]*this._sigmas[r-1]),o=Gu[(s-r-1)%Gu.length];this._blur(t,r-1,r,a,o)}e.autoClear=i}_blur(t,e,i,s,r){const a=this._pingPongRenderTarget;this._halfBlur(t,a,e,i,s,"latitudinal",r),this._halfBlur(a,t,i,i,s,"longitudinal",r)}_halfBlur(t,e,i,s,r,a,o){const l=this._renderer,c=this._blurMaterial;a!=="latitudinal"&&a!=="longitudinal"&&console.error("blur direction must be either latitudinal or longitudinal!");const u=3,h=new Re(this._lodPlanes[s],c),d=c.uniforms,p=this._sizeLods[i]-1,g=isFinite(r)?Math.PI/(2*p):2*Math.PI/(2*Yi-1),_=r/g,m=isFinite(r)?1+Math.floor(u*_):Yi;m>Yi&&console.warn(`sigmaRadians, ${r}, is too large and will clip, as it requested ${m} samples when the maximum is set to ${Yi}`);const f=[];let P=0;for(let U=0;U<Yi;++U){const k=U/_,y=Math.exp(-k*k/2);f.push(y),U===0?P+=y:U<m&&(P+=2*y)}for(let U=0;U<f.length;U++)f[U]=f[U]/P;d.envMap.value=t.texture,d.samples.value=m,d.weights.value=f,d.latitudinal.value=a==="latitudinal",o&&(d.poleAxis.value=o);const{_lodMax:L}=this;d.dTheta.value=g,d.mipInt.value=L-i;const w=this._sizeLods[s],N=3*w*(s>L-Us?s-L+Us:0),F=4*(this._cubeSize-w);va(e,N,F,3*w,2*w),l.setRenderTarget(e),l.render(h,$o)}}function ev(n){const t=[],e=[],i=[];let s=n;const r=n-Us+1+Vu.length;for(let a=0;a<r;a++){const o=Math.pow(2,s);e.push(o);let l=1/o;a>n-Us?l=Vu[a-n+Us-1]:a===0&&(l=0),i.push(l);const c=1/(o-2),u=-c,h=1+c,d=[u,u,h,u,h,h,u,u,h,h,u,h],p=6,g=6,_=3,m=2,f=1,P=new Float32Array(_*g*p),L=new Float32Array(m*g*p),w=new Float32Array(f*g*p);for(let F=0;F<p;F++){const U=F%3*2/3-1,k=F>2?0:-1,y=[U,k,0,U+2/3,k,0,U+2/3,k+1,0,U,k,0,U+2/3,k+1,0,U,k+1,0];P.set(y,_*g*F),L.set(d,m*g*F);const M=[F,F,F,F,F,F];w.set(M,f*g*F)}const N=new $e;N.setAttribute("position",new dn(P,_)),N.setAttribute("uv",new dn(L,m)),N.setAttribute("faceIndex",new dn(w,f)),t.push(N),s>Us&&s--}return{lodPlanes:t,sizeLods:e,sigmas:i}}function $u(n,t,e){const i=new es(n,t,e);return i.texture.mapping=Ka,i.texture.name="PMREM.cubeUv",i.scissorTest=!0,i}function va(n,t,e,i,s){n.viewport.set(t,e,i,s),n.scissor.set(t,e,i,s)}function nv(n,t,e){const i=new Float32Array(Yi),s=new O(0,1,0);return new Ni({name:"SphericalGaussianBlur",defines:{n:Yi,CUBEUV_TEXEL_WIDTH:1/t,CUBEUV_TEXEL_HEIGHT:1/e,CUBEUV_MAX_MIP:`${n}.0`},uniforms:{envMap:{value:null},samples:{value:1},weights:{value:i},latitudinal:{value:!1},dTheta:{value:0},mipInt:{value:0},poleAxis:{value:s}},vertexShader:bc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;
			uniform int samples;
			uniform float weights[ n ];
			uniform bool latitudinal;
			uniform float dTheta;
			uniform float mipInt;
			uniform vec3 poleAxis;

			#define ENVMAP_TYPE_CUBE_UV
			#include <cube_uv_reflection_fragment>

			vec3 getSample( float theta, vec3 axis ) {

				float cosTheta = cos( theta );
				// Rodrigues' axis-angle rotation
				vec3 sampleDirection = vOutputDirection * cosTheta
					+ cross( axis, vOutputDirection ) * sin( theta )
					+ axis * dot( axis, vOutputDirection ) * ( 1.0 - cosTheta );

				return bilinearCubeUV( envMap, sampleDirection, mipInt );

			}

			void main() {

				vec3 axis = latitudinal ? poleAxis : cross( poleAxis, vOutputDirection );

				if ( all( equal( axis, vec3( 0.0 ) ) ) ) {

					axis = vec3( vOutputDirection.z, 0.0, - vOutputDirection.x );

				}

				axis = normalize( axis );

				gl_FragColor = vec4( 0.0, 0.0, 0.0, 1.0 );
				gl_FragColor.rgb += weights[ 0 ] * getSample( 0.0, axis );

				for ( int i = 1; i < n; i++ ) {

					if ( i >= samples ) {

						break;

					}

					float theta = dTheta * float( i );
					gl_FragColor.rgb += weights[ i ] * getSample( -1.0 * theta, axis );
					gl_FragColor.rgb += weights[ i ] * getSample( theta, axis );

				}

			}
		`,blending:Ri,depthTest:!1,depthWrite:!1})}function ju(){return new Ni({name:"EquirectangularToCubeUV",uniforms:{envMap:{value:null}},vertexShader:bc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			varying vec3 vOutputDirection;

			uniform sampler2D envMap;

			#include <common>

			void main() {

				vec3 outputDirection = normalize( vOutputDirection );
				vec2 uv = equirectUv( outputDirection );

				gl_FragColor = vec4( texture2D ( envMap, uv ).rgb, 1.0 );

			}
		`,blending:Ri,depthTest:!1,depthWrite:!1})}function Xu(){return new Ni({name:"CubemapToCubeUV",uniforms:{envMap:{value:null},flipEnvMap:{value:-1}},vertexShader:bc(),fragmentShader:`

			precision mediump float;
			precision mediump int;

			uniform float flipEnvMap;

			varying vec3 vOutputDirection;

			uniform samplerCube envMap;

			void main() {

				gl_FragColor = textureCube( envMap, vec3( flipEnvMap * vOutputDirection.x, vOutputDirection.yz ) );

			}
		`,blending:Ri,depthTest:!1,depthWrite:!1})}function bc(){return`

		precision mediump float;
		precision mediump int;

		attribute float faceIndex;

		varying vec3 vOutputDirection;

		// RH coordinate system; PMREM face-indexing convention
		vec3 getDirection( vec2 uv, float face ) {

			uv = 2.0 * uv - 1.0;

			vec3 direction = vec3( uv, 1.0 );

			if ( face == 0.0 ) {

				direction = direction.zyx; // ( 1, v, u ) pos x

			} else if ( face == 1.0 ) {

				direction = direction.xzy;
				direction.xz *= -1.0; // ( -u, 1, -v ) pos y

			} else if ( face == 2.0 ) {

				direction.x *= -1.0; // ( -u, v, 1 ) pos z

			} else if ( face == 3.0 ) {

				direction = direction.zyx;
				direction.xz *= -1.0; // ( -1, v, -u ) neg x

			} else if ( face == 4.0 ) {

				direction = direction.xzy;
				direction.xy *= -1.0; // ( -u, -1, v ) neg y

			} else if ( face == 5.0 ) {

				direction.z *= -1.0; // ( u, v, -1 ) neg z

			}

			return direction;

		}

		void main() {

			vOutputDirection = getDirection( uv, faceIndex );
			gl_Position = vec4( position, 1.0 );

		}
	`}function iv(n){let t=new WeakMap,e=null;function i(o){if(o&&o.isTexture){const l=o.mapping,c=l===fl||l===pl,u=l===$s||l===js;if(c||u){let h=t.get(o);const d=h!==void 0?h.texture.pmremVersion:0;if(o.isRenderTargetTexture&&o.pmremVersion!==d)return e===null&&(e=new Wu(n)),h=c?e.fromEquirectangular(o,h):e.fromCubemap(o,h),h.texture.pmremVersion=o.pmremVersion,t.set(o,h),h.texture;if(h!==void 0)return h.texture;{const p=o.image;return c&&p&&p.height>0||u&&p&&s(p)?(e===null&&(e=new Wu(n)),h=c?e.fromEquirectangular(o):e.fromCubemap(o),h.texture.pmremVersion=o.pmremVersion,t.set(o,h),o.addEventListener("dispose",r),h.texture):null}}}return o}function s(o){let l=0;const c=6;for(let u=0;u<c;u++)o[u]!==void 0&&l++;return l===c}function r(o){const l=o.target;l.removeEventListener("dispose",r);const c=t.get(l);c!==void 0&&(t.delete(l),c.dispose())}function a(){t=new WeakMap,e!==null&&(e.dispose(),e=null)}return{get:i,dispose:a}}function sv(n){const t={};function e(i){if(t[i]!==void 0)return t[i];let s;switch(i){case"WEBGL_depth_texture":s=n.getExtension("WEBGL_depth_texture")||n.getExtension("MOZ_WEBGL_depth_texture")||n.getExtension("WEBKIT_WEBGL_depth_texture");break;case"EXT_texture_filter_anisotropic":s=n.getExtension("EXT_texture_filter_anisotropic")||n.getExtension("MOZ_EXT_texture_filter_anisotropic")||n.getExtension("WEBKIT_EXT_texture_filter_anisotropic");break;case"WEBGL_compressed_texture_s3tc":s=n.getExtension("WEBGL_compressed_texture_s3tc")||n.getExtension("MOZ_WEBGL_compressed_texture_s3tc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_s3tc");break;case"WEBGL_compressed_texture_pvrtc":s=n.getExtension("WEBGL_compressed_texture_pvrtc")||n.getExtension("WEBKIT_WEBGL_compressed_texture_pvrtc");break;default:s=n.getExtension(i)}return t[i]=s,s}return{has:function(i){return e(i)!==null},init:function(){e("EXT_color_buffer_float"),e("WEBGL_clip_cull_distance"),e("OES_texture_float_linear"),e("EXT_color_buffer_half_float"),e("WEBGL_multisampled_render_to_texture"),e("WEBGL_render_shared_exponent")},get:function(i){const s=e(i);return s===null&&ji("THREE.WebGLRenderer: "+i+" extension not supported."),s}}}function rv(n,t,e,i){const s={},r=new WeakMap;function a(h){const d=h.target;d.index!==null&&t.remove(d.index);for(const g in d.attributes)t.remove(d.attributes[g]);d.removeEventListener("dispose",a),delete s[d.id];const p=r.get(d);p&&(t.remove(p),r.delete(d)),i.releaseStatesOfGeometry(d),d.isInstancedBufferGeometry===!0&&delete d._maxInstanceCount,e.memory.geometries--}function o(h,d){return s[d.id]===!0||(d.addEventListener("dispose",a),s[d.id]=!0,e.memory.geometries++),d}function l(h){const d=h.attributes;for(const p in d)t.update(d[p],n.ARRAY_BUFFER)}function c(h){const d=[],p=h.index,g=h.attributes.position;let _=0;if(p!==null){const P=p.array;_=p.version;for(let L=0,w=P.length;L<w;L+=3){const N=P[L+0],F=P[L+1],U=P[L+2];d.push(N,F,F,U,U,N)}}else if(g!==void 0){const P=g.array;_=g.version;for(let L=0,w=P.length/3-1;L<w;L+=3){const N=L+0,F=L+1,U=L+2;d.push(N,F,F,U,U,N)}}else return;const m=new(id(d)?ld:od)(d,1);m.version=_;const f=r.get(h);f&&t.remove(f),r.set(h,m)}function u(h){const d=r.get(h);if(d){const p=h.index;p!==null&&d.version<p.version&&c(h)}else c(h);return r.get(h)}return{get:o,update:l,getWireframeAttribute:u}}function av(n,t,e){let i;function s(d){i=d}let r,a;function o(d){r=d.type,a=d.bytesPerElement}function l(d,p){n.drawElements(i,p,r,d*a),e.update(p,i,1)}function c(d,p,g){g!==0&&(n.drawElementsInstanced(i,p,r,d*a,g),e.update(p,i,g))}function u(d,p,g){if(g===0)return;t.get("WEBGL_multi_draw").multiDrawElementsWEBGL(i,p,0,r,d,0,g);let m=0;for(let f=0;f<g;f++)m+=p[f];e.update(m,i,1)}function h(d,p,g,_){if(g===0)return;const m=t.get("WEBGL_multi_draw");if(m===null)for(let f=0;f<d.length;f++)c(d[f]/a,p[f],_[f]);else{m.multiDrawElementsInstancedWEBGL(i,p,0,r,d,0,_,0,g);let f=0;for(let P=0;P<g;P++)f+=p[P]*_[P];e.update(f,i,1)}}this.setMode=s,this.setIndex=o,this.render=l,this.renderInstances=c,this.renderMultiDraw=u,this.renderMultiDrawInstances=h}function ov(n){const t={geometries:0,textures:0},e={frame:0,calls:0,triangles:0,points:0,lines:0};function i(r,a,o){switch(e.calls++,a){case n.TRIANGLES:e.triangles+=o*(r/3);break;case n.LINES:e.lines+=o*(r/2);break;case n.LINE_STRIP:e.lines+=o*(r-1);break;case n.LINE_LOOP:e.lines+=o*r;break;case n.POINTS:e.points+=o*r;break;default:console.error("THREE.WebGLInfo: Unknown draw mode:",a);break}}function s(){e.calls=0,e.triangles=0,e.points=0,e.lines=0}return{memory:t,render:e,programs:null,autoReset:!0,reset:s,update:i}}function lv(n,t,e){const i=new WeakMap,s=new Ee;function r(a,o,l){const c=a.morphTargetInfluences,u=o.morphAttributes.position||o.morphAttributes.normal||o.morphAttributes.color,h=u!==void 0?u.length:0;let d=i.get(o);if(d===void 0||d.count!==h){let M=function(){k.dispose(),i.delete(o),o.removeEventListener("dispose",M)};var p=M;d!==void 0&&d.texture.dispose();const g=o.morphAttributes.position!==void 0,_=o.morphAttributes.normal!==void 0,m=o.morphAttributes.color!==void 0,f=o.morphAttributes.position||[],P=o.morphAttributes.normal||[],L=o.morphAttributes.color||[];let w=0;g===!0&&(w=1),_===!0&&(w=2),m===!0&&(w=3);let N=o.attributes.position.count*w,F=1;N>t.maxTextureSize&&(F=Math.ceil(N/t.maxTextureSize),N=t.maxTextureSize);const U=new Float32Array(N*F*4*h),k=new sd(U,N,F,h);k.type=jn,k.needsUpdate=!0;const y=w*4;for(let I=0;I<h;I++){const V=f[I],W=P[I],Z=L[I],et=N*F*4*I;for(let z=0;z<V.count;z++){const it=z*y;g===!0&&(s.fromBufferAttribute(V,z),U[et+it+0]=s.x,U[et+it+1]=s.y,U[et+it+2]=s.z,U[et+it+3]=0),_===!0&&(s.fromBufferAttribute(W,z),U[et+it+4]=s.x,U[et+it+5]=s.y,U[et+it+6]=s.z,U[et+it+7]=0),m===!0&&(s.fromBufferAttribute(Z,z),U[et+it+8]=s.x,U[et+it+9]=s.y,U[et+it+10]=s.z,U[et+it+11]=Z.itemSize===4?s.w:1)}}d={count:h,texture:k,size:new Yt(N,F)},i.set(o,d),o.addEventListener("dispose",M)}if(a.isInstancedMesh===!0&&a.morphTexture!==null)l.getUniforms().setValue(n,"morphTexture",a.morphTexture,e);else{let g=0;for(let m=0;m<c.length;m++)g+=c[m];const _=o.morphTargetsRelative?1:1-g;l.getUniforms().setValue(n,"morphTargetBaseInfluence",_),l.getUniforms().setValue(n,"morphTargetInfluences",c)}l.getUniforms().setValue(n,"morphTargetsTexture",d.texture,e),l.getUniforms().setValue(n,"morphTargetsTextureSize",d.size)}return{update:r}}function cv(n,t,e,i){let s=new WeakMap;function r(l){const c=i.render.frame,u=l.geometry,h=t.get(l,u);if(s.get(h)!==c&&(t.update(h),s.set(h,c)),l.isInstancedMesh&&(l.hasEventListener("dispose",o)===!1&&l.addEventListener("dispose",o),s.get(l)!==c&&(e.update(l.instanceMatrix,n.ARRAY_BUFFER),l.instanceColor!==null&&e.update(l.instanceColor,n.ARRAY_BUFFER),s.set(l,c))),l.isSkinnedMesh){const d=l.skeleton;s.get(d)!==c&&(d.update(),s.set(d,c))}return h}function a(){s=new WeakMap}function o(l){const c=l.target;c.removeEventListener("dispose",o),e.remove(c.instanceMatrix),c.instanceColor!==null&&e.remove(c.instanceColor)}return{update:r,dispose:a}}const Td=new ln,qu=new gd(1,1),wd=new sd,Ad=new xm,Cd=new hd,Yu=[],Ku=[],Ju=new Float32Array(16),Zu=new Float32Array(9),Qu=new Float32Array(4);function sr(n,t,e){const i=n[0];if(i<=0||i>0)return n;const s=t*e;let r=Yu[s];if(r===void 0&&(r=new Float32Array(s),Yu[s]=r),t!==0){i.toArray(r,0);for(let a=1,o=0;a!==t;++a)o+=e,n[a].toArray(r,o)}return r}function Ye(n,t){if(n.length!==t.length)return!1;for(let e=0,i=n.length;e<i;e++)if(n[e]!==t[e])return!1;return!0}function Ke(n,t){for(let e=0,i=t.length;e<i;e++)n[e]=t[e]}function to(n,t){let e=Ku[t];e===void 0&&(e=new Int32Array(t),Ku[t]=e);for(let i=0;i!==t;++i)e[i]=n.allocateTextureUnit();return e}function uv(n,t){const e=this.cache;e[0]!==t&&(n.uniform1f(this.addr,t),e[0]=t)}function hv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2f(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Ye(e,t))return;n.uniform2fv(this.addr,t),Ke(e,t)}}function dv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3f(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else if(t.r!==void 0)(e[0]!==t.r||e[1]!==t.g||e[2]!==t.b)&&(n.uniform3f(this.addr,t.r,t.g,t.b),e[0]=t.r,e[1]=t.g,e[2]=t.b);else{if(Ye(e,t))return;n.uniform3fv(this.addr,t),Ke(e,t)}}function fv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4f(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Ye(e,t))return;n.uniform4fv(this.addr,t),Ke(e,t)}}function pv(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(Ye(e,t))return;n.uniformMatrix2fv(this.addr,!1,t),Ke(e,t)}else{if(Ye(e,i))return;Qu.set(i),n.uniformMatrix2fv(this.addr,!1,Qu),Ke(e,i)}}function mv(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(Ye(e,t))return;n.uniformMatrix3fv(this.addr,!1,t),Ke(e,t)}else{if(Ye(e,i))return;Zu.set(i),n.uniformMatrix3fv(this.addr,!1,Zu),Ke(e,i)}}function gv(n,t){const e=this.cache,i=t.elements;if(i===void 0){if(Ye(e,t))return;n.uniformMatrix4fv(this.addr,!1,t),Ke(e,t)}else{if(Ye(e,i))return;Ju.set(i),n.uniformMatrix4fv(this.addr,!1,Ju),Ke(e,i)}}function _v(n,t){const e=this.cache;e[0]!==t&&(n.uniform1i(this.addr,t),e[0]=t)}function vv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2i(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Ye(e,t))return;n.uniform2iv(this.addr,t),Ke(e,t)}}function xv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3i(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Ye(e,t))return;n.uniform3iv(this.addr,t),Ke(e,t)}}function bv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4i(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Ye(e,t))return;n.uniform4iv(this.addr,t),Ke(e,t)}}function yv(n,t){const e=this.cache;e[0]!==t&&(n.uniform1ui(this.addr,t),e[0]=t)}function Mv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y)&&(n.uniform2ui(this.addr,t.x,t.y),e[0]=t.x,e[1]=t.y);else{if(Ye(e,t))return;n.uniform2uiv(this.addr,t),Ke(e,t)}}function Sv(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z)&&(n.uniform3ui(this.addr,t.x,t.y,t.z),e[0]=t.x,e[1]=t.y,e[2]=t.z);else{if(Ye(e,t))return;n.uniform3uiv(this.addr,t),Ke(e,t)}}function Ev(n,t){const e=this.cache;if(t.x!==void 0)(e[0]!==t.x||e[1]!==t.y||e[2]!==t.z||e[3]!==t.w)&&(n.uniform4ui(this.addr,t.x,t.y,t.z,t.w),e[0]=t.x,e[1]=t.y,e[2]=t.z,e[3]=t.w);else{if(Ye(e,t))return;n.uniform4uiv(this.addr,t),Ke(e,t)}}function Tv(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s);let r;this.type===n.SAMPLER_2D_SHADOW?(qu.compareFunction=nd,r=qu):r=Td,e.setTexture2D(t||r,s)}function wv(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTexture3D(t||Ad,s)}function Av(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTextureCube(t||Cd,s)}function Cv(n,t,e){const i=this.cache,s=e.allocateTextureUnit();i[0]!==s&&(n.uniform1i(this.addr,s),i[0]=s),e.setTexture2DArray(t||wd,s)}function Rv(n){switch(n){case 5126:return uv;case 35664:return hv;case 35665:return dv;case 35666:return fv;case 35674:return pv;case 35675:return mv;case 35676:return gv;case 5124:case 35670:return _v;case 35667:case 35671:return vv;case 35668:case 35672:return xv;case 35669:case 35673:return bv;case 5125:return yv;case 36294:return Mv;case 36295:return Sv;case 36296:return Ev;case 35678:case 36198:case 36298:case 36306:case 35682:return Tv;case 35679:case 36299:case 36307:return wv;case 35680:case 36300:case 36308:case 36293:return Av;case 36289:case 36303:case 36311:case 36292:return Cv}}function Pv(n,t){n.uniform1fv(this.addr,t)}function Lv(n,t){const e=sr(t,this.size,2);n.uniform2fv(this.addr,e)}function Nv(n,t){const e=sr(t,this.size,3);n.uniform3fv(this.addr,e)}function Dv(n,t){const e=sr(t,this.size,4);n.uniform4fv(this.addr,e)}function Iv(n,t){const e=sr(t,this.size,4);n.uniformMatrix2fv(this.addr,!1,e)}function Uv(n,t){const e=sr(t,this.size,9);n.uniformMatrix3fv(this.addr,!1,e)}function kv(n,t){const e=sr(t,this.size,16);n.uniformMatrix4fv(this.addr,!1,e)}function Fv(n,t){n.uniform1iv(this.addr,t)}function Ov(n,t){n.uniform2iv(this.addr,t)}function Bv(n,t){n.uniform3iv(this.addr,t)}function zv(n,t){n.uniform4iv(this.addr,t)}function Vv(n,t){n.uniform1uiv(this.addr,t)}function Hv(n,t){n.uniform2uiv(this.addr,t)}function Gv(n,t){n.uniform3uiv(this.addr,t)}function Wv(n,t){n.uniform4uiv(this.addr,t)}function $v(n,t,e){const i=this.cache,s=t.length,r=to(e,s);Ye(i,r)||(n.uniform1iv(this.addr,r),Ke(i,r));for(let a=0;a!==s;++a)e.setTexture2D(t[a]||Td,r[a])}function jv(n,t,e){const i=this.cache,s=t.length,r=to(e,s);Ye(i,r)||(n.uniform1iv(this.addr,r),Ke(i,r));for(let a=0;a!==s;++a)e.setTexture3D(t[a]||Ad,r[a])}function Xv(n,t,e){const i=this.cache,s=t.length,r=to(e,s);Ye(i,r)||(n.uniform1iv(this.addr,r),Ke(i,r));for(let a=0;a!==s;++a)e.setTextureCube(t[a]||Cd,r[a])}function qv(n,t,e){const i=this.cache,s=t.length,r=to(e,s);Ye(i,r)||(n.uniform1iv(this.addr,r),Ke(i,r));for(let a=0;a!==s;++a)e.setTexture2DArray(t[a]||wd,r[a])}function Yv(n){switch(n){case 5126:return Pv;case 35664:return Lv;case 35665:return Nv;case 35666:return Dv;case 35674:return Iv;case 35675:return Uv;case 35676:return kv;case 5124:case 35670:return Fv;case 35667:case 35671:return Ov;case 35668:case 35672:return Bv;case 35669:case 35673:return zv;case 5125:return Vv;case 36294:return Hv;case 36295:return Gv;case 36296:return Wv;case 35678:case 36198:case 36298:case 36306:case 35682:return $v;case 35679:case 36299:case 36307:return jv;case 35680:case 36300:case 36308:case 36293:return Xv;case 36289:case 36303:case 36311:case 36292:return qv}}class Kv{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.setValue=Rv(e.type)}}class Jv{constructor(t,e,i){this.id=t,this.addr=i,this.cache=[],this.type=e.type,this.size=e.size,this.setValue=Yv(e.type)}}class Zv{constructor(t){this.id=t,this.seq=[],this.map={}}setValue(t,e,i){const s=this.seq;for(let r=0,a=s.length;r!==a;++r){const o=s[r];o.setValue(t,e[o.id],i)}}}const Ko=/(\w+)(\])?(\[|\.)?/g;function th(n,t){n.seq.push(t),n.map[t.id]=t}function Qv(n,t,e){const i=n.name,s=i.length;for(Ko.lastIndex=0;;){const r=Ko.exec(i),a=Ko.lastIndex;let o=r[1];const l=r[2]==="]",c=r[3];if(l&&(o=o|0),c===void 0||c==="["&&a+2===s){th(e,c===void 0?new Kv(o,n,t):new Jv(o,n,t));break}else{let h=e.map[o];h===void 0&&(h=new Zv(o),th(e,h)),e=h}}}class Da{constructor(t,e){this.seq=[],this.map={};const i=t.getProgramParameter(e,t.ACTIVE_UNIFORMS);for(let s=0;s<i;++s){const r=t.getActiveUniform(e,s),a=t.getUniformLocation(e,r.name);Qv(r,a,this)}}setValue(t,e,i,s){const r=this.map[e];r!==void 0&&r.setValue(t,i,s)}setOptional(t,e,i){const s=e[i];s!==void 0&&this.setValue(t,i,s)}static upload(t,e,i,s){for(let r=0,a=e.length;r!==a;++r){const o=e[r],l=i[o.id];l.needsUpdate!==!1&&o.setValue(t,l.value,s)}}static seqWithValue(t,e){const i=[];for(let s=0,r=t.length;s!==r;++s){const a=t[s];a.id in e&&i.push(a)}return i}}function eh(n,t,e){const i=n.createShader(t);return n.shaderSource(i,e),n.compileShader(i),i}const tx=37297;let ex=0;function nx(n,t){const e=n.split(`
`),i=[],s=Math.max(t-6,0),r=Math.min(t+6,e.length);for(let a=s;a<r;a++){const o=a+1;i.push(`${o===t?">":" "} ${o}: ${e[a]}`)}return i.join(`
`)}const nh=new le;function ix(n){_e._getMatrix(nh,_e.workingColorSpace,n);const t=`mat3( ${nh.elements.map(e=>e.toFixed(4))} )`;switch(_e.getTransfer(n)){case Fa:return[t,"LinearTransferOETF"];case Ce:return[t,"sRGBTransferOETF"];default:return console.warn("THREE.WebGLProgram: Unsupported color space: ",n),[t,"LinearTransferOETF"]}}function ih(n,t,e){const i=n.getShaderParameter(t,n.COMPILE_STATUS),s=n.getShaderInfoLog(t).trim();if(i&&s==="")return"";const r=/ERROR: 0:(\d+)/.exec(s);if(r){const a=parseInt(r[1]);return e.toUpperCase()+`

`+s+`

`+nx(n.getShaderSource(t),a)}else return s}function sx(n,t){const e=ix(t);return[`vec4 ${n}( vec4 value ) {`,`	return ${e[1]}( vec4( value.rgb * ${e[0]}, value.a ) );`,"}"].join(`
`)}function rx(n,t){let e;switch(t){case Rp:e="Linear";break;case Pp:e="Reinhard";break;case Lp:e="Cineon";break;case Wh:e="ACESFilmic";break;case Dp:e="AgX";break;case Ip:e="Neutral";break;case Np:e="Custom";break;default:console.warn("THREE.WebGLProgram: Unsupported toneMapping:",t),e="Linear"}return"vec3 "+n+"( vec3 color ) { return "+e+"ToneMapping( color ); }"}const xa=new O;function ax(){_e.getLuminanceCoefficients(xa);const n=xa.x.toFixed(4),t=xa.y.toFixed(4),e=xa.z.toFixed(4);return["float luminance( const in vec3 rgb ) {",`	const vec3 weights = vec3( ${n}, ${t}, ${e} );`,"	return dot( weights, rgb );","}"].join(`
`)}function ox(n){return[n.extensionClipCullDistance?"#extension GL_ANGLE_clip_cull_distance : require":"",n.extensionMultiDraw?"#extension GL_ANGLE_multi_draw : require":""].filter(vr).join(`
`)}function lx(n){const t=[];for(const e in n){const i=n[e];i!==!1&&t.push("#define "+e+" "+i)}return t.join(`
`)}function cx(n,t){const e={},i=n.getProgramParameter(t,n.ACTIVE_ATTRIBUTES);for(let s=0;s<i;s++){const r=n.getActiveAttrib(t,s),a=r.name;let o=1;r.type===n.FLOAT_MAT2&&(o=2),r.type===n.FLOAT_MAT3&&(o=3),r.type===n.FLOAT_MAT4&&(o=4),e[a]={type:r.type,location:n.getAttribLocation(t,a),locationSize:o}}return e}function vr(n){return n!==""}function sh(n,t){const e=t.numSpotLightShadows+t.numSpotLightMaps-t.numSpotLightShadowsWithMaps;return n.replace(/NUM_DIR_LIGHTS/g,t.numDirLights).replace(/NUM_SPOT_LIGHTS/g,t.numSpotLights).replace(/NUM_SPOT_LIGHT_MAPS/g,t.numSpotLightMaps).replace(/NUM_SPOT_LIGHT_COORDS/g,e).replace(/NUM_RECT_AREA_LIGHTS/g,t.numRectAreaLights).replace(/NUM_POINT_LIGHTS/g,t.numPointLights).replace(/NUM_HEMI_LIGHTS/g,t.numHemiLights).replace(/NUM_DIR_LIGHT_SHADOWS/g,t.numDirLightShadows).replace(/NUM_SPOT_LIGHT_SHADOWS_WITH_MAPS/g,t.numSpotLightShadowsWithMaps).replace(/NUM_SPOT_LIGHT_SHADOWS/g,t.numSpotLightShadows).replace(/NUM_POINT_LIGHT_SHADOWS/g,t.numPointLightShadows)}function rh(n,t){return n.replace(/NUM_CLIPPING_PLANES/g,t.numClippingPlanes).replace(/UNION_CLIPPING_PLANES/g,t.numClippingPlanes-t.numClipIntersection)}const ux=/^[ \t]*#include +<([\w\d./]+)>/gm;function Wl(n){return n.replace(ux,dx)}const hx=new Map;function dx(n,t){let e=ue[t];if(e===void 0){const i=hx.get(t);if(i!==void 0)e=ue[i],console.warn('THREE.WebGLRenderer: Shader chunk "%s" has been deprecated. Use "%s" instead.',t,i);else throw new Error("Can not resolve #include <"+t+">")}return Wl(e)}const fx=/#pragma unroll_loop_start\s+for\s*\(\s*int\s+i\s*=\s*(\d+)\s*;\s*i\s*<\s*(\d+)\s*;\s*i\s*\+\+\s*\)\s*{([\s\S]+?)}\s+#pragma unroll_loop_end/g;function ah(n){return n.replace(fx,px)}function px(n,t,e,i){let s="";for(let r=parseInt(t);r<parseInt(e);r++)s+=i.replace(/\[\s*i\s*\]/g,"[ "+r+" ]").replace(/UNROLLED_LOOP_INDEX/g,r);return s}function oh(n){let t=`precision ${n.precision} float;
	precision ${n.precision} int;
	precision ${n.precision} sampler2D;
	precision ${n.precision} samplerCube;
	precision ${n.precision} sampler3D;
	precision ${n.precision} sampler2DArray;
	precision ${n.precision} sampler2DShadow;
	precision ${n.precision} samplerCubeShadow;
	precision ${n.precision} sampler2DArrayShadow;
	precision ${n.precision} isampler2D;
	precision ${n.precision} isampler3D;
	precision ${n.precision} isamplerCube;
	precision ${n.precision} isampler2DArray;
	precision ${n.precision} usampler2D;
	precision ${n.precision} usampler3D;
	precision ${n.precision} usamplerCube;
	precision ${n.precision} usampler2DArray;
	`;return n.precision==="highp"?t+=`
#define HIGH_PRECISION`:n.precision==="mediump"?t+=`
#define MEDIUM_PRECISION`:n.precision==="lowp"&&(t+=`
#define LOW_PRECISION`),t}function mx(n){let t="SHADOWMAP_TYPE_BASIC";return n.shadowMapType===Hh?t="SHADOWMAP_TYPE_PCF":n.shadowMapType===Gh?t="SHADOWMAP_TYPE_PCF_SOFT":n.shadowMapType===ri&&(t="SHADOWMAP_TYPE_VSM"),t}function gx(n){let t="ENVMAP_TYPE_CUBE";if(n.envMap)switch(n.envMapMode){case $s:case js:t="ENVMAP_TYPE_CUBE";break;case Ka:t="ENVMAP_TYPE_CUBE_UV";break}return t}function _x(n){let t="ENVMAP_MODE_REFLECTION";if(n.envMap)switch(n.envMapMode){case js:t="ENVMAP_MODE_REFRACTION";break}return t}function vx(n){let t="ENVMAP_BLENDING_NONE";if(n.envMap)switch(n.combine){case Ya:t="ENVMAP_BLENDING_MULTIPLY";break;case Ap:t="ENVMAP_BLENDING_MIX";break;case Cp:t="ENVMAP_BLENDING_ADD";break}return t}function xx(n){const t=n.envMapCubeUVHeight;if(t===null)return null;const e=Math.log2(t)-2,i=1/t;return{texelWidth:1/(3*Math.max(Math.pow(2,e),112)),texelHeight:i,maxMip:e}}function bx(n,t,e,i){const s=n.getContext(),r=e.defines;let a=e.vertexShader,o=e.fragmentShader;const l=mx(e),c=gx(e),u=_x(e),h=vx(e),d=xx(e),p=ox(e),g=lx(r),_=s.createProgram();let m,f,P=e.glslVersion?"#version "+e.glslVersion+`
`:"";e.isRawShaderMaterial?(m=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(vr).join(`
`),m.length>0&&(m+=`
`),f=["#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g].filter(vr).join(`
`),f.length>0&&(f+=`
`)):(m=[oh(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.extensionClipCullDistance?"#define USE_CLIP_DISTANCE":"",e.batching?"#define USE_BATCHING":"",e.batchingColor?"#define USE_BATCHING_COLOR":"",e.instancing?"#define USE_INSTANCING":"",e.instancingColor?"#define USE_INSTANCING_COLOR":"",e.instancingMorph?"#define USE_INSTANCING_MORPH":"",e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.map?"#define USE_MAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+u:"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.displacementMap?"#define USE_DISPLACEMENTMAP":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.mapUv?"#define MAP_UV "+e.mapUv:"",e.alphaMapUv?"#define ALPHAMAP_UV "+e.alphaMapUv:"",e.lightMapUv?"#define LIGHTMAP_UV "+e.lightMapUv:"",e.aoMapUv?"#define AOMAP_UV "+e.aoMapUv:"",e.emissiveMapUv?"#define EMISSIVEMAP_UV "+e.emissiveMapUv:"",e.bumpMapUv?"#define BUMPMAP_UV "+e.bumpMapUv:"",e.normalMapUv?"#define NORMALMAP_UV "+e.normalMapUv:"",e.displacementMapUv?"#define DISPLACEMENTMAP_UV "+e.displacementMapUv:"",e.metalnessMapUv?"#define METALNESSMAP_UV "+e.metalnessMapUv:"",e.roughnessMapUv?"#define ROUGHNESSMAP_UV "+e.roughnessMapUv:"",e.anisotropyMapUv?"#define ANISOTROPYMAP_UV "+e.anisotropyMapUv:"",e.clearcoatMapUv?"#define CLEARCOATMAP_UV "+e.clearcoatMapUv:"",e.clearcoatNormalMapUv?"#define CLEARCOAT_NORMALMAP_UV "+e.clearcoatNormalMapUv:"",e.clearcoatRoughnessMapUv?"#define CLEARCOAT_ROUGHNESSMAP_UV "+e.clearcoatRoughnessMapUv:"",e.iridescenceMapUv?"#define IRIDESCENCEMAP_UV "+e.iridescenceMapUv:"",e.iridescenceThicknessMapUv?"#define IRIDESCENCE_THICKNESSMAP_UV "+e.iridescenceThicknessMapUv:"",e.sheenColorMapUv?"#define SHEEN_COLORMAP_UV "+e.sheenColorMapUv:"",e.sheenRoughnessMapUv?"#define SHEEN_ROUGHNESSMAP_UV "+e.sheenRoughnessMapUv:"",e.specularMapUv?"#define SPECULARMAP_UV "+e.specularMapUv:"",e.specularColorMapUv?"#define SPECULAR_COLORMAP_UV "+e.specularColorMapUv:"",e.specularIntensityMapUv?"#define SPECULAR_INTENSITYMAP_UV "+e.specularIntensityMapUv:"",e.transmissionMapUv?"#define TRANSMISSIONMAP_UV "+e.transmissionMapUv:"",e.thicknessMapUv?"#define THICKNESSMAP_UV "+e.thicknessMapUv:"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.flatShading?"#define FLAT_SHADED":"",e.skinning?"#define USE_SKINNING":"",e.morphTargets?"#define USE_MORPHTARGETS":"",e.morphNormals&&e.flatShading===!1?"#define USE_MORPHNORMALS":"",e.morphColors?"#define USE_MORPHCOLORS":"",e.morphTargetsCount>0?"#define MORPHTARGETS_TEXTURE_STRIDE "+e.morphTextureStride:"",e.morphTargetsCount>0?"#define MORPHTARGETS_COUNT "+e.morphTargetsCount:"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+l:"",e.sizeAttenuation?"#define USE_SIZEATTENUATION":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.reverseDepthBuffer?"#define USE_REVERSEDEPTHBUF":"","uniform mat4 modelMatrix;","uniform mat4 modelViewMatrix;","uniform mat4 projectionMatrix;","uniform mat4 viewMatrix;","uniform mat3 normalMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;","#ifdef USE_INSTANCING","	attribute mat4 instanceMatrix;","#endif","#ifdef USE_INSTANCING_COLOR","	attribute vec3 instanceColor;","#endif","#ifdef USE_INSTANCING_MORPH","	uniform sampler2D morphTexture;","#endif","attribute vec3 position;","attribute vec3 normal;","attribute vec2 uv;","#ifdef USE_UV1","	attribute vec2 uv1;","#endif","#ifdef USE_UV2","	attribute vec2 uv2;","#endif","#ifdef USE_UV3","	attribute vec2 uv3;","#endif","#ifdef USE_TANGENT","	attribute vec4 tangent;","#endif","#if defined( USE_COLOR_ALPHA )","	attribute vec4 color;","#elif defined( USE_COLOR )","	attribute vec3 color;","#endif","#ifdef USE_SKINNING","	attribute vec4 skinIndex;","	attribute vec4 skinWeight;","#endif",`
`].filter(vr).join(`
`),f=[oh(e),"#define SHADER_TYPE "+e.shaderType,"#define SHADER_NAME "+e.shaderName,g,e.useFog&&e.fog?"#define USE_FOG":"",e.useFog&&e.fogExp2?"#define FOG_EXP2":"",e.alphaToCoverage?"#define ALPHA_TO_COVERAGE":"",e.map?"#define USE_MAP":"",e.matcap?"#define USE_MATCAP":"",e.envMap?"#define USE_ENVMAP":"",e.envMap?"#define "+c:"",e.envMap?"#define "+u:"",e.envMap?"#define "+h:"",d?"#define CUBEUV_TEXEL_WIDTH "+d.texelWidth:"",d?"#define CUBEUV_TEXEL_HEIGHT "+d.texelHeight:"",d?"#define CUBEUV_MAX_MIP "+d.maxMip+".0":"",e.lightMap?"#define USE_LIGHTMAP":"",e.aoMap?"#define USE_AOMAP":"",e.bumpMap?"#define USE_BUMPMAP":"",e.normalMap?"#define USE_NORMALMAP":"",e.normalMapObjectSpace?"#define USE_NORMALMAP_OBJECTSPACE":"",e.normalMapTangentSpace?"#define USE_NORMALMAP_TANGENTSPACE":"",e.emissiveMap?"#define USE_EMISSIVEMAP":"",e.anisotropy?"#define USE_ANISOTROPY":"",e.anisotropyMap?"#define USE_ANISOTROPYMAP":"",e.clearcoat?"#define USE_CLEARCOAT":"",e.clearcoatMap?"#define USE_CLEARCOATMAP":"",e.clearcoatRoughnessMap?"#define USE_CLEARCOAT_ROUGHNESSMAP":"",e.clearcoatNormalMap?"#define USE_CLEARCOAT_NORMALMAP":"",e.dispersion?"#define USE_DISPERSION":"",e.iridescence?"#define USE_IRIDESCENCE":"",e.iridescenceMap?"#define USE_IRIDESCENCEMAP":"",e.iridescenceThicknessMap?"#define USE_IRIDESCENCE_THICKNESSMAP":"",e.specularMap?"#define USE_SPECULARMAP":"",e.specularColorMap?"#define USE_SPECULAR_COLORMAP":"",e.specularIntensityMap?"#define USE_SPECULAR_INTENSITYMAP":"",e.roughnessMap?"#define USE_ROUGHNESSMAP":"",e.metalnessMap?"#define USE_METALNESSMAP":"",e.alphaMap?"#define USE_ALPHAMAP":"",e.alphaTest?"#define USE_ALPHATEST":"",e.alphaHash?"#define USE_ALPHAHASH":"",e.sheen?"#define USE_SHEEN":"",e.sheenColorMap?"#define USE_SHEEN_COLORMAP":"",e.sheenRoughnessMap?"#define USE_SHEEN_ROUGHNESSMAP":"",e.transmission?"#define USE_TRANSMISSION":"",e.transmissionMap?"#define USE_TRANSMISSIONMAP":"",e.thicknessMap?"#define USE_THICKNESSMAP":"",e.vertexTangents&&e.flatShading===!1?"#define USE_TANGENT":"",e.vertexColors||e.instancingColor||e.batchingColor?"#define USE_COLOR":"",e.vertexAlphas?"#define USE_COLOR_ALPHA":"",e.vertexUv1s?"#define USE_UV1":"",e.vertexUv2s?"#define USE_UV2":"",e.vertexUv3s?"#define USE_UV3":"",e.pointsUvs?"#define USE_POINTS_UV":"",e.gradientMap?"#define USE_GRADIENTMAP":"",e.flatShading?"#define FLAT_SHADED":"",e.doubleSided?"#define DOUBLE_SIDED":"",e.flipSided?"#define FLIP_SIDED":"",e.shadowMapEnabled?"#define USE_SHADOWMAP":"",e.shadowMapEnabled?"#define "+l:"",e.premultipliedAlpha?"#define PREMULTIPLIED_ALPHA":"",e.numLightProbes>0?"#define USE_LIGHT_PROBES":"",e.decodeVideoTexture?"#define DECODE_VIDEO_TEXTURE":"",e.decodeVideoTextureEmissive?"#define DECODE_VIDEO_TEXTURE_EMISSIVE":"",e.logarithmicDepthBuffer?"#define USE_LOGDEPTHBUF":"",e.reverseDepthBuffer?"#define USE_REVERSEDEPTHBUF":"","uniform mat4 viewMatrix;","uniform vec3 cameraPosition;","uniform bool isOrthographic;",e.toneMapping!==Pi?"#define TONE_MAPPING":"",e.toneMapping!==Pi?ue.tonemapping_pars_fragment:"",e.toneMapping!==Pi?rx("toneMapping",e.toneMapping):"",e.dithering?"#define DITHERING":"",e.opaque?"#define OPAQUE":"",ue.colorspace_pars_fragment,sx("linearToOutputTexel",e.outputColorSpace),ax(),e.useDepthPacking?"#define DEPTH_PACKING "+e.depthPacking:"",`
`].filter(vr).join(`
`)),a=Wl(a),a=sh(a,e),a=rh(a,e),o=Wl(o),o=sh(o,e),o=rh(o,e),a=ah(a),o=ah(o),e.isRawShaderMaterial!==!0&&(P=`#version 300 es
`,m=[p,"#define attribute in","#define varying out","#define texture2D texture"].join(`
`)+`
`+m,f=["#define varying in",e.glslVersion===eu?"":"layout(location = 0) out highp vec4 pc_fragColor;",e.glslVersion===eu?"":"#define gl_FragColor pc_fragColor","#define gl_FragDepthEXT gl_FragDepth","#define texture2D texture","#define textureCube texture","#define texture2DProj textureProj","#define texture2DLodEXT textureLod","#define texture2DProjLodEXT textureProjLod","#define textureCubeLodEXT textureLod","#define texture2DGradEXT textureGrad","#define texture2DProjGradEXT textureProjGrad","#define textureCubeGradEXT textureGrad"].join(`
`)+`
`+f);const L=P+m+a,w=P+f+o,N=eh(s,s.VERTEX_SHADER,L),F=eh(s,s.FRAGMENT_SHADER,w);s.attachShader(_,N),s.attachShader(_,F),e.index0AttributeName!==void 0?s.bindAttribLocation(_,0,e.index0AttributeName):e.morphTargets===!0&&s.bindAttribLocation(_,0,"position"),s.linkProgram(_);function U(I){if(n.debug.checkShaderErrors){const V=s.getProgramInfoLog(_).trim(),W=s.getShaderInfoLog(N).trim(),Z=s.getShaderInfoLog(F).trim();let et=!0,z=!0;if(s.getProgramParameter(_,s.LINK_STATUS)===!1)if(et=!1,typeof n.debug.onShaderError=="function")n.debug.onShaderError(s,_,N,F);else{const it=ih(s,N,"vertex"),j=ih(s,F,"fragment");console.error("THREE.WebGLProgram: Shader Error "+s.getError()+" - VALIDATE_STATUS "+s.getProgramParameter(_,s.VALIDATE_STATUS)+`

Material Name: `+I.name+`
Material Type: `+I.type+`

Program Info Log: `+V+`
`+it+`
`+j)}else V!==""?console.warn("THREE.WebGLProgram: Program Info Log:",V):(W===""||Z==="")&&(z=!1);z&&(I.diagnostics={runnable:et,programLog:V,vertexShader:{log:W,prefix:m},fragmentShader:{log:Z,prefix:f}})}s.deleteShader(N),s.deleteShader(F),k=new Da(s,_),y=cx(s,_)}let k;this.getUniforms=function(){return k===void 0&&U(this),k};let y;this.getAttributes=function(){return y===void 0&&U(this),y};let M=e.rendererExtensionParallelShaderCompile===!1;return this.isReady=function(){return M===!1&&(M=s.getProgramParameter(_,tx)),M},this.destroy=function(){i.releaseStatesOfProgram(this),s.deleteProgram(_),this.program=void 0},this.type=e.shaderType,this.name=e.shaderName,this.id=ex++,this.cacheKey=t,this.usedTimes=1,this.program=_,this.vertexShader=N,this.fragmentShader=F,this}let yx=0;class Mx{constructor(){this.shaderCache=new Map,this.materialCache=new Map}update(t){const e=t.vertexShader,i=t.fragmentShader,s=this._getShaderStage(e),r=this._getShaderStage(i),a=this._getShaderCacheForMaterial(t);return a.has(s)===!1&&(a.add(s),s.usedTimes++),a.has(r)===!1&&(a.add(r),r.usedTimes++),this}remove(t){const e=this.materialCache.get(t);for(const i of e)i.usedTimes--,i.usedTimes===0&&this.shaderCache.delete(i.code);return this.materialCache.delete(t),this}getVertexShaderID(t){return this._getShaderStage(t.vertexShader).id}getFragmentShaderID(t){return this._getShaderStage(t.fragmentShader).id}dispose(){this.shaderCache.clear(),this.materialCache.clear()}_getShaderCacheForMaterial(t){const e=this.materialCache;let i=e.get(t);return i===void 0&&(i=new Set,e.set(t,i)),i}_getShaderStage(t){const e=this.shaderCache;let i=e.get(t);return i===void 0&&(i=new Sx(t),e.set(t,i)),i}}class Sx{constructor(t){this.id=yx++,this.code=t,this.usedTimes=0}}function Ex(n,t,e,i,s,r,a){const o=new rd,l=new Mx,c=new Set,u=[],h=s.logarithmicDepthBuffer,d=s.vertexTextures;let p=s.precision;const g={MeshDepthMaterial:"depth",MeshDistanceMaterial:"distanceRGBA",MeshNormalMaterial:"normal",MeshBasicMaterial:"basic",MeshLambertMaterial:"lambert",MeshPhongMaterial:"phong",MeshToonMaterial:"toon",MeshStandardMaterial:"physical",MeshPhysicalMaterial:"physical",MeshMatcapMaterial:"matcap",LineBasicMaterial:"basic",LineDashedMaterial:"dashed",PointsMaterial:"points",ShadowMaterial:"shadow",SpriteMaterial:"sprite"};function _(y){return c.add(y),y===0?"uv":`uv${y}`}function m(y,M,I,V,W){const Z=V.fog,et=W.geometry,z=y.isMeshStandardMaterial?V.environment:null,it=(y.isMeshStandardMaterial?e:t).get(y.envMap||z),j=it&&it.mapping===Ka?it.image.height:null,ut=g[y.type];y.precision!==null&&(p=s.getMaxPrecision(y.precision),p!==y.precision&&console.warn("THREE.WebGLProgram.getParameters:",y.precision,"not supported, using",p,"instead."));const mt=et.morphAttributes.position||et.morphAttributes.normal||et.morphAttributes.color,gt=mt!==void 0?mt.length:0;let Lt=0;et.morphAttributes.position!==void 0&&(Lt=1),et.morphAttributes.normal!==void 0&&(Lt=2),et.morphAttributes.color!==void 0&&(Lt=3);let At,X,Q,nt;if(ut){const oe=Gn[ut];At=oe.vertexShader,X=oe.fragmentShader}else At=y.vertexShader,X=y.fragmentShader,l.update(y),Q=l.getVertexShaderID(y),nt=l.getFragmentShaderID(y);const st=n.getRenderTarget(),vt=n.state.buffers.depth.getReversed(),se=W.isInstancedMesh===!0,Wt=W.isBatchedMesh===!0,Fe=!!y.map,Ie=!!y.matcap,me=!!it,B=!!y.aoMap,pn=!!y.lightMap,fe=!!y.bumpMap,pe=!!y.normalMap,Vt=!!y.displacementMap,Le=!!y.emissiveMap,zt=!!y.metalnessMap,D=!!y.roughnessMap,S=y.anisotropy>0,q=y.clearcoat>0,lt=y.dispersion>0,ht=y.iridescence>0,ot=y.sheen>0,Ft=y.transmission>0,St=S&&!!y.anisotropyMap,Pt=q&&!!y.clearcoatMap,ve=q&&!!y.clearcoatNormalMap,pt=q&&!!y.clearcoatRoughnessMap,Nt=ht&&!!y.iridescenceMap,Xt=ht&&!!y.iridescenceThicknessMap,Zt=ot&&!!y.sheenColorMap,Dt=ot&&!!y.sheenRoughnessMap,ge=!!y.specularMap,ae=!!y.specularColorMap,Te=!!y.specularIntensityMap,G=Ft&&!!y.transmissionMap,Mt=Ft&&!!y.thicknessMap,tt=!!y.gradientMap,ct=!!y.alphaMap,wt=y.alphaTest>0,Et=!!y.alphaHash,re=!!y.extensions;let Ue=Pi;y.toneMapped&&(st===null||st.isXRRenderTarget===!0)&&(Ue=n.toneMapping);const Je={shaderID:ut,shaderType:y.type,shaderName:y.name,vertexShader:At,fragmentShader:X,defines:y.defines,customVertexShaderID:Q,customFragmentShaderID:nt,isRawShaderMaterial:y.isRawShaderMaterial===!0,glslVersion:y.glslVersion,precision:p,batching:Wt,batchingColor:Wt&&W._colorsTexture!==null,instancing:se,instancingColor:se&&W.instanceColor!==null,instancingMorph:se&&W.morphTexture!==null,supportsVertexTextures:d,outputColorSpace:st===null?n.outputColorSpace:st.isXRRenderTarget===!0?st.texture.colorSpace:Ys,alphaToCoverage:!!y.alphaToCoverage,map:Fe,matcap:Ie,envMap:me,envMapMode:me&&it.mapping,envMapCubeUVHeight:j,aoMap:B,lightMap:pn,bumpMap:fe,normalMap:pe,displacementMap:d&&Vt,emissiveMap:Le,normalMapObjectSpace:pe&&y.normalMapType===zp,normalMapTangentSpace:pe&&y.normalMapType===Ja,metalnessMap:zt,roughnessMap:D,anisotropy:S,anisotropyMap:St,clearcoat:q,clearcoatMap:Pt,clearcoatNormalMap:ve,clearcoatRoughnessMap:pt,dispersion:lt,iridescence:ht,iridescenceMap:Nt,iridescenceThicknessMap:Xt,sheen:ot,sheenColorMap:Zt,sheenRoughnessMap:Dt,specularMap:ge,specularColorMap:ae,specularIntensityMap:Te,transmission:Ft,transmissionMap:G,thicknessMap:Mt,gradientMap:tt,opaque:y.transparent===!1&&y.blending===Fs&&y.alphaToCoverage===!1,alphaMap:ct,alphaTest:wt,alphaHash:Et,combine:y.combine,mapUv:Fe&&_(y.map.channel),aoMapUv:B&&_(y.aoMap.channel),lightMapUv:pn&&_(y.lightMap.channel),bumpMapUv:fe&&_(y.bumpMap.channel),normalMapUv:pe&&_(y.normalMap.channel),displacementMapUv:Vt&&_(y.displacementMap.channel),emissiveMapUv:Le&&_(y.emissiveMap.channel),metalnessMapUv:zt&&_(y.metalnessMap.channel),roughnessMapUv:D&&_(y.roughnessMap.channel),anisotropyMapUv:St&&_(y.anisotropyMap.channel),clearcoatMapUv:Pt&&_(y.clearcoatMap.channel),clearcoatNormalMapUv:ve&&_(y.clearcoatNormalMap.channel),clearcoatRoughnessMapUv:pt&&_(y.clearcoatRoughnessMap.channel),iridescenceMapUv:Nt&&_(y.iridescenceMap.channel),iridescenceThicknessMapUv:Xt&&_(y.iridescenceThicknessMap.channel),sheenColorMapUv:Zt&&_(y.sheenColorMap.channel),sheenRoughnessMapUv:Dt&&_(y.sheenRoughnessMap.channel),specularMapUv:ge&&_(y.specularMap.channel),specularColorMapUv:ae&&_(y.specularColorMap.channel),specularIntensityMapUv:Te&&_(y.specularIntensityMap.channel),transmissionMapUv:G&&_(y.transmissionMap.channel),thicknessMapUv:Mt&&_(y.thicknessMap.channel),alphaMapUv:ct&&_(y.alphaMap.channel),vertexTangents:!!et.attributes.tangent&&(pe||S),vertexColors:y.vertexColors,vertexAlphas:y.vertexColors===!0&&!!et.attributes.color&&et.attributes.color.itemSize===4,pointsUvs:W.isPoints===!0&&!!et.attributes.uv&&(Fe||ct),fog:!!Z,useFog:y.fog===!0,fogExp2:!!Z&&Z.isFogExp2,flatShading:y.flatShading===!0,sizeAttenuation:y.sizeAttenuation===!0,logarithmicDepthBuffer:h,reverseDepthBuffer:vt,skinning:W.isSkinnedMesh===!0,morphTargets:et.morphAttributes.position!==void 0,morphNormals:et.morphAttributes.normal!==void 0,morphColors:et.morphAttributes.color!==void 0,morphTargetsCount:gt,morphTextureStride:Lt,numDirLights:M.directional.length,numPointLights:M.point.length,numSpotLights:M.spot.length,numSpotLightMaps:M.spotLightMap.length,numRectAreaLights:M.rectArea.length,numHemiLights:M.hemi.length,numDirLightShadows:M.directionalShadowMap.length,numPointLightShadows:M.pointShadowMap.length,numSpotLightShadows:M.spotShadowMap.length,numSpotLightShadowsWithMaps:M.numSpotLightShadowsWithMaps,numLightProbes:M.numLightProbes,numClippingPlanes:a.numPlanes,numClipIntersection:a.numIntersection,dithering:y.dithering,shadowMapEnabled:n.shadowMap.enabled&&I.length>0,shadowMapType:n.shadowMap.type,toneMapping:Ue,decodeVideoTexture:Fe&&y.map.isVideoTexture===!0&&_e.getTransfer(y.map.colorSpace)===Ce,decodeVideoTextureEmissive:Le&&y.emissiveMap.isVideoTexture===!0&&_e.getTransfer(y.emissiveMap.colorSpace)===Ce,premultipliedAlpha:y.premultipliedAlpha,doubleSided:y.side===Tn,flipSided:y.side===bn,useDepthPacking:y.depthPacking>=0,depthPacking:y.depthPacking||0,index0AttributeName:y.index0AttributeName,extensionClipCullDistance:re&&y.extensions.clipCullDistance===!0&&i.has("WEBGL_clip_cull_distance"),extensionMultiDraw:(re&&y.extensions.multiDraw===!0||Wt)&&i.has("WEBGL_multi_draw"),rendererExtensionParallelShaderCompile:i.has("KHR_parallel_shader_compile"),customProgramCacheKey:y.customProgramCacheKey()};return Je.vertexUv1s=c.has(1),Je.vertexUv2s=c.has(2),Je.vertexUv3s=c.has(3),c.clear(),Je}function f(y){const M=[];if(y.shaderID?M.push(y.shaderID):(M.push(y.customVertexShaderID),M.push(y.customFragmentShaderID)),y.defines!==void 0)for(const I in y.defines)M.push(I),M.push(y.defines[I]);return y.isRawShaderMaterial===!1&&(P(M,y),L(M,y),M.push(n.outputColorSpace)),M.push(y.customProgramCacheKey),M.join()}function P(y,M){y.push(M.precision),y.push(M.outputColorSpace),y.push(M.envMapMode),y.push(M.envMapCubeUVHeight),y.push(M.mapUv),y.push(M.alphaMapUv),y.push(M.lightMapUv),y.push(M.aoMapUv),y.push(M.bumpMapUv),y.push(M.normalMapUv),y.push(M.displacementMapUv),y.push(M.emissiveMapUv),y.push(M.metalnessMapUv),y.push(M.roughnessMapUv),y.push(M.anisotropyMapUv),y.push(M.clearcoatMapUv),y.push(M.clearcoatNormalMapUv),y.push(M.clearcoatRoughnessMapUv),y.push(M.iridescenceMapUv),y.push(M.iridescenceThicknessMapUv),y.push(M.sheenColorMapUv),y.push(M.sheenRoughnessMapUv),y.push(M.specularMapUv),y.push(M.specularColorMapUv),y.push(M.specularIntensityMapUv),y.push(M.transmissionMapUv),y.push(M.thicknessMapUv),y.push(M.combine),y.push(M.fogExp2),y.push(M.sizeAttenuation),y.push(M.morphTargetsCount),y.push(M.morphAttributeCount),y.push(M.numDirLights),y.push(M.numPointLights),y.push(M.numSpotLights),y.push(M.numSpotLightMaps),y.push(M.numHemiLights),y.push(M.numRectAreaLights),y.push(M.numDirLightShadows),y.push(M.numPointLightShadows),y.push(M.numSpotLightShadows),y.push(M.numSpotLightShadowsWithMaps),y.push(M.numLightProbes),y.push(M.shadowMapType),y.push(M.toneMapping),y.push(M.numClippingPlanes),y.push(M.numClipIntersection),y.push(M.depthPacking)}function L(y,M){o.disableAll(),M.supportsVertexTextures&&o.enable(0),M.instancing&&o.enable(1),M.instancingColor&&o.enable(2),M.instancingMorph&&o.enable(3),M.matcap&&o.enable(4),M.envMap&&o.enable(5),M.normalMapObjectSpace&&o.enable(6),M.normalMapTangentSpace&&o.enable(7),M.clearcoat&&o.enable(8),M.iridescence&&o.enable(9),M.alphaTest&&o.enable(10),M.vertexColors&&o.enable(11),M.vertexAlphas&&o.enable(12),M.vertexUv1s&&o.enable(13),M.vertexUv2s&&o.enable(14),M.vertexUv3s&&o.enable(15),M.vertexTangents&&o.enable(16),M.anisotropy&&o.enable(17),M.alphaHash&&o.enable(18),M.batching&&o.enable(19),M.dispersion&&o.enable(20),M.batchingColor&&o.enable(21),y.push(o.mask),o.disableAll(),M.fog&&o.enable(0),M.useFog&&o.enable(1),M.flatShading&&o.enable(2),M.logarithmicDepthBuffer&&o.enable(3),M.reverseDepthBuffer&&o.enable(4),M.skinning&&o.enable(5),M.morphTargets&&o.enable(6),M.morphNormals&&o.enable(7),M.morphColors&&o.enable(8),M.premultipliedAlpha&&o.enable(9),M.shadowMapEnabled&&o.enable(10),M.doubleSided&&o.enable(11),M.flipSided&&o.enable(12),M.useDepthPacking&&o.enable(13),M.dithering&&o.enable(14),M.transmission&&o.enable(15),M.sheen&&o.enable(16),M.opaque&&o.enable(17),M.pointsUvs&&o.enable(18),M.decodeVideoTexture&&o.enable(19),M.decodeVideoTextureEmissive&&o.enable(20),M.alphaToCoverage&&o.enable(21),y.push(o.mask)}function w(y){const M=g[y.type];let I;if(M){const V=Gn[M];I=Nm.clone(V.uniforms)}else I=y.uniforms;return I}function N(y,M){let I;for(let V=0,W=u.length;V<W;V++){const Z=u[V];if(Z.cacheKey===M){I=Z,++I.usedTimes;break}}return I===void 0&&(I=new bx(n,M,y,r),u.push(I)),I}function F(y){if(--y.usedTimes===0){const M=u.indexOf(y);u[M]=u[u.length-1],u.pop(),y.destroy()}}function U(y){l.remove(y)}function k(){l.dispose()}return{getParameters:m,getProgramCacheKey:f,getUniforms:w,acquireProgram:N,releaseProgram:F,releaseShaderCache:U,programs:u,dispose:k}}function Tx(){let n=new WeakMap;function t(a){return n.has(a)}function e(a){let o=n.get(a);return o===void 0&&(o={},n.set(a,o)),o}function i(a){n.delete(a)}function s(a,o,l){n.get(a)[o]=l}function r(){n=new WeakMap}return{has:t,get:e,remove:i,update:s,dispose:r}}function wx(n,t){return n.groupOrder!==t.groupOrder?n.groupOrder-t.groupOrder:n.renderOrder!==t.renderOrder?n.renderOrder-t.renderOrder:n.material.id!==t.material.id?n.material.id-t.material.id:n.z!==t.z?n.z-t.z:n.id-t.id}function lh(n,t){return n.groupOrder!==t.groupOrder?n.groupOrder-t.groupOrder:n.renderOrder!==t.renderOrder?n.renderOrder-t.renderOrder:n.z!==t.z?t.z-n.z:n.id-t.id}function ch(){const n=[];let t=0;const e=[],i=[],s=[];function r(){t=0,e.length=0,i.length=0,s.length=0}function a(h,d,p,g,_,m){let f=n[t];return f===void 0?(f={id:h.id,object:h,geometry:d,material:p,groupOrder:g,renderOrder:h.renderOrder,z:_,group:m},n[t]=f):(f.id=h.id,f.object=h,f.geometry=d,f.material=p,f.groupOrder=g,f.renderOrder=h.renderOrder,f.z=_,f.group=m),t++,f}function o(h,d,p,g,_,m){const f=a(h,d,p,g,_,m);p.transmission>0?i.push(f):p.transparent===!0?s.push(f):e.push(f)}function l(h,d,p,g,_,m){const f=a(h,d,p,g,_,m);p.transmission>0?i.unshift(f):p.transparent===!0?s.unshift(f):e.unshift(f)}function c(h,d){e.length>1&&e.sort(h||wx),i.length>1&&i.sort(d||lh),s.length>1&&s.sort(d||lh)}function u(){for(let h=t,d=n.length;h<d;h++){const p=n[h];if(p.id===null)break;p.id=null,p.object=null,p.geometry=null,p.material=null,p.group=null}}return{opaque:e,transmissive:i,transparent:s,init:r,push:o,unshift:l,finish:u,sort:c}}function Ax(){let n=new WeakMap;function t(i,s){const r=n.get(i);let a;return r===void 0?(a=new ch,n.set(i,[a])):s>=r.length?(a=new ch,r.push(a)):a=r[s],a}function e(){n=new WeakMap}return{get:t,dispose:e}}function Cx(){const n={};return{get:function(t){if(n[t.id]!==void 0)return n[t.id];let e;switch(t.type){case"DirectionalLight":e={direction:new O,color:new ne};break;case"SpotLight":e={position:new O,direction:new O,color:new ne,distance:0,coneCos:0,penumbraCos:0,decay:0};break;case"PointLight":e={position:new O,color:new ne,distance:0,decay:0};break;case"HemisphereLight":e={direction:new O,skyColor:new ne,groundColor:new ne};break;case"RectAreaLight":e={color:new ne,position:new O,halfWidth:new O,halfHeight:new O};break}return n[t.id]=e,e}}}function Rx(){const n={};return{get:function(t){if(n[t.id]!==void 0)return n[t.id];let e;switch(t.type){case"DirectionalLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Yt};break;case"SpotLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Yt};break;case"PointLight":e={shadowIntensity:1,shadowBias:0,shadowNormalBias:0,shadowRadius:1,shadowMapSize:new Yt,shadowCameraNear:1,shadowCameraFar:1e3};break}return n[t.id]=e,e}}}let Px=0;function Lx(n,t){return(t.castShadow?2:0)-(n.castShadow?2:0)+(t.map?1:0)-(n.map?1:0)}function Nx(n){const t=new Cx,e=Rx(),i={version:0,hash:{directionalLength:-1,pointLength:-1,spotLength:-1,rectAreaLength:-1,hemiLength:-1,numDirectionalShadows:-1,numPointShadows:-1,numSpotShadows:-1,numSpotMaps:-1,numLightProbes:-1},ambient:[0,0,0],probe:[],directional:[],directionalShadow:[],directionalShadowMap:[],directionalShadowMatrix:[],spot:[],spotLightMap:[],spotShadow:[],spotShadowMap:[],spotLightMatrix:[],rectArea:[],rectAreaLTC1:null,rectAreaLTC2:null,point:[],pointShadow:[],pointShadowMap:[],pointShadowMatrix:[],hemi:[],numSpotLightShadowsWithMaps:0,numLightProbes:0};for(let c=0;c<9;c++)i.probe.push(new O);const s=new O,r=new Kt,a=new Kt;function o(c){let u=0,h=0,d=0;for(let y=0;y<9;y++)i.probe[y].set(0,0,0);let p=0,g=0,_=0,m=0,f=0,P=0,L=0,w=0,N=0,F=0,U=0;c.sort(Lx);for(let y=0,M=c.length;y<M;y++){const I=c[y],V=I.color,W=I.intensity,Z=I.distance,et=I.shadow&&I.shadow.map?I.shadow.map.texture:null;if(I.isAmbientLight)u+=V.r*W,h+=V.g*W,d+=V.b*W;else if(I.isLightProbe){for(let z=0;z<9;z++)i.probe[z].addScaledVector(I.sh.coefficients[z],W);U++}else if(I.isDirectionalLight){const z=t.get(I);if(z.color.copy(I.color).multiplyScalar(I.intensity),I.castShadow){const it=I.shadow,j=e.get(I);j.shadowIntensity=it.intensity,j.shadowBias=it.bias,j.shadowNormalBias=it.normalBias,j.shadowRadius=it.radius,j.shadowMapSize=it.mapSize,i.directionalShadow[p]=j,i.directionalShadowMap[p]=et,i.directionalShadowMatrix[p]=I.shadow.matrix,P++}i.directional[p]=z,p++}else if(I.isSpotLight){const z=t.get(I);z.position.setFromMatrixPosition(I.matrixWorld),z.color.copy(V).multiplyScalar(W),z.distance=Z,z.coneCos=Math.cos(I.angle),z.penumbraCos=Math.cos(I.angle*(1-I.penumbra)),z.decay=I.decay,i.spot[_]=z;const it=I.shadow;if(I.map&&(i.spotLightMap[N]=I.map,N++,it.updateMatrices(I),I.castShadow&&F++),i.spotLightMatrix[_]=it.matrix,I.castShadow){const j=e.get(I);j.shadowIntensity=it.intensity,j.shadowBias=it.bias,j.shadowNormalBias=it.normalBias,j.shadowRadius=it.radius,j.shadowMapSize=it.mapSize,i.spotShadow[_]=j,i.spotShadowMap[_]=et,w++}_++}else if(I.isRectAreaLight){const z=t.get(I);z.color.copy(V).multiplyScalar(W),z.halfWidth.set(I.width*.5,0,0),z.halfHeight.set(0,I.height*.5,0),i.rectArea[m]=z,m++}else if(I.isPointLight){const z=t.get(I);if(z.color.copy(I.color).multiplyScalar(I.intensity),z.distance=I.distance,z.decay=I.decay,I.castShadow){const it=I.shadow,j=e.get(I);j.shadowIntensity=it.intensity,j.shadowBias=it.bias,j.shadowNormalBias=it.normalBias,j.shadowRadius=it.radius,j.shadowMapSize=it.mapSize,j.shadowCameraNear=it.camera.near,j.shadowCameraFar=it.camera.far,i.pointShadow[g]=j,i.pointShadowMap[g]=et,i.pointShadowMatrix[g]=I.shadow.matrix,L++}i.point[g]=z,g++}else if(I.isHemisphereLight){const z=t.get(I);z.skyColor.copy(I.color).multiplyScalar(W),z.groundColor.copy(I.groundColor).multiplyScalar(W),i.hemi[f]=z,f++}}m>0&&(n.has("OES_texture_float_linear")===!0?(i.rectAreaLTC1=bt.LTC_FLOAT_1,i.rectAreaLTC2=bt.LTC_FLOAT_2):(i.rectAreaLTC1=bt.LTC_HALF_1,i.rectAreaLTC2=bt.LTC_HALF_2)),i.ambient[0]=u,i.ambient[1]=h,i.ambient[2]=d;const k=i.hash;(k.directionalLength!==p||k.pointLength!==g||k.spotLength!==_||k.rectAreaLength!==m||k.hemiLength!==f||k.numDirectionalShadows!==P||k.numPointShadows!==L||k.numSpotShadows!==w||k.numSpotMaps!==N||k.numLightProbes!==U)&&(i.directional.length=p,i.spot.length=_,i.rectArea.length=m,i.point.length=g,i.hemi.length=f,i.directionalShadow.length=P,i.directionalShadowMap.length=P,i.pointShadow.length=L,i.pointShadowMap.length=L,i.spotShadow.length=w,i.spotShadowMap.length=w,i.directionalShadowMatrix.length=P,i.pointShadowMatrix.length=L,i.spotLightMatrix.length=w+N-F,i.spotLightMap.length=N,i.numSpotLightShadowsWithMaps=F,i.numLightProbes=U,k.directionalLength=p,k.pointLength=g,k.spotLength=_,k.rectAreaLength=m,k.hemiLength=f,k.numDirectionalShadows=P,k.numPointShadows=L,k.numSpotShadows=w,k.numSpotMaps=N,k.numLightProbes=U,i.version=Px++)}function l(c,u){let h=0,d=0,p=0,g=0,_=0;const m=u.matrixWorldInverse;for(let f=0,P=c.length;f<P;f++){const L=c[f];if(L.isDirectionalLight){const w=i.directional[h];w.direction.setFromMatrixPosition(L.matrixWorld),s.setFromMatrixPosition(L.target.matrixWorld),w.direction.sub(s),w.direction.transformDirection(m),h++}else if(L.isSpotLight){const w=i.spot[p];w.position.setFromMatrixPosition(L.matrixWorld),w.position.applyMatrix4(m),w.direction.setFromMatrixPosition(L.matrixWorld),s.setFromMatrixPosition(L.target.matrixWorld),w.direction.sub(s),w.direction.transformDirection(m),p++}else if(L.isRectAreaLight){const w=i.rectArea[g];w.position.setFromMatrixPosition(L.matrixWorld),w.position.applyMatrix4(m),a.identity(),r.copy(L.matrixWorld),r.premultiply(m),a.extractRotation(r),w.halfWidth.set(L.width*.5,0,0),w.halfHeight.set(0,L.height*.5,0),w.halfWidth.applyMatrix4(a),w.halfHeight.applyMatrix4(a),g++}else if(L.isPointLight){const w=i.point[d];w.position.setFromMatrixPosition(L.matrixWorld),w.position.applyMatrix4(m),d++}else if(L.isHemisphereLight){const w=i.hemi[_];w.direction.setFromMatrixPosition(L.matrixWorld),w.direction.transformDirection(m),_++}}}return{setup:o,setupView:l,state:i}}function uh(n){const t=new Nx(n),e=[],i=[];function s(u){c.camera=u,e.length=0,i.length=0}function r(u){e.push(u)}function a(u){i.push(u)}function o(){t.setup(e)}function l(u){t.setupView(e,u)}const c={lightsArray:e,shadowsArray:i,camera:null,lights:t,transmissionRenderTarget:{}};return{init:s,state:c,setupLights:o,setupLightsView:l,pushLight:r,pushShadow:a}}function Dx(n){let t=new WeakMap;function e(s,r=0){const a=t.get(s);let o;return a===void 0?(o=new uh(n),t.set(s,[o])):r>=a.length?(o=new uh(n),a.push(o)):o=a[r],o}function i(){t=new WeakMap}return{get:e,dispose:i}}const Ix=`void main() {
	gl_Position = vec4( position, 1.0 );
}`,Ux=`uniform sampler2D shadow_pass;
uniform vec2 resolution;
uniform float radius;
#include <packing>
void main() {
	const float samples = float( VSM_SAMPLES );
	float mean = 0.0;
	float squared_mean = 0.0;
	float uvStride = samples <= 1.0 ? 0.0 : 2.0 / ( samples - 1.0 );
	float uvStart = samples <= 1.0 ? 0.0 : - 1.0;
	for ( float i = 0.0; i < samples; i ++ ) {
		float uvOffset = uvStart + i * uvStride;
		#ifdef HORIZONTAL_PASS
			vec2 distribution = unpackRGBATo2Half( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( uvOffset, 0.0 ) * radius ) / resolution ) );
			mean += distribution.x;
			squared_mean += distribution.y * distribution.y + distribution.x * distribution.x;
		#else
			float depth = unpackRGBAToDepth( texture2D( shadow_pass, ( gl_FragCoord.xy + vec2( 0.0, uvOffset ) * radius ) / resolution ) );
			mean += depth;
			squared_mean += depth * depth;
		#endif
	}
	mean = mean / samples;
	squared_mean = squared_mean / samples;
	float std_dev = sqrt( squared_mean - mean * mean );
	gl_FragColor = pack2HalfToRGBA( vec2( mean, std_dev ) );
}`;function kx(n,t,e){let i=new pc;const s=new Yt,r=new Yt,a=new Ee,o=new jm({depthPacking:Bp}),l=new Xm,c={},u=e.maxTextureSize,h={[ui]:bn,[bn]:ui,[Tn]:Tn},d=new Ni({defines:{VSM_SAMPLES:8},uniforms:{shadow_pass:{value:null},resolution:{value:new Yt},radius:{value:4}},vertexShader:Ix,fragmentShader:Ux}),p=d.clone();p.defines.HORIZONTAL_PASS=1;const g=new $e;g.setAttribute("position",new dn(new Float32Array([-1,-1,.5,3,-1,.5,-1,3,.5]),3));const _=new Re(g,d),m=this;this.enabled=!1,this.autoUpdate=!0,this.needsUpdate=!1,this.type=Hh;let f=this.type;this.render=function(F,U,k){if(m.enabled===!1||m.autoUpdate===!1&&m.needsUpdate===!1||F.length===0)return;const y=n.getRenderTarget(),M=n.getActiveCubeFace(),I=n.getActiveMipmapLevel(),V=n.state;V.setBlending(Ri),V.buffers.color.setClear(1,1,1,1),V.buffers.depth.setTest(!0),V.setScissorTest(!1);const W=f!==ri&&this.type===ri,Z=f===ri&&this.type!==ri;for(let et=0,z=F.length;et<z;et++){const it=F[et],j=it.shadow;if(j===void 0){console.warn("THREE.WebGLShadowMap:",it,"has no shadow.");continue}if(j.autoUpdate===!1&&j.needsUpdate===!1)continue;s.copy(j.mapSize);const ut=j.getFrameExtents();if(s.multiply(ut),r.copy(j.mapSize),(s.x>u||s.y>u)&&(s.x>u&&(r.x=Math.floor(u/ut.x),s.x=r.x*ut.x,j.mapSize.x=r.x),s.y>u&&(r.y=Math.floor(u/ut.y),s.y=r.y*ut.y,j.mapSize.y=r.y)),j.map===null||W===!0||Z===!0){const gt=this.type!==ri?{minFilter:An,magFilter:An}:{};j.map!==null&&j.map.dispose(),j.map=new es(s.x,s.y,gt),j.map.texture.name=it.name+".shadowMap",j.camera.updateProjectionMatrix()}n.setRenderTarget(j.map),n.clear();const mt=j.getViewportCount();for(let gt=0;gt<mt;gt++){const Lt=j.getViewport(gt);a.set(r.x*Lt.x,r.y*Lt.y,r.x*Lt.z,r.y*Lt.w),V.viewport(a),j.updateMatrices(it,gt),i=j.getFrustum(),w(U,k,j.camera,it,this.type)}j.isPointLightShadow!==!0&&this.type===ri&&P(j,k),j.needsUpdate=!1}f=this.type,m.needsUpdate=!1,n.setRenderTarget(y,M,I)};function P(F,U){const k=t.update(_);d.defines.VSM_SAMPLES!==F.blurSamples&&(d.defines.VSM_SAMPLES=F.blurSamples,p.defines.VSM_SAMPLES=F.blurSamples,d.needsUpdate=!0,p.needsUpdate=!0),F.mapPass===null&&(F.mapPass=new es(s.x,s.y)),d.uniforms.shadow_pass.value=F.map.texture,d.uniforms.resolution.value=F.mapSize,d.uniforms.radius.value=F.radius,n.setRenderTarget(F.mapPass),n.clear(),n.renderBufferDirect(U,null,k,d,_,null),p.uniforms.shadow_pass.value=F.mapPass.texture,p.uniforms.resolution.value=F.mapSize,p.uniforms.radius.value=F.radius,n.setRenderTarget(F.map),n.clear(),n.renderBufferDirect(U,null,k,p,_,null)}function L(F,U,k,y){let M=null;const I=k.isPointLight===!0?F.customDistanceMaterial:F.customDepthMaterial;if(I!==void 0)M=I;else if(M=k.isPointLight===!0?l:o,n.localClippingEnabled&&U.clipShadows===!0&&Array.isArray(U.clippingPlanes)&&U.clippingPlanes.length!==0||U.displacementMap&&U.displacementScale!==0||U.alphaMap&&U.alphaTest>0||U.map&&U.alphaTest>0){const V=M.uuid,W=U.uuid;let Z=c[V];Z===void 0&&(Z={},c[V]=Z);let et=Z[W];et===void 0&&(et=M.clone(),Z[W]=et,U.addEventListener("dispose",N)),M=et}if(M.visible=U.visible,M.wireframe=U.wireframe,y===ri?M.side=U.shadowSide!==null?U.shadowSide:U.side:M.side=U.shadowSide!==null?U.shadowSide:h[U.side],M.alphaMap=U.alphaMap,M.alphaTest=U.alphaTest,M.map=U.map,M.clipShadows=U.clipShadows,M.clippingPlanes=U.clippingPlanes,M.clipIntersection=U.clipIntersection,M.displacementMap=U.displacementMap,M.displacementScale=U.displacementScale,M.displacementBias=U.displacementBias,M.wireframeLinewidth=U.wireframeLinewidth,M.linewidth=U.linewidth,k.isPointLight===!0&&M.isMeshDistanceMaterial===!0){const V=n.properties.get(M);V.light=k}return M}function w(F,U,k,y,M){if(F.visible===!1)return;if(F.layers.test(U.layers)&&(F.isMesh||F.isLine||F.isPoints)&&(F.castShadow||F.receiveShadow&&M===ri)&&(!F.frustumCulled||i.intersectsObject(F))){F.modelViewMatrix.multiplyMatrices(k.matrixWorldInverse,F.matrixWorld);const W=t.update(F),Z=F.material;if(Array.isArray(Z)){const et=W.groups;for(let z=0,it=et.length;z<it;z++){const j=et[z],ut=Z[j.materialIndex];if(ut&&ut.visible){const mt=L(F,ut,y,M);F.onBeforeShadow(n,F,U,k,W,mt,j),n.renderBufferDirect(k,null,W,mt,F,j),F.onAfterShadow(n,F,U,k,W,mt,j)}}}else if(Z.visible){const et=L(F,Z,y,M);F.onBeforeShadow(n,F,U,k,W,et,null),n.renderBufferDirect(k,null,W,et,F,null),F.onAfterShadow(n,F,U,k,W,et,null)}}const V=F.children;for(let W=0,Z=V.length;W<Z;W++)w(V[W],U,k,y,M)}function N(F){F.target.removeEventListener("dispose",N);for(const k in c){const y=c[k],M=F.target.uuid;M in y&&(y[M].dispose(),delete y[M])}}}const Fx={[al]:ol,[ll]:hl,[cl]:dl,[Ws]:ul,[ol]:al,[hl]:ll,[dl]:cl,[ul]:Ws};function Ox(n,t){function e(){let G=!1;const Mt=new Ee;let tt=null;const ct=new Ee(0,0,0,0);return{setMask:function(wt){tt!==wt&&!G&&(n.colorMask(wt,wt,wt,wt),tt=wt)},setLocked:function(wt){G=wt},setClear:function(wt,Et,re,Ue,Je){Je===!0&&(wt*=Ue,Et*=Ue,re*=Ue),Mt.set(wt,Et,re,Ue),ct.equals(Mt)===!1&&(n.clearColor(wt,Et,re,Ue),ct.copy(Mt))},reset:function(){G=!1,tt=null,ct.set(-1,0,0,0)}}}function i(){let G=!1,Mt=!1,tt=null,ct=null,wt=null;return{setReversed:function(Et){if(Mt!==Et){const re=t.get("EXT_clip_control");Mt?re.clipControlEXT(re.LOWER_LEFT_EXT,re.ZERO_TO_ONE_EXT):re.clipControlEXT(re.LOWER_LEFT_EXT,re.NEGATIVE_ONE_TO_ONE_EXT);const Ue=wt;wt=null,this.setClear(Ue)}Mt=Et},getReversed:function(){return Mt},setTest:function(Et){Et?st(n.DEPTH_TEST):vt(n.DEPTH_TEST)},setMask:function(Et){tt!==Et&&!G&&(n.depthMask(Et),tt=Et)},setFunc:function(Et){if(Mt&&(Et=Fx[Et]),ct!==Et){switch(Et){case al:n.depthFunc(n.NEVER);break;case ol:n.depthFunc(n.ALWAYS);break;case ll:n.depthFunc(n.LESS);break;case Ws:n.depthFunc(n.LEQUAL);break;case cl:n.depthFunc(n.EQUAL);break;case ul:n.depthFunc(n.GEQUAL);break;case hl:n.depthFunc(n.GREATER);break;case dl:n.depthFunc(n.NOTEQUAL);break;default:n.depthFunc(n.LEQUAL)}ct=Et}},setLocked:function(Et){G=Et},setClear:function(Et){wt!==Et&&(Mt&&(Et=1-Et),n.clearDepth(Et),wt=Et)},reset:function(){G=!1,tt=null,ct=null,wt=null,Mt=!1}}}function s(){let G=!1,Mt=null,tt=null,ct=null,wt=null,Et=null,re=null,Ue=null,Je=null;return{setTest:function(oe){G||(oe?st(n.STENCIL_TEST):vt(n.STENCIL_TEST))},setMask:function(oe){Mt!==oe&&!G&&(n.stencilMask(oe),Mt=oe)},setFunc:function(oe,mn,Dn){(tt!==oe||ct!==mn||wt!==Dn)&&(n.stencilFunc(oe,mn,Dn),tt=oe,ct=mn,wt=Dn)},setOp:function(oe,mn,Dn){(Et!==oe||re!==mn||Ue!==Dn)&&(n.stencilOp(oe,mn,Dn),Et=oe,re=mn,Ue=Dn)},setLocked:function(oe){G=oe},setClear:function(oe){Je!==oe&&(n.clearStencil(oe),Je=oe)},reset:function(){G=!1,Mt=null,tt=null,ct=null,wt=null,Et=null,re=null,Ue=null,Je=null}}}const r=new e,a=new i,o=new s,l=new WeakMap,c=new WeakMap;let u={},h={},d=new WeakMap,p=[],g=null,_=!1,m=null,f=null,P=null,L=null,w=null,N=null,F=null,U=new ne(0,0,0),k=0,y=!1,M=null,I=null,V=null,W=null,Z=null;const et=n.getParameter(n.MAX_COMBINED_TEXTURE_IMAGE_UNITS);let z=!1,it=0;const j=n.getParameter(n.VERSION);j.indexOf("WebGL")!==-1?(it=parseFloat(/^WebGL (\d)/.exec(j)[1]),z=it>=1):j.indexOf("OpenGL ES")!==-1&&(it=parseFloat(/^OpenGL ES (\d)/.exec(j)[1]),z=it>=2);let ut=null,mt={};const gt=n.getParameter(n.SCISSOR_BOX),Lt=n.getParameter(n.VIEWPORT),At=new Ee().fromArray(gt),X=new Ee().fromArray(Lt);function Q(G,Mt,tt,ct){const wt=new Uint8Array(4),Et=n.createTexture();n.bindTexture(G,Et),n.texParameteri(G,n.TEXTURE_MIN_FILTER,n.NEAREST),n.texParameteri(G,n.TEXTURE_MAG_FILTER,n.NEAREST);for(let re=0;re<tt;re++)G===n.TEXTURE_3D||G===n.TEXTURE_2D_ARRAY?n.texImage3D(Mt,0,n.RGBA,1,1,ct,0,n.RGBA,n.UNSIGNED_BYTE,wt):n.texImage2D(Mt+re,0,n.RGBA,1,1,0,n.RGBA,n.UNSIGNED_BYTE,wt);return Et}const nt={};nt[n.TEXTURE_2D]=Q(n.TEXTURE_2D,n.TEXTURE_2D,1),nt[n.TEXTURE_CUBE_MAP]=Q(n.TEXTURE_CUBE_MAP,n.TEXTURE_CUBE_MAP_POSITIVE_X,6),nt[n.TEXTURE_2D_ARRAY]=Q(n.TEXTURE_2D_ARRAY,n.TEXTURE_2D_ARRAY,1,1),nt[n.TEXTURE_3D]=Q(n.TEXTURE_3D,n.TEXTURE_3D,1,1),r.setClear(0,0,0,1),a.setClear(1),o.setClear(0),st(n.DEPTH_TEST),a.setFunc(Ws),fe(!1),pe($c),st(n.CULL_FACE),B(Ri);function st(G){u[G]!==!0&&(n.enable(G),u[G]=!0)}function vt(G){u[G]!==!1&&(n.disable(G),u[G]=!1)}function se(G,Mt){return h[G]!==Mt?(n.bindFramebuffer(G,Mt),h[G]=Mt,G===n.DRAW_FRAMEBUFFER&&(h[n.FRAMEBUFFER]=Mt),G===n.FRAMEBUFFER&&(h[n.DRAW_FRAMEBUFFER]=Mt),!0):!1}function Wt(G,Mt){let tt=p,ct=!1;if(G){tt=d.get(Mt),tt===void 0&&(tt=[],d.set(Mt,tt));const wt=G.textures;if(tt.length!==wt.length||tt[0]!==n.COLOR_ATTACHMENT0){for(let Et=0,re=wt.length;Et<re;Et++)tt[Et]=n.COLOR_ATTACHMENT0+Et;tt.length=wt.length,ct=!0}}else tt[0]!==n.BACK&&(tt[0]=n.BACK,ct=!0);ct&&n.drawBuffers(tt)}function Fe(G){return g!==G?(n.useProgram(G),g=G,!0):!1}const Ie={[qi]:n.FUNC_ADD,[up]:n.FUNC_SUBTRACT,[hp]:n.FUNC_REVERSE_SUBTRACT};Ie[dp]=n.MIN,Ie[fp]=n.MAX;const me={[pp]:n.ZERO,[mp]:n.ONE,[gp]:n.SRC_COLOR,[sl]:n.SRC_ALPHA,[Mp]:n.SRC_ALPHA_SATURATE,[bp]:n.DST_COLOR,[vp]:n.DST_ALPHA,[_p]:n.ONE_MINUS_SRC_COLOR,[rl]:n.ONE_MINUS_SRC_ALPHA,[yp]:n.ONE_MINUS_DST_COLOR,[xp]:n.ONE_MINUS_DST_ALPHA,[Sp]:n.CONSTANT_COLOR,[Ep]:n.ONE_MINUS_CONSTANT_COLOR,[Tp]:n.CONSTANT_ALPHA,[wp]:n.ONE_MINUS_CONSTANT_ALPHA};function B(G,Mt,tt,ct,wt,Et,re,Ue,Je,oe){if(G===Ri){_===!0&&(vt(n.BLEND),_=!1);return}if(_===!1&&(st(n.BLEND),_=!0),G!==cp){if(G!==m||oe!==y){if((f!==qi||w!==qi)&&(n.blendEquation(n.FUNC_ADD),f=qi,w=qi),oe)switch(G){case Fs:n.blendFuncSeparate(n.ONE,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case jc:n.blendFunc(n.ONE,n.ONE);break;case Xc:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case qc:n.blendFuncSeparate(n.ZERO,n.SRC_COLOR,n.ZERO,n.SRC_ALPHA);break;default:console.error("THREE.WebGLState: Invalid blending: ",G);break}else switch(G){case Fs:n.blendFuncSeparate(n.SRC_ALPHA,n.ONE_MINUS_SRC_ALPHA,n.ONE,n.ONE_MINUS_SRC_ALPHA);break;case jc:n.blendFunc(n.SRC_ALPHA,n.ONE);break;case Xc:n.blendFuncSeparate(n.ZERO,n.ONE_MINUS_SRC_COLOR,n.ZERO,n.ONE);break;case qc:n.blendFunc(n.ZERO,n.SRC_COLOR);break;default:console.error("THREE.WebGLState: Invalid blending: ",G);break}P=null,L=null,N=null,F=null,U.set(0,0,0),k=0,m=G,y=oe}return}wt=wt||Mt,Et=Et||tt,re=re||ct,(Mt!==f||wt!==w)&&(n.blendEquationSeparate(Ie[Mt],Ie[wt]),f=Mt,w=wt),(tt!==P||ct!==L||Et!==N||re!==F)&&(n.blendFuncSeparate(me[tt],me[ct],me[Et],me[re]),P=tt,L=ct,N=Et,F=re),(Ue.equals(U)===!1||Je!==k)&&(n.blendColor(Ue.r,Ue.g,Ue.b,Je),U.copy(Ue),k=Je),m=G,y=!1}function pn(G,Mt){G.side===Tn?vt(n.CULL_FACE):st(n.CULL_FACE);let tt=G.side===bn;Mt&&(tt=!tt),fe(tt),G.blending===Fs&&G.transparent===!1?B(Ri):B(G.blending,G.blendEquation,G.blendSrc,G.blendDst,G.blendEquationAlpha,G.blendSrcAlpha,G.blendDstAlpha,G.blendColor,G.blendAlpha,G.premultipliedAlpha),a.setFunc(G.depthFunc),a.setTest(G.depthTest),a.setMask(G.depthWrite),r.setMask(G.colorWrite);const ct=G.stencilWrite;o.setTest(ct),ct&&(o.setMask(G.stencilWriteMask),o.setFunc(G.stencilFunc,G.stencilRef,G.stencilFuncMask),o.setOp(G.stencilFail,G.stencilZFail,G.stencilZPass)),Le(G.polygonOffset,G.polygonOffsetFactor,G.polygonOffsetUnits),G.alphaToCoverage===!0?st(n.SAMPLE_ALPHA_TO_COVERAGE):vt(n.SAMPLE_ALPHA_TO_COVERAGE)}function fe(G){M!==G&&(G?n.frontFace(n.CW):n.frontFace(n.CCW),M=G)}function pe(G){G!==op?(st(n.CULL_FACE),G!==I&&(G===$c?n.cullFace(n.BACK):G===lp?n.cullFace(n.FRONT):n.cullFace(n.FRONT_AND_BACK))):vt(n.CULL_FACE),I=G}function Vt(G){G!==V&&(z&&n.lineWidth(G),V=G)}function Le(G,Mt,tt){G?(st(n.POLYGON_OFFSET_FILL),(W!==Mt||Z!==tt)&&(n.polygonOffset(Mt,tt),W=Mt,Z=tt)):vt(n.POLYGON_OFFSET_FILL)}function zt(G){G?st(n.SCISSOR_TEST):vt(n.SCISSOR_TEST)}function D(G){G===void 0&&(G=n.TEXTURE0+et-1),ut!==G&&(n.activeTexture(G),ut=G)}function S(G,Mt,tt){tt===void 0&&(ut===null?tt=n.TEXTURE0+et-1:tt=ut);let ct=mt[tt];ct===void 0&&(ct={type:void 0,texture:void 0},mt[tt]=ct),(ct.type!==G||ct.texture!==Mt)&&(ut!==tt&&(n.activeTexture(tt),ut=tt),n.bindTexture(G,Mt||nt[G]),ct.type=G,ct.texture=Mt)}function q(){const G=mt[ut];G!==void 0&&G.type!==void 0&&(n.bindTexture(G.type,null),G.type=void 0,G.texture=void 0)}function lt(){try{n.compressedTexImage2D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function ht(){try{n.compressedTexImage3D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function ot(){try{n.texSubImage2D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function Ft(){try{n.texSubImage3D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function St(){try{n.compressedTexSubImage2D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function Pt(){try{n.compressedTexSubImage3D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function ve(){try{n.texStorage2D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function pt(){try{n.texStorage3D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function Nt(){try{n.texImage2D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function Xt(){try{n.texImage3D(...arguments)}catch(G){console.error("THREE.WebGLState:",G)}}function Zt(G){At.equals(G)===!1&&(n.scissor(G.x,G.y,G.z,G.w),At.copy(G))}function Dt(G){X.equals(G)===!1&&(n.viewport(G.x,G.y,G.z,G.w),X.copy(G))}function ge(G,Mt){let tt=c.get(Mt);tt===void 0&&(tt=new WeakMap,c.set(Mt,tt));let ct=tt.get(G);ct===void 0&&(ct=n.getUniformBlockIndex(Mt,G.name),tt.set(G,ct))}function ae(G,Mt){const ct=c.get(Mt).get(G);l.get(Mt)!==ct&&(n.uniformBlockBinding(Mt,ct,G.__bindingPointIndex),l.set(Mt,ct))}function Te(){n.disable(n.BLEND),n.disable(n.CULL_FACE),n.disable(n.DEPTH_TEST),n.disable(n.POLYGON_OFFSET_FILL),n.disable(n.SCISSOR_TEST),n.disable(n.STENCIL_TEST),n.disable(n.SAMPLE_ALPHA_TO_COVERAGE),n.blendEquation(n.FUNC_ADD),n.blendFunc(n.ONE,n.ZERO),n.blendFuncSeparate(n.ONE,n.ZERO,n.ONE,n.ZERO),n.blendColor(0,0,0,0),n.colorMask(!0,!0,!0,!0),n.clearColor(0,0,0,0),n.depthMask(!0),n.depthFunc(n.LESS),a.setReversed(!1),n.clearDepth(1),n.stencilMask(4294967295),n.stencilFunc(n.ALWAYS,0,4294967295),n.stencilOp(n.KEEP,n.KEEP,n.KEEP),n.clearStencil(0),n.cullFace(n.BACK),n.frontFace(n.CCW),n.polygonOffset(0,0),n.activeTexture(n.TEXTURE0),n.bindFramebuffer(n.FRAMEBUFFER,null),n.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),n.bindFramebuffer(n.READ_FRAMEBUFFER,null),n.useProgram(null),n.lineWidth(1),n.scissor(0,0,n.canvas.width,n.canvas.height),n.viewport(0,0,n.canvas.width,n.canvas.height),u={},ut=null,mt={},h={},d=new WeakMap,p=[],g=null,_=!1,m=null,f=null,P=null,L=null,w=null,N=null,F=null,U=new ne(0,0,0),k=0,y=!1,M=null,I=null,V=null,W=null,Z=null,At.set(0,0,n.canvas.width,n.canvas.height),X.set(0,0,n.canvas.width,n.canvas.height),r.reset(),a.reset(),o.reset()}return{buffers:{color:r,depth:a,stencil:o},enable:st,disable:vt,bindFramebuffer:se,drawBuffers:Wt,useProgram:Fe,setBlending:B,setMaterial:pn,setFlipSided:fe,setCullFace:pe,setLineWidth:Vt,setPolygonOffset:Le,setScissorTest:zt,activeTexture:D,bindTexture:S,unbindTexture:q,compressedTexImage2D:lt,compressedTexImage3D:ht,texImage2D:Nt,texImage3D:Xt,updateUBOMapping:ge,uniformBlockBinding:ae,texStorage2D:ve,texStorage3D:pt,texSubImage2D:ot,texSubImage3D:Ft,compressedTexSubImage2D:St,compressedTexSubImage3D:Pt,scissor:Zt,viewport:Dt,reset:Te}}function Bx(n,t,e,i,s,r,a){const o=t.has("WEBGL_multisampled_render_to_texture")?t.get("WEBGL_multisampled_render_to_texture"):null,l=typeof navigator>"u"?!1:/OculusBrowser/g.test(navigator.userAgent),c=new Yt,u=new WeakMap;let h;const d=new WeakMap;let p=!1;try{p=typeof OffscreenCanvas<"u"&&new OffscreenCanvas(1,1).getContext("2d")!==null}catch{}function g(D,S){return p?new OffscreenCanvas(D,S):Sr("canvas")}function _(D,S,q){let lt=1;const ht=zt(D);if((ht.width>q||ht.height>q)&&(lt=q/Math.max(ht.width,ht.height)),lt<1)if(typeof HTMLImageElement<"u"&&D instanceof HTMLImageElement||typeof HTMLCanvasElement<"u"&&D instanceof HTMLCanvasElement||typeof ImageBitmap<"u"&&D instanceof ImageBitmap||typeof VideoFrame<"u"&&D instanceof VideoFrame){const ot=Math.floor(lt*ht.width),Ft=Math.floor(lt*ht.height);h===void 0&&(h=g(ot,Ft));const St=S?g(ot,Ft):h;return St.width=ot,St.height=Ft,St.getContext("2d").drawImage(D,0,0,ot,Ft),console.warn("THREE.WebGLRenderer: Texture has been resized from ("+ht.width+"x"+ht.height+") to ("+ot+"x"+Ft+")."),St}else return"data"in D&&console.warn("THREE.WebGLRenderer: Image in DataTexture is too big ("+ht.width+"x"+ht.height+")."),D;return D}function m(D){return D.generateMipmaps}function f(D){n.generateMipmap(D)}function P(D){return D.isWebGLCubeRenderTarget?n.TEXTURE_CUBE_MAP:D.isWebGL3DRenderTarget?n.TEXTURE_3D:D.isWebGLArrayRenderTarget||D.isCompressedArrayTexture?n.TEXTURE_2D_ARRAY:n.TEXTURE_2D}function L(D,S,q,lt,ht=!1){if(D!==null){if(n[D]!==void 0)return n[D];console.warn("THREE.WebGLRenderer: Attempt to use non-existing WebGL internal format '"+D+"'")}let ot=S;if(S===n.RED&&(q===n.FLOAT&&(ot=n.R32F),q===n.HALF_FLOAT&&(ot=n.R16F),q===n.UNSIGNED_BYTE&&(ot=n.R8)),S===n.RED_INTEGER&&(q===n.UNSIGNED_BYTE&&(ot=n.R8UI),q===n.UNSIGNED_SHORT&&(ot=n.R16UI),q===n.UNSIGNED_INT&&(ot=n.R32UI),q===n.BYTE&&(ot=n.R8I),q===n.SHORT&&(ot=n.R16I),q===n.INT&&(ot=n.R32I)),S===n.RG&&(q===n.FLOAT&&(ot=n.RG32F),q===n.HALF_FLOAT&&(ot=n.RG16F),q===n.UNSIGNED_BYTE&&(ot=n.RG8)),S===n.RG_INTEGER&&(q===n.UNSIGNED_BYTE&&(ot=n.RG8UI),q===n.UNSIGNED_SHORT&&(ot=n.RG16UI),q===n.UNSIGNED_INT&&(ot=n.RG32UI),q===n.BYTE&&(ot=n.RG8I),q===n.SHORT&&(ot=n.RG16I),q===n.INT&&(ot=n.RG32I)),S===n.RGB_INTEGER&&(q===n.UNSIGNED_BYTE&&(ot=n.RGB8UI),q===n.UNSIGNED_SHORT&&(ot=n.RGB16UI),q===n.UNSIGNED_INT&&(ot=n.RGB32UI),q===n.BYTE&&(ot=n.RGB8I),q===n.SHORT&&(ot=n.RGB16I),q===n.INT&&(ot=n.RGB32I)),S===n.RGBA_INTEGER&&(q===n.UNSIGNED_BYTE&&(ot=n.RGBA8UI),q===n.UNSIGNED_SHORT&&(ot=n.RGBA16UI),q===n.UNSIGNED_INT&&(ot=n.RGBA32UI),q===n.BYTE&&(ot=n.RGBA8I),q===n.SHORT&&(ot=n.RGBA16I),q===n.INT&&(ot=n.RGBA32I)),S===n.RGB&&q===n.UNSIGNED_INT_5_9_9_9_REV&&(ot=n.RGB9_E5),S===n.RGBA){const Ft=ht?Fa:_e.getTransfer(lt);q===n.FLOAT&&(ot=n.RGBA32F),q===n.HALF_FLOAT&&(ot=n.RGBA16F),q===n.UNSIGNED_BYTE&&(ot=Ft===Ce?n.SRGB8_ALPHA8:n.RGBA8),q===n.UNSIGNED_SHORT_4_4_4_4&&(ot=n.RGBA4),q===n.UNSIGNED_SHORT_5_5_5_1&&(ot=n.RGB5_A1)}return(ot===n.R16F||ot===n.R32F||ot===n.RG16F||ot===n.RG32F||ot===n.RGBA16F||ot===n.RGBA32F)&&t.get("EXT_color_buffer_float"),ot}function w(D,S){let q;return D?S===null||S===ts||S===Xs?q=n.DEPTH24_STENCIL8:S===jn?q=n.DEPTH32F_STENCIL8:S===Mr&&(q=n.DEPTH24_STENCIL8,console.warn("DepthTexture: 16 bit depth attachment is not supported with stencil. Using 24-bit attachment.")):S===null||S===ts||S===Xs?q=n.DEPTH_COMPONENT24:S===jn?q=n.DEPTH_COMPONENT32F:S===Mr&&(q=n.DEPTH_COMPONENT16),q}function N(D,S){return m(D)===!0||D.isFramebufferTexture&&D.minFilter!==An&&D.minFilter!==wn?Math.log2(Math.max(S.width,S.height))+1:D.mipmaps!==void 0&&D.mipmaps.length>0?D.mipmaps.length:D.isCompressedTexture&&Array.isArray(D.image)?S.mipmaps.length:1}function F(D){const S=D.target;S.removeEventListener("dispose",F),k(S),S.isVideoTexture&&u.delete(S)}function U(D){const S=D.target;S.removeEventListener("dispose",U),M(S)}function k(D){const S=i.get(D);if(S.__webglInit===void 0)return;const q=D.source,lt=d.get(q);if(lt){const ht=lt[S.__cacheKey];ht.usedTimes--,ht.usedTimes===0&&y(D),Object.keys(lt).length===0&&d.delete(q)}i.remove(D)}function y(D){const S=i.get(D);n.deleteTexture(S.__webglTexture);const q=D.source,lt=d.get(q);delete lt[S.__cacheKey],a.memory.textures--}function M(D){const S=i.get(D);if(D.depthTexture&&(D.depthTexture.dispose(),i.remove(D.depthTexture)),D.isWebGLCubeRenderTarget)for(let lt=0;lt<6;lt++){if(Array.isArray(S.__webglFramebuffer[lt]))for(let ht=0;ht<S.__webglFramebuffer[lt].length;ht++)n.deleteFramebuffer(S.__webglFramebuffer[lt][ht]);else n.deleteFramebuffer(S.__webglFramebuffer[lt]);S.__webglDepthbuffer&&n.deleteRenderbuffer(S.__webglDepthbuffer[lt])}else{if(Array.isArray(S.__webglFramebuffer))for(let lt=0;lt<S.__webglFramebuffer.length;lt++)n.deleteFramebuffer(S.__webglFramebuffer[lt]);else n.deleteFramebuffer(S.__webglFramebuffer);if(S.__webglDepthbuffer&&n.deleteRenderbuffer(S.__webglDepthbuffer),S.__webglMultisampledFramebuffer&&n.deleteFramebuffer(S.__webglMultisampledFramebuffer),S.__webglColorRenderbuffer)for(let lt=0;lt<S.__webglColorRenderbuffer.length;lt++)S.__webglColorRenderbuffer[lt]&&n.deleteRenderbuffer(S.__webglColorRenderbuffer[lt]);S.__webglDepthRenderbuffer&&n.deleteRenderbuffer(S.__webglDepthRenderbuffer)}const q=D.textures;for(let lt=0,ht=q.length;lt<ht;lt++){const ot=i.get(q[lt]);ot.__webglTexture&&(n.deleteTexture(ot.__webglTexture),a.memory.textures--),i.remove(q[lt])}i.remove(D)}let I=0;function V(){I=0}function W(){const D=I;return D>=s.maxTextures&&console.warn("THREE.WebGLTextures: Trying to use "+D+" texture units while this GPU supports only "+s.maxTextures),I+=1,D}function Z(D){const S=[];return S.push(D.wrapS),S.push(D.wrapT),S.push(D.wrapR||0),S.push(D.magFilter),S.push(D.minFilter),S.push(D.anisotropy),S.push(D.internalFormat),S.push(D.format),S.push(D.type),S.push(D.generateMipmaps),S.push(D.premultiplyAlpha),S.push(D.flipY),S.push(D.unpackAlignment),S.push(D.colorSpace),S.join()}function et(D,S){const q=i.get(D);if(D.isVideoTexture&&Vt(D),D.isRenderTargetTexture===!1&&D.version>0&&q.__version!==D.version){const lt=D.image;if(lt===null)console.warn("THREE.WebGLRenderer: Texture marked for update but no image data found.");else if(lt.complete===!1)console.warn("THREE.WebGLRenderer: Texture marked for update but image is incomplete");else{X(q,D,S);return}}e.bindTexture(n.TEXTURE_2D,q.__webglTexture,n.TEXTURE0+S)}function z(D,S){const q=i.get(D);if(D.version>0&&q.__version!==D.version){X(q,D,S);return}e.bindTexture(n.TEXTURE_2D_ARRAY,q.__webglTexture,n.TEXTURE0+S)}function it(D,S){const q=i.get(D);if(D.version>0&&q.__version!==D.version){X(q,D,S);return}e.bindTexture(n.TEXTURE_3D,q.__webglTexture,n.TEXTURE0+S)}function j(D,S){const q=i.get(D);if(D.version>0&&q.__version!==D.version){Q(q,D,S);return}e.bindTexture(n.TEXTURE_CUBE_MAP,q.__webglTexture,n.TEXTURE0+S)}const ut={[Ki]:n.REPEAT,[zn]:n.CLAMP_TO_EDGE,[ml]:n.MIRRORED_REPEAT},mt={[An]:n.NEAREST,[kp]:n.NEAREST_MIPMAP_NEAREST,[Hr]:n.NEAREST_MIPMAP_LINEAR,[wn]:n.LINEAR,[mo]:n.LINEAR_MIPMAP_NEAREST,[oi]:n.LINEAR_MIPMAP_LINEAR},gt={[Vp]:n.NEVER,[Xp]:n.ALWAYS,[Hp]:n.LESS,[nd]:n.LEQUAL,[Gp]:n.EQUAL,[jp]:n.GEQUAL,[Wp]:n.GREATER,[$p]:n.NOTEQUAL};function Lt(D,S){if(S.type===jn&&t.has("OES_texture_float_linear")===!1&&(S.magFilter===wn||S.magFilter===mo||S.magFilter===Hr||S.magFilter===oi||S.minFilter===wn||S.minFilter===mo||S.minFilter===Hr||S.minFilter===oi)&&console.warn("THREE.WebGLRenderer: Unable to use linear filtering with floating point textures. OES_texture_float_linear not supported on this device."),n.texParameteri(D,n.TEXTURE_WRAP_S,ut[S.wrapS]),n.texParameteri(D,n.TEXTURE_WRAP_T,ut[S.wrapT]),(D===n.TEXTURE_3D||D===n.TEXTURE_2D_ARRAY)&&n.texParameteri(D,n.TEXTURE_WRAP_R,ut[S.wrapR]),n.texParameteri(D,n.TEXTURE_MAG_FILTER,mt[S.magFilter]),n.texParameteri(D,n.TEXTURE_MIN_FILTER,mt[S.minFilter]),S.compareFunction&&(n.texParameteri(D,n.TEXTURE_COMPARE_MODE,n.COMPARE_REF_TO_TEXTURE),n.texParameteri(D,n.TEXTURE_COMPARE_FUNC,gt[S.compareFunction])),t.has("EXT_texture_filter_anisotropic")===!0){if(S.magFilter===An||S.minFilter!==Hr&&S.minFilter!==oi||S.type===jn&&t.has("OES_texture_float_linear")===!1)return;if(S.anisotropy>1||i.get(S).__currentAnisotropy){const q=t.get("EXT_texture_filter_anisotropic");n.texParameterf(D,q.TEXTURE_MAX_ANISOTROPY_EXT,Math.min(S.anisotropy,s.getMaxAnisotropy())),i.get(S).__currentAnisotropy=S.anisotropy}}}function At(D,S){let q=!1;D.__webglInit===void 0&&(D.__webglInit=!0,S.addEventListener("dispose",F));const lt=S.source;let ht=d.get(lt);ht===void 0&&(ht={},d.set(lt,ht));const ot=Z(S);if(ot!==D.__cacheKey){ht[ot]===void 0&&(ht[ot]={texture:n.createTexture(),usedTimes:0},a.memory.textures++,q=!0),ht[ot].usedTimes++;const Ft=ht[D.__cacheKey];Ft!==void 0&&(ht[D.__cacheKey].usedTimes--,Ft.usedTimes===0&&y(S)),D.__cacheKey=ot,D.__webglTexture=ht[ot].texture}return q}function X(D,S,q){let lt=n.TEXTURE_2D;(S.isDataArrayTexture||S.isCompressedArrayTexture)&&(lt=n.TEXTURE_2D_ARRAY),S.isData3DTexture&&(lt=n.TEXTURE_3D);const ht=At(D,S),ot=S.source;e.bindTexture(lt,D.__webglTexture,n.TEXTURE0+q);const Ft=i.get(ot);if(ot.version!==Ft.__version||ht===!0){e.activeTexture(n.TEXTURE0+q);const St=_e.getPrimaries(_e.workingColorSpace),Pt=S.colorSpace===Ai?null:_e.getPrimaries(S.colorSpace),ve=S.colorSpace===Ai||St===Pt?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,S.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,S.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,S.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,ve);let pt=_(S.image,!1,s.maxTextureSize);pt=Le(S,pt);const Nt=r.convert(S.format,S.colorSpace),Xt=r.convert(S.type);let Zt=L(S.internalFormat,Nt,Xt,S.colorSpace,S.isVideoTexture);Lt(lt,S);let Dt;const ge=S.mipmaps,ae=S.isVideoTexture!==!0,Te=Ft.__version===void 0||ht===!0,G=ot.dataReady,Mt=N(S,pt);if(S.isDepthTexture)Zt=w(S.format===qs,S.type),Te&&(ae?e.texStorage2D(n.TEXTURE_2D,1,Zt,pt.width,pt.height):e.texImage2D(n.TEXTURE_2D,0,Zt,pt.width,pt.height,0,Nt,Xt,null));else if(S.isDataTexture)if(ge.length>0){ae&&Te&&e.texStorage2D(n.TEXTURE_2D,Mt,Zt,ge[0].width,ge[0].height);for(let tt=0,ct=ge.length;tt<ct;tt++)Dt=ge[tt],ae?G&&e.texSubImage2D(n.TEXTURE_2D,tt,0,0,Dt.width,Dt.height,Nt,Xt,Dt.data):e.texImage2D(n.TEXTURE_2D,tt,Zt,Dt.width,Dt.height,0,Nt,Xt,Dt.data);S.generateMipmaps=!1}else ae?(Te&&e.texStorage2D(n.TEXTURE_2D,Mt,Zt,pt.width,pt.height),G&&e.texSubImage2D(n.TEXTURE_2D,0,0,0,pt.width,pt.height,Nt,Xt,pt.data)):e.texImage2D(n.TEXTURE_2D,0,Zt,pt.width,pt.height,0,Nt,Xt,pt.data);else if(S.isCompressedTexture)if(S.isCompressedArrayTexture){ae&&Te&&e.texStorage3D(n.TEXTURE_2D_ARRAY,Mt,Zt,ge[0].width,ge[0].height,pt.depth);for(let tt=0,ct=ge.length;tt<ct;tt++)if(Dt=ge[tt],S.format!==Pn)if(Nt!==null)if(ae){if(G)if(S.layerUpdates.size>0){const wt=zu(Dt.width,Dt.height,S.format,S.type);for(const Et of S.layerUpdates){const re=Dt.data.subarray(Et*wt/Dt.data.BYTES_PER_ELEMENT,(Et+1)*wt/Dt.data.BYTES_PER_ELEMENT);e.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,tt,0,0,Et,Dt.width,Dt.height,1,Nt,re)}S.clearLayerUpdates()}else e.compressedTexSubImage3D(n.TEXTURE_2D_ARRAY,tt,0,0,0,Dt.width,Dt.height,pt.depth,Nt,Dt.data)}else e.compressedTexImage3D(n.TEXTURE_2D_ARRAY,tt,Zt,Dt.width,Dt.height,pt.depth,0,Dt.data,0,0);else console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()");else ae?G&&e.texSubImage3D(n.TEXTURE_2D_ARRAY,tt,0,0,0,Dt.width,Dt.height,pt.depth,Nt,Xt,Dt.data):e.texImage3D(n.TEXTURE_2D_ARRAY,tt,Zt,Dt.width,Dt.height,pt.depth,0,Nt,Xt,Dt.data)}else{ae&&Te&&e.texStorage2D(n.TEXTURE_2D,Mt,Zt,ge[0].width,ge[0].height);for(let tt=0,ct=ge.length;tt<ct;tt++)Dt=ge[tt],S.format!==Pn?Nt!==null?ae?G&&e.compressedTexSubImage2D(n.TEXTURE_2D,tt,0,0,Dt.width,Dt.height,Nt,Dt.data):e.compressedTexImage2D(n.TEXTURE_2D,tt,Zt,Dt.width,Dt.height,0,Dt.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .uploadTexture()"):ae?G&&e.texSubImage2D(n.TEXTURE_2D,tt,0,0,Dt.width,Dt.height,Nt,Xt,Dt.data):e.texImage2D(n.TEXTURE_2D,tt,Zt,Dt.width,Dt.height,0,Nt,Xt,Dt.data)}else if(S.isDataArrayTexture)if(ae){if(Te&&e.texStorage3D(n.TEXTURE_2D_ARRAY,Mt,Zt,pt.width,pt.height,pt.depth),G)if(S.layerUpdates.size>0){const tt=zu(pt.width,pt.height,S.format,S.type);for(const ct of S.layerUpdates){const wt=pt.data.subarray(ct*tt/pt.data.BYTES_PER_ELEMENT,(ct+1)*tt/pt.data.BYTES_PER_ELEMENT);e.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,ct,pt.width,pt.height,1,Nt,Xt,wt)}S.clearLayerUpdates()}else e.texSubImage3D(n.TEXTURE_2D_ARRAY,0,0,0,0,pt.width,pt.height,pt.depth,Nt,Xt,pt.data)}else e.texImage3D(n.TEXTURE_2D_ARRAY,0,Zt,pt.width,pt.height,pt.depth,0,Nt,Xt,pt.data);else if(S.isData3DTexture)ae?(Te&&e.texStorage3D(n.TEXTURE_3D,Mt,Zt,pt.width,pt.height,pt.depth),G&&e.texSubImage3D(n.TEXTURE_3D,0,0,0,0,pt.width,pt.height,pt.depth,Nt,Xt,pt.data)):e.texImage3D(n.TEXTURE_3D,0,Zt,pt.width,pt.height,pt.depth,0,Nt,Xt,pt.data);else if(S.isFramebufferTexture){if(Te)if(ae)e.texStorage2D(n.TEXTURE_2D,Mt,Zt,pt.width,pt.height);else{let tt=pt.width,ct=pt.height;for(let wt=0;wt<Mt;wt++)e.texImage2D(n.TEXTURE_2D,wt,Zt,tt,ct,0,Nt,Xt,null),tt>>=1,ct>>=1}}else if(ge.length>0){if(ae&&Te){const tt=zt(ge[0]);e.texStorage2D(n.TEXTURE_2D,Mt,Zt,tt.width,tt.height)}for(let tt=0,ct=ge.length;tt<ct;tt++)Dt=ge[tt],ae?G&&e.texSubImage2D(n.TEXTURE_2D,tt,0,0,Nt,Xt,Dt):e.texImage2D(n.TEXTURE_2D,tt,Zt,Nt,Xt,Dt);S.generateMipmaps=!1}else if(ae){if(Te){const tt=zt(pt);e.texStorage2D(n.TEXTURE_2D,Mt,Zt,tt.width,tt.height)}G&&e.texSubImage2D(n.TEXTURE_2D,0,0,0,Nt,Xt,pt)}else e.texImage2D(n.TEXTURE_2D,0,Zt,Nt,Xt,pt);m(S)&&f(lt),Ft.__version=ot.version,S.onUpdate&&S.onUpdate(S)}D.__version=S.version}function Q(D,S,q){if(S.image.length!==6)return;const lt=At(D,S),ht=S.source;e.bindTexture(n.TEXTURE_CUBE_MAP,D.__webglTexture,n.TEXTURE0+q);const ot=i.get(ht);if(ht.version!==ot.__version||lt===!0){e.activeTexture(n.TEXTURE0+q);const Ft=_e.getPrimaries(_e.workingColorSpace),St=S.colorSpace===Ai?null:_e.getPrimaries(S.colorSpace),Pt=S.colorSpace===Ai||Ft===St?n.NONE:n.BROWSER_DEFAULT_WEBGL;n.pixelStorei(n.UNPACK_FLIP_Y_WEBGL,S.flipY),n.pixelStorei(n.UNPACK_PREMULTIPLY_ALPHA_WEBGL,S.premultiplyAlpha),n.pixelStorei(n.UNPACK_ALIGNMENT,S.unpackAlignment),n.pixelStorei(n.UNPACK_COLORSPACE_CONVERSION_WEBGL,Pt);const ve=S.isCompressedTexture||S.image[0].isCompressedTexture,pt=S.image[0]&&S.image[0].isDataTexture,Nt=[];for(let ct=0;ct<6;ct++)!ve&&!pt?Nt[ct]=_(S.image[ct],!0,s.maxCubemapSize):Nt[ct]=pt?S.image[ct].image:S.image[ct],Nt[ct]=Le(S,Nt[ct]);const Xt=Nt[0],Zt=r.convert(S.format,S.colorSpace),Dt=r.convert(S.type),ge=L(S.internalFormat,Zt,Dt,S.colorSpace),ae=S.isVideoTexture!==!0,Te=ot.__version===void 0||lt===!0,G=ht.dataReady;let Mt=N(S,Xt);Lt(n.TEXTURE_CUBE_MAP,S);let tt;if(ve){ae&&Te&&e.texStorage2D(n.TEXTURE_CUBE_MAP,Mt,ge,Xt.width,Xt.height);for(let ct=0;ct<6;ct++){tt=Nt[ct].mipmaps;for(let wt=0;wt<tt.length;wt++){const Et=tt[wt];S.format!==Pn?Zt!==null?ae?G&&e.compressedTexSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt,0,0,Et.width,Et.height,Zt,Et.data):e.compressedTexImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt,ge,Et.width,Et.height,0,Et.data):console.warn("THREE.WebGLRenderer: Attempt to load unsupported compressed texture format in .setTextureCube()"):ae?G&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt,0,0,Et.width,Et.height,Zt,Dt,Et.data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt,ge,Et.width,Et.height,0,Zt,Dt,Et.data)}}}else{if(tt=S.mipmaps,ae&&Te){tt.length>0&&Mt++;const ct=zt(Nt[0]);e.texStorage2D(n.TEXTURE_CUBE_MAP,Mt,ge,ct.width,ct.height)}for(let ct=0;ct<6;ct++)if(pt){ae?G&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,Nt[ct].width,Nt[ct].height,Zt,Dt,Nt[ct].data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,ge,Nt[ct].width,Nt[ct].height,0,Zt,Dt,Nt[ct].data);for(let wt=0;wt<tt.length;wt++){const re=tt[wt].image[ct].image;ae?G&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt+1,0,0,re.width,re.height,Zt,Dt,re.data):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt+1,ge,re.width,re.height,0,Zt,Dt,re.data)}}else{ae?G&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,0,0,Zt,Dt,Nt[ct]):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,0,ge,Zt,Dt,Nt[ct]);for(let wt=0;wt<tt.length;wt++){const Et=tt[wt];ae?G&&e.texSubImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt+1,0,0,Zt,Dt,Et.image[ct]):e.texImage2D(n.TEXTURE_CUBE_MAP_POSITIVE_X+ct,wt+1,ge,Zt,Dt,Et.image[ct])}}}m(S)&&f(n.TEXTURE_CUBE_MAP),ot.__version=ht.version,S.onUpdate&&S.onUpdate(S)}D.__version=S.version}function nt(D,S,q,lt,ht,ot){const Ft=r.convert(q.format,q.colorSpace),St=r.convert(q.type),Pt=L(q.internalFormat,Ft,St,q.colorSpace),ve=i.get(S),pt=i.get(q);if(pt.__renderTarget=S,!ve.__hasExternalTextures){const Nt=Math.max(1,S.width>>ot),Xt=Math.max(1,S.height>>ot);ht===n.TEXTURE_3D||ht===n.TEXTURE_2D_ARRAY?e.texImage3D(ht,ot,Pt,Nt,Xt,S.depth,0,Ft,St,null):e.texImage2D(ht,ot,Pt,Nt,Xt,0,Ft,St,null)}e.bindFramebuffer(n.FRAMEBUFFER,D),pe(S)?o.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,lt,ht,pt.__webglTexture,0,fe(S)):(ht===n.TEXTURE_2D||ht>=n.TEXTURE_CUBE_MAP_POSITIVE_X&&ht<=n.TEXTURE_CUBE_MAP_NEGATIVE_Z)&&n.framebufferTexture2D(n.FRAMEBUFFER,lt,ht,pt.__webglTexture,ot),e.bindFramebuffer(n.FRAMEBUFFER,null)}function st(D,S,q){if(n.bindRenderbuffer(n.RENDERBUFFER,D),S.depthBuffer){const lt=S.depthTexture,ht=lt&&lt.isDepthTexture?lt.type:null,ot=w(S.stencilBuffer,ht),Ft=S.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,St=fe(S);pe(S)?o.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,St,ot,S.width,S.height):q?n.renderbufferStorageMultisample(n.RENDERBUFFER,St,ot,S.width,S.height):n.renderbufferStorage(n.RENDERBUFFER,ot,S.width,S.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,Ft,n.RENDERBUFFER,D)}else{const lt=S.textures;for(let ht=0;ht<lt.length;ht++){const ot=lt[ht],Ft=r.convert(ot.format,ot.colorSpace),St=r.convert(ot.type),Pt=L(ot.internalFormat,Ft,St,ot.colorSpace),ve=fe(S);q&&pe(S)===!1?n.renderbufferStorageMultisample(n.RENDERBUFFER,ve,Pt,S.width,S.height):pe(S)?o.renderbufferStorageMultisampleEXT(n.RENDERBUFFER,ve,Pt,S.width,S.height):n.renderbufferStorage(n.RENDERBUFFER,Pt,S.width,S.height)}}n.bindRenderbuffer(n.RENDERBUFFER,null)}function vt(D,S){if(S&&S.isWebGLCubeRenderTarget)throw new Error("Depth Texture with cube render targets is not supported");if(e.bindFramebuffer(n.FRAMEBUFFER,D),!(S.depthTexture&&S.depthTexture.isDepthTexture))throw new Error("renderTarget.depthTexture must be an instance of THREE.DepthTexture");const lt=i.get(S.depthTexture);lt.__renderTarget=S,(!lt.__webglTexture||S.depthTexture.image.width!==S.width||S.depthTexture.image.height!==S.height)&&(S.depthTexture.image.width=S.width,S.depthTexture.image.height=S.height,S.depthTexture.needsUpdate=!0),et(S.depthTexture,0);const ht=lt.__webglTexture,ot=fe(S);if(S.depthTexture.format===Os)pe(S)?o.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,ht,0,ot):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_ATTACHMENT,n.TEXTURE_2D,ht,0);else if(S.depthTexture.format===qs)pe(S)?o.framebufferTexture2DMultisampleEXT(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,ht,0,ot):n.framebufferTexture2D(n.FRAMEBUFFER,n.DEPTH_STENCIL_ATTACHMENT,n.TEXTURE_2D,ht,0);else throw new Error("Unknown depthTexture format")}function se(D){const S=i.get(D),q=D.isWebGLCubeRenderTarget===!0;if(S.__boundDepthTexture!==D.depthTexture){const lt=D.depthTexture;if(S.__depthDisposeCallback&&S.__depthDisposeCallback(),lt){const ht=()=>{delete S.__boundDepthTexture,delete S.__depthDisposeCallback,lt.removeEventListener("dispose",ht)};lt.addEventListener("dispose",ht),S.__depthDisposeCallback=ht}S.__boundDepthTexture=lt}if(D.depthTexture&&!S.__autoAllocateDepthBuffer){if(q)throw new Error("target.depthTexture not supported in Cube render targets");vt(S.__webglFramebuffer,D)}else if(q){S.__webglDepthbuffer=[];for(let lt=0;lt<6;lt++)if(e.bindFramebuffer(n.FRAMEBUFFER,S.__webglFramebuffer[lt]),S.__webglDepthbuffer[lt]===void 0)S.__webglDepthbuffer[lt]=n.createRenderbuffer(),st(S.__webglDepthbuffer[lt],D,!1);else{const ht=D.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,ot=S.__webglDepthbuffer[lt];n.bindRenderbuffer(n.RENDERBUFFER,ot),n.framebufferRenderbuffer(n.FRAMEBUFFER,ht,n.RENDERBUFFER,ot)}}else if(e.bindFramebuffer(n.FRAMEBUFFER,S.__webglFramebuffer),S.__webglDepthbuffer===void 0)S.__webglDepthbuffer=n.createRenderbuffer(),st(S.__webglDepthbuffer,D,!1);else{const lt=D.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,ht=S.__webglDepthbuffer;n.bindRenderbuffer(n.RENDERBUFFER,ht),n.framebufferRenderbuffer(n.FRAMEBUFFER,lt,n.RENDERBUFFER,ht)}e.bindFramebuffer(n.FRAMEBUFFER,null)}function Wt(D,S,q){const lt=i.get(D);S!==void 0&&nt(lt.__webglFramebuffer,D,D.texture,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,0),q!==void 0&&se(D)}function Fe(D){const S=D.texture,q=i.get(D),lt=i.get(S);D.addEventListener("dispose",U);const ht=D.textures,ot=D.isWebGLCubeRenderTarget===!0,Ft=ht.length>1;if(Ft||(lt.__webglTexture===void 0&&(lt.__webglTexture=n.createTexture()),lt.__version=S.version,a.memory.textures++),ot){q.__webglFramebuffer=[];for(let St=0;St<6;St++)if(S.mipmaps&&S.mipmaps.length>0){q.__webglFramebuffer[St]=[];for(let Pt=0;Pt<S.mipmaps.length;Pt++)q.__webglFramebuffer[St][Pt]=n.createFramebuffer()}else q.__webglFramebuffer[St]=n.createFramebuffer()}else{if(S.mipmaps&&S.mipmaps.length>0){q.__webglFramebuffer=[];for(let St=0;St<S.mipmaps.length;St++)q.__webglFramebuffer[St]=n.createFramebuffer()}else q.__webglFramebuffer=n.createFramebuffer();if(Ft)for(let St=0,Pt=ht.length;St<Pt;St++){const ve=i.get(ht[St]);ve.__webglTexture===void 0&&(ve.__webglTexture=n.createTexture(),a.memory.textures++)}if(D.samples>0&&pe(D)===!1){q.__webglMultisampledFramebuffer=n.createFramebuffer(),q.__webglColorRenderbuffer=[],e.bindFramebuffer(n.FRAMEBUFFER,q.__webglMultisampledFramebuffer);for(let St=0;St<ht.length;St++){const Pt=ht[St];q.__webglColorRenderbuffer[St]=n.createRenderbuffer(),n.bindRenderbuffer(n.RENDERBUFFER,q.__webglColorRenderbuffer[St]);const ve=r.convert(Pt.format,Pt.colorSpace),pt=r.convert(Pt.type),Nt=L(Pt.internalFormat,ve,pt,Pt.colorSpace,D.isXRRenderTarget===!0),Xt=fe(D);n.renderbufferStorageMultisample(n.RENDERBUFFER,Xt,Nt,D.width,D.height),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+St,n.RENDERBUFFER,q.__webglColorRenderbuffer[St])}n.bindRenderbuffer(n.RENDERBUFFER,null),D.depthBuffer&&(q.__webglDepthRenderbuffer=n.createRenderbuffer(),st(q.__webglDepthRenderbuffer,D,!0)),e.bindFramebuffer(n.FRAMEBUFFER,null)}}if(ot){e.bindTexture(n.TEXTURE_CUBE_MAP,lt.__webglTexture),Lt(n.TEXTURE_CUBE_MAP,S);for(let St=0;St<6;St++)if(S.mipmaps&&S.mipmaps.length>0)for(let Pt=0;Pt<S.mipmaps.length;Pt++)nt(q.__webglFramebuffer[St][Pt],D,S,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+St,Pt);else nt(q.__webglFramebuffer[St],D,S,n.COLOR_ATTACHMENT0,n.TEXTURE_CUBE_MAP_POSITIVE_X+St,0);m(S)&&f(n.TEXTURE_CUBE_MAP),e.unbindTexture()}else if(Ft){for(let St=0,Pt=ht.length;St<Pt;St++){const ve=ht[St],pt=i.get(ve);e.bindTexture(n.TEXTURE_2D,pt.__webglTexture),Lt(n.TEXTURE_2D,ve),nt(q.__webglFramebuffer,D,ve,n.COLOR_ATTACHMENT0+St,n.TEXTURE_2D,0),m(ve)&&f(n.TEXTURE_2D)}e.unbindTexture()}else{let St=n.TEXTURE_2D;if((D.isWebGL3DRenderTarget||D.isWebGLArrayRenderTarget)&&(St=D.isWebGL3DRenderTarget?n.TEXTURE_3D:n.TEXTURE_2D_ARRAY),e.bindTexture(St,lt.__webglTexture),Lt(St,S),S.mipmaps&&S.mipmaps.length>0)for(let Pt=0;Pt<S.mipmaps.length;Pt++)nt(q.__webglFramebuffer[Pt],D,S,n.COLOR_ATTACHMENT0,St,Pt);else nt(q.__webglFramebuffer,D,S,n.COLOR_ATTACHMENT0,St,0);m(S)&&f(St),e.unbindTexture()}D.depthBuffer&&se(D)}function Ie(D){const S=D.textures;for(let q=0,lt=S.length;q<lt;q++){const ht=S[q];if(m(ht)){const ot=P(D),Ft=i.get(ht).__webglTexture;e.bindTexture(ot,Ft),f(ot),e.unbindTexture()}}}const me=[],B=[];function pn(D){if(D.samples>0){if(pe(D)===!1){const S=D.textures,q=D.width,lt=D.height;let ht=n.COLOR_BUFFER_BIT;const ot=D.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT,Ft=i.get(D),St=S.length>1;if(St)for(let Pt=0;Pt<S.length;Pt++)e.bindFramebuffer(n.FRAMEBUFFER,Ft.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+Pt,n.RENDERBUFFER,null),e.bindFramebuffer(n.FRAMEBUFFER,Ft.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+Pt,n.TEXTURE_2D,null,0);e.bindFramebuffer(n.READ_FRAMEBUFFER,Ft.__webglMultisampledFramebuffer),e.bindFramebuffer(n.DRAW_FRAMEBUFFER,Ft.__webglFramebuffer);for(let Pt=0;Pt<S.length;Pt++){if(D.resolveDepthBuffer&&(D.depthBuffer&&(ht|=n.DEPTH_BUFFER_BIT),D.stencilBuffer&&D.resolveStencilBuffer&&(ht|=n.STENCIL_BUFFER_BIT)),St){n.framebufferRenderbuffer(n.READ_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.RENDERBUFFER,Ft.__webglColorRenderbuffer[Pt]);const ve=i.get(S[Pt]).__webglTexture;n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0,n.TEXTURE_2D,ve,0)}n.blitFramebuffer(0,0,q,lt,0,0,q,lt,ht,n.NEAREST),l===!0&&(me.length=0,B.length=0,me.push(n.COLOR_ATTACHMENT0+Pt),D.depthBuffer&&D.resolveDepthBuffer===!1&&(me.push(ot),B.push(ot),n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,B)),n.invalidateFramebuffer(n.READ_FRAMEBUFFER,me))}if(e.bindFramebuffer(n.READ_FRAMEBUFFER,null),e.bindFramebuffer(n.DRAW_FRAMEBUFFER,null),St)for(let Pt=0;Pt<S.length;Pt++){e.bindFramebuffer(n.FRAMEBUFFER,Ft.__webglMultisampledFramebuffer),n.framebufferRenderbuffer(n.FRAMEBUFFER,n.COLOR_ATTACHMENT0+Pt,n.RENDERBUFFER,Ft.__webglColorRenderbuffer[Pt]);const ve=i.get(S[Pt]).__webglTexture;e.bindFramebuffer(n.FRAMEBUFFER,Ft.__webglFramebuffer),n.framebufferTexture2D(n.DRAW_FRAMEBUFFER,n.COLOR_ATTACHMENT0+Pt,n.TEXTURE_2D,ve,0)}e.bindFramebuffer(n.DRAW_FRAMEBUFFER,Ft.__webglMultisampledFramebuffer)}else if(D.depthBuffer&&D.resolveDepthBuffer===!1&&l){const S=D.stencilBuffer?n.DEPTH_STENCIL_ATTACHMENT:n.DEPTH_ATTACHMENT;n.invalidateFramebuffer(n.DRAW_FRAMEBUFFER,[S])}}}function fe(D){return Math.min(s.maxSamples,D.samples)}function pe(D){const S=i.get(D);return D.samples>0&&t.has("WEBGL_multisampled_render_to_texture")===!0&&S.__useRenderToTexture!==!1}function Vt(D){const S=a.render.frame;u.get(D)!==S&&(u.set(D,S),D.update())}function Le(D,S){const q=D.colorSpace,lt=D.format,ht=D.type;return D.isCompressedTexture===!0||D.isVideoTexture===!0||q!==Ys&&q!==Ai&&(_e.getTransfer(q)===Ce?(lt!==Pn||ht!==hi)&&console.warn("THREE.WebGLTextures: sRGB encoded textures have to use RGBAFormat and UnsignedByteType."):console.error("THREE.WebGLTextures: Unsupported texture color space:",q)),S}function zt(D){return typeof HTMLImageElement<"u"&&D instanceof HTMLImageElement?(c.width=D.naturalWidth||D.width,c.height=D.naturalHeight||D.height):typeof VideoFrame<"u"&&D instanceof VideoFrame?(c.width=D.displayWidth,c.height=D.displayHeight):(c.width=D.width,c.height=D.height),c}this.allocateTextureUnit=W,this.resetTextureUnits=V,this.setTexture2D=et,this.setTexture2DArray=z,this.setTexture3D=it,this.setTextureCube=j,this.rebindTextures=Wt,this.setupRenderTarget=Fe,this.updateRenderTargetMipmap=Ie,this.updateMultisampleRenderTarget=pn,this.setupDepthRenderbuffer=se,this.setupFrameBufferTexture=nt,this.useMultisampledRTT=pe}function zx(n,t){function e(i,s=Ai){let r;const a=_e.getTransfer(s);if(i===hi)return n.UNSIGNED_BYTE;if(i===ac)return n.UNSIGNED_SHORT_4_4_4_4;if(i===oc)return n.UNSIGNED_SHORT_5_5_5_1;if(i===qh)return n.UNSIGNED_INT_5_9_9_9_REV;if(i===jh)return n.BYTE;if(i===Xh)return n.SHORT;if(i===Mr)return n.UNSIGNED_SHORT;if(i===rc)return n.INT;if(i===ts)return n.UNSIGNED_INT;if(i===jn)return n.FLOAT;if(i===Rr)return n.HALF_FLOAT;if(i===Yh)return n.ALPHA;if(i===Kh)return n.RGB;if(i===Pn)return n.RGBA;if(i===Jh)return n.LUMINANCE;if(i===Zh)return n.LUMINANCE_ALPHA;if(i===Os)return n.DEPTH_COMPONENT;if(i===qs)return n.DEPTH_STENCIL;if(i===Qh)return n.RED;if(i===lc)return n.RED_INTEGER;if(i===td)return n.RG;if(i===cc)return n.RG_INTEGER;if(i===uc)return n.RGBA_INTEGER;if(i===Aa||i===Ca||i===Ra||i===Pa)if(a===Ce)if(r=t.get("WEBGL_compressed_texture_s3tc_srgb"),r!==null){if(i===Aa)return r.COMPRESSED_SRGB_S3TC_DXT1_EXT;if(i===Ca)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT;if(i===Ra)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT;if(i===Pa)return r.COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT}else return null;else if(r=t.get("WEBGL_compressed_texture_s3tc"),r!==null){if(i===Aa)return r.COMPRESSED_RGB_S3TC_DXT1_EXT;if(i===Ca)return r.COMPRESSED_RGBA_S3TC_DXT1_EXT;if(i===Ra)return r.COMPRESSED_RGBA_S3TC_DXT3_EXT;if(i===Pa)return r.COMPRESSED_RGBA_S3TC_DXT5_EXT}else return null;if(i===gl||i===_l||i===vl||i===xl)if(r=t.get("WEBGL_compressed_texture_pvrtc"),r!==null){if(i===gl)return r.COMPRESSED_RGB_PVRTC_4BPPV1_IMG;if(i===_l)return r.COMPRESSED_RGB_PVRTC_2BPPV1_IMG;if(i===vl)return r.COMPRESSED_RGBA_PVRTC_4BPPV1_IMG;if(i===xl)return r.COMPRESSED_RGBA_PVRTC_2BPPV1_IMG}else return null;if(i===bl||i===yl||i===Ml)if(r=t.get("WEBGL_compressed_texture_etc"),r!==null){if(i===bl||i===yl)return a===Ce?r.COMPRESSED_SRGB8_ETC2:r.COMPRESSED_RGB8_ETC2;if(i===Ml)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ETC2_EAC:r.COMPRESSED_RGBA8_ETC2_EAC}else return null;if(i===Sl||i===El||i===Tl||i===wl||i===Al||i===Cl||i===Rl||i===Pl||i===Ll||i===Nl||i===Dl||i===Il||i===Ul||i===kl)if(r=t.get("WEBGL_compressed_texture_astc"),r!==null){if(i===Sl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_4x4_KHR:r.COMPRESSED_RGBA_ASTC_4x4_KHR;if(i===El)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x4_KHR:r.COMPRESSED_RGBA_ASTC_5x4_KHR;if(i===Tl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_5x5_KHR:r.COMPRESSED_RGBA_ASTC_5x5_KHR;if(i===wl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x5_KHR:r.COMPRESSED_RGBA_ASTC_6x5_KHR;if(i===Al)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_6x6_KHR:r.COMPRESSED_RGBA_ASTC_6x6_KHR;if(i===Cl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x5_KHR:r.COMPRESSED_RGBA_ASTC_8x5_KHR;if(i===Rl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x6_KHR:r.COMPRESSED_RGBA_ASTC_8x6_KHR;if(i===Pl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_8x8_KHR:r.COMPRESSED_RGBA_ASTC_8x8_KHR;if(i===Ll)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x5_KHR:r.COMPRESSED_RGBA_ASTC_10x5_KHR;if(i===Nl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x6_KHR:r.COMPRESSED_RGBA_ASTC_10x6_KHR;if(i===Dl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x8_KHR:r.COMPRESSED_RGBA_ASTC_10x8_KHR;if(i===Il)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_10x10_KHR:r.COMPRESSED_RGBA_ASTC_10x10_KHR;if(i===Ul)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x10_KHR:r.COMPRESSED_RGBA_ASTC_12x10_KHR;if(i===kl)return a===Ce?r.COMPRESSED_SRGB8_ALPHA8_ASTC_12x12_KHR:r.COMPRESSED_RGBA_ASTC_12x12_KHR}else return null;if(i===La||i===Fl||i===Ol)if(r=t.get("EXT_texture_compression_bptc"),r!==null){if(i===La)return a===Ce?r.COMPRESSED_SRGB_ALPHA_BPTC_UNORM_EXT:r.COMPRESSED_RGBA_BPTC_UNORM_EXT;if(i===Fl)return r.COMPRESSED_RGB_BPTC_SIGNED_FLOAT_EXT;if(i===Ol)return r.COMPRESSED_RGB_BPTC_UNSIGNED_FLOAT_EXT}else return null;if(i===ed||i===Bl||i===zl||i===Vl)if(r=t.get("EXT_texture_compression_rgtc"),r!==null){if(i===La)return r.COMPRESSED_RED_RGTC1_EXT;if(i===Bl)return r.COMPRESSED_SIGNED_RED_RGTC1_EXT;if(i===zl)return r.COMPRESSED_RED_GREEN_RGTC2_EXT;if(i===Vl)return r.COMPRESSED_SIGNED_RED_GREEN_RGTC2_EXT}else return null;return i===Xs?n.UNSIGNED_INT_24_8:n[i]!==void 0?n[i]:null}return{convert:e}}const Vx=`
void main() {

	gl_Position = vec4( position, 1.0 );

}`,Hx=`
uniform sampler2DArray depthColor;
uniform float depthWidth;
uniform float depthHeight;

void main() {

	vec2 coord = vec2( gl_FragCoord.x / depthWidth, gl_FragCoord.y / depthHeight );

	if ( coord.x >= 1.0 ) {

		gl_FragDepth = texture( depthColor, vec3( coord.x - 1.0, coord.y, 1 ) ).r;

	} else {

		gl_FragDepth = texture( depthColor, vec3( coord.x, coord.y, 0 ) ).r;

	}

}`;class Gx{constructor(){this.texture=null,this.mesh=null,this.depthNear=0,this.depthFar=0}init(t,e,i){if(this.texture===null){const s=new ln,r=t.properties.get(s);r.__webglTexture=e.texture,(e.depthNear!==i.depthNear||e.depthFar!==i.depthFar)&&(this.depthNear=e.depthNear,this.depthFar=e.depthFar),this.texture=s}}getMesh(t){if(this.texture!==null&&this.mesh===null){const e=t.cameras[0].viewport,i=new Ni({vertexShader:Vx,fragmentShader:Hx,uniforms:{depthColor:{value:this.texture},depthWidth:{value:e.z},depthHeight:{value:e.w}}});this.mesh=new Re(new Nr(20,20),i)}return this.mesh}reset(){this.texture=null,this.mesh=null}getDepthTexture(){return this.texture}}class Wx extends rs{constructor(t,e){super();const i=this;let s=null,r=1,a=null,o="local-floor",l=1,c=null,u=null,h=null,d=null,p=null,g=null;const _=new Gx,m=e.getContextAttributes();let f=null,P=null;const L=[],w=[],N=new Yt;let F=null;const U=new an;U.viewport=new Ee;const k=new an;k.viewport=new Ee;const y=[U,k],M=new ug;let I=null,V=null;this.cameraAutoUpdate=!0,this.enabled=!1,this.isPresenting=!1,this.getController=function(X){let Q=L[X];return Q===void 0&&(Q=new ko,L[X]=Q),Q.getTargetRaySpace()},this.getControllerGrip=function(X){let Q=L[X];return Q===void 0&&(Q=new ko,L[X]=Q),Q.getGripSpace()},this.getHand=function(X){let Q=L[X];return Q===void 0&&(Q=new ko,L[X]=Q),Q.getHandSpace()};function W(X){const Q=w.indexOf(X.inputSource);if(Q===-1)return;const nt=L[Q];nt!==void 0&&(nt.update(X.inputSource,X.frame,c||a),nt.dispatchEvent({type:X.type,data:X.inputSource}))}function Z(){s.removeEventListener("select",W),s.removeEventListener("selectstart",W),s.removeEventListener("selectend",W),s.removeEventListener("squeeze",W),s.removeEventListener("squeezestart",W),s.removeEventListener("squeezeend",W),s.removeEventListener("end",Z),s.removeEventListener("inputsourceschange",et);for(let X=0;X<L.length;X++){const Q=w[X];Q!==null&&(w[X]=null,L[X].disconnect(Q))}I=null,V=null,_.reset(),t.setRenderTarget(f),p=null,d=null,h=null,s=null,P=null,At.stop(),i.isPresenting=!1,t.setPixelRatio(F),t.setSize(N.width,N.height,!1),i.dispatchEvent({type:"sessionend"})}this.setFramebufferScaleFactor=function(X){r=X,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change framebuffer scale while presenting.")},this.setReferenceSpaceType=function(X){o=X,i.isPresenting===!0&&console.warn("THREE.WebXRManager: Cannot change reference space type while presenting.")},this.getReferenceSpace=function(){return c||a},this.setReferenceSpace=function(X){c=X},this.getBaseLayer=function(){return d!==null?d:p},this.getBinding=function(){return h},this.getFrame=function(){return g},this.getSession=function(){return s},this.setSession=async function(X){if(s=X,s!==null){if(f=t.getRenderTarget(),s.addEventListener("select",W),s.addEventListener("selectstart",W),s.addEventListener("selectend",W),s.addEventListener("squeeze",W),s.addEventListener("squeezestart",W),s.addEventListener("squeezeend",W),s.addEventListener("end",Z),s.addEventListener("inputsourceschange",et),m.xrCompatible!==!0&&await e.makeXRCompatible(),F=t.getPixelRatio(),t.getSize(N),typeof XRWebGLBinding<"u"&&"createProjectionLayer"in XRWebGLBinding.prototype){let nt=null,st=null,vt=null;m.depth&&(vt=m.stencil?e.DEPTH24_STENCIL8:e.DEPTH_COMPONENT24,nt=m.stencil?qs:Os,st=m.stencil?Xs:ts);const se={colorFormat:e.RGBA8,depthFormat:vt,scaleFactor:r};h=new XRWebGLBinding(s,e),d=h.createProjectionLayer(se),s.updateRenderState({layers:[d]}),t.setPixelRatio(1),t.setSize(d.textureWidth,d.textureHeight,!1),P=new es(d.textureWidth,d.textureHeight,{format:Pn,type:hi,depthTexture:new gd(d.textureWidth,d.textureHeight,st,void 0,void 0,void 0,void 0,void 0,void 0,nt),stencilBuffer:m.stencil,colorSpace:t.outputColorSpace,samples:m.antialias?4:0,resolveDepthBuffer:d.ignoreDepthValues===!1,resolveStencilBuffer:d.ignoreDepthValues===!1})}else{const nt={antialias:m.antialias,alpha:!0,depth:m.depth,stencil:m.stencil,framebufferScaleFactor:r};p=new XRWebGLLayer(s,e,nt),s.updateRenderState({baseLayer:p}),t.setPixelRatio(1),t.setSize(p.framebufferWidth,p.framebufferHeight,!1),P=new es(p.framebufferWidth,p.framebufferHeight,{format:Pn,type:hi,colorSpace:t.outputColorSpace,stencilBuffer:m.stencil,resolveDepthBuffer:p.ignoreDepthValues===!1,resolveStencilBuffer:p.ignoreDepthValues===!1})}P.isXRRenderTarget=!0,this.setFoveation(l),c=null,a=await s.requestReferenceSpace(o),At.setContext(s),At.start(),i.isPresenting=!0,i.dispatchEvent({type:"sessionstart"})}},this.getEnvironmentBlendMode=function(){if(s!==null)return s.environmentBlendMode},this.getDepthTexture=function(){return _.getDepthTexture()};function et(X){for(let Q=0;Q<X.removed.length;Q++){const nt=X.removed[Q],st=w.indexOf(nt);st>=0&&(w[st]=null,L[st].disconnect(nt))}for(let Q=0;Q<X.added.length;Q++){const nt=X.added[Q];let st=w.indexOf(nt);if(st===-1){for(let se=0;se<L.length;se++)if(se>=w.length){w.push(nt),st=se;break}else if(w[se]===null){w[se]=nt,st=se;break}if(st===-1)break}const vt=L[st];vt&&vt.connect(nt)}}const z=new O,it=new O;function j(X,Q,nt){z.setFromMatrixPosition(Q.matrixWorld),it.setFromMatrixPosition(nt.matrixWorld);const st=z.distanceTo(it),vt=Q.projectionMatrix.elements,se=nt.projectionMatrix.elements,Wt=vt[14]/(vt[10]-1),Fe=vt[14]/(vt[10]+1),Ie=(vt[9]+1)/vt[5],me=(vt[9]-1)/vt[5],B=(vt[8]-1)/vt[0],pn=(se[8]+1)/se[0],fe=Wt*B,pe=Wt*pn,Vt=st/(-B+pn),Le=Vt*-B;if(Q.matrixWorld.decompose(X.position,X.quaternion,X.scale),X.translateX(Le),X.translateZ(Vt),X.matrixWorld.compose(X.position,X.quaternion,X.scale),X.matrixWorldInverse.copy(X.matrixWorld).invert(),vt[10]===-1)X.projectionMatrix.copy(Q.projectionMatrix),X.projectionMatrixInverse.copy(Q.projectionMatrixInverse);else{const zt=Wt+Vt,D=Fe+Vt,S=fe-Le,q=pe+(st-Le),lt=Ie*Fe/D*zt,ht=me*Fe/D*zt;X.projectionMatrix.makePerspective(S,q,lt,ht,zt,D),X.projectionMatrixInverse.copy(X.projectionMatrix).invert()}}function ut(X,Q){Q===null?X.matrixWorld.copy(X.matrix):X.matrixWorld.multiplyMatrices(Q.matrixWorld,X.matrix),X.matrixWorldInverse.copy(X.matrixWorld).invert()}this.updateCamera=function(X){if(s===null)return;let Q=X.near,nt=X.far;_.texture!==null&&(_.depthNear>0&&(Q=_.depthNear),_.depthFar>0&&(nt=_.depthFar)),M.near=k.near=U.near=Q,M.far=k.far=U.far=nt,(I!==M.near||V!==M.far)&&(s.updateRenderState({depthNear:M.near,depthFar:M.far}),I=M.near,V=M.far),U.layers.mask=X.layers.mask|2,k.layers.mask=X.layers.mask|4,M.layers.mask=U.layers.mask|k.layers.mask;const st=X.parent,vt=M.cameras;ut(M,st);for(let se=0;se<vt.length;se++)ut(vt[se],st);vt.length===2?j(M,U,k):M.projectionMatrix.copy(U.projectionMatrix),mt(X,M,st)};function mt(X,Q,nt){nt===null?X.matrix.copy(Q.matrixWorld):(X.matrix.copy(nt.matrixWorld),X.matrix.invert(),X.matrix.multiply(Q.matrixWorld)),X.matrix.decompose(X.position,X.quaternion,X.scale),X.updateMatrixWorld(!0),X.projectionMatrix.copy(Q.projectionMatrix),X.projectionMatrixInverse.copy(Q.projectionMatrixInverse),X.isPerspectiveCamera&&(X.fov=Ks*2*Math.atan(1/X.projectionMatrix.elements[5]),X.zoom=1)}this.getCamera=function(){return M},this.getFoveation=function(){if(!(d===null&&p===null))return l},this.setFoveation=function(X){l=X,d!==null&&(d.fixedFoveation=X),p!==null&&p.fixedFoveation!==void 0&&(p.fixedFoveation=X)},this.hasDepthSensing=function(){return _.texture!==null},this.getDepthSensingMesh=function(){return _.getMesh(M)};let gt=null;function Lt(X,Q){if(u=Q.getViewerPose(c||a),g=Q,u!==null){const nt=u.views;p!==null&&(t.setRenderTargetFramebuffer(P,p.framebuffer),t.setRenderTarget(P));let st=!1;nt.length!==M.cameras.length&&(M.cameras.length=0,st=!0);for(let Wt=0;Wt<nt.length;Wt++){const Fe=nt[Wt];let Ie=null;if(p!==null)Ie=p.getViewport(Fe);else{const B=h.getViewSubImage(d,Fe);Ie=B.viewport,Wt===0&&(t.setRenderTargetTextures(P,B.colorTexture,d.ignoreDepthValues?void 0:B.depthStencilTexture),t.setRenderTarget(P))}let me=y[Wt];me===void 0&&(me=new an,me.layers.enable(Wt),me.viewport=new Ee,y[Wt]=me),me.matrix.fromArray(Fe.transform.matrix),me.matrix.decompose(me.position,me.quaternion,me.scale),me.projectionMatrix.fromArray(Fe.projectionMatrix),me.projectionMatrixInverse.copy(me.projectionMatrix).invert(),me.viewport.set(Ie.x,Ie.y,Ie.width,Ie.height),Wt===0&&(M.matrix.copy(me.matrix),M.matrix.decompose(M.position,M.quaternion,M.scale)),st===!0&&M.cameras.push(me)}const vt=s.enabledFeatures;if(vt&&vt.includes("depth-sensing")&&s.depthUsage=="gpu-optimized"&&h){const Wt=h.getDepthInformation(nt[0]);Wt&&Wt.isValid&&Wt.texture&&_.init(t,Wt,s.renderState)}}for(let nt=0;nt<L.length;nt++){const st=w[nt],vt=L[nt];st!==null&&vt!==void 0&&vt.update(st,Q,c||a)}gt&&gt(X,Q),Q.detectedPlanes&&i.dispatchEvent({type:"planesdetected",data:Q}),g=null}const At=new Ed;At.setAnimationLoop(Lt),this.setAnimationLoop=function(X){gt=X},this.dispose=function(){}}}const $i=new yn,$x=new Kt;function jx(n,t){function e(m,f){m.matrixAutoUpdate===!0&&m.updateMatrix(),f.value.copy(m.matrix)}function i(m,f){f.color.getRGB(m.fogColor.value,cd(n)),f.isFog?(m.fogNear.value=f.near,m.fogFar.value=f.far):f.isFogExp2&&(m.fogDensity.value=f.density)}function s(m,f,P,L,w){f.isMeshBasicMaterial||f.isMeshLambertMaterial?r(m,f):f.isMeshToonMaterial?(r(m,f),h(m,f)):f.isMeshPhongMaterial?(r(m,f),u(m,f)):f.isMeshStandardMaterial?(r(m,f),d(m,f),f.isMeshPhysicalMaterial&&p(m,f,w)):f.isMeshMatcapMaterial?(r(m,f),g(m,f)):f.isMeshDepthMaterial?r(m,f):f.isMeshDistanceMaterial?(r(m,f),_(m,f)):f.isMeshNormalMaterial?r(m,f):f.isLineBasicMaterial?(a(m,f),f.isLineDashedMaterial&&o(m,f)):f.isPointsMaterial?l(m,f,P,L):f.isSpriteMaterial?c(m,f):f.isShadowMaterial?(m.color.value.copy(f.color),m.opacity.value=f.opacity):f.isShaderMaterial&&(f.uniformsNeedUpdate=!1)}function r(m,f){m.opacity.value=f.opacity,f.color&&m.diffuse.value.copy(f.color),f.emissive&&m.emissive.value.copy(f.emissive).multiplyScalar(f.emissiveIntensity),f.map&&(m.map.value=f.map,e(f.map,m.mapTransform)),f.alphaMap&&(m.alphaMap.value=f.alphaMap,e(f.alphaMap,m.alphaMapTransform)),f.bumpMap&&(m.bumpMap.value=f.bumpMap,e(f.bumpMap,m.bumpMapTransform),m.bumpScale.value=f.bumpScale,f.side===bn&&(m.bumpScale.value*=-1)),f.normalMap&&(m.normalMap.value=f.normalMap,e(f.normalMap,m.normalMapTransform),m.normalScale.value.copy(f.normalScale),f.side===bn&&m.normalScale.value.negate()),f.displacementMap&&(m.displacementMap.value=f.displacementMap,e(f.displacementMap,m.displacementMapTransform),m.displacementScale.value=f.displacementScale,m.displacementBias.value=f.displacementBias),f.emissiveMap&&(m.emissiveMap.value=f.emissiveMap,e(f.emissiveMap,m.emissiveMapTransform)),f.specularMap&&(m.specularMap.value=f.specularMap,e(f.specularMap,m.specularMapTransform)),f.alphaTest>0&&(m.alphaTest.value=f.alphaTest);const P=t.get(f),L=P.envMap,w=P.envMapRotation;L&&(m.envMap.value=L,$i.copy(w),$i.x*=-1,$i.y*=-1,$i.z*=-1,L.isCubeTexture&&L.isRenderTargetTexture===!1&&($i.y*=-1,$i.z*=-1),m.envMapRotation.value.setFromMatrix4($x.makeRotationFromEuler($i)),m.flipEnvMap.value=L.isCubeTexture&&L.isRenderTargetTexture===!1?-1:1,m.reflectivity.value=f.reflectivity,m.ior.value=f.ior,m.refractionRatio.value=f.refractionRatio),f.lightMap&&(m.lightMap.value=f.lightMap,m.lightMapIntensity.value=f.lightMapIntensity,e(f.lightMap,m.lightMapTransform)),f.aoMap&&(m.aoMap.value=f.aoMap,m.aoMapIntensity.value=f.aoMapIntensity,e(f.aoMap,m.aoMapTransform))}function a(m,f){m.diffuse.value.copy(f.color),m.opacity.value=f.opacity,f.map&&(m.map.value=f.map,e(f.map,m.mapTransform))}function o(m,f){m.dashSize.value=f.dashSize,m.totalSize.value=f.dashSize+f.gapSize,m.scale.value=f.scale}function l(m,f,P,L){m.diffuse.value.copy(f.color),m.opacity.value=f.opacity,m.size.value=f.size*P,m.scale.value=L*.5,f.map&&(m.map.value=f.map,e(f.map,m.uvTransform)),f.alphaMap&&(m.alphaMap.value=f.alphaMap,e(f.alphaMap,m.alphaMapTransform)),f.alphaTest>0&&(m.alphaTest.value=f.alphaTest)}function c(m,f){m.diffuse.value.copy(f.color),m.opacity.value=f.opacity,m.rotation.value=f.rotation,f.map&&(m.map.value=f.map,e(f.map,m.mapTransform)),f.alphaMap&&(m.alphaMap.value=f.alphaMap,e(f.alphaMap,m.alphaMapTransform)),f.alphaTest>0&&(m.alphaTest.value=f.alphaTest)}function u(m,f){m.specular.value.copy(f.specular),m.shininess.value=Math.max(f.shininess,1e-4)}function h(m,f){f.gradientMap&&(m.gradientMap.value=f.gradientMap)}function d(m,f){m.metalness.value=f.metalness,f.metalnessMap&&(m.metalnessMap.value=f.metalnessMap,e(f.metalnessMap,m.metalnessMapTransform)),m.roughness.value=f.roughness,f.roughnessMap&&(m.roughnessMap.value=f.roughnessMap,e(f.roughnessMap,m.roughnessMapTransform)),f.envMap&&(m.envMapIntensity.value=f.envMapIntensity)}function p(m,f,P){m.ior.value=f.ior,f.sheen>0&&(m.sheenColor.value.copy(f.sheenColor).multiplyScalar(f.sheen),m.sheenRoughness.value=f.sheenRoughness,f.sheenColorMap&&(m.sheenColorMap.value=f.sheenColorMap,e(f.sheenColorMap,m.sheenColorMapTransform)),f.sheenRoughnessMap&&(m.sheenRoughnessMap.value=f.sheenRoughnessMap,e(f.sheenRoughnessMap,m.sheenRoughnessMapTransform))),f.clearcoat>0&&(m.clearcoat.value=f.clearcoat,m.clearcoatRoughness.value=f.clearcoatRoughness,f.clearcoatMap&&(m.clearcoatMap.value=f.clearcoatMap,e(f.clearcoatMap,m.clearcoatMapTransform)),f.clearcoatRoughnessMap&&(m.clearcoatRoughnessMap.value=f.clearcoatRoughnessMap,e(f.clearcoatRoughnessMap,m.clearcoatRoughnessMapTransform)),f.clearcoatNormalMap&&(m.clearcoatNormalMap.value=f.clearcoatNormalMap,e(f.clearcoatNormalMap,m.clearcoatNormalMapTransform),m.clearcoatNormalScale.value.copy(f.clearcoatNormalScale),f.side===bn&&m.clearcoatNormalScale.value.negate())),f.dispersion>0&&(m.dispersion.value=f.dispersion),f.iridescence>0&&(m.iridescence.value=f.iridescence,m.iridescenceIOR.value=f.iridescenceIOR,m.iridescenceThicknessMinimum.value=f.iridescenceThicknessRange[0],m.iridescenceThicknessMaximum.value=f.iridescenceThicknessRange[1],f.iridescenceMap&&(m.iridescenceMap.value=f.iridescenceMap,e(f.iridescenceMap,m.iridescenceMapTransform)),f.iridescenceThicknessMap&&(m.iridescenceThicknessMap.value=f.iridescenceThicknessMap,e(f.iridescenceThicknessMap,m.iridescenceThicknessMapTransform))),f.transmission>0&&(m.transmission.value=f.transmission,m.transmissionSamplerMap.value=P.texture,m.transmissionSamplerSize.value.set(P.width,P.height),f.transmissionMap&&(m.transmissionMap.value=f.transmissionMap,e(f.transmissionMap,m.transmissionMapTransform)),m.thickness.value=f.thickness,f.thicknessMap&&(m.thicknessMap.value=f.thicknessMap,e(f.thicknessMap,m.thicknessMapTransform)),m.attenuationDistance.value=f.attenuationDistance,m.attenuationColor.value.copy(f.attenuationColor)),f.anisotropy>0&&(m.anisotropyVector.value.set(f.anisotropy*Math.cos(f.anisotropyRotation),f.anisotropy*Math.sin(f.anisotropyRotation)),f.anisotropyMap&&(m.anisotropyMap.value=f.anisotropyMap,e(f.anisotropyMap,m.anisotropyMapTransform))),m.specularIntensity.value=f.specularIntensity,m.specularColor.value.copy(f.specularColor),f.specularColorMap&&(m.specularColorMap.value=f.specularColorMap,e(f.specularColorMap,m.specularColorMapTransform)),f.specularIntensityMap&&(m.specularIntensityMap.value=f.specularIntensityMap,e(f.specularIntensityMap,m.specularIntensityMapTransform))}function g(m,f){f.matcap&&(m.matcap.value=f.matcap)}function _(m,f){const P=t.get(f).light;m.referencePosition.value.setFromMatrixPosition(P.matrixWorld),m.nearDistance.value=P.shadow.camera.near,m.farDistance.value=P.shadow.camera.far}return{refreshFogUniforms:i,refreshMaterialUniforms:s}}function Xx(n,t,e,i){let s={},r={},a=[];const o=n.getParameter(n.MAX_UNIFORM_BUFFER_BINDINGS);function l(P,L){const w=L.program;i.uniformBlockBinding(P,w)}function c(P,L){let w=s[P.id];w===void 0&&(g(P),w=u(P),s[P.id]=w,P.addEventListener("dispose",m));const N=L.program;i.updateUBOMapping(P,N);const F=t.render.frame;r[P.id]!==F&&(d(P),r[P.id]=F)}function u(P){const L=h();P.__bindingPointIndex=L;const w=n.createBuffer(),N=P.__size,F=P.usage;return n.bindBuffer(n.UNIFORM_BUFFER,w),n.bufferData(n.UNIFORM_BUFFER,N,F),n.bindBuffer(n.UNIFORM_BUFFER,null),n.bindBufferBase(n.UNIFORM_BUFFER,L,w),w}function h(){for(let P=0;P<o;P++)if(a.indexOf(P)===-1)return a.push(P),P;return console.error("THREE.WebGLRenderer: Maximum number of simultaneously usable uniforms groups reached."),0}function d(P){const L=s[P.id],w=P.uniforms,N=P.__cache;n.bindBuffer(n.UNIFORM_BUFFER,L);for(let F=0,U=w.length;F<U;F++){const k=Array.isArray(w[F])?w[F]:[w[F]];for(let y=0,M=k.length;y<M;y++){const I=k[y];if(p(I,F,y,N)===!0){const V=I.__offset,W=Array.isArray(I.value)?I.value:[I.value];let Z=0;for(let et=0;et<W.length;et++){const z=W[et],it=_(z);typeof z=="number"||typeof z=="boolean"?(I.__data[0]=z,n.bufferSubData(n.UNIFORM_BUFFER,V+Z,I.__data)):z.isMatrix3?(I.__data[0]=z.elements[0],I.__data[1]=z.elements[1],I.__data[2]=z.elements[2],I.__data[3]=0,I.__data[4]=z.elements[3],I.__data[5]=z.elements[4],I.__data[6]=z.elements[5],I.__data[7]=0,I.__data[8]=z.elements[6],I.__data[9]=z.elements[7],I.__data[10]=z.elements[8],I.__data[11]=0):(z.toArray(I.__data,Z),Z+=it.storage/Float32Array.BYTES_PER_ELEMENT)}n.bufferSubData(n.UNIFORM_BUFFER,V,I.__data)}}}n.bindBuffer(n.UNIFORM_BUFFER,null)}function p(P,L,w,N){const F=P.value,U=L+"_"+w;if(N[U]===void 0)return typeof F=="number"||typeof F=="boolean"?N[U]=F:N[U]=F.clone(),!0;{const k=N[U];if(typeof F=="number"||typeof F=="boolean"){if(k!==F)return N[U]=F,!0}else if(k.equals(F)===!1)return k.copy(F),!0}return!1}function g(P){const L=P.uniforms;let w=0;const N=16;for(let U=0,k=L.length;U<k;U++){const y=Array.isArray(L[U])?L[U]:[L[U]];for(let M=0,I=y.length;M<I;M++){const V=y[M],W=Array.isArray(V.value)?V.value:[V.value];for(let Z=0,et=W.length;Z<et;Z++){const z=W[Z],it=_(z),j=w%N,ut=j%it.boundary,mt=j+ut;w+=ut,mt!==0&&N-mt<it.storage&&(w+=N-mt),V.__data=new Float32Array(it.storage/Float32Array.BYTES_PER_ELEMENT),V.__offset=w,w+=it.storage}}}const F=w%N;return F>0&&(w+=N-F),P.__size=w,P.__cache={},this}function _(P){const L={boundary:0,storage:0};return typeof P=="number"||typeof P=="boolean"?(L.boundary=4,L.storage=4):P.isVector2?(L.boundary=8,L.storage=8):P.isVector3||P.isColor?(L.boundary=16,L.storage=12):P.isVector4?(L.boundary=16,L.storage=16):P.isMatrix3?(L.boundary=48,L.storage=48):P.isMatrix4?(L.boundary=64,L.storage=64):P.isTexture?console.warn("THREE.WebGLRenderer: Texture samplers can not be part of an uniforms group."):console.warn("THREE.WebGLRenderer: Unsupported uniform value type.",P),L}function m(P){const L=P.target;L.removeEventListener("dispose",m);const w=a.indexOf(L.__bindingPointIndex);a.splice(w,1),n.deleteBuffer(s[L.id]),delete s[L.id],delete r[L.id]}function f(){for(const P in s)n.deleteBuffer(s[P]);a=[],s={},r={}}return{bind:l,update:c,dispose:f}}class qx{constructor(t={}){const{canvas:e=um(),context:i=null,depth:s=!0,stencil:r=!1,alpha:a=!1,antialias:o=!1,premultipliedAlpha:l=!0,preserveDrawingBuffer:c=!1,powerPreference:u="default",failIfMajorPerformanceCaveat:h=!1,reverseDepthBuffer:d=!1}=t;this.isWebGLRenderer=!0;let p;if(i!==null){if(typeof WebGLRenderingContext<"u"&&i instanceof WebGLRenderingContext)throw new Error("THREE.WebGLRenderer: WebGL 1 is not supported since r163.");p=i.getContextAttributes().alpha}else p=a;const g=new Uint32Array(4),_=new Int32Array(4);let m=null,f=null;const P=[],L=[];this.domElement=e,this.debug={checkShaderErrors:!0,onShaderError:null},this.autoClear=!0,this.autoClearColor=!0,this.autoClearDepth=!0,this.autoClearStencil=!0,this.sortObjects=!0,this.clippingPlanes=[],this.localClippingEnabled=!1,this._outputColorSpace=ze,this.toneMapping=Pi,this.toneMappingExposure=1;const w=this;let N=!1,F=0,U=0,k=null,y=-1,M=null;const I=new Ee,V=new Ee;let W=null;const Z=new ne(0);let et=0,z=e.width,it=e.height,j=1,ut=null,mt=null;const gt=new Ee(0,0,z,it),Lt=new Ee(0,0,z,it);let At=!1;const X=new pc;let Q=!1,nt=!1;this.transmissionResolutionScale=1;const st=new Kt,vt=new Kt,se=new O,Wt=new Ee,Fe={background:null,fog:null,environment:null,overrideMaterial:null,isScene:!0};let Ie=!1;function me(){return k===null?j:1}let B=i;function pn(A,H){return e.getContext(A,H)}try{const A={alpha:!0,depth:s,stencil:r,antialias:o,premultipliedAlpha:l,preserveDrawingBuffer:c,powerPreference:u,failIfMajorPerformanceCaveat:h};if("setAttribute"in e&&e.setAttribute("data-engine",`three.js r${sc}`),e.addEventListener("webglcontextlost",ct,!1),e.addEventListener("webglcontextrestored",wt,!1),e.addEventListener("webglcontextcreationerror",Et,!1),B===null){const H="webgl2";if(B=pn(H,A),B===null)throw pn(H)?new Error("Error creating WebGL context with your selected attributes."):new Error("Error creating WebGL context.")}}catch(A){throw console.error("THREE.WebGLRenderer: "+A.message),A}let fe,pe,Vt,Le,zt,D,S,q,lt,ht,ot,Ft,St,Pt,ve,pt,Nt,Xt,Zt,Dt,ge,ae,Te,G;function Mt(){fe=new sv(B),fe.init(),ae=new zx(B,fe),pe=new J0(B,fe,t,ae),Vt=new Ox(B,fe),pe.reverseDepthBuffer&&d&&Vt.buffers.depth.setReversed(!0),Le=new ov(B),zt=new Tx,D=new Bx(B,fe,Vt,zt,pe,ae,Le),S=new Q0(w),q=new iv(w),lt=new fg(B),Te=new Y0(B,lt),ht=new rv(B,lt,Le,Te),ot=new cv(B,ht,lt,Le),Zt=new lv(B,pe,D),pt=new Z0(zt),Ft=new Ex(w,S,q,fe,pe,Te,pt),St=new jx(w,zt),Pt=new Ax,ve=new Dx(fe),Xt=new q0(w,S,q,Vt,ot,p,l),Nt=new kx(w,ot,pe),G=new Xx(B,Le,pe,Vt),Dt=new K0(B,fe,Le),ge=new av(B,fe,Le),Le.programs=Ft.programs,w.capabilities=pe,w.extensions=fe,w.properties=zt,w.renderLists=Pt,w.shadowMap=Nt,w.state=Vt,w.info=Le}Mt();const tt=new Wx(w,B);this.xr=tt,this.getContext=function(){return B},this.getContextAttributes=function(){return B.getContextAttributes()},this.forceContextLoss=function(){const A=fe.get("WEBGL_lose_context");A&&A.loseContext()},this.forceContextRestore=function(){const A=fe.get("WEBGL_lose_context");A&&A.restoreContext()},this.getPixelRatio=function(){return j},this.setPixelRatio=function(A){A!==void 0&&(j=A,this.setSize(z,it,!1))},this.getSize=function(A){return A.set(z,it)},this.setSize=function(A,H,Y=!0){if(tt.isPresenting){console.warn("THREE.WebGLRenderer: Can't change size while VR device is presenting.");return}z=A,it=H,e.width=Math.floor(A*j),e.height=Math.floor(H*j),Y===!0&&(e.style.width=A+"px",e.style.height=H+"px"),this.setViewport(0,0,A,H)},this.getDrawingBufferSize=function(A){return A.set(z*j,it*j).floor()},this.setDrawingBufferSize=function(A,H,Y){z=A,it=H,j=Y,e.width=Math.floor(A*Y),e.height=Math.floor(H*Y),this.setViewport(0,0,A,H)},this.getCurrentViewport=function(A){return A.copy(I)},this.getViewport=function(A){return A.copy(gt)},this.setViewport=function(A,H,Y,K){A.isVector4?gt.set(A.x,A.y,A.z,A.w):gt.set(A,H,Y,K),Vt.viewport(I.copy(gt).multiplyScalar(j).round())},this.getScissor=function(A){return A.copy(Lt)},this.setScissor=function(A,H,Y,K){A.isVector4?Lt.set(A.x,A.y,A.z,A.w):Lt.set(A,H,Y,K),Vt.scissor(V.copy(Lt).multiplyScalar(j).round())},this.getScissorTest=function(){return At},this.setScissorTest=function(A){Vt.setScissorTest(At=A)},this.setOpaqueSort=function(A){ut=A},this.setTransparentSort=function(A){mt=A},this.getClearColor=function(A){return A.copy(Xt.getClearColor())},this.setClearColor=function(){Xt.setClearColor(...arguments)},this.getClearAlpha=function(){return Xt.getClearAlpha()},this.setClearAlpha=function(){Xt.setClearAlpha(...arguments)},this.clear=function(A=!0,H=!0,Y=!0){let K=0;if(A){let $=!1;if(k!==null){const ft=k.texture.format;$=ft===uc||ft===cc||ft===lc}if($){const ft=k.texture.type,xt=ft===hi||ft===ts||ft===Mr||ft===Xs||ft===ac||ft===oc,Ct=Xt.getClearColor(),It=Xt.getClearAlpha(),Qt=Ct.r,te=Ct.g,Ht=Ct.b;xt?(g[0]=Qt,g[1]=te,g[2]=Ht,g[3]=It,B.clearBufferuiv(B.COLOR,0,g)):(_[0]=Qt,_[1]=te,_[2]=Ht,_[3]=It,B.clearBufferiv(B.COLOR,0,_))}else K|=B.COLOR_BUFFER_BIT}H&&(K|=B.DEPTH_BUFFER_BIT),Y&&(K|=B.STENCIL_BUFFER_BIT,this.state.buffers.stencil.setMask(4294967295)),B.clear(K)},this.clearColor=function(){this.clear(!0,!1,!1)},this.clearDepth=function(){this.clear(!1,!0,!1)},this.clearStencil=function(){this.clear(!1,!1,!0)},this.dispose=function(){e.removeEventListener("webglcontextlost",ct,!1),e.removeEventListener("webglcontextrestored",wt,!1),e.removeEventListener("webglcontextcreationerror",Et,!1),Xt.dispose(),Pt.dispose(),ve.dispose(),zt.dispose(),S.dispose(),q.dispose(),ot.dispose(),Te.dispose(),G.dispose(),Ft.dispose(),tt.dispose(),tt.removeEventListener("sessionstart",kr),tt.removeEventListener("sessionend",Fr),Kn.stop()};function ct(A){A.preventDefault(),console.log("THREE.WebGLRenderer: Context Lost."),N=!0}function wt(){console.log("THREE.WebGLRenderer: Context Restored."),N=!1;const A=Le.autoReset,H=Nt.enabled,Y=Nt.autoUpdate,K=Nt.needsUpdate,$=Nt.type;Mt(),Le.autoReset=A,Nt.enabled=H,Nt.autoUpdate=Y,Nt.needsUpdate=K,Nt.type=$}function Et(A){console.error("THREE.WebGLRenderer: A WebGL context could not be created. Reason: ",A.statusMessage)}function re(A){const H=A.target;H.removeEventListener("dispose",re),Ue(H)}function Ue(A){Je(A),zt.remove(A)}function Je(A){const H=zt.get(A).programs;H!==void 0&&(H.forEach(function(Y){Ft.releaseProgram(Y)}),A.isShaderMaterial&&Ft.releaseShaderCache(A))}this.renderBufferDirect=function(A,H,Y,K,$,ft){H===null&&(H=Fe);const xt=$.isMesh&&$.matrixWorld.determinant()<0,Ct=lo(A,H,Y,K,$);Vt.setMaterial(K,xt);let It=Y.index,Qt=1;if(K.wireframe===!0){if(It=ht.getWireframeAttribute(Y),It===void 0)return;Qt=2}const te=Y.drawRange,Ht=Y.attributes.position;let ce=te.start*Qt,be=(te.start+te.count)*Qt;ft!==null&&(ce=Math.max(ce,ft.start*Qt),be=Math.min(be,(ft.start+ft.count)*Qt)),It!==null?(ce=Math.max(ce,0),be=Math.min(be,It.count)):Ht!=null&&(ce=Math.max(ce,0),be=Math.min(be,Ht.count));const Oe=be-ce;if(Oe<0||Oe===1/0)return;Te.setup($,K,Ct,Y,It);let ke,xe=Dt;if(It!==null&&(ke=lt.get(It),xe=ge,xe.setIndex(ke)),$.isMesh)K.wireframe===!0?(Vt.setLineWidth(K.wireframeLinewidth*me()),xe.setMode(B.LINES)):xe.setMode(B.TRIANGLES);else if($.isLine){let $t=K.linewidth;$t===void 0&&($t=1),Vt.setLineWidth($t*me()),$.isLineSegments?xe.setMode(B.LINES):$.isLineLoop?xe.setMode(B.LINE_LOOP):xe.setMode(B.LINE_STRIP)}else $.isPoints?xe.setMode(B.POINTS):$.isSprite&&xe.setMode(B.TRIANGLES);if($.isBatchedMesh)if($._multiDrawInstances!==null)ji("THREE.WebGLRenderer: renderMultiDrawInstances has been deprecated and will be removed in r184. Append to renderMultiDraw arguments and use indirection."),xe.renderMultiDrawInstances($._multiDrawStarts,$._multiDrawCounts,$._multiDrawCount,$._multiDrawInstances);else if(fe.get("WEBGL_multi_draw"))xe.renderMultiDraw($._multiDrawStarts,$._multiDrawCounts,$._multiDrawCount);else{const $t=$._multiDrawStarts,je=$._multiDrawCounts,ye=$._multiDrawCount,gn=It?lt.get(It).bytesPerElement:1,De=zt.get(K).currentProgram.getUniforms();for(let Qe=0;Qe<ye;Qe++)De.setValue(B,"_gl_DrawID",Qe),xe.render($t[Qe]/gn,je[Qe])}else if($.isInstancedMesh)xe.renderInstances(ce,Oe,$.count);else if(Y.isInstancedBufferGeometry){const $t=Y._maxInstanceCount!==void 0?Y._maxInstanceCount:1/0,je=Math.min(Y.instanceCount,$t);xe.renderInstances(ce,Oe,je)}else xe.render(ce,Oe)};function oe(A,H,Y){A.transparent===!0&&A.side===Tn&&A.forceSinglePass===!1?(A.side=bn,A.needsUpdate=!0,ps(A,H,Y),A.side=ui,A.needsUpdate=!0,ps(A,H,Y),A.side=Tn):ps(A,H,Y)}this.compile=function(A,H,Y=null){Y===null&&(Y=A),f=ve.get(Y),f.init(H),L.push(f),Y.traverseVisible(function($){$.isLight&&$.layers.test(H.layers)&&(f.pushLight($),$.castShadow&&f.pushShadow($))}),A!==Y&&A.traverseVisible(function($){$.isLight&&$.layers.test(H.layers)&&(f.pushLight($),$.castShadow&&f.pushShadow($))}),f.setupLights();const K=new Set;return A.traverse(function($){if(!($.isMesh||$.isPoints||$.isLine||$.isSprite))return;const ft=$.material;if(ft)if(Array.isArray(ft))for(let xt=0;xt<ft.length;xt++){const Ct=ft[xt];oe(Ct,Y,$),K.add(Ct)}else oe(ft,Y,$),K.add(ft)}),f=L.pop(),K},this.compileAsync=function(A,H,Y=null){const K=this.compile(A,H,Y);return new Promise($=>{function ft(){if(K.forEach(function(xt){zt.get(xt).currentProgram.isReady()&&K.delete(xt)}),K.size===0){$(A);return}setTimeout(ft,10)}fe.get("KHR_parallel_shader_compile")!==null?ft():setTimeout(ft,10)})};let mn=null;function Dn(A){mn&&mn(A)}function kr(){Kn.stop()}function Fr(){Kn.start()}const Kn=new Ed;Kn.setAnimationLoop(Dn),typeof self<"u"&&Kn.setContext(self),this.setAnimationLoop=function(A){mn=A,tt.setAnimationLoop(A),A===null?Kn.stop():Kn.start()},tt.addEventListener("sessionstart",kr),tt.addEventListener("sessionend",Fr),this.render=function(A,H){if(H!==void 0&&H.isCamera!==!0){console.error("THREE.WebGLRenderer.render: camera is not an instance of THREE.Camera.");return}if(N===!0)return;if(A.matrixWorldAutoUpdate===!0&&A.updateMatrixWorld(),H.parent===null&&H.matrixWorldAutoUpdate===!0&&H.updateMatrixWorld(),tt.enabled===!0&&tt.isPresenting===!0&&(tt.cameraAutoUpdate===!0&&tt.updateCamera(H),H=tt.getCamera()),A.isScene===!0&&A.onBeforeRender(w,A,H,k),f=ve.get(A,L.length),f.init(H),L.push(f),vt.multiplyMatrices(H.projectionMatrix,H.matrixWorldInverse),X.setFromProjectionMatrix(vt),nt=this.localClippingEnabled,Q=pt.init(this.clippingPlanes,nt),m=Pt.get(A,P.length),m.init(),P.push(m),tt.enabled===!0&&tt.isPresenting===!0){const ft=w.xr.getDepthSensingMesh();ft!==null&&or(ft,H,-1/0,w.sortObjects)}or(A,H,0,w.sortObjects),m.finish(),w.sortObjects===!0&&m.sort(ut,mt),Ie=tt.enabled===!1||tt.isPresenting===!1||tt.hasDepthSensing()===!1,Ie&&Xt.addToRenderList(m,A),this.info.render.frame++,Q===!0&&pt.beginShadows();const Y=f.state.shadowsArray;Nt.render(Y,A,H),Q===!0&&pt.endShadows(),this.info.autoReset===!0&&this.info.reset();const K=m.opaque,$=m.transmissive;if(f.setupLights(),H.isArrayCamera){const ft=H.cameras;if($.length>0)for(let xt=0,Ct=ft.length;xt<Ct;xt++){const It=ft[xt];lr(K,$,A,It)}Ie&&Xt.render(A);for(let xt=0,Ct=ft.length;xt<Ct;xt++){const It=ft[xt];Or(m,A,It,It.viewport)}}else $.length>0&&lr(K,$,A,H),Ie&&Xt.render(A),Or(m,A,H);k!==null&&U===0&&(D.updateMultisampleRenderTarget(k),D.updateRenderTargetMipmap(k)),A.isScene===!0&&A.onAfterRender(w,A,H),Te.resetDefaultState(),y=-1,M=null,L.pop(),L.length>0?(f=L[L.length-1],Q===!0&&pt.setGlobalState(w.clippingPlanes,f.state.camera)):f=null,P.pop(),P.length>0?m=P[P.length-1]:m=null};function or(A,H,Y,K){if(A.visible===!1)return;if(A.layers.test(H.layers)){if(A.isGroup)Y=A.renderOrder;else if(A.isLOD)A.autoUpdate===!0&&A.update(H);else if(A.isLight)f.pushLight(A),A.castShadow&&f.pushShadow(A);else if(A.isSprite){if(!A.frustumCulled||X.intersectsSprite(A)){K&&Wt.setFromMatrixPosition(A.matrixWorld).applyMatrix4(vt);const xt=ot.update(A),Ct=A.material;Ct.visible&&m.push(A,xt,Ct,Y,Wt.z,null)}}else if((A.isMesh||A.isLine||A.isPoints)&&(!A.frustumCulled||X.intersectsObject(A))){const xt=ot.update(A),Ct=A.material;if(K&&(A.boundingSphere!==void 0?(A.boundingSphere===null&&A.computeBoundingSphere(),Wt.copy(A.boundingSphere.center)):(xt.boundingSphere===null&&xt.computeBoundingSphere(),Wt.copy(xt.boundingSphere.center)),Wt.applyMatrix4(A.matrixWorld).applyMatrix4(vt)),Array.isArray(Ct)){const It=xt.groups;for(let Qt=0,te=It.length;Qt<te;Qt++){const Ht=It[Qt],ce=Ct[Ht.materialIndex];ce&&ce.visible&&m.push(A,xt,ce,Y,Wt.z,Ht)}}else Ct.visible&&m.push(A,xt,Ct,Y,Wt.z,null)}}const ft=A.children;for(let xt=0,Ct=ft.length;xt<Ct;xt++)or(ft[xt],H,Y,K)}function Or(A,H,Y,K){const $=A.opaque,ft=A.transmissive,xt=A.transparent;f.setupLightsView(Y),Q===!0&&pt.setGlobalState(w.clippingPlanes,Y),K&&Vt.viewport(I.copy(K)),$.length>0&&fs($,H,Y),ft.length>0&&fs(ft,H,Y),xt.length>0&&fs(xt,H,Y),Vt.buffers.depth.setTest(!0),Vt.buffers.depth.setMask(!0),Vt.buffers.color.setMask(!0),Vt.setPolygonOffset(!1)}function lr(A,H,Y,K){if((Y.isScene===!0?Y.overrideMaterial:null)!==null)return;f.state.transmissionRenderTarget[K.id]===void 0&&(f.state.transmissionRenderTarget[K.id]=new es(1,1,{generateMipmaps:!0,type:fe.has("EXT_color_buffer_half_float")||fe.has("EXT_color_buffer_float")?Rr:hi,minFilter:oi,samples:4,stencilBuffer:r,resolveDepthBuffer:!1,resolveStencilBuffer:!1,colorSpace:_e.workingColorSpace}));const ft=f.state.transmissionRenderTarget[K.id],xt=K.viewport||I;ft.setSize(xt.z*w.transmissionResolutionScale,xt.w*w.transmissionResolutionScale);const Ct=w.getRenderTarget();w.setRenderTarget(ft),w.getClearColor(Z),et=w.getClearAlpha(),et<1&&w.setClearColor(16777215,.5),w.clear(),Ie&&Xt.render(Y);const It=w.toneMapping;w.toneMapping=Pi;const Qt=K.viewport;if(K.viewport!==void 0&&(K.viewport=void 0),f.setupLightsView(K),Q===!0&&pt.setGlobalState(w.clippingPlanes,K),fs(A,Y,K),D.updateMultisampleRenderTarget(ft),D.updateRenderTargetMipmap(ft),fe.has("WEBGL_multisampled_render_to_texture")===!1){let te=!1;for(let Ht=0,ce=H.length;Ht<ce;Ht++){const be=H[Ht],Oe=be.object,ke=be.geometry,xe=be.material,$t=be.group;if(xe.side===Tn&&Oe.layers.test(K.layers)){const je=xe.side;xe.side=bn,xe.needsUpdate=!0,cr(Oe,Y,K,ke,xe,$t),xe.side=je,xe.needsUpdate=!0,te=!0}}te===!0&&(D.updateMultisampleRenderTarget(ft),D.updateRenderTargetMipmap(ft))}w.setRenderTarget(Ct),w.setClearColor(Z,et),Qt!==void 0&&(K.viewport=Qt),w.toneMapping=It}function fs(A,H,Y){const K=H.isScene===!0?H.overrideMaterial:null;for(let $=0,ft=A.length;$<ft;$++){const xt=A[$],Ct=xt.object,It=xt.geometry,Qt=K===null?xt.material:K,te=xt.group;Ct.layers.test(Y.layers)&&cr(Ct,H,Y,It,Qt,te)}}function cr(A,H,Y,K,$,ft){A.onBeforeRender(w,H,Y,K,$,ft),A.modelViewMatrix.multiplyMatrices(Y.matrixWorldInverse,A.matrixWorld),A.normalMatrix.getNormalMatrix(A.modelViewMatrix),$.onBeforeRender(w,H,Y,K,A,ft),$.transparent===!0&&$.side===Tn&&$.forceSinglePass===!1?($.side=bn,$.needsUpdate=!0,w.renderBufferDirect(Y,H,K,$,A,ft),$.side=ui,$.needsUpdate=!0,w.renderBufferDirect(Y,H,K,$,A,ft),$.side=Tn):w.renderBufferDirect(Y,H,K,$,A,ft),A.onAfterRender(w,H,Y,K,$,ft)}function ps(A,H,Y){H.isScene!==!0&&(H=Fe);const K=zt.get(A),$=f.state.lights,ft=f.state.shadowsArray,xt=$.state.version,Ct=Ft.getParameters(A,$.state,ft,H,Y),It=Ft.getProgramCacheKey(Ct);let Qt=K.programs;K.environment=A.isMeshStandardMaterial?H.environment:null,K.fog=H.fog,K.envMap=(A.isMeshStandardMaterial?q:S).get(A.envMap||K.environment),K.envMapRotation=K.environment!==null&&A.envMap===null?H.environmentRotation:A.envMapRotation,Qt===void 0&&(A.addEventListener("dispose",re),Qt=new Map,K.programs=Qt);let te=Qt.get(It);if(te!==void 0){if(K.currentProgram===te&&K.lightsStateVersion===xt)return zr(A,Ct),te}else Ct.uniforms=Ft.getUniforms(A),A.onBeforeCompile(Ct,w),te=Ft.acquireProgram(Ct,It),Qt.set(It,te),K.uniforms=Ct.uniforms;const Ht=K.uniforms;return(!A.isShaderMaterial&&!A.isRawShaderMaterial||A.clipping===!0)&&(Ht.clippingPlanes=pt.uniform),zr(A,Ct),K.needsLights=uo(A),K.lightsStateVersion=xt,K.needsLights&&(Ht.ambientLightColor.value=$.state.ambient,Ht.lightProbe.value=$.state.probe,Ht.directionalLights.value=$.state.directional,Ht.directionalLightShadows.value=$.state.directionalShadow,Ht.spotLights.value=$.state.spot,Ht.spotLightShadows.value=$.state.spotShadow,Ht.rectAreaLights.value=$.state.rectArea,Ht.ltc_1.value=$.state.rectAreaLTC1,Ht.ltc_2.value=$.state.rectAreaLTC2,Ht.pointLights.value=$.state.point,Ht.pointLightShadows.value=$.state.pointShadow,Ht.hemisphereLights.value=$.state.hemi,Ht.directionalShadowMap.value=$.state.directionalShadowMap,Ht.directionalShadowMatrix.value=$.state.directionalShadowMatrix,Ht.spotShadowMap.value=$.state.spotShadowMap,Ht.spotLightMatrix.value=$.state.spotLightMatrix,Ht.spotLightMap.value=$.state.spotLightMap,Ht.pointShadowMap.value=$.state.pointShadowMap,Ht.pointShadowMatrix.value=$.state.pointShadowMatrix),K.currentProgram=te,K.uniformsList=null,te}function Br(A){if(A.uniformsList===null){const H=A.currentProgram.getUniforms();A.uniformsList=Da.seqWithValue(H.seq,A.uniforms)}return A.uniformsList}function zr(A,H){const Y=zt.get(A);Y.outputColorSpace=H.outputColorSpace,Y.batching=H.batching,Y.batchingColor=H.batchingColor,Y.instancing=H.instancing,Y.instancingColor=H.instancingColor,Y.instancingMorph=H.instancingMorph,Y.skinning=H.skinning,Y.morphTargets=H.morphTargets,Y.morphNormals=H.morphNormals,Y.morphColors=H.morphColors,Y.morphTargetsCount=H.morphTargetsCount,Y.numClippingPlanes=H.numClippingPlanes,Y.numIntersection=H.numClipIntersection,Y.vertexAlphas=H.vertexAlphas,Y.vertexTangents=H.vertexTangents,Y.toneMapping=H.toneMapping}function lo(A,H,Y,K,$){H.isScene!==!0&&(H=Fe),D.resetTextureUnits();const ft=H.fog,xt=K.isMeshStandardMaterial?H.environment:null,Ct=k===null?w.outputColorSpace:k.isXRRenderTarget===!0?k.texture.colorSpace:Ys,It=(K.isMeshStandardMaterial?q:S).get(K.envMap||xt),Qt=K.vertexColors===!0&&!!Y.attributes.color&&Y.attributes.color.itemSize===4,te=!!Y.attributes.tangent&&(!!K.normalMap||K.anisotropy>0),Ht=!!Y.morphAttributes.position,ce=!!Y.morphAttributes.normal,be=!!Y.morphAttributes.color;let Oe=Pi;K.toneMapped&&(k===null||k.isXRRenderTarget===!0)&&(Oe=w.toneMapping);const ke=Y.morphAttributes.position||Y.morphAttributes.normal||Y.morphAttributes.color,xe=ke!==void 0?ke.length:0,$t=zt.get(K),je=f.state.lights;if(Q===!0&&(nt===!0||A!==M)){const Ge=A===M&&K.id===y;pt.setState(K,A,Ge)}let ye=!1;K.version===$t.__version?($t.needsLights&&$t.lightsStateVersion!==je.state.version||$t.outputColorSpace!==Ct||$.isBatchedMesh&&$t.batching===!1||!$.isBatchedMesh&&$t.batching===!0||$.isBatchedMesh&&$t.batchingColor===!0&&$.colorTexture===null||$.isBatchedMesh&&$t.batchingColor===!1&&$.colorTexture!==null||$.isInstancedMesh&&$t.instancing===!1||!$.isInstancedMesh&&$t.instancing===!0||$.isSkinnedMesh&&$t.skinning===!1||!$.isSkinnedMesh&&$t.skinning===!0||$.isInstancedMesh&&$t.instancingColor===!0&&$.instanceColor===null||$.isInstancedMesh&&$t.instancingColor===!1&&$.instanceColor!==null||$.isInstancedMesh&&$t.instancingMorph===!0&&$.morphTexture===null||$.isInstancedMesh&&$t.instancingMorph===!1&&$.morphTexture!==null||$t.envMap!==It||K.fog===!0&&$t.fog!==ft||$t.numClippingPlanes!==void 0&&($t.numClippingPlanes!==pt.numPlanes||$t.numIntersection!==pt.numIntersection)||$t.vertexAlphas!==Qt||$t.vertexTangents!==te||$t.morphTargets!==Ht||$t.morphNormals!==ce||$t.morphColors!==be||$t.toneMapping!==Oe||$t.morphTargetsCount!==xe)&&(ye=!0):(ye=!0,$t.__version=K.version);let gn=$t.currentProgram;ye===!0&&(gn=ps(K,H,$));let De=!1,Qe=!1,Bi=!1;const Ne=gn.getUniforms(),cn=$t.uniforms;if(Vt.useProgram(gn.program)&&(De=!0,Qe=!0,Bi=!0),K.id!==y&&(y=K.id,Qe=!0),De||M!==A){Vt.buffers.depth.getReversed()?(st.copy(A.projectionMatrix),dm(st),fm(st),Ne.setValue(B,"projectionMatrix",st)):Ne.setValue(B,"projectionMatrix",A.projectionMatrix),Ne.setValue(B,"viewMatrix",A.matrixWorldInverse);const Xe=Ne.map.cameraPosition;Xe!==void 0&&Xe.setValue(B,se.setFromMatrixPosition(A.matrixWorld)),pe.logarithmicDepthBuffer&&Ne.setValue(B,"logDepthBufFC",2/(Math.log(A.far+1)/Math.LN2)),(K.isMeshPhongMaterial||K.isMeshToonMaterial||K.isMeshLambertMaterial||K.isMeshBasicMaterial||K.isMeshStandardMaterial||K.isShaderMaterial)&&Ne.setValue(B,"isOrthographic",A.isOrthographicCamera===!0),M!==A&&(M=A,Qe=!0,Bi=!0)}if($.isSkinnedMesh){Ne.setOptional(B,$,"bindMatrix"),Ne.setOptional(B,$,"bindMatrixInverse");const Ge=$.skeleton;Ge&&(Ge.boneTexture===null&&Ge.computeBoneTexture(),Ne.setValue(B,"boneTexture",Ge.boneTexture,D))}$.isBatchedMesh&&(Ne.setOptional(B,$,"batchingTexture"),Ne.setValue(B,"batchingTexture",$._matricesTexture,D),Ne.setOptional(B,$,"batchingIdTexture"),Ne.setValue(B,"batchingIdTexture",$._indirectTexture,D),Ne.setOptional(B,$,"batchingColorTexture"),$._colorsTexture!==null&&Ne.setValue(B,"batchingColorTexture",$._colorsTexture,D));const nn=Y.morphAttributes;if((nn.position!==void 0||nn.normal!==void 0||nn.color!==void 0)&&Zt.update($,Y,gn),(Qe||$t.receiveShadow!==$.receiveShadow)&&($t.receiveShadow=$.receiveShadow,Ne.setValue(B,"receiveShadow",$.receiveShadow)),K.isMeshGouraudMaterial&&K.envMap!==null&&(cn.envMap.value=It,cn.flipEnvMap.value=It.isCubeTexture&&It.isRenderTargetTexture===!1?-1:1),K.isMeshStandardMaterial&&K.envMap===null&&H.environment!==null&&(cn.envMapIntensity.value=H.environmentIntensity),Qe&&(Ne.setValue(B,"toneMappingExposure",w.toneMappingExposure),$t.needsLights&&co(cn,Bi),ft&&K.fog===!0&&St.refreshFogUniforms(cn,ft),St.refreshMaterialUniforms(cn,K,j,it,f.state.transmissionRenderTarget[A.id]),Da.upload(B,Br($t),cn,D)),K.isShaderMaterial&&K.uniformsNeedUpdate===!0&&(Da.upload(B,Br($t),cn,D),K.uniformsNeedUpdate=!1),K.isSpriteMaterial&&Ne.setValue(B,"center",$.center),Ne.setValue(B,"modelViewMatrix",$.modelViewMatrix),Ne.setValue(B,"normalMatrix",$.normalMatrix),Ne.setValue(B,"modelMatrix",$.matrixWorld),K.isShaderMaterial||K.isRawShaderMaterial){const Ge=K.uniformsGroups;for(let Xe=0,ms=Ge.length;Xe<ms;Xe++){const Jn=Ge[Xe];G.update(Jn,gn),G.bind(Jn,gn)}}return gn}function co(A,H){A.ambientLightColor.needsUpdate=H,A.lightProbe.needsUpdate=H,A.directionalLights.needsUpdate=H,A.directionalLightShadows.needsUpdate=H,A.pointLights.needsUpdate=H,A.pointLightShadows.needsUpdate=H,A.spotLights.needsUpdate=H,A.spotLightShadows.needsUpdate=H,A.rectAreaLights.needsUpdate=H,A.hemisphereLights.needsUpdate=H}function uo(A){return A.isMeshLambertMaterial||A.isMeshToonMaterial||A.isMeshPhongMaterial||A.isMeshStandardMaterial||A.isShadowMaterial||A.isShaderMaterial&&A.lights===!0}this.getActiveCubeFace=function(){return F},this.getActiveMipmapLevel=function(){return U},this.getRenderTarget=function(){return k},this.setRenderTargetTextures=function(A,H,Y){zt.get(A.texture).__webglTexture=H,zt.get(A.depthTexture).__webglTexture=Y;const K=zt.get(A);K.__hasExternalTextures=!0,K.__autoAllocateDepthBuffer=Y===void 0,K.__autoAllocateDepthBuffer||fe.has("WEBGL_multisampled_render_to_texture")===!0&&(console.warn("THREE.WebGLRenderer: Render-to-texture extension was disabled because an external texture was provided"),K.__useRenderToTexture=!1)},this.setRenderTargetFramebuffer=function(A,H){const Y=zt.get(A);Y.__webglFramebuffer=H,Y.__useDefaultFramebuffer=H===void 0};const ho=B.createFramebuffer();this.setRenderTarget=function(A,H=0,Y=0){k=A,F=H,U=Y;let K=!0,$=null,ft=!1,xt=!1;if(A){const It=zt.get(A);if(It.__useDefaultFramebuffer!==void 0)Vt.bindFramebuffer(B.FRAMEBUFFER,null),K=!1;else if(It.__webglFramebuffer===void 0)D.setupRenderTarget(A);else if(It.__hasExternalTextures)D.rebindTextures(A,zt.get(A.texture).__webglTexture,zt.get(A.depthTexture).__webglTexture);else if(A.depthBuffer){const Ht=A.depthTexture;if(It.__boundDepthTexture!==Ht){if(Ht!==null&&zt.has(Ht)&&(A.width!==Ht.image.width||A.height!==Ht.image.height))throw new Error("WebGLRenderTarget: Attached DepthTexture is initialized to the incorrect size.");D.setupDepthRenderbuffer(A)}}const Qt=A.texture;(Qt.isData3DTexture||Qt.isDataArrayTexture||Qt.isCompressedArrayTexture)&&(xt=!0);const te=zt.get(A).__webglFramebuffer;A.isWebGLCubeRenderTarget?(Array.isArray(te[H])?$=te[H][Y]:$=te[H],ft=!0):A.samples>0&&D.useMultisampledRTT(A)===!1?$=zt.get(A).__webglMultisampledFramebuffer:Array.isArray(te)?$=te[Y]:$=te,I.copy(A.viewport),V.copy(A.scissor),W=A.scissorTest}else I.copy(gt).multiplyScalar(j).floor(),V.copy(Lt).multiplyScalar(j).floor(),W=At;if(Y!==0&&($=ho),Vt.bindFramebuffer(B.FRAMEBUFFER,$)&&K&&Vt.drawBuffers(A,$),Vt.viewport(I),Vt.scissor(V),Vt.setScissorTest(W),ft){const It=zt.get(A.texture);B.framebufferTexture2D(B.FRAMEBUFFER,B.COLOR_ATTACHMENT0,B.TEXTURE_CUBE_MAP_POSITIVE_X+H,It.__webglTexture,Y)}else if(xt){const It=zt.get(A.texture),Qt=H;B.framebufferTextureLayer(B.FRAMEBUFFER,B.COLOR_ATTACHMENT0,It.__webglTexture,Y,Qt)}else if(A!==null&&Y!==0){const It=zt.get(A.texture);B.framebufferTexture2D(B.FRAMEBUFFER,B.COLOR_ATTACHMENT0,B.TEXTURE_2D,It.__webglTexture,Y)}y=-1},this.readRenderTargetPixels=function(A,H,Y,K,$,ft,xt){if(!(A&&A.isWebGLRenderTarget)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");return}let Ct=zt.get(A).__webglFramebuffer;if(A.isWebGLCubeRenderTarget&&xt!==void 0&&(Ct=Ct[xt]),Ct){Vt.bindFramebuffer(B.FRAMEBUFFER,Ct);try{const It=A.texture,Qt=It.format,te=It.type;if(!pe.textureFormatReadable(Qt)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in RGBA or implementation defined format.");return}if(!pe.textureTypeReadable(te)){console.error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not in UnsignedByteType or implementation defined type.");return}H>=0&&H<=A.width-K&&Y>=0&&Y<=A.height-$&&B.readPixels(H,Y,K,$,ae.convert(Qt),ae.convert(te),ft)}finally{const It=k!==null?zt.get(k).__webglFramebuffer:null;Vt.bindFramebuffer(B.FRAMEBUFFER,It)}}},this.readRenderTargetPixelsAsync=async function(A,H,Y,K,$,ft,xt){if(!(A&&A.isWebGLRenderTarget))throw new Error("THREE.WebGLRenderer.readRenderTargetPixels: renderTarget is not THREE.WebGLRenderTarget.");let Ct=zt.get(A).__webglFramebuffer;if(A.isWebGLCubeRenderTarget&&xt!==void 0&&(Ct=Ct[xt]),Ct){const It=A.texture,Qt=It.format,te=It.type;if(!pe.textureFormatReadable(Qt))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in RGBA or implementation defined format.");if(!pe.textureTypeReadable(te))throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: renderTarget is not in UnsignedByteType or implementation defined type.");if(H>=0&&H<=A.width-K&&Y>=0&&Y<=A.height-$){Vt.bindFramebuffer(B.FRAMEBUFFER,Ct);const Ht=B.createBuffer();B.bindBuffer(B.PIXEL_PACK_BUFFER,Ht),B.bufferData(B.PIXEL_PACK_BUFFER,ft.byteLength,B.STREAM_READ),B.readPixels(H,Y,K,$,ae.convert(Qt),ae.convert(te),0);const ce=k!==null?zt.get(k).__webglFramebuffer:null;Vt.bindFramebuffer(B.FRAMEBUFFER,ce);const be=B.fenceSync(B.SYNC_GPU_COMMANDS_COMPLETE,0);return B.flush(),await hm(B,be,4),B.bindBuffer(B.PIXEL_PACK_BUFFER,Ht),B.getBufferSubData(B.PIXEL_PACK_BUFFER,0,ft),B.deleteBuffer(Ht),B.deleteSync(be),ft}else throw new Error("THREE.WebGLRenderer.readRenderTargetPixelsAsync: requested read bounds are out of range.")}},this.copyFramebufferToTexture=function(A,H=null,Y=0){A.isTexture!==!0&&(ji("WebGLRenderer: copyFramebufferToTexture function signature has changed."),H=arguments[0]||null,A=arguments[1]);const K=Math.pow(2,-Y),$=Math.floor(A.image.width*K),ft=Math.floor(A.image.height*K),xt=H!==null?H.x:0,Ct=H!==null?H.y:0;D.setTexture2D(A,0),B.copyTexSubImage2D(B.TEXTURE_2D,Y,0,0,xt,Ct,$,ft),Vt.unbindTexture()};const fo=B.createFramebuffer(),po=B.createFramebuffer();this.copyTextureToTexture=function(A,H,Y=null,K=null,$=0,ft=null){A.isTexture!==!0&&(ji("WebGLRenderer: copyTextureToTexture function signature has changed."),K=arguments[0]||null,A=arguments[1],H=arguments[2],ft=arguments[3]||0,Y=null),ft===null&&($!==0?(ji("WebGLRenderer: copyTextureToTexture function signature has changed to support src and dst mipmap levels."),ft=$,$=0):ft=0);let xt,Ct,It,Qt,te,Ht,ce,be,Oe;const ke=A.isCompressedTexture?A.mipmaps[ft]:A.image;if(Y!==null)xt=Y.max.x-Y.min.x,Ct=Y.max.y-Y.min.y,It=Y.isBox3?Y.max.z-Y.min.z:1,Qt=Y.min.x,te=Y.min.y,Ht=Y.isBox3?Y.min.z:0;else{const nn=Math.pow(2,-$);xt=Math.floor(ke.width*nn),Ct=Math.floor(ke.height*nn),A.isDataArrayTexture?It=ke.depth:A.isData3DTexture?It=Math.floor(ke.depth*nn):It=1,Qt=0,te=0,Ht=0}K!==null?(ce=K.x,be=K.y,Oe=K.z):(ce=0,be=0,Oe=0);const xe=ae.convert(H.format),$t=ae.convert(H.type);let je;H.isData3DTexture?(D.setTexture3D(H,0),je=B.TEXTURE_3D):H.isDataArrayTexture||H.isCompressedArrayTexture?(D.setTexture2DArray(H,0),je=B.TEXTURE_2D_ARRAY):(D.setTexture2D(H,0),je=B.TEXTURE_2D),B.pixelStorei(B.UNPACK_FLIP_Y_WEBGL,H.flipY),B.pixelStorei(B.UNPACK_PREMULTIPLY_ALPHA_WEBGL,H.premultiplyAlpha),B.pixelStorei(B.UNPACK_ALIGNMENT,H.unpackAlignment);const ye=B.getParameter(B.UNPACK_ROW_LENGTH),gn=B.getParameter(B.UNPACK_IMAGE_HEIGHT),De=B.getParameter(B.UNPACK_SKIP_PIXELS),Qe=B.getParameter(B.UNPACK_SKIP_ROWS),Bi=B.getParameter(B.UNPACK_SKIP_IMAGES);B.pixelStorei(B.UNPACK_ROW_LENGTH,ke.width),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,ke.height),B.pixelStorei(B.UNPACK_SKIP_PIXELS,Qt),B.pixelStorei(B.UNPACK_SKIP_ROWS,te),B.pixelStorei(B.UNPACK_SKIP_IMAGES,Ht);const Ne=A.isDataArrayTexture||A.isData3DTexture,cn=H.isDataArrayTexture||H.isData3DTexture;if(A.isDepthTexture){const nn=zt.get(A),Ge=zt.get(H),Xe=zt.get(nn.__renderTarget),ms=zt.get(Ge.__renderTarget);Vt.bindFramebuffer(B.READ_FRAMEBUFFER,Xe.__webglFramebuffer),Vt.bindFramebuffer(B.DRAW_FRAMEBUFFER,ms.__webglFramebuffer);for(let Jn=0;Jn<It;Jn++)Ne&&(B.framebufferTextureLayer(B.READ_FRAMEBUFFER,B.COLOR_ATTACHMENT0,zt.get(A).__webglTexture,$,Ht+Jn),B.framebufferTextureLayer(B.DRAW_FRAMEBUFFER,B.COLOR_ATTACHMENT0,zt.get(H).__webglTexture,ft,Oe+Jn)),B.blitFramebuffer(Qt,te,xt,Ct,ce,be,xt,Ct,B.DEPTH_BUFFER_BIT,B.NEAREST);Vt.bindFramebuffer(B.READ_FRAMEBUFFER,null),Vt.bindFramebuffer(B.DRAW_FRAMEBUFFER,null)}else if($!==0||A.isRenderTargetTexture||zt.has(A)){const nn=zt.get(A),Ge=zt.get(H);Vt.bindFramebuffer(B.READ_FRAMEBUFFER,fo),Vt.bindFramebuffer(B.DRAW_FRAMEBUFFER,po);for(let Xe=0;Xe<It;Xe++)Ne?B.framebufferTextureLayer(B.READ_FRAMEBUFFER,B.COLOR_ATTACHMENT0,nn.__webglTexture,$,Ht+Xe):B.framebufferTexture2D(B.READ_FRAMEBUFFER,B.COLOR_ATTACHMENT0,B.TEXTURE_2D,nn.__webglTexture,$),cn?B.framebufferTextureLayer(B.DRAW_FRAMEBUFFER,B.COLOR_ATTACHMENT0,Ge.__webglTexture,ft,Oe+Xe):B.framebufferTexture2D(B.DRAW_FRAMEBUFFER,B.COLOR_ATTACHMENT0,B.TEXTURE_2D,Ge.__webglTexture,ft),$!==0?B.blitFramebuffer(Qt,te,xt,Ct,ce,be,xt,Ct,B.COLOR_BUFFER_BIT,B.NEAREST):cn?B.copyTexSubImage3D(je,ft,ce,be,Oe+Xe,Qt,te,xt,Ct):B.copyTexSubImage2D(je,ft,ce,be,Qt,te,xt,Ct);Vt.bindFramebuffer(B.READ_FRAMEBUFFER,null),Vt.bindFramebuffer(B.DRAW_FRAMEBUFFER,null)}else cn?A.isDataTexture||A.isData3DTexture?B.texSubImage3D(je,ft,ce,be,Oe,xt,Ct,It,xe,$t,ke.data):H.isCompressedArrayTexture?B.compressedTexSubImage3D(je,ft,ce,be,Oe,xt,Ct,It,xe,ke.data):B.texSubImage3D(je,ft,ce,be,Oe,xt,Ct,It,xe,$t,ke):A.isDataTexture?B.texSubImage2D(B.TEXTURE_2D,ft,ce,be,xt,Ct,xe,$t,ke.data):A.isCompressedTexture?B.compressedTexSubImage2D(B.TEXTURE_2D,ft,ce,be,ke.width,ke.height,xe,ke.data):B.texSubImage2D(B.TEXTURE_2D,ft,ce,be,xt,Ct,xe,$t,ke);B.pixelStorei(B.UNPACK_ROW_LENGTH,ye),B.pixelStorei(B.UNPACK_IMAGE_HEIGHT,gn),B.pixelStorei(B.UNPACK_SKIP_PIXELS,De),B.pixelStorei(B.UNPACK_SKIP_ROWS,Qe),B.pixelStorei(B.UNPACK_SKIP_IMAGES,Bi),ft===0&&H.generateMipmaps&&B.generateMipmap(je),Vt.unbindTexture()},this.copyTextureToTexture3D=function(A,H,Y=null,K=null,$=0){return A.isTexture!==!0&&(ji("WebGLRenderer: copyTextureToTexture3D function signature has changed."),Y=arguments[0]||null,K=arguments[1]||null,A=arguments[2],H=arguments[3],$=arguments[4]||0),ji('WebGLRenderer: copyTextureToTexture3D function has been deprecated. Use "copyTextureToTexture" instead.'),this.copyTextureToTexture(A,H,Y,K,$)},this.initRenderTarget=function(A){zt.get(A).__webglFramebuffer===void 0&&D.setupRenderTarget(A)},this.initTexture=function(A){A.isCubeTexture?D.setTextureCube(A,0):A.isData3DTexture?D.setTexture3D(A,0):A.isDataArrayTexture||A.isCompressedArrayTexture?D.setTexture2DArray(A,0):D.setTexture2D(A,0),Vt.unbindTexture()},this.resetState=function(){F=0,U=0,k=null,Vt.reset(),Te.reset()},typeof __THREE_DEVTOOLS__<"u"&&__THREE_DEVTOOLS__.dispatchEvent(new CustomEvent("observe",{detail:this}))}get coordinateSystem(){return li}get outputColorSpace(){return this._outputColorSpace}set outputColorSpace(t){this._outputColorSpace=t;const e=this.getContext();e.drawingBufferColorspace=_e._getDrawingBufferColorSpace(t),e.unpackColorSpace=_e._getUnpackColorSpace()}}const hh={type:"change"},yc={type:"start"},Rd={type:"end"},ba=new Pr,dh=new wi,Yx=Math.cos(70*Ds.DEG2RAD),qe=new O,vn=2*Math.PI,Pe={NONE:-1,ROTATE:0,DOLLY:1,PAN:2,TOUCH_ROTATE:3,TOUCH_PAN:4,TOUCH_DOLLY_PAN:5,TOUCH_DOLLY_ROTATE:6},Jo=1e-6;class Kx extends hg{constructor(t,e=null){super(t,e),this.state=Pe.NONE,this.enabled=!0,this.target=new O,this.cursor=new O,this.minDistance=0,this.maxDistance=1/0,this.minZoom=0,this.maxZoom=1/0,this.minTargetRadius=0,this.maxTargetRadius=1/0,this.minPolarAngle=0,this.maxPolarAngle=Math.PI,this.minAzimuthAngle=-1/0,this.maxAzimuthAngle=1/0,this.enableDamping=!1,this.dampingFactor=.05,this.enableZoom=!0,this.zoomSpeed=1,this.enableRotate=!0,this.rotateSpeed=1,this.keyRotateSpeed=1,this.enablePan=!0,this.panSpeed=1,this.screenSpacePanning=!0,this.keyPanSpeed=7,this.zoomToCursor=!1,this.autoRotate=!1,this.autoRotateSpeed=2,this.keys={LEFT:"ArrowLeft",UP:"ArrowUp",RIGHT:"ArrowRight",BOTTOM:"ArrowDown"},this.mouseButtons={LEFT:ks.ROTATE,MIDDLE:ks.DOLLY,RIGHT:ks.PAN},this.touches={ONE:Is.ROTATE,TWO:Is.DOLLY_PAN},this.target0=this.target.clone(),this.position0=this.object.position.clone(),this.zoom0=this.object.zoom,this._domElementKeyEvents=null,this._lastPosition=new O,this._lastQuaternion=new Nn,this._lastTargetPosition=new O,this._quat=new Nn().setFromUnitVectors(t.up,new O(0,1,0)),this._quatInverse=this._quat.clone().invert(),this._spherical=new Ou,this._sphericalDelta=new Ou,this._scale=1,this._panOffset=new O,this._rotateStart=new Yt,this._rotateEnd=new Yt,this._rotateDelta=new Yt,this._panStart=new Yt,this._panEnd=new Yt,this._panDelta=new Yt,this._dollyStart=new Yt,this._dollyEnd=new Yt,this._dollyDelta=new Yt,this._dollyDirection=new O,this._mouse=new Yt,this._performCursorZoom=!1,this._pointers=[],this._pointerPositions={},this._controlActive=!1,this._onPointerMove=Zx.bind(this),this._onPointerDown=Jx.bind(this),this._onPointerUp=Qx.bind(this),this._onContextMenu=ab.bind(this),this._onMouseWheel=nb.bind(this),this._onKeyDown=ib.bind(this),this._onTouchStart=sb.bind(this),this._onTouchMove=rb.bind(this),this._onMouseDown=tb.bind(this),this._onMouseMove=eb.bind(this),this._interceptControlDown=ob.bind(this),this._interceptControlUp=lb.bind(this),this.domElement!==null&&this.connect(),this.update()}connect(){this.domElement.addEventListener("pointerdown",this._onPointerDown),this.domElement.addEventListener("pointercancel",this._onPointerUp),this.domElement.addEventListener("contextmenu",this._onContextMenu),this.domElement.addEventListener("wheel",this._onMouseWheel,{passive:!1}),this.domElement.getRootNode().addEventListener("keydown",this._interceptControlDown,{passive:!0,capture:!0}),this.domElement.style.touchAction="none"}disconnect(){this.domElement.removeEventListener("pointerdown",this._onPointerDown),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.domElement.removeEventListener("pointercancel",this._onPointerUp),this.domElement.removeEventListener("wheel",this._onMouseWheel),this.domElement.removeEventListener("contextmenu",this._onContextMenu),this.stopListenToKeyEvents(),this.domElement.getRootNode().removeEventListener("keydown",this._interceptControlDown,{capture:!0}),this.domElement.style.touchAction="auto"}dispose(){this.disconnect()}getPolarAngle(){return this._spherical.phi}getAzimuthalAngle(){return this._spherical.theta}getDistance(){return this.object.position.distanceTo(this.target)}listenToKeyEvents(t){t.addEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=t}stopListenToKeyEvents(){this._domElementKeyEvents!==null&&(this._domElementKeyEvents.removeEventListener("keydown",this._onKeyDown),this._domElementKeyEvents=null)}saveState(){this.target0.copy(this.target),this.position0.copy(this.object.position),this.zoom0=this.object.zoom}reset(){this.target.copy(this.target0),this.object.position.copy(this.position0),this.object.zoom=this.zoom0,this.object.updateProjectionMatrix(),this.dispatchEvent(hh),this.update(),this.state=Pe.NONE}update(t=null){const e=this.object.position;qe.copy(e).sub(this.target),qe.applyQuaternion(this._quat),this._spherical.setFromVector3(qe),this.autoRotate&&this.state===Pe.NONE&&this._rotateLeft(this._getAutoRotationAngle(t)),this.enableDamping?(this._spherical.theta+=this._sphericalDelta.theta*this.dampingFactor,this._spherical.phi+=this._sphericalDelta.phi*this.dampingFactor):(this._spherical.theta+=this._sphericalDelta.theta,this._spherical.phi+=this._sphericalDelta.phi);let i=this.minAzimuthAngle,s=this.maxAzimuthAngle;isFinite(i)&&isFinite(s)&&(i<-Math.PI?i+=vn:i>Math.PI&&(i-=vn),s<-Math.PI?s+=vn:s>Math.PI&&(s-=vn),i<=s?this._spherical.theta=Math.max(i,Math.min(s,this._spherical.theta)):this._spherical.theta=this._spherical.theta>(i+s)/2?Math.max(i,this._spherical.theta):Math.min(s,this._spherical.theta)),this._spherical.phi=Math.max(this.minPolarAngle,Math.min(this.maxPolarAngle,this._spherical.phi)),this._spherical.makeSafe(),this.enableDamping===!0?this.target.addScaledVector(this._panOffset,this.dampingFactor):this.target.add(this._panOffset),this.target.sub(this.cursor),this.target.clampLength(this.minTargetRadius,this.maxTargetRadius),this.target.add(this.cursor);let r=!1;if(this.zoomToCursor&&this._performCursorZoom||this.object.isOrthographicCamera)this._spherical.radius=this._clampDistance(this._spherical.radius);else{const a=this._spherical.radius;this._spherical.radius=this._clampDistance(this._spherical.radius*this._scale),r=a!=this._spherical.radius}if(qe.setFromSpherical(this._spherical),qe.applyQuaternion(this._quatInverse),e.copy(this.target).add(qe),this.object.lookAt(this.target),this.enableDamping===!0?(this._sphericalDelta.theta*=1-this.dampingFactor,this._sphericalDelta.phi*=1-this.dampingFactor,this._panOffset.multiplyScalar(1-this.dampingFactor)):(this._sphericalDelta.set(0,0,0),this._panOffset.set(0,0,0)),this.zoomToCursor&&this._performCursorZoom){let a=null;if(this.object.isPerspectiveCamera){const o=qe.length();a=this._clampDistance(o*this._scale);const l=o-a;this.object.position.addScaledVector(this._dollyDirection,l),this.object.updateMatrixWorld(),r=!!l}else if(this.object.isOrthographicCamera){const o=new O(this._mouse.x,this._mouse.y,0);o.unproject(this.object);const l=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),this.object.updateProjectionMatrix(),r=l!==this.object.zoom;const c=new O(this._mouse.x,this._mouse.y,0);c.unproject(this.object),this.object.position.sub(c).add(o),this.object.updateMatrixWorld(),a=qe.length()}else console.warn("WARNING: OrbitControls.js encountered an unknown camera type - zoom to cursor disabled."),this.zoomToCursor=!1;a!==null&&(this.screenSpacePanning?this.target.set(0,0,-1).transformDirection(this.object.matrix).multiplyScalar(a).add(this.object.position):(ba.origin.copy(this.object.position),ba.direction.set(0,0,-1).transformDirection(this.object.matrix),Math.abs(this.object.up.dot(ba.direction))<Yx?this.object.lookAt(this.target):(dh.setFromNormalAndCoplanarPoint(this.object.up,this.target),ba.intersectPlane(dh,this.target))))}else if(this.object.isOrthographicCamera){const a=this.object.zoom;this.object.zoom=Math.max(this.minZoom,Math.min(this.maxZoom,this.object.zoom/this._scale)),a!==this.object.zoom&&(this.object.updateProjectionMatrix(),r=!0)}return this._scale=1,this._performCursorZoom=!1,r||this._lastPosition.distanceToSquared(this.object.position)>Jo||8*(1-this._lastQuaternion.dot(this.object.quaternion))>Jo||this._lastTargetPosition.distanceToSquared(this.target)>Jo?(this.dispatchEvent(hh),this._lastPosition.copy(this.object.position),this._lastQuaternion.copy(this.object.quaternion),this._lastTargetPosition.copy(this.target),!0):!1}_getAutoRotationAngle(t){return t!==null?vn/60*this.autoRotateSpeed*t:vn/60/60*this.autoRotateSpeed}_getZoomScale(t){const e=Math.abs(t*.01);return Math.pow(.95,this.zoomSpeed*e)}_rotateLeft(t){this._sphericalDelta.theta-=t}_rotateUp(t){this._sphericalDelta.phi-=t}_panLeft(t,e){qe.setFromMatrixColumn(e,0),qe.multiplyScalar(-t),this._panOffset.add(qe)}_panUp(t,e){this.screenSpacePanning===!0?qe.setFromMatrixColumn(e,1):(qe.setFromMatrixColumn(e,0),qe.crossVectors(this.object.up,qe)),qe.multiplyScalar(t),this._panOffset.add(qe)}_pan(t,e){const i=this.domElement;if(this.object.isPerspectiveCamera){const s=this.object.position;qe.copy(s).sub(this.target);let r=qe.length();r*=Math.tan(this.object.fov/2*Math.PI/180),this._panLeft(2*t*r/i.clientHeight,this.object.matrix),this._panUp(2*e*r/i.clientHeight,this.object.matrix)}else this.object.isOrthographicCamera?(this._panLeft(t*(this.object.right-this.object.left)/this.object.zoom/i.clientWidth,this.object.matrix),this._panUp(e*(this.object.top-this.object.bottom)/this.object.zoom/i.clientHeight,this.object.matrix)):(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - pan disabled."),this.enablePan=!1)}_dollyOut(t){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale/=t:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_dollyIn(t){this.object.isPerspectiveCamera||this.object.isOrthographicCamera?this._scale*=t:(console.warn("WARNING: OrbitControls.js encountered an unknown camera type - dolly/zoom disabled."),this.enableZoom=!1)}_updateZoomParameters(t,e){if(!this.zoomToCursor)return;this._performCursorZoom=!0;const i=this.domElement.getBoundingClientRect(),s=t-i.left,r=e-i.top,a=i.width,o=i.height;this._mouse.x=s/a*2-1,this._mouse.y=-(r/o)*2+1,this._dollyDirection.set(this._mouse.x,this._mouse.y,1).unproject(this.object).sub(this.object.position).normalize()}_clampDistance(t){return Math.max(this.minDistance,Math.min(this.maxDistance,t))}_handleMouseDownRotate(t){this._rotateStart.set(t.clientX,t.clientY)}_handleMouseDownDolly(t){this._updateZoomParameters(t.clientX,t.clientX),this._dollyStart.set(t.clientX,t.clientY)}_handleMouseDownPan(t){this._panStart.set(t.clientX,t.clientY)}_handleMouseMoveRotate(t){this._rotateEnd.set(t.clientX,t.clientY),this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);const e=this.domElement;this._rotateLeft(vn*this._rotateDelta.x/e.clientHeight),this._rotateUp(vn*this._rotateDelta.y/e.clientHeight),this._rotateStart.copy(this._rotateEnd),this.update()}_handleMouseMoveDolly(t){this._dollyEnd.set(t.clientX,t.clientY),this._dollyDelta.subVectors(this._dollyEnd,this._dollyStart),this._dollyDelta.y>0?this._dollyOut(this._getZoomScale(this._dollyDelta.y)):this._dollyDelta.y<0&&this._dollyIn(this._getZoomScale(this._dollyDelta.y)),this._dollyStart.copy(this._dollyEnd),this.update()}_handleMouseMovePan(t){this._panEnd.set(t.clientX,t.clientY),this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd),this.update()}_handleMouseWheel(t){this._updateZoomParameters(t.clientX,t.clientY),t.deltaY<0?this._dollyIn(this._getZoomScale(t.deltaY)):t.deltaY>0&&this._dollyOut(this._getZoomScale(t.deltaY)),this.update()}_handleKeyDown(t){let e=!1;switch(t.code){case this.keys.UP:t.ctrlKey||t.metaKey||t.shiftKey?this.enableRotate&&this._rotateUp(vn*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(0,this.keyPanSpeed),e=!0;break;case this.keys.BOTTOM:t.ctrlKey||t.metaKey||t.shiftKey?this.enableRotate&&this._rotateUp(-vn*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(0,-this.keyPanSpeed),e=!0;break;case this.keys.LEFT:t.ctrlKey||t.metaKey||t.shiftKey?this.enableRotate&&this._rotateLeft(vn*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(this.keyPanSpeed,0),e=!0;break;case this.keys.RIGHT:t.ctrlKey||t.metaKey||t.shiftKey?this.enableRotate&&this._rotateLeft(-vn*this.keyRotateSpeed/this.domElement.clientHeight):this.enablePan&&this._pan(-this.keyPanSpeed,0),e=!0;break}e&&(t.preventDefault(),this.update())}_handleTouchStartRotate(t){if(this._pointers.length===1)this._rotateStart.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._rotateStart.set(i,s)}}_handleTouchStartPan(t){if(this._pointers.length===1)this._panStart.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._panStart.set(i,s)}}_handleTouchStartDolly(t){const e=this._getSecondPointerPosition(t),i=t.pageX-e.x,s=t.pageY-e.y,r=Math.sqrt(i*i+s*s);this._dollyStart.set(0,r)}_handleTouchStartDollyPan(t){this.enableZoom&&this._handleTouchStartDolly(t),this.enablePan&&this._handleTouchStartPan(t)}_handleTouchStartDollyRotate(t){this.enableZoom&&this._handleTouchStartDolly(t),this.enableRotate&&this._handleTouchStartRotate(t)}_handleTouchMoveRotate(t){if(this._pointers.length==1)this._rotateEnd.set(t.pageX,t.pageY);else{const i=this._getSecondPointerPosition(t),s=.5*(t.pageX+i.x),r=.5*(t.pageY+i.y);this._rotateEnd.set(s,r)}this._rotateDelta.subVectors(this._rotateEnd,this._rotateStart).multiplyScalar(this.rotateSpeed);const e=this.domElement;this._rotateLeft(vn*this._rotateDelta.x/e.clientHeight),this._rotateUp(vn*this._rotateDelta.y/e.clientHeight),this._rotateStart.copy(this._rotateEnd)}_handleTouchMovePan(t){if(this._pointers.length===1)this._panEnd.set(t.pageX,t.pageY);else{const e=this._getSecondPointerPosition(t),i=.5*(t.pageX+e.x),s=.5*(t.pageY+e.y);this._panEnd.set(i,s)}this._panDelta.subVectors(this._panEnd,this._panStart).multiplyScalar(this.panSpeed),this._pan(this._panDelta.x,this._panDelta.y),this._panStart.copy(this._panEnd)}_handleTouchMoveDolly(t){const e=this._getSecondPointerPosition(t),i=t.pageX-e.x,s=t.pageY-e.y,r=Math.sqrt(i*i+s*s);this._dollyEnd.set(0,r),this._dollyDelta.set(0,Math.pow(this._dollyEnd.y/this._dollyStart.y,this.zoomSpeed)),this._dollyOut(this._dollyDelta.y),this._dollyStart.copy(this._dollyEnd);const a=(t.pageX+e.x)*.5,o=(t.pageY+e.y)*.5;this._updateZoomParameters(a,o)}_handleTouchMoveDollyPan(t){this.enableZoom&&this._handleTouchMoveDolly(t),this.enablePan&&this._handleTouchMovePan(t)}_handleTouchMoveDollyRotate(t){this.enableZoom&&this._handleTouchMoveDolly(t),this.enableRotate&&this._handleTouchMoveRotate(t)}_addPointer(t){this._pointers.push(t.pointerId)}_removePointer(t){delete this._pointerPositions[t.pointerId];for(let e=0;e<this._pointers.length;e++)if(this._pointers[e]==t.pointerId){this._pointers.splice(e,1);return}}_isTrackingPointer(t){for(let e=0;e<this._pointers.length;e++)if(this._pointers[e]==t.pointerId)return!0;return!1}_trackPointer(t){let e=this._pointerPositions[t.pointerId];e===void 0&&(e=new Yt,this._pointerPositions[t.pointerId]=e),e.set(t.pageX,t.pageY)}_getSecondPointerPosition(t){const e=t.pointerId===this._pointers[0]?this._pointers[1]:this._pointers[0];return this._pointerPositions[e]}_customWheelEvent(t){const e=t.deltaMode,i={clientX:t.clientX,clientY:t.clientY,deltaY:t.deltaY};switch(e){case 1:i.deltaY*=16;break;case 2:i.deltaY*=100;break}return t.ctrlKey&&!this._controlActive&&(i.deltaY*=10),i}}function Jx(n){this.enabled!==!1&&(this._pointers.length===0&&(this.domElement.setPointerCapture(n.pointerId),this.domElement.addEventListener("pointermove",this._onPointerMove),this.domElement.addEventListener("pointerup",this._onPointerUp)),!this._isTrackingPointer(n)&&(this._addPointer(n),n.pointerType==="touch"?this._onTouchStart(n):this._onMouseDown(n)))}function Zx(n){this.enabled!==!1&&(n.pointerType==="touch"?this._onTouchMove(n):this._onMouseMove(n))}function Qx(n){switch(this._removePointer(n),this._pointers.length){case 0:this.domElement.releasePointerCapture(n.pointerId),this.domElement.removeEventListener("pointermove",this._onPointerMove),this.domElement.removeEventListener("pointerup",this._onPointerUp),this.dispatchEvent(Rd),this.state=Pe.NONE;break;case 1:const t=this._pointers[0],e=this._pointerPositions[t];this._onTouchStart({pointerId:t,pageX:e.x,pageY:e.y});break}}function tb(n){let t;switch(n.button){case 0:t=this.mouseButtons.LEFT;break;case 1:t=this.mouseButtons.MIDDLE;break;case 2:t=this.mouseButtons.RIGHT;break;default:t=-1}switch(t){case ks.DOLLY:if(this.enableZoom===!1)return;this._handleMouseDownDolly(n),this.state=Pe.DOLLY;break;case ks.ROTATE:if(n.ctrlKey||n.metaKey||n.shiftKey){if(this.enablePan===!1)return;this._handleMouseDownPan(n),this.state=Pe.PAN}else{if(this.enableRotate===!1)return;this._handleMouseDownRotate(n),this.state=Pe.ROTATE}break;case ks.PAN:if(n.ctrlKey||n.metaKey||n.shiftKey){if(this.enableRotate===!1)return;this._handleMouseDownRotate(n),this.state=Pe.ROTATE}else{if(this.enablePan===!1)return;this._handleMouseDownPan(n),this.state=Pe.PAN}break;default:this.state=Pe.NONE}this.state!==Pe.NONE&&this.dispatchEvent(yc)}function eb(n){switch(this.state){case Pe.ROTATE:if(this.enableRotate===!1)return;this._handleMouseMoveRotate(n);break;case Pe.DOLLY:if(this.enableZoom===!1)return;this._handleMouseMoveDolly(n);break;case Pe.PAN:if(this.enablePan===!1)return;this._handleMouseMovePan(n);break}}function nb(n){this.enabled===!1||this.enableZoom===!1||this.state!==Pe.NONE||(n.preventDefault(),this.dispatchEvent(yc),this._handleMouseWheel(this._customWheelEvent(n)),this.dispatchEvent(Rd))}function ib(n){this.enabled!==!1&&this._handleKeyDown(n)}function sb(n){switch(this._trackPointer(n),this._pointers.length){case 1:switch(this.touches.ONE){case Is.ROTATE:if(this.enableRotate===!1)return;this._handleTouchStartRotate(n),this.state=Pe.TOUCH_ROTATE;break;case Is.PAN:if(this.enablePan===!1)return;this._handleTouchStartPan(n),this.state=Pe.TOUCH_PAN;break;default:this.state=Pe.NONE}break;case 2:switch(this.touches.TWO){case Is.DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchStartDollyPan(n),this.state=Pe.TOUCH_DOLLY_PAN;break;case Is.DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchStartDollyRotate(n),this.state=Pe.TOUCH_DOLLY_ROTATE;break;default:this.state=Pe.NONE}break;default:this.state=Pe.NONE}this.state!==Pe.NONE&&this.dispatchEvent(yc)}function rb(n){switch(this._trackPointer(n),this.state){case Pe.TOUCH_ROTATE:if(this.enableRotate===!1)return;this._handleTouchMoveRotate(n),this.update();break;case Pe.TOUCH_PAN:if(this.enablePan===!1)return;this._handleTouchMovePan(n),this.update();break;case Pe.TOUCH_DOLLY_PAN:if(this.enableZoom===!1&&this.enablePan===!1)return;this._handleTouchMoveDollyPan(n),this.update();break;case Pe.TOUCH_DOLLY_ROTATE:if(this.enableZoom===!1&&this.enableRotate===!1)return;this._handleTouchMoveDollyRotate(n),this.update();break;default:this.state=Pe.NONE}}function ab(n){this.enabled!==!1&&n.preventDefault()}function ob(n){n.key==="Control"&&(this._controlActive=!0,this.domElement.getRootNode().addEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}function lb(n){n.key==="Control"&&(this._controlActive=!1,this.domElement.getRootNode().removeEventListener("keyup",this._interceptControlUp,{passive:!0,capture:!0}))}class $l extends Di{constructor(t){super(t)}load(t,e,i,s){const r=this,a=new _c(this.manager);a.setPath(this.path),a.setResponseType("arraybuffer"),a.setRequestHeader(this.requestHeader),a.setWithCredentials(this.withCredentials),a.load(t,function(o){try{e(r.parse(o))}catch(l){s?s(l):console.error(l),r.manager.itemError(t)}},i,s)}parse(t){function e(c){const u=new DataView(c),h=32/8*3+32/8*3*3+16/8,d=u.getUint32(80,!0);if(80+32/8+d*h===u.byteLength)return!0;const g=[115,111,108,105,100];for(let _=0;_<5;_++)if(i(g,u,_))return!1;return!0}function i(c,u,h){for(let d=0,p=c.length;d<p;d++)if(c[d]!==u.getUint8(h+d))return!1;return!0}function s(c){const u=new DataView(c),h=u.getUint32(80,!0);let d,p,g,_=!1,m,f,P,L,w;for(let I=0;I<70;I++)u.getUint32(I,!1)==1129270351&&u.getUint8(I+4)==82&&u.getUint8(I+5)==61&&(_=!0,m=new Float32Array(h*3*3),f=u.getUint8(I+6)/255,P=u.getUint8(I+7)/255,L=u.getUint8(I+8)/255,w=u.getUint8(I+9)/255);const N=84,F=50,U=new $e,k=new Float32Array(h*3*3),y=new Float32Array(h*3*3),M=new ne;for(let I=0;I<h;I++){const V=N+I*F,W=u.getFloat32(V,!0),Z=u.getFloat32(V+4,!0),et=u.getFloat32(V+8,!0);if(_){const z=u.getUint16(V+48,!0);(z&32768)===0?(d=(z&31)/31,p=(z>>5&31)/31,g=(z>>10&31)/31):(d=f,p=P,g=L)}for(let z=1;z<=3;z++){const it=V+z*12,j=I*3*3+(z-1)*3;k[j]=u.getFloat32(it,!0),k[j+1]=u.getFloat32(it+4,!0),k[j+2]=u.getFloat32(it+8,!0),y[j]=W,y[j+1]=Z,y[j+2]=et,_&&(M.setRGB(d,p,g,ze),m[j]=M.r,m[j+1]=M.g,m[j+2]=M.b)}}return U.setAttribute("position",new dn(k,3)),U.setAttribute("normal",new dn(y,3)),_&&(U.setAttribute("color",new dn(m,3)),U.hasColors=!0,U.alpha=w),U}function r(c){const u=new $e,h=/solid([\s\S]*?)endsolid/g,d=/facet([\s\S]*?)endfacet/g,p=/solid\s(.+)/;let g=0;const _=/[\s]+([+-]?(?:\d*)(?:\.\d*)?(?:[eE][+-]?\d+)?)/.source,m=new RegExp("vertex"+_+_+_,"g"),f=new RegExp("normal"+_+_+_,"g"),P=[],L=[],w=[],N=new O;let F,U=0,k=0,y=0;for(;(F=h.exec(c))!==null;){k=y;const M=F[0],I=(F=p.exec(M))!==null?F[1]:"";for(w.push(I);(F=d.exec(M))!==null;){let Z=0,et=0;const z=F[0];for(;(F=f.exec(z))!==null;)N.x=parseFloat(F[1]),N.y=parseFloat(F[2]),N.z=parseFloat(F[3]),et++;for(;(F=m.exec(z))!==null;)P.push(parseFloat(F[1]),parseFloat(F[2]),parseFloat(F[3])),L.push(N.x,N.y,N.z),Z++,y++;et!==1&&console.error("THREE.STLLoader: Something isn't right with the normal of face number "+g),Z!==3&&console.error("THREE.STLLoader: Something isn't right with the vertices of face number "+g),g++}const V=k,W=y-k;u.userData.groupNames=w,u.addGroup(V,W,U),U++}return u.setAttribute("position",new Ae(P,3)),u.setAttribute("normal",new Ae(L,3)),u}function a(c){return typeof c!="string"?new TextDecoder().decode(c):c}function o(c){if(typeof c=="string"){const u=new Uint8Array(c.length);for(let h=0;h<c.length;h++)u[h]=c.charCodeAt(h)&255;return u.buffer||u}else return c}const l=o(t);return e(l)?s(l):r(a(t))}}class fh extends sg{constructor(t){super(t)}parse(t){function e(z){switch(z.image_type){case d:case _:if(z.colormap_length>256||z.colormap_size!==24||z.colormap_type!==1)throw new Error("THREE.TGALoader: Invalid type colormap data for indexed type.");break;case p:case g:case m:case f:if(z.colormap_type)throw new Error("THREE.TGALoader: Invalid type colormap data for colormap type.");break;case h:throw new Error("THREE.TGALoader: No data.");default:throw new Error("THREE.TGALoader: Invalid type "+z.image_type)}if(z.width<=0||z.height<=0)throw new Error("THREE.TGALoader: Invalid image size.");if(z.pixel_size!==8&&z.pixel_size!==16&&z.pixel_size!==24&&z.pixel_size!==32)throw new Error("THREE.TGALoader: Invalid pixel size "+z.pixel_size)}function i(z,it,j,ut,mt){let gt,Lt;const At=j.pixel_size>>3,X=j.width*j.height*At;if(it&&(Lt=mt.subarray(ut,ut+=j.colormap_length*(j.colormap_size>>3))),z){gt=new Uint8Array(X);let Q,nt,st,vt=0;const se=new Uint8Array(At);for(;vt<X;)if(Q=mt[ut++],nt=(Q&127)+1,Q&128){for(st=0;st<At;++st)se[st]=mt[ut++];for(st=0;st<nt;++st)gt.set(se,vt+st*At);vt+=At*nt}else{for(nt*=At,st=0;st<nt;++st)gt[vt+st]=mt[ut++];vt+=nt}}else gt=mt.subarray(ut,ut+=it?j.width*j.height:X);return{pixel_data:gt,palettes:Lt}}function s(z,it,j,ut,mt,gt,Lt,At,X){const Q=X;let nt,st=0,vt,se;const Wt=M.width;for(se=it;se!==ut;se+=j)for(vt=mt;vt!==Lt;vt+=gt,st++)nt=At[st],z[(vt+Wt*se)*4+3]=255,z[(vt+Wt*se)*4+2]=Q[nt*3+0],z[(vt+Wt*se)*4+1]=Q[nt*3+1],z[(vt+Wt*se)*4+0]=Q[nt*3+2];return z}function r(z,it,j,ut,mt,gt,Lt,At){let X,Q=0,nt,st;const vt=M.width;for(st=it;st!==ut;st+=j)for(nt=mt;nt!==Lt;nt+=gt,Q+=2)X=At[Q+0]+(At[Q+1]<<8),z[(nt+vt*st)*4+0]=(X&31744)>>7,z[(nt+vt*st)*4+1]=(X&992)>>2,z[(nt+vt*st)*4+2]=(X&31)<<3,z[(nt+vt*st)*4+3]=X&32768?0:255;return z}function a(z,it,j,ut,mt,gt,Lt,At){let X=0,Q,nt;const st=M.width;for(nt=it;nt!==ut;nt+=j)for(Q=mt;Q!==Lt;Q+=gt,X+=3)z[(Q+st*nt)*4+3]=255,z[(Q+st*nt)*4+2]=At[X+0],z[(Q+st*nt)*4+1]=At[X+1],z[(Q+st*nt)*4+0]=At[X+2];return z}function o(z,it,j,ut,mt,gt,Lt,At){let X=0,Q,nt;const st=M.width;for(nt=it;nt!==ut;nt+=j)for(Q=mt;Q!==Lt;Q+=gt,X+=4)z[(Q+st*nt)*4+2]=At[X+0],z[(Q+st*nt)*4+1]=At[X+1],z[(Q+st*nt)*4+0]=At[X+2],z[(Q+st*nt)*4+3]=At[X+3];return z}function l(z,it,j,ut,mt,gt,Lt,At){let X,Q=0,nt,st;const vt=M.width;for(st=it;st!==ut;st+=j)for(nt=mt;nt!==Lt;nt+=gt,Q++)X=At[Q],z[(nt+vt*st)*4+0]=X,z[(nt+vt*st)*4+1]=X,z[(nt+vt*st)*4+2]=X,z[(nt+vt*st)*4+3]=255;return z}function c(z,it,j,ut,mt,gt,Lt,At){let X=0,Q,nt;const st=M.width;for(nt=it;nt!==ut;nt+=j)for(Q=mt;Q!==Lt;Q+=gt,X+=2)z[(Q+st*nt)*4+0]=At[X+0],z[(Q+st*nt)*4+1]=At[X+0],z[(Q+st*nt)*4+2]=At[X+0],z[(Q+st*nt)*4+3]=At[X+1];return z}function u(z,it,j,ut,mt){let gt,Lt,At,X,Q,nt;switch((M.flags&P)>>L){default:case F:gt=0,At=1,Q=it,Lt=0,X=1,nt=j;break;case w:gt=0,At=1,Q=it,Lt=j-1,X=-1,nt=-1;break;case U:gt=it-1,At=-1,Q=-1,Lt=0,X=1,nt=j;break;case N:gt=it-1,At=-1,Q=-1,Lt=j-1,X=-1,nt=-1;break}if(W)switch(M.pixel_size){case 8:l(z,Lt,X,nt,gt,At,Q,ut);break;case 16:c(z,Lt,X,nt,gt,At,Q,ut);break;default:throw new Error("THREE.TGALoader: Format not supported.")}else switch(M.pixel_size){case 8:s(z,Lt,X,nt,gt,At,Q,ut,mt);break;case 16:r(z,Lt,X,nt,gt,At,Q,ut);break;case 24:a(z,Lt,X,nt,gt,At,Q,ut);break;case 32:o(z,Lt,X,nt,gt,At,Q,ut);break;default:throw new Error("THREE.TGALoader: Format not supported.")}return z}const h=0,d=1,p=2,g=3,_=9,m=10,f=11,P=48,L=4,w=0,N=1,F=2,U=3;if(t.length<19)throw new Error("THREE.TGALoader: Not enough data to contain header.");let k=0;const y=new Uint8Array(t),M={id_length:y[k++],colormap_type:y[k++],image_type:y[k++],colormap_index:y[k++]|y[k++]<<8,colormap_length:y[k++]|y[k++]<<8,colormap_size:y[k++],origin:[y[k++]|y[k++]<<8,y[k++]|y[k++]<<8],width:y[k++]|y[k++]<<8,height:y[k++]|y[k++]<<8,pixel_size:y[k++],flags:y[k++]};if(e(M),M.id_length+k>t.length)throw new Error("THREE.TGALoader: No data.");k+=M.id_length;let I=!1,V=!1,W=!1;switch(M.image_type){case _:I=!0,V=!0;break;case d:V=!0;break;case m:I=!0;break;case p:break;case f:I=!0,W=!0;break;case g:W=!0;break}const Z=new Uint8Array(M.width*M.height*4),et=i(I,V,M,k,y);return u(Z,M.width,M.height,et.pixel_data,et.palettes),{data:Z,width:M.width,height:M.height,flipY:!0,generateMipmaps:!0,minFilter:oi}}}class cb extends Di{load(t,e,i,s){const r=this,a=r.path===""?Sd.extractUrlBase(t):r.path,o=new _c(r.manager);o.setPath(r.path),o.setRequestHeader(r.requestHeader),o.setWithCredentials(r.withCredentials),o.load(t,function(l){try{e(r.parse(l,a))}catch(c){s?s(c):console.error(c),r.manager.itemError(t)}},i,s)}parse(t,e){function i(x,v){const C=[],T=x.childNodes;for(let R=0,J=T.length;R<J;R++){const rt=T[R];rt.nodeName===v&&C.push(rt)}return C}function s(x){if(x.length===0)return[];const v=x.trim().split(/\s+/),C=new Array(v.length);for(let T=0,R=v.length;T<R;T++)C[T]=v[T];return C}function r(x){if(x.length===0)return[];const v=x.trim().split(/\s+/),C=new Array(v.length);for(let T=0,R=v.length;T<R;T++)C[T]=parseFloat(v[T]);return C}function a(x){if(x.length===0)return[];const v=x.trim().split(/\s+/),C=new Array(v.length);for(let T=0,R=v.length;T<R;T++)C[T]=parseInt(v[T]);return C}function o(x){return x.substring(1)}function l(){return"three_default_"+Jn++}function c(x){return Object.keys(x).length===0}function u(x){return{unit:h(i(x,"unit")[0]),upAxis:d(i(x,"up_axis")[0])}}function h(x){return x!==void 0&&x.hasAttribute("meter")===!0?parseFloat(x.getAttribute("meter")):1}function d(x){return x!==void 0?x.textContent:"Y_UP"}function p(x,v,C,T){const R=i(x,v)[0];if(R!==void 0){const J=i(R,C);for(let rt=0;rt<J.length;rt++)T(J[rt])}}function g(x,v){for(const C in x){const T=x[C];T.build=v(x[C])}}function _(x,v){return x.build!==void 0||(x.build=v(x)),x.build}function m(x){const v={sources:{},samplers:{},channels:{}};let C=!1;for(let T=0,R=x.childNodes.length;T<R;T++){const J=x.childNodes[T];if(J.nodeType!==1)continue;let rt;switch(J.nodeName){case"source":rt=J.getAttribute("id"),v.sources[rt]=tt(J);break;case"sampler":rt=J.getAttribute("id"),v.samplers[rt]=f(J);break;case"channel":rt=J.getAttribute("target"),v.channels[rt]=P(J);break;case"animation":m(J),C=!0;break;default:console.log(J)}}C===!1&&(qt.animations[x.getAttribute("id")||Ds.generateUUID()]=v)}function f(x){const v={inputs:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"input":const J=o(R.getAttribute("source")),rt=R.getAttribute("semantic");v.inputs[rt]=J;break}}return v}function P(x){const v={};let T=x.getAttribute("target").split("/");const R=T.shift();let J=T.shift();const rt=J.indexOf("(")!==-1,Rt=J.indexOf(".")!==-1;if(Rt)T=J.split("."),J=T.shift(),v.member=T.shift();else if(rt){const _t=J.split("(");J=_t.shift();for(let Tt=0;Tt<_t.length;Tt++)_t[Tt]=parseInt(_t[Tt].replace(/\)/,""));v.indices=_t}return v.id=R,v.sid=J,v.arraySyntax=rt,v.memberSyntax=Rt,v.sampler=o(x.getAttribute("source")),v}function L(x){const v=[],C=x.channels,T=x.samplers,R=x.sources;for(const J in C)if(C.hasOwnProperty(J)){const rt=C[J],Rt=T[rt.sampler],_t=Rt.inputs.INPUT,Tt=Rt.inputs.OUTPUT,Bt=R[_t],dt=R[Tt],Ot=N(rt,Bt,dt);M(Ot,v)}return v}function w(x){return _(qt.animations[x],L)}function N(x,v,C){const T=qt.nodes[x.id],R=ce(T.id),J=T.transforms[x.sid],rt=T.matrix.clone().transpose();let Rt,_t,Tt,Bt,dt,Ot;const Ut={};switch(J){case"matrix":for(Tt=0,Bt=v.array.length;Tt<Bt;Tt++)if(Rt=v.array[Tt],_t=Tt*C.stride,Ut[Rt]===void 0&&(Ut[Rt]={}),x.arraySyntax===!0){const Be=C.array[_t],we=x.indices[0]+4*x.indices[1];Ut[Rt][we]=Be}else for(dt=0,Ot=C.stride;dt<Ot;dt++)Ut[Rt][dt]=C.array[_t+dt];break;case"translate":console.warn('THREE.ColladaLoader: Animation transform type "%s" not yet implemented.',J);break;case"rotate":console.warn('THREE.ColladaLoader: Animation transform type "%s" not yet implemented.',J);break;case"scale":console.warn('THREE.ColladaLoader: Animation transform type "%s" not yet implemented.',J);break}const jt=F(Ut,rt);return{name:R.uuid,keyframes:jt}}function F(x,v){const C=[];for(const R in x)C.push({time:parseFloat(R),value:x[R]});C.sort(T);for(let R=0;R<16;R++)I(C,R,v.elements[R]);return C;function T(R,J){return R.time-J.time}}const U=new O,k=new O,y=new Nn;function M(x,v){const C=x.keyframes,T=x.name,R=[],J=[],rt=[],Rt=[];for(let _t=0,Tt=C.length;_t<Tt;_t++){const Bt=C[_t],dt=Bt.time,Ot=Bt.value;H.fromArray(Ot).transpose(),H.decompose(U,y,k),R.push(dt),J.push(U.x,U.y,U.z),rt.push(y.x,y.y,y.z,y.w),Rt.push(k.x,k.y,k.z)}return J.length>0&&v.push(new Zs(T+".position",R,J)),rt.length>0&&v.push(new Dr(T+".quaternion",R,rt)),Rt.length>0&&v.push(new Zs(T+".scale",R,Rt)),v}function I(x,v,C){let T,R=!0,J,rt;for(J=0,rt=x.length;J<rt;J++)T=x[J],T.value[v]===void 0?T.value[v]=null:R=!1;if(R===!0)for(J=0,rt=x.length;J<rt;J++)T=x[J],T.value[v]=C;else V(x,v)}function V(x,v){let C,T;for(let R=0,J=x.length;R<J;R++){const rt=x[R];if(rt.value[v]===null){if(C=W(x,R,v),T=Z(x,R,v),C===null){rt.value[v]=T.value[v];continue}if(T===null){rt.value[v]=C.value[v];continue}et(rt,C,T,v)}}}function W(x,v,C){for(;v>=0;){const T=x[v];if(T.value[C]!==null)return T;v--}return null}function Z(x,v,C){for(;v<x.length;){const T=x[v];if(T.value[C]!==null)return T;v++}return null}function et(x,v,C,T){if(C.time-v.time===0){x.value[T]=v.value[T];return}x.value[T]=(x.time-v.time)*(C.value[T]-v.value[T])/(C.time-v.time)+v.value[T]}function z(x){const v={name:x.getAttribute("id")||"default",start:parseFloat(x.getAttribute("start")||0),end:parseFloat(x.getAttribute("end")||0),animations:[]};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"instance_animation":v.animations.push(o(R.getAttribute("url")));break}}qt.clips[x.getAttribute("id")]=v}function it(x){const v=[],C=x.name,T=x.end-x.start||-1,R=x.animations;for(let J=0,rt=R.length;J<rt;J++){const Rt=w(R[J]);for(let _t=0,Tt=Rt.length;_t<Tt;_t++)v.push(Rt[_t])}return new Iu(C,T,v)}function j(x){return _(qt.clips[x],it)}function ut(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"skin":v.id=o(R.getAttribute("source")),v.skin=mt(R);break;case"morph":v.id=o(R.getAttribute("source")),console.warn("THREE.ColladaLoader: Morph target animation not supported yet.");break}}qt.controllers[x.getAttribute("id")]=v}function mt(x){const v={sources:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"bind_shape_matrix":v.bindShapeMatrix=r(R.textContent);break;case"source":const J=R.getAttribute("id");v.sources[J]=tt(R);break;case"joints":v.joints=gt(R);break;case"vertex_weights":v.vertexWeights=Lt(R);break}}return v}function gt(x){const v={inputs:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"input":const J=R.getAttribute("semantic"),rt=o(R.getAttribute("source"));v.inputs[J]=rt;break}}return v}function Lt(x){const v={inputs:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"input":const J=R.getAttribute("semantic"),rt=o(R.getAttribute("source")),Rt=parseInt(R.getAttribute("offset"));v.inputs[J]={id:rt,offset:Rt};break;case"vcount":v.vcount=a(R.textContent);break;case"v":v.v=a(R.textContent);break}}return v}function At(x){const v={id:x.id},C=qt.geometries[v.id];return x.skin!==void 0&&(v.skin=X(x.skin),C.sources.skinIndices=v.skin.indices,C.sources.skinWeights=v.skin.weights),v}function X(x){const C={joints:[],indices:{array:[],stride:4},weights:{array:[],stride:4}},T=x.sources,R=x.vertexWeights,J=R.vcount,rt=R.v,Rt=R.inputs.JOINT.offset,_t=R.inputs.WEIGHT.offset,Tt=x.sources[x.joints.inputs.JOINT],Bt=x.sources[x.joints.inputs.INV_BIND_MATRIX],dt=T[R.inputs.WEIGHT.id].array;let Ot=0,Ut,jt,Gt;for(Ut=0,Gt=J.length;Ut<Gt;Ut++){const we=J[Ut],Me=[];for(jt=0;jt<we;jt++){const Se=rt[Ot+Rt],Zn=rt[Ot+_t],_n=dt[Zn];Me.push({index:Se,weight:_n}),Ot+=2}for(Me.sort(Be),jt=0;jt<4;jt++){const Se=Me[jt];Se!==void 0?(C.indices.array.push(Se.index),C.weights.array.push(Se.weight)):(C.indices.array.push(0),C.weights.array.push(0))}}for(x.bindShapeMatrix?C.bindMatrix=new Kt().fromArray(x.bindShapeMatrix).transpose():C.bindMatrix=new Kt().identity(),Ut=0,Gt=Tt.array.length;Ut<Gt;Ut++){const we=Tt.array[Ut],Me=new Kt().fromArray(Bt.array,Ut*Bt.stride).transpose();C.joints.push({name:we,boneInverse:Me})}return C;function Be(we,Me){return Me.weight-we.weight}}function Q(x){return _(qt.controllers[x],At)}function nt(x){const v={init_from:i(x,"init_from")[0].textContent};qt.images[x.getAttribute("id")]=v}function st(x){return x.build!==void 0?x.build:x.init_from}function vt(x){const v=qt.images[x];return v!==void 0?_(v,st):(console.warn("THREE.ColladaLoader: Couldn't find image with ID:",x),null)}function se(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"profile_COMMON":v.profile=Wt(R);break}}qt.effects[x.getAttribute("id")]=v}function Wt(x){const v={surfaces:{},samplers:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"newparam":Fe(R,v);break;case"technique":v.technique=B(R);break;case"extra":v.extra=zt(R);break}}return v}function Fe(x,v){const C=x.getAttribute("sid");for(let T=0,R=x.childNodes.length;T<R;T++){const J=x.childNodes[T];if(J.nodeType===1)switch(J.nodeName){case"surface":v.surfaces[C]=Ie(J);break;case"sampler2D":v.samplers[C]=me(J);break}}}function Ie(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"init_from":v.init_from=R.textContent;break}}return v}function me(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"source":v.source=R.textContent;break}}return v}function B(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"constant":case"lambert":case"blinn":case"phong":v.type=R.nodeName,v.parameters=pn(R);break;case"extra":v.extra=zt(R);break}}return v}function pn(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"emission":case"diffuse":case"specular":case"bump":case"ambient":case"shininess":case"transparency":v[R.nodeName]=fe(R);break;case"transparent":v[R.nodeName]={opaque:R.hasAttribute("opaque")?R.getAttribute("opaque"):"A_ONE",data:fe(R)};break}}return v}function fe(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"color":v[R.nodeName]=r(R.textContent);break;case"float":v[R.nodeName]=parseFloat(R.textContent);break;case"texture":v[R.nodeName]={id:R.getAttribute("texture"),extra:pe(R)};break}}return v}function pe(x){const v={technique:{}};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"extra":Vt(R,v);break}}return v}function Vt(x,v){for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"technique":Le(R,v);break}}}function Le(x,v){for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"repeatU":case"repeatV":case"offsetU":case"offsetV":v.technique[R.nodeName]=parseFloat(R.textContent);break;case"wrapU":case"wrapV":R.textContent.toUpperCase()==="TRUE"?v.technique[R.nodeName]=1:R.textContent.toUpperCase()==="FALSE"?v.technique[R.nodeName]=0:v.technique[R.nodeName]=parseInt(R.textContent);break;case"bump":v[R.nodeName]=S(R);break}}}function zt(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"technique":v.technique=D(R);break}}return v}function D(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"double_sided":v[R.nodeName]=parseInt(R.textContent);break;case"bump":v[R.nodeName]=S(R);break}}return v}function S(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"texture":v[R.nodeName]={id:R.getAttribute("texture"),texcoord:R.getAttribute("texcoord"),extra:pe(R)};break}}return v}function q(x){return x}function lt(x){return _(qt.effects[x],q)}function ht(x){const v={name:x.getAttribute("name")};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"instance_effect":v.url=o(R.getAttribute("url"));break}}qt.materials[x.getAttribute("id")]=v}function ot(x){let v,C=x.slice((x.lastIndexOf(".")-1>>>0)+2);switch(C=C.toLowerCase(),C){case"tga":v=nn;break;default:v=cn}return v}function Ft(x){const v=lt(x.url),C=v.profile.technique;let T;switch(C.type){case"phong":case"blinn":T=new br;break;case"lambert":T=new $m;break;default:T=new Er;break}T.name=x.name||"";function R(_t,Tt=null){const Bt=v.profile.samplers[_t.id];let dt=null;if(Bt!==void 0){const Ot=v.profile.surfaces[Bt.source];dt=vt(Ot.init_from)}else console.warn("THREE.ColladaLoader: Undefined sampler. Access image directly (see #12530)."),dt=vt(_t.id);if(dt!==null){const Ot=ot(dt);if(Ot!==void 0){const Ut=Ot.load(dt),jt=_t.extra;if(jt!==void 0&&jt.technique!==void 0&&c(jt.technique)===!1){const Gt=jt.technique;Ut.wrapS=Gt.wrapU?Ki:zn,Ut.wrapT=Gt.wrapV?Ki:zn,Ut.offset.set(Gt.offsetU||0,Gt.offsetV||0),Ut.repeat.set(Gt.repeatU||1,Gt.repeatV||1)}else Ut.wrapS=Ki,Ut.wrapT=Ki;return Tt!==null&&(Ut.colorSpace=Tt),Ut}else return console.warn("THREE.ColladaLoader: Loader for texture %s not found.",dt),null}else return console.warn("THREE.ColladaLoader: Couldn't create texture with ID:",_t.id),null}const J=C.parameters;for(const _t in J){const Tt=J[_t];switch(_t){case"diffuse":Tt.color&&T.color.fromArray(Tt.color),Tt.texture&&(T.map=R(Tt.texture,ze));break;case"specular":Tt.color&&T.specular&&T.specular.fromArray(Tt.color),Tt.texture&&(T.specularMap=R(Tt.texture));break;case"bump":Tt.texture&&(T.normalMap=R(Tt.texture));break;case"ambient":Tt.texture&&(T.lightMap=R(Tt.texture,ze));break;case"shininess":Tt.float&&T.shininess&&(T.shininess=Tt.float);break;case"emission":Tt.color&&T.emissive&&T.emissive.fromArray(Tt.color),Tt.texture&&(T.emissiveMap=R(Tt.texture,ze));break}}_e.toWorkingColorSpace(T.color,ze),T.specular&&_e.toWorkingColorSpace(T.specular,ze),T.emissive&&_e.toWorkingColorSpace(T.emissive,ze);let rt=J.transparent,Rt=J.transparency;if(Rt===void 0&&rt&&(Rt={float:1}),rt===void 0&&Rt&&(rt={opaque:"A_ONE",data:{color:[1,1,1,1]}}),rt&&Rt)if(rt.data.texture)T.transparent=!0;else{const _t=rt.data.color;switch(rt.opaque){case"A_ONE":T.opacity=_t[3]*Rt.float;break;case"RGB_ZERO":T.opacity=1-_t[0]*Rt.float;break;case"A_ZERO":T.opacity=1-_t[3]*Rt.float;break;case"RGB_ONE":T.opacity=_t[0]*Rt.float;break;default:console.warn('THREE.ColladaLoader: Invalid opaque type "%s" of transparent tag.',rt.opaque)}T.opacity<1&&(T.transparent=!0)}if(C.extra!==void 0&&C.extra.technique!==void 0){const _t=C.extra.technique;for(const Tt in _t){const Bt=_t[Tt];switch(Tt){case"double_sided":T.side=Bt===1?Tn:ui;break;case"bump":T.normalMap=R(Bt.texture),T.normalScale=new Yt(1,1);break}}}return T}function St(x){return _(qt.materials[x],Ft)}function Pt(x){const v={name:x.getAttribute("name")};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"optics":v.optics=ve(R);break}}qt.cameras[x.getAttribute("id")]=v}function ve(x){for(let v=0;v<x.childNodes.length;v++){const C=x.childNodes[v];switch(C.nodeName){case"technique_common":return pt(C)}}return{}}function pt(x){const v={};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];switch(T.nodeName){case"perspective":case"orthographic":v.technique=T.nodeName,v.parameters=Nt(T);break}}return v}function Nt(x){const v={};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];switch(T.nodeName){case"xfov":case"yfov":case"xmag":case"ymag":case"znear":case"zfar":case"aspect_ratio":v[T.nodeName]=parseFloat(T.textContent);break}}return v}function Xt(x){let v;switch(x.optics.technique){case"perspective":v=new an(x.optics.parameters.yfov,x.optics.parameters.aspect_ratio,x.optics.parameters.znear,x.optics.parameters.zfar);break;case"orthographic":let C=x.optics.parameters.ymag,T=x.optics.parameters.xmag;const R=x.optics.parameters.aspect_ratio;T=T===void 0?C*R:T,C=C===void 0?T/R:C,T*=.5,C*=.5,v=new xc(-T,T,C,-C,x.optics.parameters.znear,x.optics.parameters.zfar);break;default:v=new an;break}return v.name=x.name||"",v}function Zt(x){const v=qt.cameras[x];return v!==void 0?_(v,Xt):(console.warn("THREE.ColladaLoader: Couldn't find camera with ID:",x),null)}function Dt(x){let v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"technique_common":v=ge(R);break}}qt.lights[x.getAttribute("id")]=v}function ge(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"directional":case"point":case"spot":case"ambient":v.technique=R.nodeName,v.parameters=ae(R)}}return v}function ae(x){const v={};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"color":const J=r(R.textContent);v.color=new ne().fromArray(J),_e.toWorkingColorSpace(v.color,ze);break;case"falloff_angle":v.falloffAngle=parseFloat(R.textContent);break;case"quadratic_attenuation":const rt=parseFloat(R.textContent);v.distance=rt?Math.sqrt(1/rt):0;break}}return v}function Te(x){let v;switch(x.technique){case"directional":v=new Na;break;case"point":v=new lg;break;case"spot":v=new ag;break;case"ambient":v=new Md;break}return x.parameters.color&&v.color.copy(x.parameters.color),x.parameters.distance&&(v.distance=x.parameters.distance),v}function G(x){const v=qt.lights[x];return v!==void 0?_(v,Te):(console.warn("THREE.ColladaLoader: Couldn't find light with ID:",x),null)}function Mt(x){const v={name:x.getAttribute("name"),sources:{},vertices:{},primitives:[]},C=i(x,"mesh")[0];if(C!==void 0){for(let T=0;T<C.childNodes.length;T++){const R=C.childNodes[T];if(R.nodeType!==1)continue;const J=R.getAttribute("id");switch(R.nodeName){case"source":v.sources[J]=tt(R);break;case"vertices":v.vertices=ct(R);break;case"polygons":console.warn("THREE.ColladaLoader: Unsupported primitive type: ",R.nodeName);break;case"lines":case"linestrips":case"polylist":case"triangles":v.primitives.push(wt(R));break;default:console.log(R)}}qt.geometries[x.getAttribute("id")]=v}}function tt(x){const v={array:[],stride:3};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"float_array":v.array=r(T.textContent);break;case"Name_array":v.array=s(T.textContent);break;case"technique_common":const R=i(T,"accessor")[0];R!==void 0&&(v.stride=parseInt(R.getAttribute("stride")));break}}return v}function ct(x){const v={};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];T.nodeType===1&&(v[T.getAttribute("semantic")]=o(T.getAttribute("source")))}return v}function wt(x){const v={type:x.nodeName,material:x.getAttribute("material"),count:parseInt(x.getAttribute("count")),inputs:{},stride:0,hasUV:!1};for(let C=0,T=x.childNodes.length;C<T;C++){const R=x.childNodes[C];if(R.nodeType===1)switch(R.nodeName){case"input":const J=o(R.getAttribute("source")),rt=R.getAttribute("semantic"),Rt=parseInt(R.getAttribute("offset")),_t=parseInt(R.getAttribute("set")),Tt=_t>0?rt+_t:rt;v.inputs[Tt]={id:J,offset:Rt},v.stride=Math.max(v.stride,Rt+1),rt==="TEXCOORD"&&(v.hasUV=!0);break;case"vcount":v.vcount=a(R.textContent);break;case"p":v.p=a(R.textContent);break}}return v}function Et(x){const v={};for(let C=0;C<x.length;C++){const T=x[C];v[T.type]===void 0&&(v[T.type]=[]),v[T.type].push(T)}return v}function re(x){let v=0;for(let C=0,T=x.length;C<T;C++)x[C].hasUV===!0&&v++;v>0&&v<x.length&&(x.uvsNeedsFix=!0)}function Ue(x){const v={},C=x.sources,T=x.vertices,R=x.primitives;if(R.length===0)return{};const J=Et(R);for(const rt in J){const Rt=J[rt];re(Rt),v[rt]=Je(Rt,C,T)}return v}function Je(x,v,C){const T={},R={array:[],stride:0},J={array:[],stride:0},rt={array:[],stride:0},Rt={array:[],stride:0},_t={array:[],stride:0},Tt={array:[],stride:4},Bt={array:[],stride:4},dt=new $e,Ot=[];let Ut=0;for(let jt=0;jt<x.length;jt++){const Gt=x[jt],Be=Gt.inputs;let we=0;switch(Gt.type){case"lines":case"linestrips":we=Gt.count*2;break;case"triangles":we=Gt.count*3;break;case"polylist":for(let Me=0;Me<Gt.count;Me++){const Se=Gt.vcount[Me];switch(Se){case 3:we+=3;break;case 4:we+=6;break;default:we+=(Se-2)*3;break}}break;default:console.warn("THREE.ColladaLoader: Unknown primitive type:",Gt.type)}dt.addGroup(Ut,we,jt),Ut+=we,Gt.material&&Ot.push(Gt.material);for(const Me in Be){const Se=Be[Me];switch(Me){case"VERTEX":for(const Zn in C){const _n=C[Zn];switch(Zn){case"POSITION":const gs=R.array.length;if(oe(Gt,v[_n],Se.offset,R.array),R.stride=v[_n].stride,v.skinWeights&&v.skinIndices&&(oe(Gt,v.skinIndices,Se.offset,Tt.array),oe(Gt,v.skinWeights,Se.offset,Bt.array)),Gt.hasUV===!1&&x.uvsNeedsFix===!0){const bf=(R.array.length-gs)/R.stride;for(let Gc=0;Gc<bf;Gc++)rt.array.push(0,0)}break;case"NORMAL":oe(Gt,v[_n],Se.offset,J.array),J.stride=v[_n].stride;break;case"COLOR":oe(Gt,v[_n],Se.offset,_t.array),_t.stride=v[_n].stride;break;case"TEXCOORD":oe(Gt,v[_n],Se.offset,rt.array),rt.stride=v[_n].stride;break;case"TEXCOORD1":oe(Gt,v[_n],Se.offset,Rt.array),rt.stride=v[_n].stride;break;default:console.warn('THREE.ColladaLoader: Semantic "%s" not handled in geometry build process.',Zn)}}break;case"NORMAL":oe(Gt,v[Se.id],Se.offset,J.array),J.stride=v[Se.id].stride;break;case"COLOR":oe(Gt,v[Se.id],Se.offset,_t.array,!0),_t.stride=v[Se.id].stride;break;case"TEXCOORD":oe(Gt,v[Se.id],Se.offset,rt.array),rt.stride=v[Se.id].stride;break;case"TEXCOORD1":oe(Gt,v[Se.id],Se.offset,Rt.array),Rt.stride=v[Se.id].stride;break}}}return R.array.length>0&&dt.setAttribute("position",new Ae(R.array,R.stride)),J.array.length>0&&dt.setAttribute("normal",new Ae(J.array,J.stride)),_t.array.length>0&&dt.setAttribute("color",new Ae(_t.array,_t.stride)),rt.array.length>0&&dt.setAttribute("uv",new Ae(rt.array,rt.stride)),Rt.array.length>0&&dt.setAttribute("uv1",new Ae(Rt.array,Rt.stride)),Tt.array.length>0&&dt.setAttribute("skinIndex",new Ae(Tt.array,Tt.stride)),Bt.array.length>0&&dt.setAttribute("skinWeight",new Ae(Bt.array,Bt.stride)),T.data=dt,T.type=x[0].type,T.materialKeys=Ot,T}function oe(x,v,C,T,R=!1){const J=x.p,rt=x.stride,Rt=x.vcount;function _t(dt){let Ot=J[dt+C]*Bt;const Ut=Ot+Bt;for(;Ot<Ut;Ot++)T.push(Tt[Ot]);if(R){const jt=T.length-Bt-1;Ge.setRGB(T[jt+0],T[jt+1],T[jt+2],ze),T[jt+0]=Ge.r,T[jt+1]=Ge.g,T[jt+2]=Ge.b}}const Tt=v.array,Bt=v.stride;if(x.vcount!==void 0){let dt=0;for(let Ot=0,Ut=Rt.length;Ot<Ut;Ot++){const jt=Rt[Ot];if(jt===4){const Gt=dt+rt*0,Be=dt+rt*1,we=dt+rt*2,Me=dt+rt*3;_t(Gt),_t(Be),_t(Me),_t(Be),_t(we),_t(Me)}else if(jt===3){const Gt=dt+rt*0,Be=dt+rt*1,we=dt+rt*2;_t(Gt),_t(Be),_t(we)}else if(jt>4)for(let Gt=1,Be=jt-2;Gt<=Be;Gt++){const we=dt+rt*0,Me=dt+rt*Gt,Se=dt+rt*(Gt+1);_t(we),_t(Me),_t(Se)}dt+=rt*jt}}else for(let dt=0,Ot=J.length;dt<Ot;dt+=rt)_t(dt)}function mn(x){return _(qt.geometries[x],Ue)}function Dn(x){const v={name:x.getAttribute("name")||"",joints:{},links:[]};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"technique_common":Kn(T,v);break}}qt.kinematicsModels[x.getAttribute("id")]=v}function kr(x){return x.build!==void 0?x.build:x}function Fr(x){return _(qt.kinematicsModels[x],kr)}function Kn(x,v){for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"joint":v.joints[T.getAttribute("sid")]=or(T);break;case"link":v.links.push(lr(T));break}}}function or(x){let v;for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"prismatic":case"revolute":v=Or(T);break}}return v}function Or(x){const v={sid:x.getAttribute("sid"),name:x.getAttribute("name")||"",axis:new O,limits:{min:0,max:0},type:x.nodeName,static:!1,zeroPosition:0,middlePosition:0};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"axis":const R=r(T.textContent);v.axis.fromArray(R);break;case"limits":const J=T.getElementsByTagName("max")[0],rt=T.getElementsByTagName("min")[0];v.limits.max=parseFloat(J.textContent),v.limits.min=parseFloat(rt.textContent);break}}return v.limits.min>=v.limits.max&&(v.static=!0),v.middlePosition=(v.limits.min+v.limits.max)/2,v}function lr(x){const v={sid:x.getAttribute("sid"),name:x.getAttribute("name")||"",attachments:[],transforms:[]};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"attachment_full":v.attachments.push(fs(T));break;case"matrix":case"translate":case"rotate":v.transforms.push(cr(T));break}}return v}function fs(x){const v={joint:x.getAttribute("joint").split("/").pop(),transforms:[],links:[]};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"link":v.links.push(lr(T));break;case"matrix":case"translate":case"rotate":v.transforms.push(cr(T));break}}return v}function cr(x){const v={type:x.nodeName},C=r(x.textContent);switch(v.type){case"matrix":v.obj=new Kt,v.obj.fromArray(C).transpose();break;case"translate":v.obj=new O,v.obj.fromArray(C);break;case"rotate":v.obj=new O,v.obj.fromArray(C),v.angle=Ds.degToRad(C[3]);break}return v}function ps(x){const v={name:x.getAttribute("name")||"",rigidBodies:{}};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"rigid_body":v.rigidBodies[T.getAttribute("name")]={},Br(T,v.rigidBodies[T.getAttribute("name")]);break}}qt.physicsModels[x.getAttribute("id")]=v}function Br(x,v){for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"technique_common":zr(T,v);break}}}function zr(x,v){for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"inertia":v.inertia=r(T.textContent);break;case"mass":v.mass=r(T.textContent)[0];break}}}function lo(x){const v={bindJointAxis:[]};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"bind_joint_axis":v.bindJointAxis.push(co(T));break}}qt.kinematicsScenes[o(x.getAttribute("url"))]=v}function co(x){const v={target:x.getAttribute("target").split("/").pop()};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType===1)switch(T.nodeName){case"axis":const R=T.getElementsByTagName("param")[0];v.axis=R.textContent;const J=v.axis.split("inst_").pop().split("axis")[0];v.jointIndex=J.substring(0,J.length-1);break}}return v}function uo(x){return x.build!==void 0?x.build:x}function ho(x){return _(qt.kinematicsScenes[x],uo)}function fo(){const x=Object.keys(qt.kinematicsModels)[0],v=Object.keys(qt.kinematicsScenes)[0],C=Object.keys(qt.visualScenes)[0];if(x===void 0||v===void 0)return;const T=Fr(x),R=ho(v),J=xe(C),rt=R.bindJointAxis,Rt={};for(let Bt=0,dt=rt.length;Bt<dt;Bt++){const Ot=rt[Bt],Ut=De.querySelector('[sid="'+Ot.target+'"]');if(Ut){const jt=Ut.parentElement;_t(Ot.jointIndex,jt)}}function _t(Bt,dt){const Ot=dt.getAttribute("name"),Ut=T.joints[Bt];J.traverse(function(jt){jt.name===Ot&&(Rt[Bt]={object:jt,transforms:po(dt),joint:Ut,position:Ut.zeroPosition})})}const Tt=new Kt;ms={joints:T&&T.joints,getJointValue:function(Bt){const dt=Rt[Bt];if(dt)return dt.position;console.warn("THREE.ColladaLoader: Joint "+Bt+" doesn't exist.")},setJointValue:function(Bt,dt){const Ot=Rt[Bt];if(Ot){const Ut=Ot.joint;if(dt>Ut.limits.max||dt<Ut.limits.min)console.warn("THREE.ColladaLoader: Joint "+Bt+" value "+dt+" outside of limits (min: "+Ut.limits.min+", max: "+Ut.limits.max+").");else if(Ut.static)console.warn("THREE.ColladaLoader: Joint "+Bt+" is static.");else{const jt=Ot.object,Gt=Ut.axis,Be=Ot.transforms;H.identity();for(let we=0;we<Be.length;we++){const Me=Be[we];if(Me.sid&&Me.sid.indexOf(Bt)!==-1)switch(Ut.type){case"revolute":H.multiply(Tt.makeRotationAxis(Gt,Ds.degToRad(dt)));break;case"prismatic":H.multiply(Tt.makeTranslation(Gt.x*dt,Gt.y*dt,Gt.z*dt));break;default:console.warn("THREE.ColladaLoader: Unknown joint type: "+Ut.type);break}else switch(Me.type){case"matrix":H.multiply(Me.obj);break;case"translate":H.multiply(Tt.makeTranslation(Me.obj.x,Me.obj.y,Me.obj.z));break;case"scale":H.scale(Me.obj);break;case"rotate":H.multiply(Tt.makeRotationAxis(Me.obj,Me.angle));break}}jt.matrix.copy(H),jt.matrix.decompose(jt.position,jt.quaternion,jt.scale),Rt[Bt].position=dt}}else console.log("THREE.ColladaLoader: "+Bt+" does not exist.")}}}function po(x){const v=[],C=De.querySelector('[id="'+x.id+'"]');for(let T=0;T<C.childNodes.length;T++){const R=C.childNodes[T];if(R.nodeType!==1)continue;let J,rt;switch(R.nodeName){case"matrix":J=r(R.textContent);const Rt=new Kt().fromArray(J).transpose();v.push({sid:R.getAttribute("sid"),type:R.nodeName,obj:Rt});break;case"translate":case"scale":J=r(R.textContent),rt=new O().fromArray(J),v.push({sid:R.getAttribute("sid"),type:R.nodeName,obj:rt});break;case"rotate":J=r(R.textContent),rt=new O().fromArray(J);const _t=Ds.degToRad(J[3]);v.push({sid:R.getAttribute("sid"),type:R.nodeName,obj:rt,angle:_t});break}}return v}function A(x){const v=x.getElementsByTagName("node");for(let C=0;C<v.length;C++){const T=v[C];T.hasAttribute("id")===!1&&T.setAttribute("id",l())}}const H=new Kt,Y=new O;function K(x){const v={name:x.getAttribute("name")||"",type:x.getAttribute("type"),id:x.getAttribute("id"),sid:x.getAttribute("sid"),matrix:new Kt,nodes:[],instanceCameras:[],instanceControllers:[],instanceLights:[],instanceGeometries:[],instanceNodes:[],transforms:{}};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];if(T.nodeType!==1)continue;let R;switch(T.nodeName){case"node":v.nodes.push(T.getAttribute("id")),K(T);break;case"instance_camera":v.instanceCameras.push(o(T.getAttribute("url")));break;case"instance_controller":v.instanceControllers.push($(T));break;case"instance_light":v.instanceLights.push(o(T.getAttribute("url")));break;case"instance_geometry":v.instanceGeometries.push($(T));break;case"instance_node":v.instanceNodes.push(o(T.getAttribute("url")));break;case"matrix":R=r(T.textContent),v.matrix.multiply(H.fromArray(R).transpose()),v.transforms[T.getAttribute("sid")]=T.nodeName;break;case"translate":R=r(T.textContent),Y.fromArray(R),v.matrix.multiply(H.makeTranslation(Y.x,Y.y,Y.z)),v.transforms[T.getAttribute("sid")]=T.nodeName;break;case"rotate":R=r(T.textContent);const J=Ds.degToRad(R[3]);v.matrix.multiply(H.makeRotationAxis(Y.fromArray(R),J)),v.transforms[T.getAttribute("sid")]=T.nodeName;break;case"scale":R=r(T.textContent),v.matrix.scale(Y.fromArray(R)),v.transforms[T.getAttribute("sid")]=T.nodeName;break;case"extra":break;default:console.log(T)}}return Ht(v.id)?console.warn("THREE.ColladaLoader: There is already a node with ID %s. Exclude current node from further processing.",v.id):qt.nodes[v.id]=v,v}function $(x){const v={id:o(x.getAttribute("url")),materials:{},skeletons:[]};for(let C=0;C<x.childNodes.length;C++){const T=x.childNodes[C];switch(T.nodeName){case"bind_material":const R=T.getElementsByTagName("instance_material");for(let J=0;J<R.length;J++){const rt=R[J],Rt=rt.getAttribute("symbol"),_t=rt.getAttribute("target");v.materials[Rt]=o(_t)}break;case"skeleton":v.skeletons.push(o(T.textContent));break}}return v}function ft(x,v){const C=[],T=[];let R,J,rt;for(R=0;R<x.length;R++){const Tt=x[R];let Bt;if(Ht(Tt))Bt=ce(Tt),xt(Bt,v,C);else if(ke(Tt)){const Ot=qt.visualScenes[Tt].children;for(let Ut=0;Ut<Ot.length;Ut++){const jt=Ot[Ut];if(jt.type==="JOINT"){const Gt=ce(jt.id);xt(Gt,v,C)}}}else console.error("THREE.ColladaLoader: Unable to find root bone of skeleton with ID:",Tt)}for(R=0;R<v.length;R++)for(J=0;J<C.length;J++)if(rt=C[J],rt.bone.name===v[R].name){T[R]=rt,rt.processed=!0;break}for(R=0;R<C.length;R++)rt=C[R],rt.processed===!1&&(T.push(rt),rt.processed=!0);const Rt=[],_t=[];for(R=0;R<T.length;R++)rt=T[R],Rt.push(rt.bone),_t.push(rt.boneInverse);return new fc(Rt,_t)}function xt(x,v,C){x.traverse(function(T){if(T.isBone===!0){let R;for(let J=0;J<v.length;J++){const rt=v[J];if(rt.name===T.name){R=rt.boneInverse;break}}R===void 0&&(R=new Kt),C.push({bone:T,boneInverse:R,processed:!1})}})}function Ct(x){const v=[],C=x.matrix,T=x.nodes,R=x.type,J=x.instanceCameras,rt=x.instanceControllers,Rt=x.instanceLights,_t=x.instanceGeometries,Tt=x.instanceNodes;for(let dt=0,Ot=T.length;dt<Ot;dt++)v.push(ce(T[dt]));for(let dt=0,Ot=J.length;dt<Ot;dt++){const Ut=Zt(J[dt]);Ut!==null&&v.push(Ut.clone())}for(let dt=0,Ot=rt.length;dt<Ot;dt++){const Ut=rt[dt],jt=Q(Ut.id),Gt=mn(jt.id),Be=te(Gt,Ut.materials),we=Ut.skeletons,Me=jt.skin.joints,Se=ft(we,Me);for(let Zn=0,_n=Be.length;Zn<_n;Zn++){const gs=Be[Zn];gs.isSkinnedMesh&&(gs.bind(Se,jt.skin.bindMatrix),gs.normalizeSkinWeights()),v.push(gs)}}for(let dt=0,Ot=Rt.length;dt<Ot;dt++){const Ut=G(Rt[dt]);Ut!==null&&v.push(Ut.clone())}for(let dt=0,Ot=_t.length;dt<Ot;dt++){const Ut=_t[dt],jt=mn(Ut.id),Gt=te(jt,Ut.materials);for(let Be=0,we=Gt.length;Be<we;Be++)v.push(Gt[Be])}for(let dt=0,Ot=Tt.length;dt<Ot;dt++)v.push(ce(Tt[dt]).clone());let Bt;if(T.length===0&&v.length===1)Bt=v[0];else{Bt=R==="JOINT"?new fd:new He;for(let dt=0;dt<v.length;dt++)Bt.add(v[dt])}return Bt.name=R==="JOINT"?x.sid:x.name,Bt.matrix.copy(C),Bt.matrix.decompose(Bt.position,Bt.quaternion,Bt.scale),Bt}const It=new Er({name:Di.DEFAULT_MATERIAL_NAME,color:16711935});function Qt(x,v){const C=[];for(let T=0,R=x.length;T<R;T++){const J=v[x[T]];J===void 0?(console.warn("THREE.ColladaLoader: Material with key %s not found. Apply fallback material.",x[T]),C.push(It)):C.push(St(J))}return C}function te(x,v){const C=[];for(const T in x){const R=x[T],J=Qt(R.materialKeys,v);if(J.length===0&&(T==="lines"||T==="linestrips"?J.push(new Ci):J.push(new br)),T==="lines"||T==="linestrips")for(let Tt=0,Bt=J.length;Tt<Bt;Tt++){const dt=J[Tt];if(dt.isMeshPhongMaterial===!0||dt.isMeshLambertMaterial===!0){const Ot=new Ci;Ot.color.copy(dt.color),Ot.opacity=dt.opacity,Ot.transparent=dt.transparent,J[Tt]=Ot}}const rt=R.data.attributes.skinIndex!==void 0,Rt=J.length===1?J[0]:J;let _t;switch(T){case"lines":_t=new _r(R.data,Rt);break;case"linestrips":_t=new mc(R.data,Rt);break;case"triangles":case"polylist":rt?_t=new Bm(R.data,Rt):_t=new Re(R.data,Rt);break}C.push(_t)}return C}function Ht(x){return qt.nodes[x]!==void 0}function ce(x){return _(qt.nodes[x],Ct)}function be(x){const v={name:x.getAttribute("name"),children:[]};A(x);const C=i(x,"node");for(let T=0;T<C.length;T++)v.children.push(K(C[T]));qt.visualScenes[x.getAttribute("id")]=v}function Oe(x){const v=new He;v.name=x.name;const C=x.children;for(let T=0;T<C.length;T++){const R=C[T];v.add(ce(R.id))}return v}function ke(x){return qt.visualScenes[x]!==void 0}function xe(x){return _(qt.visualScenes[x],Oe)}function $t(x){const v=i(x,"instance_visual_scene")[0];return xe(o(v.getAttribute("url")))}function je(){const x=qt.clips;if(c(x)===!0){if(c(qt.animations)===!1){const v=[];for(const C in qt.animations){const T=w(C);for(let R=0,J=T.length;R<J;R++)v.push(T[R])}Xe.push(new Iu("default",-1,v))}}else for(const v in x)Xe.push(j(v))}function ye(x){let v="";const C=[x];for(;C.length;){const T=C.shift();T.nodeType===Node.TEXT_NODE?v+=T.textContent:(v+=`
`,C.push(...T.childNodes))}return v.trim()}if(t.length===0)return{scene:new dd};const gn=new DOMParser().parseFromString(t,"application/xml"),De=i(gn,"COLLADA")[0],Qe=gn.getElementsByTagName("parsererror")[0];if(Qe!==void 0){const x=i(Qe,"div")[0];let v;return x?v=x.textContent:v=ye(Qe),console.error(`THREE.ColladaLoader: Failed to parse collada file.
`,v),null}const Bi=De.getAttribute("version");console.debug("THREE.ColladaLoader: File version",Bi);const Ne=u(i(De,"asset")[0]),cn=new yd(this.manager);cn.setPath(this.resourcePath||e).setCrossOrigin(this.crossOrigin);let nn;fh&&(nn=new fh(this.manager),nn.setPath(this.resourcePath||e));const Ge=new ne,Xe=[];let ms={},Jn=0;const qt={animations:{},clips:{},controllers:{},images:{},effects:{},materials:{},cameras:{},lights:{},geometries:{},nodes:{},visualScenes:{},kinematicsModels:{},physicsModels:{},kinematicsScenes:{}};p(De,"library_animations","animation",m),p(De,"library_animation_clips","animation_clip",z),p(De,"library_controllers","controller",ut),p(De,"library_images","image",nt),p(De,"library_effects","effect",se),p(De,"library_materials","material",ht),p(De,"library_cameras","camera",Pt),p(De,"library_lights","light",Dt),p(De,"library_geometries","geometry",Mt),p(De,"library_nodes","node",K),p(De,"library_visual_scenes","visual_scene",be),p(De,"library_kinematics_models","kinematics_model",Dn),p(De,"library_physics_models","physics_model",ps),p(De,"scene","instance_kinematics_scene",lo),g(qt.animations,L),g(qt.clips,it),g(qt.controllers,At),g(qt.images,st),g(qt.effects,q),g(qt.materials,Ft),g(qt.cameras,Xt),g(qt.lights,Te),g(qt.geometries,Ue),g(qt.visualScenes,Oe),je(),fo();const Vr=$t(i(De,"scene")[0]);return Vr.animations=Xe,Ne.upAxis==="Z_UP"&&(console.warn("THREE.ColladaLoader: You are loading an asset with a Z-UP coordinate system. The loader just rotates the asset to transform it into Y-UP. The vertex data are not converted, see #24289."),Vr.rotation.set(-Math.PI/2,0,0)),Vr.scale.multiplyScalar(Ne.unit),{get animations(){return console.warn("THREE.ColladaLoader: Please access animations over scene.animations now."),Xe},kinematics:ms,library:qt,scene:Vr}}}const ph=new O,ub=new yn,ya=new Kt,bi=new Kt,Ma=new Nn,Sa=new O(1,1,1),Ea=new O;class eo extends Ve{constructor(...t){super(...t),this.urdfNode=null,this.urdfName=""}copy(t,e){return super.copy(t,e),this.urdfNode=t.urdfNode,this.urdfName=t.urdfName,this}}class hb extends eo{constructor(...t){super(...t),this.isURDFCollider=!0,this.type="URDFCollider"}}class db extends eo{constructor(...t){super(...t),this.isURDFVisual=!0,this.type="URDFVisual"}}class Pd extends eo{constructor(...t){super(...t),this.isURDFLink=!0,this.type="URDFLink",this.inertial={mass:0,origin:{xyz:[0,0,0],rpy:[0,0,0]},inertia:{ixx:0,ixy:0,ixz:0,iyy:0,iyz:0,izz:0}}}copy(t,e){return super.copy(t,e),this.inertial={mass:t.inertial.mass,origin:{xyz:[...t.inertial.origin.xyz],rpy:[...t.inertial.origin.rpy]},inertia:{...t.inertial.inertia}},this}}class Ld extends eo{get jointType(){return this._jointType}set jointType(t){if(this.jointType!==t)switch(this._jointType=t,this.matrixWorldNeedsUpdate=!0,t){case"fixed":this.jointValue=[];break;case"continuous":case"revolute":case"prismatic":this.jointValue=new Array(1).fill(0);break;case"planar":this.jointValue=new Array(3).fill(0),this.axis=new O(0,0,1);break;case"floating":this.jointValue=new Array(6).fill(0);break}}get angle(){return this.jointValue[0]}constructor(...t){super(...t),this.isURDFJoint=!0,this.type="URDFJoint",this.jointValue=null,this.jointType="fixed",this.axis=new O(1,0,0),this.limit={lower:0,upper:0,effort:0,velocity:0},this.ignoreLimits=!1,this.origPosition=null,this.origQuaternion=null,this.mimicJoints=[]}copy(t,e){return super.copy(t,e),this.jointType=t.jointType,this.axis=t.axis.clone(),this.limit.lower=t.limit.lower,this.limit.upper=t.limit.upper,this.limit.effort=t.limit.effort,this.limit.velocity=t.limit.velocity,this.ignoreLimits=!1,this.jointValue=[...t.jointValue],this.origPosition=t.origPosition?t.origPosition.clone():null,this.origQuaternion=t.origQuaternion?t.origQuaternion.clone():null,this.mimicJoints=[...t.mimicJoints],this}setJointValue(...t){t=t.map(i=>i===null?null:parseFloat(i)),(!this.origPosition||!this.origQuaternion)&&(this.origPosition=this.position.clone(),this.origQuaternion=this.quaternion.clone());let e=!1;switch(this.mimicJoints.forEach(i=>{e=i.updateFromMimickedJoint(...t)||e}),this.jointType){case"fixed":return e;case"continuous":case"revolute":{let i=t[0];return i==null||i===this.jointValue[0]?e:(!this.ignoreLimits&&this.jointType==="revolute"&&(i=Math.min(this.limit.upper,i),i=Math.max(this.limit.lower,i)),this.quaternion.setFromAxisAngle(this.axis,i).premultiply(this.origQuaternion),this.jointValue[0]!==i?(this.jointValue[0]=i,this.matrixWorldNeedsUpdate=!0,!0):e)}case"prismatic":{let i=t[0];return i==null||i===this.jointValue[0]?e:(this.ignoreLimits||(i=Math.min(this.limit.upper,i),i=Math.max(this.limit.lower,i)),this.position.copy(this.origPosition),ph.copy(this.axis).applyEuler(this.rotation),this.position.addScaledVector(ph,i),this.jointValue[0]!==i?(this.jointValue[0]=i,this.matrixWorldNeedsUpdate=!0,!0):e)}case"floating":return this.jointValue.every((i,s)=>t[s]===i||t[s]===null)?e:(this.jointValue[0]=t[0]!==null?t[0]:this.jointValue[0],this.jointValue[1]=t[1]!==null?t[1]:this.jointValue[1],this.jointValue[2]=t[2]!==null?t[2]:this.jointValue[2],this.jointValue[3]=t[3]!==null?t[3]:this.jointValue[3],this.jointValue[4]=t[4]!==null?t[4]:this.jointValue[4],this.jointValue[5]=t[5]!==null?t[5]:this.jointValue[5],bi.compose(this.origPosition,this.origQuaternion,Sa),Ma.setFromEuler(ub.set(this.jointValue[3],this.jointValue[4],this.jointValue[5],"XYZ")),Ea.set(this.jointValue[0],this.jointValue[1],this.jointValue[2]),ya.compose(Ea,Ma,Sa),bi.premultiply(ya),this.position.setFromMatrixPosition(bi),this.rotation.setFromRotationMatrix(bi),this.matrixWorldNeedsUpdate=!0,!0);case"planar":return this.jointValue.every((i,s)=>t[s]===i||t[s]===null)?e:(this.jointValue[0]=t[0]!==null?t[0]:this.jointValue[0],this.jointValue[1]=t[1]!==null?t[1]:this.jointValue[1],this.jointValue[2]=t[2]!==null?t[2]:this.jointValue[2],bi.compose(this.origPosition,this.origQuaternion,Sa),Ma.setFromAxisAngle(this.axis,this.jointValue[2]),Ea.set(this.jointValue[0],this.jointValue[1],0),ya.compose(Ea,Ma,Sa),bi.premultiply(ya),this.position.setFromMatrixPosition(bi),this.rotation.setFromRotationMatrix(bi),this.matrixWorldNeedsUpdate=!0,!0)}return e}}class mh extends Ld{constructor(...t){super(...t),this.type="URDFMimicJoint",this.mimicJoint=null,this.offset=0,this.multiplier=1}updateFromMimickedJoint(...t){const e=t.map(i=>i===null?null:i*this.multiplier+this.offset);return super.setJointValue(...e)}copy(t,e){return super.copy(t,e),this.mimicJoint=t.mimicJoint,this.offset=t.offset,this.multiplier=t.multiplier,this}}class fb extends Pd{constructor(...t){super(...t),this.isURDFRobot=!0,this.urdfNode=null,this.urdfRobotNode=null,this.robotName=null,this.links=null,this.joints=null,this.colliders=null,this.visual=null,this.frames=null}copy(t,e){super.copy(t,e),this.urdfRobotNode=t.urdfRobotNode,this.robotName=t.robotName,this.links={},this.joints={},this.colliders={},this.visual={},this.traverse(i=>{i.isURDFJoint&&i.urdfName in t.joints&&(this.joints[i.urdfName]=i),i.isURDFLink&&i.urdfName in t.links&&(this.links[i.urdfName]=i),i.isURDFCollider&&i.urdfName in t.colliders&&(this.colliders[i.urdfName]=i),i.isURDFVisual&&i.urdfName in t.visual&&(this.visual[i.urdfName]=i)});for(const i in this.joints)this.joints[i].mimicJoints=this.joints[i].mimicJoints.map(s=>this.joints[s.name]);return this.frames={...this.colliders,...this.visual,...this.links,...this.joints},this}getFrame(t){return this.frames[t]}setJointValue(t,...e){const i=this.joints[t];return i?i.setJointValue(...e):!1}setJointValues(t){let e=!1;for(const i in t){const s=t[i];Array.isArray(s)?e=this.setJointValue(i,...s)||e:e=this.setJointValue(i,s)||e}return e}}const Zo=new Nn,gh=new yn;function yi(n){return n?n.trim().split(/\s+/g).map(t=>parseFloat(t)):[0,0,0]}function _h(n,t,e=!1){e||n.rotation.set(0,0,0),gh.set(t[0],t[1],t[2],"ZYX"),Zo.setFromEuler(gh),Zo.multiply(n.quaternion),n.quaternion.copy(Zo)}class pb{constructor(t){this.manager=t||bd,this.loadMeshCb=this.defaultMeshLoader.bind(this),this.parseVisual=!0,this.parseCollision=!1,this.packages="",this.workingPath="",this.fetchOptions={}}loadAsync(t){return new Promise((e,i)=>{this.load(t,e,null,i)})}load(t,e,i,s){const r=this.manager,a=Sd.extractUrlBase(t),o=this.manager.resolveURL(t);r.itemStart(o),fetch(o,this.fetchOptions).then(l=>{if(l.ok)return i&&i(null),l.text();throw new Error(`URDFLoader: Failed to load url '${o}' with error code ${l.status} : ${l.statusText}.`)}).then(l=>{const c=this.parse(l,this.workingPath||a);e(c),r.itemEnd(o)}).catch(l=>{s?s(l):console.error("URDFLoader: Error loading file.",l),r.itemError(o),r.itemEnd(o)})}parse(t,e=this.workingPath){const i=this.packages,s=this.loadMeshCb,r=this.parseVisual,a=this.parseCollision,o=this.manager,l={},c={},u={};function h(P){if(!/^package:\/\//.test(P))return e?e+P:P;const[L,w]=P.replace(/^package:\/\//,"").split(/\/(.+)/);if(typeof i=="string")return i.endsWith(L)?i+"/"+w:i+"/"+L+"/"+w;if(typeof i=="function")return i(L)+"/"+w;if(typeof i=="object")return L in i?i[L]+"/"+w:(console.error(`URDFLoader : ${L} not found in provided package list.`),null)}function d(P){let L;P instanceof Document?L=[...P.children]:P instanceof Element?L=[P]:L=[...new DOMParser().parseFromString(P,"text/xml").children];const w=L.filter(N=>N.nodeName==="robot").pop();return p(w)}function p(P){const L=[...P.children],w=L.filter(I=>I.nodeName.toLowerCase()==="link"),N=L.filter(I=>I.nodeName.toLowerCase()==="joint"),F=L.filter(I=>I.nodeName.toLowerCase()==="material"),U=new fb;U.robotName=P.getAttribute("name"),U.urdfRobotNode=P,F.forEach(I=>{const V=I.getAttribute("name");u[V]=m(I)});const k={},y={};w.forEach(I=>{const V=I.getAttribute("name"),W=P.querySelector(`child[link="${V}"]`)===null;l[V]=_(I,k,y,W?U:null)}),N.forEach(I=>{const V=I.getAttribute("name");c[V]=g(I)}),U.joints=c,U.links=l,U.colliders=y,U.visual=k;const M=Object.values(c);return M.forEach(I=>{I instanceof mh&&c[I.mimicJoint].mimicJoints.push(I)}),M.forEach(I=>{const V=new Set,W=Z=>{if(V.has(Z))throw new Error("URDFLoader: Detected an infinite loop of mimic joints.");V.add(Z),Z.mimicJoints.forEach(et=>{W(et)})};W(I)}),U.frames={...y,...k,...l,...c},U}function g(P){const L=[...P.children],w=P.getAttribute("type");let N;const F=L.find(V=>V.nodeName.toLowerCase()==="mimic");F?(N=new mh,N.mimicJoint=F.getAttribute("joint"),N.multiplier=parseFloat(F.getAttribute("multiplier")||1),N.offset=parseFloat(F.getAttribute("offset")||0)):N=new Ld,N.urdfNode=P,N.name=P.getAttribute("name"),N.urdfName=N.name,N.jointType=w;let U=null,k=null,y=[0,0,0],M=[0,0,0];L.forEach(V=>{const W=V.nodeName.toLowerCase();W==="origin"?(y=yi(V.getAttribute("xyz")),M=yi(V.getAttribute("rpy"))):W==="child"?k=l[V.getAttribute("link")]:W==="parent"?U=l[V.getAttribute("link")]:W==="limit"&&(N.limit.lower=parseFloat(V.getAttribute("lower")||N.limit.lower),N.limit.upper=parseFloat(V.getAttribute("upper")||N.limit.upper),N.limit.effort=parseFloat(V.getAttribute("effort")||N.limit.effort),N.limit.velocity=parseFloat(V.getAttribute("velocity")||N.limit.velocity))}),U.add(N),N.add(k),_h(N,M),N.position.set(y[0],y[1],y[2]);const I=L.filter(V=>V.nodeName.toLowerCase()==="axis")[0];if(I){const V=I.getAttribute("xyz").split(/\s+/g).map(W=>parseFloat(W));N.axis=new O(V[0],V[1],V[2]),N.axis.normalize()}return N}function _(P,L,w,N=null){N===null&&(N=new Pd);const F=[...P.children];N.name=P.getAttribute("name"),N.urdfName=N.name,N.urdfNode=P;const U=F.find(k=>k.nodeName.toLowerCase()==="inertial");return U&&[...U.children].forEach(k=>{const y=k.nodeName.toLowerCase();y==="origin"?(N.inertial.origin.xyz=yi(k.getAttribute("xyz")),N.inertial.origin.rpy=yi(k.getAttribute("rpy"))):y==="mass"?N.inertial.mass=parseFloat(k.getAttribute("value"))||0:y==="inertia"&&(N.inertial.inertia.ixx=parseFloat(k.getAttribute("ixx"))||0,N.inertial.inertia.ixy=parseFloat(k.getAttribute("ixy"))||0,N.inertial.inertia.ixz=parseFloat(k.getAttribute("ixz"))||0,N.inertial.inertia.iyy=parseFloat(k.getAttribute("iyy"))||0,N.inertial.inertia.iyz=parseFloat(k.getAttribute("iyz"))||0,N.inertial.inertia.izz=parseFloat(k.getAttribute("izz"))||0)}),r&&F.filter(y=>y.nodeName.toLowerCase()==="visual").forEach(y=>{const M=f(y,u);if(N.add(M),y.hasAttribute("name")){const I=y.getAttribute("name");M.name=I,M.urdfName=I,L[I]=M}}),a&&F.filter(y=>y.nodeName.toLowerCase()==="collision").forEach(y=>{const M=f(y);if(N.add(M),y.hasAttribute("name")){const I=y.getAttribute("name");M.name=I,M.urdfName=I,w[I]=M}}),N}function m(P){const L=[...P.children],w=new br;return w.name=P.getAttribute("name")||"",L.forEach(N=>{const F=N.nodeName.toLowerCase();if(F==="color"){const U=N.getAttribute("rgba").split(/\s/g).map(k=>parseFloat(k));w.color.setRGB(U[0],U[1],U[2]),w.opacity=U[3],w.transparent=U[3]<1,w.depthWrite=!w.transparent}else if(F==="texture"){const U=N.getAttribute("filename");if(U){const k=new yd(o),y=h(U);w.map=k.load(y),w.map.colorSpace=ze}}}),w}function f(P,L={}){const w=P.nodeName.toLowerCase()==="collision",N=[...P.children];let F=null;const U=N.filter(y=>y.nodeName.toLowerCase()==="material")[0];if(U){const y=U.getAttribute("name");y&&y in L?F=L[y]:F=m(U)}else F=new br;const k=w?new hb:new db;return k.urdfNode=P,N.forEach(y=>{const M=y.nodeName.toLowerCase();if(M==="geometry"){const I=y.children[0].nodeName.toLowerCase();if(I==="mesh"){const V=y.children[0].getAttribute("filename"),W=h(V);if(W!==null){const Z=y.children[0].getAttribute("scale");if(Z){const et=yi(Z);k.scale.set(et[0],et[1],et[2])}s(W,o,(et,z)=>{z?console.error("URDFLoader: Error loading mesh.",z):et&&(et instanceof Re&&(et.material=F),et.position.set(0,0,0),et.quaternion.identity(),k.add(et))})}}else if(I==="box"){const V=new Re;V.geometry=new Bn(1,1,1),V.material=F;const W=yi(y.children[0].getAttribute("size"));V.scale.set(W[0],W[1],W[2]),k.add(V)}else if(I==="sphere"){const V=new Re;V.geometry=new Tr(1,30,30),V.material=F;const W=parseFloat(y.children[0].getAttribute("radius"))||0;V.scale.set(W,W,W),k.add(V)}else if(I==="cylinder"){const V=new Re;V.geometry=new Lr(1,1,1,30),V.material=F;const W=parseFloat(y.children[0].getAttribute("radius"))||0,Z=parseFloat(y.children[0].getAttribute("length"))||0;V.scale.set(W,Z,W),V.rotation.set(Math.PI/2,0,0),k.add(V)}}else if(M==="origin"){const I=yi(y.getAttribute("xyz")),V=yi(y.getAttribute("rpy"));k.position.set(I[0],I[1],I[2]),k.rotation.set(0,0,0),_h(k,V)}}),k}return d(t)}defaultMeshLoader(t,e,i){/\.stl$/i.test(t)?new $l(e).load(t,r=>{const a=new Re(r,new br);i(a)},null,r=>i(null,r)):/\.dae$/i.test(t)?new cb(e).load(t,r=>i(r.scene),null,r=>i(null,r)):console.warn(`URDFLoader: Could not load model at ${t}.
No loader available`)}}const mb="/urdf/unoarm.urdf",gb="/models/part.stl",vh=.001,Ia=.082951078,Nd=.035,_b=Nd,vb=.12,xb=3;class bb{constructor(t,e={}){Wc(this,"animate",()=>{var i,s;this.animationFrame=requestAnimationFrame(this.animate);const t=performance.now(),e=Math.min(.1,Math.max(0,(t-this.lastAnimationTime)/1e3));if(this.lastAnimationTime=t,this.updateRenderedRobot(e),this.updateRenderedMujocoObject(e),this.resize(),this.controls.update(),this.renderer.render(this.scene,this.camera),this.frameCounter+=1,t-this.frameWindowStart>=1e3){const r=Math.round(this.frameCounter*1e3/(t-this.frameWindowStart));(s=(i=this.callbacks).onFps)==null||s.call(i,r),this.frameCounter=0,this.frameWindowStart=t}});this.canvas=t,this.callbacks=e,this.scene=new dd,this.scene.background=new ne(1581863),this.camera=new an(42,1,.01,100),this.renderer=new qx({canvas:t,antialias:!0,alpha:!1}),this.renderer.shadowMap.enabled=!0,this.renderer.shadowMap.type=Gh,this.renderer.toneMapping=Wh,this.renderer.toneMappingExposure=1.18,this.renderer.outputColorSpace=ze,this.controls=new Kx(this.camera,t),this.controls.enableDamping=!0,this.controls.dampingFactor=.07,this.controls.minDistance=.7,this.controls.maxDistance=8,this.controls.target.set(0,.85+Ia,0),this.robot=null,this.robotBounds=null,this.jointLimits=new Map,this.jointTargets=new Map,this.jointDisplayValues=new Map,this.targetGroup=new He,this.tcpFrameGroup=new He,this.benchmarkGroup=new He,this.visionGroup=new He,this.visionPartGeometry=null,this.visionPartMarker=null,this.visionPartPoint=null,this.visionPartLoadPromise=null,this.mujocoExtraGeometryPromises={},this.cameraMarkerGroup=new He,this.platformGroup=new He,this.workspaceBoundsGroup=new He,this.manualCollisionGroup=new He,this.objectMarkerGroup=new He,this.mujocoSceneGroup=new He,this.mujocoTable=null,this.mujocoObject=null,this.mujocoObjectState=null,this.mujocoObjectTargetPosition=null,this.mujocoObjectTargetQuaternion=null,this.mujocoObjectPendingJump=null,this.cloudPoints=null,this.cameraExtrinsic=null,this.cloudVisible=!0,this.frameCounter=0,this.frameWindowStart=performance.now(),this.lastAnimationTime=this.frameWindowStart,this.lastWidth=0,this.lastHeight=0,this.scene.add(this.targetGroup,this.tcpFrameGroup,this.benchmarkGroup,this.visionGroup,this.cameraMarkerGroup,this.platformGroup,this.workspaceBoundsGroup,this.manualCollisionGroup,this.objectMarkerGroup,this.mujocoSceneGroup),this.addLighting(),this.addGround(),this.setDefaultView(),this.resizeObserver=new ResizeObserver(()=>this.resize()),this.resizeObserver.observe(t.parentElement),this.loadRobot(),this.loadVisionPartModel(),this.animate()}addLighting(){const t=new Na(16777215,1.45);t.position.set(3.5,5.5,4),t.castShadow=!0,t.shadow.mapSize.setScalar(2048),t.shadow.camera.near=.1,t.shadow.camera.far=15,t.shadow.camera.left=-3,t.shadow.camera.right=3,t.shadow.camera.top=3,t.shadow.camera.bottom=-3,this.scene.add(t);const e=new Na(12180973,.62);e.position.set(-4,3,-2.5),this.scene.add(e);const i=new Na(16768432,.34);i.position.set(0,2,-4),this.scene.add(i),this.scene.add(new Md(16777215,.5))}addGround(){const t=new Re(new Nr(12,12),new Wm({color:4937815,roughness:.82,metalness:.08,clearcoat:.08,side:Tn}));t.rotation.x=-Math.PI/2,t.position.y=-.002,t.receiveShadow=!0,this.scene.add(t);const e=this.createGrid();e.position.y=.001,this.scene.add(e)}createGrid(){const t=new He,e=8,i=40,s=e/i,r=e/2,a=new Ci({color:3425095,transparent:!0,opacity:.68}),o=new Ci({color:5399142,transparent:!0,opacity:.72});for(let l=0;l<=i;l+=1){const c=-r+l*s,u=new $e().setFromPoints([new O(-r,0,c),new O(r,0,c)]),h=new $e().setFromPoints([new O(c,0,-r),new O(c,0,r)]),d=l%5===0?o:a;t.add(new _r(u,d),new _r(h,d))}return t}loadRobot(){const t=new xd,e=new pb(t);let i=null;t.onProgress=(s,r,a)=>{var o,l;(l=(o=this.callbacks).onLoadProgress)==null||l.call(o,r,a)},t.onLoad=()=>{var s,r;i&&(this.robot=i,this.scene.add(i),i.updateMatrixWorld(!0),this.robotBounds=new Li().setFromObject(i),this.fitView(),(r=(s=this.callbacks).onLoaded)==null||r.call(s,i))},e.load(mb,s=>{i=s,s.name="UnoArmURDF",s.rotation.set(0,0,0),s.scale.setScalar(1),s.position.set(0,Ia,0),s.traverse(r=>{r.castShadow=!0,r.receiveShadow=!0,r.material&&(r.material.side=Tn,"roughness"in r.material&&(r.material.roughness=.58),r.material.needsUpdate=!0)})},void 0,s=>{var r,a;console.error("Failed to load UnoArm URDF",s),(a=(r=this.callbacks).onLoadError)==null||a.call(r,s)})}applyJointState(t){if(!this.robot||!(t!=null&&t.name)||!(t!=null&&t.position))return 0;let e=0;return t.name.forEach((i,s)=>{var l;const r=(l=this.robot.joints)==null?void 0:l[i];let a=Number(t.position[s]);if(!r||!Number.isFinite(a))return;const o=this.jointLimits.get(i);o&&(r.limit&&(r.limit.lower=o.lower,r.limit.upper=o.upper),a=Math.min(o.upper,Math.max(o.lower,a))),this.jointTargets.set(i,a),this.jointDisplayValues.has(i)||(this.jointDisplayValues.set(i,a),r.setJointValue(a)),e+=1}),e}updateRenderedRobot(t){var s;if(!this.robot||this.jointTargets.size===0)return;const e=1-Math.exp(-Math.max(0,t)/Nd);let i=!1;for(const[r,a]of this.jointTargets){const o=(s=this.robot.joints)==null?void 0:s[r];if(!o)continue;const l=this.jointDisplayValues.get(r)??a;let c=a-l;o.jointType==="continuous"&&(c=Math.atan2(Math.sin(c),Math.cos(c)));const u=Math.abs(c)<1e-5?a:l+c*e;Math.abs(u-l)>1e-7&&(i=!0),this.jointDisplayValues.set(r,u),o.setJointValue(u)}i&&this.robot.updateMatrixWorld(!0)}setJointLimits(t={}){this.jointLimits=new Map(Object.entries(t))}loadVisionPartModel(){if(this.visionPartLoadPromise)return this.visionPartLoadPromise;const t=new $l;return this.visionPartLoadPromise=new Promise(e=>{t.load(gb,i=>{var a;i.computeVertexNormals(),i.computeBoundingBox();const s=new O;(a=i.boundingBox)==null||a.getCenter(s),i.translate(-s.x,-s.y,-s.z),i.computeBoundingBox(),this.visionPartGeometry=i;const r=this.ensureVisionPartMarker();this.ensureMujocoObject(),r&&this.visionPartPoint&&(r.position.copy(this.visionPartPoint),r.visible=!0),e(i)},void 0,i=>{var s,r;(r=(s=this.callbacks).onStatus)==null||r.call(s,`零件模型加载失败: ${(i==null?void 0:i.message)||i}`),e(null)})}),this.visionPartLoadPromise}ensureVisionPartMarker(){if(!this.visionPartGeometry)return null;if(this.visionPartMarker)return this.visionPartMarker;const t=new He,e=new He,i=new Re(this.visionPartGeometry,new Fn({color:15775311,emissive:4139788,roughness:.42,metalness:.08}));i.castShadow=!0,i.receiveShadow=!0;const s=new _r(new Nu(this.visionPartGeometry,35),new Ci({color:16767371,transparent:!0,opacity:.55}));return e.add(i,s),e.scale.setScalar(vh),e.rotation.x=-Math.PI/2,t.add(e),t.visible=!!this.visionPartPoint,this.visionPartMarker=t,this.visionGroup.add(t),t}setTargetPose(t){if(Mi(this.targetGroup),!t||!xn([t.x,t.y,t.z]))return;const e=On([t.x,t.y,t.z]),i=new Re(new gc(.035,.043,40),new Fn({color:3723965,emissive:739906,side:Tn}));i.position.copy(e),i.quaternion.setFromUnitVectors(new O(0,0,1),new O(0,1,0)),this.targetGroup.add(i);const s=bh(t.roll,t.pitch,t.yaw),r=Wn(s).normalize();this.targetGroup.add(new ga(r,e,.16,3723965,.035,.02))}setBenchmarkSamples(t=[],e=null){Mi(this.benchmarkGroup);for(const i of(t||[]).slice(0,120)){if(!xn([i.x,i.y,i.z]))continue;const s=String(i.id)===String(e),r=i.ik&&i.ik.ok!==!0,a=s?4441222:r?15622754:Sb(i.source),o=On([i.x,i.y,i.z]),l=new Re(new Tr(s?.023:.016,16,10),new Fn({color:a,emissive:a,emissiveIntensity:s?.28:.12,transparent:!0,opacity:r?.68:.92,depthWrite:!1}));l.position.copy(o),this.benchmarkGroup.add(l);const c=Wn(bh(i.roll,i.pitch,i.yaw)).normalize();this.benchmarkGroup.add(new ga(c,o,s?.11:.07,a,s?.026:.018,s?.014:.009))}}setTcpFrame(t){if(Mi(this.tcpFrameGroup),!t||!xn(t.position)||!Mh(t.orientation))return;const e=On(t.position),i=[{axis:[1,0,0],color:16731469,length:.12},{axis:[0,1,0],color:4441222,length:.105},{axis:[0,0,1],color:5089535,length:.095}];for(const s of i){const r=Wn(Ta(s.axis,t.orientation)).normalize();this.tcpFrameGroup.add(new ga(r,e,s.length,s.color,.024,.012))}}setVisionTarget(t){const e=t==null?void 0:t.urdf_xyz_m;if(!xn(e)){this.visionPartPoint=null,this.visionPartMarker&&(this.visionPartMarker.visible=!1);return}this.visionPartPoint=On(e);const i=this.ensureVisionPartMarker();i?(i.position.copy(this.visionPartPoint),i.visible=!0):this.loadVisionPartModel()}setObjectMarkers(t=[]){Mi(this.objectMarkerGroup);for(const e of t.slice(0,16)){const i=e.center_camera_xyz_m;if(!xn(i)||!this.cameraExtrinsic)continue;const s=On(yh(i,this.cameraExtrinsic)),r=new Re(new Tr(.014,16,10),new Fn({color:6801389,emissive:1522763}));r.position.copy(s),this.objectMarkerGroup.add(r)}}setMujocoScene(t){var a;const e=t==null?void 0:t.table,i=t==null?void 0:t.object;if(this.mujocoObjectState=i||null,xn(e==null?void 0:e.center)&&xn(e==null?void 0:e.size)){const o=[Number(e.size[0]),Number(e.size[2]),Number(e.size[1])];this.mujocoTable?Sh(this.mujocoTable.userData.sceneSize,o)||(this.mujocoTable.geometry.dispose(),this.mujocoTable.geometry=new Bn(...o),this.mujocoTable.userData.sceneSize=o):(this.mujocoTable=new Re(new Bn(...o),new Fn({color:e.color||2436399,roughness:.9,metalness:.04})),this.mujocoTable.userData.sceneSize=o,this.mujocoSceneGroup.add(this.mujocoTable)),this.mujocoTable.receiveShadow=!0,this.mujocoTable.castShadow=!0,this.mujocoTable.position.copy(On(e.center))}if(this.ensureMujocoObject(),xn(i==null?void 0:i.position)){const o=On(i.position),l=!!(i!=null&&i.attached),c=this.mujocoObjectTargetPosition;let u=!0;if(!l&&c&&c.distanceTo(o)>vb){const p=this.mujocoObjectPendingJump;p&&p.position.distanceTo(o)<.02?p.samples+=1:this.mujocoObjectPendingJump={position:o.clone(),samples:1},this.mujocoObjectPendingJump.samples>=xb?(this.mujocoObjectTargetPosition=o,this.mujocoObjectPendingJump=null):u=!1}else this.mujocoObjectTargetPosition=o,this.mujocoObjectPendingJump=null;u&&!c&&this.mujocoObject&&this.mujocoObject.position.copy(o);const d=i.orientation_wxyz;if(u&&Mh(d)){const p=!!this.mujocoObjectTargetQuaternion,g=[d[1],d[2],d[3],d[0]],_=Wn(Ta([1,0,0],g)).normalize(),m=Wn(Ta([0,0,1],g)).normalize(),f=Wn(Ta([0,1,0],g)).multiplyScalar(-1).normalize();this.mujocoObjectTargetQuaternion=new Nn().setFromRotationMatrix(new Kt().makeBasis(_,m,f)),this.mujocoObject&&!p&&this.mujocoObject.quaternion.copy(this.mujocoObjectTargetQuaternion)}}const s=(t==null?void 0:t.extras)||[];this.mujocoExtras||(this.mujocoExtras={});for(const o of s){if(!o.name||!xn(o.position)||!o.size)continue;const l=o.type==="slot"&&o.model_url,c=o.type==="box",u=c?o.size.slice(0,3).map(Number):[Number(o.size[0]),Number(o.size[1])],h=()=>c?new Bn(u[0],u[2],u[1]):new Lr(u[0],u[0],u[1],24);if(!this.mujocoExtras[o.name]&&l){const d=new He;d.userData.sceneSize=u,this.mujocoSceneGroup.add(d),this.mujocoExtras[o.name]=d,new $l().load(o.model_url,p=>{var m;p.computeVertexNormals(),p.computeBoundingBox();const g=new O;(m=p.boundingBox)==null||m.getCenter(g),p.translate(-g.x,-g.y,-g.z);const _=new Re(p,new Fn({color:o.color||"#b8bdc7",roughness:.55,metalness:.18}));_.scale.setScalar(Number(o.model_scale)||.001),_.rotation.y=Math.PI,_.castShadow=!0,_.receiveShadow=!0,d.add(_)},void 0,p=>{var g,_;(_=(g=this.callbacks).onStatus)==null||_.call(g,`槽模型加载失败: ${(p==null?void 0:p.message)||p}`)})}else if(this.mujocoExtras[o.name])!l&&!Sh(this.mujocoExtras[o.name].userData.sceneSize,u)&&(this.mujocoExtras[o.name].geometry.dispose(),this.mujocoExtras[o.name].geometry=h(),this.mujocoExtras[o.name].userData.sceneSize=u);else{const d=new Re(h(),new Fn({color:o.color||"#cccccc",roughness:.6,metalness:.05}));d.castShadow=!0,d.receiveShadow=!0,d.userData.sceneSize=u,this.mujocoSceneGroup.add(d),this.mujocoExtras[o.name]=d}this.mujocoExtras[o.name].position.copy(On(o.position))}const r=new Set(s.map(o=>o.name));for(const[o,l]of Object.entries(this.mujocoExtras))r.has(o)||(this.mujocoSceneGroup.remove(l),(a=l.traverse)==null||a.call(l,c=>{var u,h,d,p;(h=(u=c.geometry)==null?void 0:u.dispose)==null||h.call(u),(p=(d=c.material)==null?void 0:d.dispose)==null||p.call(d)}),delete this.mujocoExtras[o])}ensureMujocoObject(){if(this.mujocoObject||!this.visionPartGeometry)return this.mujocoObject;const t=this.mujocoObjectState||{},e=new He,i=new Re(this.visionPartGeometry,new Fn({color:t.color||12986408,roughness:.46,metalness:.48}));return i.scale.setScalar(vh),i.rotation.x=-Math.PI/2,i.castShadow=!0,i.receiveShadow=!0,e.add(i),this.mujocoObject=e,this.mujocoSceneGroup.add(e),this.mujocoObjectTargetPosition&&e.position.copy(this.mujocoObjectTargetPosition),this.mujocoObjectTargetQuaternion&&e.quaternion.copy(this.mujocoObjectTargetQuaternion),e}updateRenderedMujocoObject(t){if(!this.mujocoObject)return;const e=1-Math.exp(-Math.max(0,t)/_b);this.mujocoObjectTargetPosition&&this.mujocoObject.position.lerp(this.mujocoObjectTargetPosition,e),this.mujocoObjectTargetQuaternion&&this.mujocoObject.quaternion.slerp(this.mujocoObjectTargetQuaternion,e)}setCameraExtrinsic(t){if(this.cameraExtrinsic=t?{...t}:null,Mi(this.cameraMarkerGroup),!t||!xn([t.x,t.y,t.z]))return;const e=On([t.x,t.y,t.z]),i=Qo(t,[1,0,0]),s=Qo(t,[0,1,0]),r=Qo(t,[0,0,1]),a=new Kt().makeBasis(i,s,r),o=new He;o.position.copy(e),o.quaternion.setFromRotationMatrix(a);const l=new Re(new Bn(.124,.029,.026),new Fn({color:10135725,roughness:.52,metalness:.16,transparent:!0,opacity:.45}));o.add(l),this.cameraMarkerGroup.add(o),this.cameraMarkerGroup.add(new ga(r,e,.18,6801389,.03,.018))}setPlatformObstacle(t){Mi(this.platformGroup);const e=(t==null?void 0:t.platform_obstacle)??t,i=Array.isArray(e)?e:Array.isArray(e==null?void 0:e.platform_obstacles)?e.platform_obstacles:e?[e]:[];for(const s of i)this.addCollisionBox(this.platformGroup,s,{fill:14901091,edge:15628154,opacity:.14})}setManualCollisionBoxes(t=[]){Mi(this.manualCollisionGroup);for(const e of t||[])this.addCollisionBox(this.manualCollisionGroup,e,{fill:e.enabled===!1?8095113:15249485,edge:e.enabled===!1?10135725:16762719,opacity:e.enabled===!1?.08:.18})}setWorkspaceBounds(t){if(Mi(this.workspaceBoundsGroup),!(t!=null&&t.enabled)||!xn(t.min)||!xn(t.max))return;const e=t.min.map(Number),i=t.max.map(Number),s=i.map((a,o)=>a-e[o]);if(s.some(a=>!Number.isFinite(a)||a<=0))return;const r=e.map((a,o)=>(a+i[o])/2);this.addCollisionBox(this.workspaceBoundsGroup,{center:r,dimensions:s,rpy:[0,0,0]},{fill:5089498,edge:6801389,opacity:.06})}addCollisionBox(t,e,i){if(!xn(e==null?void 0:e.center)||!xn(e==null?void 0:e.dimensions))return;const[s,r,a]=e.dimensions.map(Number),o=new Bn(s,a,r),l=new Re(o,new Fn({color:i.fill,transparent:!0,opacity:i.opacity,depthWrite:!1}));xh(l,e);const c=new _r(new Nu(o),new Ci({color:i.edge,transparent:!0,opacity:.9}));xh(c,e),t.add(l,c)}async loadScenePointCloud(){if(!this.cloudVisible||!this.cameraExtrinsic)return null;const t=await fetch("/api/scene_pointcloud.bin",{cache:"no-store"});if(!t.ok)throw new Error(`HTTP ${t.status}`);const e=await t.arrayBuffer();if(e.byteLength<8)throw new Error("点云数据为空");const i=new DataView(e),s=i.getUint32(0,!0),r=i.getUint32(4,!0),a=8+r*6*4;if(!r||e.byteLength<a)throw new Error("点云数据格式无效");const o=new Float32Array(e,8,r*6),l=new Float32Array(r*3),c=new Float32Array(r*3);for(let p=0;p<r;p+=1){const g=p*6,_=p*3,m=yh([o[g],o[g+1],o[g+2]],this.cameraExtrinsic),f=On(m);l[_]=f.x,l[_+1]=f.y,l[_+2]=f.z,c[_]=o[g+3],c[_+1]=o[g+4],c[_+2]=o[g+5]}const u=new $e;u.setAttribute("position",new dn(l,3)),u.setAttribute("color",new dn(c,3));const h=new md({size:.005,vertexColors:!0,transparent:!0,opacity:.82,depthWrite:!1}),d=new Gm(u,h);return d.frustumCulled=!1,this.cloudPoints&&(this.scene.remove(this.cloudPoints),Id(this.cloudPoints)),this.cloudPoints=d,this.scene.add(d),{sequence:s,count:r}}setCloudVisible(t){this.cloudVisible=!!t,this.cloudPoints&&(this.cloudPoints.visible=this.cloudVisible)}setDefaultView(){this.camera.position.set(2.8,1.65,3.7),this.controls.target.set(0,.85+Ia,0),this.controls.update()}frontView(){const t=this.controls.target.clone();this.camera.position.set(t.x,t.y+.15,t.z+3.9),this.camera.up.set(0,1,0),this.controls.update()}fitView(){if(!this.robot){this.setDefaultView();return}this.robot.updateMatrixWorld(!0);const t=new Li().setFromObject(this.robot),e=t.getCenter(new O),i=t.getSize(new O),r=Math.max(i.x,i.y,i.z,1)/(2*Math.tan(this.camera.fov*Math.PI/360));this.controls.target.copy(e),this.camera.position.set(e.x+r*.62,e.y+r*.18,e.z+r*1.08),this.camera.near=Math.max(.01,r/100),this.camera.far=Math.max(20,r*10),this.camera.updateProjectionMatrix(),this.controls.update()}resetView(){this.setDefaultView()}resize(){const t=this.canvas.parentElement;if(!t)return;const e=Math.max(1,t.clientWidth),i=Math.max(1,t.clientHeight);e===this.lastWidth&&i===this.lastHeight||(this.lastWidth=e,this.lastHeight=i,this.renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2)),this.renderer.setSize(e,i,!1),this.camera.aspect=e/i,this.camera.updateProjectionMatrix())}dispose(){cancelAnimationFrame(this.animationFrame),this.resizeObserver.disconnect(),this.controls.dispose(),this.renderer.dispose()}}function On(n){return new O(Number(n[0]),Number(n[2])+Ia,-Number(n[1]))}function Wn(n){return new O(Number(n[0]),Number(n[2]),-Number(n[1]))}function xh(n,t){n.position.copy(On(t.center)),n.quaternion.setFromRotationMatrix(yb(t.rpy||[0,0,0]))}function yb(n){const t=Mb(Number(n[0]||0),Number(n[1]||0),Number(n[2]||0)),e=[t[0][0],t[1][0],t[2][0]],i=[t[0][1],t[1][1],t[2][1]],s=[t[0][2],t[1][2],t[2][2]],r=Wn(e).normalize(),a=Wn(s).normalize(),o=Wn(i).multiplyScalar(-1).normalize();return new Kt().makeBasis(r,a,o)}function Mb(n,t,e){const i=Math.sin(n),s=Math.cos(n),r=Math.sin(t),a=Math.cos(t),o=Math.sin(e),l=Math.cos(e);return[[l*a,l*r*i-o*s,l*r*s+o*i],[o*a,o*r*i+l*s,o*r*s-l*i],[-r,a*i,a*s]]}function bh(n=0,t=0,e=0){const i=Math.cos(Number(t)),s=Math.sin(Number(t)),r=Math.cos(Number(e)),a=Math.sin(Number(e));return[r*i,a*i,-s]}function Sb(n){return n==="box_top"?5089498:n==="edge"?15249485:n==="random"?14148068:10135725}function Dd(n){const t=Math.cos(Number(n.roll)),e=Math.sin(Number(n.roll)),i=Math.cos(Number(n.pitch)),s=Math.sin(Number(n.pitch)),r=Math.cos(Number(n.yaw)),a=Math.sin(Number(n.yaw));return[[r*i,r*s*e-a*t,r*s*t+a*e],[a*i,a*s*e+r*t,a*s*t-r*e],[-s,i*e,i*t]]}function Qo(n,t){const e=Dd(n),i=[e[0][0]*t[0]+e[0][1]*t[1]+e[0][2]*t[2],e[1][0]*t[0]+e[1][1]*t[1]+e[1][2]*t[2],e[2][0]*t[0]+e[2][1]*t[1]+e[2][2]*t[2]];return Wn(i).normalize()}function yh(n,t){const e=Dd(t);return[Number(t.x)+e[0][0]*n[0]+e[0][1]*n[1]+e[0][2]*n[2],Number(t.y)+e[1][0]*n[0]+e[1][1]*n[1]+e[1][2]*n[2],Number(t.z)+e[2][0]*n[0]+e[2][1]*n[1]+e[2][2]*n[2]]}function xn(n){return Array.isArray(n)&&n.length===3&&n.every(t=>Number.isFinite(Number(t)))}function Mh(n){return Array.isArray(n)&&n.length===4&&n.every(t=>Number.isFinite(Number(t)))}function Sh(n,t){return Array.isArray(n)&&n.length===t.length&&n.every((e,i)=>Math.abs(Number(e)-Number(t[i]))<1e-9)}function Ta(n,t){const[e,i,s]=n.map(Number),[r,a,o,l]=t.map(Number),c=2*(a*s-o*i),u=2*(o*e-r*s),h=2*(r*i-a*e);return[e+l*c+(a*h-o*u),i+l*u+(o*c-r*h),s+l*h+(r*u-a*c)]}function Mi(n){for(;n.children.length;){const t=n.children.pop();Id(t)}}function Id(n){var t;(t=n==null?void 0:n.traverse)==null||t.call(n,e=>{var i,s,r,a;(s=(i=e.geometry)==null?void 0:i.dispose)==null||s.call(i),Array.isArray(e.material)?e.material.forEach(o=>{var l;return(l=o==null?void 0:o.dispose)==null?void 0:l.call(o)}):(a=(r=e.material)==null?void 0:r.dispose)==null||a.call(r)})}/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Ud=(n,t,e=[])=>{const i=document.createElementNS("http://www.w3.org/2000/svg",n);return Object.keys(t).forEach(s=>{i.setAttribute(s,String(t[s]))}),e.length&&e.forEach(s=>{const r=Ud(...s);i.appendChild(r)}),i};var Eb=([n,t,e])=>Ud(n,t,e);/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Tb=n=>Array.from(n.attributes).reduce((t,e)=>(t[e.name]=e.value,t),{}),wb=n=>typeof n=="string"?n:!n||!n.class?"":n.class&&typeof n.class=="string"?n.class.split(" "):n.class&&Array.isArray(n.class)?n.class:"",Ab=n=>n.flatMap(wb).map(e=>e.trim()).filter(Boolean).filter((e,i,s)=>s.indexOf(e)===i).join(" "),Cb=n=>n.replace(/(\w)(\w*)(_|-|\s*)/g,(t,e,i)=>e.toUpperCase()+i.toLowerCase()),Eh=(n,{nameAttr:t,icons:e,attrs:i})=>{var g;const s=n.getAttribute(t);if(s==null)return;const r=Cb(s),a=e[r];if(!a)return console.warn(`${n.outerHTML} icon name was not found in the provided icons object.`);const o=Tb(n),[l,c,u]=a,h={...c,"data-lucide":s,...i,...o},d=Ab(["lucide",`lucide-${s}`,o,i]);d&&Object.assign(h,{class:d});const p=Eb([l,h,u]);return(g=n.parentNode)==null?void 0:g.replaceChild(p,n)};/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ee={xmlns:"http://www.w3.org/2000/svg",width:24,height:24,viewBox:"0 0 24 24",fill:"none",stroke:"currentColor","stroke-width":2,"stroke-linecap":"round","stroke-linejoin":"round"};/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Rb=["svg",ee,[["path",{d:"M12 17V3"}],["path",{d:"m6 11 6 6 6-6"}],["path",{d:"M19 21H5"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Pb=["svg",ee,[["path",{d:"m18 9-6-6-6 6"}],["path",{d:"M12 3v14"}],["path",{d:"M5 21h14"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Lb=["svg",ee,[["path",{d:"m5 12 7-7 7 7"}],["path",{d:"M12 19V5"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Nb=["svg",ee,[["path",{d:"M12 8V4H8"}],["rect",{width:"16",height:"12",x:"4",y:"8",rx:"2"}],["path",{d:"M2 14h2"}],["path",{d:"M20 14h2"}],["path",{d:"M15 13v2"}],["path",{d:"M9 13v2"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Db=["svg",ee,[["path",{d:"M21 8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16Z"}],["path",{d:"m3.3 7 8.7 5 8.7-5"}],["path",{d:"M12 22V12"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Ib=["svg",ee,[["path",{d:"M12 5a3 3 0 1 0-5.997.125 4 4 0 0 0-2.526 5.77 4 4 0 0 0 .556 6.588A4 4 0 1 0 12 18Z"}],["path",{d:"M9 13a4.5 4.5 0 0 0 3-4"}],["path",{d:"M6.003 5.125A3 3 0 0 0 6.401 6.5"}],["path",{d:"M3.477 10.896a4 4 0 0 1 .585-.396"}],["path",{d:"M6 18a4 4 0 0 1-1.967-.516"}],["path",{d:"M12 13h4"}],["path",{d:"M12 18h6a2 2 0 0 1 2 2v1"}],["path",{d:"M12 8h8"}],["path",{d:"M16 8V5a2 2 0 0 1 2-2"}],["circle",{cx:"16",cy:"13",r:".5"}],["circle",{cx:"18",cy:"3",r:".5"}],["circle",{cx:"20",cy:"21",r:".5"}],["circle",{cx:"20",cy:"8",r:".5"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Ub=["svg",ee,[["path",{d:"M14.5 4h-5L7 7H4a2 2 0 0 0-2 2v9a2 2 0 0 0 2 2h16a2 2 0 0 0 2-2V9a2 2 0 0 0-2-2h-3l-2.5-3z"}],["circle",{cx:"12",cy:"13",r:"3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const kb=["svg",ee,[["circle",{cx:"12",cy:"12",r:"10"}],["path",{d:"m9 12 2 2 4-4"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Fb=["svg",ee,[["circle",{cx:"12",cy:"12",r:"10"}],["path",{d:"m15 9-6 6"}],["path",{d:"m9 9 6 6"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Ob=["svg",ee,[["path",{d:"M12 13v8l-4-4"}],["path",{d:"m12 21 4-4"}],["path",{d:"M4.393 15.269A7 7 0 1 1 15.71 8h1.79a4.5 4.5 0 0 1 2.436 8.284"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Bb=["svg",ee,[["path",{d:"M17.5 19H9a7 7 0 1 1 6.71-9h1.79a4.5 4.5 0 1 1 0 9Z"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const zb=["svg",ee,[["polyline",{points:"9 10 4 15 9 20"}],["path",{d:"M20 4v7a4 4 0 0 1-4 4H4"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Vb=["svg",ee,[["circle",{cx:"12",cy:"12",r:"10"}],["line",{x1:"22",x2:"18",y1:"12",y2:"12"}],["line",{x1:"6",x2:"2",y1:"12",y2:"12"}],["line",{x1:"12",x2:"12",y1:"6",y2:"2"}],["line",{x1:"12",x2:"12",y1:"22",y2:"18"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Hb=["svg",ee,[["circle",{cx:"12",cy:"12",r:"3"}],["path",{d:"M3 7V5a2 2 0 0 1 2-2h2"}],["path",{d:"M17 3h2a2 2 0 0 1 2 2v2"}],["path",{d:"M21 17v2a2 2 0 0 1-2 2h-2"}],["path",{d:"M7 21H5a2 2 0 0 1-2-2v-2"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Gb=["svg",ee,[["path",{d:"M2 12h6"}],["path",{d:"M22 12h-6"}],["path",{d:"M12 2v2"}],["path",{d:"M12 8v2"}],["path",{d:"M12 14v2"}],["path",{d:"M12 20v2"}],["path",{d:"m19 9-3 3 3 3"}],["path",{d:"m5 15 3-3-3-3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Wb=["svg",ee,[["path",{d:"m12 14 4-4"}],["path",{d:"M3.34 19a10 10 0 1 1 17.32 0"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const $b=["svg",ee,[["path",{d:"M18 11.5V9a2 2 0 0 0-2-2a2 2 0 0 0-2 2v1.4"}],["path",{d:"M14 10V8a2 2 0 0 0-2-2a2 2 0 0 0-2 2v2"}],["path",{d:"M10 9.9V9a2 2 0 0 0-2-2a2 2 0 0 0-2 2v5"}],["path",{d:"M6 14a2 2 0 0 0-2-2a2 2 0 0 0-2 2"}],["path",{d:"M18 11a2 2 0 1 1 4 0v3a8 8 0 0 1-8 8h-4a8 8 0 0 1-8-8 2 2 0 1 1 4 0"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const jb=["svg",ee,[["path",{d:"M18 11V6a2 2 0 0 0-2-2a2 2 0 0 0-2 2"}],["path",{d:"M14 10V4a2 2 0 0 0-2-2a2 2 0 0 0-2 2v2"}],["path",{d:"M10 10.5V6a2 2 0 0 0-2-2a2 2 0 0 0-2 2v8"}],["path",{d:"M18 8a2 2 0 1 1 4 0v6a8 8 0 0 1-8 8h-2c-2.8 0-4.5-.86-5.99-2.34l-3.6-3.6a2 2 0 0 1 2.83-2.82L7 15"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Xb=["svg",ee,[["path",{d:"M15 21v-8a1 1 0 0 0-1-1h-4a1 1 0 0 0-1 1v8"}],["path",{d:"M3 10a2 2 0 0 1 .709-1.528l7-5.999a2 2 0 0 1 2.582 0l7 5.999A2 2 0 0 1 21 10v9a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2z"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const qb=["svg",ee,[["path",{d:"m3 17 2 2 4-4"}],["path",{d:"m3 7 2 2 4-4"}],["path",{d:"M13 6h8"}],["path",{d:"M13 12h8"}],["path",{d:"M13 18h8"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Yb=["svg",ee,[["path",{d:"M21 12h-8"}],["path",{d:"M21 6H8"}],["path",{d:"M21 18h-8"}],["path",{d:"M3 6v4c0 1.1.9 2 2 2h3"}],["path",{d:"M3 10v6c0 1.1.9 2 2 2h3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Kb=["svg",ee,[["path",{d:"M19.914 11.105A7.298 7.298 0 0 0 20 10a8 8 0 0 0-16 0c0 4.993 5.539 10.193 7.399 11.799a1 1 0 0 0 1.202 0 32 32 0 0 0 .824-.738"}],["circle",{cx:"12",cy:"10",r:"3"}],["path",{d:"M16 18h6"}],["path",{d:"M19 15v6"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Jb=["svg",ee,[["path",{d:"M5 3v16h16"}],["path",{d:"m5 19 6-6"}],["path",{d:"m2 6 3-3 3 3"}],["path",{d:"m18 16 3 3-3 3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Zb=["svg",ee,[["path",{d:"M12 16h.01"}],["path",{d:"M12 8v4"}],["path",{d:"M15.312 2a2 2 0 0 1 1.414.586l4.688 4.688A2 2 0 0 1 22 8.688v6.624a2 2 0 0 1-.586 1.414l-4.688 4.688a2 2 0 0 1-1.414.586H8.688a2 2 0 0 1-1.414-.586l-4.688-4.688A2 2 0 0 1 2 15.312V8.688a2 2 0 0 1 .586-1.414l4.688-4.688A2 2 0 0 1 8.688 2z"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const Qb=["svg",ee,[["path",{d:"M12 22v-9"}],["path",{d:"M15.17 2.21a1.67 1.67 0 0 1 1.63 0L21 4.57a1.93 1.93 0 0 1 0 3.36L8.82 14.79a1.655 1.655 0 0 1-1.64 0L3 12.43a1.93 1.93 0 0 1 0-3.36z"}],["path",{d:"M20 13v3.87a2.06 2.06 0 0 1-1.11 1.83l-6 3.08a1.93 1.93 0 0 1-1.78 0l-6-3.08A2.06 2.06 0 0 1 4 16.87V13"}],["path",{d:"M21 12.43a1.93 1.93 0 0 0 0-3.36L8.83 2.2a1.64 1.64 0 0 0-1.63 0L3 4.57a1.93 1.93 0 0 0 0 3.36l12.18 6.86a1.636 1.636 0 0 0 1.63 0z"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ty=["svg",ee,[["rect",{width:"18",height:"18",x:"3",y:"3",rx:"2"}],["path",{d:"M15 3v18"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ey=["svg",ee,[["circle",{cx:"12",cy:"5",r:"1"}],["path",{d:"m9 20 3-6 3 6"}],["path",{d:"m6 8 6 2 6-2"}],["path",{d:"M12 10v4"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ny=["svg",ee,[["polygon",{points:"6 3 20 12 6 21 6 3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const iy=["svg",ee,[["path",{d:"M5 12h14"}],["path",{d:"M12 5v14"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const sy=["svg",ee,[["path",{d:"M18.36 6.64A9 9 0 0 1 20.77 15"}],["path",{d:"M6.16 6.16a9 9 0 1 0 12.68 12.68"}],["path",{d:"M12 2v4"}],["path",{d:"m2 2 20 20"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ry=["svg",ee,[["path",{d:"M12 2v10"}],["path",{d:"M18.4 6.6a9 9 0 1 1-12.77.04"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ay=["svg",ee,[["path",{d:"M3 12a9 9 0 0 1 9-9 9.75 9.75 0 0 1 6.74 2.74L21 8"}],["path",{d:"M21 3v5h-5"}],["path",{d:"M21 12a9 9 0 0 1-9 9 9.75 9.75 0 0 1-6.74-2.74L3 16"}],["path",{d:"M8 16H3v5"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const oy=["svg",ee,[["path",{d:"M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8"}],["path",{d:"M3 3v5h5"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const ly=["svg",ee,[["circle",{cx:"6",cy:"19",r:"3"}],["path",{d:"M9 19h8.5a3.5 3.5 0 0 0 0-7h-11a3.5 3.5 0 0 1 0-7H15"}],["circle",{cx:"18",cy:"5",r:"3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const cy=["svg",ee,[["path",{d:"M15.2 3a2 2 0 0 1 1.4.6l3.8 3.8a2 2 0 0 1 .6 1.4V19a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2z"}],["path",{d:"M17 21v-7a1 1 0 0 0-1-1H8a1 1 0 0 0-1 1v7"}],["path",{d:"M7 3v4a1 1 0 0 0 1 1h7"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const uy=["svg",ee,[["path",{d:"M3 7V5a2 2 0 0 1 2-2h2"}],["path",{d:"M17 3h2a2 2 0 0 1 2 2v2"}],["path",{d:"M21 17v2a2 2 0 0 1-2 2h-2"}],["path",{d:"M7 21H5a2 2 0 0 1-2-2v-2"}],["circle",{cx:"12",cy:"12",r:"1"}],["path",{d:"M18.944 12.33a1 1 0 0 0 0-.66 7.5 7.5 0 0 0-13.888 0 1 1 0 0 0 0 .66 7.5 7.5 0 0 0 13.888 0"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const hy=["svg",ee,[["path",{d:"M3 7V5a2 2 0 0 1 2-2h2"}],["path",{d:"M17 3h2a2 2 0 0 1 2 2v2"}],["path",{d:"M21 17v2a2 2 0 0 1-2 2h-2"}],["path",{d:"M7 21H5a2 2 0 0 1-2-2v-2"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const dy=["svg",ee,[["path",{d:"M15 12h-5"}],["path",{d:"M15 8h-5"}],["path",{d:"M19 17V5a2 2 0 0 0-2-2H4"}],["path",{d:"M8 21h12a2 2 0 0 0 2-2v-1a1 1 0 0 0-1-1H11a1 1 0 0 0-1 1v1a2 2 0 1 1-4 0V5a2 2 0 1 0-4 0v2a1 1 0 0 0 1 1h3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const fy=["svg",ee,[["circle",{cx:"11",cy:"11",r:"8"}],["path",{d:"m21 21-4.3-4.3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const py=["svg",ee,[["path",{d:"M20 13c0 5-3.5 7.5-7.66 8.95a1 1 0 0 1-.67-.01C7.5 20.5 4 18 4 13V6a1 1 0 0 1 1-1c2 0 4.5-1.2 6.24-2.72a1.17 1.17 0 0 1 1.52 0C14.51 3.81 17 5 19 5a1 1 0 0 1 1 1z"}],["path",{d:"M12 8v4"}],["path",{d:"M12 16h.01"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const my=["svg",ee,[["path",{d:"M20 13c0 5-3.5 7.5-7.66 8.95a1 1 0 0 1-.67-.01C7.5 20.5 4 18 4 13V6a1 1 0 0 1 1-1c2 0 4.5-1.2 6.24-2.72a1.17 1.17 0 0 1 1.52 0C14.51 3.81 17 5 19 5a1 1 0 0 1 1 1z"}],["path",{d:"m9 12 2 2 4-4"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const gy=["svg",ee,[["line",{x1:"21",x2:"14",y1:"4",y2:"4"}],["line",{x1:"10",x2:"3",y1:"4",y2:"4"}],["line",{x1:"21",x2:"12",y1:"12",y2:"12"}],["line",{x1:"8",x2:"3",y1:"12",y2:"12"}],["line",{x1:"21",x2:"16",y1:"20",y2:"20"}],["line",{x1:"12",x2:"3",y1:"20",y2:"20"}],["line",{x1:"14",x2:"14",y1:"2",y2:"6"}],["line",{x1:"8",x2:"8",y1:"10",y2:"14"}],["line",{x1:"16",x2:"16",y1:"18",y2:"22"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const _y=["svg",ee,[["path",{d:"M3 6h18"}],["path",{d:"M19 6v14c0 1-1 2-2 2H7c-1 0-2-1-2-2V6"}],["path",{d:"M8 6V4c0-1 1-2 2-2h4c1 0 2 1 2 2v2"}],["line",{x1:"10",x2:"10",y1:"11",y2:"17"}],["line",{x1:"14",x2:"14",y1:"11",y2:"17"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const vy=["svg",ee,[["path",{d:"m21.73 18-8-14a2 2 0 0 0-3.48 0l-8 14A2 2 0 0 0 4 21h16a2 2 0 0 0 1.73-3"}],["path",{d:"M12 9v4"}],["path",{d:"M12 17h.01"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const xy=["svg",ee,[["path",{d:"M16 12h6"}],["path",{d:"M8 12H2"}],["path",{d:"M12 2v2"}],["path",{d:"M12 8v2"}],["path",{d:"M12 14v2"}],["path",{d:"M12 20v2"}],["path",{d:"m19 15 3-3-3-3"}],["path",{d:"m5 9-3 3 3 3"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const by=["svg",ee,[["path",{d:"M21 17v2a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-2"}],["path",{d:"M21 7V5a2 2 0 0 0-2-2H5a2 2 0 0 0-2 2v2"}],["circle",{cx:"12",cy:"12",r:"1"}],["path",{d:"M18.944 12.33a1 1 0 0 0 0-.66 7.5 7.5 0 0 0-13.888 0 1 1 0 0 0 0 .66 7.5 7.5 0 0 0 13.888 0"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const yy=["svg",ee,[["path",{d:"m21.64 3.64-1.28-1.28a1.21 1.21 0 0 0-1.72 0L2.36 18.64a1.21 1.21 0 0 0 0 1.72l1.28 1.28a1.2 1.2 0 0 0 1.72 0L21.64 5.36a1.2 1.2 0 0 0 0-1.72"}],["path",{d:"m14 7 3 3"}],["path",{d:"M5 6v4"}],["path",{d:"M19 14v4"}],["path",{d:"M10 2v2"}],["path",{d:"M7 8H3"}],["path",{d:"M21 16h-4"}],["path",{d:"M11 3H9"}]]];/**
 * @license lucide v0.468.0 - ISC
 *
 * This source code is licensed under the ISC license.
 * See the LICENSE file in the root directory of this source tree.
 */const My=({icons:n={},nameAttr:t="data-lucide",attrs:e={}}={})=>{if(!Object.values(n).length)throw new Error(`Please provide an icons object.
If you want to use all the icons you can import it like:
 \`import { createIcons, icons } from 'lucide';
lucide.createIcons({icons});\``);if(typeof document>"u")throw new Error("`createIcons()` only works in a browser environment.");const i=document.querySelectorAll(`[${t}]`);if(Array.from(i).forEach(s=>Eh(s,{nameAttr:t,icons:n,attrs:e})),t==="data-lucide"){const s=document.querySelectorAll("[icon-name]");s.length>0&&(console.warn("[Lucide] Some icons were found with the now deprecated icon-name attribute. These will still be replaced for backwards compatibility, but will no longer be supported in v1.0 and you should switch to data-lucide"),Array.from(s).forEach(r=>Eh(r,{nameAttr:"icon-name",icons:n,attrs:e})))}},Sy={ArrowDownToLine:Rb,ArrowUp:Lb,ArrowUpFromLine:Pb,Bot:Nb,Box:Db,BrainCircuit:Ib,Camera:Ub,CircleCheck:kb,CircleX:Fb,Cloud:Bb,CloudDownload:Ob,CornerDownLeft:zb,Crosshair:Vb,Focus:Hb,FoldHorizontal:Gb,Gauge:Wb,Grab:$b,Hand:jb,House:Xb,ListChecks:qb,ListTree:Yb,MapPinPlus:Kb,Move3d:Jb,OctagonAlert:Zb,PackageOpen:Qb,PanelRight:ty,PersonStanding:ey,Play:ny,Plus:iy,Power:ry,PowerOff:sy,RefreshCw:ay,RotateCcw:oy,Route:ly,Save:cy,Scan:hy,Search:fy,ScanEye:uy,ScrollText:dy,ShieldAlert:py,ShieldCheck:my,SlidersHorizontal:gy,Trash2:_y,TriangleAlert:vy,UnfoldHorizontal:xy,View:by,WandSparkles:yy};function E(n){return document.getElementById(n)}function os(){My({icons:Sy,attrs:{"stroke-width":1.8}})}function kt(n,t=3){const e=Number(n);return Number.isFinite(e)?e.toFixed(t):"--"}function Jt(n){return String(n??"").replaceAll("&","&amp;").replaceAll("<","&lt;").replaceAll(">","&gt;").replaceAll('"',"&quot;").replaceAll("'","&#039;")}function Xn(n,t,e,i="neutral"){const s=E(n);if(!s)return;const r=s.querySelector("b"),a=s.querySelector("span:last-child");r&&(r.textContent=t),a&&(a.textContent=e),s.dataset.state=i}function tn(n,t,e="neutral"){const i=E(n);i&&(i.textContent=t,i.dataset.state=e)}function ie(n,t="good",e=3600){const i=E("toastStack"),s=document.createElement("div"),r=t==="bad"?"circle-x":t==="warn"?"triangle-alert":"circle-check";s.className="toast",s.dataset.kind=t,s.innerHTML=`<i data-lucide="${r}"></i><p>${Jt(n)}</p>`,i.appendChild(s),os(),window.setTimeout(()=>s.remove(),e)}function Ey(n){var s;const t=(s=n==null?void 0:n.payload)==null?void 0:s.failure;if(!t)return(n==null?void 0:n.message)||"未知错误";const e=[];t.stage_label&&e.push(t.stage_label),t.title&&e.push(t.title);const i=t.message||(n==null?void 0:n.message);return i&&!e.includes(i)&&e.push(i),t.moveit_error_code!==void 0&&e.push(`MoveIt=${t.moveit_error_code}${t.moveit_error_name?` ${t.moveit_error_name}`:""}`),t.hint&&e.push(t.hint),e.filter(Boolean).join(" | ")}function ls(n,t="确认操作",e="确认"){const i=E("confirmDialog");return E("confirmTitle").textContent=t,E("confirmMessage").textContent=n,E("confirmAccept").textContent=e,i.hidden=!1,new Promise(s=>{const r=u=>{i.hidden=!0,E("confirmCancel").removeEventListener("click",a),E("confirmAccept").removeEventListener("click",o),i.removeEventListener("click",l),window.removeEventListener("keydown",c),s(u)},a=()=>r(!1),o=()=>r(!0),l=u=>{u.target===i&&a()},c=u=>{u.key==="Escape"&&a(),u.key==="Enter"&&o()};E("confirmCancel").addEventListener("click",a),E("confirmAccept").addEventListener("click",o),i.addEventListener("click",l),window.addEventListener("keydown",c),E("confirmCancel").focus()})}async function de(n,t,e={}){const{confirm:i,confirmTitle:s,success:r=`${n}完成`,quiet:a=!1,onError:o=null}=e;if(document.body.dataset.busy)return ie("已有命令正在执行，请等待完成","warn",2200),null;if(i&&!await ls(i,s))return null;E("footerMessage").textContent=`${n}...`,document.body.dataset.busy="true",document.querySelectorAll("button").forEach(l=>{l.disabled||(l.dataset.busyDisabled="true",l.disabled=!0)});try{const l=await t();return E("footerMessage").textContent=r,a||ie(r),l}catch(l){const c=`${n}失败：${Ey(l)}`;return E("footerMessage").textContent=c,ie(c,"bad",9e3),o==null||o(l,c),null}finally{document.querySelectorAll('button[data-busy-disabled="true"]').forEach(l=>{l.disabled=!1,delete l.dataset.busyDisabled}),delete document.body.dataset.busy}}const Ga=[["x","X",-1,1,.001,.327],["y","Y",-1,1,.001,.018],["z","Z",.1,1.8,.001,.514]],Wa=[["x","X",-2,2,.001,0],["y","Y",-2,2,.001,0],["z","Z",0,3,.001,1.4],["roll","Roll",-3.142,3.142,.001,0],["pitch","Pitch",-3.142,3.142,.001,-1.571],["yaw","Yaw",-3.142,3.142,.001,3.142],["image_rotation_deg","Image Rot",0,270,90,180]],jl=[["1","q"],["2","w"],["3","e"],["4","r"],["5","t"],["6","y"],["7","u"]],Ty=.8*Math.PI/180,wy=180,ns=0,wr=-.91,Ay=350,Cy=900,Ry={right:{Right_Gripper_Joint:1,Right_Gripper_Left_Support_Joint:-1,Right_Gripper_Left_2_Joint:1,Right_Gripper_Right_2_Joint:-1,Right_Gripper_Right_1_Joint:-1,Right_Gripper_Right_Support_Joint:-1},left:{Left_Gripper_Joint:1,Left_Gripper_Left_Support_Joint:-1,Left_Gripper_Left_2_Joint:1,Left_Gripper_Right_2_Joint:-1,Left_Gripper_Right_1_Joint:-1,Left_Gripper_Right_Support_Joint:-1}},Py={approach:"接近",grasp:"抓取",lift:"抬起",move:"移动",release:"释放",close:"闭合",pick:"自动夹取",wait:"等待"},Ly="side",Ny={vertical:{roll:0,pitch:-Math.PI/2,yaw:Math.PI,grasp_orientation_mode:"vertical_grasp"},side:{roll:Math.PI/2,pitch:0,yaw:0,grasp_orientation_mode:"side_grasp"},z_parallel:{roll:0,pitch:Math.PI/6,yaw:0,grasp_orientation_mode:"angled_grasp"}},Th={vertical_grasp:{title:"垂直抓取",summary:"IK 先验证垂直末端姿态，再按候选姿态逐个规划。",scanLabel:"垂直候选",candidateText:n=>gr(n)},side_grasp:{title:"侧面抓取",summary:"IK 扫描侧向接近姿态，找到可规划候选后锁定。",scanLabel:"侧向候选",candidateText:n=>gr(n)},angled_grasp:{title:"斜向抓取",summary:"IK 扫描斜向抓取姿态，兼顾目标方向和碰撞约束。",scanLabel:"斜向候选",candidateText:n=>gr(n)},z_parallel_grasp:{title:"斜向抓取",summary:"IK 扫描斜向抓取姿态，兼顾目标方向和碰撞约束。",scanLabel:"斜向候选",candidateText:n=>gr(n)},flexible_grasp:{title:"灵活抓取",summary:"IK 扫描圆柱接触方向，先确认预抓取，再确认抓取位姿。",scanLabel:"接触角",candidateText:n=>Number.isFinite(Number(n==null?void 0:n.theta_deg))?`theta ${kt(n.theta_deg,1)}°`:gr(n)}},Dy={vertical_grasp:"垂直抓取",angled_grasp:"斜向抓取",side_grasp:"侧面抓取"},kd="unoarm.omplConfig",Ln={planner_id:"RRTConnectkConfigDefault",planning_time:1,attempts:1,ik_timeout:.6},Iy={box_top:"箱上",edge:"边缘",random:"随机",custom:"自定义"},Ar=5;function Fd(n={}){const t=Number(n.plate_x??-.08),e=Number(n.plate_y??-.72),i=Number(n.calibration_block_x??.13),s=Number(n.calibration_block_y??-.62),r=(a,o,l,c,u)=>u.map(([h,d],p)=>({id:`${a}-${p+1}`,surface:a,surfaceLabel:o,x:h,y:d,spawnZ:l,validation:c.includes(p),sample:null}));return[...r("table","桌面",.875,[1,6],[[-.16,-.84],[0,-.84],[.16,-.84],[.16,-.78],[.16,-.72],[.16,-.69],[-.16,-.6],[.02,-.6],[.05,-.68]]),...r("plate","盘内",.89,[4],[[t,e],[t-.025,e],[t+.025,e],[t,e-.025],[t,e+.025]]),...r("block","标定块",.925,[1,4],[[i,s],[i-.02,s],[i+.02,s],[i,s-.015],[i,s+.015]])]}const b={arm:localStorage.getItem("unoarm.activeArm")==="left"?"left":"right",activePanel:"motion",linksByArm:{right:[],left:[]},jointState:null,jointConfig:null,jointLimitsByArm:{right:{},left:{}},kinematics:null,ompl:{options:null,config:tS(),selectedPresetId:null},benchmark:{options:null,defaultsApplied:!1,selectedPlanners:[],samples:[],selectedSampleId:null,result:null,progress:null},visionTarget:null,cameraExtrinsic:null,cameraCalibration:{points:Fd(),analysis:null,fit:null,sceneReady:!1,running:!1},points:[],presets:[],posesLib:[],sequence:RS(),cloudVisible:localStorage.getItem("unoarm.cloudVisible")!=="false",reachabilityVisible:localStorage.getItem("unoarm.reachabilityVisible")!=="false",graspPoseMode:Mc(localStorage.getItem("unoarm.graspPoseMode")),platformObstacleVisible:!1,platformObstacleApplied:!1,manualCollision:{boxes:[],applied:!1},workspaceBoundsByArm:{right:{enabled:!1,min:[-.8,-.65,.2],max:[.35,.35,1.55]},left:{enabled:!1,min:[-.35,-.65,.2],max:[.8,.35,1.55]}},lastPlanByArm:{right:null,left:null},lastDecisionError:null,robotByArm:{right:{motors_enabled:null,estop_active:null},left:{motors_enabled:null,estop_active:null}},gripperByArm:{right:{state:"open",arm:"right",joint:"Right_Gripper_Joint",position:ns,simulated:!1},left:{state:"open",arm:"left",joint:"Left_Gripper_Joint",position:ns,simulated:!1}},gripperCommand:{right:{target:null,at:0,inFlight:!1},left:{target:null,at:0,inFlight:!1}},lastStatusAt:0};let at;const Vs=new Set;let tl=!1,yr=null,wh=0,Ua=null,Xl={};function Uy(){zy(),Vy(),Ah("velocityScaling","velocityValue",n=>Number(n).toFixed(2)),Ah("accelerationScaling","accelerationValue",n=>Number(n).toFixed(2)),Hy(),$y(),jy(),Bd(),Hd(),Ql({anyConnected:!1,anyEnabled:!1,allEnabled:!1,anyEstop:!1}),uf(),Vd(),Qi(),qn(),Hs(),oo(),Oi(),ds(),fi(),Fi(),er(),os(),ky(),By(),Fy(),Oy()}function ky(){at=new bb(E("robotScene"),{onLoadProgress:(n,t)=>{E("sceneLoadState").querySelector("span:last-child").textContent=`正在加载 UnoArm URDF ${n}/${t}`},onLoaded:n=>{E("sceneLoadState").classList.add("ready"),E("sceneLoadState").querySelector("span:last-child").textContent="真实 URDF 已加载",E("sceneModelState").textContent=`${Object.keys(n.joints||{}).length} 个关节`,at.applyJointState(b.jointState),jd(),window.setTimeout(()=>{E("sceneLoadState").hidden=!0},1e3)},onLoadError:n=>{E("sceneLoadState").querySelector("span:last-child").textContent="URDF 加载失败",E("sceneModelState").textContent="模型不可用",ie(`URDF 加载失败：${n.message||n}`,"bad",6e3)},onFps:n=>{E("sceneFps").textContent=`${n} FPS`}}),at.setCloudVisible(b.cloudVisible),at.setTargetPose(fn())}async function Fy(){await Promise.allSettled([Mn(),ki(),wc(),Ii(),JM(),nf(),qd(),Yd(),Kd(),Cc(),zc(),Vc(),af(!1),WM(!1),VM(!1),kc(),ar(),Ir(!1)]),Nc()}function Oy(){Ls(Mn,50),Ls(wc,400),Ls(ki,600),Ls(qd,2e3),Ls(Yd,1e3),Ls(Kd,1e3),window.setInterval(()=>{b.activePanel==="logs"&&E("logAutoRefresh").checked&&qa()},3e3)}function Ls(n,t){let e=!1;window.setInterval(async()=>{if(!e){e=!0;try{await n()}finally{e=!1}}},t)}function By(){const n=()=>{E("footerClock").textContent=new Date().toLocaleTimeString("zh-CN",{hour12:!1})};n(),window.setInterval(n,1e3)}function zy(){const n=E("poseFields");n.innerHTML=Ga.map(([t,e,i,s,r,a])=>`
    <label class="pose-field pose-slider-field">
      <span>${e} / m</span>
      <output id="pose_${t}_value">${kt(a,3)}</output>
      <input id="pose_${t}" type="range" min="${i}" max="${s}" step="${r}" value="${a}">
    </label>
  `).join("");for(const[t]of Ga)E(`pose_${t}`).addEventListener("input",()=>{Od(t),at==null||at.setTargetPose(fn())})}function Od(n){const t=E(`pose_${n}_value`);t&&(t.textContent=kt(Number(E(`pose_${n}`).value),3))}function Vy(){const n=E("cameraFields");n.innerHTML=Wa.map(([t,e,i,s,r,a])=>`
    <label class="pose-field">
      <span>${e}</span>
      <input id="camera_${t}" type="number" min="${i}" max="${s}" step="${r}" value="${a}">
    </label>
  `).join("");for(const[t]of Wa)E(`camera_${t}`).addEventListener("input",()=>{b.cameraExtrinsic=Qs(),at==null||at.setCameraExtrinsic(b.cameraExtrinsic),E("extrinsicStatus").textContent="外参已修改，尚未保存"})}function Ah(n,t,e){const i=E(n),s=E(t);if(!i||!s)return;const r=()=>{s.textContent=e(i.value)};i.addEventListener("input",r),r()}function Hy(){document.querySelectorAll("[data-panel]").forEach(n=>{n.addEventListener("click",()=>Gy(n.dataset.panel))}),E("toggleInspector").addEventListener("click",()=>{E("inspectorPanel").classList.toggle("open")}),document.querySelectorAll("[data-arm]").forEach(n=>{n.addEventListener("click",()=>no(n.dataset.arm))}),document.querySelectorAll("[data-grasp-mode]").forEach(n=>{n.addEventListener("click",()=>Sc(n.dataset.graspMode))}),E("fitViewButton").addEventListener("click",()=>at.fitView()),E("frontViewButton").addEventListener("click",()=>at.frontView()),E("resetViewButton").addEventListener("click",()=>at.resetView()),document.querySelector(".scene-panel").addEventListener("click",n=>{n.target.closest("button")||(window.innerWidth<=760&&E("controlPanel").classList.remove("open"),window.innerWidth<=1040&&E("inspectorPanel").classList.remove("open"))})}function Gy(n){b.activePanel=n,document.querySelectorAll("[data-panel]").forEach(t=>{t.classList.toggle("active",t.dataset.panel===n)}),document.querySelectorAll("[data-panel-content]").forEach(t=>{t.classList.toggle("active",t.dataset.panelContent===n)}),window.innerWidth<=760&&E("controlPanel").classList.add("open"),n==="logs"&&qa(),n==="ompl"&&ar(),n==="benchmark"&&Ir(),n==="vlm"&&sf(),n==="poses"&&rf(),n==="camera-calibration"&&(Pc(),Zl()),n==="calibration"&&(Ic(),kc())}async function no(n){Tc(),b.arm=n==="left"?"left":"right",localStorage.setItem("unoarm.activeArm",b.arm),Bd(),await Promise.allSettled([Mn(),ki(),wc(),Ii()]),b.activePanel==="calibration"&&Ic(),E("footerMessage").textContent=`当前控制：${en()}`}function Bd(){document.querySelectorAll("[data-arm]").forEach(n=>{const t=n.dataset.arm===b.arm;n.classList.toggle("active",t),n.setAttribute("aria-selected",String(t))}),E("jointPanelTitle").textContent=`${en()}关节`,Dc(),at==null||at.setWorkspaceBounds(is()),$a(),$d(),Xd(),jd(),Oi(),fi(),b.activePanel==="benchmark"&&Ir(!1)}function Mc(n){return n==="vertical"||n==="side"||n==="z_parallel"?n:Ly}function Sc(n){b.graspPoseMode=Mc(n),localStorage.setItem("unoarm.graspPoseMode",b.graspPoseMode),Hd(),Hs(),at==null||at.setTargetPose(fn()),zd()}function zd(){const n=b.reachabilityVisible&&(b.arm==="right"||b.arm==="left"),t=currentReachabilityMode();renderReachabilityOverlayState({mode:t,arm:b.arm,loading:n,visible:n});const e=at==null?void 0:at.setGraspReachabilityMode(b.graspPoseMode,{visible:n,arm:b.arm});e&&typeof e.then=="function"&&e.then(i=>{var s,r;if(!i){renderReachabilityOverlayState({mode:t,arm:b.arm,visible:!1});return}currentReachabilityMode()===((s=i.data)==null?void 0:s.mode)&&b.arm===(((r=i.data)==null?void 0:r.arm)||b.arm)&&renderReachabilityOverlayState({mode:i.data.mode,arm:i.data.arm||b.arm,data:i.data})}).catch(i=>{renderReachabilityOverlayState({mode:t,arm:b.arm,error:i})})}function Vd(){const n=E("reachabilityOverlayToggle");n&&(n.checked=b.reachabilityVisible);const t=E("reachabilityOverlayToggleLabel");t&&(t.textContent=b.reachabilityVisible?"显示":"隐藏")}function Wy(n){b.reachabilityVisible=!!n.currentTarget.checked,localStorage.setItem("unoarm.reachabilityVisible",String(b.reachabilityVisible)),Vd(),zd()}function Hd(){document.querySelectorAll("[data-grasp-mode]").forEach(n=>{const t=n.dataset.graspMode===b.graspPoseMode;n.classList.toggle("active",t),n.setAttribute("aria-selected",String(t))})}function $y(){const n=(t,e,i)=>{var s;return(s=E(t))==null?void 0:s.addEventListener(e,i)};n("syncTcpButton","click",Jy),n("planButton","click",sM),n("executeButton","click",oM),n("readVisionButton","click",()=>iM(!0)),n("pickMujocoObjectButton","click",ef),n("pickPlacePlateButton","click",vM),n("leftDropCylinderButton","click",xM),n("pickPlaceTableButton","click",bM),n("savePointQuickButton","click",Fh),n("savePointButton","click",Fh),n("savePresetButton","click",SS),n("emergencyStop","click",wM),n("enableButton","click",yM),n("disableButton","click",MM),n("resetEstopButton","click",SM),n("homeButton","click",EM),n("taskResetButton","click",TM),n("gripperOpenButton","click",()=>Ch("gripper_open")),n("gripperCloseButton","click",()=>Ch("gripper_close")),n("reloadExtrinsicButton","click",nf),n("saveExtrinsicButton","click",AM),n("reloadVlmButton","click",sf),n("saveVlmButton","click",CM),n("vlmConfirmButton","click",PM),n("moveObjectButton","click",gM),n("rebuildSceneButton","click",_M),n("scenePreset","change",nM),n("showPlate","change",Jd),n("reloadPosesButton","click",rf),n("recordPoseButton","click",LM),n("addPoseButton","click",NM),n("toggleCloudButton","click",kM),n("saveCloudButton","click",FM),n("rebuildCloudButton","click",OM),n("loadPlatformButton","click",BM),n("applyPlatformButton","click",zM),n("addCollisionBoxButton","click",$M),n("saveCollisionBoxesButton","click",XM),n("applyCollisionBoxesButton","click",qM),n("clearCollisionBoxesButton","click",YM),n("manualCollisionFields","input",Dh),n("manualCollisionFields","change",Dh),n("manualCollisionFields","click",jM),n("workspaceBoundsFields","input",Nh),n("workspaceBoundsFields","change",Nh),n("saveWorkspaceBoundsButton","click",GM),n("loadJointConfigButton","click",Ic),n("saveJointConfigButton","click",ZM),n("reloadKinematicsButton","click",kc),n("saveKinematicsButton","click",QM),n("reloadOmplConfigButton","click",ar),n("applyOmplPresetButton","click",nS),n("saveOmplPresetButton","click",iS),n("deleteOmplPresetButton","click",sS),n("omplPlannerSelect","change",wa),n("omplPlanningTime","input",wa),n("omplAttempts","input",wa),n("omplIkTimeout","input",wa),n("generateBenchmarkButton","click",dS),n("runBenchmarkButton","click",fS),n("saveBenchmarkPresetButton","click",lS),n("exportBenchmarkCsvButton","click",cS),n("benchmarkSamples","click",MS),n("benchmarkPlanners","change",vS),n("reachabilityOverlayToggle","change",Wy),n("cameraCalibrationPlateToggle","change",Rh),n("cameraCalibrationBlockToggle","change",Rh),n("runCameraCalibrationButton","click",fM),n("clearCameraCalibrationButton","click",pM),n("applyCameraTranslationButton","click",mM),n("cameraCalibrationPoints","click",hM),n("cameraCalibrationPoints","change",dM),["benchmarkCollisionBox","benchmarkBoxCount","benchmarkBoxOffsetCm","benchmarkEdgeCount","benchmarkEdgeDistanceCm","benchmarkRandomCount","benchmarkSeed"].forEach(t=>n(t,t==="benchmarkCollisionBox"?"change":"input",gS)),["benchmarkPlanningTime","benchmarkAttempts","benchmarkIkTimeout"].forEach(t=>{n(t,"input",_S)}),n("addSequenceButton","click",TS),n("clearSequenceButton","click",AS),n("executeSequenceButton","click",CS),n("refreshLogsButton","click",qa),n("logLevelFilter","change",qa),n("pointsList","click",Oh),n("presetsList","click",Oh),n("posesList","click",UM),n("sequenceList","click",wS)}function jy(){window.addEventListener("keydown",n=>{if(Xy(n.target)||!E("confirmDialog").hidden)return;const t=n.key.toLowerCase();if(t==="tab"||t==="["||t==="]"){if(n.preventDefault(),n.repeat)return;const i=t==="["?"left":t==="]"||b.arm==="left"?"right":"left";no(i);return}const e=ql(t);if(e){if(n.preventDefault(),!Ec()){n.repeat||ie(`${en()}尚未使能，不能使用关节键盘控制`,"warn");return}Vs.add(t),Yl(t,!0),n.repeat||Kl(e.joint,e.direction),qy()}}),window.addEventListener("keyup",n=>{const t=n.key.toLowerCase();ql(t)&&(Vs.delete(t),Yl(t,!1),Vs.size||Gd())}),window.addEventListener("blur",Tc),E("jointTelemetry").addEventListener("click",n=>{const t=n.target.closest("[data-jog-joint]");!t||t.disabled||Kl(Number(t.dataset.jogJoint),Number(t.dataset.jogDirection))})}function Xy(n){return n instanceof HTMLElement&&(n.isContentEditable||["INPUT","TEXTAREA","SELECT"].includes(n.tagName))}function ql(n){for(let t=0;t<jl.length;t+=1){const[e,i]=jl[t];if(n===e)return{joint:t+1,direction:1};if(n===i)return{joint:t+1,direction:-1}}return null}function Ec(){const n=b.robotByArm[b.arm];return(n==null?void 0:n.motors_enabled)===!0&&(n==null?void 0:n.estop_active)!==!0}function Yl(n,t){document.querySelectorAll(`[data-jog-key="${n}"]`).forEach(e=>{e.classList.toggle("pressed",t)})}function qy(){yr||(yr=window.setInterval(()=>{const n=Vs.values().next().value,t=ql(n);t&&Kl(t.joint,t.direction)},wy))}function Gd(){yr&&(window.clearInterval(yr),yr=null)}function Tc(){Vs.clear(),Gd(),document.querySelectorAll("[data-jog-key].pressed").forEach(n=>n.classList.remove("pressed"))}async function Kl(n,t){if(!(tl||!Ec())){tl=!0;try{const e=await yt("/api/joint_jog",{arm:b.arm,joint:n,delta_rad:t*Ty,duration_sec:.25}),i=Number(e.target_rad)*180/Math.PI;E("footerMessage").textContent=`${en()} J${n} → ${Number.isFinite(i)?`${i.toFixed(1)}°`:"已发送"}`}catch(e){Tc(),Date.now()-wh>1500&&(ie(`关节键盘控制失败：${e.message}`,"bad",5e3),wh=Date.now())}finally{tl=!1}}}function fn(){const n=Object.fromEntries(Ga.map(([s])=>[s,Number(E(`pose_${s}`).value)])),t=Mc(b.graspPoseMode),e=Ny[t];return{...n,roll:e.roll,pitch:e.pitch,yaw:e.yaw,arm:b.arm,velocity_scaling:Number(E("velocityScaling").value),acceleration_scaling:Number(E("accelerationScaling").value),orientation_tolerance:.5,stay_near:!1,avoid_platform:E("avoidPlatform").checked,collision_boxes:cf(),grasp_orientation_mode:e.grasp_orientation_mode,...ro()}}function cs(){return fn()}function on(n,t){var i;const e=Number((i=E(n))==null?void 0:i.value);return Number.isFinite(e)?e:t}function Yy(n,t){const e=Array.isArray(n)?n.slice(0,3).map(i=>Number(i)):["x","y","z"].map(i=>Number(n==null?void 0:n[i]));if(e.length!==3||!e.every(Number.isFinite))throw new Error(`${t}坐标无效`);return e}function Ky(n){const t=Array.isArray(n==null?void 0:n.extras)?n.extras:[],e=t.find(s=>(s==null?void 0:s.type)==="cylinder"||(s==null?void 0:s.name)==="cylinder")||t.find(s=>(s==null?void 0:s.type)==="slot"||(s==null?void 0:s.name)==="plate");if(!e)throw new Error("场景中没有圆柱/半圆槽目标");const i=Yy(e.position,`${e.name||e.type||"目标"}`);return{...e,center:i,label:e.type==="slot"||e.name==="plate"?"半圆槽":"圆柱"}}function us(n){for(const[t]of Ga)Number.isFinite(Number(n==null?void 0:n[t]))&&(E(`pose_${t}`).value=n[t],Od(t));at==null||at.setTargetPose(fn())}function Qs(){return Object.fromEntries(Wa.map(([n])=>[n,Number(E(`camera_${n}`).value)]))}function io(n){for(const[t]of Wa)Number.isFinite(Number(n==null?void 0:n[t]))&&(E(`camera_${t}`).value=n[t]);b.cameraExtrinsic=Qs(),at==null||at.setCameraExtrinsic(b.cameraExtrinsic)}async function Mn(){var n,t,e;try{const i=await Ef();if(b.lastStatusAt=Date.now(),b.jointState=i.joint_state||null,b.lastPlanByArm={...b.lastPlanByArm,...i.last_plan||{}},at==null||at.applyJointState(b.jointState),i.mujoco_scene&&(at==null||at.setMujocoScene(i.mujoco_scene)),i.gripper)for(const o of["right","left"])i.gripper[o]&&Ji(o,i.gripper[o],{updateBadge:o===b.arm,sceneFallback:!1});$a(),Hs();const s=(n=i.last_plan)==null?void 0:n[b.arm],r=((t=i.status)==null?void 0:t[b.arm])||"就绪",a=(e=s==null?void 0:s.ompl)!=null&&e.planner_id?` / ${s.ompl.planner_id}`:"";Xn("statusPlan","规划",s?`${s.points??"--"} 点 / ${kt(s.duration,2)} s${a}`:r,s?"good":"neutral"),Xn("statusConnection","控制服务","在线","good"),!document.body.dataset.busy&&E("footerMessage").textContent.startsWith("等待")&&(E("footerMessage").textContent="R007 控制服务在线")}catch{Xn("statusConnection","控制服务","离线","bad"),Xn("statusPlan","规划","状态不可用","bad"),document.body.dataset.busy||(E("footerMessage").textContent="R007 控制服务离线")}}async function ki(){var n;try{const t=await Tf();let e=!1,i=!1,s=!1,r=!0,a=!0;for(const o of["right","left"]){const l=((n=t.state)==null?void 0:n[o])||{};b.robotByArm[o]={motors_enabled:l.motors_enabled,estop_active:l.estop_active};const c=E(`${o}ArmState`),u=l.motors_enabled,h=l.estop_active;e||(e=u!=null),s||(s=u===!0),r&&(r=u===!0),a&&(a=u!=null),i||(i=h===!0),c.children[1].textContent=u==null?"未连接":u?"已使能":"未使能",c.children[2].textContent=h==null?"--":h?"急停":"正常",c.dataset.state=h?"bad":u?"good":"neutral"}r&&(r=a),tn("robotOverallState",i?"急停":e?"在线":"未连接",i?"bad":e?"good":"bad"),Ql({anyConnected:e,anyEnabled:s,allEnabled:r,anyEstop:i}),$a()}catch{b.robotByArm.right={motors_enabled:null,estop_active:null},b.robotByArm.left={motors_enabled:null,estop_active:null},tn("robotOverallState","离线","bad"),Ql({anyConnected:!1,anyEnabled:!1,allEnabled:!1,anyEstop:!1}),$a()}}async function wc(){try{const[n,t]=await Promise.all([nl("right"),nl("left")]);b.linksByArm.right=n.links||[],b.linksByArm.left=t.links||[],$d()}catch{Xn("statusTcp","末端位置","读取失败","bad")}}function Wd(){var t;const n=b.arm==="left"?"Left_Gripper_TCP":"Right_Gripper_TCP";return(t=b.linksByArm[b.arm])==null?void 0:t.find(e=>e.name===n&&e.position)}function $d(){const n=Wd(),t=n==null?void 0:n.position;E("tcpX").textContent=kt(t==null?void 0:t[0]),E("tcpY").textContent=kt(t==null?void 0:t[1]),E("tcpZ").textContent=kt(t==null?void 0:t[2]),at==null||at.setTcpFrame(n),Xn("statusTcp","末端位置",t?`${kt(t[0],2)}, ${kt(t[1],2)}, ${kt(t[2],2)}`:"等待数据",t?"good":"neutral")}function Jy(){const n=Wd();if(!n){ie("当前没有可用的末端位置数据","warn");return}us({...fn(),x:n.position[0],y:n.position[1],z:n.position[2]}),ie(`已同步 ${en()}末端位置`)}function $a(){var r;const n=E("jointTelemetry"),t=b.jointState,e=b.arm==="left"?"Left_Joint":"Right_Joint",i=Ec(),s=[];for(let a=1;a<=7;a+=1){const o=((r=t==null?void 0:t.name)==null?void 0:r.indexOf(`${e}${a}`))??-1,c=(o>=0?Number(t.position[o]):NaN)*180/Math.PI,u=Number.isFinite(c)?Math.min(50,Math.abs(c)/3.6):0,h=c<0?"translateX(-100%)":"none",[d,p]=jl[a-1];s.push(`
      <div class="joint-telemetry-row">
        <b>J${a}</b>
        <div class="joint-meter"><span style="width:${u}%;transform:${h}"></span></div>
        <output>${Number.isFinite(c)?`${c.toFixed(1)}°`:"--"}</output>
        <div class="joint-jog-keys">
          <button type="button" data-jog-joint="${a}" data-jog-direction="1" data-jog-key="${d}"
            title="关节 ${a} 正向微动" ${i?"":"disabled"}>${d.toUpperCase()}</button>
          <button type="button" data-jog-joint="${a}" data-jog-direction="-1" data-jog-key="${p}"
            title="关节 ${a} 反向微动" ${i?"":"disabled"}>${p.toUpperCase()}</button>
        </div>
      </div>
    `)}n.innerHTML=s.join(""),Vs.forEach(a=>Yl(a,!0)),E("jointStamp").textContent=t?"实时":"--"}async function Ii(){try{const n=await wf(b.arm);Ji(b.arm,n,{sceneFallback:!1})}catch{tn("gripperState","未知","bad")}}function Ac(n){return n==="left"?"Left_Gripper_Joint":"Right_Gripper_Joint"}function ja(n){const t=Number(n==null?void 0:n.position);return Number.isFinite(t)?t:(n==null?void 0:n.state)==="closed"?wr:ns}function Zy(n){const t=Number(n==null?void 0:n.target_position);return Number.isFinite(t)?t:((n==null?void 0:n.target_state)==="closed"?"closed":n==null?void 0:n.state)==="closed"?wr:ns}function Qy(n,t={}){const e=n==="left"?"left":"right",i=t.state==="closed"?"closed":"open",s=t.target_state==="closed"?"closed":t.target_state==="open"?"open":i,r=Number(t.progress);return{...t,arm:e,state:i,target_state:s,joint:t.joint||Ac(e),position:ja(t),target_position:Zy(t),progress:Number.isFinite(r)?Math.max(0,Math.min(1,r)):void 0,moving:t.moving===!0,simulated:t.simulated===!0}}function Ji(n,t,e={}){const i=Qy(n,t),s=b.gripperCommand[i.arm];s!=null&&s.target&&i.state!==s.target&&Date.now()-s.at<Cy&&(i.state=s.target,i.target_state=s.target,i.target_position=s.target==="closed"?wr:ns,i.moving=!0),b.gripperByArm[i.arm]=i,(e.forceScene===!0||e.sceneFallback!==!1&&!eM(i.arm))&&(at==null||at.applyJointState(tM(i.arm,i))),e.updateBadge!==!1&&i.arm===b.arm&&Xd()}function jd(){for(const n of["right","left"])Ji(n,b.gripperByArm[n],{updateBadge:n===b.arm})}function tM(n,t){const e=Ry[n==="left"?"left":"right"],i=ja(t);return{name:Object.keys(e),position:Object.values(e).map(s=>i*s),velocity:Object.keys(e).map(()=>0)}}function eM(n){var e;const t=Ac(n);return Array.isArray((e=b.jointState)==null?void 0:e.name)&&b.jointState.name.includes(t)}function Xd(){const n=b.gripperByArm[b.arm],e=((n==null?void 0:n.target_state)||(n==null?void 0:n.state))==="closed",i=(n==null?void 0:n.moving)===!0,s=Number(n==null?void 0:n.progress),r=i&&Number.isFinite(s)?` ${Math.round(s*100)}%`:"",a=(n==null?void 0:n.simulated)===!0?" · 仿真":"",o=i?`${e?"闭合中":"张开中"}${r}`:`${e?"已闭合":"已张开"}`;tn("gripperState",`${o}${a}`,i?"warn":"good")}async function qd(){var n;try{const[t,e]=await Promise.allSettled([Bh(),Af()]);if(t.status==="fulfilled"&&((n=t.value)!=null&&n.urdf_xyz_m)){b.visionTarget=t.value;const i=b.visionTarget.urdf_xyz_m;at==null||at.setVisionTarget(b.visionTarget),E("visionX").textContent=kt(i[0]),E("visionY").textContent=kt(i[1]),E("visionZ").textContent=kt(i[2]),Xn("statusVision","视觉",`${kt(i[0],2)}, ${kt(i[1],2)}, ${kt(i[2],2)}`,"good")}else Xn("statusVision","视觉","无目标","neutral");if(e.status==="fulfilled"){const i=e.value.objects||e.value.detections||[];at==null||at.setObjectMarkers(i),tn("cloudState","视觉在线","good")}}catch{Xn("statusVision","视觉","离线","bad"),tn("cloudState","未连接","bad")}}async function Yd(){await Jl({getStatus:Cf,imageId:"simCameraImage",placeholderId:"simCameraPlaceholder",stateId:"simCameraState",infoId:"simCameraInfo",snapshotPath:"/mujoco_camera/snapshot.jpg",streamPath:"/mujoco_camera/stream.mjpg",waitingText:"等待头顶 D455"})}async function Kd(){await Promise.allSettled([Jl({getStatus:Rf,imageId:"rightWristCameraImage",placeholderId:"rightWristCameraPlaceholder",stateId:"rightWristCameraState",infoId:"rightWristCameraInfo",snapshotPath:"/right_wrist_camera/snapshot.jpg",streamPath:"/right_wrist_camera/stream.mjpg",waitingText:"等待右腕 RGB-D"}),Jl({getStatus:Pf,imageId:"leftWristCameraImage",placeholderId:"leftWristCameraPlaceholder",stateId:"leftWristCameraState",infoId:"leftWristCameraInfo",snapshotPath:"/left_wrist_camera/snapshot.jpg",streamPath:"/left_wrist_camera/stream.mjpg",waitingText:"等待左腕 RGB-D"})])}async function Cc(){var n;try{const t=await ic();at==null||at.setMujocoScene(t),Xl=t.presets||Xl,t.config&&!((n=E("scenePreset"))!=null&&n.dataset.initialized)&&(Zd(t.config),E("scenePreset").dataset.initialized="1")}catch{}}function Jd(){var t;const n=!!((t=E("showPlate"))!=null&&t.checked);E("platePositionFields").hidden=!n}function Zd(n={}){const t=E("scenePreset");t&&(t.value=n.preset&&t.querySelector(`option[value="${n.preset}"]`)?n.preset:"custom");const e=Array.isArray(n.objects)?n.objects:[];E("objX").value=n.object_x??0,E("objY").value=n.object_y??-.73,E("tableX").value=n.table_x??.4,E("tableY").value=n.table_y??.3,E("plateX").value=n.plate_x??-.08,E("plateY").value=n.plate_y??-.72,E("showPlate").checked=e.includes("plate"),E("showCylinder").checked=e.includes("cylinder"),E("scenePresetDescription").textContent=n.description||"自定义物体与桌面布局",Jd()}function nM(){const n=E("scenePreset").value,t=Xl[n];t?Zd({...t,preset:n}):E("scenePresetDescription").textContent="自定义物体与桌面布局"}async function Jl({getStatus:n,imageId:t,placeholderId:e,stateId:i,infoId:s,snapshotPath:r,streamPath:a,waitingText:o}){const l=E(t),c=E(e);try{const u=await n();if(!u.ready)throw new Error(u.error||"等待摄像头图像");return l.onload=()=>{l.hidden=!1,c.hidden=!0},l.onerror=()=>{l.hidden=!0,c.hidden=!1},a?l.dataset.streamPath!==a&&(l.dataset.streamPath=a,l.src=a):l.src=`${r}?t=${Date.now()}`,tn(i,"实时","good"),E(s).textContent=`${u.width} × ${u.height} · ${u.frame_id||u.topic}`,u}catch(u){return l.hidden=!0,c.hidden=!1,tn(i,"未连接","bad"),E(s).textContent=u.message||o,null}}async function iM(n){await de("读取视觉目标",async()=>{const t=await Bh();if(!(t!=null&&t.urdf_xyz_m))throw new Error((t==null?void 0:t.error)||"未找到有效视觉目标");if(b.visionTarget=t,at.setVisionTarget(t),n){const[e,i,s]=t.urdf_xyz_m;us({...fn(),x:e,y:i,z:s})}return t})}async function sM(){await de("运动规划",async()=>{var t,e,i;b.lastDecisionError=null,Hs({pending:!0});const n=await yt("/api/plan",fn());return((t=n.plan)!=null&&t.platform_obstacles||(e=n.plan)!=null&&e.platform_obstacle)&&at.setPlatformObstacle(n.plan.platform_obstacles||n.plan.platform_obstacle),so((i=n.plan)==null?void 0:i.manual_collision),n.plan&&(b.lastPlanByArm[b.arm]=n.plan,Hs()),await Mn(),n},{onError:n=>{var s;const t=(s=n==null?void 0:n.payload)==null?void 0:s.failure;b.lastDecisionError=t||{title:n.message},Hs();const e=t!=null&&t.stage_label?`${t.stage_label}失败`:"规划失败",i=(t==null?void 0:t.title)||n.message;Xn("statusPlan",e,i,"bad")}})}function Hs(n={}){var g;const t=fn(),e=b.lastPlanByArm[b.arm],i=(e==null?void 0:e.grasp_orientation)||null,s=t.grasp_orientation_mode,r=!!(i&&aM(i.mode,s)),a=r?i.mode:s,o=Th[a]||Th[t.grasp_orientation_mode],l=Array.isArray(i==null?void 0:i.rejected_candidates)?i.rejected_candidates.length:0,c=Number(i==null?void 0:i.candidate_index),u=r&&Number.isFinite(c)?Math.max(1,c):null,h=r?Math.max(u||1,l+1):3,d=rM({detail:o,orientation:i,plan:e,pose:t,selected:r,pending:n.pending,activeIndex:u,totalScan:h});b.lastDecisionError&&d.push({label:"失败",detail:b.lastDecisionError.stage_label||b.lastDecisionError.title||"未找到可行候选",state:"failed"});const p=`
    <div class="decision-title">
      <strong>${Jt(o.title)}</strong>
      <span>${Jt(r?"已确定":n.pending?"规划中":"预览")}</span>
    </div>
    <p>${Jt(o.summary)}</p>
    <div class="decision-chain">
      ${d.map(_=>`
        <div class="decision-step" data-state="${_.state}">
          <b>${Jt(_.label)}</b>
          <span>${Jt(_.detail)}</span>
        </div>
      `).join("")}
    </div>
    ${r?`
      <div class="decision-meta">
        <span>候选 ${u??"--"}</span>
        <span>拒绝 ${l}</span>
        <span>${Jt(((g=e==null?void 0:e.ompl)==null?void 0:g.planner_id)||"IK")}</span>
      </div>
    `:""}
  `;for(const _ of["decisionPreview","activeDecisionPreview"]){const m=E(_);m&&(m.innerHTML=p)}tn("decisionState",r?"已确定":n.pending?"规划中":"预览",r?"good":n.pending?"warn":"neutral")}function rM({detail:n,orientation:t,plan:e,pose:i,selected:s,pending:r,activeIndex:a,totalScan:o}){var h,d;const l=Math.min(o,6),c=[],u=s&&String((e==null?void 0:e.mode)||"").includes("direct IK fallback");for(let p=1;p<=l;p+=1){const g=s&&a&&p<a,_=s&&a===p,f=_?"done":g?"failed":r&&p===1?"active":"idle",P=_?u?"failed":"done":g?"failed":"idle",L=_?n.candidateText(t):`IK ${kt(i.ik_timeout,2)}s`,w=_?u?"OMPL失败，使用IK直达":`${((h=e==null?void 0:e.ompl)==null?void 0:h.planner_id)||"OMPL"} ${kt(((d=e==null?void 0:e.ompl)==null?void 0:d.planning_time)??i.planning_time,1)}s`:g?"未通过":"等待IK通过";c.push({label:`${n.scanLabel}${p} IK`,detail:L,state:f}),c.push({label:`${n.scanLabel}${p} OMPL`,detail:w,state:P})}return c.push({label:s?"确定":r?"等待结果":"待确定",detail:s?`${n.title} · ${(e==null?void 0:e.mode)||"规划完成"}`:n.title,state:s?"done":r?"active":"idle"}),c}function gr(n){const t=[];for(const e of["roll","pitch","yaw"]){const i=Number(n==null?void 0:n[e]);Number.isFinite(i)&&t.push(`${e} ${kt(i,2)}`)}return t.length?t.join(" / "):"候选姿态"}function aM(n,t){return n===t?!0:n==="angled_grasp"&&t==="z_parallel_grasp"}async function oM(){await de("执行规划",async()=>{const n=await yt("/api/execute",{arm:b.arm});return await Mn(),n},{confirm:`即将让${en()}执行最后一次规划，确认工作区安全后继续。`,confirmTitle:"确认执行轨迹"})}async function Ch(n){const t=["approach","grasp"].includes(n)?cs():fn(),i={approach:["接近目标","/api/approach",{pose:t,offset_z:on("pickApproach",.1)}],lift:["抬起机械臂","/api/lift",{arm:b.arm,offset_z:on("pickLift",.1)}],grasp:["抓取目标","/api/grasp",{pose:t}],place:["放置目标","/api/place",{pose:t}],gripper_open:["张开夹爪","/api/gripper",{arm:b.arm,action:"open"}],gripper_close:["闭合夹爪","/api/gripper",{arm:b.arm,action:"close"}]}[n];if(!i)return;if(n==="gripper_open"||n==="gripper_close"){const r=b.arm,a={...b.gripperByArm[r]||{}},o=n==="gripper_close"?"closed":"open",l=b.gripperCommand[r],c=Date.now();if(l.inFlight||l.target===o&&c-l.at<Ay)return;b.gripperCommand[r]={target:o,at:c,inFlight:!0},Ji(r,{state:o,target_state:o,arm:r,joint:Ac(r),position:ja(a),target_position:o==="closed"?wr:ns,moving:ja(a)!==(o==="closed"?wr:ns),simulated:!0},{sceneFallback:!1}),E("footerMessage").textContent=`${i[0]}已发送`;try{const u=await yt(i[1],i[2]);u!=null&&u.result&&Ji(r,u.result,{sceneFallback:!1})}catch(u){Ji(r,a,{sceneFallback:!1}),ie(`${i[0]}失败：${u.message}`,"bad",5e3)}finally{b.gripperCommand[r]={target:o,at:Date.now(),inFlight:!1}}setTimeout(()=>{Ii(),Mn()},120);return}const s=await de(i[0],async()=>yt(i[1],i[2]),{confirm:`${i[0]}将驱动${en()}，确认周围无人员或障碍物。`});(n==="gripper_open"||n==="gripper_close")&&Ji(b.arm,(s==null?void 0:s.result)||{},{forceScene:!0}),so(Cr(s)),Ii(),Mn()}async function lM(n={}){const t=Number.isFinite(Number(n.approachHeight))?Number(n.approachHeight):on("pickApproach",.1),e=Number.isFinite(Number(n.descendDistance))?Number(n.descendDistance):on("pickDescend",.05),i=await de("自动夹取",async()=>yt("/api/pick",{pose:n.pose||cs(),approach_height:t,descend_distance:e,hold_seconds:on("pickHold",1),lift_height:on("pickLift",.1)}),{confirm:`即将由${en()}执行完整夹取流程，请确认目标位姿和现场安全。`,confirmTitle:"确认自动夹取"});so(Cr(i)),Ii()}function Rc(n){var s,r;const t=b.cameraCalibration.points[n];if(!t)throw new Error("标定点不存在");const e=Number(((s=document.querySelector(`[data-calibration-x="${n}"]`))==null?void 0:s.value)??t.x),i=Number(((r=document.querySelector(`[data-calibration-y="${n}"]`))==null?void 0:r.value)??t.y);if(!Number.isFinite(e)||!Number.isFinite(i))throw new Error("标定点坐标无效");return t.x=e,t.y=i,t}async function Zl(){try{const n=await ic(),t=new Set((n.extras||[]).map(e=>e.name));E("cameraCalibrationPlateToggle").checked=t.has("plate"),E("cameraCalibrationBlockToggle").checked=t.has("calibration_block"),b.cameraCalibration.sceneReady=t.has("plate")&&t.has("calibration_block"),b.cameraCalibration.points.some(e=>e.sample)||(b.cameraCalibration.points=Fd(n.config||{})),b.cameraCalibration.sceneReady||(E("cameraCalibrationStatus").textContent="当前场景缺少盘子或 50 mm 标定块，请重建并重启仿真"),er()}catch(n){b.cameraCalibration.sceneReady=!1,E("cameraCalibrationStatus").textContent=`场景读取失败：${n.message}`,er()}}async function Rh(){const n=E("cameraCalibrationPlateToggle"),t=E("cameraCalibrationBlockToggle"),e=!!n.checked,i=!!t.checked;n.disabled=!0,t.disabled=!0,E("cameraCalibrationStatus").textContent="正在更新 MuJoCo 标定物...";try{await Lf({plate:e,calibration_block:i}),await Promise.all([Cc(),Zl()]);const s=[e?"盘子":null,i?"标定块":null].filter(Boolean);E("cameraCalibrationStatus").textContent=s.length?`MuJoCo 标定物已更新：显示${s.join("、")}`:"盘子和标定块已从 MuJoCo 场景隐藏"}catch(s){E("cameraCalibrationStatus").textContent=`标定物更新失败：${s.message}`,ie(`标定物显隐失败：${s.message}`,"bad",6e3),await Zl()}finally{n.disabled=!1,t.disabled=!1}}function Pc(){const n=E("cameraCalibrationImage"),t="/mujoco_camera/stream.mjpg";n&&n.dataset.streamPath!==t&&(n.dataset.streamPath=t,n.src=t)}function tr(n,t){b.cameraCalibration.running=n,t&&(E("cameraCalibrationStatus").textContent=t),er()}function er(){const n=b.cameraCalibration,t=n.points.filter(d=>d.sample),e=E("cameraCalibrationPoints");if(e){let d=null;e.innerHTML=n.points.map((p,g)=>{const _=p.sample,m=_?`${kt(_.xy_error_mm,1)} / ${kt(_.z_error_mm,1)}`:"待采集",f=p.surface!==d?`<div class="camera-calibration-group"><b>${Jt(p.surfaceLabel)}</b><span>${p.surface==="table"?"9 点":"5 点"}</span></div>`:"";return d=p.surface,`
        ${f}
        <div class="camera-calibration-point" data-state="${_?_.xy_error_mm<=10?"good":"warn":"neutral"}">
          <span class="camera-calibration-point-index">${g+1}</span>
          <label>X<input type="number" step="0.01" value="${kt(p.x,2)}" data-calibration-x="${g}" ${n.running?"disabled":""}></label>
          <label>Y<input type="number" step="0.01" value="${kt(p.y,2)}" data-calibration-y="${g}" ${n.running?"disabled":""}></label>
          <span class="camera-calibration-point-error" title="XY / Z 误差 (mm)">${Jt(m)}</span>
          <span class="camera-calibration-role">${p.validation?"验证":"拟合"}</span>
          <button class="icon-button" type="button" data-calibration-action="move" data-calibration-index="${g}" aria-label="移动定子到标定点 ${g+1}" title="移动定子" ${n.running?"disabled":""}><i data-lucide="move-3d"></i></button>
          <button class="icon-button primary" type="button" data-calibration-action="capture" data-calibration-index="${g}" aria-label="采集标定点 ${g+1}" title="采集 RGB-D 与真值" ${n.running?"disabled":""}><i data-lucide="crosshair"></i></button>
        </div>`}).join("")}const i=E("cameraCalibrationResults");i&&(i.innerHTML=t.length?`
      <div class="camera-calibration-result camera-calibration-result-head"><span>点位</span><span>真值 XYZ</span><span>视觉 XYZ</span><span>XY / Z</span></div>
      ${t.map(d=>{const p=d.sample,g=p.truth_xyz_m,_=p.estimated_world_xyz_m;return`<div class="camera-calibration-result">
          <span>P${n.points.indexOf(d)+1}</span>
          <span>${g.map(m=>kt(m,3)).join(", ")}</span>
          <span>${_.map(m=>kt(m,3)).join(", ")}</span>
          <b data-state="${p.xy_error_mm<=10&&p.z_error_mm<=10?"good":"bad"}">${kt(p.xy_error_mm,1)} / ${kt(p.z_error_mm,1)}</b>
        </div>`}).join("")}`:'<div class="empty-state">尚无采样结果</div>');const s=n.analysis,r=n.fit,a=r==null?void 0:r.validation_metrics,o=E("cameraCalibrationMetrics");o&&(o.innerHTML=s?`
      <div><span>当前 RMS XY / Z</span><b>${kt(s.rms_xy_error_mm,1)} / ${kt(s.rms_z_error_mm,1)} mm</b></div>
      <div><span>当前最大 XY / Z</span><b>${kt(s.max_xy_error_mm,1)} / ${kt(s.max_z_error_mm,1)} mm</b></div>
      <div><span>拟合后验证 RMS</span><b>${a?`${kt(a.rms_xy_error_mm,1)} / ${kt(a.rms_z_error_mm,1)} mm`:"--"}</b></div>
      <div><span>拟合后验证最大</span><b>${a?`${kt(a.max_xy_error_mm,1)} / ${kt(a.max_z_error_mm,1)} mm`:"--"}</b></div>`:"");const l=!!(a&&a.count>=5&&a.rms_xy_error_mm<=10&&a.rms_z_error_mm<=10&&a.max_xy_error_mm<=15&&a.max_z_error_mm<=15);let c="neutral",u=`已采集 ${t.length}/${n.points.length}，每点 ${Ar} 帧中位数`;n.sceneReady?r&&l?(c="good",u=`刚体外参验证通过：${r.fit_count} 个拟合点 + ${r.validation_count} 个独立验证点`):r&&a&&(c="bad",u="刚体拟合后的验证误差仍超限，优先检查内参、深度尺度或目标中心提取"):(c="bad",u="场景缺少盘子或标定块，不能执行多高度拟合");const h=E("cameraCalibrationDiagnosis");h&&(h.dataset.state=c,h.textContent=u),E("cameraCalibrationCount").textContent=`${t.length} / ${n.points.length}`,tn("cameraCalibrationState",n.running?"采集中":r?"已拟合":t.length?"采样中":"未采样",n.running?"warn":r?"good":"neutral"),E("runCameraCalibrationButton").disabled=n.running||!n.sceneReady,E("clearCameraCalibrationButton").disabled=n.running||!t.length,E("applyCameraTranslationButton").disabled=n.running||!l,os()}function cM(n){if(!Array.isArray(n)||n.length<4)return 1/0;const t=Number(n[1]),e=Number(n[2]),i=Math.max(-1,Math.min(1,1-2*(t*t+e*e)));return Math.acos(i)}async function uM(n,t=6500){var r,a,o;const e=Date.now()+t;let i=0;const s={table:.8435,plate:.8595,block:.8935}[n.surface];for(;Date.now()<e;){const l=await ec(),c=(r=l==null?void 0:l.target)==null?void 0:r.object_center,u=((a=l==null?void 0:l.target)==null?void 0:a.object_motion)||{},h=Math.abs(Number(c==null?void 0:c.x)-n.x)<=.012&&Math.abs(Number(c==null?void 0:c.y)-n.y)<=.012&&Math.abs(Number(c==null?void 0:c.z)-s)<=.012,d=Number(u.linear_speed_mps)<=.003&&Number(u.angular_speed_radps)<=.05&&cM((o=l==null?void 0:l.target)==null?void 0:o.orientation_wxyz)<=.12;if(i=h&&d?i+1:0,i>=3)return l;await new Promise(p=>window.setTimeout(p,120))}throw new Error(`P${b.cameraCalibration.points.indexOf(n)+1} 未在${n.surfaceLabel}稳定落定`)}async function Qd(n){const t=Rc(n);await zh({x:t.x,y:t.y,z:t.spawnZ}),await uM(t),Pc()}async function tf(n){var _;const t=Rc(n),e=[];let i=null,s=null;const r=Date.now()+12e3;for(;e.length<Ar&&Date.now()<r;){let m;try{m=await Df(),s=null}catch(L){s=L,await new Promise(w=>window.setTimeout(w,120));continue}if(m.rgb_stamp===i){await new Promise(L=>window.setTimeout(L,80));continue}const f=await ec(),P=(_=f==null?void 0:f.target)==null?void 0:_.object_center;if(![P==null?void 0:P.x,P==null?void 0:P.y,P==null?void 0:P.z].every(L=>Number.isFinite(Number(L))))throw new Error("/api/mujoco/grasp_target 未返回有效真值");e.push({...m,truth_xyz_m:[Number(P.x),Number(P.y),Number(P.z)]}),i=m.rgb_stamp}if(e.length<Ar){const m=s!=null&&s.message?`：${s.message}`:"";throw new Error(`等待 5 个独立且同步的 RGB-D 帧超时${m}`)}const a=m=>{const f=m.map(Number).sort((P,L)=>P-L);return f[Math.floor(f.length/2)]},o=m=>[0,1,2].map(f=>a(e.map(P=>P[m][f]))),l=o("truth_xyz_m"),c=o("estimated_world_xyz_m"),u=o("camera_xyz_m"),h=l.map((m,f)=>m-c[f]);t.sample={...e[Math.floor(e.length/2)],surface:t.surface,validation:t.validation,frame_count:e.length,truth_xyz_m:l,camera_xyz_m:u,estimated_world_xyz_m:c,error_m:h,xy_error_mm:Math.hypot(h[0],h[1])*1e3,z_error_mm:Math.abs(h[2])*1e3};const d=b.cameraCalibration.points.filter(m=>m.sample).map(m=>m.sample),p=await If(d);b.cameraCalibration.analysis=p.analysis;const g=new Set(d.filter(m=>!m.validation).map(m=>m.surface));if(d.filter(m=>!m.validation).length>=6&&g.size>=2)try{const m=await Uf(d,b.cameraExtrinsic);b.cameraCalibration.fit=m.fit}catch{b.cameraCalibration.fit=null}Pc()}async function hM(n){const t=n.target.closest("[data-calibration-action]");if(!t||b.cameraCalibration.running)return;const e=Number(t.dataset.calibrationIndex),i=t.dataset.calibrationAction;try{tr(!0,i==="move"?`正在移动定子到 P${e+1}...`:`正在采集 P${e+1}...`),i==="move"?await Qd(e):await tf(e),E("cameraCalibrationStatus").textContent=i==="move"?`定子已移动到 P${e+1}`:`P${e+1} 已记录 ${Ar} 帧中位数`}catch(s){E("cameraCalibrationStatus").textContent=`操作失败：${s.message}`,ie(`相机标定失败：${s.message}`,"bad",6e3)}finally{tr(!1)}}function dM(n){const t=Number(n.target.dataset.calibrationX??n.target.dataset.calibrationY);Number.isInteger(t)&&Rc(t)}async function fM(){if(!b.cameraCalibration.running){b.cameraCalibration.points.forEach(n=>{n.sample=null}),b.cameraCalibration.analysis=null,b.cameraCalibration.fit=null;try{tr(!0,"开始三点自动采集...");for(let n=0;n<b.cameraCalibration.points.length;n+=1)E("cameraCalibrationStatus").textContent=`P${n+1}/${b.cameraCalibration.points.length} ${b.cameraCalibration.points[n].surfaceLabel}：移动并等待稳定...`,await Qd(n),E("cameraCalibrationStatus").textContent=`P${n+1}/${b.cameraCalibration.points.length}：采集 ${Ar} 个 RGB-D 帧...`,await tf(n),er();E("cameraCalibrationStatus").textContent="19 点三高度采集完成，已生成刚体外参拟合与独立验证"}catch(n){E("cameraCalibrationStatus").textContent=`自动采集失败：${n.message}`,ie(`自动标定失败：${n.message}`,"bad",6e3)}finally{tr(!1)}}}function pM(){b.cameraCalibration.points.forEach(n=>{n.sample=null}),b.cameraCalibration.analysis=null,b.cameraCalibration.fit=null,E("cameraCalibrationStatus").textContent="采样已清空",er()}async function mM(){const n=b.cameraCalibration.fit,t=n==null?void 0:n.fitted_extrinsic;if(!(!t||!await ls(`将保存 ${n.fit_count} 个拟合点求得的固定头部相机 XYZ + RPY，并以 ${n.validation_count} 个独立点的误差作为验收结果。`,"应用刚体外参拟合","保存外参")))try{tr(!0,"正在保存固定头部相机外参...");const i=await yt("/api/camera_extrinsic",t);io(i.extrinsic||t),b.cameraCalibration.points.forEach(s=>{s.sample=null}),b.cameraCalibration.analysis=null,b.cameraCalibration.fit=null,E("cameraCalibrationStatus").textContent="刚体外参已保存，请重新执行 19 点采集验证",ie("固定头部相机 XYZ + RPY 外参已保存","good",5e3)}catch(i){E("cameraCalibrationStatus").textContent=`保存失败：${i.message}`,ie(`保存相机外参失败：${i.message}`,"bad",6e3)}finally{tr(!1)}}async function gM(){const n=Number(E("objX").value),t=Number(E("objY").value);if(!Number.isFinite(n)||!Number.isFinite(t)){ie("坐标无效");return}await de("移动方块",async()=>zh({x:n,y:t})),ie("方块已移动")}async function _M(){const n=E("showPlate").checked,t=E("showCylinder").checked,e=Number(E("tableX").value),i=Number(E("tableY").value),s=Number(E("objX").value),r=Number(E("objY").value),a=Number(E("plateX").value),o=Number(E("plateY").value),l=E("scenePreset").value;E("sceneStatus").textContent="重建中... (需要重启仿真生效)";const c=await de("重建场景",async()=>Promise.race([Nf({preset:l,plate:n,cylinder:t,table_x:e,table_y:i,object_x:s,object_y:r,plate_x:a,plate_y:o}),new Promise((h,d)=>window.setTimeout(()=>d(new Error("场景重建超过 40 秒，请检查后端日志")),4e4))]),{onError:h=>{E("sceneStatus").textContent=`重建失败：${(h==null?void 0:h.message)||"未知错误"}`}});if(!c)return;const u=c.objects||"";E("sceneStatus").textContent="场景已保存并重建 ("+(u||"基础场景")+"), 重启仿真后生效"}async function ef(){let n;try{const t=await nc();if(n=t==null?void 0:t.target,![n==null?void 0:n.x,n==null?void 0:n.y,n==null?void 0:n.z].every(i=>Number.isFinite(Number(i))))throw new Error("头部 RGB-D 未返回有效的定子中心坐标");Sc("z_parallel"),us(n);const e=n.object_center||n;E("footerMessage").textContent=`视觉定子中心：X ${kt(e.x,3)} / Y ${kt(e.y,3)} / Z ${kt(e.z,3)} m；已应用TCP偏移`}catch(t){ie(`视觉定位失败：${t.message}`,"bad",5e3);return}await lM({approachHeight:.1,descendDistance:.1,pose:{...cs(),planner_id:"RRTConnectkConfigDefault",planning_time:5,attempts:4,ik_timeout:1.5,mujoco_object_center:n.object_center||n,mujoco_nominal_tcp:{x:n.x,y:n.y,z:n.z}}})}async function vM(){var a,o,l,c;const n=E("pickPlacePlateButton"),t=n==null?void 0:n.querySelector("span"),e=(n==null?void 0:n.dataset.stage)==="place";let i;try{const u=await nc();if(i=u==null?void 0:u.target,![i==null?void 0:i.x,i==null?void 0:i.y,i==null?void 0:i.z].every(h=>Number.isFinite(Number(h))))throw new Error("头部 RGB-D 未返回有效的定子中心坐标")}catch(u){E("pickPlaceStatus").textContent=`读取目标失败：${u.message}`;return}if(!e){E("pickPlaceStatus").textContent="正在拿起定子...",await ef();const u=await yt("/api/status");!!((o=(a=u==null?void 0:u.mujoco_scene)==null?void 0:a.object)!=null&&o.attached)?(n&&(n.dataset.stage="place"),t&&(t.textContent="放入槽中"),E("pickPlaceStatus").textContent="定子已拿起；再次点击放入槽中"):E("pickPlaceStatus").textContent="定子未能稳定拿起，请重试";return}E("pickPlaceStatus").textContent="正在移动到槽并放置...";const s=await de("放入槽中",()=>yt("/api/mujoco/pick_place_plate",{pose:{...cs(),arm:b.arm,x:i.x,y:i.y,z:i.z,grasp_orientation_mode:"angled_grasp",planner_id:"RRTConnectkConfigDefault",planning_time:5,attempts:4,ik_timeout:1.5,mujoco_object_center:i.object_center||i,mujoco_nominal_tcp:{x:i.x,y:i.y,z:i.z}},hover_height:.12}),{confirm:"机械臂将把已拿起的定子移动到槽上方并松爪。",confirmTitle:"确认放入槽中",onError:u=>{E("pickPlaceStatus").textContent=`任务失败：${(u==null?void 0:u.message)||"未知错误"}`}});if(!s)return;const r=!!((l=s.result)!=null&&l.completed);E("pickPlaceStatus").textContent=r?"任务完成：定子已释放到槽中":`任务未完成：停在 ${((c=s.result)==null?void 0:c.stage)||"未知"} 阶段`,r&&(n&&(n.dataset.stage="pick"),t&&(t.textContent="拿起定子")),Mn()}async function xM(){var d,p,g;let n,t;try{b.arm!=="left"&&await no("left");const[_,m]=await Promise.all([ec(),ic()]);if(n=_==null?void 0:_.target,![n==null?void 0:n.x,n==null?void 0:n.y,n==null?void 0:n.z].every(P=>Number.isFinite(Number(P))))throw new Error("仿真目标坐标无效");t=Ky(m),Sc("z_parallel"),us(n);const f=n.object_center||n;E("footerMessage").textContent=`智能抓取目标：转子 (${kt(f.x,3)}, ${kt(f.y,3)}, ${kt(f.z,3)}) → ${t.label}中心 (${t.center.map(P=>kt(P,3)).join(", ")})`}catch(_){E("pickPlaceStatus").textContent=`准备失败：${_.message}`,ie(`准备智能抓取任务失败：${_.message}`,"bad",5e3);return}E("pickPlaceStatus").textContent="等待确认：从转子上方下探搜索可解抓取点...";const e=await de("智能抓取转子入圆柱",async()=>(E("pickPlaceStatus").textContent=`正在扫描垂直/斜向/侧面抓取，并移动到${t.label}中心上方...`,kf({smart_search:!0,grasp_modes:["vertical_grasp","angled_grasp","side_grasp"],search_offsets:[.06,.045,.03,.015,.005,-.005,-.015,-.03,-.045],approach_height:.065,descend_distance:.065,hold_seconds:.45,lift_height:.055,fall_seconds:1.8,drop_heights:[0,.015,.03,.05],release_descend_height:.045})),{confirm:"将用左手从转子上方开始搜索可解点；若不可解会逐步下探，并在垂直、斜向、侧面三种抓取中选择可执行方案，保持转子与桌面水平后移动到圆柱/半圆槽中心上方开爪。",confirmTitle:"确认智能抓取投放",success:"智能抓取投放完成",onError:_=>{E("pickPlaceStatus").textContent=`任务失败：${(_==null?void 0:_.message)||"未知错误"}`}}),i=e==null?void 0:e.result;if(!i)return;if(!i.completed){E("pickPlaceStatus").textContent=`任务未完成：${i.error||i.stage||"未知阶段"}`,Mn(),Ii();return}const s=(Array.isArray(i.steps)?i.steps:[]).find(_=>(_==null?void 0:_.action)==="smart_grasp_search"),r=(d=s==null?void 0:s.search)==null?void 0:d.mode,a=Dy[r]||"已选抓取",o=Number((p=s==null?void 0:s.search)==null?void 0:p.probe_offset_m),l=Number(i.xy_error_m),c=Number((g=i.horizontal_transfer)==null?void 0:g.level_error_deg),u=Number.isFinite(o)?`${a} / 下探偏移 ${kt(o*1e3,1)} mm`:a,h=Number.isFinite(c)?`，水平误差 ${kt(c,2)}°`:"";E("pickPlaceStatus").textContent=Number.isFinite(l)?`任务完成：${u}${h}，最终 XY 误差 ${kt(l*1e3,1)} mm`:`任务完成：${u}${h}，转子已释放到目标中心上方`,Mn(),Ii()}async function bM(){var i,s;let n;try{const r=await nc();if(n=r==null?void 0:r.target,![n==null?void 0:n.x,n==null?void 0:n.y,n==null?void 0:n.z].every(a=>Number.isFinite(Number(a))))throw new Error("未识别到红色方块的有效坐标")}catch(r){E("pickPlaceStatus").textContent=`识别失败：${r.message}`;return}E("pickPlaceStatus").textContent="正在执行：头部 RGB-D 定位 → 抓取 → 返回桌面原位 → 放置...";const t=await de("抓取并放回桌面原位",()=>yt("/api/mujoco/pick_place_table",{pose:{...cs(),arm:b.arm,x:n.x,y:n.y,z:n.z,grasp_orientation_mode:"angled_grasp",planner_id:"RRTConnectkConfigDefault",planning_time:5,attempts:4,ik_timeout:1.5,mujoco_object_center:n.object_center||n,mujoco_nominal_tcp:{x:n.x,y:n.y,z:n.z}},hover_height:.12}),{confirm:"将识别并抓取红色方块，然后放回桌面初始位置。",confirmTitle:"确认执行放回原位任务",onError:r=>{E("pickPlaceStatus").textContent=`任务失败：${(r==null?void 0:r.message)||"未知错误"}`}});if(!t)return;const e=!!((i=t.result)!=null&&i.completed);E("pickPlaceStatus").textContent=e?"任务完成：红色方块已放回桌面原位":`任务未完成：停在 ${((s=t.result)==null?void 0:s.stage)||"未知"} 阶段`,Mn()}async function yM(){await de("双臂使能",()=>yt("/api/enable",{arm:"both",enabled:!0}),{confirm:"即将使能左右机械臂，确认急停可用且工作区安全。"}),ki()}async function MM(){await de("取消使能",()=>yt("/api/enable",{arm:"both",enabled:!1})),ki()}async function SM(){await de("复位急停",()=>yt("/api/reset_estop",{arm:"both"}),{confirm:"确认急停原因已经排除，然后复位左右臂急停状态。"}),ki()}async function EM(){const n=await de("双臂规划回零",()=>{var t;return yt("/api/home_zero",{arm:"both",avoid_platform:((t=E("avoidPlatform"))==null?void 0:t.checked)!==!1,collision_boxes:cf(),velocity_scaling:Number(E("velocityScaling").value),acceleration_scaling:Number(E("accelerationScaling").value),...ro()})},{confirm:"将先用 MoveIt 规划避障路径，再按规划轨迹让左右臂依次回零。",confirmTitle:"确认双臂规划回零"});so(Cr(n)),Mn()}async function TM(){await de("任务归零",()=>yt("/api/mujoco/task_reset",{}),{confirm:"将张开双夹爪、把转子恢复到近左手起始位置，并让双臂归零。",confirmTitle:"确认任务归零"})&&(await Promise.allSettled([Mn(),ki(),Ii(),Cc()]),ie("任务已恢复到初始状态","good",5e3))}async function wM(){if(await ls("将立即向左右臂发送急停命令。","紧急停止","立即停止"))try{await yt("/api/estop",{arm:"both",active:!0}),ie("已发送双臂急停命令","warn",5e3),ki()}catch(n){ie(`急停命令发送失败：${n.message}`,"bad",6e3)}}async function nf(){try{const n=await Ff();io(n.extrinsic||{}),E("extrinsicStatus").textContent=`已读取 ${n.path||"camera_extrinsic.json"}`}catch(n){E("extrinsicStatus").textContent=`读取失败：${n.message}`}}async function AM(){await de("保存相机外参",async()=>{const n=await yt("/api/camera_extrinsic",Qs());return io(n.extrinsic||Qs()),E("extrinsicStatus").textContent=`已保存 ${n.path||"camera_extrinsic.json"}`,n},{confirm:"该配置将写入相机外参文件，并影响视觉目标与点云坐标。"})}async function sf(){try{const n=await Kf(),t=n.config||{};E("vlmApiKey").value=t.api_key||"",E("vlmBaseUrl").value=t.base_url||"",E("vlmModel").value=t.model||"",E("vlmPrompt").value=t.prompt||"",E("vlmOffsetX").value=t.offset_x??0,E("vlmOffsetY").value=t.offset_y??0,E("vlmOffsetZ").value=t.offset_z??-.1,E("vlmStatus").textContent=`已读取 ${n.path||"vlm_config.json"}`}catch(n){E("vlmStatus").textContent=`读取失败：${n.message}`}}function Ph(){return{api_key:E("vlmApiKey").value.trim(),base_url:E("vlmBaseUrl").value.trim(),model:E("vlmModel").value.trim(),prompt:E("vlmPrompt").value,offset_x:Number(E("vlmOffsetX").value)||0,offset_y:Number(E("vlmOffsetY").value)||0,offset_z:Number(E("vlmOffsetZ").value)||0}}async function CM(){await de("保存 VLM 配置",async()=>{const n=await Jf(Ph()),t=n.config||Ph();return E("vlmApiKey").value=t.api_key||"",E("vlmBaseUrl").value=t.base_url||"",E("vlmModel").value=t.model||"",E("vlmPrompt").value=t.prompt||"",E("vlmOffsetX").value=t.offset_x??0,E("vlmOffsetY").value=t.offset_y??0,E("vlmOffsetZ").value=t.offset_z??-.1,E("vlmStatus").textContent=`已保存 ${n.path||"vlm_config.json"}`,n})}function Si(n,t){const e=E("vlmStageBox"),i=E("vlmStageText");e&&(e.style.display="flex",e.classList.remove("done","failed"),t==="done"&&e.classList.add("done"),t==="failed"&&e.classList.add("failed")),i&&(i.textContent=n)}function RM(){const n=E("vlmOutputBox");n&&(n.style.display="none");for(const t of["vlmOutVlm","vlmOutBbox","vlmOutPixel","vlmOutDepth","vlmOutWorld","vlmOutPick"]){const e=E(t);e&&(e.style.display="none")}}function Ei(n,t,e){const i=E("vlmOutputBox");i&&(i.style.display="block");const s=E(n);s&&(s.style.display="flex");const r=E(t);r&&(r.textContent=e)}async function PM(){var e,i,s,r,a,o;const n=E("vlmGraspPrompt").value.trim();if(!n){E("vlmGraspStatus").textContent="请输入指令";return}const t=E("vlmConfirmButton");t&&(t.disabled=!0),RM(),E("vlmGraspStatus").textContent="",Si("获取相机画面中...");try{const c=(await fetch("/api/vlm_grasp_stream",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify({prompt:n,execute:!0,arm:b.arm,place_in_plate:/放(?:到|入|进)?[^，。]{0,8}(?:盘|plate)/i.test(n),place_on_table:/放(?:到|在)?[^，。]{0,8}(?:桌面|桌上|table)/i.test(n)})})).body.getReader(),u=new TextDecoder;let h="",d=null;for(;;){const{done:p,value:g}=await c.read();if(p)break;h+=u.decode(g,{stream:!0});const _=h.split(`
`);h=_.pop()||"";for(const m of _){if(!m.startsWith("data: "))continue;const f=JSON.parse(m.slice(6)),P=f.stage;if(f.status&&Si(f.status),P==="vlm_done")Ei("vlmOutVlm","vlmOutVlmVal",f.label||"-"),Ei("vlmOutBbox","vlmOutBboxVal",(f.bbox_norm||f.bbox||[]).map(L=>Number(L).toFixed(3)).join(", "));else if(P==="depth_done"){const L=f.pixel||{};Ei("vlmOutPixel","vlmOutPixelVal",`(${L.u}, ${L.v})`);const w=f.depth||{};Ei("vlmOutDepth","vlmOutDepthVal",`${Number(w.value_m||0).toFixed(3)} m`);const N=f.world||{};Ei("vlmOutWorld","vlmOutWorldVal",`X:${N.x}  Y:${N.y}  Z:${N.z}`)}else if(P==="pick_done"){const L=f.pick||{},w=L.completed===!0,N=L.error||((i=(e=L.grasp_debug)==null?void 0:e.lift)==null?void 0:i.reason)||((r=(s=L.grasp_debug)==null?void 0:s.close)==null?void 0:r.reason);Ei("vlmOutPick","vlmOutPickVal",w?"成功":N||"未完成"),Si(w?"抓取完成":"抓取未完成",w?"done":"failed")}else if(P==="place")Ei("vlmOutPick","vlmOutPickVal","抓取成功，正在放入盘子"),Si(f.status||"正在移动到盘子...");else if(P==="place_done"){const L=((a=f.place)==null?void 0:a.completed)===!0,w=(o=f.place)==null?void 0:o.error;Ei("vlmOutPick","vlmOutPickVal",L?"已放入盘子":w||"放置未完成"),Si(L?"抓取并放置完成":"放置未完成",L?"done":"failed")}else P==="error"?(d=f.error,Si("执行失败","failed")):P==="done"&&(!d&&!E("vlmOutPick")||E("vlmOutPick").style.display==="none")&&(d||Si("定位完成","done"))}}E("vlmGraspStatus").textContent=d||"执行完成"}catch(l){Si("执行失败","failed"),E("vlmGraspStatus").textContent=`错误: ${l.message}`}finally{t&&(t.disabled=!1)}}async function rf(){try{const n=await Zf();b.posesLib=n.poses_lib&&n.poses_lib.poses||[],Lc(),E("posesStatus").textContent=`已读取 ${n.path||"poses.json"} (${b.posesLib.length} 个姿态)`}catch(n){E("posesStatus").textContent=`读取失败：${n.message}`}}function Lc(){const n=E("posesList");if(!b.posesLib||b.posesLib.length===0){n.innerHTML='<div class="empty-hint">暂无姿态，点「记录当前位姿」添加</div>';return}n.innerHTML=b.posesLib.map((t,e)=>`
    <div class="library-item">
      <div class="library-item-info">
        <span class="library-item-name">${Jt(t.name||t.id||`姿态${e+1}`)}</span>
        <span class="library-item-meta">${Jt(t.id||"")} · (${kt(t.x,3)}, ${kt(t.y,3)}, ${kt(t.z,3)})</span>
      </div>
      <div class="library-item-actions">
        <button class="icon-button" data-pose-preview="${e}" type="button" title="预览(移动到此姿态)"><i data-lucide="play"></i></button>
        <button class="icon-button" data-pose-delete="${e}" type="button" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join(""),os()}async function LM(){const n=E("poseEditor");n.style.display==="none"&&(n.style.display="flex"),E("poseX").value="读取中...",E("poseY").value="",E("poseZ").value="";try{const i=((await nl(b.arm)).links||[]).find(s=>s.name&&s.name.includes("TCP"));if(!i||!i.position){ie("无法读取当前 TCP 坐标");return}E("poseX").value=Number(i.position[0]).toFixed(3),E("poseY").value=Number(i.position[1]).toFixed(3),E("poseZ").value=Number(i.position[2]).toFixed(3),E("poseOrientationMode").value=b.graspPoseMode||"vertical",ie(`已读取实时坐标: (${Number(i.position[0]).toFixed(3)}, ${Number(i.position[1]).toFixed(3)}, ${Number(i.position[2]).toFixed(3)})`)}catch(t){ie(`读取坐标失败: ${t.message}`)}}async function NM(){const n={id:E("poseId").value.trim(),name:E("poseNameInput").value.trim(),description:E("poseDesc").value.trim(),x:Number(E("poseX").value),y:Number(E("poseY").value),z:Number(E("poseZ").value),grasp_orientation_mode:E("poseOrientationMode").value,arm:b.arm};if(!n.id){ie("请填写 ID (英文)");return}if(!Number.isFinite(n.x)||!Number.isFinite(n.y)||!Number.isFinite(n.z)){ie("坐标无效");return}b.posesLib||(b.posesLib=[]);const t=b.posesLib.findIndex(e=>e.id===n.id);if(t>=0){if(!await ls(`ID「${n.id}」已存在，是否覆盖？`,"确认覆盖"))return;b.posesLib[t]=n}else b.posesLib.push(n);await de("保存姿态库",async()=>{const e=await Qf({version:1,poses:b.posesLib});return b.posesLib=e.poses_lib&&e.poses_lib.poses||[],Lc(),E("posesStatus").textContent=`已保存 (${b.posesLib.length} 个姿态)`,e}),E("poseId").value="",E("poseNameInput").value="",E("poseDesc").value=""}async function DM(n){const t=b.posesLib[n];t&&(us({x:t.x,y:t.y,z:t.z}),t.grasp_orientation_mode&&(b.graspPoseMode=t.grasp_orientation_mode),await de("预览姿态",async()=>(await yt("/api/plan",fn()),yt("/api/execute",{arm:t.arm||"right"}))),ie(`已移动到: ${t.name||t.id}`))}async function IM(n){const t=b.posesLib[n];t&&await ls(`删除姿态「${t.name||t.id}」？`,"确认删除")&&await de("删除姿态",async()=>{const e=await tp({id:t.id,name:t.name});return b.posesLib=e.poses_lib&&e.poses_lib.poses||[],Lc(),E("posesStatus").textContent=`已删除 (${b.posesLib.length} 个姿态)`,e})}async function UM(n){var i,s;const t=(i=n.target.closest("[data-pose-preview]"))==null?void 0:i.dataset.posePreview,e=(s=n.target.closest("[data-pose-delete]"))==null?void 0:s.dataset.poseDelete;if(t!==void 0){await DM(Number(t));return}if(e!==void 0){await IM(Number(e));return}}async function Nc(){try{const n=await at.loadScenePointCloud();tn("cloudState",`${n.count} 点`,"good")}catch{tn("cloudState","暂无点云","neutral")}}function kM(){b.cloudVisible=!b.cloudVisible,localStorage.setItem("unoarm.cloudVisible",String(b.cloudVisible)),at.setCloudVisible(b.cloudVisible),uf()}async function FM(){await de("保存场景点云",async()=>{const n=await yt("/api/scene_pointcloud/save",{});return await Nc(),n})}async function OM(){await de("按当前外参重建点云",async()=>{const n=await yt("/api/camera_extrinsic",Qs());io(n.extrinsic||Qs());const t=await yt("/api/scene_pointcloud/save",{});return await Nc(),E("extrinsicStatus").textContent=`已保存 ${n.path||"camera_extrinsic.json"}`,t},{confirm:"将保存当前相机外参，并让视觉服务按该外参重新保存静态场景点云。",confirmTitle:"确认重建点云"})}async function af(n){try{const e=(await jf()).platform_obstacle;return KM(e)?(at==null||at.setPlatformObstacle(e),b.platformObstacleVisible=!0,b.platformObstacleApplied=!0,E("platformState").textContent="避障区已显示",Qi(),n&&ie("已显示平台避障区"),e):(at==null||at.setPlatformObstacle(null),b.platformObstacleVisible=!1,b.platformObstacleApplied=!1,E("platformState").textContent="避障区未加载",Qi(),n&&ie("当前没有已应用的避障区","warn"),null)}catch(t){return E("platformState").textContent=`读取失败：${t.message}`,Qi(),null}}async function BM(){if(b.platformObstacleVisible){at==null||at.setPlatformObstacle(null),b.platformObstacleVisible=!1,E("platformState").textContent=b.platformObstacleApplied?"避障区已隐藏，规划场景仍保留":"避障区已隐藏",Qi();return}await af(!0)}async function zM(){if(b.platformObstacleApplied){await de("取消平台避障区",async()=>{const n=await yt("/api/platform_obstacle/clear",{});return at==null||at.setPlatformObstacle(null),b.platformObstacleVisible=!1,b.platformObstacleApplied=!1,E("platformState").textContent="避障区已从规划场景移除",Qi(),n},{confirm:"将从 MoveIt 规划场景移除当前平台避障区，后续规划不再使用该避障区。",confirmTitle:"确认取消避障区"});return}await de("应用平台避障区",async()=>{const n=await yt("/api/platform_obstacle/apply",{});return at.setPlatformObstacle(n.platform_obstacle),b.platformObstacleVisible=!0,b.platformObstacleApplied=!0,E("platformState").textContent="避障区已应用到规划场景",Qi(),n})}async function VM(n=!1){try{const t=await qf();b.workspaceBoundsByArm={right:Zi(t.right,"right"),left:Zi(t.left,"left")},Dc(),at==null||at.setWorkspaceBounds(is()),lf("已读取"),n&&ie("已读取工作区边界")}catch(t){E("workspaceBoundsState").textContent=`读取失败：${t.message}`}}function HM(n=b.arm){return n==="left"?{enabled:!1,min:[-.35,-.65,.2],max:[.8,.35,1.55]}:{enabled:!1,min:[-.8,-.65,.2],max:[.35,.35,1.55]}}function is(){return Zi(b.workspaceBoundsByArm[b.arm],b.arm)}function Zi(n={},t=b.arm){var s;const e=((s=b.workspaceBoundsByArm)==null?void 0:s[t])||HM(t),i=(r,a)=>{const o=Array.isArray(n[r])?n[r]:a;return[0,1,2].map(l=>{const c=Number(o==null?void 0:o[l]);return Number.isFinite(c)?c:a[l]})};return{enabled:!!n.enabled,min:i("min",e.min),max:i("max",e.max)}}function Dc(){const n=is(),t=E("workspaceBoundsFields");t&&(t.innerHTML=`
    <label class="workspace-enable"><input id="workspaceBoundsEnabled" type="checkbox" ${n.enabled?"checked":""}><span>启用${en()}边界拒绝</span></label>
    <div class="compact-fields workspace-bounds-grid">
      ${Lh("min",n.min,["X min","Y min","Z min"])}
      ${Lh("max",n.max,["X max","Y max","Z max"])}
    </div>
  `)}function Lh(n,t,e){return e.map((i,s)=>`
    <label>${i}<input data-workspace-bound="${n}" data-workspace-axis="${s}"
      type="number" step="0.001" value="${Number(t[s]??0)}" /><span>m</span></label>
  `).join("")}function of(){var t;const n=e=>[0,1,2].map(i=>{var r,a;const s=Number((a=(r=E("workspaceBoundsFields"))==null?void 0:r.querySelector(`[data-workspace-bound="${e}"][data-workspace-axis="${i}"]`))==null?void 0:a.value);return Number.isFinite(s)?s:0});return{enabled:((t=E("workspaceBoundsEnabled"))==null?void 0:t.checked)===!0,min:n("min"),max:n("max")}}function Nh(){b.workspaceBoundsByArm[b.arm]=Zi(of(),b.arm),at==null||at.setWorkspaceBounds(is()),E("workspaceBoundsState").textContent=`${en()}工作区边界已修改，保存后生效`}async function GM(){b.workspaceBoundsByArm[b.arm]=Zi(of(),b.arm),await de("保存工作区边界",async()=>{const n=await Yf({arm:b.arm,...is()});return b.workspaceBoundsByArm={right:Zi(n.right||b.workspaceBoundsByArm.right,"right"),left:Zi(n.left||b.workspaceBoundsByArm.left,"left")},Dc(),at==null||at.setWorkspaceBounds(is()),lf("已保存"),n})}function lf(n=""){const e=is().enabled?`${n?`${n} · `:""}${en()}工作区已启用`:`${n?`${n} · `:""}${en()}工作区未启用`;E("workspaceBoundsState").textContent=e}async function WM(n=!1){var t,e;try{const i=await Xf();b.manualCollision.boxes=i.boxes||[],b.manualCollision.applied=!!((t=i.summary)!=null&&t.applied&&!((e=i.summary)!=null&&e.cleared)),hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent=b.manualCollision.boxes.length?`已读取 ${b.manualCollision.boxes.length} 个碰撞箱`:"尚未添加手动碰撞箱",n&&ie("已读取手动碰撞箱")}catch(i){E("manualCollisionState").textContent=`读取失败：${i.message}`,qn()}}function $M(){const n=fn(),t=b.manualCollision.boxes.length+1;b.manualCollision.boxes.push({id:`box_${t}`,name:`碰撞箱 ${t}`,enabled:!0,center:[Number(n.x),Number(n.y),Number(n.z)],dimensions:[.18,.18,.18],rpy:[0,0,0]}),b.manualCollision.applied=!1,hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent="已添加碰撞箱，下一次规划/控制自动生效"}function hs(){const n=E("manualCollisionFields"),t=b.manualCollision.boxes||[];if(!t.length){Hn("manualCollisionFields","没有手动碰撞箱"),Vn();return}n.innerHTML=t.map((e,i)=>`
    <div class="collision-box-row" data-collision-index="${i}">
      <div class="collision-box-header">
        <input data-collision-field="name" type="text" value="${Jt(e.name||`碰撞箱 ${i+1}`)}" aria-label="碰撞箱名称">
        <label><input data-collision-field="enabled" type="checkbox" ${e.enabled===!1?"":"checked"}><span>启用</span></label>
        <button class="icon-button" type="button" data-collision-delete="${i}" aria-label="删除碰撞箱" title="删除碰撞箱">
          <i data-lucide="trash-2"></i>
        </button>
      </div>
      <div class="compact-fields collision-fields">
        ${el(i,e,"center",["X","Y","Z"],"m",.001)}
        ${el(i,e,"dimensions",["长","宽","高"],"m",.001)}
        ${el(i,e,"rpy",["Roll","Pitch","Yaw"],"rad",.001)}
      </div>
    </div>
  `).join(""),Vn(),os()}function el(n,t,e,i,s,r){const a=Array.isArray(t[e])?t[e]:[0,0,0];return i.map((o,l)=>`
    <label>${o}<input data-collision-vector="${e}" data-collision-axis="${l}" data-collision-index="${n}"
      type="number" step="${r}" value="${Number(a[l]??0)}" /><span>${s}</span></label>
  `).join("")}function Dh(){b.manualCollision.boxes=rr(),b.manualCollision.applied=!1,at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),qn(),E("manualCollisionState").textContent="配置已修改，下一次规划/控制自动同步到 MoveIt"}function jM(n){const t=n.target.closest("[data-collision-delete]");t&&(b.manualCollision.boxes=rr(),b.manualCollision.boxes.splice(Number(t.dataset.collisionDelete),1),b.manualCollision.applied=!1,hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent="已删除碰撞箱，下一次规划/控制自动更新 MoveIt")}function rr(){return Array.from(document.querySelectorAll(".collision-box-row")).map((n,t)=>{var r,a,o;const e=Number(n.dataset.collisionIndex),i=b.manualCollision.boxes[e]||{},s=l=>[0,1,2].map(c=>{const u=n.querySelector(`[data-collision-vector="${l}"][data-collision-axis="${c}"]`),h=Number(u==null?void 0:u.value);return Number.isFinite(h)?h:0});return{id:i.id||`box_${t+1}`,name:((a=(r=n.querySelector('[data-collision-field="name"]'))==null?void 0:r.value)==null?void 0:a.trim())||`碰撞箱 ${t+1}`,enabled:((o=n.querySelector('[data-collision-field="enabled"]'))==null?void 0:o.checked)!==!1,center:s("center"),dimensions:s("dimensions").map(l=>Math.max(.005,Math.abs(l))),rpy:s("rpy")}})}function cf(){return document.querySelector(".collision-box-row")&&(b.manualCollision.boxes=rr()),b.manualCollision.boxes||[]}async function XM(){b.manualCollision.boxes=rr(),await de("保存手动碰撞箱",async()=>{const n=await ep({boxes:b.manualCollision.boxes});return b.manualCollision.boxes=n.boxes||[],b.manualCollision.applied=!1,hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent=`已保存 ${b.manualCollision.boxes.length} 个碰撞箱，下一次规划/控制自动生效`,n})}async function qM(){b.manualCollision.boxes=rr(),await de("应用手动碰撞箱",async()=>{const n=await np({boxes:b.manualCollision.boxes}),t=n.manual_collision||{};return b.manualCollision.boxes=t.boxes||b.manualCollision.boxes,b.manualCollision.applied=Number(t.count||0)>0,hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent=`已应用 ${t.count??0} 个启用碰撞箱到 MoveIt`,n})}async function YM(){await de("清除手动碰撞箱",async()=>{const n=await ip(),t=n.manual_collision||{};return b.manualCollision.boxes=t.boxes||b.manualCollision.boxes,b.manualCollision.applied=!1,hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent="已从 MoveIt 移除手动碰撞箱，网页配置仍保留",n},{confirm:"将从 MoveIt PlanningScene 移除手动碰撞箱，但保留网页配置。",confirmTitle:"确认清除手动碰撞箱"})}function qn(){const t=(b.manualCollision.boxes||[]).filter(e=>e.enabled!==!1).length;E("applyCollisionBoxesButton").querySelector("span").textContent=b.manualCollision.applied?`已应用 ${t} 个`:"提前应用",$n("applyCollisionBoxesButton",b.manualCollision.applied?"active-good":t?"ready-warning":"inactive",b.manualCollision.applied),$n("clearCollisionBoxesButton",b.manualCollision.applied?"ready-warning":"inactive",!1)}function so(n){n&&(b.manualCollision.boxes=n.boxes||b.manualCollision.boxes,b.manualCollision.applied=!!(n.applied&&Number(n.count||0)>0),hs(),at==null||at.setManualCollisionBoxes(b.manualCollision.boxes),Vn(),qn(),E("manualCollisionState").textContent=b.manualCollision.applied?`已自动同步 ${n.count??0} 个碰撞箱到 MoveIt`:"当前没有启用的碰撞箱")}function Cr(n){if(!n||typeof n!="object")return null;if(n.manual_collision)return n.manual_collision;for(const t of["plan","result","results","approach","descend","lift"]){const e=Cr(n[t]);if(e)return e}for(const t of Object.values(n))if(t&&typeof t=="object"&&t!==n){const e=Cr(t);if(e)return e}return null}function Ql({anyConnected:n,anyEnabled:t,allEnabled:e,anyEstop:i}){$n("enableButton",e?"active-good":"ready-good",e),E("enableButton").querySelector("span").textContent=e?"已全部使能":"双臂使能",$n("disableButton",t?"ready-danger":"inactive",!1),E("disableButton").querySelector("span").textContent=t?"取消使能":"未使能",$n("resetEstopButton",i?"ready-warning":"inactive",i),E("resetEstopButton").querySelector("span").textContent=i?"复位急停":"急停正常",$n("homeButton",n&&e&&!i?"ready-neutral":"inactive",!1),E("homeButton").querySelector("span").textContent="双臂规划回零",$n("taskResetButton",n&&e&&!i?"ready-neutral":"inactive",!1),E("taskResetButton").querySelector("span").textContent="任务归零"}function uf(){const n=E("toggleCloudButton");n.querySelector("span").textContent=b.cloudVisible?"隐藏点云":"显示点云",$n("toggleCloudButton",b.cloudVisible?"active-neutral":"inactive",b.cloudVisible)}function Qi(){const n=E("loadPlatformButton");n.querySelector("span").textContent=b.platformObstacleVisible?"隐藏避障区":"显示避障区",$n("loadPlatformButton",b.platformObstacleVisible?"active-neutral":"ready-neutral",b.platformObstacleVisible);const t=E("applyPlatformButton");t.querySelector("span").textContent=b.platformObstacleApplied?"取消避障区":"应用避障区",$n("applyPlatformButton",b.platformObstacleApplied?"active-warning":"ready-warning",b.platformObstacleApplied)}function $n(n,t,e){const i=E(n);i.dataset.mode=t,i.setAttribute("aria-pressed",String(!!e))}function KM(n){return n?Array.isArray(n)?n.length>0:Array.isArray(n.platform_obstacles)?n.platform_obstacles.length>0:!!(n.center||n.dimensions):!1}async function Ic(){var n;E("jointConfigStatus").textContent=`正在读取${en()}配置...`;try{const t=await il(b.arm);b.jointConfig=t,Uc(t),hf(t),E("jointConfigStatus").textContent=`已读取 ${((n=t.files)==null?void 0:n.bridge)||en()}`}catch(t){E("jointConfigStatus").textContent=`读取失败：${t.message}`}}async function JM(){(await Promise.allSettled([il("right"),il("left")])).forEach(t=>{t.status==="fulfilled"&&Uc(t.value)})}function Uc(n){const t=(n==null?void 0:n.arm)==="left"?"left":"right",e=(n==null?void 0:n.joint_names)||[],i={};e.forEach((s,r)=>{var o;const a=(o=n==null?void 0:n.joint_limits_rad)==null?void 0:o[r];i[s]=a!=null&&a.enabled&&Number.isFinite(Number(a.lower))&&Number.isFinite(Number(a.upper))?{lower:Number(a.lower),upper:Number(a.upper)}:{lower:-1e6,upper:1e6}}),b.jointLimitsByArm[t]=i,at==null||at.setJointLimits({...b.jointLimitsByArm.right,...b.jointLimitsByArm.left})}function hf(n){const t=n.joint_names||[],e=E("jointConfigFields");e.innerHTML=`
    <div class="joint-config-header"><span>关节</span><span>零位</span><span>限位</span><span>下限</span><span>上限</span></div>
    ${t.map((i,s)=>{var r,a,o,l;return`
      <div class="joint-config-row">
        <b>J${s+1}</b>
        <input id="cfg_offset_${s}" type="number" step="1" value="${Number(((r=n.zero_offsets)==null?void 0:r[s])||0)}" aria-label="${Jt(i)}零位">
        <input id="cfg_limit_${s}" type="checkbox" ${(a=n.limit_enabled)!=null&&a[s]?"checked":""} aria-label="${Jt(i)}启用限位">
        <input id="cfg_min_${s}" type="number" step="1" value="${Number(((o=n.raw_limit_a)==null?void 0:o[s])||0)}" aria-label="${Jt(i)}下限">
        <input id="cfg_max_${s}" type="number" step="1" value="${Number(((l=n.raw_limit_b)==null?void 0:l[s])||0)}" aria-label="${Jt(i)}上限">
      </div>
    `}).join("")}
  `}async function ZM(){var e;const n=((e=b.jointConfig)==null?void 0:e.joint_names)||[];if(!n.length){ie("请先读取关节配置","warn");return}const t={arm:b.arm,zero_offsets:[],limit_enabled:[],raw_limit_a:[],raw_limit_b:[]};n.forEach((i,s)=>{t.zero_offsets.push(Number.parseInt(E(`cfg_offset_${s}`).value,10)),t.limit_enabled.push(E(`cfg_limit_${s}`).checked),t.raw_limit_a.push(Number.parseInt(E(`cfg_min_${s}`).value,10)),t.raw_limit_b.push(Number.parseInt(E(`cfg_max_${s}`).value,10))}),await de("保存关节配置",async()=>{const i=await yt("/api/joint_config",t);return b.jointConfig=i,Uc(i),hf(i),i},{confirm:`将覆盖${en()}的编码器零位与原始限位配置。该操作会影响实际运动范围。`,confirmTitle:"确认写入关节配置"})}async function kc(){E("kinematicsStatus").textContent="正在读取求解器配置...";try{const n=await Of();b.kinematics=n,df(n)}catch(n){E("kinematicsStatus").textContent=`读取失败：${n.message}`}}function Ih(n,t){if(!n)return"--";const e=t.find(i=>i.id===n.solver_id||i.plugin===n.solver_plugin);return e?e.label:n.solver_plugin||n.solver_id||"--"}function df(n){var o,l;const t=n.options||[],e=E("ikSolverSelect"),i=n.active_solver_id||"kdl",s=t.some(c=>c.id===i);if(e.innerHTML=[...!s&&i?[{id:i,label:i==="mixed"?"左右臂配置不一致":i,available:!1,plugin:""}]:[],...t].map(c=>`
    <option value="${Jt(c.id)}" ${c.available?"":"disabled"} ${c.id===i?"selected":""}>
      ${Jt(c.label)}${c.available?"":"（未安装）"}
    </option>
  `).join(""),i!=="mixed"&&t.some(c=>c.id===i&&c.available))e.value=i;else{const c=t.find(u=>u.available);c&&(e.value=c.id)}const r=(o=n.groups)==null?void 0:o.arm,a=(l=n.groups)==null?void 0:l.left_arm;E("ikRightSolver").textContent=`右臂 ${Ih(r,t)}`,E("ikLeftSolver").textContent=`左臂 ${Ih(a,t)}`,E("kinematicsStatus").textContent=`配置文件：${n.path||"--"}。保存后需重启 MoveIt 才会生效。`}async function QM(){var i,s;const n=E("ikSolverSelect").value;if(!n){ie("请选择可用的 IK 求解器","warn");return}const t=(s=(i=b.kinematics)==null?void 0:i.options)==null?void 0:s.find(r=>r.id===n),e=(t==null?void 0:t.label)||n;await de("保存 IK 求解器",async()=>{const r=await yt("/api/kinematics",{solver_id:n,target:"both"});return b.kinematics=r,df(r),r},{confirm:`将左右臂 MoveIt IK 求解器配置切换为 ${e}。已运行的 MoveIt 不会动态切换，保存后需要重启 MoveIt。`,confirmTitle:"确认切换 IK 求解器",success:`IK 求解器已写入：${e}，重启 MoveIt 后生效`})}function tS(){try{return ss(JSON.parse(localStorage.getItem(kd)||"{}"))}catch{return{...Ln}}}function ss(n={}){return{planner_id:String(n.planner_id||Ln.planner_id),planning_time:ai(n.planning_time,Ln.planning_time),attempts:Math.max(1,Math.round(ai(n.attempts,Ln.attempts))),ik_timeout:ai(n.ik_timeout,Ln.ik_timeout)}}function ai(n,t){const e=Number(n);return Number.isFinite(e)?e:t}async function ar(){var n;E("omplConfigStatus").textContent="正在读取 OMPL 配置...";try{const t=await Bf();b.ompl.options=t;const e=((n=t.recommended)==null?void 0:n.config)||t.defaults||Ln,i=new Set((t.planners||[]).map(r=>r.planner_id)),s=ss(b.ompl.config);b.ompl.config=i.has(s.planner_id)?s:ss(e),ao(),eS()}catch(t){E("omplConfigStatus").textContent=`读取失败：${t.message}`,Hn("omplPlannerDetails","OMPL 配置不可用")}}function eS(){var a;const n=b.ompl.options||{},t=n.planners||[],e=n.presets||[],i=E("omplPresetSelect");i.innerHTML=e.map(o=>`
    <option value="${Jt(o.id)}">
      ${Jt(o.custom?`自定义 · ${o.label}`:o.label)}
    </option>
  `).join("");const s=new Set(e.map(o=>o.id));b.ompl.selectedPresetId&&s.has(b.ompl.selectedPresetId)?i.value=b.ompl.selectedPresetId:(a=n.recommended)!=null&&a.id&&(i.value=n.recommended.id);const r=E("omplPlannerSelect");r.innerHTML=t.map(o=>`
    <option value="${Jt(o.planner_id)}">
      ${Jt(o.label||o.planner_id)}
    </option>
  `).join(""),t.some(o=>o.planner_id===b.ompl.config.planner_id)&&(r.value=b.ompl.config.planner_id),ff(b.ompl.config),rS(n.recommended),Oc(),E("omplConfigStatus").textContent=`运行时生效 · 配置文件：${n.path||"--"}`}function ff(n){const t=ss(n);E("omplPlannerSelect").value=t.planner_id,E("omplPlanningTime").value=t.planning_time,E("omplAttempts").value=t.attempts,E("omplIkTimeout").value=t.ik_timeout}function ro(){const n=E("omplPlannerSelect");if(!n)return{...b.ompl.config};const t=ss({planner_id:n.value||b.ompl.config.planner_id,planning_time:E("omplPlanningTime").value,attempts:E("omplAttempts").value,ik_timeout:E("omplIkTimeout").value});return b.ompl.config=t,t}function ao(){localStorage.setItem(kd,JSON.stringify(b.ompl.config))}function wa(){b.ompl.config=ro(),ao(),Oc();const n=Fc(b.ompl.config.planner_id);E("omplConfigStatus").textContent=`下一次规划使用 ${(n==null?void 0:n.label)||b.ompl.config.planner_id} · ${b.ompl.config.planning_time}s / ${b.ompl.config.attempts} 次`}function nS(){var e,i;const n=E("omplPresetSelect").value,t=(((e=b.ompl.options)==null?void 0:e.presets)||[]).find(s=>s.id===n)||((i=b.ompl.options)==null?void 0:i.recommended);b.ompl.selectedPresetId=(t==null?void 0:t.id)||null,b.ompl.config=ss((t==null?void 0:t.config)||Ln),ff(b.ompl.config),ao(),Oc(),ie(`已应用 OMPL 配置：${(t==null?void 0:t.label)||"推荐配置"}`),E("omplConfigStatus").textContent=`已应用 ${(t==null?void 0:t.label)||"推荐配置"}，下一次规划生效`}async function iS(){const n=pf("omplPresetName"),t=ro(),e=await de("保存规划预设",()=>Vh({name:n,source:"ompl",config:t}),{quiet:!0,success:"规划预设已保存"});if(!e)return;const i=mf(e.presets,n,t);b.ompl.selectedPresetId=(i==null?void 0:i.id)||null,await ar(),E("omplConfigStatus").textContent=`已保存到预设库：${(i==null?void 0:i.label)||n}`,ie(`已保存规划预设：${(i==null?void 0:i.label)||n}`)}async function sS(){var i;const n=E("omplPresetSelect").value,t=(((i=b.ompl.options)==null?void 0:i.presets)||[]).find(s=>s.id===n);if(!(t!=null&&t.custom)){ie("只能删除自定义预设","warn");return}!await ls(`删除自定义预设「${t.label}」？`,"删除规划预设","删除")||!await de("删除规划预设",()=>zf({id:t.id}),{quiet:!0,success:"规划预设已删除"})||(b.ompl.selectedPresetId=null,await ar(),ie(`已删除规划预设：${t.label}`))}function pf(n){const t=E(n),e=String((t==null?void 0:t.value)||"").trim();if(e)return e;const i=`规划预设 ${new Date().toLocaleString("zh-CN",{hour12:!1})}`;return t&&(t.value=i),i}function mf(n=[],t,e){const i=n.find(s=>s.name===t||s.label===t);return i||n.find(s=>{const r=s.config||{};return r.planner_id===e.planner_id&&Number(r.planning_time)===Number(e.planning_time)&&Number(r.attempts)===Number(e.attempts)&&Number(r.ik_timeout)===Number(e.ik_timeout)})||null}function Fc(n){var t;return(((t=b.ompl.options)==null?void 0:t.planners)||[]).find(e=>e.planner_id===n)||null}function rS(n){const t=E("omplRecommended"),e=(n==null?void 0:n.config)||Ln,i=Fc(e.planner_id);t.innerHTML=`
    <div class="ompl-recommendation-main">
      <strong>${Jt((n==null?void 0:n.label)||"推荐配置")}</strong>
      <span>${Jt((i==null?void 0:i.label)||e.planner_id)}</span>
    </div>
    <div class="ompl-config-chips">
      <span>规划 ${kt(e.planning_time,1)}s</span>
      <span>尝试 ${Number(e.attempts)} 次</span>
      <span>IK ${kt(e.ik_timeout,1)}s</span>
    </div>
    <p>${Jt((n==null?void 0:n.description)||"用于快速调试的默认配置。")}</p>
  `}function Oc(){const n=Fc(b.ompl.config.planner_id),t=E("omplPlannerDetails");if(!n){Hn("omplPlannerDetails","请选择规划器");return}const e=Object.entries(n.config||{});t.innerHTML=`
    <div class="ompl-detail-row"><b>planner_id</b><span>${Jt(n.planner_id)}</span></div>
    <div class="ompl-detail-row"><b>type</b><span>${Jt(n.type||"--")}</span></div>
    <div class="ompl-detail-row"><b>groups</b><span>${Jt((n.groups||[]).join(", ")||"--")}</span></div>
    ${e.map(([i,s])=>`
      <div class="ompl-detail-row"><b>${Jt(i)}</b><span>${Jt(String(s))}</span></div>
    `).join("")}
  `}async function Ir(n=!1){const t=E("benchmarkStatus");t&&b.activePanel==="benchmark"&&(t.textContent="正在读取跑分配置...");try{const e=await Vf(b.arm);b.benchmark.options=e,aS(e),Vn(),gf(),Oi(),ds(),fi(),t&&n&&(t.textContent="跑分配置已读取")}catch(e){t&&(t.textContent=`跑分配置读取失败：${e.message}`),Hn("benchmarkPlanners","规划器配置不可用")}}function aS(n={}){const t=Number(n.max_samples||120);for(const i of["benchmarkBoxCount","benchmarkEdgeCount","benchmarkRandomCount"]){const s=E(i);s&&Number.isFinite(t)&&(s.max=String(t))}if(b.benchmark.defaultsApplied)return;const e=n.defaults||{};Ti("benchmarkBoxCount",e.box_top_count),Ti("benchmarkBoxOffsetCm",e.box_top_offset_cm),Ti("benchmarkEdgeCount",e.edge_count),Ti("benchmarkEdgeDistanceCm",e.edge_distance_cm),Ti("benchmarkRandomCount",e.random_count),Ti("benchmarkPlanningTime",e.planning_time),Ti("benchmarkAttempts",e.attempts),Ti("benchmarkIkTimeout",e.ik_timeout),b.benchmark.defaultsApplied=!0}function Ti(n,t){const e=E(n),i=Number(t);e&&Number.isFinite(i)&&(e.value=String(t))}function Vn(){var a,o;const n=E("benchmarkCollisionBox");if(!n)return;const t=n.value,e=(b.manualCollision.boxes||[]).filter(l=>l.enabled!==!1),i=Array.isArray((a=b.benchmark.options)==null?void 0:a.collision_boxes)?b.benchmark.options.collision_boxes:[],s=e.length?e:i;if(!s.length){n.innerHTML='<option value="">没有启用的碰撞箱</option>',n.disabled=!0;const l=E("benchmarkBoxCount");l&&(l.value="0");return}n.disabled=!1,n.innerHTML=s.map((l,c)=>{const u=l.id||l.name||`box_${c+1}`,h=l.name||l.id||`碰撞箱 ${c+1}`;return`<option value="${Jt(u)}">${Jt(h)}</option>`}).join("");const r=new Set(s.map((l,c)=>String(l.id||l.name||`box_${c+1}`)));n.value=r.has(t)?t:((o=n.options[0])==null?void 0:o.value)||""}function gf(){var r,a;const n=E("benchmarkPlanners");if(!n)return;const t=((r=b.benchmark.options)==null?void 0:r.planners)||((a=b.ompl.options)==null?void 0:a.planners)||[];if(!t.length){Hn("benchmarkPlanners","未读取到适用于当前手臂的规划器");return}let e=new Set(b.benchmark.selectedPlanners);e.size||(e=new Set([b.ompl.config.planner_id]));const i=t.filter(o=>e.has(o.planner_id)).map(o=>o.planner_id);b.benchmark.selectedPlanners=i.length?i:[t[0].planner_id];const s=new Set(b.benchmark.selectedPlanners);n.innerHTML=t.map(o=>`
    <label class="planner-option">
      <input type="checkbox" value="${Jt(o.planner_id)}" ${s.has(o.planner_id)?"checked":""}>
      <span>
        <b>${Jt(o.label||o.planner_id)}</b>
        <small>${Jt(o.planner_id)}</small>
      </span>
    </label>
  `).join("")}function Ur(){var i,s,r,a;const n=E("benchmarkPlanners"),t=Array.from((n==null?void 0:n.querySelectorAll('input[type="checkbox"]'))||[]);if(t.length)return b.benchmark.selectedPlanners=t.filter(o=>o.checked).map(o=>o.value),[...b.benchmark.selectedPlanners];if(b.benchmark.selectedPlanners.length)return[...b.benchmark.selectedPlanners];const e=((i=b.ompl.config)==null?void 0:i.planner_id)||((a=(r=(s=b.benchmark.options)==null?void 0:s.planners)==null?void 0:r[0])==null?void 0:a.planner_id);return e?[e]:[]}function _f(){var n,t;return Ur().length||((t=(n=b.benchmark.result)==null?void 0:n.planners)==null?void 0:t.length)||0}function Bc(){var r,a,o,l,c,u,h;const n=fn();document.querySelector(".collision-box-row")&&(b.manualCollision.boxes=rr());const t=(r=E("benchmarkSeed"))==null?void 0:r.value,e=Number(t),i={...n,avoid_collisions:((a=E("avoidPlatform"))==null?void 0:a.checked)!==!1,box_top_count:Gs("benchmarkBoxCount",0),box_top_offset_cm:ai((o=E("benchmarkBoxOffsetCm"))==null?void 0:o.value,5),edge_count:Gs("benchmarkEdgeCount",0),edge_distance_cm:ai((l=E("benchmarkEdgeDistanceCm"))==null?void 0:l.value,5),random_count:Gs("benchmarkRandomCount",0),planning_time:ai((c=E("benchmarkPlanningTime"))==null?void 0:c.value,Ln.planning_time),attempts:Math.max(1,Math.round(ai((u=E("benchmarkAttempts"))==null?void 0:u.value,Ln.attempts))),ik_timeout:ai((h=E("benchmarkIkTimeout"))==null?void 0:h.value,Ln.ik_timeout),collision_boxes:b.manualCollision.boxes||[],planners:Ur()},s=E("benchmarkCollisionBox");return s!=null&&s.disabled?i.box_top_count=0:s!=null&&s.value&&(i.collision_box_id=s.value),t!==""&&Number.isFinite(e)&&(i.seed=Math.max(1,Math.round(e))),i}function oS(n){var s,r;const t=Ur();let e=t[0]||((s=b.ompl.config)==null?void 0:s.planner_id)||Ln.planner_id;const i=Object.values(((r=b.benchmark.result)==null?void 0:r.summary)||{}).filter(a=>!t.length||t.includes(a.planner_id));return i.length&&(i.sort((a,o)=>{const l=Number(o.success_rate||0)-Number(a.success_rate||0);return l||Number(a.mean_elapsed_ms||1/0)-Number(o.mean_elapsed_ms||1/0)}),e=i[0].planner_id||e),ss({planner_id:e,planning_time:n.planning_time,attempts:n.attempts,ik_timeout:n.ik_timeout})}async function lS(){const n=pf("benchmarkPresetName"),t=Bc(),e=oS(t),i=await de("保存跑分配置",()=>Vh({name:n,source:"benchmark",config:e}),{quiet:!0,success:"跑分配置已保存"});if(!i)return;const s=mf(i.presets,n,e);b.ompl.config=e,b.ompl.selectedPresetId=(s==null?void 0:s.id)||null,ao(),await ar(),E("benchmarkStatus").textContent=`已保存到规划预设库：${(s==null?void 0:s.label)||n}`,ie(`已保存跑分配置：${(s==null?void 0:s.label)||n}`)}async function cS(){if(!b.benchmark.result){E("benchmarkStatus").textContent="没有可导出的跑分结果",ie("请先完成一次跑分","warn");return}const n={arm:b.arm,...b.benchmark.result},t=await de("导出跑分 CSV",()=>$f({result:n}),{quiet:!0,success:"跑分 CSV 已导出"});t&&(uS(t),E("benchmarkStatus").textContent=`CSV 已保存：${t.path}`,ie("跑分 CSV 已导出"))}function uS(n){if(!(n!=null&&n.csv))return;const t=new Blob([n.csv],{type:"text/csv;charset=utf-8"}),e=URL.createObjectURL(t),i=document.createElement("a");i.href=e,i.download=(n.path||"unoarm_benchmark.csv").split("/").pop()||"unoarm_benchmark.csv",document.body.appendChild(i),i.click(),i.remove(),URL.revokeObjectURL(e)}function Gs(n,t){var e;return Math.max(0,Math.round(ai((e=E(n))==null?void 0:e.value,t)))}function hS(){return Gs("benchmarkBoxCount",0)+Gs("benchmarkEdgeCount",0)+Gs("benchmarkRandomCount",0)}async function dS(){b.benchmark.options||await Ir(!1);const n=Bc(),t=await de("生成跑分测试点",()=>Gf(n),{quiet:!0,success:"测试点已生成"});t&&(b.benchmark.options={...b.benchmark.options,...t},b.benchmark.samples=t.samples||[],b.benchmark.selectedSampleId=null,b.benchmark.result=null,b.benchmark.progress=null,t.seed&&E("benchmarkSeed")&&(E("benchmarkSeed").value=t.seed),at==null||at.setBenchmarkSamples(b.benchmark.samples),Vn(),gf(),Oi(),ds(),fi(),Fi(),E("benchmarkStatus").textContent=`已生成 ${b.benchmark.samples.length} 个测试点，种子 ${t.seed??"--"}`,ie(`已生成 ${b.benchmark.samples.length} 个跑分测试点`))}async function fS(){b.benchmark.options||await Ir(!1);const n=Bc();if(!n.planners.length){E("benchmarkStatus").textContent="至少选择一个规划器",ie("至少选择一个规划器","warn");return}b.benchmark.samples.length&&(n.samples=b.benchmark.samples),pS(n);const t=await de("规划器跑分",()=>Wf(n),{quiet:!0,success:"跑分完成"});if(await tc(),vf(),!t)return;b.benchmark.result=t,b.benchmark.samples=t.samples||b.benchmark.samples,b.benchmark.selectedSampleId=null,at==null||at.setBenchmarkSamples(b.benchmark.samples),Oi(),ds(),fi();const e=Number(t.total_points||b.benchmark.samples.length),i=Number(t.valid_points||0);E("benchmarkStatus").textContent=`跑分完成：IK 有效 ${i}/${e}，耗时 ${kt(Number(t.elapsed_ms||0)/1e3,2)} s`,ie(`跑分完成：有效点 ${i}/${e}`),Fi(),Mn()}function pS(n={}){vf();const t=b.benchmark.samples.length||Number(n.box_top_count||0)+Number(n.edge_count||0)+Number(n.random_count||0),e=Ur().length;b.benchmark.progress={running:!0,stage:"starting",label:"等待 MoveIt 服务",percent:0,total_points:t,completed_steps:0,total_steps:t+t*e,ik_done:0,ik_ok:0,plan_done:0,plan_total:t*e,elapsed_ms:0,eta_sec:null},Fi(),Ua=window.setInterval(tc,500),tc()}function vf(){Ua&&(window.clearInterval(Ua),Ua=null)}async function tc(){try{const n=await Hf(b.arm);b.benchmark.progress=n,Fi()}catch{}}function Fi(){const n=E("benchmarkProgress");if(!n)return;const t=b.benchmark.progress;if(!t){n.hidden=!0;return}n.hidden=!1;const e=Math.max(0,Math.min(100,Number(t.percent||0)));E("benchmarkProgressStage").textContent=mS(t),E("benchmarkProgressPercent").textContent=`${kt(e,1)}%`,E("benchmarkProgressBar").style.width=`${e}%`,E("benchmarkProgressMeta").textContent=Uh(t),n.dataset.state=t.stage==="failed"||t.stage==="blocked"?"bad":t.stage==="done"?"good":t.running?"running":"neutral",t.running?(tn("benchmarkState","跑分中","warn"),E("benchmarkStatus").textContent=Uh(t)):(t.stage==="failed"||t.stage==="blocked")&&(tn("benchmarkState","跑分失败","bad"),E("benchmarkStatus").textContent=t.error||t.label||"跑分失败")}function mS(n){return n.error&&(n.stage==="failed"||n.stage==="blocked")?n.label||"跑分失败":n.label||(n.stage==="ik"?"IK 检查":n.stage==="planning"?"规划中":n.stage==="done"?"跑分完成":"等待跑分")}function Uh(n){const t=[],e=Number(n.total_points||0),i=Number(n.ik_done||0),s=Number(n.ik_ok||0);e&&t.push(`IK ${i}/${e}，有效 ${s}`);const r=Number(n.plan_total||0);return r&&t.push(`规划 ${Number(n.plan_done||0)}/${r}`),n.current_sample_id!==null&&n.current_sample_id!==void 0&&t.push(`点 #${n.current_sample_id}`),(n.current_planner_label||n.current_planner_id)&&t.push(n.current_planner_label||n.current_planner_id),Number(n.elapsed_ms)>0&&t.push(`已用 ${kh(Number(n.elapsed_ms)/1e3)}`),n.running&&Number(n.eta_sec)>0&&t.push(`预计剩余 ${kh(n.eta_sec)}`),n.error&&t.push(n.error),t.join(" · ")||"等待跑分"}function kh(n){const t=Number(n);return Number.isFinite(t)?t<10?`${kt(t,1)}s`:t<90?`${Math.round(t)}s`:`${Math.floor(t/60)}m${Math.round(t%60)}s`:"--"}function gS(){document.body.dataset.busy||(b.benchmark.samples=[],b.benchmark.result=null,b.benchmark.selectedSampleId=null,b.benchmark.progress=null,at==null||at.setBenchmarkSamples([]),Oi(),ds(),fi(),Fi(),E("benchmarkStatus").textContent="点集参数已修改，重新生成后生效")}function _S(){b.benchmark.result&&(b.benchmark.result=null,b.benchmark.progress=null,ds(),fi(),Fi()),Oi(),E("benchmarkStatus").textContent=b.benchmark.samples.length?"规划参数已修改，重新跑分后生效":"等待生成测试点"}function vS(){Ur(),b.benchmark.result&&(b.benchmark.result=null),b.benchmark.progress=null,Oi(),ds(),fi(),Fi(),E("benchmarkStatus").textContent=b.benchmark.samples.length?"规划器选择已修改，重新跑分后生效":"等待生成测试点"}function Xa(){var n;return((n=b.benchmark.result)==null?void 0:n.samples)||b.benchmark.samples||[]}function Oi(){const n=E("benchmarkQuickStats");if(!n)return;const t=Xa(),e=b.benchmark.result,i=hS(),s=Number((e==null?void 0:e.total_points)??t.length),r=Number((e==null?void 0:e.valid_points)??t.filter(o=>{var l;return(l=o.ik)==null?void 0:l.ok}).length),a=_f();n.innerHTML=[["请求点",i],["已生成",t.length||0],["IK 有效",e?`${r}/${s}`:"--"],["规划器",a]].map(([o,l])=>`
    <div class="benchmark-metric"><span>${Jt(o)}</span><b>${Jt(l)}</b></div>
  `).join(""),e?tn("benchmarkState",r?"结果已生成":"无有效点",r?"good":"bad"):t.length?tn("benchmarkState",`已生成 ${t.length}`,"good"):tn("benchmarkState","未生成","neutral")}function ds(){const n=E("benchmarkSummary");if(!n)return;const t=b.benchmark.result,e=Object.values((t==null?void 0:t.summary)||{});if(!e.length){Hn("benchmarkSummary","跑分后生成规划器成功率结果");return}const i=new Map((t.planners||[]).map((s,r)=>[s.planner_id,r]));e.sort((s,r)=>(i.get(s.planner_id)??999)-(i.get(r.planner_id)??999)),n.innerHTML=e.map(s=>{const r=Math.max(0,Math.min(1,Number(s.success_rate||0))),a=`${Math.round(r*100)}%`;return`
      <div class="benchmark-score-row">
        <div>
          <strong>${Jt(s.label||s.planner_id)}</strong>
          <small>${Jt(s.planner_id)}</small>
        </div>
        <div class="benchmark-score-meter"><span style="width:${r*100}%"></span></div>
        <b>${a}</b>
        <small>${Number(s.planned_ok||0)}/${Number(s.valid_points||0)} · ${kt(s.mean_elapsed_ms,1)} ms</small>
      </div>
    `}).join("")}function fi(){var l;const n=E("benchmarkSamples");if(!n)return;const t=Xa(),e=b.benchmark.result,i=Number((e==null?void 0:e.total_points)??t.length),s=Number((e==null?void 0:e.valid_points)??t.filter(c=>{var u;return(u=c.ik)==null?void 0:u.ok}).length),r=E("benchmarkSetBadge");if(r&&(r.textContent=`${s}/${i||0}`),at==null||at.setBenchmarkSamples(t,b.benchmark.selectedSampleId),!t.length){Hn("benchmarkSamples","暂无测试点");return}const a=yS(e),o=((l=e==null?void 0:e.planners)==null?void 0:l.length)||_f();n.innerHTML=t.map(c=>{var L;const u=[c.x,c.y,c.z].map(w=>kt(w,3)).join(", "),h=a.get(Number(c.id))||[],d=h.filter(w=>w.ok).length,p=!!c.ik,g=((L=c.ik)==null?void 0:L.ok)===!0,_=String(c.id)===String(b.benchmark.selectedSampleId),m=p&&!g?"未计入":e?`${d}/${o}`:"待跑分",f=xS(c.ik),P=bS(h,o);return`
      <div class="benchmark-sample ${_?"active":""}" data-benchmark-sample="${Jt(c.id)}">
        <div>
          <strong>#${Jt(c.id)} ${Jt(Iy[c.source]||c.source||"测试点")}</strong>
          <small>X/Y/Z ${Jt(u)}${f?` · ${Jt(f)}`:""}${P?` · ${Jt(P)}`:""}</small>
        </div>
        <div class="benchmark-sample-state">
          <span data-state="${p?g?"good":"bad":"neutral"}">${p?g?"IK 通过":"IK 失败":"未跑 IK"}</span>
          <span data-state="${e?d?"good":"bad":"neutral"}">规划 ${Jt(m)}</span>
        </div>
      </div>
    `}).join("")}function xS(n){if(!n)return"";const t=Number(n.scanned_count??n.candidate_index),e=Number(n.candidate_count),i=n.mode?`${n.mode} `:"";return Number.isFinite(t)&&Number.isFinite(e)&&e>1?`姿态 ${i}${t}/${e}`:Number.isFinite(t)&&t>1?`姿态 ${i}第 ${t} 个`:i?`姿态 ${i}`:""}function bS(n,t){if(!(n!=null&&n.length)||t!==1)return"";const e=n[0],i=Number(e.scanned_count??e.candidate_index),s=Number(e.candidate_count);return Number.isFinite(i)&&Number.isFinite(s)&&s>1?`规划姿态 ${i}/${s}`:""}function yS(n){const t=new Map;for(const e of Object.values((n==null?void 0:n.results)||{}))for(const i of e||[]){const s=Number(i.sample_id);t.has(s)||t.set(s,[]),t.get(s).push(i)}return t}function MS(n){const t=n.target.closest("[data-benchmark-sample]");if(!t)return;const e=Xa().find(i=>String(i.id)===String(t.dataset.benchmarkSample));e&&(b.benchmark.selectedSampleId=e.id,us(e),at==null||at.setBenchmarkSamples(Xa(),b.benchmark.selectedSampleId),fi(),E("footerMessage").textContent=`已选中跑分点 #${e.id}：${kt(e.x,3)}, ${kt(e.y,3)}, ${kt(e.z,3)}`)}async function zc(){try{const n=await sp();b.points=n.points||[],xf("pointsList",b.points,"point")}catch(n){Hn("pointsList",n.message)}}async function Vc(){try{const n=await rp();b.presets=n.presets||[],xf("presetsList",b.presets,"preset")}catch(n){Hn("presetsList",n.message)}}function xf(n,t,e){const i=E(n);if(!t.length){Hn(n,e==="point"?"暂无点位":"暂无预设");return}i.innerHTML=t.map((s,r)=>`
    <div class="list-item">
      <div>
        <strong>${Jt(s.name||`${e==="point"?"点位":"预设"} ${r+1}`)}</strong>
        <small>${en(s.arm)} · X ${kt(s.x)} · Y ${kt(s.y)} · Z ${kt(s.z)}</small>
      </div>
      <div class="item-actions">
        <button class="icon-button" type="button" data-library-use="${e}:${r}" aria-label="载入" title="载入"><i data-lucide="corner-down-left"></i></button>
        <button class="icon-button" type="button" data-library-delete="${e}:${r}" aria-label="删除" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join(""),os()}function Hn(n,t){E(n).innerHTML=`<p class="empty-state">${Jt(t)}</p>`}async function Fh(){const n=E("pointName").value.trim()||new Date().toLocaleTimeString("zh-CN",{hour12:!1});await de("保存点位",()=>yt("/api/points",{...fn(),name:n})),E("pointName").value="",zc()}async function SS(){const n=E("presetName").value.trim()||new Date().toLocaleTimeString("zh-CN",{hour12:!1});await de("保存抓取预设",()=>yt("/api/presets",{...cs(),name:n,pick_approach:on("pickApproach",.1),pick_descend:on("pickDescend",.05),pick_hold:on("pickHold",1),pick_lift:on("pickLift",.1)})),E("presetName").value="",Vc()}async function Oh(n){var o,l,c;const t=(o=n.target.closest("[data-library-use]"))==null?void 0:o.dataset.libraryUse,e=(l=n.target.closest("[data-library-delete]"))==null?void 0:l.dataset.libraryDelete;if(!t&&!e)return;const[i,s]=(t||e).split(":"),r=Number(s),a=i==="point"?b.points:b.presets;if(t){const u=a[r];if(!u)return;u.arm&&await no(u.arm),us(u),i==="preset"&&ES(u),ie(`已载入${i==="point"?"点位":"预设"}：${u.name||r+1}`);return}await ls(`删除“${((c=a[r])==null?void 0:c.name)||r+1}”？`,"确认删除")&&(await yt(`/api/${i==="point"?"points":"presets"}/delete`,{index:r}),i==="point"?zc():Vc())}function ES(n){const t=[["pickApproach","pick_approach"],["pickDescend","pick_descend"],["pickHold","pick_hold"],["pickLift","pick_lift"]];for(const[e,i]of t){const s=E(e);s&&Number.isFinite(Number(n[i]))&&(s.value=n[i])}}function TS(){const n=E("sequenceType").value;let t={type:n,arm:b.arm};["approach","grasp","pick"].includes(n)&&(t.pose=cs()),n==="move"&&(t.pose=fn()),n==="approach"&&(t.offset_z=on("pickApproach",.1)),n==="lift"&&(t.offset_z=on("pickLift",.1)),n==="pick"&&(t={...t,approach_height:on("pickApproach",.1),descend_distance:on("pickDescend",.05),hold_seconds:on("pickHold",1),lift_height:on("pickLift",.1)}),n==="wait"&&(t.seconds=1),b.sequence.push(t),Hc(),oo()}function oo(){const n=E("sequenceList");if(!b.sequence.length){Hn("sequenceList","暂无动作步骤");return}n.innerHTML=b.sequence.map((t,e)=>`
    <div class="list-item">
      <div><strong>${e+1}. ${Py[t.type]||t.type}</strong><small>${en(t.arm)}${t.seconds?` · ${t.seconds} s`:""}</small></div>
      <div class="item-actions">
        <button class="icon-button" type="button" data-sequence-up="${e}" aria-label="上移" title="上移"><i data-lucide="arrow-up"></i></button>
        <button class="icon-button" type="button" data-sequence-delete="${e}" aria-label="删除" title="删除"><i data-lucide="trash-2"></i></button>
      </div>
    </div>
  `).join(""),os()}function wS(n){var i,s;const t=(i=n.target.closest("[data-sequence-up]"))==null?void 0:i.dataset.sequenceUp,e=(s=n.target.closest("[data-sequence-delete]"))==null?void 0:s.dataset.sequenceDelete;if(t!==void 0){const r=Number(t);r>0&&([b.sequence[r-1],b.sequence[r]]=[b.sequence[r],b.sequence[r-1]])}else if(e!==void 0)b.sequence.splice(Number(e),1);else return;Hc(),oo()}function AS(){b.sequence=[],Hc(),oo()}function Hc(){localStorage.setItem("unoarm.sequence",JSON.stringify(b.sequence))}async function CS(){if(!b.sequence.length){ie("请先添加动作步骤","warn");return}await de("执行动作序列",()=>yt("/api/sequence",{steps:b.sequence}),{confirm:`即将连续执行 ${b.sequence.length} 个动作步骤，请确认双臂工作区安全。`,confirmTitle:"确认执行动作序列"})}async function qa(){try{const n=await ap(200),t=E("logLevelFilter").value,e=(n.logs||[]).map(s=>typeof s=="string"?s:JSON.stringify(s)),i=t==="all"?e:e.filter(s=>s.includes(t));E("logViewer").textContent=i.join(`
`)||"暂无日志"}catch(n){E("logViewer").textContent=`日志读取失败：${n.message}`}}function en(n=b.arm){return n==="left"?"左臂":"右臂"}function RS(){try{const n=JSON.parse(localStorage.getItem("unoarm.sequence")||"[]");return Array.isArray(n)?n:[]}catch{return[]}}Uy();
