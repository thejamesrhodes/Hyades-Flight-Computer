import json, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from common import *
S=json.load(open("steps2.json")); A2=json.load(open("an2.json"))
OI=dict(blue="#0072B2",orange="#E69F00",green="#009E73",verm="#D55E00",purple="#CC79A7",sky="#56B4E9",black="#000000",grey="#7a7a7a")
LAB={"83":"A: no bearing, lightest","84":"B: bearings and horn, flight-like"}
COL={"83":OI["blue"],"84":OI["verm"]}
plt.rcParams.update({"font.size":9,"axes.spines.top":False,"axes.spines.right":False})
def series(f):
    t,_,y=load(f); th=CAL[f](y); rows=S[f]["rows"]
    t0=np.array([r["t0"] for r in rows]); ang=np.array([r["ang"] for r in rows]); cmd=np.array([r["cmd"] for r in rows])
    return t,th,rows,t0,ang,cmd
# ---- Fig 1 overview
fig,axs=plt.subplots(2,1,figsize=(12,6.2))
for ax,f in zip(axs,("83","84")):
    t,th,rows,t0,ang,cmd=series(f); m=t>1.0
    ts=np.r_[t[m][0],t0,t[-1]]; cs=np.r_[ang[0]-cmd[0],ang,ang[-1]]
    ax.step(ts,cs,where="post",color=OI["black"],lw=1.2,label="Commanded zero (fin 2)")
    ax.plot(t[m],th[m],color=COL[f],lw=1.2,label="Measured, laser on canard")
    for k,lab in ((10,"10°"),(1,"1°"),(0.1,"0.1°")):
        ii=np.where(np.abs(np.abs(cmd)-k)<1e-6)[0]
        ax.axvspan(t0[ii[0]]-0.05,(t0[ii[-1]+1] if ii[-1]+1<len(t0) else t[-1]),color=OI["grey"],alpha=0.08,lw=0)
        ax.text(0.5*(t0[ii[0]]+(t0[ii[-1]+1] if ii[-1]+1<len(t0) else t[-1])),1.01,lab+" jogs",transform=ax.get_xaxis_transform(),ha="center",va="bottom",fontsize=8.5)
    ax.set_title(LAB[f],loc="left",weight="bold",pad=14); ax.set_ylabel("Canard angle (deg)"); ax.legend(loc="lower right",fontsize=8,frameon=False)
axs[1].set_xlabel("Video time (s)")
plt.tight_layout(); plt.savefig("bench01_overview.png",dpi=160); plt.close()
# ---- Fig 2: 0.1 deg tracking, progress vs command
fig,axs=plt.subplots(1,2,figsize=(12,4.4))
for f in ("83","84"):
    t,th,rows,t0,ang,cmd=series(f)
    idx=np.where(np.abs(np.abs(cmd)-0.1)<1e-6)[0]; s=np.sign(cmd[idx[0]])
    a0=CAL[f](rows[idx[0]]["y0"]); c0=ang[idx[0]]-cmd[idx[0]]
    m=(t>t0[idx[0]]-0.2)
    # measured progress vs time, commanded progress vs time
    cp=np.interp(t[m],np.r_[t0[idx[0]]-0.2,t0[idx]],np.r_[0,(ang[idx]-c0)*s])
    # piecewise constant command
    k=np.searchsorted(t0[idx],t[m],side="right")-1; cp=np.where(k>=0,(ang[idx][np.clip(k,0,None)]-c0)*s,0)
    mp=(th[m]-a0)*s
    axs[0].plot(cp,mp,color=COL[f],lw=1.3,label=LAB[f])
    axs[1].plot(t[m]-t0[idx[0]],cp-mp,color=COL[f],lw=1.1,label=LAB[f])
axs[0].plot([0,10.5],[0,10.5],color=OI["black"],lw=0.8,ls="-",label="1:1")
axs[0].set_xlabel("Commanded travel since the reversal (deg)"); axs[0].set_ylabel("Measured travel (deg)"); axs[0].legend(frameon=False,fontsize=8)
axs[0].set_title("0.1° jogs after a direction reversal",loc="left",weight="bold")
axs[1].axhline(0,color=OI["black"],lw=0.8)
axs[1].set_xlabel("Time since first 0.1° jog (s)"); axs[1].set_ylabel("Lag, commanded − measured (deg)"); axs[1].legend(frameon=False,fontsize=8)
axs[1].set_title("Tracking lag",loc="left",weight="bold")
plt.tight_layout(); plt.savefig("bench01_fine_steps.png",dpi=160); plt.close()
# ---- Fig 3: dynamics + per-command step distribution
fig,axs=plt.subplots(1,3,figsize=(13,4.0))
for f in ("83","84"):
    t,th,rows,t0,ang,cmd=series(f); dm=np.array(A2[f]["dm"])
    for j,(k,ax) in enumerate(((10,axs[0]),(1,axs[1]))):
        curves=[]
        for i in np.where(np.abs(np.abs(cmd)-k)<1e-6)[0]:
            r=rows[i]; m=(t>r["t0"]-0.05)&(t<r["t0"]+0.25); tt=t[m]-r["t0"]; s=th[m]
            a0=np.median(s[tt<0]); a1=np.median(th[(t>r["t1"]-0.1)&(t<r["t1"])]); D=a1-a0
            if abs(D)<0.3*k: continue
            n=(s-a0)/D
            # align on the 50 % crossing
            kk=np.where(n>=0.5)[0]
            if not len(kk): continue
            tc=tt[kk[0]]; ax.plot((tt-tc)*1000,n,color=COL[f],lw=0.6,alpha=0.45)
            curves.append(np.interp(np.arange(-40,121)/1000,tt-tc,n))
        if curves: ax.plot(np.arange(-40,121),np.median(curves,0),color=COL[f],lw=2.2,label=LAB[f])
        ax.set_xlim(-40,120); ax.set_ylim(-0.1,1.35); ax.axhline(1,color=OI["black"],lw=0.6)
        ax.set_xlabel("Time from 50 % crossing (ms)"); ax.set_title(f"{k}° steps, normalised",loc="left",weight="bold")
    axs[0].set_ylabel("Fraction of the step"); axs[0].legend(frameon=False,fontsize=8,loc="lower right")
    for k,xo in ((1,0),(0.1,1)):
        v=dm[np.abs(np.abs(cmd)-k)<1e-6]
        xs=xo+(-0.18 if f=="83" else 0.18)+np.random.default_rng(1).uniform(-0.1,0.1,len(v))
        axs[2].plot(xs,v,"o",ms=3.5,color=COL[f],alpha=0.75,mec="none",label=LAB[f] if k==1 else None)
axs[2].plot([-0.4,0.4],[1,1],color=OI["black"],lw=1); axs[2].plot([0.6,1.4],[0.1,0.1],color=OI["black"],lw=1); axs[2].axhline(0,color=OI["grey"],lw=0.6)
axs[2].set_xticks([0,1]); axs[2].set_xticklabels(["1° jogs","0.1° jogs"]); axs[2].set_ylabel("Measured step (deg); black bar = commanded")
axs[2].set_title("Every jog, measured",loc="left",weight="bold"); axs[2].set_ylim(-0.3,3.1); axs[2].legend(frameon=False,fontsize=8,loc="upper center")
plt.tight_layout(); plt.savefig("bench01_dynamics.png",dpi=160); plt.close()
print("ok")
