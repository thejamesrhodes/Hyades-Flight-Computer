# SUPERSEDED by reanalysis/ (BENCH-01 rev 2). Known defect, kept for the record: setup B's levels at -20 and
# -10 deg sit in a mechanically restricted zone, so a calibration through them stretches every B angle above
# -20 deg by up to 2.2x (7.1 px/deg against 15.7). Do not reuse B numbers from this script.
import json, numpy as np
from scipy.optimize import least_squares
from tracklib import load
S=json.load(open("steps2.json"))
R={}
def proj(p,u): a,b,c=p; return (a*u+b)/(c*u+1)
for f in ("83","84"):
    t,x,y=load(f); rows=S[f]["rows"]
    # calibration levels: settled positions after each 10 deg command, plus the level before the first
    lv={}
    r0=rows[0]; lv.setdefault(round(r0["ang"]-r0["cmd"],1),[]).append(r0["y0"])
    for r in rows:
        if abs(r["cmd"])==10: lv.setdefault(round(r["ang"],1),[]).append(r["y1"])
    A=np.array(sorted(lv)); Y=np.array([np.mean(lv[a]) for a in A])
    U=np.tan(np.radians(A))
    p0=[np.polyfit(U,Y,1)[0],np.polyfit(U,Y,1)[1],0.0]
    fit=least_squares(lambda p: proj(p,U)-Y,p0)
    # inverse: u=(y-b)/(a-c y)
    a,b,c=fit.x
    deg=lambda yy: np.degrees(np.arctan((np.asarray(yy)-b)/(a-c*np.asarray(yy))))
    resid=deg(Y)-A
    print(f,"levels",dict(zip(A.tolist(),[np.round(lv[k],1).tolist() for k in A])))
    print(f,"cal resid deg",np.round(resid,3),"px/deg local",np.round(np.gradient(Y,A),1))
    th=deg(y)
    out=[]
    for r in rows:
        r=dict(r); r["d0"]=float(deg(r["y0"])); r["d1"]=float(deg(r["y1"])); r["dm"]=r["d1"]-r["d0"]; out.append(r)
    R[f]=dict(levels={str(k):lv[k] for k in A},cal=fit.x.tolist(),resid=resid.tolist(),rows=out)
    np.save(f"th{f}.npy",np.c_[t,th])
json.dump(R,open("res.json","w"))
for f in R:
    rows=R[f]["rows"]
    for k in (10,1,0.1):
        dm=np.array([r["dm"]*np.sign(r["cmd"]) for r in rows if abs(abs(r["cmd"])-k)<1e-6])
        print(f,k,"n",len(dm),"mean %.3f sd %.3f min %.3f max %.3f  nomove(<0.03) %d"%(dm.mean(),dm.std(),dm.min(),dm.max(),(np.abs(dm)<0.03).sum()))
