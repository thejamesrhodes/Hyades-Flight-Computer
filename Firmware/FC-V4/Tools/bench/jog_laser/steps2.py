import json, numpy as np
from tracklib import load
C=json.load(open("cmds.json"))
START={"83":29.4,"84":-20.0}
def onsets(t,y,thr=60):
    v=np.gradient(y)*240; on=[]; i=0
    while i<len(v):
        if t[i]>1 and abs(v[i])>thr: on.append(i); i+=48
        else: i+=1
    return on
res={}
for f in ("83","84"):
    t,x,y=load(f); M=json.load(open(f"match{f}.json")); rel=M["t"]
    for i,v in enumerate(rel):
        if v is None: rel[i]=0.5*(rel[i-1]+rel[i+1])
    rel=np.array(rel); w=C[f]; cmd=np.array([c[3] for c in w])
    on=t[onsets(t,y)]
    big=np.abs(cmd)>=1
    d=[]
    for k in np.where(big)[0]:
        dd=on-rel[k]; j=np.argmin(np.abs(dd+0.2)); d.append(dd[j])
    d=np.array(d); off=np.median(d)
    print(f,"offset onset-release median %.3f  sd %.3f  (n=%d)"%(off,d.std(),len(d)), np.round(d,3))
    tc=rel+off-0.02   # effective command time, 20 ms before typical onset
    ang=START[f]+np.cumsum(cmd)
    rows=[]
    for i in range(len(w)):
        t0=tc[i]; t1=tc[i+1] if i+1<len(w) else t0+1.5
        pre=(t>t0-0.1)&(t<=t0); post=(t>t1-0.1)&(t<=t1)
        rows.append(dict(i=i,cmd=float(cmd[i]),ang=float(ang[i]),t0=float(t0),t1=float(t1),y0=float(np.median(y[pre])),y1=float(np.median(y[post]))))
    res[f]=dict(off=float(off),rows=rows)
json.dump(res,open("steps2.json","w"))
for f in res:
    for r in res[f]["rows"]:
        if abs(r["cmd"])>=1: print(f,"%3d %+6.1f ang %+6.1f  y %7.2f->%7.2f dy %+8.2f"%(r["i"],r["cmd"],r["ang"],r["y0"],r["y1"],r["y1"]-r["y0"]))
