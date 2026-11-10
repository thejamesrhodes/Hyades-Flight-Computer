from common import *
out={}
for f in ("83","84"):
    t,x,y=load(f); deg=CAL[f]; th=deg(y); rows=S[f]["rows"]
    cmd=np.array([r["cmd"] for r in rows]); ang=np.array([r["ang"] for r in rows]); t0=np.array([r["t0"] for r in rows]); t1=np.array([r["t1"] for r in rows])
    dm=np.array([deg(r["y1"])-deg(r["y0"]) for r in rows])*np.sign(cmd)
    # dynamics for |cmd|>=1
    dyn=[]
    for i in np.where(np.abs(cmd)>=1)[0]:
        m=(t>=t0[i]-0.1)&(t<=min(t1[i],t0[i]+0.6)); tt=t[m]; s=th[m]
        a0=np.median(s[tt<t0[i]]); a1=np.median(s[-12:]); D=a1-a0
        if abs(D)<0.3: continue
        n=(s-a0)/D
        def cross(v):
            k=np.where(n>=v)[0]; 
            if len(k)==0: return np.nan
            k=k[0]; return tt[k-1]+(v-n[k-1])/(n[k]-n[k-1])*(tt[k]-tt[k-1]) if k>0 else tt[k]
        t10,t50,t90=cross(0.1),cross(0.5),cross(0.9)
        os_=(n.max()-1)*100
        vel=np.max(np.abs(np.gradient(s,tt)))
        # settle: last time outside +/-max(0.05 deg, 2%)
        band=max(0.05,0.02*abs(D))/abs(D); k=np.where(np.abs(n-1)>band)[0]; ts=tt[k[-1]]-t10 if len(k) else np.nan
        dyn.append(dict(i=int(i),cmd=float(cmd[i]),D=float(D),t10=float(t10-t0[i]),rise=float(t90-t10),overshoot=float(os_),vmax=float(vel),settle=float(ts)))
    out[f]=dict(t=t.tolist()[::1],dm=dm.tolist(),dyn=dyn)
    for k in (10,1):
        dd=[d for d in dyn if abs(abs(d["cmd"])-k)<1e-6]
        arr=lambda key: np.array([d[key] for d in dd])
        print(f,k,"n%d rise %.1f±%.1f ms  t10 %.1f±%.1f ms  OS %.1f±%.1f %%  vmax %.0f±%.0f deg/s settle %.0f±%.0f ms"%(len(dd),1e3*arr('rise').mean(),1e3*arr('rise').std(),1e3*arr('t10').mean(),1e3*arr('t10').std(),arr('overshoot').mean(),arr('overshoot').std(),arr('vmax').mean(),arr('vmax').std(),1e3*np.nanmean(arr('settle')),1e3*np.nanstd(arr('settle'))))
    # 0.1 sequence: tracking error
    m01=np.abs(np.abs(cmd)-0.1)<1e-6; idx=np.where(m01)[0]
    sgn=np.sign(cmd[idx[0]])
    meas=np.array([deg(rows[i]["y1"]) for i in idx]); cm=ang[idx]
    e=(meas-meas[0])*sgn-(cm-cm[0])*sgn
    moves=np.abs(dm[idx])>=0.03
    runs=[]; c=0
    for mv in moves:
        c+=1
        if mv: runs.append(c); c=0
    print(f,"0.1: e range %.3f..%.3f p2p %.3f; moves %d/%d; cmds per move %s; slip sizes %s"%(e.min(),e.max(),e.max()-e.min(),moves.sum(),len(moves),runs,np.round(dm[idx][moves],2).tolist()))
json.dump(out,open("an2.json","w"))
