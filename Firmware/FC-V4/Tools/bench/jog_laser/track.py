import subprocess, numpy as np, sys, cv2
f, outp, xroi = sys.argv[1], sys.argv[2], int(sys.argv[3])
W,H=1280,720
p=subprocess.Popen(["ffmpeg","-v","error","-noautorotate","-i",f,"-f","rawvideo","-pix_fmt","rgb24","-"],stdout=subprocess.PIPE,bufsize=10**8)
ref=None; prev=None; out=[]; n=0
win=cv2.createHanningWindow((xroi//2, H//2), cv2.CV_32F)
while True:
    b=p.stdout.read(W*H*3)
    if len(b)<W*H*3: break
    a=np.frombuffer(b,np.uint8).reshape(H,W,3)
    r=a[...,0].astype(np.int16); g=a[...,1].astype(np.int16); bl=a[...,2].astype(np.int16)
    white=((r>=245)&(g>=215)&(bl>=235)).astype(np.float32)
    mag=((r>=200)&(bl-g>=25)).astype(np.float32)
    sc=cv2.boxFilter(white,-1,(9,9))*cv2.boxFilter(mag,-1,(25,25))
    sc[600:,:]=0   # exclude emitter / rocket strip at the bottom of the raw frame
    i=int(np.argmax(sc)); y,x=divmod(i,W); v=float(sc.flat[i])
    if v>0:
        y0,y1,x0,x1=max(0,y-14),y+15,max(0,x-14),x+15
        wm=white[y0:y1,x0:x1]; ys,xs=np.nonzero(wm)
        cx=x0+xs.mean(); cy=y0+ys.mean(); nc=len(xs)
    else: cx=cy=np.nan; nc=0
    gray=cv2.cvtColor(np.ascontiguousarray(a[:,:xroi]),cv2.COLOR_RGB2GRAY)
    gs=cv2.resize(gray,(xroi//2,H//2)).astype(np.float32)
    if ref is None: ref=gs
    (dx,dy),resp=cv2.phaseCorrelate(ref,gs,win)
    out.append((n,cx,cy,nc,v,2*dx,2*dy,resp)); n+=1
np.save(outp,np.array(out)); print(n)
