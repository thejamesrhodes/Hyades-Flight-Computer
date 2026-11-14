import numpy as np, scipy.signal as ss, sys, json
res={}
# Audio stream start in each file (ffprobe stream=start_time). Rev 1 used 0.181 for both, which put
# every 7984 click 0.127 s late (BENCH-01 rev 2 section 8).
A0={"7983":0.181,"7984":0.054}
for f in ("7983","7984"):
    x=np.fromfile(f"a{f}.raw",np.int16).astype(float)/32768; fs=48000
    hb=ss.sosfiltfilt(ss.butter(4,[9000,17000],'bandpass',fs=fs,output='sos'),x)
    env=np.sqrt(ss.sosfiltfilt(ss.butter(2,300,fs=fs,output='sos'),hb**2).clip(0))
    thr=8*np.median(env)
    pk,_=ss.find_peaks(env,height=thr,distance=int(0.08*fs))
    t=pk/fs+A0[f]
    # 4-6.5 kHz band (servo) envelope at 1 kHz rate
    sb=ss.sosfiltfilt(ss.butter(4,[4000,7000],'bandpass',fs=fs,output='sos'),x)
    senv=np.sqrt(ss.sosfiltfilt(ss.butter(2,50,fs=fs,output='sos'),sb**2).clip(0))[::48]
    np.save(f"senv{f}.npy",senv)
    res[f]=t.tolist(); print(f,len(t),np.round(t,2).tolist())
json.dump(res,open("clicks.json","w"))
