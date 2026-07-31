import csv,bisect,sys,statistics as st
def load(p):
    rows=[]
    with open(p) as f:
        r=csv.reader(f); next(r)
        for a in r:
            if len(a)>=3: rows.append((float(a[0]),int(a[1]),int(a[2])))
    return rows
def run(path,label):
    rows=load(path); dt=[r[0] for r in rows]
    def val(t,ch):
        i=max(bisect.bisect_right(dt,t)-1,0)
        return rows[i][1] if ch==0 else rows[i][2]
    fall=[];rise=[]
    for k in range(1,len(rows)):
        if rows[k][1]==0 and rows[k-1][1]==1: fall.append(rows[k][0])
        if rows[k][1]==1 and rows[k-1][1]==0: rise.append(rows[k][0])
    cyc=[];cur=[fall[0]]
    for t in fall[1:]:
        if t-cur[-1]>10e-6: cyc.append(cur);cur=[t]
        else: cur.append(t)
    cyc.append(cur)
    periods=[];samples=[]
    for c in cyc:
        if len(c)!=32: continue
        periods+=[c[i+1]-c[i] for i in range(31)]
        r_in=[r for r in rise if c[0]<=r<=c[-1]+1e-6]
        if len(r_in)!=32: continue
        v=0
        for r in r_in: v=(v<<1)|val(r+2e-9,1)
        samples.append((c[0], v & 0x7FF, (v>>11)&0x7FFFF))
    # unwrap TS
    ts=[];prev=None;off=0
    for t,s,pd in samples:
        if prev is not None and s<prev-1000: off+=2048
        ts.append(s+off); prev=s
    tt=[s[0] for s in samples]
    n=len(tt); mx=sum(tt)/n; my=sum(ts)/n
    num=sum((tt[i]-mx)*(ts[i]-my) for i in range(n))
    den=sum((tt[i]-mx)**2 for i in range(n))
    slope=num/den                      # ticks per second
    nominal=1e5                        # 10 us per tick
    err=(slope/nominal-1)*100
    mp=st.mean(periods)
    print(f"=== {label} ===")
    print(f"  cycles used            : {n}")
    print(f"  clock period (mean)    : {mp*1e9:.2f} ns  -> {1/mp/1e6:.4f} MHz  "
          f"(nominal 500.00 ns / 2.0000 MHz, err {(500e-9/mp-1)*100:+.3f}%)")
    print(f"  timestamp tick rate    : {slope:.1f} ticks/s (nominal 100000)")
    print(f"  timestamp period       : {1e6/slope:.4f} us/tick (nominal 10.0000)")
    print(f"  TIMESTAMP ACCURACY     : {err:+.3f} %   spec: better than 1%  -> "
          f"{'PASS' if abs(err)<1.0 else 'FAIL'}")
run(sys.argv[1],sys.argv[2])
