import csv,bisect,sys,statistics as st
from collections import Counter

def crc8_97(v24):
    crc=0
    for b in ((v24>>16)&0xFF,(v24>>8)&0xFF,v24&0xFF):
        crc^=b
        for _ in range(8):
            crc = ((crc<<1)^0x97)&0xFF if crc&0x80 else (crc<<1)&0xFF
    return crc

def parity22(pd):
    return bin(pd & 0x3FFFFF).count('1') & 1

def decode(var, raw):
    """Independent decode straight from Product Guide 5.4.2."""
    if var=='ssi1':
        return dict(pv=(raw>>23)&1, zpd=(raw>>22)&1, pd=raw&0x3FFFFF, ts=None, chk=True)
    if var=='ssi2':
        pd=(raw>>2)&0x3FFFFF; p=(raw>>1)&1; a=raw&1
        return dict(pv=(a==0), zpd=None, pd=pd, ts=None, chk=(p==parity22(pd)))
    if var=='ssi6':
        body=raw&0xFFFFFF
        return dict(pv=(raw>>23)&1, zpd=(raw>>22)&1, pd=body&0x3FFFFF, ts=None,
                    chk=(crc8_97(body)==((raw>>24)&0xFF)))
    # ssi4 / ssi9
    return dict(pv=(raw>>31)&1, zpd=(raw>>30)&1, pd=(raw>>11)&0x7FFFF,
                ts=raw&0x7FF, chk=True)

def run(path,var,n,expect_pd):
    rows=[]
    with open(path) as f:
        r=csv.reader(f); next(r)
        for a in r:
            if len(a)>=3: rows.append((float(a[0]),int(a[1]),int(a[2])))
    dt=[x[0] for x in rows]
    def data_at(t):
        return rows[max(bisect.bisect_right(dt,t)-1,0)][2]
    fall=[rows[k][0] for k in range(1,len(rows)) if rows[k][1]==0 and rows[k-1][1]==1]
    rise=[rows[k][0] for k in range(1,len(rows)) if rows[k][1]==1 and rows[k-1][1]==0]
    dl=sorted(fall[i+1]-fall[i] for i in range(len(fall)-1))
    thr=max(dl[len(dl)//2]*2.5,1e-6)
    cyc=[];cur=[fall[0]]
    for t in fall[1:]:
        if t-cur[-1]>thr: cyc.append(cur);cur=[t]
        else: cur.append(t)
    cyc.append(cur)

    counts=Counter(len(c) for c in cyc)
    ok=bad=chkfail=0; tmus=[]; gaps=[]
    for c in cyc:
        if len(c)!=n: continue
        T=(c[-1]-c[0])/(len(c)-1)
        r_in=[x for x in rise if c[0]<=x<=c[-1]+T]
        if len(r_in)!=n: continue
        v=0
        for x in r_in: v=(v<<1)|data_at(x+2e-9)
        d=decode(var,v)
        if not d['chk']: chkfail+=1
        if d['pd']==expect_pd and d['chk']: ok+=1
        else: bad+=1
        # Tmu: last falling -> data high and stays high
        i=bisect.bisect_right(dt,c[-1]); tm=None
        lim=c[-1]+400e-6
        while i<len(rows) and dt[i]<lim:
            if rows[i][2]==1 and rows[i-1][2]==0: tm=dt[i]
            i+=1
        if tm: tmus.append(tm-c[-1])
        gaps.append(data_at(r_in[-1]+10e-6))
    print(f"  clocks/cycle : {dict(counts)}   (expect all {n})")
    if tmus:
        tv=[t for t in tmus if t<100e-6]
        print(f"  Tmu          : mean {st.mean(tv)*1e6:.2f} us  range {min(tv)*1e6:.2f}-{max(tv)*1e6:.2f}")
    print(f"  gap level    : {dict(Counter(gaps))}  (0=LOW)")
    print(f"  frames       : ok={ok} bad={bad} integrity-fail={chkfail}")

run(sys.argv[1],sys.argv[2],int(sys.argv[3]),int(sys.argv[4]))
