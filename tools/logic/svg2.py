import csv,bisect,sys
def load(p):
    rows=[]
    with open(p) as f:
        r=csv.reader(f); next(r)
        for a in r:
            if len(a)>=3: rows.append((float(a[0]),int(a[1]),int(a[2])))
    return rows
def cycles_of(rows):
    fall=[rows[k][0] for k in range(1,len(rows)) if rows[k][1]==0 and rows[k-1][1]==1]
    dl=sorted(fall[i+1]-fall[i] for i in range(len(fall)-1))
    thr=max(dl[len(dl)//2]*2.5,1e-6)
    cyc=[];cur=[fall[0]]
    for t in fall[1:]:
        if t-cur[-1]>thr: cyc.append(cur);cur=[t]
        else: cur.append(t)
    cyc.append(cur);return cyc
def rising(rows,tstart,tend):
    """Clock rising edges inside the window -- the edges the specification says
    the encoder should set each data bit on."""
    return [rows[k][0] for k in range(1,len(rows))
            if rows[k][1]==1 and rows[k-1][1]==0 and tstart<=rows[k][0]<=tend]
W=980;PAD=58
def draw(rows,y,tstart,tend,title,ticks):
    dt=[r[0] for r in rows]
    def val(t,ch):
        i=max(bisect.bisect_right(dt,t)-1,0)
        return rows[i][1] if ch==0 else rows[i][2]
    def X(t): return PAD+(W-PAD-14)*(t-tstart)/(tend-tstart)
    s=[f'<text x="{PAD}" y="{y-14}" class="ttl">{title}</text>']
    # Rising-edge grid first, so the traces are drawn over it. Each line is an
    # edge on which DATA is supposed to change; it visibly does not.
    for t in rising(rows,tstart,tend):
        s.append(f'<line x1="{X(t):.1f}" y1="{y+8}" x2="{X(t):.1f}" y2="{y+92}" class="rg"/>')
    for ci,(lbl,col) in enumerate([("CLK  ch0","#1d4ed8"),("DATA ch1","#b91c1c")]):
        base=y+ci*48+28
        s.append(f'<text x="6" y="{base-5}" class="lbl">{lbl}</text>')
        s.append(f'<line x1="{PAD}" y1="{base}" x2="{W-14}" y2="{base}" class="ax"/>')
        pts=[];prev=None;n=1600;span=tend-tstart
        for i in range(n+1):
            t=tstart+span*i/n; v=val(t,ci); x=PAD+(W-PAD-14)*i/n
            if prev is not None and prev!=v: pts.append(f"{x:.1f},{base-20*prev:.1f}")
            pts.append(f"{x:.1f},{base-20*v:.1f}"); prev=v
        s.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{col}" stroke-width="1.6"/>')
    for tt,lab,dy,cls in ticks:
        x=X(tt)
        s.append(f'<line x1="{x:.1f}" y1="{y+6}" x2="{x:.1f}" y2="{y+96}" class="{cls}"/>')
        s.append(f'<text x="{x+4:.1f}" y="{y+96+dy}" class="ann {cls}t">{lab}</text>')
    return "\n".join(s)

a=load(sys.argv[1]); b=load(sys.argv[2])
ca=cycles_of(a); cb=cycles_of(b)
c=ca[5]; t0=c[0]
dt=[r[0] for r in a]
tmu=None;i=bisect.bisect_right(dt,c[-1]);lim=ca[6][0]
while i<len(a) and dt[i]<lim:
    if a[i][2]==1 and a[i-1][2]==0: tmu=dt[i]
    i+=1
p1=draw(a,44,t0-2.5e-6,t0+40e-6,
  "One SSI4 Read Cycle at 2 MHz (PV=1) — 32 clocks, Error Flag LOW in the gap, then DATA returns HIGH",
  [(t0,"first falling edge",0,"f1"),(c[-1],"32nd clock",0,"gd"),
   (tmu,f"Tmu = {(tmu-c[-1])*1e6:.2f} us",14,"gd")])
c2=cb[5]; u0=c2[0]
p2=draw(b,232,u0-140e-9,u0+1.3e-6,
  "Zoom, PV forced to 0: first falling edge -> DATA driven valid (the EXTI3 handover)",
  [(u0,"first falling edge",0,"f1"),(u0+207e-9,"DATA valid, mean 207 ns",0,"gd"),
   (u0+250e-9,"250 ns budget",14,"gd")])
svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="392" viewBox="0 0 {W} 392">
<style>.ttl{{font:600 12.5px system-ui,sans-serif;fill:#0f172a}}
.lbl{{font:10.5px ui-monospace,monospace;fill:#475569}}
.ann{{font:10px system-ui,sans-serif;fill:#475569}}
.ax{{stroke:#e2e8f0;stroke-width:1}}
.gd{{stroke:#94a3b8;stroke-width:1;stroke-dasharray:3 3}} .gdt{{fill:#475569}}
.rg{{stroke:#cbd5e1;stroke-width:0.7}}
.f1{{stroke:#047857;stroke-width:1.5}} .f1t{{fill:#047857;font-weight:600}}
.key{{font:10px system-ui,sans-serif;fill:#64748b}}</style>
<rect width="100%" height="100%" fill="#ffffff"/>
{p1}
{p2}
<line x1="{PAD}" y1="366" x2="{PAD+22}" y2="366" class="rg"/>
<text x="{PAD+28}" y="369" class="key">clock rising edges — DATA should change on these; it changes half a period early, on the falls between them</text>
<line x1="{PAD}" y1="380" x2="{PAD+22}" y2="380" class="f1"/>
<text x="{PAD+28}" y="383" class="key">first falling edge — starts the Read Cycle and triggers the EXTI3 handover</text>
</svg>'''
open(sys.argv[3],"w").write(svg); print("wrote",sys.argv[3])
