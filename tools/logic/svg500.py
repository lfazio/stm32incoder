"""One complete SSI4 Read Cycle at 500 kHz -- the ordinary operating point.

At 2 MHz a whole cycle is 84 us of which the interesting parts are hundreds of
nanoseconds wide, so that figure has to zoom. At 500 kHz one bit is 2 us and the
whole frame fits on the page at a scale where the field layout is legible, which
is what makes this the useful picture of normal operation.
"""
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

rows=load(sys.argv[1]); out=sys.argv[2]
cyc=cycles_of(rows); c=cyc[5]
dt=[r[0] for r in rows]
def val(t,ch):
    i=max(bisect.bisect_right(dt,t)-1,0)
    return rows[i][1] if ch==0 else rows[i][2]

T=(c[-1]-c[0])/(len(c)-1)                      # clock period, measured
# Where in the bit cell to sample depends on which edge the encoder drives on.
# Falling-edge builds set the bit at the falling edge; rising-edge builds set it
# half a period later. Sampling a quarter period into the cell works for both,
# so the offset is all that changes.
MODE   = sys.argv[3] if len(sys.argv) > 3 else "falling"
RISING = MODE in ("rising", "timer")
TITLE = {
 "falling": "One complete SSI4 Read Cycle at 500 kHz — the ordinary operating point",
 "rising":  "DATA on the rising edge (SIMENC_RISING_EDGE) — one Read Cycle at 500 kHz",
 "timer":   "DATA shifted by the clock itself (SIMENC_TIMER_DATA) — one Read Cycle at 500 kHz",
}[MODE]
KEY = {
 "falling": "clock rising edges — one per bit; DATA should change on these and changes half a period early instead",
 "rising":  "clock rising edges — one per bit; DATA now changes on these, 8 ns after, as 5.4.1 note 2 requires",
 "timer":   "clock rising edges — one per bit; each bit is written at its rising edge and stays valid across the falling edge, where the controller samples",
}[MODE]
bits=[val(t + (T*0.75 if RISING else T*0.25), 1) for t in c]
raw=0
for b in bits: raw=(raw<<1)|b
pv=(raw>>31)&1; zpd=(raw>>30)&1; pd=(raw>>11)&0x7FFFF; ts=raw&0x7FF

# DATA returns HIGH one Tmu after the last falling edge and stays HIGH until the
# next Read Cycle, so Tmu is the LAST rise in the gap. There can be an earlier
# one: when the last data bit D0 is 1 the line stays high until the Error Flag
# interrupt pulls it down, which shows as a pulse at the start of the gap. That
# is a real defect, not a plotting artefact -- it is annotated below rather than
# skipped over.
tmu=None; i=bisect.bisect_right(dt,c[-1]); stale=None
while i<len(rows) and dt[i]<cyc[6][0]:
    if rows[i][2]==1 and rows[i-1][2]==0:
        if tmu is not None or stale is None: stale = dt[i] if tmu is None else stale
        tmu=dt[i]
    i+=1
# the stale-bit pulse: line high at the last clock, dropping later in the gap
pulse=None
if val(c[-1] + T*0.6, 1) == 1:
    j=bisect.bisect_right(dt,c[-1])
    while j<len(rows) and dt[j]<cyc[6][0]:
        if rows[j][2]==0 and rows[j-1][2]==1: pulse=dt[j]; break
        j+=1

W=980;PAD=58;RIGHT=14
tstart=c[0]-5e-6; tend=(tmu if tmu else c[-1])+9e-6
def X(t): return PAD+(W-PAD-RIGHT)*(t-tstart)/(tend-tstart)

s=[]
Y=64
s.append(f'<text x="{PAD}" y="{Y-40}" class="ttl">{TITLE}</text>')
s.append(f'<text x="{PAD}" y="{Y-24}" class="sub">32 clocks · T = {T*1e9:.0f} ns · decoded PV={pv} ZPD={zpd} PD={pd} TS={ts} · Tmu = {(tmu-c[-1])*1e6:.2f} us</text>')

# Field bands over the bit cells: bit i occupies falling edge i .. i+1.
def band(i0,i1,label,fill):
    x0=X(c[i0]); x1=X(c[i1]+T) if i1==len(c)-1 else X(c[i1+1])
    s.append(f'<rect x="{x0:.1f}" y="{Y-14}" width="{x1-x0:.1f}" height="15" fill="{fill}"/>')
    s.append(f'<text x="{(x0+x1)/2:.1f}" y="{Y-3}" class="fld" text-anchor="middle">{label}</text>')
band(0,0,"PV","#bfdbfe"); band(1,1,"ZPD","#bbf7d0")
band(2,20,"PD[18:0] — position, 19 bits","#fde68a")
band(21,31,"TS[10:0] — time stamp, 10 us steps","#fbcfe8")

# Rising-edge grid: the edges the specification says DATA should change on.
for k in range(1,len(rows)):
    if rows[k][1]==1 and rows[k-1][1]==0 and tstart<=rows[k][0]<=tend:
        s.append(f'<line x1="{X(rows[k][0]):.1f}" y1="{Y+6}" x2="{X(rows[k][0]):.1f}" y2="{Y+108}" class="rg"/>')

for ci,(lbl,col) in enumerate([("CLK  ch0","#1d4ed8"),("DATA ch1","#b91c1c")]):
    base=Y+ci*54+34
    s.append(f'<text x="6" y="{base-5}" class="lbl">{lbl}</text>')
    s.append(f'<line x1="{PAD}" y1="{base}" x2="{W-RIGHT}" y2="{base}" class="ax"/>')
    pts=[];prev=None;n=3600
    for i in range(n+1):
        t=tstart+(tend-tstart)*i/n; v=val(t,ci); x=PAD+(W-PAD-RIGHT)*i/n
        if prev is not None and prev!=v: pts.append(f"{x:.1f},{base-22*prev:.1f}")
        pts.append(f"{x:.1f},{base-22*v:.1f}"); prev=v
    s.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{col}" stroke-width="1.6"/>')

def mark(t,lab,cls,dy=0):
    """Labels near the right edge are anchored to the right of their rule, so
    the last annotation is not clipped off the canvas."""
    x=X(t); near_edge = x > W*0.78
    anc = 'end' if near_edge else 'start'
    s.append(f'<line x1="{x:.1f}" y1="{Y+4}" x2="{x:.1f}" y2="{Y+112}" class="{cls}"/>')
    s.append(f'<text x="{x+(-4 if near_edge else 4):.1f}" y="{Y+112+dy}" '
             f'class="ann {cls}t" text-anchor="{anc}">{lab}</text>')
mark(c[0],"first falling edge","f1")
mark(c[-1],"32nd clock","gd")
if pulse: mark(pulse,"stale D0 held until the Error Flag lands","bad",26)
if tmu: mark(tmu,f"DATA HIGH again — Tmu = {(tmu-c[-1])*1e6:.2f} us","gd",13)
xg0,xg1=X(c[-1]),X(tmu if tmu else c[-1])
s.append(f'<text x="{(xg0+xg1)/2:.1f}" y="{Y+80}" class="gaplab" text-anchor="middle">gap: Error Flag = NOT PV = 0</text>')

svg=f'''<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="222" viewBox="0 0 {W} 222">
<style>.ttl{{font:600 12.5px system-ui,sans-serif;fill:#0f172a}}
.sub{{font:10.5px ui-monospace,monospace;fill:#64748b}}
.lbl{{font:10.5px ui-monospace,monospace;fill:#475569}}
.ann{{font:10px system-ui,sans-serif;fill:#475569}}
.fld{{font:9.5px system-ui,sans-serif;fill:#334155}}
.gaplab{{font:9.5px system-ui,sans-serif;fill:#94a3b8}}
.ax{{stroke:#e2e8f0;stroke-width:1}}
.gd{{stroke:#94a3b8;stroke-width:1;stroke-dasharray:3 3}} .gdt{{fill:#475569}}
.rg{{stroke:#cbd5e1;stroke-width:0.7}}
.f1{{stroke:#047857;stroke-width:1.5}} .f1t{{fill:#047857;font-weight:600}}
.bad{{stroke:#dc2626;stroke-width:1.3;stroke-dasharray:2 2}}
.badt{{fill:#dc2626;font-weight:600}}
.key{{font:10px system-ui,sans-serif;fill:#64748b}}</style>
<rect width="100%" height="100%" fill="#ffffff"/>
{chr(10).join(s)}
<line x1="{PAD}" y1="206" x2="{PAD+22}" y2="206" class="rg"/>
<text x="{PAD+28}" y="209" class="key">{KEY}</text>
</svg>'''
open(out,"w").write(svg)
print(f"wrote {out}  T={T*1e9:.1f}ns  raw={raw:08X} pv={pv} zpd={zpd} pd={pd} ts={ts}")
