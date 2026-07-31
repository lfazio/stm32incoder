import csv, sys, statistics as st

def load(path):
    rows=[]
    with open(path) as f:
        r=csv.reader(f); next(r)
        for a in r:
            if len(a)<3: continue
            rows.append((float(a[0]), int(a[1]), int(a[2])))
    return rows

def analyse(path, label, expect_pd, expect_pv):
    rows=load(path)
    # reconstruct edges
    fall=[]; rise=[]
    prev=rows[0]
    for t,c,d in rows[1:]:
        if c==0 and prev[1]==1: fall.append(t)
        if c==1 and prev[1]==0: rise.append(t)
        prev=(t,c,d)
    # data as step function
    dt=[r[0] for r in rows]; dv=[r[2] for r in rows]
    import bisect
    def data_at(t):
        i=bisect.bisect_right(dt,t)-1
        return dv[max(i,0)]
    def next_data_change_after(t):
        i=bisect.bisect_right(dt,t)
        base=data_at(t)
        while i<len(rows):
            if dv[i]!=base: return dt[i]
            i+=1
        return None
    # group falling edges into cycles (gap > 10us starts a new cycle)
    # Split cycles on a gap relative to the clock period, not a fixed 10 us:
    # at the 100 kHz end T *is* 10 us, so a fixed threshold shreds every frame.
    deltas=sorted(fall[i+1]-fall[i] for i in range(len(fall)-1))
    med=deltas[len(deltas)//2] if deltas else 1e-6
    thresh=max(med*2.5, 1e-6)
    cycles=[]; cur=[fall[0]]
    for t in fall[1:]:
        if t-cur[-1] > thresh: cycles.append(cur); cur=[t]
        else: cur.append(t)
    cycles.append(cur)

    periods=[]; counts=[]; tmus=[]; hand=[]; frames=[]; gaplv=[]; efl=[]
    for cy in cycles:
        counts.append(len(cy))
        if len(cy)>1:
            periods += [cy[i+1]-cy[i] for i in range(len(cy)-1)]
        # rising edges inside this cycle
        # window must scale with the clock period: the last rising edge is
        # half a period after the last falling edge, which is 2.9 us at
        # 175 kHz -- a fixed 1 us window silently drops it.
        Tloc=(cy[-1]-cy[0])/(len(cy)-1) if len(cy)>1 else 1e-6
        r_in=[r for r in rise if cy[0] <= r <= cy[-1]+Tloc]
        if len(r_in)==len(cy)==32:
            bits=[data_at(r+2e-9) for r in r_in]
            v=0
            for b in bits: v=(v<<1)|b
            frames.append(v)
        # Tmu: last falling edge -> DATA goes HIGH *and stays high* until the
        # next cycle. Scan forward for the final 0->1 data transition before the
        # next cycle's first falling edge (or end of capture).
        last=cy[-1]
        nxt = None
        for c2 in cycles:
            if c2[0] > cy[-1]: nxt = c2[0]; break
        limit = nxt if nxt else dt[-1]
        i=bisect.bisect_right(dt,last)
        tm=None
        while i<len(rows) and dt[i] < limit:
            if dv[i]==1 and dv[i-1]==0: tm=dt[i]
            i+=1
        if tm: tmus.append(tm-last)
        # gap level = data level ~1us after last rising edge
        # sample mid-gap, not right after the last rising edge: the emulator
        # needs its end-of-frame interrupt to run before it can present the
        # Error Flag, so the line still shows D0 for a short while.
        if r_in: gaplv.append(data_at(r_in[-1]+10e-6))
        # how long until the Error Flag actually appears
        if r_in:
            j=bisect.bisect_right(dt,r_in[-1]); base=data_at(r_in[-1]+1e-9)
            while j<len(rows) and dt[j]-r_in[-1] < 15e-6:
                if dv[j]!=base: efl.append(dt[j]-r_in[-1]); break
                j+=1
        # handover: first falling edge -> first data change after it
        c=next_data_change_after(cy[0])
        if c is not None and c-cy[0] < 1e-6: hand.append(c-cy[0])

    print(f"=== {label} ===")
    print(f"cycles                 : {len(cycles)}")
    from collections import Counter
    print(f"clocks per cycle       : {dict(Counter(counts))}")
    if periods:
        print(f"clock period           : min {min(periods)*1e9:.1f} ns  "
              f"mean {st.mean(periods)*1e9:.2f} ns  max {max(periods)*1e9:.1f} ns "
              f"-> {1/st.mean(periods)/1e6:.4f} MHz")
    if tmus:
        print(f"Tmu (last fall->DATA hi): min {min(tmus)*1e6:.2f} us  "
              f"mean {st.mean(tmus)*1e6:.2f} us  max {max(tmus)*1e6:.2f} us")
    if gaplv:
        print(f"gap level              : {dict(Counter(gaplv))}  (0=LOW)")
    if efl:
        print(f"last rising -> Error Flag: mean {st.mean(efl)*1e6:.2f} us  max {max(efl)*1e6:.2f} us")
    if hand:
        print(f"1st fall -> DATA valid : min {min(hand)*1e9:.0f} ns  "
              f"mean {st.mean(hand)*1e9:.0f} ns  max {max(hand)*1e9:.0f} ns  (n={len(hand)})")
    ok=bad=0
    for v in frames:
        pv=(v>>31)&1; zpd=(v>>30)&1; pd=(v>>11)&0x7FFFF
        if pd==expect_pd and pv==expect_pv and zpd==1: ok+=1
        else:
            bad+=1
            if bad<=3: print(f"   MISMATCH raw={v:08X} pv={pv} zpd={zpd} pd={pd}")
    print(f"decoded frames         : {len(frames)}  ok={ok} bad={bad}")
    return frames

analyse(sys.argv[1], sys.argv[2], int(sys.argv[3]), int(sys.argv[4]))
