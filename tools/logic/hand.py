import csv,bisect,sys,statistics as st
def run(path,label):
    rows=[]
    with open(path) as f:
        r=csv.reader(f); next(r)
        for a in r:
            if len(a)>=3: rows.append((float(a[0]),int(a[1]),int(a[2])))
    dt=[r[0] for r in rows]
    fall=[rows[k][0] for k in range(1,len(rows)) if rows[k][1]==0 and rows[k-1][1]==1]
    cyc=[];cur=[fall[0]]
    for t in fall[1:]:
        if t-cur[-1]>10e-6: cyc.append(cur);cur=[t]
        else: cur.append(t)
    cyc.append(cur)
    per=[];hand=[]
    for c in cyc:
        if len(c)!=32: continue
        per+=[c[i+1]-c[i] for i in range(31)]
        i=bisect.bisect_right(dt,c[0]); base=rows[max(i-1,0)][2]
        while i<len(rows) and dt[i]-c[0] < 1e-6:
            if rows[i][2]!=base: hand.append(dt[i]-c[0]); break
            i+=1
    mp=st.mean(per); budget=mp/2
    print(f"=== {label} ===")
    print(f"  clock period      : {mp*1e9:.2f} ns   -> half-period budget {budget*1e9:.1f} ns")
    if hand:
        print(f"  handover (n={len(hand)})   : min {min(hand)*1e9:.0f}  mean {st.mean(hand)*1e9:.0f}  "
              f"max {max(hand)*1e9:.0f} ns")
        print(f"  worst-case margin : {(budget-max(hand))*1e9:.0f} ns "
              f"({(budget-max(hand))/budget*100:.1f} % of budget)")
    else: print("  no data transition at bit 0 -- did you set 'err on'?")
run(sys.argv[1],sys.argv[2])
