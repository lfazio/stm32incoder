import json,urllib.request,sys
URL="http://127.0.0.1:10530"
def call(method, params, tid=1):
    req=urllib.request.Request(URL, method="POST",
        data=json.dumps({"jsonrpc":"2.0","id":tid,"method":method,"params":params}).encode(),
        headers={"Content-Type":"application/json","Accept":"application/json, text/event-stream"})
    raw=urllib.request.urlopen(req, timeout=180).read().decode()
    for line in raw.splitlines():
        line=line.strip()
        if line.startswith("data: "): line=line[6:]
        if line.startswith("{"):
            d=json.loads(line)
            if "error" in d: raise RuntimeError(d["error"])
            return d["result"]
    raise RuntimeError("no json in: "+raw[:300])
def tool(name, args):
    r=call("tools/call", {"name":name,"arguments":args})
    txts=[c.get("text","") for c in r.get("content",[]) if c.get("type")=="text"]
    body="\n".join(txts)
    if r.get("isError"): raise RuntimeError(body)
    try: return json.loads(body)
    except Exception: return body
