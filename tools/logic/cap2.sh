#!/bin/bash
# cap2.sh <outdir> <console-cmd> <afterTriggerSeconds> <rate>
stty -F /dev/ttyACM0 921600 raw -echo -crtscts
# value is set by the caller, not here
python3 - "$1" "$2" "$3" "$4" <<'PY'
import sys,time,os
sys.path.insert(0,'.')
from mcp import tool
d,cmd,after,rate=sys.argv[1],sys.argv[2],float(sys.argv[3]),int(sys.argv[4])
cap = tool("start_capture", {"deviceId":"CEF34EB8B9845F50",
  "logicDeviceConfiguration":{"logicChannels":{"digitalChannels":[0,1]},
    "digitalSampleRate":rate,"digitalThresholdVolts":3.3},
  "captureConfiguration":{"bufferSizeMegabytes":2048,
    "digitalCaptureMode":{"triggerType":2,"triggerChannelIndex":0,"afterTriggerSeconds":after}}})
cid=cap["captureId"]; time.sleep(1.0)
open('/dev/ttyACM0','wb',buffering=0).write((cmd+"\r").encode())
tool("wait_capture", {"captureId": cid})
os.makedirs(d, exist_ok=True)
tool("export_raw_data_csv", {"captureId": cid,"directory": os.path.abspath(d),
     "analogDownsampleRatio":1,"logicChannels":{"digitalChannels":[0,1]}})
PY
