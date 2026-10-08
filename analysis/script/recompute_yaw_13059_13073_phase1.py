"""Recompute CAD-derived line sensor and BMI geometry for yaw EKF Phase 1.
Inputs are read only. Run inside the repository using numpy:
python analysis/script/recompute_yaw_13059_13073_phase1.py
"""
import csv
import json
import pathlib
import re
import subprocess
import sys
import numpy as np
ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "analysis/yaw_13059_13073_phase1"
def footprints(path):
    text = path.read_text(encoding="utf-8-sig")
    out = {}
    for b in re.split(r"\(\s*footprint\s+", text)[1:]:
        ref = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', b) or re.search(r'\(fp_text\s+reference\s+"([^"]+)"', b)
        at = re.search(r'^\s*\(at\s+([^()\n]+)\)', b, re.M)
        if not ref or not at:
            continue
        out[ref.group(1)] = {"at": list(map(float, at.group(1).split())),
            "nets": {int(a): s for a, s in re.findall(r'\(pad\s+"(\d+)"[\s\S]{0,400}?\(net\s+\d+\s+"([^"]+)"', b)}}
    return out
def read_csv(path):
    with path.open(encoding="utf-8",newline="") as f:
        return list(csv.DictReader(f))
def write_csv(path, rows):
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f,fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(rows)
def main():
    # Re-run the repository's AP214 assembler, rather than trusting stale CSV outputs.
    # This refreshes analysis/yi_12838_12848/* geometry derived files only.
    source = ROOT / "analysis/script/analyze_yi_step.py"
    for name in ("Machine/robotrace_v3.STEP","Circuit/robotrace_v2_Linesensor/robotrace_v2_Linesensor.kicad_pcb",
                 "Circuit/robotrace_v2_main_v2/robotrace_v2_main_v2.kicad_pcb"):
        if not (ROOT/name).is_file():
            raise FileNotFoundError(name)
    subprocess.run([sys.executable,str(source)],cwd=ROOT,check=True)
    old = ROOT / "analysis/yi_12838_12848"
    cs = {row["ref"]:row for row in read_csv(old/"sensor_centers.csv") if row["board"]=="line"}
    line = footprints(ROOT/"Circuit/robotrace_v2_Linesensor/robotrace_v2_Linesensor.kicad_pcb")
    mainpcb = footprints(ROOT/"Circuit/robotrace_v2_main_v2/robotrace_v2_main_v2.kicad_pcb")
    fw = (ROOT/"robotrace_v2/Core/Src/main.c").read_text(encoding="utf-8-sig")
    init = fw.split("static void MX_ADC1_Init(void)\n{",1)[1].split("static void MX_ADC2_Init",1)[0]
    ranks = {int(rank):int(ch) for ch,rank in re.findall(r'sConfig.Channel\s*=\s*ADC_CHANNEL_(\d+);[\s\S]{0,150}?sConfig.Rank\s*=\s*(\d+);',init)}
    j4 = {int(s.split("ADC_IN")[1]): pin for pin,s in mainpcb["J4"]["nets"].items() if s.startswith("ADC_IN")}
    qpin = {q:int(re.search(r"J1-Pin_(\d+)",d["nets"][1]).group(1)) for q,d in line.items() if re.fullmatch(r"Q\d+",q)}
    adc2q = {ch:next(q for q,p in qpin.items() if p==15-pin) for ch,pin in j4.items()}
    sensors=[]
    emitters=[]
    for ch in range(10):
        q=adc2q[ranks[ch+1]]
        d="D"+q[1:]
        c=cs[q]
        x=float(c["forward_mm"])
        y=-float(c["X_mm"])
        u,v,*_=line[q]["at"]
        du,dv,*_=line[d]["at"]
        # Extract the PCB affine placement from the previously validated AP214 analysis.
        # For this STEP: X_body=(40.151700071-v), Y_body=u, Z_body=-6.16 on PCB.
        fx=40.151700071-v
        fd=40.151700071-dv
        if abs(fx-x)>.03 or abs(y-u)>.03:
            raise AssertionError("STEP/KiCad footprint mismatch: "+q)
        sensors.append({"channel":ch,"reference":q,"forward_mm":x,"right_mm":y,"up_mm":-6.16})
        emitters.append({"channel":ch,"receiver":q,"emitter":d,"receiver_forward_mm":x,
            "receiver_right_mm":y,"emitter_forward_mm":fd,"emitter_right_mm":du,
            "geometric_mid_forward_mm":(x+fd)/2,
            "center_separation_mm":float(np.hypot(fd-x,du-y))})
    if len(sensors)!=10 or not all(sensors[i]["right_mm"]<sensors[i+1]["right_mm"] for i in range(9)):
        raise AssertionError("ADC channel order / FFC parity mismatch")
    axle=json.loads((old/"axle_geometry.json").read_text())
    solids=read_csv(old/"step_solids.csv")
    imu=[]
    for row in solids:
        if row["name"]!="robotrace_v2_main_v2":
            continue
        xmin,xmax,zmin,zmax,ymin,ymax=[float(row[k]) for k in ("xmin","xmax","zmin","zmax","ymin","ymax")]
        dx=xmax-xmin;dz=zmax-zmin;dy=ymax-ymin;cx=(xmax+xmin)/2;cz=(zmax+zmin)/2;cy=(ymax+ymin)/2
        if abs(dx-3)<.02 and abs(dz-4.5)<.02 and abs(dy-.95)<.02 and abs(cx-.09917)<.3 and abs(cz+23.67523)<.3:
            imu.append({"forward_mm":cz-axle["midpoint_Z_mm"],"right_mm":-cx,"up_mm":cy-8.25})
    if len(imu)!=1:
        raise AssertionError("BMI088 STEP package not uniquely identified")
    OUT.mkdir(parents=True,exist_ok=True)
    write_csv(OUT/"sensor_geometry_from_cad.csv",sensors)
    write_csv(OUT/"emitter_geometry_from_cad.csv",emitters)
    (OUT/"geometry_from_cad.json").write_text(json.dumps({
        "axle_mid_cad_Z_mm":axle["midpoint_Z_mm"],"axle_spacing_front_rear_mm":axle["wheelbase_mm"],
        "bmi_package_center_body_mm":imu[0],
        "cad_to_body_homogeneous":[[0,0,1,-axle["midpoint_Z_mm"]],[-1,0,0,0],[0,1,0,-8.25],[0,0,0,1]],
        "line_pcb_to_body_homogeneous":[[0,-1,0,40.151700071],[1,0,0,0],[0,0,1,-6.16],[0,0,0,1]],
        "optical_center_measured":False,
        "effective_encoder_track_measured":False
        },ensure_ascii=False,indent=2),encoding="utf-8")
    print("Phase 1 CAD recomputation OK:",len(sensors),"sensors, IMU",imu[0])
if __name__=="__main__":
    main()
