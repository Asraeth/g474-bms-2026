"""Copy or verify the generated protocol bundle in the separate GUI fork."""
import argparse,pathlib,shutil,sys
p=argparse.ArgumentParser();p.add_argument('gui',type=pathlib.Path);p.add_argument('--check',action='store_true');a=p.parse_args()
root=pathlib.Path(__file__).resolve().parents[1]
if not (a.gui/'bms_gui.py').is_file():sys.exit('Destination is not the GUI fork')
files={root/'can'/n:a.gui/'protocol'/n for n in ['bms_can1.dbc','ivt_can2.dbc','charger_can3.dbc','registry.json']}
files[root/'can/bms_model.json']=a.gui/'bms_model.json'
files[root/'can/bms_model_bench.json']=a.gui/'bms_model_bench.json'
changed=[]
for src,dst in files.items():
 if not dst.exists() or src.read_bytes()!=dst.read_bytes():
  changed.append(str(dst))
  if not a.check:dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(src,dst)
if a.check and changed:sys.exit('Protocol drift: '+', '.join(changed))
print('PASS GUI protocol bundle '+('verified' if a.check else 'synchronized'))
