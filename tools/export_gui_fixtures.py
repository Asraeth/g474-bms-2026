"""Export compact actual-firmware CAN/SoC fixtures to the separate GUI fork.

Run the can_layout and soc_model scenarios first. These are recorded production
outputs, not a second hand-written protocol encoder.
"""
import argparse,csv,pathlib,shutil
p=argparse.ArgumentParser();p.add_argument('gui',type=pathlib.Path);p.add_argument('--results',type=pathlib.Path,default=pathlib.Path('test/build/profile4-normal'));a=p.parse_args()
if not (a.gui/'bms_gui.py').is_file():p.error('GUI fork required')
last={}
with (a.results/'can_layout.csv').open(newline='') as f:
 reader=csv.DictReader(f);fields=reader.fieldnames
 for row in reader:
  if row['from_mcu']=='1' and row['bus']=='1':last[(row['id'],row['ext'])]=row
target=a.gui/'protocol';target.mkdir(exist_ok=True)
with (target/'firmware_trace.csv').open('w',newline='') as f:
 writer=csv.DictWriter(f,fieldnames=fields,lineterminator='\n');writer.writeheader();writer.writerows(sorted(last.values(),key=lambda r:float(r['time_s'])))
shutil.copyfile(a.results/'soc_model.csv',target/'soc_model.csv')
print(f'PASS exported {len(last)} actual-firmware CAN messages and SoC curve')
