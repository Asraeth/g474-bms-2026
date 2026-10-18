"""Run migrated independent PEC/counter/IVT contracts against final drivers.
Wrapper and virtual-bench safety contracts are exercised by the integrated
peripheral simulator (test/integration_cases.h and test/tests.c).
"""
import argparse,json,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];os.chdir(root)
p=argparse.ArgumentParser();p.add_argument('--sanitize',action='store_true');a=p.parse_args()
out=root/'test/build'/('migrated-ubsan' if a.sanitize else 'migrated');out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('gcc') or r'C:\msys64\mingw64\bin\gcc.exe'
flags=['-std=gnu11','-O1','-g','-Wall','-Wextra','-Werror','-Wno-int-to-pointer-cast','-Wno-pointer-to-int-cast','-DBMS_BOARD_PROFILE=1','-DBMS_IGNORE_LOCAL_CONFIG','-DUSE_HAL_DRIVER','-DSTM32G474xx']
flags+=['-I'+d for d in ['Core/Inc','Core/custom_lib/inc','Drivers/STM32G4xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include']]
if a.sanitize:flags+=['-fsanitize=undefined,float-cast-overflow','-fsanitize-undefined-trap-on-error','-fno-sanitize-recover=all']
cases={'ivt':['tests/ivt_s_test.c','Core/custom_lib/src/ivt_s.c'],'counter':['tests/adbms_counter_test.c','Core/custom_lib/src/bms_command_counter.c'],'transport':['test/migrated_transport.c','Core/custom_lib/src/bms_command_counter.c']}
results=[]
for name,sources in cases.items():
 binary=out/(name+('.exe' if os.name=='nt' else ''))
 r=subprocess.run([cc,*flags,*sources,'-o',str(binary),'-lm'],capture_output=True,text=True)
 (out/(name+'-build.log')).write_text(r.stdout+r.stderr)
 if r.returncode:print(r.stdout+r.stderr);sys.exit(r.returncode)
 try:r=subprocess.run([str(binary)],capture_output=True,text=True)
 except OSError as exc:
  (out/'results.json').write_text(json.dumps({'blocked':str(exc),'case':name},indent=2));raise
 (out/(name+'.log')).write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,end='')
 results.append({'name':name,'pass':r.returncode==0})
 if r.returncode:sys.exit(r.returncode)
(out/'results.json').write_text(json.dumps(results,indent=2))
print('PASS migrated IVT, counter and transport contracts')
