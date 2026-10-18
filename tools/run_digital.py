"""Native host runner; builds real firmware against the lead independent peripherals."""
import argparse, json, os, pathlib, shutil, subprocess, sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
os.chdir(ROOT)
p=argparse.ArgumentParser();p.add_argument('tests',nargs='*');p.add_argument('--variant');p.add_argument('--profile',type=int,default=4);p.add_argument('--sanitize',action='store_true');p.add_argument('--pessimistic',action='store_true');p.add_argument('--wrap',action='store_true');p.add_argument('--no-build',action='store_true');p.add_argument('--build-only',action='store_true');a=p.parse_args()
label='profile'+str(a.profile)+'-'+('ubsan' if a.sanitize else 'normal')+('-pessimistic' if a.pessimistic else '')+('-wrap' if a.wrap else '')+('-'+pathlib.Path(a.variant).stem if a.variant else '')
out=ROOT/'test'/'build'/label;out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('gcc') or r'C:\msys64\mingw64\bin\gcc.exe'
flags=['-std=gnu11','-g','-O1','-DBMS_SIMULATION_BUILD','-DBMS_BOARD_PROFILE='+str(a.profile),'-DBMS_IGNORE_LOCAL_CONFIG','-DSTM32G474xx','-DUSE_HAL_DRIVER','-include','test/include/cmsis_sim.h','-Wall','-Wextra','-Wno-unused-parameter','-Wno-old-style-declaration','-Wno-int-to-pointer-cast','-Wno-pointer-to-int-cast','-fstack-protector-strong']
flags += ['-I'+i for i in ['test/include','test/sim','Core/Inc','Core/custom_lib/inc','Drivers/STM32G4xx_HAL_Driver/Inc','Drivers/STM32G4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include']]
if a.variant:
 flags.remove('-DBMS_IGNORE_LOCAL_CONFIG');flags+=['-I'+str(pathlib.Path(a.variant).parent),'-DBMS_LOCAL_CONFIG_FILE="'+pathlib.Path(a.variant).name+'"']
if a.sanitize:flags+=['-fsanitize=undefined,float-cast-overflow','-fsanitize-undefined-trap-on-error','-fno-sanitize-recover=all']
if a.pessimistic:os.environ['SIM_PESSIMISTIC']='1'
if a.wrap:flags+=['-DSIM_TICK_OFFSET_MS=4294964296u']
sources=['Core/Src/bms_charger.c','Core/custom_lib/src/bms_charger_codec.c','Core/Src/bms_health.c', 'Core/custom_lib/src/ivt_s.c', 'Core/custom_lib/src/bms_command_counter.c', 'Core/Src/bms_can.c', 'Core/Src/bms_gpio.c', 'Core/Src/bms_ivt.c', 'Core/Src/bms_safety.c', 'Core/Src/bms_soc.c', 'Core/Src/bms_precharge.c', 'Core/Src/bms_inverter.c', 'Core/Src/bms_settings.c', 'Core/Src/uartDMA.c', 'Core/Src/gpio.c', 'Core/Src/dma.c', 'Core/Src/fdcan.c', 'Core/Src/spi.c', 'Core/Src/tim.c', 'Core/Src/usart.c', 'Core/Src/stm32g4xx_hal_msp.c', 'Core/custom_lib/src/bms_cmdlist.c', 'Core/custom_lib/src/bms_libWrapper.c', 'Core/custom_lib/src/bms_mcuWrapper.c', 'Core/custom_lib/src/bms_utility.c', 'test/sim/sim_core.c', 'test/sim/sim_can.c', 'test/sim/sim_adbms.c', 'test/sim/sim_ivt.c', 'test/sim/sim_flash.c', 'test/tests.c']
binary=out/('bms_tests.exe' if os.name=='nt' else 'bms_tests')
if not a.no_build:
 objects=[]
 with (out/'build.log').open('w') as log:
  for src in sources+['Core/Src/main.c']:
   obj=out/(src.replace('/','_')+'.o');extra=['-Dmain=firmware_main','-DError_Handler=Error_Handler_firmware'] if src=='Core/Src/main.c' else []
   r=subprocess.run([cc,*flags,*extra,'-c',src,'-o',str(obj)],stdout=log,stderr=log)
   if r.returncode:print((out/'build.log').read_text());sys.exit(r.returncode)
   objects.append(str(obj))
  r=subprocess.run([cc,*objects,'-o',str(binary),'-lm','-fstack-protector-strong'],stdout=log,stderr=log)
  if r.returncode:print((out/'build.log').read_text());sys.exit(r.returncode)
if a.build_only:
 print('BUILT '+label+'; not executed');sys.exit(0)
names=a.tests or subprocess.check_output([str(binary),'--list'],text=True).split()
results=[]
for name in names:
 r=subprocess.run([str(binary),name,str(out)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
 (out/(name+'.result.txt')).write_text(r.stdout)
 results.append({'name':name,'pass':r.returncode==0,'returncode':r.returncode})
 print(('PASS ' if r.returncode==0 else 'FAIL ')+name,flush=True)
 if r.returncode:print(r.stdout[-7000:],flush=True)
(out/'results.json').write_text(json.dumps(results,indent=2))
print(str(sum(r['pass'] for r in results))+' passed / '+str(len(results))+' total')
sys.exit(any(not r['pass'] for r in results))
