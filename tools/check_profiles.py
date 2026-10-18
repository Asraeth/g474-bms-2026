"""Compile real configuration profiles and prove invalid routing is rejected."""
import json,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];os.chdir(root)
out=root/'test/build/profiles';out.mkdir(parents=True,exist_ok=True)
cc=os.environ.get('CC') or shutil.which('gcc') or r'C:\msys64\mingw64\bin\gcc.exe'
includes=['Core/Inc','Core/custom_lib/inc','Drivers/STM32G4xx_HAL_Driver/Inc','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include']
flags=['-std=gnu11','-fsyntax-only','-DUSE_HAL_DRIVER','-DSTM32G474xx','-DBMS_IGNORE_LOCAL_CONFIG']+['-I'+i for i in includes]
cases=[('lead',1,'',True),('v3',2,'',True),('bench',3,'',True),('digital',4,'',True),('digital_on_arm',4,'#define __arm__ 1\n',False),
 ('duplicate_temperature',3,'#include "profiles/board_bench.h"\n#undef BMS_TEMP2_GPIO\n#define BMS_TEMP2_GPIO 1\n',False),
 ('temperature_out_of_range',3,'#include "profiles/board_bench.h"\n#undef BMS_TEMP1_GPIO\n#define BMS_TEMP1_GPIO 11\n',False),
 ('led_temperature_overlap',3,'#include "profiles/board_bench.h"\n#undef BMS_PROFILE_LED_RED_GPIO\n#define BMS_PROFILE_LED_RED_GPIO 1\n',False),
 ('duplicate_leds',3,'#include "profiles/board_bench.h"\n#undef BMS_PROFILE_LED_RED_GPIO\n#define BMS_PROFILE_LED_RED_GPIO 10\n',False)]
results=[]
for name,profile,prefix,ok in cases:
 source=out/(name+'.c');source.write_text(prefix+'#include "bms_config.h"\n')
 r=subprocess.run([cc,*flags,'-DBMS_BOARD_PROFILE='+str(profile),'-DBMS_SIMULATION_BUILD',str(source)],capture_output=True,text=True)
 (out/(name+'.log')).write_text(r.stdout+r.stderr)
 passed=(r.returncode==0)==ok;results.append({'name':name,'pass':passed,'expected_compile':ok})
 if not passed:print(r.stdout+r.stderr);sys.exit('FAIL profile '+name)
(out/'results.json').write_text(json.dumps(results,indent=2))
print(f'PASS {len(results)} profile acceptance/rejection contracts')
