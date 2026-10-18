"""Native ARM build, strict production warnings; no IDE installation state is changed."""
import argparse,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];os.chdir(root)
p=argparse.ArgumentParser();p.add_argument('mode',choices=['debug','release']);p.add_argument('--profile',type=int,default=3);a=p.parse_args()
cc=os.environ.get('ARM_GCC') or shutil.which('arm-none-eabi-gcc')
if not cc:
 candidates=list(pathlib.Path(r'C:\ST\STM32CubeIDE_2.2.0').glob('**/arm-none-eabi-gcc.exe'))
 if not candidates:sys.exit('ARM GCC not found; set ARM_GCC')
 cc=str(candidates[0])
out=root/'build'/(a.mode+'-profile'+str(a.profile));out.mkdir(parents=True,exist_ok=True)
mcu=['-mcpu=cortex-m4','-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-mthumb']
flags=mcu+['-DUSE_HAL_DRIVER','-DSTM32G474xx','-DBMS_BOARD_PROFILE='+str(a.profile),'-std=gnu11','-ffunction-sections','-fdata-sections','-Wall','-Werror']+(['-O0','-g3','-DDEBUG'] if a.mode=='debug' else ['-Os','-g0'])
flags+=['-I'+i for i in ['Core/Inc','Core/custom_lib/inc','Drivers/STM32G4xx_HAL_Driver/Inc','Drivers/STM32G4xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32G4xx/Include','Drivers/CMSIS/Include']]
sources=sorted(list(pathlib.Path('Core/Src').glob('*.c'))+list(pathlib.Path('Core/custom_lib/src').glob('*.c'))+list(pathlib.Path('Drivers/STM32G4xx_HAL_Driver/Src').glob('*.c')))
objs=[]
with (out/'build.log').open('w') as log:
 for src in sources:
  obj=out/(str(src).replace('\\','_').replace('/','_')+'.o');extra=['-Wextra','-Wshadow','-Wformat=2','-Wno-unused-parameter','-Wno-old-style-declaration'] if str(src).startswith('Core') else []
  r=subprocess.run([cc,*flags,*extra,'-c',str(src),'-o',str(obj)],stdout=log,stderr=log)
  if r.returncode:print((out/'build.log').read_text());sys.exit(r.returncode)
  objs.append(str(obj))
 obj=out/'startup.o'
 subprocess.run([cc,*mcu,'-x','assembler-with-cpp','-c','Core/Startup/startup_stm32g474retx.s','-o',str(obj)],check=True,stdout=log,stderr=log);objs.append(str(obj))
 elf=out/'g474-bms.elf'
 r=subprocess.run([cc,*mcu,'-T','STM32G474RETX_FLASH.ld','--specs=nosys.specs','--specs=nano.specs','-u','_printf_float','-u','_scanf_float','-Wl,-Map='+str(out/'g474-bms.map'),'-Wl,--gc-sections','-static',*objs,'-Wl,--start-group','-lc','-lm','-Wl,--end-group','-o',str(elf)],stdout=log,stderr=log)
 if r.returncode:print((out/'build.log').read_text());sys.exit(r.returncode)
subprocess.run([str(pathlib.Path(cc).with_name('arm-none-eabi-size.exe' if os.name=='nt' else 'arm-none-eabi-size')),str(elf)],check=True)
print('PASS ARM '+a.mode+' profile '+str(a.profile)+'; production -Werror')
