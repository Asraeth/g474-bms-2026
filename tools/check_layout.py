"""Check actual ARM/host compiler register layout against datasheet golden bytes.

Each --target is mandatory evidence, never a silently skipped compiler check.
"""
import argparse,json,os,pathlib,shutil,subprocess,sys
root=pathlib.Path(__file__).resolve().parents[1];os.chdir(root)
p=argparse.ArgumentParser();p.add_argument('--target',choices=['arm','host'],required=True);a=p.parse_args()
expected={'probe_cfa':'85a5985a0216','probe_cfb':'bc3a12aa0180','probe_pwma':'21436587a9cb','probe_pwmb':'ed1f00000000','probe_adcv':'03f6','probe_adsv':'01e9','probe_adax':'05da'}
out=root/'test/build/layout';out.mkdir(parents=True,exist_ok=True)
if a.target=='arm':
 cc=os.environ.get('ARM_GCC') or shutil.which('arm-none-eabi-gcc')
 if not cc:
  candidates=list(pathlib.Path(r'C:\ST\STM32CubeIDE_2.2.0').glob('**/arm-none-eabi-gcc.exe'))
  if not candidates:sys.exit('ARM GCC required')
  cc=str(candidates[0])
 obj=out/'probe_arm.o'
 subprocess.run([cc,'-mcpu=cortex-m4','-mthumb','-std=gnu11','-O2','-fdata-sections','-ICore/custom_lib/inc','-c','test/layout_probe.c','-o',str(obj)],check=True)
 objcopy=str(pathlib.Path(cc).with_name('arm-none-eabi-objcopy'+('.exe' if os.name=='nt' else '')))
 actual={}
 for symbol in expected:
  raw=out/(symbol+'.arm.bin')
  subprocess.run([objcopy,'-O','binary','--only-section=.rodata.'+symbol,str(obj),str(raw)],check=True)
  actual[symbol]=raw.read_bytes().hex()
else:
 cc=os.environ.get('CC') or shutil.which('gcc') or r'C:\msys64\mingw64\bin\gcc.exe'
 source=out/'dump_host.c'
 declarations=''.join('extern const '+typ+' '+sym+';' for sym,typ in zip(expected,['ad68_cfa_t','ad68_cfb_t','ad68_pwma_t','ad68_pwmb_t','ADCV_t','ADSV_t','ADAX_t']))
 dumps=''.join('dump("'+sym+'", &'+sym+',sizeof '+sym+');' for sym in expected)
 source.write_text('#include <stdio.h>\n#include "bms_datatypes.h"\n#include "bms_cmdlist.h"\n'+declarations+'\nstatic void dump(const char*n,const void*p,size_t z){printf("%s ",n);for(size_t i=0;i<z;i++)printf("%02x",((const unsigned char*)p)[i]);puts("");}\nint main(void){'+dumps+'return 0;}')
 binary=out/('dump_host.exe' if os.name=='nt' else 'dump_host')
 subprocess.run([cc,'-std=gnu11','-ICore/custom_lib/inc','test/layout_probe.c',str(source),'-o',str(binary)],check=True)
 actual=dict(line.split() for line in subprocess.check_output([str(binary)],text=True).splitlines())
for symbol,want in expected.items():
 if actual[symbol]!=want:sys.exit(f'FAIL {a.target} {symbol}: {actual[symbol]} != {want}')
(out/(a.target+'-results.json')).write_text(json.dumps({'target':a.target,'pass':True,'vectors':actual},indent=2))
print(f'PASS {a.target} register layout: {len(expected)} datasheet vectors')
