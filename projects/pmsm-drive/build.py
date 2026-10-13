"""Build and execute the host C tests; optionally build the same core for browsers."""
from pathlib import Path
import argparse, subprocess, os
p=argparse.ArgumentParser();p.add_argument('--gcc',default='gcc');p.add_argument('--clang');a=p.parse_args()
root=Path(__file__).resolve().parent
os.environ['PATH']=str(Path(a.gcc).resolve().parent)+os.pathsep+os.environ.get('PATH','')
build=root/'build';build.mkdir(exist_ok=True)
sources=[root/'firmware'/name for name in ('math_port.c','plant.c','control.c','runtime.c','protocol.c')]
exe=build/('servo-tests.exe' if os.name=='nt' else 'servo-tests')
subprocess.run([a.gcc,'-std=c11','-O2','-Wall','-Wextra','-Werror','-fno-builtin',*map(str,sources),str(root/'firmware/tests.c'),'-lm','-o',str(exe)],check=True)
output=subprocess.check_output([str(exe)],text=True)
(root/'native-validation.json').write_text(output,encoding='utf-8')
print(output)
if a.clang:
    subprocess.run([a.clang,'--target=wasm32-unknown-unknown','-std=c11','-O2','-nostdlib','-ffreestanding','-fno-builtin','-Wl,--no-entry','-Wl,--export-all','-Wl,--allow-undefined','-Wl,--initial-memory=262144','-Wl,--max-memory=1048576',*map(str,sources),'-o',str(root/'servo.wasm')],check=True)
    print('C core compiled to servo.wasm')
