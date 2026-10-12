"""Build the portable C core and optional official FreeRTOS Windows host port."""
import argparse
import os
from pathlib import Path
import subprocess

parser=argparse.ArgumentParser()
parser.add_argument('--gcc',default='gcc')
parser.add_argument('--clang',help='Path to a WebAssembly-capable clang compiler')
parser.add_argument('--freertos',help='FreeRTOS-Kernel V11.2.0 source directory')
args=parser.parse_args()
root=Path(__file__).resolve().parent
firmware=root/'firmware'
build=root/'build'
build.mkdir(exist_ok=True)
suffix='.exe' if os.name=='nt' else ''
def run(command):
    subprocess.run([str(x) for x in command],check=True)
run([args.gcc,'-std=c11','-O2','-Wall','-Wextra',firmware/'device.c',firmware/'tests.c','-o',build/('device-tests'+suffix)])
run([build/('device-tests'+suffix)])
if args.clang:
    run([args.clang,'--target=wasm32-unknown-unknown','-std=c11','-Oz','-nostdlib','-ffreestanding','-fno-builtin',
         '-Wl,--no-entry','-Wl,--export-all','-Wl,--initial-memory=262144','-Wl,--max-memory=1048576',
         firmware/'device.c','-o',root/'device.wasm'])
if args.freertos:
    if os.name!='nt':
        raise SystemExit('This FreeRTOS build uses the official Windows MSVC-MingW port.')
    kernel=Path(args.freertos).resolve()
    sources=[firmware/'device.c',firmware/'freertos_app.c',kernel/'tasks.c',kernel/'queue.c',kernel/'list.c',kernel/'portable/MemMang/heap_4.c',kernel/'portable/MSVC-MingW/port.c']
    run([args.gcc,'-std=c11','-O2','-D_WIN32_WINNT=0x0600','-I'+str(firmware/'host_compat'),'-I'+str(firmware),'-I'+str(kernel/'include'),'-I'+str(kernel/'portable/MSVC-MingW'),*sources,'-lwinmm','-o',build/'freertos-host.exe'])
    run([build/'freertos-host.exe'])
