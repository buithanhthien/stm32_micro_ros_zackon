"""Merge ARM archives and flatten ROS package include directories for CubeIDE."""
from pathlib import Path
import shutil
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
install = root / 'build-linux/uros/firmware/mcu_ws/install'
out = root / 'micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros'
ar = root / '.tools/root/usr/bin/arm-none-eabi-ar'
out.mkdir(parents=True, exist_ok=True)
archives = sorted((install / 'lib').rglob('*.a'))
if not archives:
    raise SystemExit('No micro-ROS archives were built')
with tempfile.TemporaryDirectory(dir=root / 'build-linux') as tmp:
    combined = Path(tmp) / 'libmicroros.a'
    script = f'CREATE {combined}\n' + ''.join(f'ADDLIB {p}\n' for p in archives) + 'SAVE\nEND\n'
    subprocess.run([str(ar), '-M'], input=script, text=True, check=True)
    shutil.copy2(combined, out / 'libmicroros.a')
shutil.copytree(install / 'include', out / 'include', dirs_exist_ok=True)
for package in (out / 'include').iterdir():
    nested = package / package.name
    if nested.is_dir():
        shutil.copytree(nested, package, dirs_exist_ok=True)
print(f'Packed {len(archives)} archives: {out}')
