from pathlib import Path
import shutil, subprocess
base=Path(r'C:\Espressif\frameworks\idf-extract\esp-idf-5.5.4\components\bt\controller\lib_esp32')
zip=Path(r'C:\Espressif\bt-lib-exact.zip'); tmp=Path(r'C:\Espressif\bt-lib-exact')
subprocess.run(['curl.exe','-L','--retry','3','--fail','--output',str(zip),'https://codeload.github.com/espressif/esp32-bt-lib/zip/a348e3fbc75411acd88357af9e1'],check=True)
shutil.rmtree(tmp,ignore_errors=True); tmp.mkdir(); subprocess.run(['tar','-xf',str(zip),'-C',str(tmp)],check=True)
src=next(tmp.iterdir()); shutil.rmtree(base,ignore_errors=True); base.mkdir(parents=True); shutil.copytree(src,base,dirs_exist_ok=True)
print(list(base.rglob('libbtdm_app.a')))
