from pathlib import Path
import shutil, subprocess
base=Path(r'C:\Espressif\frameworks\idf-extract\esp-idf-5.5.4\components\heap\tlsf')
zip=Path(r'C:\Espressif\tlsf-exact.zip'); tmp=Path(r'C:\Espressif\tlsf-exact')
subprocess.run(['curl.exe','-L','--retry','3','--fail','--output',str(zip),'https://codeload.github.com/espressif/tlsf/zip/2867f6883a12920b1969ff9624c0ab0e4185c2ce'],check=True)
shutil.rmtree(tmp,ignore_errors=True); tmp.mkdir(); subprocess.run(['tar','-xf',str(zip),'-C',str(tmp)],check=True)
src=next(tmp.iterdir()); shutil.rmtree(base,ignore_errors=True); base.mkdir(parents=True); shutil.copytree(src,base,dirs_exist_ok=True)
print((base/'include'/'tlsf.h').exists(), list(base.iterdir()))
