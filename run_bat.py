import os

tdm_path = os.path.abspath('../Embedded_Tools/TDM_GCC_32/bin/')
arm_gcc_path = os.path.abspath('../Embedded_Tools/gcc-arm-none-eabi/bin/')

os.environ['PATH'] = tdm_path + os.pathsep + arm_gcc_path + os.pathsep + os.environ['PATH']
pc_bat = os.path.abspath("scripts\\build.bat")
mcu_app30_bat = os.path.abspath("scripts\\build_app30.bat")
mcu_app31_bat = os.path.abspath("scripts\\build_app31.bat")

os.system(pc_bat)
os.system(mcu_app30_bat)
os.system(mcu_app31_bat)