from building import *
Import('rtconfig')

cwd     = GetCurrentDir()
src	= []
path = [cwd]

src += ['max31865.c']

if GetDepend('PKG_MAX31865_USING_SENSOR_V1_DEVICE'):
    src += ['sensor_maxim_max31865.c']
    
group = DefineGroup('max31865', src, depend = ['PKG_USING_MAX31865'], CPPPATH = path)

Return('group')