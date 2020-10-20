from building import *
Import('rtconfig')

cwd     = GetCurrentDir()
src	= Glob('*.c')
path = [cwd]

group = DefineGroup('max31865', src, depend = ['PKG_USING_MAX31865'], CPPPATH = path)

Return('group')