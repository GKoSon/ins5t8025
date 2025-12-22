# RT-Thread building script for bridge
from building import *

cwd = GetCurrentDir()

# 获取源文件
src = Glob('src/*.c')
src += Glob('util/*.c')

# 包含路径
CPPPATH = [cwd + '/inc', cwd + '/util']

# 检查是否定义了 USING_INS5T8025_DEMO 宏
# 使用 GetDepend 而不是 GetDefine
if GetDepend('USING_INS5T8025_DEMO'):
    src += Glob('example/*.c')
    # 添加示例代码的头文件路径
    CPPPATH += [cwd + '/example']

# 创建组件组
group = DefineGroup('ins5t8025', src, depend=['PKG_USING_INS5T8025'], CPPPATH=CPPPATH)

Return('group')
