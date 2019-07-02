from __future__ import print_function
import sysconfig
import re
conf = sysconfig.get_config_vars()

#for v in ('CC', 'CFLAGS', 'CCSHARED', 'INCLUDEPY', 'BLDSHARED', 'BLDLIBRARY', 'LDFLAGS', 'EXT_SUFFIX', 'MULTIARCH'):
    #print("{};{}".format(v, re.sub("[\t ]+", "\\ ", conf.get(v, ''))), end=';')
    #vals[v] = re.sub("[\t ]+", "\\ ", conf.get(v, ''))


print('CFLAGS', end=';')
c_flags = []
# Grab compiler flags minus the compiler itself.
c_flags.extend(conf.get('CC', '').split()[2:])
c_flags.extend(conf.get('CFLAGS', '').split())
c_flags.extend(conf.get('CCSHARED', '').split())
c_flags.append('-I{}'.format(conf.get('INCLUDEPY', '')))
c_flags.append('-Wno-strict-prototypes')
c_flags = [x for x in c_flags if not x.startswith('-O')]
print(' '.join(c_flags), end=';')


print('LINKER', end=';')
print(conf.get('BLDSHARED', '').split()[0], end=';')

print('LDFLAGS', end=';')
print(' '.join(conf.get('BLDSHARED', '').split()[1:]) + ' ' + conf.get('BLDLIBRARY', '') + ' ' + conf.get('LDFLAGS', ''), end=';')

print('EXT_SUFFIX', end=';')
ext_suffix = '.so'
if 'EXT_SUFFIX' in conf:
    ext_suffix = conf['EXT_SUFFIX']
elif 'MULTIARCH' in conf:
    ext_suffix = '.' + conf['MULTIARCH'] + '.so'

print(ext_suffix, end=';')

# TODO: Strip out -g from 1 and 3. Strip out -O2 from 3.
