# CZMQ's stable API as a static library, for RayFEM's self-contained wheels.
# CZMQ's CMake build always links its programs against libzmq, which in a
# wheel is a target of the same Meson build; compiling the library here makes
# that dependency explicit. The parent overrides 'libzmq' with its static one.
project('czmq-static', 'c', version: '4.2.1', default_options: ['default_library=static'])
cc = meson.get_compiler('c')
zmq_dep = dependency('libzmq')
platform = configuration_data()
foreach header : [['linux/wireless.h', 'HAVE_LINUX_WIRELESS_H'], ['net/if_media.h', 'HAVE_NET_IF_MEDIA_H']]
    if cc.has_header(header[0])
        platform.set(header[1], 1)
    endif
endforeach
if cc.has_header('net/if.h', prefix: '#include <sys/socket.h>')
    platform.set('HAVE_NET_IF_H', 1)
endif
foreach function : ['getifaddrs', 'freeifaddrs']
    if cc.has_function(function, prefix: '#include <ifaddrs.h>')
        platform.set('HAVE_' + function.to_upper(), 1)
    endif
endforeach
platform.set10('HAVE_DECL_AI_V4MAPPED', cc.has_header_symbol('netdb.h', 'AI_V4MAPPED'))
configure_file(output: 'platform.h', configuration: platform)
sources = []
foreach name : ['zactor', 'zarmour', 'zcert', 'zcertstore', 'zchunk', 'zclock', 'zconfig', 'zdigest', 'zdir',
                'zdir_patch', 'zfile', 'zframe', 'zhash', 'zhashx', 'ziflist', 'zlist', 'zlistx', 'zloop', 'zmsg',
                'zpoller', 'zsock', 'zstr', 'zsys', 'zuuid', 'zauth', 'zbeacon', 'zgossip', 'zmonitor', 'zproxy',
                'zrex', 'zgossip_msg']
    sources += 'src' / name + '.c'
endforeach
czmq = static_library('czmq', sources,
    include_directories: include_directories('.', 'include', 'src'),
    dependencies: zmq_dep,
    c_args: ['-DCZMQ_STATIC', '-U_FORTIFY_SOURCE', '-D_FORTIFY_SOURCE=0', '-w'],
    pic: true)
czmq_dep = declare_dependency(link_with: czmq, dependencies: zmq_dep,
    include_directories: include_directories('include'), compile_args: ['-DCZMQ_STATIC'])
