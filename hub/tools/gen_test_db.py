"""Write a compile_commands.json for the host test programs.

    python hub/tools/gen_test_db.py

The suites are built by build_*_test.bat with MSVC and appear in no CMake
project, so an editor calls every alias in shared.hxx and shared.hxx unknown. A
.clangd cannot fix it: firmware/.clangd sets `CompilationDatabase: build`, so
clangd INFERS the command from app/main.cxx, cross-compiled for arm-none-eabi,
and flags cannot cancel a wrong target. Regenerate after adding a suite or an
include path to a .bat.
"""
import io
import json
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))

# (test directory, flags) - mirroring the .bat files in each.
TARGETS = [
    (os.path.join('firmware', 'tests'),
     ['-std=c++20', '-DBIBO_FAKE_HAL', '-D_CRT_SECURE_NO_WARNINGS',
      # MSVC's STL static_asserts Clang 19+; clangd here is 18, so every host
      # test failed on <cstddef> with STL1000. Editor-only - the .bat uses cl.
      '-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
      '-I' + os.path.join(ROOT, 'firmware', 'lib'),
      '-I' + os.path.join(ROOT, 'firmware')]),
    (os.path.join('hub', 'tests'),
     ['-std=c++20', '-D_CRT_SECURE_NO_WARNINGS',
      '-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
      '-I' + os.path.join(ROOT, 'shared'),
      '-I' + os.path.join(ROOT, 'hub', 'src'),
      '-I' + os.path.join(ROOT, 'hub', 'third_party', 'imgui')]),

    # hub/src itself. build/cmake/compile_commands.json does NOT describe it (47
    # entries, none in hub/src) and clangd searches only the file's own directory
    # and its ANCESTORS anyway. Flags are hub/build.bat's own CFLAGS and INC.
    (os.path.join('hub', 'src'),
     ['-std=c++20', '-D_CRT_SECURE_NO_WARNINGS',
      '-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
      '-I' + os.path.join(ROOT, 'hub', 'third_party', 'imgui'),
      '-I' + os.path.join(ROOT, 'hub', 'third_party', 'imgui', 'backends'),
      '-I' + os.path.join(ROOT, 'hub', 'src'),
      '-I' + os.path.join(ROOT, 'shared'),
      '-I' + os.path.join(ROOT, 'vendor', 'rplidar_sdk', 'sdk', 'include'),
      '-I' + os.path.join(ROOT, 'vendor', 'rplidar_sdk', 'sdk', 'src')]),
]

for rel, flags in TARGETS:
    d = os.path.join(ROOT, rel)
    if not os.path.isdir(d):
        continue

    entries = []
    for name in sorted(os.listdir(d)):
        if not name.endswith('.cxx'):
            continue
        entries.append({
            'directory': d.replace('\\', '/'),
            'file': os.path.join(d, name).replace('\\', '/'),
            'arguments': ['clang++'] + flags + ['-c', name],
        })

    out = os.path.join(d, 'compile_commands.json')
    io.open(out, 'w', encoding='utf-8').write(json.dumps(entries, indent=2))
    print('  %-24s %2d entr%s' % (rel, len(entries),
                                  'y' if len(entries) == 1 else 'ies'))
