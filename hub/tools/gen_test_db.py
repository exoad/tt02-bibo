"""Write a compile_commands.json for the host test programs.

    python hub/tools/gen_test_db.py

WHY. The test suites are built by build_*_test.bat with MSVC and appear in no
CMake project, so an editor has no flags for them and reports every alias in
shared.hxx and types.hxx as an unknown type.

Adding flags to a .clangd was not enough, and the reason is worth writing down:
firmware/.clangd sets `CompilationDatabase: build`, so clangd INFERS a command
for an unknown file from the nearest entry in it - which is app/main.cxx, cross
-compiled for arm-none-eabi. The test then got `--target=arm-none-eabi` plus
`-mcpu=cortex-m33` with the host flags appended, and failed 21 ways. Extra
flags cannot cancel a wrong target; the file needs an entry of its own.

Regenerate after adding a suite or changing an include path in a .bat. The
flags here are the ones those scripts actually pass - checked, not assumed.
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
      # MSVC's STL static_asserts that Clang is 19+; clangd here is 18, so
      # every host test failed on <cstddef> with STL1000 rather than on
      # anything in the file. This is the documented escape hatch, and it
      # affects only what the EDITOR parses - the .bat builds use cl.
      '-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
      '-I' + os.path.join(ROOT, 'firmware', 'lib'),
      '-I' + os.path.join(ROOT, 'firmware')]),
    (os.path.join('hub', 'tests'),
     ['-std=c++20', '-D_CRT_SECURE_NO_WARNINGS',
      '-D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH',
      '-I' + os.path.join(ROOT, 'shared'),
      '-I' + os.path.join(ROOT, 'hub', 'src'),
      '-I' + os.path.join(ROOT, 'hub', 'third_party', 'imgui')]),

    # hub/src itself, for the same reason and it is the bigger one.
    #
    # There IS a build/cmake/compile_commands.json, and it does NOT describe
    # this directory - 47 entries, none of them hub/src, and older than the
    # sources. clangd would not have found it anyway: it searches the file's
    # own directory and its ANCESTORS, and build/cmake is neither.
    #
    # These are hub/build.bat's own CFLAGS and INC, read out of it.
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
