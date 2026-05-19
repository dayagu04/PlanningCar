"""Build the C++ extension nav_core_cpp using g++ (no cmake required).

The project lives under a path with CJK characters, which MSYS2's linker
cannot handle reliably. We therefore stage sources/output in an ASCII temp
directory, compile there, then copy the .pyd back into src/.
"""

import os
import shutil
import subprocess
import sys
import sysconfig
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CPP_DIR = os.path.join(ROOT, "cpp")
INC_DIR = os.path.join(CPP_DIR, "include")
SRC_DIR = os.path.join(CPP_DIR, "src")
OUT_DIR = os.path.join(ROOT, "src")

EXT_SUFFIX = sysconfig.get_config_var("EXT_SUFFIX") or ".pyd"
EXT_NAME = "nav_core_cpp" + EXT_SUFFIX
TARGET = os.path.join(OUT_DIR, EXT_NAME)

SOURCES = ["nav_core.cpp", "nav_planner.cpp", "bindings.cpp"]


def find_compiler():
    for cand in ("g++", "c++"):
        path = shutil.which(cand)
        if path:
            return path
    if os.path.exists(r"C:\msys64\ucrt64\bin\g++.exe"):
        return r"C:\msys64\ucrt64\bin\g++.exe"
    raise RuntimeError("No C++ compiler found (need g++).")


def python_paths():
    base = sys.base_prefix
    include = sysconfig.get_path("include")
    if not include or not os.path.exists(include):
        include = os.path.join(base, "Include")
    libs_dir = os.path.join(base, "libs")
    if not os.path.isdir(libs_dir):
        libs_dir = sysconfig.get_config_var("LIBDIR") or ""
    version_tag = f"python{sys.version_info.major}{sys.version_info.minor}"
    return include, libs_dir, version_tag


def pybind11_include():
    import pybind11
    return pybind11.get_include()


def stage_sources(workdir: str):
    """Copy headers and sources into ASCII-safe workdir."""
    work_inc = os.path.join(workdir, "include")
    work_src = os.path.join(workdir, "src")
    os.makedirs(work_inc, exist_ok=True)
    os.makedirs(work_src, exist_ok=True)
    for fname in os.listdir(INC_DIR):
        shutil.copy2(os.path.join(INC_DIR, fname), work_inc)
    for fname in SOURCES:
        shutil.copy2(os.path.join(SRC_DIR, fname), work_src)
    return work_inc, work_src


def main():
    cxx = find_compiler()
    py_inc, py_libs, py_ver = python_paths()
    pb_inc = pybind11_include()

    # ASCII-safe staging dir under TEMP
    workdir = tempfile.mkdtemp(prefix="navcore_build_")
    out_pyd = os.path.join(workdir, EXT_NAME)
    work_inc, work_src = stage_sources(workdir)

    print(f"[build_cpp] compiler  : {cxx}")
    print(f"[build_cpp] workdir   : {workdir}")
    print(f"[build_cpp] python    : inc={py_inc}  libs={py_libs} ({py_ver})")
    print(f"[build_cpp] pybind11  : {pb_inc}")
    print(f"[build_cpp] target    : {TARGET}")

    cmd = [
        cxx,
        "-O3", "-Wall", "-Wextra", "-std=c++17",
        "-shared", "-static-libgcc", "-static-libstdc++",
        f"-I{work_inc}",
        f"-I{py_inc}",
        f"-I{pb_inc}",
        *[os.path.join(work_src, s) for s in SOURCES],
        f"-L{py_libs}",
        f"-l{py_ver}",
        "-o", out_pyd,
    ]
    print("[build_cpp] running compiler ...")
    res = subprocess.run(cmd, cwd=workdir)
    if res.returncode != 0:
        sys.exit(res.returncode)
    if not os.path.exists(out_pyd):
        print(f"[build_cpp] FAIL: output not produced: {out_pyd}")
        sys.exit(1)

    os.makedirs(OUT_DIR, exist_ok=True)
    shutil.copy2(out_pyd, TARGET)
    shutil.rmtree(workdir, ignore_errors=True)
    print(f"[build_cpp] OK -> {TARGET}")


if __name__ == "__main__":
    main()

