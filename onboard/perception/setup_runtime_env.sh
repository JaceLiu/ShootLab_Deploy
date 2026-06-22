#!/usr/bin/env bash
# Shared runtime paths for the onboard perception scripts.
#
# This machine is JetPack 6 / Python 3.10.  Keep conda's lib directory first so
# packages such as OpenCV load the matching libstdc++ instead of Ubuntu's older
# system copy.

ROBOMIMIC_CONDA_ENV="${ROBOMIMIC_CONDA_ENV:-/home/unitree/miniconda3/envs/robomimic}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/share/opt/cyclonedds-0.10.5}"
export CYCLONEDDS_HOME

_runtime_ld_paths=(
    "$ROBOMIMIC_CONDA_ENV/lib"
    "$CYCLONEDDS_HOME/lib"
    "/usr/lib/aarch64-linux-gnu"
)

for _cuda_compat in /usr/local/cuda-12.6/compat /usr/local/cuda/compat /usr/local/cuda-12.1/compat; do
    [[ -d "$_cuda_compat" ]] && _runtime_ld_paths+=("$_cuda_compat")
done

_runtime_ld_path=""
for _path in "${_runtime_ld_paths[@]}"; do
    if [[ -d "$_path" ]]; then
        if [[ -n "$_runtime_ld_path" ]]; then
            _runtime_ld_path="${_runtime_ld_path}:$_path"
        else
            _runtime_ld_path="$_path"
        fi
    fi
done

if [[ -n "$_runtime_ld_path" ]]; then
    export LD_LIBRARY_PATH="${_runtime_ld_path}:${LD_LIBRARY_PATH:-}"
fi

unset _runtime_ld_paths _runtime_ld_path _cuda_compat _path
