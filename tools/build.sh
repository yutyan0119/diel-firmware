#!/usr/bin/env bash
set -euo pipefail

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=${DIEL_WEST_WORKSPACE:-"$root/.west-workspace"}

case "${1:-all}" in
    all) variants=(normal diagnostic power-save power-save-diagnostic) ;;
    normal|diagnostic|power-save|power-save-diagnostic) variants=("$1") ;;
    *) echo 'Usage: bash tools/build.sh [all|normal|diagnostic|power-save|power-save-diagnostic]' >&2; exit 2 ;;
esac

mkdir -p "$workspace/config" "$root/dist" "$root/build"
for test in ec_logic ec_standby; do
    cc -std=c11 -Wall -Wextra -Werror "$root/tests/${test}_test.c" -o "$root/build/${test}_test"
    "$root/build/${test}_test"
done

cd "$workspace"
cp "$root/config/west.yml" config/west.yml
[[ -d .west ]] || west init -l config
west update --fetch-opt=--filter=tree:0
west zephyr-export
for variant in "${variants[@]}"; do
    west_args=(-S studio-rpc-usb-uart)
    cmake_args=(-DCONFIG_ZMK_STUDIO=y)
    conf=()
    if [[ "$variant" == *diagnostic ]]; then
        west_args=()
        cmake_args=("-DEXTRA_DTC_OVERLAY_FILE=$root/config/diagnostic.overlay")
        conf=("$root/config/diagnostic.conf" "$root/config/diagnostic-xtal.conf")
    fi
    if [[ "$variant" == power-save* ]]; then
        conf+=("$root/config/power-save.conf")
    fi
    if (( ${#conf[@]} )); then
        conf_list=$(IFS=';'; echo "${conf[*]}")
        cmake_args+=("-DEXTRA_CONF_FILE=$conf_list")
    fi
    target="diel-$variant"
    west build -p always -s zmk/app -d "$root/build/$target" -b diel//zmk "${west_args[@]}" -- \
        "-DZMK_EXTRA_MODULES=$root" "-DZMK_CONFIG=$root/config" "${cmake_args[@]}"
    cp "$root/build/$target/zephyr/zmk.uf2" "$root/dist/$target.uf2"
done
python3 "$root/tools/verify-uf2.py"
