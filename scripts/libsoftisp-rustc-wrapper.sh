#!/data/data/com.termux/files/usr/bin/bash
# rustc wrapper used only by build-apk.sh.
#
# cargo-apk2 compiles the softisp cdylib itself and packages the artifact it
# finds under target/. Patching a copy in cam-rust/lib would never affect that
# artifact, so retune the cdylib immediately after rustc links it. Build
# scripts and dependents are left untouched.
set -euo pipefail

rustc="$1"
shift
args=("$@")

"$rustc" "${args[@]}"

crate_name=""
output=""
out_dir=""
is_cdylib=0
for ((i = 0; i < ${#args[@]}; i++)); do
    arg="${args[$i]}"
    case "$arg" in
        --crate-name)
            if ((i + 1 < ${#args[@]})); then
                crate_name="${args[$((i + 1))]}"
            fi
            ;;
        --crate-name=*)
            crate_name="${arg#--crate-name=}"
            ;;
        -o)
            if ((i + 1 < ${#args[@]})); then
                output="${args[$((i + 1))]}"
            fi
            ;;
        --out-dir)
            if ((i + 1 < ${#args[@]})); then
                out_dir="${args[$((i + 1))]}"
            fi
            ;;
        --out-dir=*)
            out_dir="${arg#--out-dir=}"
            ;;
        --crate-type)
            if ((i + 1 < ${#args[@]})) && [[ "${args[$((i + 1))]}" == *cdylib* ]]; then
                is_cdylib=1
            fi
            ;;
        --crate-type=*)
            if [[ "${arg#--crate-type=}" == *cdylib* ]]; then
                is_cdylib=1
            fi
            ;;
    esac
done

if [ -z "$output" ] && [ "$is_cdylib" -eq 1 ] && [ -n "$out_dir" ] && [ -n "$crate_name" ]; then
    output="$out_dir/lib${crate_name}.so"
fi

targets=()
if [ -n "$output" ] && [[ "$output" == *.so ]]; then
    targets+=("$output")
fi
if [ "$is_cdylib" -eq 1 ] && [ -n "$out_dir" ] && [ -n "$crate_name" ]; then
    dep_target="$out_dir/lib${crate_name}.so"
    final_target="$(dirname "$out_dir")/lib${crate_name}.so"
    targets+=("$dep_target" "$final_target")
fi

if [ -n "${SOFTISP_WRAPPER_LOG:-}" ]; then
    printf '%s|%s|cdylib=%s\n' "$crate_name" "$output" "$is_cdylib" >> "$SOFTISP_WRAPPER_LOG"
fi

if [ "$crate_name" != "softisp" ]; then
    exit 0
fi

if ! command -v patchelf >/dev/null 2>&1; then
    echo "  WARN: patchelf not found; $output keeps libc++_shared.so NEEDED" >&2
    exit 0
fi

patched=0
for target in "${targets[@]}"; do
    [ -f "$target" ] || continue
    if readelf -d "$target" 2>/dev/null | grep -q '\[libc++_shared\.so\]'; then
        patchelf --replace-needed libc++_shared.so libsoftisp_cpp_shared.so "$target"
        echo "  patchelf: $target libc++_shared.so -> libsoftisp_cpp_shared.so"
        patched=1
    fi
done

if [ "$patched" -eq 0 ] && [ ${#targets[@]} -gt 0 ]; then
    : # nothing to patch yet; target may not exist in this invocation
fi
