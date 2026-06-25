#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "usage: $0 <executable-path>" >&2
    exit 2
fi

exe_path="$1"
if [[ ! -e "$exe_path" ]]; then
    echo "error: executable not found: $exe_path" >&2
    exit 1
fi

required_caps="cap_net_raw,cap_net_admin,cap_sys_nice=ep"
current_caps="$(getcap "$exe_path" 2>/dev/null || true)"

if [[ "$current_caps" == *"$required_caps"* ]]; then
    echo "capabilities already set on $exe_path"
    echo "$current_caps"
    exit 0
fi

echo "applying Linux capabilities to $exe_path"
sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice+ep "$exe_path"
getcap "$exe_path"
