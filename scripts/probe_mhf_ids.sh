#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: scripts/probe_mhf_ids.sh [iface] [start_id] [end_id] [cmd]

Defaults:
  iface    can0
  start_id 0
  end_id   32
  cmd      9C

Environment variables:
  TIMEOUT_S       candump wait timeout in seconds, default 0.20
  TRIES           tries per id, default 2
  LISTEN_DELAY_S  delay after starting candump before cansend, default 0.02
  GAP_S           gap between tries, default 0.03
  PAYLOAD         full 8-byte payload hex, default CMD + 14 zeros

Examples:
  scripts/probe_mhf_ids.sh
  scripts/probe_mhf_ids.sh can1 1 32
  scripts/probe_mhf_ids.sh can0 0 32 90
  TIMEOUT_S=0.5 TRIES=3 scripts/probe_mhf_ids.sh can0 0 32 9C
EOF
}

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || {
        echo "missing command: $1" >&2
        exit 1
    }
}

is_uint() {
    [[ "$1" =~ ^[0-9]+$ ]]
}

iface="${1:-can0}"
start_id="${2:-0}"
end_id="${3:-32}"
cmd="${4:-9C}"

TIMEOUT_S="${TIMEOUT_S:-0.20}"
TRIES="${TRIES:-2}"
LISTEN_DELAY_S="${LISTEN_DELAY_S:-0.02}"
GAP_S="${GAP_S:-0.03}"

cmd="${cmd^^}"
payload="${PAYLOAD:-${cmd}00000000000000}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

need_cmd cansend
need_cmd candump
need_cmd timeout

is_uint "$start_id" || {
    echo "start_id must be an integer" >&2
    exit 1
}
is_uint "$end_id" || {
    echo "end_id must be an integer" >&2
    exit 1
}
is_uint "$TRIES" || {
    echo "TRIES must be an integer" >&2
    exit 1
}
[[ "$payload" =~ ^[0-9A-Fa-f]{16}$ ]] || {
    echo "payload must be exactly 16 hex chars, got: $payload" >&2
    exit 1
}

if (( start_id > end_id )); then
    echo "start_id must be <= end_id" >&2
    exit 1
fi

tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT

printf 'probe iface=%s ids=%d..%d payload=%s tries=%s timeout=%ss\n' \
    "$iface" "$start_id" "$end_id" "$payload" "$TRIES" "$TIMEOUT_S"
printf '%-4s %-7s %-7s %-6s %s\n' "id" "tx" "rx" "result" "reply"

for id in $(seq "$start_id" "$end_id"); do
    tx_id=$((0x140 + id))
    rx_id=$((0x180 + id))
    tx_frame="$(printf '%03X#%s' "$tx_id" "$payload")"
    rx_filter="$(printf '%03X:7FF' "$rx_id")"

    result="MISS"
    reply="-"

    for attempt in $(seq 1 "$TRIES"); do
        outfile="$tmpdir/id_${id}_try_${attempt}.log"

        timeout "${TIMEOUT_S}s" candump -L -n 1 "$iface,$rx_filter" >"$outfile" 2>/dev/null &
        dump_pid=$!
        sleep "$LISTEN_DELAY_S"

        if ! cansend "$iface" "$tx_frame" 2>/dev/null; then
            wait "$dump_pid" 2>/dev/null || true
            result="SENDERR"
            reply="-"
            break
        fi

        wait "$dump_pid" 2>/dev/null || true
        if [[ -s "$outfile" ]]; then
            result="OK"
            reply="$(tr -d '\n' <"$outfile")"
            break
        fi

        sleep "$GAP_S"
    done

    printf '%-4d 0x%03X  0x%03X  %-6s %s\n' "$id" "$tx_id" "$rx_id" "$result" "$reply"
done
