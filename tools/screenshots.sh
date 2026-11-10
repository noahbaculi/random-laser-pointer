#!/bin/sh
# Regenerates docs/screenshots from the simulator. Needs port 8080 free
set -eu
cd "$(dirname "$0")/.."

base=http://localhost:8080
if curl -s -o /dev/null "$base/status"; then
    echo "port 8080 in use, stop the running sim" >&2
    exit 1
fi

pw() {
    out=$(playwright-cli -s=rlp-screenshots "$@") || {
        printf 'playwright-cli %s failed:\n%s\n' "$1" "$out" >&2
        return 1
    }
}

tmp=$(mktemp -d)
sim_pid=
http_pid=
cleanup() {
    if [ -n "$sim_pid" ]; then kill "$sim_pid" 2>/dev/null || true; fi
    if [ -n "$http_pid" ]; then kill "$http_pid" 2>/dev/null || true; fi
    playwright-cli -s=rlp-screenshots close >/dev/null 2>&1 || true
    rm -rf "$tmp"
}
trap cleanup EXIT
trap 'exit 1' INT TERM

host=$(rustc -vV | sed -n 's/^host: //p')
cargo build -q --target "$host" --no-default-features --features sim --bin sim
SIM_SETTINGS="$tmp/settings.bin" "target/$host/debug/sim" >"$tmp/sim.log" 2>&1 &
sim_pid=$!
i=0
until curl -s -o /dev/null "$base/status"; do
    i=$((i + 1))
    if [ "$i" -ge 50 ]; then
        echo "sim did not start:" >&2
        cat "$tmp/sim.log" >&2
        exit 1
    fi
    sleep 0.2
done

# Convex so `/points` accepts it, inside travel 491..1966
curl -fsS -o /dev/null "$base/set?name=Living+room"
curl -fsS -o /dev/null "$base/points?r=0&p=700,650;1250,560;1750,800;1700,1500;1100,1750;640,1300"
curl -fsS -o /dev/null "$base/mode?m=random"

wait_for() { pw run-code "async page => await page.waitForFunction(() => $1)"; }

# Loads the page and waits for the first `/status`
load() {
    pw goto "$base/"
    wait_for "!document.body.classList.contains('offline') && document.getElementById('roomName').textContent"
}

shot() {
    wait_for "document.getElementById('toast').classList.contains('hidden')"
    pw press Escape
    pw eval "() => { document.activeElement?.blur(); window.getSelection()?.removeAllRanges(); window.scrollTo(0, 0) }"
    pw screenshot --filename="docs/screenshots/screenshot-$1.png"
}

mkdir -p docs/screenshots
pw open
pw resize 390 844

load
shot random

pw eval "() => document.querySelector('[data-m=manual]').click()"
wait_for "!document.getElementById('bManual').classList.contains('hidden')"
shot manual

curl -fsS -o /dev/null "$base/mode?m=random"
load
pw eval "() => document.getElementById('editOutline').click()"
wait_for "!document.getElementById('outline').classList.contains('hidden')"
shot outline

# Compose the three phones onto a transparent sheet
python3 -m http.server 13381 --bind 127.0.0.1 --directory . >"$tmp/http.log" 2>&1 &
http_pid=$!
i=0
until curl -s -o /dev/null http://127.0.0.1:13381/; do
    i=$((i + 1))
    if [ "$i" -ge 50 ]; then
        echo "http server did not start:" >&2
        cat "$tmp/http.log" >&2
        exit 1
    fi
    sleep 0.2
done
pw resize 1450 1000
pw goto http://127.0.0.1:13381/tools/screenshot-sheet.html
pw run-code "async page => await page.screenshot({ path: 'docs/screenshots/readme-preview.png', type: 'png', omitBackground: true })"
