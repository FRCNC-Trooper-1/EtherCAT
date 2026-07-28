#!/usr/bin/env bash
# extract-drive-facts.sh — Pull the EtherCAT/CiA402 facts that matter out of a
# servo drive manual PDF.
#
# Drive manuals run to hundreds of pages and are impractical to share or read in
# full. This extracts the specific objects and sections a master implementation
# depends on, in a form small enough to paste into a conversation or drop into
# docs/hardware-qualification/.
#
#   ./scripts/extract-drive-facts.sh <manual.pdf> [output.txt]
#
# Requires poppler-utils:  sudo apt install -y poppler-utils
#
# Copyright (c) 2026 Front Range CNC. BSD 3-Clause.

set -uo pipefail

PDF="${1:-}"
OUT="${2:-drive-facts.txt}"
CTX_LINES="${CTX_LINES:-24}"     # lines of context per hit
MAX_HITS="${MAX_HITS:-3}"        # occurrences to show per object

if [[ -z "$PDF" ]]; then
    echo "usage: $0 <manual.pdf> [output.txt]" >&2
    exit 1
fi

if [[ ! -r "$PDF" ]]; then
    echo "ERROR: cannot read '${PDF}'" >&2
    exit 1
fi

if ! command -v pdftotext >/dev/null 2>&1; then
    echo "ERROR: pdftotext not found." >&2
    echo "  sudo apt install -y poppler-utils" >&2
    exit 1
fi

TXT=$(mktemp /tmp/drive-manual-XXXXXX.txt)
trap 'rm -f "$TXT"' EXIT

echo "Converting ${PDF} ..." >&2
pdftotext -layout "$PDF" "$TXT" 2>/dev/null || {
    echo "ERROR: pdftotext failed" >&2
    exit 1
}
echo "  $(wc -l < "$TXT") lines extracted" >&2

# --------------------------------------------------------------- helpers ----

# Print bounded context around the first MAX_HITS matches of a pattern.
section() {
    local title="$1" pattern="$2"
    {
        echo
        echo "================================================================"
        echo "== ${title}"
        echo "================================================================"
        local hits
        hits=$(grep -n -i -E "$pattern" "$TXT" 2>/dev/null | head -"$MAX_HITS" | cut -d: -f1)
        if [[ -z "$hits" ]]; then
            echo "(no matches)"
            return
        fi
        local n
        for n in $hits; do
            local start=$(( n > 4 ? n - 4 : 1 ))
            local end=$(( n + CTX_LINES ))
            echo "--- line ${n} ---"
            sed -n "${start},${end}p" "$TXT" | sed 's/[[:space:]]*$//' | grep -v '^$'
            echo
        done
    }
}

# ---------------------------------------------------------------- report ----

{
    echo "Drive manual fact extraction"
    echo "source : $(basename "$PDF")"
    echo "size   : $(du -h "$PDF" | cut -f1)"
    echo "lines  : $(wc -l < "$TXT")"

    # The single most important question: a fixed interpolation time period
    # pins the entire machine's cycle time regardless of what the PC can do.
    section "0x60C2 Interpolation Time Period (RO or RW?)" '60C2|60 C2|Interpolation Time Period'

    section "Communication cycle times / DC Sync0 support" \
            'communications? cycle|cycle time|DC cycle|Sync0|SYNC0|synchroniz(ation|ing) cycle'

    section "0x1C12 / 0x1C13 PDO assignment" '1C12|1C13'

    section "SDO Complete Access" 'complete access'

    section "0x1C32 / 0x1C33 SM sync parameters, sync error counter" \
            '1C32|1C33|sync error counter|synchronization error'

    section "0x6060 / 0x6061 Modes of operation (CSP = 8)" \
            '6060|6061|Modes of [Oo]peration'

    section "Default PDO mappings 0x1600 / 0x1A00" '1600|1A00|1601|1A01'

    section "0x6065 following error / 0x607D software limits" \
            '6065|607D|[Ff]ollowing [Ee]rror [Ww]indow|[Ss]oftware [Pp]osition [Ll]imit'

    section "Scaling: 0x6091 gear, 0x6092 feed constant, 0x608F encoder" \
            '6091|6092|608F|[Gg]ear [Rr]atio|[Ff]eed [Cc]onstant|[Ee]ncoder [Rr]esolution'

    section "Identity: vendor ID / product code / revision" \
            'vendor ID|product code|1018|[Ii]dentity [Oo]bject'

    section "LRW / logical read-write restrictions" \
            'LRW|logical read.?write|LRD|LWR'

    section "STO / functional safety rating" \
            'safe torque off|\bSTO\b|SIL ?[23]|PL ?[de]|13849'
} > "$OUT" 2>/dev/null

SZ=$(wc -c < "$OUT")
LN=$(wc -l < "$OUT")

echo >&2
echo "Wrote ${OUT}  (${LN} lines, ${SZ} bytes)" >&2
echo >&2
echo "If that is small enough, paste the whole file." >&2
echo "Otherwise start with the highest-value section:" >&2
echo >&2
echo "  sed -n '/== 0x60C2/,/^== /p' ${OUT}" >&2
echo "  sed -n '/== Communication cycle/,/^== /p' ${OUT}" >&2
