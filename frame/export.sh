#!/usr/bin/env bash
# Render every part to a print-ready STL plus a top-down PNG preview.
# Usage: ./export.sh   (run from the frame/ directory)
set -euo pipefail

# Locate the OpenSCAD CLI (PATH, /Applications, or ~/Applications)
BIN="$(command -v openscad || true)"
for cand in \
    /Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD \
    "$HOME/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD"; do
    [ -z "$BIN" ] && [ -x "$cand" ] && BIN="$cand"
done
[ -z "$BIN" ] && { echo "OpenSCAD not found. Install it, then re-run."; exit 1; }

cd "$(dirname "$0")"
mkdir -p stl png

shopt -s nullglob
fail=0
for f in parts/*.scad; do
    name="$(basename "$f" .scad)"
    printf '%-22s ' "$name"
    if "$BIN" -o "stl/$name.stl" "$f" 2>"png/$name.log"; then
        "$BIN" --imgsize=1000,800 --projection=ortho --camera=0,0,400,0,0,0 \
               --autocenter --viewall --colorscheme=Tomorrow \
               -o "png/$name.png" "$f" 2>>"png/$name.log"
        echo "OK -> stl/$name.stl"
    else
        echo "FAILED (see png/$name.log)"; fail=1
    fi
done

# Assembly preview (not exported as STL)
"$BIN" --imgsize=1200,900 --colorscheme=Tomorrow --viewall --autocenter \
       -o png/assembly.png assembly.scad 2>png/assembly.log \
    && echo "assembly preview -> png/assembly.png"

exit $fail
