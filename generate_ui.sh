#!/bin/bash
UI_DIR="./ui"
OUT_DIR="./scripts/ui"

while read file; do
    pyside-uic ${UI_DIR}/${file} -o ${OUT_DIR}/`basename ui_${file} .ui`.py
done < <(ls -A -1 $UI_DIR)