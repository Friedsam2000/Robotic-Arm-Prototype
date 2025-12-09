#!/bin/bash

# File extensions to delete
extensions=(
  "*.aux"
  "*.bbl"
  "*.lot"
  "*.lof"
  "*.lol"
  "*.loa"
  "*.loe"
  "*.toc"
  "*.blg"
  "*.pdf"
  "*.out"
  "*.log"
  "*.1"
  "*.mp"
  "*.gz"
)

# Loop through each extension and delete files
for ext in "${extensions[@]}"; do
  find . -type f -name "$ext" -exec rm -f {} +
done
