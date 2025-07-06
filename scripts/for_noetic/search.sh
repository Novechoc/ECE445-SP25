#!/bin/bash

SEARCH_DIR="$1"
SEARCH_STRING="$2"

find "$SEARCH_DIR" -type f -name "*.py" | while read -r file; do
 if grep -q "$SEARCH_STRING" "$file"; then
	 # echo "Found in: $file"
	 grep -Hn "$SEARCH_STRING" "$file" 2>/dev/null
 fi
done
