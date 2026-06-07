#!/bin/bash
# Copyright 2026 Tatsukiyano
# Ultimate Style Fixer for ROX2026

set -e

# 1. C++ Formatting (Uncrustify)
echo "[STYLE] Running Uncrustify reformat..."
ament_uncrustify --reformat lazytatzv_ws/src || true

# 2. Python Formatting (Black)
# Install black if not present
if ! command -v black &> /dev/null; then
    pip3 install black --quiet
fi
echo "[STYLE] Running Black (Python) reformat..."
black lazytatzv_ws/src --line-length 100 --quiet || true

# 3. Add Copyright Headers if missing
echo "[STYLE] Ensuring Copyright headers..."
find lazytatzv_ws/src -name "*.cpp" -o -name "*.hpp" -o -name "*.py" | while read -r file; do
    if ! grep -q "Copyright" "$file"; then
        sed -i '1i // Copyright 2026 Tatsukiyano' "$file"
    fi
done

# 4. Remove trailing whitespaces
echo "[STYLE] Cleaning trailing whitespaces..."
find lazytatzv_ws/src -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.py" -o -name "*.xml" -o -name "*.yaml" \) -exec sed -i 's/[[:space:]]*$//' {} +

echo "[STYLE] All style issues resolved!"
