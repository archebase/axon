#!/bin/bash
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -e

DIST_DIR="$1"
OUTPUT_FILE="$2"

if [ -z "$DIST_DIR" ] || [ -z "$OUTPUT_FILE" ]; then
    echo "Usage: $0 <dist_dir> <output_file>"
    exit 1
fi

if [ ! -d "$DIST_DIR" ]; then
    echo "Error: Directory $DIST_DIR does not exist"
    exit 1
fi

if ! command -v xxd >/dev/null 2>&1; then
    echo "Error: xxd not found. Install it with: sudo apt-get install xxd"
    exit 1
fi

echo "Embedding assets from $DIST_DIR into $OUTPUT_FILE"

# Start generating the C++ file
cat > "$OUTPUT_FILE" << 'EOF'
/*
 * SPDX-FileCopyrightText: 2026 ArcheBase
 * SPDX-License-Identifier: MulanPSL-2.0
 *
 * Auto-generated file - DO NOT EDIT
 */

#include "embedded_assets.hpp"
#include <cstring>

EOF

# Function to get MIME type
get_mime_type() {
    local file="$1"
    case "$file" in
        *.html) echo "text/html" ;;
        *.css) echo "text/css" ;;
        *.js) echo "application/javascript" ;;
        *.json) echo "application/json" ;;
        *.png) echo "image/png" ;;
        *.jpg|*.jpeg) echo "image/jpeg" ;;
        *.svg) echo "image/svg+xml" ;;
        *.ico) echo "image/x-icon" ;;
        *.woff) echo "font/woff" ;;
        *.woff2) echo "font/woff2" ;;
        *.ttf) echo "font/ttf" ;;
        *) echo "application/octet-stream" ;;
    esac
}

# Generate embedded data for each file
file_count=0
while IFS= read -r file; do
    # Get relative path
    rel_path="${file#$DIST_DIR}"
    # Ensure path starts with /
    if [[ ! "$rel_path" =~ ^/ ]]; then
        rel_path="/$rel_path"
    fi

    # Create a valid C identifier from the path
    identifier=$(echo "$rel_path" | sed 's/[^a-zA-Z0-9]/_/g')

    # Get MIME type
    mime_type=$(get_mime_type "$file")

    # Generate byte array using xxd
    echo "// File: $rel_path" >> "$OUTPUT_FILE"
    echo "static const unsigned char asset_data_${identifier}[] = {" >> "$OUTPUT_FILE"
    xxd -i < "$file" >> "$OUTPUT_FILE"
    echo "};" >> "$OUTPUT_FILE"
    echo "static const size_t asset_size_${identifier} = sizeof(asset_data_${identifier});" >> "$OUTPUT_FILE"
    echo "" >> "$OUTPUT_FILE"

    file_count=$((file_count + 1))
done < <(find "$DIST_DIR" -type f)

# Generate the initialization function
cat >> "$OUTPUT_FILE" << 'EOF'

void EmbeddedAssets::init() {
    if (initialized_) return;

EOF

# Add each asset to the map
while IFS= read -r file; do
    rel_path="${file#$DIST_DIR}"
    if [[ ! "$rel_path" =~ ^/ ]]; then
        rel_path="/$rel_path"
    fi

    identifier=$(echo "$rel_path" | sed 's/[^a-zA-Z0-9]/_/g')
    mime_type=$(get_mime_type "$file")

    cat >> "$OUTPUT_FILE" << EOF
    assets_["$rel_path"] = Asset{
        reinterpret_cast<const char*>(asset_data_${identifier}),
        asset_size_${identifier},
        "$mime_type"
    };
EOF
done < <(find "$DIST_DIR" -type f)

cat >> "$OUTPUT_FILE" << 'EOF'

    initialized_ = true;
}
EOF

echo "Generated $OUTPUT_FILE with $file_count embedded files"
