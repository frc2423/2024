#!/bin/bash

# Directory containing VSIX files
VSIX_DIR="./vscode-extensions"

# Loop through each VSIX file in the directory and install it
for vsix in ${VSIX_DIR}/*.vsix; do
    echo "Installing extension from $vsix..."
    code --install-extension "$vsix"
done

echo "All extensions have been installed."
