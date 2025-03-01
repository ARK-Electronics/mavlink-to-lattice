#!/bin/bash

# Setup XDG default paths
DEFAULT_XDG_CONF_HOME="$HOME/.config"
DEFAULT_XDG_DATA_HOME="$HOME/.local/share"
export XDG_CONFIG_HOME="${XDG_CONFIG_HOME:-$DEFAULT_XDG_CONF_HOME}"
export XDG_DATA_HOME="${XDG_DATA_HOME:-$DEFAULT_XDG_DATA_HOME}"
THIS_DIR="$(dirname "$(realpath "$BASH_SOURCE")")"

pushd .
cd "$THIS_DIR"
make
popd

# Setup project directory
cp $THIS_DIR/build/mavlink-to-lattice ~/.local/bin
mkdir -p $XDG_DATA_HOME/polaris
cp $THIS_DIR/config.toml $XDG_DATA_HOME/polaris/

# Modify config file if ENV variables are set
CONFIG_FILE="$XDG_DATA_HOME/polaris/config.toml"

if [ -n "$POLARIS_API_KEY" ]; then
	echo "Setting polaris_api_key to: $POLARIS_API_KEY"
	sed -i "s/^polaris_api_key = \".*\"/polaris_api_key = \"$POLARIS_API_KEY\"/" "$CONFIG_FILE"
fi
