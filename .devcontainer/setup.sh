#!/bin/bash

# Allow Docker to connect to Wayland
if [ -n "$XDG_RUNTIME_DIR" ]; then
    if [ -S "$XDG_RUNTIME_DIR/wayland-0" ]; then
        chmod 777 "$XDG_RUNTIME_DIR/wayland-0" 2>/dev/null || true
    fi
    xhost +local:docker 2>/dev/null || true
fi

echo "Wayland/X11 access configured for Docker"