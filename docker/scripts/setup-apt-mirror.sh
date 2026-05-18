#!/bin/sh
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -eu

mirror="${1:-default}"
default_mirror="http://archive.ubuntu.com/ubuntu"
ubuntu_mirror="$default_mirror"
ros1_mirror="http://packages.ros.org/ros/ubuntu"
ros2_mirror="http://packages.ros.org/ros2/ubuntu"

case "$mirror" in
    ""|default|none|off)
        ;;
    tsinghua|tuna)
        ubuntu_mirror="http://mirrors.tuna.tsinghua.edu.cn/ubuntu"
        ros1_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu"
        ros2_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
        ;;
    http://*|https://*)
        ubuntu_mirror="${mirror%/}"
        ;;
    *)
        echo "Unsupported AXON_APT_MIRROR value: ${mirror}" >&2
        exit 2
        ;;
esac

mkdir -p /etc/apt
printf "%s\n" "$ubuntu_mirror" > /etc/apt/axon-ubuntu-mirror-url

if [ "$ubuntu_mirror" = "$default_mirror" ]; then
    echo "Using default Ubuntu apt mirror."
    exit 0
fi

echo "Using Ubuntu apt mirror: ${ubuntu_mirror}"

rewrite_file() {
    file="$1"
    [ -f "$file" ] || return 0

    sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|https://archive.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|http://security.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|https://security.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports/|${ubuntu_mirror}-ports/|g" \
        -e "s|https://ports.ubuntu.com/ubuntu-ports/|${ubuntu_mirror}-ports/|g" \
        -e "s|http://packages.ros.org/ros/ubuntu/|${ros1_mirror}/|g" \
        -e "s|https://packages.ros.org/ros/ubuntu/|${ros1_mirror}/|g" \
        -e "s|http://packages.ros.org/ros2/ubuntu/|${ros2_mirror}/|g" \
        -e "s|https://packages.ros.org/ros2/ubuntu/|${ros2_mirror}/|g" \
        -e "s|http://archive.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|https://archive.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|http://security.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|https://security.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports|${ubuntu_mirror}-ports|g" \
        -e "s|https://ports.ubuntu.com/ubuntu-ports|${ubuntu_mirror}-ports|g" \
        -e "s|http://packages.ros.org/ros/ubuntu|${ros1_mirror}|g" \
        -e "s|https://packages.ros.org/ros/ubuntu|${ros1_mirror}|g" \
        -e "s|http://packages.ros.org/ros2/ubuntu|${ros2_mirror}|g" \
        -e "s|https://packages.ros.org/ros2/ubuntu|${ros2_mirror}|g" \
        "$file"
}

rewrite_file /etc/apt/sources.list

for file in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
    rewrite_file "$file"
done
