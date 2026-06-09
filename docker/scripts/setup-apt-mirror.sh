#!/bin/sh
# SPDX-FileCopyrightText: 2026 ArcheBase
# SPDX-License-Identifier: MulanPSL-2.0

set -eu

mirror="${1:-default}"
default_mirror="http://archive.ubuntu.com/ubuntu"
ubuntu_mirror="$default_mirror"
ubuntu_ports_mirror="http://ports.ubuntu.com/ubuntu-ports"
ros1_mirror="http://packages.ros.org/ros/ubuntu"
ros2_mirror="http://packages.ros.org/ros2/ubuntu"
llvm_mirror="https://apt.llvm.org"
strip_ros_sources=0

case "$mirror" in
    ""|default|none|off)
        ;;
    tsinghua|tuna)
        ubuntu_mirror="http://mirrors.tuna.tsinghua.edu.cn/ubuntu"
        ros1_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu"
        ros2_mirror="http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
        llvm_mirror="https://mirrors.tuna.tsinghua.edu.cn/llvm-apt"
        strip_ros_sources=1
        ;;
    http://*|https://*)
        ubuntu_mirror="${mirror%/}"
        ;;
    *)
        echo "Unsupported AXON_APT_MIRROR value: ${mirror}" >&2
        exit 2
        ;;
esac

if [ "$ubuntu_mirror" = "$default_mirror" ]; then
    ubuntu_ports_mirror="http://ports.ubuntu.com/ubuntu-ports"
else
    case "$ubuntu_mirror" in
        *-ports|*/ubuntu-ports) ubuntu_ports_mirror="$ubuntu_mirror" ;;
        *) ubuntu_ports_mirror="${ubuntu_mirror}-ports" ;;
    esac
fi

mkdir -p /etc/apt
printf "%s\n" "$ubuntu_mirror" > /etc/apt/axon-ubuntu-mirror-url
printf "%s\n" "$llvm_mirror" > /etc/apt/axon-llvm-mirror-url
mkdir -p /etc/apt/apt.conf.d
cat > /etc/apt/apt.conf.d/80axon-retries <<EOF
Acquire::Retries "5";
Acquire::http::Timeout "60";
Acquire::https::Timeout "60";
EOF

if [ "$ubuntu_mirror" = "$default_mirror" ]; then
    echo "Using default Ubuntu apt mirror."
    exit 0
fi

echo "Using Ubuntu apt mirror: ${ubuntu_mirror}"
echo "Using LLVM apt mirror: ${llvm_mirror}"

rewrite_file() {
    file="$1"
    [ -f "$file" ] || return 0

    sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|https://archive.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|http://security.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|https://security.ubuntu.com/ubuntu/|${ubuntu_mirror}/|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports/|${ubuntu_ports_mirror}/|g" \
        -e "s|https://ports.ubuntu.com/ubuntu-ports/|${ubuntu_ports_mirror}/|g" \
        -e "s|http://packages.ros.org/ros/ubuntu/|${ros1_mirror}/|g" \
        -e "s|https://packages.ros.org/ros/ubuntu/|${ros1_mirror}/|g" \
        -e "s|http://packages.ros.org/ros2/ubuntu/|${ros2_mirror}/|g" \
        -e "s|https://packages.ros.org/ros2/ubuntu/|${ros2_mirror}/|g" \
        -e "s|http://archive.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|https://archive.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|http://security.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|https://security.ubuntu.com/ubuntu|${ubuntu_mirror}|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports|${ubuntu_ports_mirror}|g" \
        -e "s|https://ports.ubuntu.com/ubuntu-ports|${ubuntu_ports_mirror}|g" \
        -e "s|http://packages.ros.org/ros/ubuntu|${ros1_mirror}|g" \
        -e "s|https://packages.ros.org/ros/ubuntu|${ros1_mirror}|g" \
        -e "s|http://packages.ros.org/ros2/ubuntu|${ros2_mirror}|g" \
        -e "s|https://packages.ros.org/ros2/ubuntu|${ros2_mirror}|g" \
        -e "s|https://apt.llvm.org|${llvm_mirror}|g" \
        "$file"

    if [ "$strip_ros_sources" -eq 1 ] && { grep -Fq "$ros1_mirror" "$file" || grep -Fq "$ros2_mirror" "$file"; }; then
        sed -i \
            -e '/^Types:/s/[[:space:]]deb-src//g' \
            -e '/^Types:/s/deb-src[[:space:]]//g' \
            -e '/^Types:[[:space:]]*$/s/.*/Types: deb/' \
            -e "\|^[[:space:]]*deb-src[[:space:]].*${ros1_mirror}|d" \
            -e "\|^[[:space:]]*deb-src[[:space:]].*${ros2_mirror}|d" \
            "$file"
    fi
}

rewrite_file /etc/apt/sources.list

for file in /etc/apt/sources.list.d/*.list /etc/apt/sources.list.d/*.sources; do
    rewrite_file "$file"
done
