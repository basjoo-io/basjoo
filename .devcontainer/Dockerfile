ARG BASE_IMAGE="ubuntu:latest"
FROM ${BASE_IMAGE}

ARG GCC_VERSION_MAJOR
ARG CLANG_VERSION_MAJOR
ARG CMAKE_VERSION
ARG XMAKE_VERSION
ARG ROS_DISTRO

ARG USER=devcontainer
ARG PASSWD=""
ARG UID=1000
ARG GID=1000
ARG SHELL=/bin/zsh
ARG TZ="UTC"

ARG UBUNTU_APT_REPOSITORY_URL="http://archive.ubuntu.com"
# ARG UBUNTU_APT_REPOSITORY_URL="https://mirrors.tuna.tsinghua.edu.cn"
ARG LLVM_APT_REPOSITORY_URL="https://apt.llvm.org"
# ARG LLVM_APT_REPOSITORY_URL="https://mirrors.tuna.tsinghua.edu.cn/llvm-apt"
ARG ROS_APT_REPOSITORY_URL="http://packages.ros.org/ros2/ubuntu"
# ARG ROS_APT_REPOSITORY_URL="https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu"
ARG PYPI_REPOSITORY_URL="https://pypi.org/simple"
# ARG PYPI_REPOSITORY_URL="https://pypi.tuna.tsinghua.edu.cn/simple"

RUN export DEBIAN_FRONTEND="noninteractive" \
    && sed -i "s|http://archive.ubuntu.com|${UBUNTU_APT_REPOSITORY_URL}|g" /etc/apt/sources.list.d/ubuntu.sources \
    && sed -i "/^Components:/ s/$/ restricted universe multiverse/" /etc/apt/sources.list.d/ubuntu.sources

RUN apt-get update \
    && apt-get -y upgrade \
    && apt-get -y install --no-install-recommends lsb-release software-properties-common \
    build-essential python-dev-is-python3 pinentry-curses netcat-openbsd ninja-build cppcheck gdb \
    ccache valgrind autoconf-archive rsync openssh-client gpg gpg-agent dialog locales tzdata \
    ca-certificates curl wget zip unzip tar git git-lfs sudo vim zsh libxcb-cursor0 xauth gnuplot \
    qtwayland5 libreadline-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* \
    && sed -i '/en_US.UTF-8/s/^# //g' /etc/locale.gen && sed -i '/zh_CN.UTF-8/s/^# //g' /etc/locale.gen \
    && locale-gen && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && sed -i "$ a\export TERM=\"xterm-256color\"" /etc/bash.bashrc \
    && sed -i "$ a\export TERM=\"xterm-256color\"" /etc/zsh/zshrc

RUN if [ -n "${GCC_VERSION_MAJOR}" ]; then \
    apt-get update \
    && apt-get -y install --no-install-recommends gcc-${GCC_VERSION_MAJOR} g++-${GCC_VERSION_MAJOR} \
    gfortran-${GCC_VERSION_MAJOR} libstdc++-${GCC_VERSION_MAJOR}-dev libgfortran-${GCC_VERSION_MAJOR}-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* \
    && for tool in gcc g++ gfortran cpp gcc-ar gcc-nm gcc-ranlib gcov gcov-tool gcov-dump lto-dump; do \
    update-alternatives --remove-all ${tool} 2>/dev/null || true; \
    done \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-${GCC_VERSION_MAJOR} ${GCC_VERSION_MAJOR} \
    --slave /usr/bin/g++ g++ /usr/bin/g++-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gfortran gfortran /usr/bin/gfortran-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/cpp cpp /usr/bin/cpp-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcc-ar gcc-ar /usr/bin/gcc-ar-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcc-nm gcc-nm /usr/bin/gcc-nm-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcc-ranlib gcc-ranlib /usr/bin/gcc-ranlib-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcov gcov /usr/bin/gcov-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcov-tool gcov-tool /usr/bin/gcov-tool-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/gcov-dump gcov-dump /usr/bin/gcov-dump-${GCC_VERSION_MAJOR} \
    --slave /usr/bin/lto-dump lto-dump /usr/bin/lto-dump-${GCC_VERSION_MAJOR} \
    && update-alternatives --set gcc /usr/bin/gcc-${GCC_VERSION_MAJOR}; \
    fi

# Add LLVM-APT ppa and install LLVM compilers and libraries.
RUN if [ -n "${CLANG_VERSION_MAJOR}" ]; then \
    apt-get update \
    && bash -c "$(curl -fsSL ${LLVM_APT_REPOSITORY_URL}/llvm.sh)" "" ${CLANG_VERSION_MAJOR} all -m ${LLVM_APT_REPOSITORY_URL} \
    && apt-get -y install --no-install-recommends flang-${CLANG_VERSION_MAJOR} libflang-${CLANG_VERSION_MAJOR}-dev \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* \
    && sed -i "$ a\export PATH=\"/usr/lib/llvm-${CLANG_VERSION_MAJOR}/bin:\$\{PATH\}\"" /etc/bash.bashrc \
    && sed -i "$ a\export PATH=\"/usr/lib/llvm-${CLANG_VERSION_MAJOR}/bin:\$\{PATH\}\"" /etc/zsh/zshrc; \
    fi

# Install specific version of cmake.
RUN if [ -n "${CMAKE_VERSION}" ]; then \
    apt-get update \
    && bash -c "$(curl -fsSL https://raw.githubusercontent.com/devcontainers/templates/main/src/cpp/.devcontainer/reinstall-cmake.sh)" "" ${CMAKE_VERSION} \
    && rm -rf /var/lib/apt/lists/*; \
    fi

# Install specific version of xmake.
RUN if [ -n "${XMAKE_VERSION}" ]; then \
    env XMAKE_ROOT=y prefix=/usr/local bash -c "$(curl -fsSL https://xmake.io/shget.text)" "" v${XMAKE_VERSION}; \
    fi

# Install the ROS 2 base packages matching the underlying Ubuntu release.
RUN if [ -n "${ROS_DISTRO}" ]; then \
    apt-get update \
    && export ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')" \
    && curl -fsSL -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb && rm -f /tmp/ros2-apt-source.deb \
    && sed -i "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_REPOSITORY_URL}|g" /etc/apt/sources.list.d/ros2.sources \
    && apt-get update && apt-get -y install --no-install-recommends ros-dev-tools ros-${ROS_DISTRO}-ros-base \
    && apt-get autoremove -y && apt-get clean -y && rm -rf /var/lib/apt/lists/* \
    && sed -i "$ a\export ROS_DISTRO=\"${ROS_DISTRO}\"\nsource /opt/ros/${ROS_DISTRO}/setup.bash" /etc/bash.bashrc \
    && sed -i "$ a\export ROS_DISTRO=\"${ROS_DISTRO}\"\nsource /opt/ros/${ROS_DISTRO}/setup.zsh" /etc/zsh/zshrc; \
    fi

RUN export UV_INSTALL_DIR=/usr/local/bin UV_TOOL_BIN_DIR=/usr/local/bin UV_TOOL_DIR=/usr/local/share/uv/tools UV_NO_CACHE=1 UV_DEFAULT_INDEX=${PYPI_REPOSITORY_URL} \
    && sh -c "$(curl -fsSL https://astral.sh/uv/install.sh)" \
    && uv tool install pyright@latest \
    && uv tool install ruff@latest \
    && uv tool install ty@latest \
    && sed -i "$ a\export UV_TOOL_BIN_DIR=\"/usr/local/bin\"\nexport UV_TOOL_DIR=\"/usr/local/share/uv/tools\"\nexport UV_NO_CACHE=\"1\"\nexport UV_DEFAULT_INDEX=\"${PYPI_REPOSITORY_URL}\"" /etc/bash.bashrc \
    && sed -i "$ a\export UV_TOOL_BIN_DIR=\"/usr/local/bin\"\nexport UV_TOOL_DIR=\"/usr/local/share/uv/tools\"\nexport UV_NO_CACHE=\"1\"\nexport UV_DEFAULT_INDEX=\"${PYPI_REPOSITORY_URL}\"" /etc/zsh/zshrc

RUN { id ubuntu >/dev/null 2>&1 && userdel -f -r ubuntu || true; } \
    && { groupdel ubuntu >/dev/null 2>&1 || true; } \
    && groupadd -g ${GID} ${USER} \
    && useradd -s ${SHELL} -u ${UID} -g ${GID} -G sudo -m ${USER} \
    && if [ -n "${PASSWD}" ]; then \
    echo "root:${PASSWD}" | chpasswd; \
    echo "${USER}:${PASSWD}" | chpasswd; \
    else \
    passwd -d root; \
    passwd -d "${USER}"; \
    fi

USER ${USER}
RUN zsh -c "$(curl -fsSL https://raw.githubusercontent.com/basjoo-io/basjoo/master/.devcontainer/configure-user.zsh)"
