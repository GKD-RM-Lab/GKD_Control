FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai
SHELL ["/bin/bash", "-c"]

RUN apt-get clean && apt-get update && \
    apt-get install -y git curl wget make cmake sudo build-essential \
                       ca-certificates lsb-release software-properties-common gnupg zsh && \
    rm -rf /var/lib/apt/lists/*

# 安装 LLVM 21
RUN curl -fsSL  -fsSL https://mirrors.tuna.tsinghua.edu.cn/llvm-apt/llvm.sh | bash -s -- 21 -m https://mirrors.tuna.tsinghua.edu.cn/llvm-apt && \
    apt-get install -y llvm-21 clang-21 clangd-21 clang-tidy-21 clang-format-21 lld-21 lldb-21 libc++-21-dev libc++abi-21-dev && \
    apt-get clean && rm -rf /var/lib/apt/lists/*
    
RUN update-alternatives --install /usr/bin/clang clang /usr/lib/llvm-21/bin/clang 210
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/lib/llvm-21/bin/clang++ 210
RUN update-alternatives --install /usr/bin/clangd clangd /usr/lib/llvm-21/bin/clangd 210
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/lib/llvm-21/bin/clang-format 210
RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/lib/llvm-21/bin/clang-tidy 210

RUN update-alternatives --install /usr/bin/lld lld /usr/lib/llvm-21/bin/lld 210
RUN update-alternatives --install /usr/bin/ld.lld ld.lld /usr/lib/llvm-21/bin/ld.lld 210
RUN update-alternatives --install /usr/bin/opt opt /usr/lib/llvm-21/bin/opt 210
RUN update-alternatives --install /usr/bin/llc llc /usr/lib/llvm-21/bin/llc 210

RUN update-alternatives --install /usr/bin/llvm-ar llvm-ar /usr/lib/llvm-21/bin/llvm-ar 210
RUN update-alternatives --install /usr/bin/llvm-ranlib llvm-ranlib /usr/lib/llvm-21/bin/llvm-ranlib 210
RUN update-alternatives --install /usr/bin/llvm-nm llvm-nm /usr/lib/llvm-21/bin/llvm-nm 210
RUN update-alternatives --install /usr/bin/llvm-objdump llvm-objdump /usr/lib/llvm-21/bin/llvm-objdump 210
RUN update-alternatives --install /usr/bin/llvm-strip llvm-strip /usr/lib/llvm-21/bin/llvm-strip 210
RUN update-alternatives --install /usr/bin/llvm-link llvm-link /usr/lib/llvm-21/bin/llvm-link 210
RUN update-alternatives --install /usr/bin/llvm-config llvm-config /usr/lib/llvm-21/bin/llvm-config 210

RUN mkdir -p /etc/clang && \
    echo "--stdlib=libc++"                          >  /etc/clang/default.cfg && \
    echo "-nostdinc++"                              >> /etc/clang/default.cfg && \
    echo "-nostdlib++"                              >> /etc/clang/default.cfg && \
    echo "-I/usr/lib/llvm-21/include/c++/v1"        >> /etc/clang/default.cfg && \
    echo "-L/usr/lib/llvm-21/lib"                   >> /etc/clang/default.cfg && \
    echo "-lc++"                                     >> /etc/clang/default.cfg && \
    echo "-lc++abi"                                  >> /etc/clang/default.cfg


# 创建用户
ARG USERNAME=gkd
RUN adduser --disabled-password --gecos "" $USERNAME && \
    usermod -aG sudo $USERNAME

USER $USERNAME
WORKDIR /home/$USERNAME
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

RUN bash <(curl -fsSL https://xmake.io/shget.text)

# 安装 oh-my-zsh
RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended && \
    sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/' /home/${USERNAME}/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git sudo docker)/' /home/${USERNAME}/.zshrc

ENV SHELL=/usr/bin/zsh

CMD ["zsh"]
