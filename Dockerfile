FROM archlinux

RUN pacman -Syu --needed --noconfirm base-devel git go ccache
ENV MAKEFLAGS="-j$(nproc)"
RUN echo 'MAKEFLAGS="-j$(nproc)"' >> /etc/makepkg.conf
RUN git clone https://aur.archlinux.org/yay.git
RUN sudo chmod 777 -R ./yay

RUN useradd --shell=/bin/false build && usermod -L build
RUN echo "build ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN echo "root ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
RUN mkdir /home/build
RUN chmod -R 777 /home/build && chown -R build:build /home/build

USER build
RUN cd yay && makepkg -si --noconfirm
RUN yay -Syu --mflags="-j$(nproc)" --noconfirm casadi eigen-git
RUN rm -rf /home/build/.cache/yay/*
USER root

RUN echo "$(nproc)"

COPY ./src /src
COPY ./nlp.so .

WORKDIR /src

RUN rm -rf ./build && mkdir ./build
WORKDIR /src/build

RUN cmake .. -DCMAKE_BUILD_TYPE=Release
RUN make -j$(nproc)

CMD bash -c './controller'

# Current command: sudo docker run -e "TERM=xterm-256color" -it --net=host loukordos/walking_controller