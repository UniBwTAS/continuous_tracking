FROM andreasr30/continuous_clustering_demo:master

ENV HOME /root
ARG DEBIAN_FRONTEND=noninteractive

# copy scripts into docker image
COPY scripts/* /usr/local/bin/

# install dependencies and build workspace
WORKDIR ${HOME}/catkin_ws/src
COPY . ./continuous_tracking
RUN clone_repositories_and_install_dependencies.sh --from-source && rm -rf /var/lib/apt/lists/* && rm -rf ${HOME}/.ros
RUN catkin build