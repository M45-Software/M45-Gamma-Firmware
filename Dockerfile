ARG IDF_VERSION=v5.5.6
FROM espressif/idf:${IDF_VERSION}

WORKDIR /project

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV IDF_COMPONENT_MANAGER=0

CMD ["bash", "-lc", "scripts/build.sh --build"]
