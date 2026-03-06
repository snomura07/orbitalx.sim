FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /usr/local/config && chmod 755 /usr/local/config

WORKDIR /work
