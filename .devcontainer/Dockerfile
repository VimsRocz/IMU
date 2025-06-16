FROM mcr.microsoft.com/vscode/devcontainers/python:3.10

# system deps for filterpy and plotting
RUN apt-get update && \
    apt-get install -y build-essential libatlas-base-dev && \
    rm -rf /var/lib/apt/lists/*

# switch back to vscode user
USER vscode
