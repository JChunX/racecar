FROM golang:1.21.0

SHELL ["/bin/bash", "-c"]

RUN apt-get update --fix-missing && \
    apt-get install -y curl

RUN ARCH=$(uname -m) && \
    if [ "$ARCH" = "x86_64" ]; then \
      BIN_ARCH="amd64"; \
    elif [ "$ARCH" = "aarch64" ]; then \
      BIN_ARCH="arm64"; \
    else \
      echo "Unsupported architecture: $ARCH"; \
      exit 1; \
    fi && \
    curl -L https://github.com/foxglove/foxglove-cli/releases/latest/download/foxglove-linux-${BIN_ARCH} -o foxglove

RUN mv foxglove /usr/local/bin/foxglove
RUN chmod +x /usr/local/bin/foxglove
