FROM python:3.12-slim

ARG DEBIAN_FRONTEND=noninteractive

# Install Docker CLI for container management
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    ARCH="$(uname -m)" && \
    curl -fsSL "https://download.docker.com/linux/static/stable/${ARCH}/docker-27.5.1.tgz" | \
    tar xz --strip-components=1 -C /usr/local/bin docker/docker && \
    apt-get purge -y curl && \
    apt-get autoremove -y && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

RUN python manage.py collectstatic --noinput 2>/dev/null || true

EXPOSE 8000

HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
  CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/api/health/')" || exit 1

CMD ["daphne", "-b", "0.0.0.0", "-p", "8000", "dashboard.asgi:application"]
