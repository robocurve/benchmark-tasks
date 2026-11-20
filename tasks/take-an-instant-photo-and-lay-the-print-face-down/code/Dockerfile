# robocurve dev image: Isaac Sim 6.1.0 (NGC binary, driver-tested on a 560.35.05 consumer-GPU workstation)
# + Isaac Lab-Arena release/0.3.1 (locked venv) + kairosiann/yam-isaac-lab-arena + Inspect Robots.
# Mirrors yam-isaac-lab-arena/setup.sh. Build: docker/build.sh ; run: docker/enter.sh
FROM nvcr.io/nvidia/isaac-sim:6.1.0
USER root
ENV HOME=/root
ENV ACCEPT_EULA=Y OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y DEBIAN_FRONTEND=noninteractive \
    ARENA_DIR=/workspace/IsaacLab-Arena PATH=/root/.local/bin:$PATH
RUN apt-get update && apt-get install -y --no-install-recommends git curl ca-certificates build-essential \
    && rm -rf /var/lib/apt/lists/*
RUN curl -LsSf https://astral.sh/uv/install.sh | sh && uv --version && uv tool install cmake
WORKDIR /workspace
# Arena's submodules use SSH URLs (user "git" at github.com) -> rewrite to HTTPS.
ARG GH_SSH_USER=git
RUN git -c url."https://github.com/".insteadOf=${GH_SSH_USER}@github.com: clone --branch release/0.3.1 \
        https://github.com/isaac-sim/IsaacLab-Arena.git $ARENA_DIR \
    && git -c url."https://github.com/".insteadOf=${GH_SSH_USER}@github.com: -C $ARENA_DIR submodule update --init --recursive
# robomimic -> egl-probe builds with CMake and declares a minimum CMake 4 refuses.
RUN cd $ARENA_DIR && PATH="$(uv tool dir --bin):$PATH" CMAKE_POLICY_VERSION_MINIMUM=3.5 uv sync --frozen --inexact
# YAM embodiment + Inspect Robots policies (pinned commit bd62771e60320433893bd1d2df0fc79b5ddcd6d1).
RUN git clone https://github.com/kairosiann/yam-isaac-lab-arena.git /workspace/yam-isaac-lab-arena \
    && git -C /workspace/yam-isaac-lab-arena checkout bd62771e60320433893bd1d2df0fc79b5ddcd6d1 \
    && uv pip install --python $ARENA_DIR/.venv/bin/python --override /workspace/yam-isaac-lab-arena/overrides.txt \
         -e /workspace/yam-isaac-lab-arena \
         "inspect-robots-dream[client] @ git+https://github.com/kairosiann/inspect-robots-dream" \
         inspect-robots-agent pytest \
    && ln -sf "$($ARENA_DIR/.venv/bin/python -c 'import imageio_ffmpeg; print(imageio_ffmpeg.get_ffmpeg_exe())')" $ARENA_DIR/.venv/bin/ffmpeg \
    && $ARENA_DIR/.venv/bin/inspect-robots list embodiments | grep -E '^\s*-\s+yam_arena$'
# Arena 0.3.1 points ARENA_NUCLEUS_DIR / ISAAC_STAGING_NUCLEUS_DIR at the omniverse-content-staging bucket,
# where the 6.1 assets now 404; the same paths are served by the production bucket (checked 2026-11-03).
RUN sed -i 's/\.replace("omniverse-content-production", "omniverse-content-staging")//' \
        $ARENA_DIR/isaaclab_arena/assets/nucleus.py \
    && ! grep -q "content-staging" $ARENA_DIR/isaaclab_arena/assets/nucleus.py
# Consumer GPUs get no CUDA forward-compat: drop the injected compat libcuda so the host driver is used.
RUN for f in /etc/ld.so.conf.d/*nvcr*.conf; do mv "$f" "$f.disabled"; done; ldconfig; \
    printf 'source %s/.venv/bin/activate\nexport OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y\n' "$ARENA_DIR" > $ARENA_DIR/yam.env
WORKDIR /workspace
ENTRYPOINT ["/bin/bash", "-lc"]
CMD ["source $ARENA_DIR/yam.env && exec bash"]
