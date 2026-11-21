# Native runtime

Use Linux, Python 3.12 and a supported NVIDIA RTX GPU. These are the revisions
used for the recorded trials; installing the Python task alone does not install
the Isaac Sim runtime. Use a dedicated environment and select an available GPU.

| Component | Recorded version or revision |
| --- | --- |
| Isaac Sim | 6.1.0.0 |
| Isaac Lab-Arena | c7edf3a9d5d8276565208f72422f8f8a0806d6cf (release/0.3.1) |
| IsaacLab submodule | 91a0906f93802eb859192dcb0bca879f891e92d6 |
| yam-isaac-lab-arena | bd62771e60320433893bd1d2df0fc79b5ddcd6d1 |
| inspect-robots / inspect-robots-yam | 1.0.0 / 0.38.0 |
| yam-arena | 0.1.0 |
| dreamlabs-client | 1.0.14 |
| inspect-robots-dream | b5e936a201cec66c54c297e522c920d98445e339 (0.1.0) |

One way to create the runtime in a fresh working directory with `uv`:

```sh
export UV_LINK_MODE=copy CMAKE_POLICY_VERSION_MINIMUM=3.5
runtime_root="$PWD/.runtime"
git clone https://github.com/kairosiann/yam-isaac-lab-arena.git "$runtime_root/yam-arena"
git -C "$runtime_root/yam-arena" checkout bd62771e60320433893bd1d2df0fc79b5ddcd6d1
git init "$runtime_root/IsaacLab-Arena"
git -C "$runtime_root/IsaacLab-Arena" remote add origin https://github.com/isaac-sim/IsaacLab-Arena.git
git -C "$runtime_root/IsaacLab-Arena" fetch --depth 1 origin c7edf3a9d5d8276565208f72422f8f8a0806d6cf
git -C "$runtime_root/IsaacLab-Arena" checkout --detach FETCH_HEAD
git -c url.https://github.com/.insteadOf=git@github.com: -C "$runtime_root/IsaacLab-Arena" submodule update --init --recursive --depth 1
(cd "$runtime_root/IsaacLab-Arena" && uv sync --frozen --inexact)
git clone https://github.com/kairosiann/inspect-robots-dream.git "$runtime_root/inspect-robots-dream"
git -C "$runtime_root/inspect-robots-dream" checkout b5e936a201cec66c54c297e522c920d98445e339
uv pip install --python "$runtime_root/IsaacLab-Arena/.venv/bin/python" \
  --override "$runtime_root/yam-arena/overrides.txt" \
  -e "$runtime_root/yam-arena" -e "$runtime_root/inspect-robots-dream" \
  'inspect-robots==1.0.0' 'inspect-robots-yam==0.38.0' 'dreamlabs-client==1.0.14'
source "$runtime_root/IsaacLab-Arena/.venv/bin/activate"
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
```

Then install this task from its own folder: `uv pip install --python "$VIRTUAL_ENV/bin/python" -e '.[test]'`.
Use `'.[dream,test]'` for the model runner. Point `CUDA_VISIBLE_DEVICES` at a GPU
allocated to this experiment. Runtime downloads/caches are not task assets and
must not be added to a task PR. The renderer needs an operational NVIDIA driver;
the offline grader tests do not establish native runtime availability.

For Dream, put `DREAM_API_KEY` in a private dotenv file (file mode 600, parent
directory mode 700). Supply that file with `--credential-file`; the runner reads
it only in its own process. No credential belongs in the task card or evidence.
