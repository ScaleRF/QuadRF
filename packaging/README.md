# Building QuadRF packages

Releases are built and published by GitHub Actions (`.github/workflows/release.yml`).
A tag matching `v*` (for example `v1.0.0`) builds the arm64 Debian bookworm
packages, signs the apt repository with the packaging GPG key, attaches `.deb`
files to the GitHub Release, writes release notes on that GitHub Release, and
deploys the signed apt tree to GitHub Pages at
<https://scalerf.github.io/QuadRF/>.

That Pages URL is the apt source. After adding it, `sudo apt install quadrf`
works like any other Debian repository. The GPG signature is created in CI
(reprepro signs `dists/bookworm/Release`); Pages only hosts the already-signed
files plus `quadrf.gpg`.

Pull requests and pushes to `main` run `.github/workflows/ci.yml`, which builds
the QuadRF packages in the same bookworm arm64 container without publishing.

## GitHub Actions

### One-time repository setup

1. Enable GitHub Pages for this repository: **Settings → Pages → Source:
   GitHub Actions**.
2. Create a dedicated packaging signing key (do not use a personal key):

   ```bash
   gpg --quick-generate-key "QuadRF Packaging <packages@scalerf.com>" default default 0
   gpg --list-secret-keys --keyid-format long
   gpg --export-secret-keys --armor <KEY_ID>
   ```

3. Store the armored secret key as the repository secret `QUADRF_GPG_PRIVATE_KEY`.
   If the key has a passphrase, also set `QUADRF_GPG_PASSPHRASE`.
4. Export the public key once and keep it; clients install
   `https://scalerf.github.io/QuadRF/quadrf.gpg`. Changing keys later requires
   users to replace that file.

### Publishing a release

```bash
git tag v1.0.0
git push origin v1.0.0
```

Or run **Actions → Release packages → Run workflow**.

The workflow:

- pulls or builds a cached `debian:bookworm` arm64 builder image
  (`packaging/Dockerfile.builder`)
- runs `make -C packaging quadrf openocd qradiolink kasmvnc repo`
- skips `quadrf-mesh` (that tree is a separate repository)
- imports `QUADRF_GPG_PRIVATE_KEY` and lets reprepro sign `Release`

## Maintainer builds

The same container path used in CI:

```bash
./packaging/build-in-container.sh quadrf
./packaging/build-in-container.sh quadrf openocd qradiolink kasmvnc
```

On an arm64 Debian bookworm machine, without Docker:

```bash
sudo apt build-dep .
make -C packaging quadrf        # packages from this tree, into packaging/out/
make -C packaging openocd qradiolink kasmvnc
```

`make -C packaging` also tries `quadrf-mesh`, which copies pre-built mesh
packages from a sibling checkout when `QUADRF_MESH_DIR` is set.

Artefacts land in `packaging/out/`. The builder image is tagged
`quadrf-builder:bookworm-<hash>` from `Dockerfile.builder` and `debian/control`.
Override the image name with `QUADRF_BUILD_IMAGE`, or set `DEB_BUILD_OPTIONS`
(default `parallel=$(nproc)`).
