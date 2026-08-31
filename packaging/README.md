# Building QuadRF packages

Releases are built and published by GitHub Actions (`.github/workflows/release.yml`).
A tag matching `v*` (for example `v1.0.1`) builds the arm64 Debian trixie
packages, signs the apt repository with the packaging GPG key, attaches `.deb`
files to the GitHub Release, writes release notes on that GitHub Release, and
deploys the signed apt tree to GitHub Pages at
<https://scalerf.github.io/QuadRF/>.

That Pages URL is the apt source. After adding it, `sudo apt install quadrf`
works like any other Debian repository. The GPG signature is created in CI
(reprepro signs `dists/trixie/Release`); Pages only hosts the already-signed
files plus `quadrf.gpg`.

Pull requests and pushes to `main` run `.github/workflows/ci.yml`, which builds
the QuadRF packages in the same trixie arm64 container without publishing.

## GitHub Actions

### Publishing a release

```bash
git tag v1.0.1
git push origin v1.0.1
```

Or run **Actions → Release packages → Run workflow**.

The workflow:

- pulls or builds a cached `debian:trixie` arm64 builder image
  (`packaging/Dockerfile.builder`)
- runs `make -C packaging quadrf openocd qradiolink kasmvnc repo`
- imports `QUADRF_GPG_PRIVATE_KEY` and lets reprepro sign `Release`

## Maintainer builds

The same container path used in CI:

```bash
./packaging/build-in-container.sh quadrf
./packaging/build-in-container.sh quadrf openocd qradiolink kasmvnc
```

On an arm64 Debian trixie machine, without Docker:

```bash
sudo apt build-dep .
make -C packaging quadrf        # packages from this tree, into packaging/out/
make -C packaging openocd qradiolink kasmvnc
```

Artefacts land in `packaging/out/`. The builder image is tagged
`quadrf-builder:trixie-<hash>` from `Dockerfile.builder` and `debian/control`.
Override the image name with `QUADRF_BUILD_IMAGE`, or set `DEB_BUILD_OPTIONS`
(default `parallel=$(nproc)`).
