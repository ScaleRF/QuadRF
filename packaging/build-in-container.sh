#!/bin/bash
# Build arm64 packages inside a Debian trixie container.
# Used when the host is not arm64. Requires docker with binfmt arm64.
#
# Uses a cached builder image (packaging/Dockerfile.builder) so apt and
# Build-Depends are not reinstalled under qemu on every run. The live tree
# is always mounted; only rebuild the image when the Dockerfile or
# debian/control changes (or pass --rebuild-image).
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
top="$(cd "${here}/.." && pwd)"
dockerfile="${here}/Dockerfile.builder"
platform="${QUADRF_BUILD_PLATFORM:-linux/arm64}"
base_name="${QUADRF_BUILD_IMAGE_NAME:-quadrf-builder}"

rebuild_image=0
args=()
for arg in "$@"; do
    case "${arg}" in
        --rebuild-image) rebuild_image=1 ;;
        *) args+=("${arg}") ;;
    esac
done
# "${arr[*]:-default}" is not valid bash for empty arrays; set after parse.
if [ ${#args[@]} -eq 0 ]; then
    targets="all repo"
else
    targets="${args[*]}"
fi

if ! command -v docker >/dev/null; then
    echo "docker is not installed" >&2
    exit 1
fi

docker_cmd=(docker)
if ! docker info >/dev/null 2>&1; then
    if command -v sudo >/dev/null && sudo docker info >/dev/null 2>&1; then
        docker_cmd=(sudo docker)
    else
        echo "cannot talk to the docker daemon" >&2
        exit 1
    fi
fi

# Tag tracks Dockerfile.builder + debian/control so Build-Depends changes
# force a fresh image without baking the rest of the tree into layers.
hash="$(
    cat "${dockerfile}" "${top}/debian/control" |
        sha256sum |
        awk '{print substr($1, 1, 12)}'
)"
image="${QUADRF_BUILD_IMAGE:-${base_name}:trixie-${hash}}"

need_build=0
if [ "${rebuild_image}" -eq 1 ]; then
    need_build=1
elif ! "${docker_cmd[@]}" image inspect "${image}" >/dev/null 2>&1; then
    need_build=1
fi

if [ "${need_build}" -eq 1 ]; then
    echo "building cached image ${image} (${platform})"
    ctx="$(mktemp -d)"
    trap 'rm -rf "${ctx}"' EXIT
    cp "${dockerfile}" "${ctx}/Dockerfile"
    mkdir -p "${ctx}/debian"
    cp "${top}/debian/control" "${ctx}/debian/control"
    "${docker_cmd[@]}" build \
        --platform "${platform}" \
        -t "${image}" \
        "${ctx}"
    rm -rf "${ctx}"
    trap - EXIT
else
    echo "using cached image ${image}"
fi

# Parallelism for dpkg-buildpackage / make. Override with DEB_BUILD_OPTIONS.
nproc_host="$(nproc 2>/dev/null || echo 2)"
deb_build_options="${DEB_BUILD_OPTIONS:-parallel=${nproc_host}}"

echo "building on ${platform} with ${image}: make -C packaging ${targets}"

# shellcheck disable=SC2086
"${docker_cmd[@]}" run --rm --platform "${platform}" \
    -v "${top}:/src" \
    -w /src \
    -e DEBIAN_FRONTEND=noninteractive \
    -e "DEB_BUILD_OPTIONS=${deb_build_options}" \
    -e QUADRF_GPG_PRIVATE_KEY \
    -e QUADRF_GPG_PASSPHRASE \
    -e QUADRF_KEY_NAME \
    -e QUADRF_KEY_EMAIL \
    "${image}" \
    make -C packaging ${targets}
