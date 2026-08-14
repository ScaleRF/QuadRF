#!/bin/bash
# Export the packaging public key as quadrf.gpg at the repository root.
# Clients install it with the same curl | tee path used in production.
#
# Binary (dearmored) output, not --armor: apt's signed-by picks the parser by
# file extension, and a .gpg name with armored content fails with NO_PUBKEY.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
export GNUPGHOME="${GNUPGHOME:-${here}/.gnupg}"
out="${here}/quadrf.gpg"

if ! gpg --list-secret-keys --with-colons | grep -q '^sec:'; then
    echo "no signing key in ${GNUPGHOME}; run gen-key.sh first" >&2
    exit 1
fi

gpg --batch --export > "${out}.tmp"
mv "${out}.tmp" "${out}"
echo "exported ${out}"
