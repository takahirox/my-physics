#!/usr/bin/env bash
set -euo pipefail

SOURCE_URL="https://github.com/onyekpeu/IO-VNBD.git"
SOURCE_COMMIT="118939602e3422d47b8ab0807b623751c3ac135b"
SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
REPOSITORY_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$REPOSITORY_ROOT/validation/io_vnbd/acquisition.tsv"
MODE=""
SPLIT="all"
OUTPUT_DIR="$REPOSITORY_ROOT/target/io-vnbd/raw"
CACHE_DIR="${TMPDIR:-/tmp}/my-physics-io-vnbd-cache"

usage() {
  cat <<'EOF'
Usage: scripts/fetch-io-vnbd.sh OUTPUT [--split NAME]
       scripts/fetch-io-vnbd.sh MODE [OPTIONS]

Modes (exactly one is required):
  --list          List the hash-pinned run selection; no network or writes.
  --dry-run       Print the selected files and intended locations; no network or writes.
  --fetch         Clone/fetch the pinned source, download selected Git LFS objects,
                  verify SHA-256 and byte size, then copy them into OUTPUT.
  --verify-only   Verify files already present in OUTPUT; no network.

Options:
  --split NAME    all, calibration, validation, holdout, or smoke (default: all)
  --output PATH   Prepared-data directory (default: target/io-vnbd/raw)
  --cache PATH    Git/LFS cache directory (default: $TMPDIR/my-physics-io-vnbd-cache)
  -h, --help      Show this help.

Raw IO-VNBD files are intentionally not committed. See validation/io_vnbd/README.md.
EOF
}

die() {
  printf 'fetch-io-vnbd: %s\n' "$*" >&2
  exit 1
}

set_mode() {
  [ -z "$MODE" ] || die "choose exactly one mode"
  MODE="$1"
}

# Stable convenience contract used by the IO-specific correlation runner.
# A non-option first argument is an output directory and implies --fetch.
if [ "$#" -gt 0 ] && [[ "$1" != -* ]]; then
  MODE="--fetch"
  OUTPUT_DIR="$1"
  shift
fi

while [ "$#" -gt 0 ]; do
  case "$1" in
    --list|--dry-run|--fetch|--verify-only) set_mode "$1"; shift ;;
    --split) [ "$#" -ge 2 ] || die "--split requires a value"; SPLIT="$2"; shift 2 ;;
    --output) [ "$#" -ge 2 ] || die "--output requires a path"; OUTPUT_DIR="$2"; shift 2 ;;
    --cache) [ "$#" -ge 2 ] || die "--cache requires a path"; CACHE_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
done

[ -n "$MODE" ] || { usage >&2; exit 2; }
case "$SPLIT" in all|calibration|validation|holdout|smoke) ;; *) die "invalid split: $SPLIT" ;; esac
[ -f "$MANIFEST" ] || die "manifest not found: $MANIFEST"
[ -n "$OUTPUT_DIR" ] && [ "$OUTPUT_DIR" != "/" ] || die "unsafe output path"
[ -n "$CACHE_DIR" ] && [ "$CACHE_DIR" != "/" ] || die "unsafe cache path"

validate_manifest() {
  awk -F '\t' '
    /^#/ { next }
    $1 == "run_id" { header++; next }
    {
      rows++
      if (NF != 9 || $1 !~ /^[A-Za-z0-9-]+$/ || seen[$1]++ ||
          $2 !~ /^(calibration|validation|holdout|smoke)$/ ||
          $6 !~ /^[0-9]+$/ || $6 <= 0 ||
          length($7) != 64 || $7 !~ /^[0-9a-f]+$/ ||
          $8 ~ /^\// || $8 ~ /(^|\/)\.\.\// || $8 !~ /\.csv$/) {
        printf "invalid acquisition manifest row %d for run %s\n", NR, $1 > "/dev/stderr"
        failed=1
      }
    }
    END {
      if (header != 1 || rows < 1) {
        print "acquisition manifest must have one header and at least one row" > "/dev/stderr"
        failed=1
      }
      exit failed
    }
  ' "$MANIFEST" || die "acquisition manifest validation failed"
}

validate_manifest

sha256_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  elif command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  else
    die "sha256sum or shasum is required"
  fi
}

selected_rows() {
  awk -F '\t' -v wanted="$SPLIT" '
    /^#/ || $1 == "run_id" { next }
    wanted == "all" || $2 == wanted { print }
  ' "$MANIFEST"
}

verify_one() {
  local run_id="$1" expected_bytes="$2" expected_sha="$3" file="$4"
  [ -f "$file" ] || die "$run_id missing: $file"
  local actual_bytes actual_sha
  actual_bytes="$(wc -c < "$file" | tr -d '[:space:]')"
  [ "$actual_bytes" = "$expected_bytes" ] || die "$run_id byte-size mismatch: expected $expected_bytes, got $actual_bytes"
  actual_sha="$(sha256_file "$file")"
  [ "$actual_sha" = "$expected_sha" ] || die "$run_id SHA-256 mismatch: expected $expected_sha, got $actual_sha"
}

if [ "$MODE" = "--list" ]; then
  printf 'source\t%s\ncommit\t%s\n' "$SOURCE_URL" "$SOURCE_COMMIT"
  printf 'run_id\tsplit\tbytes\tsha256\tsource_path\n'
  selected_rows | awk -F '\t' 'BEGIN { OFS="\t" } { print $1, $2, $6, $7, $8 }'
  exit 0
fi

if [ "$MODE" = "--dry-run" ]; then
  printf 'Pinned source: %s @ %s\n' "$SOURCE_URL" "$SOURCE_COMMIT"
  printf 'Cache: %s\nOutput: %s\n' "$CACHE_DIR" "$OUTPUT_DIR"
  selected_rows | awk -F '\t' -v output="$OUTPUT_DIR" '{ printf "%s [%s]: %s -> %s/%s.csv\n", $1, $2, $8, output, $1 }'
  exit 0
fi

if [ "$MODE" = "--verify-only" ]; then
  count=0
  while IFS=$'\t' read -r run_id split purpose synchronization pressure bytes sha source_path scenario; do
    verify_one "$run_id" "$bytes" "$sha" "$OUTPUT_DIR/$run_id.csv"
    count=$((count + 1))
  done < <(selected_rows)
  printf 'Verified %d IO-VNBD file(s) for split %s.\n' "$count" "$SPLIT"
  exit 0
fi

command -v git >/dev/null 2>&1 || die "git is required"
git lfs version >/dev/null 2>&1 || die "git-lfs is required"
CACHE_MARKER="$CACHE_DIR/.my-physics-io-vnbd-cache-v1"
if [ -e "$CACHE_DIR" ]; then
  [ -d "$CACHE_DIR" ] || die "cache path is not a directory: $CACHE_DIR"
  if [ ! -f "$CACHE_MARKER" ]; then
    if [ -n "$(find "$CACHE_DIR" -mindepth 1 -maxdepth 1 -print -quit)" ]; then
      die "refusing unmarked non-empty cache directory: $CACHE_DIR"
    fi
    printf 'source=%s\ncommit=%s\n' "$SOURCE_URL" "$SOURCE_COMMIT" > "$CACHE_MARKER"
  fi
else
  mkdir -p "$CACHE_DIR"
  printf 'source=%s\ncommit=%s\n' "$SOURCE_URL" "$SOURCE_COMMIT" > "$CACHE_MARKER"
fi
grep -Fxq "source=$SOURCE_URL" "$CACHE_MARKER" || die "cache marker source mismatch"
grep -Fxq "commit=$SOURCE_COMMIT" "$CACHE_MARKER" || die "cache marker commit mismatch"
mkdir -p "$OUTPUT_DIR"
SOURCE_DIR="$CACHE_DIR/repository"

if [ ! -d "$SOURCE_DIR/.git" ]; then
  [ ! -e "$SOURCE_DIR" ] || die "marked cache contains a non-repository source path: $SOURCE_DIR"
  GIT_LFS_SKIP_SMUDGE=1 git clone --no-checkout "$SOURCE_URL" "$SOURCE_DIR"
else
  actual_url="$(git -C "$SOURCE_DIR" remote get-url origin)"
  [ "$actual_url" = "$SOURCE_URL" ] || die "cache origin mismatch: $actual_url"
fi

git -C "$SOURCE_DIR" fetch --depth 1 origin "$SOURCE_COMMIT"
GIT_LFS_SKIP_SMUDGE=1 git -C "$SOURCE_DIR" checkout --detach --force "$SOURCE_COMMIT"
[ "$(git -C "$SOURCE_DIR" rev-parse HEAD)" = "$SOURCE_COMMIT" ] || die "source commit verification failed"

count=0
while IFS=$'\t' read -r run_id split purpose synchronization pressure bytes sha source_path scenario; do
  destination="$OUTPUT_DIR/$run_id.csv"
  if [ -f "$destination" ]; then
    verify_one "$run_id" "$bytes" "$sha" "$destination"
    printf 'Already verified %s [%s]; skipped.\n' "$run_id" "$split"
    count=$((count + 1))
    continue
  fi
  pointer="$(git -C "$SOURCE_DIR" show "$SOURCE_COMMIT:$source_path")"
  printf '%s\n' "$pointer" | grep -Fq "oid sha256:$sha" || die "$run_id LFS pointer OID differs from manifest"
  printf '%s\n' "$pointer" | grep -Fq "size $bytes" || die "$run_id LFS pointer size differs from manifest"

  git -C "$SOURCE_DIR" lfs pull --include="$source_path" --exclude="" origin "$SOURCE_COMMIT"
  source_file="$SOURCE_DIR/$source_path"
  verify_one "$run_id" "$bytes" "$sha" "$source_file"
  prepared_tmp="$(mktemp "$OUTPUT_DIR/.${run_id}.XXXXXX")"
  cp "$source_file" "$prepared_tmp"
  verify_one "$run_id" "$bytes" "$sha" "$prepared_tmp"
  chmod 0444 "$prepared_tmp"
  mv "$prepared_tmp" "$destination"
  verify_one "$run_id" "$bytes" "$sha" "$destination"
  printf 'Prepared %s [%s].\n' "$run_id" "$split"
  count=$((count + 1))
done < <(selected_rows)

printf 'Prepared and verified %d IO-VNBD file(s) in %s.\n' "$count" "$OUTPUT_DIR"
