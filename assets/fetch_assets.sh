#!/usr/bin/env bash
url="https://file.yuyuko.cc/envoy"
assets=("bun_zipper_res4.ply" "sphere.ply" "watertight.ply")
if test -z "$1"
then
  base_dir="."
else
  base_dir="$1"
fi
for asset in "${assets[@]}"; do
  echo "saving file to $base_dir/$asset"
  curl "$url/$asset" > "$base_dir/$asset"
done
