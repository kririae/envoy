#!/usr/bin/env bash
url="https://file.yuyuko.cc/envoy"
assets=("bun_zipper_res4.ply" "sphere.ply" "watertight.ply")
for asset in "${assets[@]}"; do
  curl "$url/$asset" > "$asset"
done
