# Envoy

[![linux](https://github.com/kririae/envoy/actions/workflows/linux.yml/badge.svg?branch=main)](https://github.com/kririae/envoy/actions/workflows/linux.yml)

Envoy is an experimental and personal(for now) BVH backend for renderers, 
with the following ideas inspired by [Nanite](https://advances.realtimerendering.com/s2021/Karis_Nanite_SIGGRAPH_Advances_2021_final.pdf)
and [Disney Hyperion Renderer](https://www.disneyanimation.com/publications/the-design-and-evolution-of-disneys-hyperion-renderer/),
1. Generally, a single mesh manipulated on an artist's workstation is relatively easy to process.
2. The number of these meshes can be large in industrial cases.
3. Some post-processing or simplification can be performed on these meshes to achieve real-time performance in exchange for negligible visual effects. 
4. LoD is required to achieve maximum performance. There's no need to perform that large number of triangle intersections even if BVH effectively reduces the effort. 
5. While assets need to be loaded from disk (streaming or virtual geometry), the latency of intersection can be large. Techniques to reduce or hide latency are required. We want to make this process not that intrusive.

## Usage

`xmake` is required. 
Just download this repository, install all the dependencies through `xmake`, then execute `xmake r envoy`.

## TODO

- [x] SIMD triangles intersection
- [ ] Complement simple non-continuous partition with Z-order curve
- [ ] Implement "Maximizing Parallelism in the Construction of BVHs" with mesh partition
- [ ] Implement page management system between disk and memory
- [ ] Test partition heuristics with [METIS](https://github.com/KarypisLab/METIS)
- [ ] Implement or use [CGAL](https://github.com/CGAL/cgal) to simplify mesh and build the LoD DAG presented in Nanite
- [ ] Implement the real-time LoD DAG splitting
