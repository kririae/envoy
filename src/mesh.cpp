#include "mesh.h"

#include <filesystem>

#include "envoy.h"
#include "linalg.h"
#include "resource_manager.h"

EVY_NAMESPACE_BEGIN

// Code modified from https://github.com/vilya/miniply
TriangleMesh MakeTriangleMesh(const std::string &path, GResource &resource) {
  if (!std::filesystem::exists(path)) Error("file {} do not exists", path);
  miniply::PLYReader reader(path.c_str());

  uint32_t             face_idxs[3];
  miniply::PLYElement *face_elem =
      reader.get_element(reader.find_element(miniply::kPLYFaceElement));
  if (face_elem == nullptr) Error("face element failed to initialize");
  face_elem->convert_list_to_fixed_size(
      face_elem->find_property("vertex_indices"), 3, face_idxs);

  uint32_t indexes[3];
  bool     got_verts = false, got_faces = false;

  Info("ply mesh {} is loading", path);
  TriangleMesh mesh;
  while (reader.has_element() && (!got_verts || !got_faces)) {
    if (reader.element_is(miniply::kPLYVertexElement) &&
        reader.load_element() && reader.find_pos(indexes)) {
      mesh.num_vertices = reader.num_rows();

      // extract position
      mesh.vertex_positions = std::span{
          resource.alloc<float3[], 16>(mesh.num_vertices), mesh.num_vertices};
      reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float,
                                mesh.vertex_positions.data());

      // extract normal
      if (reader.find_normal(indexes)) {
        mesh.vertex_normals = std::span{
            resource.alloc<float3[], 16>(mesh.num_vertices), mesh.num_vertices};
        reader.extract_properties(indexes, 3, miniply::PLYPropertyType::Float,
                                  mesh.vertex_normals.data());
        Info("normal found in ply file");
      } else {
        Info("normal not found in ply file");
      }

      // extract UV
      if (reader.find_texcoord(indexes)) {
        mesh.vertex_uv = std::span{
            resource.alloc<float2[], 16>(mesh.num_vertices), mesh.num_vertices};
        // mesh->uv = std::make_unique<Vector2f[]>(mesh->nVert);
        reader.extract_properties(indexes, 2, miniply::PLYPropertyType::Float,
                                  mesh.vertex_uv.data());
      }

      got_verts = true;
    } else if (!got_faces && reader.element_is(miniply::kPLYFaceElement) &&
               reader.load_element()) {
      mesh.num_indices = reader.num_rows() * 3;
      mesh.indices = std::span{resource.alloc<uint32_t[], 16>(mesh.num_indices),
                               mesh.num_indices};
      // mesh->ind = std::make_unique<int[]>(mesh->nInd);
      reader.extract_properties(face_idxs, 3, miniply::PLYPropertyType::Int,
                                mesh.indices.data());
      got_faces = true;
    }

    if (got_verts && got_faces) break;
    reader.next_element();
  }

  if (!got_verts || !got_faces) Error("failed to create mesh");
  return mesh;
}

EVY_NAMESPACE_END