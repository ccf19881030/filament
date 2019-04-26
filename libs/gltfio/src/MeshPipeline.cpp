/*
 * Copyright (C) 2019 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gltfio/MeshPipeline.h>

#include <xatlas.h>

#define CGLTF_WRITE_IMPLEMENTATION
#include "cgltf_write.h"

#include <utils/Log.h>

#include <math/mat4.h>
#include <math/vec3.h>
#include <math/vec4.h>

#include <fstream>
#include <vector>

using namespace filament::math;
using namespace utils;

using std::vector;

namespace {

using AssetHandle = gltfio::MeshPipeline::AssetHandle;

static const char* POSITION = "POSITION";
static const char* GENERATOR_ID = "gltfio";

// Bookkeeping structure for baking a single primitive + node pair.
struct BakedPrim {
    const cgltf_node* sourceNode;
    const cgltf_mesh* sourceMesh;
    const cgltf_primitive* sourcePrimitive;
    float3* bakedPositions;
    uint32_t* bakedIndices;
    float3 bakedMin;
    float3 bakedMax;
};

// Utility class for simplifying population of cgltf arrays and ensuring that the memory is freed.
template <typename T>
class ArrayHolder {
    vector<std::unique_ptr<T[]> > mItems;
public:
    T* alloc(size_t count) {
        mItems.push_back(std::make_unique<T[]>(count));
        return mItems.back().get();
    }
};

// Private implementation for MeshPipeline.
class Pipeline {
public:
    const cgltf_data* flatten(const cgltf_data* sourceAsset, uint32_t flags);
    const cgltf_data* parameterize(const cgltf_data* sourceAsset);
    void addSourceAsset(cgltf_data* asset);
    ~Pipeline();

private:
    void bakeTransform(BakedPrim* prim, const mat4f& transform);
    void populateResult(const BakedPrim* prims, size_t numPrims, size_t numVerts);
    bool filterPrim(const cgltf_primitive& prim);

    uint32_t mFlattenFlags;
    vector<cgltf_data*> mSourceAssets;

    struct {
        ArrayHolder<cgltf_data> resultAssets;
        ArrayHolder<cgltf_scene> scenes;
        ArrayHolder<cgltf_node*> nodePointers;
        ArrayHolder<cgltf_node> nodes;
        ArrayHolder<cgltf_mesh> meshes;
        ArrayHolder<cgltf_primitive> prims;
        ArrayHolder<cgltf_attribute> attributes;
        ArrayHolder<cgltf_accessor> accessors;
        ArrayHolder<cgltf_buffer_view> views;
        ArrayHolder<cgltf_buffer> buffers;
        ArrayHolder<uint8_t> bufferData;
    } mStorage;
};

// Performs in-place mutation of a cgltf primitive to ensure that the POSITION attribute is the
// first item in its list of attributes, which makes life easier for pipeline operations.
void movePositionAttribute(cgltf_primitive* prim) {
    for (cgltf_size i = 0; i < prim->attributes_count; ++i) {
        if (prim->attributes[i].type == cgltf_attribute_type_position) {
            std::swap(prim->attributes[i], prim->attributes[0]);
            return;
        }
    }
}

// Returns true if the given cgltf asset has been flattened by the mesh pipeline and is therefore
// amenable to subsequent pipeline operations like baking and exporting.
bool isFlattened(const cgltf_data* asset) {
    return asset && asset->buffers_count == 1 && asset->nodes_count == asset->meshes_count &&
            asset->asset.generator == GENERATOR_ID;
}

std::ifstream::pos_type getFileSize(const char* filename) {
    std::ifstream in(filename, std::ifstream::ate | std::ifstream::binary);
    return in.tellg();
}

// Returns true if the given primitive should be baked, false if it should culled from the result.
bool Pipeline::filterPrim(const cgltf_primitive& prim) {
    const bool filterTriangles = mFlattenFlags & gltfio::MeshPipeline::FILTER_TRIANGLES;
    if (filterTriangles && prim.type != cgltf_primitive_type_triangles) {
        return false;
    }
    const cgltf_attribute* pos = prim.attributes_count ? prim.attributes : nullptr;
    if (!pos || !pos->data || !pos->data->count) {
        return false;
    }
    if (pos->data->is_sparse) {
        return false;
    }
    if (!prim.indices || prim.indices->is_sparse) {
        // TODO: generate trivial indices
        return false;
    }
    return true;
}

const cgltf_data* Pipeline::flatten(const cgltf_data* sourceAsset, uint32_t flags) {
    mFlattenFlags = flags;

    // TODO: do not iterate through all nodes, instead traverse a specific scene hierarchy.

    // Sanitize each attribute list such that POSITIONS is always the first entry.
    // Also determine the number of primitives that will be baked.
    size_t numPrims = 0;
    for (cgltf_size i = 0; i < sourceAsset->nodes_count; ++i) {
        const cgltf_node& node = sourceAsset->nodes[i];
        if (node.mesh) {
            for (cgltf_size j = 0; j < node.mesh->primitives_count; ++j) {
                cgltf_primitive& sourcePrim = node.mesh->primitives[j];
                movePositionAttribute(&sourcePrim);
                if (filterPrim(sourcePrim)) {
                    numPrims++;
                }
            }
        }
    }
    vector<BakedPrim> bakedPrims;
    bakedPrims.reserve(numPrims);

    // Count the total number of vertices and start filling in the BakedPrim structs.
    int numVertices = 0, numIndices = 0;
    for (cgltf_size i = 0; i < sourceAsset->nodes_count; ++i) {
        const cgltf_node& node = sourceAsset->nodes[i];
        if (node.mesh) {
            for (cgltf_size j = 0; j < node.mesh->primitives_count; ++j) {
                const cgltf_primitive& sourcePrim = node.mesh->primitives[j];
                if (filterPrim(sourcePrim)) {
                    numVertices += sourcePrim.attributes[0].data->count;
                    numIndices += sourcePrim.indices->count;
                    bakedPrims.push_back({
                        .sourceNode = &node,
                        .sourceMesh = node.mesh,
                        .sourcePrimitive = &sourcePrim,
                    });
                }
            }
        }
    }

    // Allocate a buffer large enough to hold vertex positions and indices.
    const size_t vertexDataSize = sizeof(float3) * numVertices;
    const size_t indexDataSize = sizeof(uint32_t) * numIndices;
    uint8_t* bufferData = mStorage.bufferData.alloc(vertexDataSize + indexDataSize);
    float3* vertexData = (float3*) bufferData;
    uint32_t* indexData = (uint32_t*) (bufferData + vertexDataSize);

    // Next, perform the actual baking: convert all vertex positions to fp32, transform them by
    // their respective node matrix, etc.
    const cgltf_node* node = nullptr;
    mat4f matrix;
    for (size_t i = 0; i < numPrims; ++i) {
        BakedPrim& bakedPrim = bakedPrims[i];
        if (bakedPrim.sourceNode != node) {
            node = bakedPrim.sourceNode;
            cgltf_node_transform_world(node, &matrix[0][0]);
        }
        const cgltf_primitive* sourcePrim = bakedPrim.sourcePrimitive;
        bakedPrim.bakedPositions = vertexData;
        bakedPrim.bakedIndices = indexData;
        vertexData += sourcePrim->attributes[0].data->count;
        indexData += sourcePrim->indices->count;
        bakeTransform(&bakedPrim, matrix);
    }

    // Allocate memory for the various cgltf structures.
    cgltf_data* resultAsset = mStorage.resultAssets.alloc(1);
    cgltf_scene* scene = mStorage.scenes.alloc(1);
    cgltf_node** nodePointers = mStorage.nodePointers.alloc(numPrims);
    cgltf_node* nodes = mStorage.nodes.alloc(numPrims);
    cgltf_mesh* meshes = mStorage.meshes.alloc(numPrims);
    cgltf_primitive* prims = mStorage.prims.alloc(numPrims);
    cgltf_buffer_view* views = mStorage.views.alloc(2 * numPrims);
    cgltf_accessor* accessors = mStorage.accessors.alloc(2 * numPrims);
    cgltf_attribute* attributes = mStorage.attributes.alloc(numPrims);
    cgltf_buffer* buffer = mStorage.buffers.alloc(1);

    // Populate the fields of the cgltf structures.
    cgltf_size positionsOffset = 0;
    cgltf_size indicesOffset = vertexDataSize;
    for (size_t index = 0; index < numPrims; ++index) {
        BakedPrim& bakedPrim = bakedPrims[index];

        nodePointers[index] = nodes + index;

        nodes[index] = {
            .name = bakedPrim.sourceNode->name,
            .mesh = meshes + index,
        };

        meshes[index] = {
            .name = bakedPrim.sourceMesh->name,
            .primitives = prims + index,
            .primitives_count = 1,
        };

        cgltf_accessor& indicesAccessor = accessors[2 * index] = {
            .component_type = cgltf_component_type_r_32u,
            .type = cgltf_type_scalar,
            .count = bakedPrim.sourcePrimitive->indices->count,
            .buffer_view = views + index * 2,
        };

        cgltf_buffer_view& indicesBufferView = views[2 * index] = {
            .buffer = buffer,
            .offset = indicesOffset,
            .size = indicesAccessor.count * sizeof(uint32_t)
        };
        indicesOffset += indicesBufferView.size;

        cgltf_accessor& positionsAccessor = accessors[2 * index + 1] = {
            .component_type = cgltf_component_type_r_32f,
            .type = cgltf_type_vec3,
            .count = bakedPrim.sourcePrimitive->attributes[0].data->count,
            .buffer_view = views + index * 2 + 1,
            .has_min = true,
            .has_max = true,
        };

        cgltf_buffer_view& positionsBufferView = views[2 * index + 1] = {
            .buffer = buffer,
            .offset = positionsOffset,
            .size = positionsAccessor.count * sizeof(float3)
        };
        positionsOffset += positionsBufferView.size;

        *((float3*) positionsAccessor.min) = bakedPrim.bakedMin;
        *((float3*) positionsAccessor.max) = bakedPrim.bakedMax;

        cgltf_attribute& positionsAttribute = attributes[index] = {
            .name = (char*) POSITION,
            .type = cgltf_attribute_type_position,
            .data = &positionsAccessor
        };

        prims[index] = {
            .type = cgltf_primitive_type_triangles,
            .indices = &indicesAccessor,
            .attributes = &positionsAttribute,
            .attributes_count = 1,
        };
    }

    scene->name = sourceAsset->scene->name;
    scene->nodes = nodePointers;
    scene->nodes_count = numPrims;

    buffer->size = vertexDataSize + indexDataSize;
    buffer->data = bufferData;

    resultAsset->file_type = sourceAsset->file_type;
    resultAsset->file_data = sourceAsset->file_data;
    resultAsset->asset = sourceAsset->asset;
    resultAsset->asset.generator = (char*) GENERATOR_ID;
    resultAsset->meshes = meshes;
    resultAsset->meshes_count = numPrims;
    resultAsset->accessors = accessors;
    resultAsset->accessors_count = numPrims * 2;
    resultAsset->buffer_views = views;
    resultAsset->buffer_views_count = numPrims * 2;
    resultAsset->buffers = buffer;
    resultAsset->buffers_count = 1;
    resultAsset->nodes = nodes;
    resultAsset->nodes_count = numPrims;
    resultAsset->scenes = scene;
    resultAsset->scenes_count = 1;
    resultAsset->scene = scene;
    return resultAsset;
}

void Pipeline::bakeTransform(BakedPrim* prim, const mat4f& transform) {
    const cgltf_primitive* source = prim->sourcePrimitive;
    const cgltf_attribute* sourcePositions = source->attributes;
    const size_t numVerts = sourcePositions->data->count;

    // Read position data, converting to float if necessary.
    cgltf_float* writePtr = &prim->bakedPositions->x;
    for (cgltf_size index = 0; index < numVerts; ++index, writePtr += 3) {
        cgltf_accessor_read_float(sourcePositions->data, index, writePtr, 3);
    }

    // Prepare for computing the post-transformed bounding box.
    float3& minpt = prim->bakedMin = std::numeric_limits<float>::max();
    float3& maxpt = prim->bakedMax = std::numeric_limits<float>::lowest();

    // Transform the positions and compute the new bounding box.
    float3* bakedPositions = prim->bakedPositions;
    for (cgltf_size index = 0; index < numVerts; ++index) {
        float3& pt = bakedPositions[index];
        pt = (transform * float4(pt, 1.0f)).xyz;
        minpt = min(minpt, pt);
        maxpt = max(maxpt, pt);
    }

    // Read index data, converting to uint32 if necessary.
    uint32_t* bakedIndices = prim->bakedIndices;
    for (cgltf_size index = 0, len = source->indices->count; index < len; ++index) {
        bakedIndices[index] = cgltf_accessor_read_index(source->indices, index);
    }
}

const cgltf_data* Pipeline::parameterize(const cgltf_data* sourceAsset) {
    auto atlas = xatlas::Create();

    // TODO: xatlas::AddMesh for each mesh in the sourceAsset, like this:
    // Return an error for non-flat source asset.
    //
    //      xatlas::MeshDecl mesh;
    //      mesh.vertexCount = verts.size();
    //      mesh.vertexPositionData = verts.data();
    //      mesh.vertexPositionStride = sizeof(verts[0]);
    //      mesh.indexCount = prim.indices->count;
    //      xatlas::AddMesh(atlas, mesh);

    utils::slog.i << "Computing charts..." << utils::io::endl;
    xatlas::ChartOptions coptions;
    xatlas::ComputeCharts(atlas, coptions);

    utils::slog.i << "Parameterizing charts..." << utils::io::endl;
    xatlas::ParameterizeCharts(atlas);

    utils::slog.i << "Packing charts..." << utils::io::endl;
    xatlas::PackCharts(atlas);

    utils::slog.i << "Produced "
            << atlas->atlasCount << " atlases, "
            << atlas->chartCount << " charts, "
            << atlas->meshCount  << " meshes." << utils::io::endl;

    // TODO: allocate a new cgltf_data, similar to the end of flatten()

    xatlas::Destroy(atlas);
    atlas = nullptr;
    return nullptr;
}

void Pipeline::addSourceAsset(cgltf_data* asset) {
    mSourceAssets.push_back(asset);
}

Pipeline::~Pipeline() {
    for (auto asset : mSourceAssets) {
        cgltf_free(asset);
    }
}

} // anonymous namespace

namespace gltfio {

MeshPipeline::MeshPipeline() {
    mImpl = new Pipeline();
}

MeshPipeline::~MeshPipeline() {
    Pipeline* impl = (Pipeline*) mImpl;
    delete impl;
}

AssetHandle MeshPipeline::flatten(AssetHandle source, uint32_t flags) {
    Pipeline* impl = (Pipeline*) mImpl;
    return impl->flatten((const cgltf_data*) source, flags);
}

AssetHandle MeshPipeline::load(const utils::Path& fileOrDirectory) {
    utils::Path filename = fileOrDirectory;
    if (!filename.exists()) {
        utils::slog.e << "file " << filename << " not found!" << utils::io::endl;
        return nullptr;
    }
    if (filename.isDirectory()) {
        auto files = filename.listContents();
        for (auto file : files) {
            if (file.getExtension() == "gltf") {
                filename = file;
                break;
            }
        }
        if (filename.isDirectory()) {
            utils::slog.e << "no glTF file found in " << filename << utils::io::endl;
            return nullptr;
        }
    }

    // Peek at the file size to allow pre-allocation.
    long contentSize = static_cast<long>(getFileSize(filename.c_str()));
    if (contentSize <= 0) {
        utils::slog.e << "Unable to open " << filename << utils::io::endl;
        exit(1);
    }

    // Consume the glTF file.
    std::ifstream in(filename.c_str(), std::ifstream::in);
    vector<uint8_t> buffer(static_cast<unsigned long>(contentSize));
    if (!in.read((char*) buffer.data(), contentSize)) {
        utils::slog.e << "Unable to read " << filename << utils::io::endl;
        exit(1);
    }

    // Parse the glTF file.
    cgltf_options options { cgltf_file_type_gltf };
    cgltf_data* sourceAsset;
    cgltf_result result = cgltf_parse(&options, buffer.data(), contentSize, &sourceAsset);
    if (result != cgltf_result_success) {
        return nullptr;
    }
    Pipeline* impl = (Pipeline*) mImpl;
    impl->addSourceAsset(sourceAsset);

    // Load external resources.
    utils::Path resourceFolder = filename.getParent();
    cgltf_load_buffers(&options, sourceAsset, resourceFolder.c_str());

    return sourceAsset;
}

void MeshPipeline::save(AssetHandle handle, const utils::Path& jsonPath,
        const utils::Path& binPath) {
    cgltf_data* asset = (cgltf_data*) handle;

    if (isFlattened(asset)) {
        utils::slog.e << "Only flattened assets can be exported to disk." << utils::io::endl;
        return;
    }

    std::string binName = binPath.getName();
    asset->buffers[0].uri = (char*) (binName.c_str());
    cgltf_options options { cgltf_file_type_gltf };
    cgltf_write_file(&options, jsonPath.c_str(), asset);
    asset->buffers[0].uri = nullptr;

    std::ofstream binFile(binPath.c_str(), std::ios::binary);
    binFile.write((char*) asset->buffers[0].data, asset->buffers[0].size);
}

AssetHandle MeshPipeline::parameterize(AssetHandle source) {
    Pipeline* impl = (Pipeline*) mImpl;
    return impl->parameterize((const cgltf_data*) source);
}

}  // namespace gltfio
