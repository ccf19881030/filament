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

#include "AtlasGenerator.h"

#include <xatlas.h>

#include <utils/Log.h>

#include <math/mat4.h>
#include <math/vec3.h>

#include <vector>
#include <limits>

using filament::math::mat4f;
using filament::math::float3;
using filament::math::float4;

static const char* GENERATOR_STRING = "atlasgen";

struct AtlasGeneratorImpl {
    const cgltf_data* source;
    cgltf_data output;
    bool discard = false;
    xatlas::Atlas *atlas;
    std::vector<float3> verts;
    void execute();
    void addToAtlas(const cgltf_primitive& prim, const mat4f& transform);
};

AtlasGenerator::AtlasGenerator(const cgltf_data* source) {
    mImpl = new AtlasGeneratorImpl();
    mImpl->source = source;
}

AtlasGenerator::~AtlasGenerator() {
    // TODO: free all the data that was allocated in this file

    // NOTE: we do not call cgltf_free on the output because the structure contains a mix of data
    // owned by the source asset and data that was allocated in this file.

    delete mImpl;
}

void AtlasGenerator::setDiscardTextures(bool b) {
    mImpl->discard = b;
}

void AtlasGenerator::execute() {
    mImpl->execute();
}

const cgltf_data* AtlasGenerator::getResult() const {
    return &mImpl->output;
}

const cgltf_attribute* findPositionAttribute(const cgltf_primitive& prim) {
    for (cgltf_size i = 0; i < prim.attributes_count; ++i) {
        if (prim.attributes[i].type == cgltf_attribute_type_position) {
            return &prim.attributes[i];
        }
    }
    return 0;
}

void AtlasGeneratorImpl::addToAtlas(const cgltf_primitive& prim, const mat4f& transform) {
    const cgltf_attribute* pos = findPositionAttribute(prim);
    xatlas::MeshDecl mesh;

    if (!pos || !pos->data || !pos->data->count) {
        utils::slog.e << "Position data not found." << utils::io::endl;
        return;
    }

    if (pos->data->is_sparse) {
        utils::slog.e << "Sparse accessors are not supported." << utils::io::endl;
        return;
    }

    if (prim.indices->component_type == cgltf_component_type_r_16u) {
        mesh.indexFormat = xatlas::IndexFormat::UInt16;
    } else if (prim.indices->component_type == cgltf_component_type_r_32u) {
        mesh.indexFormat = xatlas::IndexFormat::UInt32;
    } else {
        utils::slog.e << "Indices must be uint16 or uint32." << utils::io::endl;
        return;
    }

    verts.resize(pos->data->count);
    cgltf_float* ptr = &verts.data()->x;
    for (cgltf_size index = 0; index < verts.size(); ++index) {
        cgltf_accessor_read_float(pos->data, index, ptr, 3);
        ptr += 3;
    }

    filament::math::float3 minpt = std::numeric_limits<float>::max();
    filament::math::float3 maxpt = std::numeric_limits<float>::lowest();

    for (cgltf_size index = 0; index < verts.size(); ++index) {
        float3 v = (transform * float4(verts[index], 1.0f)).xyz;
        minpt = min(minpt, v);
        maxpt = max(maxpt, v);
        verts[index] = v;
    }

    mesh.vertexCount = verts.size();
    mesh.vertexPositionData = verts.data();
    mesh.vertexPositionStride = sizeof(verts[0]);
    mesh.indexCount = prim.indices->count;

    const uint8_t* indexData = (const uint8_t*) prim.indices->buffer_view->buffer->data;
    indexData += prim.indices->buffer_view->offset;
    indexData += prim.indices->offset;
    mesh.indexData = indexData;

    xatlas::AddMesh(atlas, mesh);

    // TODO: add it into the destination cgltf, update the min / max
}

void AtlasGeneratorImpl::execute() {

    // Perform a shallow copy, we'll swap out the data later.
    output = *source;
    output.asset.generator = (char*) GENERATOR_STRING;

    atlas = xatlas::Create();
    int nontriangles = 0;

    utils::slog.i << "Transforming verts..." << utils::io::endl;
    for (cgltf_size i = 0; i < source->nodes_count; ++i) {
        const auto& node = source->nodes[i];
        if (node.mesh) {
            mat4f matrix;
            cgltf_node_transform_world(&node, &matrix[0][0]);
            for (cgltf_size j = 0; j < node.mesh->primitives_count; ++j) {
                const auto& prim = node.mesh->primitives[j];
                if (prim.type == cgltf_primitive_type_triangles) {
                    addToAtlas(prim, matrix);
                } else {
                    ++nontriangles;
                }
            }
        }
    }

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

    xatlas::Destroy(atlas);
    if (nontriangles > 0) {
        utils::slog.w << nontriangles << " non-triangle primitives removed." << utils::io::endl;
    }
}
