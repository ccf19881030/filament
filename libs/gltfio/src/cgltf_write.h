/**
 * cgltf_write - a single-file glTF 2.0 writer written in C99.
 *
 * Version: 1.0
 *
 * Website: https://github.com/jkuhlmann/cgltf
 *
 * Distributed under the MIT License, see notice at the end of this file.
 *
 * Building:
 * Include this file where you need the struct and function
 * declarations. Have exactly one source file where you define
 * `CGLTF_WRITE_IMPLEMENTATION` before including this file to get the
 * function definitions.
 *
 */
#ifndef CGLTF_WRITE_H_INCLUDED__
#define CGLTF_WRITE_H_INCLUDED__

#include "cgltf.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

cgltf_result cgltf_write_file(const cgltf_options* options, const char* path, const cgltf_data* out_data);

#ifdef __cplusplus
}
#endif

#endif /* #ifndef CGLTF_WRITE_H_INCLUDED__ */

/*
 *
 * Stop now, if you are only interested in the API.
 * Below, you find the implementation.
 *
 */

#ifdef __INTELLISENSE__
/* This makes MSVC intellisense work. */
#define CGLTF_WRITE_IMPLEMENTATION
#endif

#ifdef CGLTF_WRITE_IMPLEMENTATION

#include <stdio.h>

typedef struct {
	FILE* file;
	const cgltf_data* data;
	int depth;
	const char* indent;
	int needs_comma;
} cgltf_write_context;

static void cgltf_write_indent(cgltf_write_context* context)
{
	if (context->needs_comma)
	{
		fputs(",\n", context->file);
		context->needs_comma = 0;
	}
	else {
		fputc('\n', context->file);
	}
	for (int i = 0; i < context->depth; ++i)
	{
		fputs(context->indent, context->file);
	}
}

static void cgltf_write_line(cgltf_write_context* context, const char* line)
{
	if (line[0] == ']' || line[0] == '}')
	{
		--context->depth;
		context->needs_comma = 0;
	}
	cgltf_write_indent(context);
	fputs(line, context->file);
	int last = strlen(line) - 1;
	if (line[0] == ']' || line[0] == '}')
	{
		context->needs_comma = 1;
	}
	if (line[last] == '[' || line[last] == '{')
	{
		++context->depth;
		context->needs_comma = 0;
	}
}

static void cgltf_write_strprop(cgltf_write_context* context, const char* label, const char* val)
{
	if (val)
	{
		cgltf_write_indent(context);
		fprintf(context->file, "\"%s\": \"%s\"", label, val);
		context->needs_comma = 1;
	}
}

static void cgltf_write_intprop(cgltf_write_context* context, const char* label, int val, int def)
{
	if (val != def)
	{
		cgltf_write_indent(context);
		fprintf(context->file, "\"%s\": %d", label, val);
		context->needs_comma = 1;
	}
}

static void cgltf_write_floatprop(cgltf_write_context* context, const char* label, float val, float def)
{
	if (val != def)
	{
		cgltf_write_indent(context);
		fprintf(context->file, "\"%s\": %g", label, val);
		context->needs_comma = 1;
	}
}

static void cgltf_write_boolprop_optional(cgltf_write_context* context, const char* label, bool val, bool def)
{
	if (val != def)
	{
		cgltf_write_indent(context);
		fprintf(context->file, "\"%s\": %s", label, val ? "true" : "false");
		context->needs_comma = 1;
	}
}

static void cgltf_write_floatarrprop(cgltf_write_context* context, const char* label, const cgltf_float* vals, int dim)
{
	cgltf_write_indent(context);
	fprintf(context->file, "\"%s\": [", label);
	for (int i = 0; i < dim; ++i)
	{
		if (i != 0)
		{
			fprintf(context->file, ", %g", vals[i]);
		}
		else
		{
			fprintf(context->file, "%g", vals[i]);
		}
	}
	fputc(']', context->file);
	context->needs_comma = 1;
}

#define cgltf_write_idxprop(label, val, start) if (val) { \
		cgltf_write_indent(context); \
		fprintf(context->file, "\"%s\": %d", label, (int) (val - start)); \
		context->needs_comma = 1; }

#define cgltf_write_idxarrprop(label, dim, vals, start) if (vals) { \
		cgltf_write_indent(context); \
		fprintf(context->file, "\"%s\": [", label); \
		for (int i = 0; i < dim; ++i) { \
		if (i != 0) fprintf(context->file, ", "); \
		fprintf(context->file, "%d", (int) (vals[i] - start)); } \
		fputc(']', context->file); \
		context->needs_comma = 1; }

static int cgltf_int_from_component_type(cgltf_component_type ctype)
{
	switch (ctype)
	{
		case cgltf_component_type_invalid: return 0;
		case cgltf_component_type_r_8: return 5120;
		case cgltf_component_type_r_8u: return 5121;
		case cgltf_component_type_r_16: return 5122;
		case cgltf_component_type_r_16u: return 5124;
		case cgltf_component_type_r_32u: return 5125;
		case cgltf_component_type_r_32f: return 5126;
	}
}

static const char* cgltf_str_from_alpha_mode(cgltf_alpha_mode alpha_mode)
{
	switch (alpha_mode)
	{
		case cgltf_alpha_mode_opaque: return 0;
		case cgltf_alpha_mode_mask: return "MASK";
		case cgltf_alpha_mode_blend: return "BLEND";
	}
}

static const char* cgltf_str_from_type(cgltf_type type)
{
	switch (type)
	{
		case cgltf_type_invalid: return 0;
		case cgltf_type_scalar: return "SCALAR";
		case cgltf_type_vec2: return "VEC2";
		case cgltf_type_vec3: return "VEC3";
		case cgltf_type_vec4: return "VEC4";
		case cgltf_type_mat2: return "MAT2";
		case cgltf_type_mat3: return "MAT3";
		case cgltf_type_mat4: return "MAT4";
	}
}

static int cgltf_dim_from_type(cgltf_type type)
{
	switch (type)
	{
		case cgltf_type_invalid: return 0;
		case cgltf_type_scalar: return 1;
		case cgltf_type_vec2: return 2;
		case cgltf_type_vec3: return 3;
		case cgltf_type_vec4: return 4;
		case cgltf_type_mat2: return 4;
		case cgltf_type_mat3: return 9;
		case cgltf_type_mat4: return 16;
	}
}

static void cgltf_write_asset(cgltf_write_context* context, const cgltf_asset* asset)
{
	cgltf_write_line(context, "\"asset\": {");
	cgltf_write_strprop(context, "copyright", asset->copyright);
	cgltf_write_strprop(context, "generator", asset->generator);
	cgltf_write_strprop(context, "version", asset->version);
	cgltf_write_strprop(context, "min_version", asset->min_version);
	cgltf_write_line(context, "}");
}

static void cgltf_write_primitive(cgltf_write_context* context, const cgltf_primitive* prim)
{
	cgltf_write_intprop(context, "mode", (int) prim->type, 4);
	cgltf_write_idxprop("indices", prim->indices, context->data->accessors);
	cgltf_write_idxprop("material", prim->material, context->data->materials);
	cgltf_write_line(context, "\"attributes\": {");
	for (cgltf_size i = 0; i < prim->attributes_count; ++i)
	{
		const cgltf_attribute* attr = prim->attributes + i;
		cgltf_write_idxprop(attr->name, attr->data, context->data->accessors);
	}
	cgltf_write_line(context, "}");

	// TODO: prim->targets
}

static void cgltf_write_mesh(cgltf_write_context* context, const cgltf_mesh* mesh)
{
	cgltf_write_line(context, "{");
	cgltf_write_strprop(context, "name", mesh->name);
	
	cgltf_write_line(context, "\"primitives\": [");
	for (cgltf_size i = 0; i < mesh->primitives_count; ++i)
	{
		cgltf_write_line(context, "{");
		cgltf_write_primitive(context, mesh->primitives + i);
		cgltf_write_line(context, "}");
	}
	cgltf_write_line(context, "]");

	// TODO: mesh->weights

	cgltf_write_line(context, "}");
}

static void cgltf_write_buffer_view(cgltf_write_context* context, const cgltf_buffer_view* view)
{
	cgltf_write_line(context, "{");
	cgltf_write_idxprop("buffer", view->buffer, context->data->buffers);
	cgltf_write_intprop(context, "byteLength", view->size, -1);
	cgltf_write_intprop(context, "byteOffset", view->offset, 0);
	cgltf_write_intprop(context, "byteStride", view->stride, 0);
	// NOTE: We skip writing "target" because the spec says its usage can be inferred.
	// NOTE: The spec defines name / extensions / extras but cgltf does not read these.
	cgltf_write_line(context, "}");
}


static void cgltf_write_buffer(cgltf_write_context* context, const cgltf_buffer* buffer)
{
	cgltf_write_line(context, "{");
	cgltf_write_strprop(context, "uri", buffer->uri);
	cgltf_write_intprop(context, "byteLength", buffer->size, -1);
	// NOTE: The spec defines name / extensions / extras but cgltf does not read these.
	cgltf_write_line(context, "}");
}

static void cgltf_write_material(cgltf_write_context* context, const cgltf_material* material)
{
	cgltf_write_line(context, "{");
	cgltf_write_strprop(context, "name", material->name);
	cgltf_write_floatprop(context, "alphaCutoff", material->alpha_cutoff, 0.5f);
	cgltf_write_boolprop_optional(context, "doubleSided", material->double_sided, false);
	cgltf_write_boolprop_optional(context, "unlit", material->unlit, false);

	if (material->has_pbr_metallic_roughness)
	{
		// TODO: pbr_metallic_roughness
	}

	if (material->has_pbr_specular_glossiness)
	{
		// TODO: pbr_specular_glossiness;
	}

	// TODO: normal_texture, occlusion_texture, emissive_texture

	const cgltf_float* emissive = material->emissive_factor;
	if (emissive[0] > 0 || emissive[1] > 0 || emissive[2] > 0)
	{
		cgltf_write_floatarrprop(context, "emissiveFactor", emissive, 3);
	}

	cgltf_write_strprop(context, "alphaMode", cgltf_str_from_alpha_mode(material->alpha_mode));
	cgltf_write_line(context, "}");
}

static void cgltf_write_image(cgltf_write_context* context, const cgltf_image* image)
{
	cgltf_write_line(context, "{");
	// TODO: char* name;
	// TODO: char* uri; // usually just this
	// TODO: cgltf_buffer_view* buffer_view;
	// TODO: char* mime_type;
	cgltf_write_line(context, "}");
}

static void cgltf_write_texture(cgltf_write_context* context, const cgltf_texture* texture)
{
	cgltf_write_line(context, "{");
	// TODO: texture
	cgltf_write_line(context, "}");
}

static void cgltf_write_sampler(cgltf_write_context* context, const cgltf_sampler* sampler)
{
	cgltf_write_line(context, "{");
	// TODO: sampler
	cgltf_write_line(context, "}");
}

static void cgltf_write_node(cgltf_write_context* context, const cgltf_node* node)
{
	cgltf_write_line(context, "{");
	cgltf_write_idxarrprop("children", node->children_count, node->children, context->data->nodes);
	cgltf_write_idxprop("mesh", node->mesh, context->data->meshes);
	cgltf_write_strprop(context, "name", node->name);
	if (node->has_matrix)
	{
		cgltf_write_floatarrprop(context, "matrix", node->matrix, 16);
	}
	if (node->has_translation)
	{
		cgltf_write_floatarrprop(context, "translation", node->translation, 3);
	}
	if (node->has_rotation)
	{
		cgltf_write_floatarrprop(context, "rotation", node->rotation, 4);
	}
	if (node->has_scale)
	{
		cgltf_write_floatarrprop(context, "scale", node->scale, 3);
	}
	// TODO: skin, weights, light, camera
	cgltf_write_line(context, "}");
}

static void cgltf_write_scene(cgltf_write_context* context, const cgltf_scene* scene)
{
	cgltf_write_line(context, "{");
	cgltf_write_strprop(context, "name", scene->name);
	cgltf_write_idxarrprop("nodes", scene->nodes_count, scene->nodes, context->data->nodes);
	cgltf_write_line(context, "}");
}

static void cgltf_write_accessor(cgltf_write_context* context, const cgltf_accessor* accessor)
{
	cgltf_write_line(context, "{");
	cgltf_write_idxprop("bufferView", accessor->buffer_view, context->data->buffer_views);
	cgltf_write_intprop(context, "componentType", cgltf_int_from_component_type(accessor->component_type), 0);
	cgltf_write_strprop(context, "type", cgltf_str_from_type(accessor->type));
	int dim = cgltf_dim_from_type(accessor->type);
	cgltf_write_boolprop_optional(context, "normalized", accessor->normalized, false);
	cgltf_write_intprop(context, "byteOffset", accessor->offset, 0);
	cgltf_write_intprop(context, "count", accessor->count, -1);
	if (accessor->has_min)
	{
		cgltf_write_floatarrprop(context, "min", accessor->min, dim);
	}
	if (accessor->has_max)
	{
		cgltf_write_floatarrprop(context, "max", accessor->max, dim);        
	}
	cgltf_write_line(context, "}");
	// Note that stride is just an annotation and should not be written out.
	// TODO: accessor->sparse
}

cgltf_result cgltf_write_file(const cgltf_options* options, const char* path, const cgltf_data* data)
{
	cgltf_write_context context;
	context.file = fopen(path, "wt");	
	context.data = data;
	context.depth = 1;
	context.needs_comma = 0;
	context.indent = "  ";

	fputs("{", context.file);

	if (data->accessors_count > 0)
	{
		cgltf_write_line(&context, "\"accessors\": [");
		for (cgltf_size i = 0; i < data->accessors_count; ++i)
		{
			cgltf_write_accessor(&context, data->accessors + i);
		}
		cgltf_write_line(&context, "]");
	}

	cgltf_write_asset(&context, &data->asset);

	if (data->buffer_views_count > 0)
	{
		cgltf_write_line(&context, "\"bufferViews\": [");
		for (cgltf_size i = 0; i < data->buffer_views_count; ++i)
		{
			cgltf_write_buffer_view(&context, data->buffer_views + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->buffers_count > 0)
	{
		cgltf_write_line(&context, "\"buffers\": [");
		for (cgltf_size i = 0; i < data->buffers_count; ++i)
		{
			cgltf_write_buffer(&context, data->buffers + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->images_count > 0)
	{
		cgltf_write_line(&context, "\"images\": [");
		for (cgltf_size i = 0; i < data->images_count; ++i)
		{
			cgltf_write_image(&context, data->images + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->meshes_count > 0)
	{
		cgltf_write_line(&context, "\"meshes\": [");
		for (cgltf_size i = 0; i < data->meshes_count; ++i)
		{
			cgltf_write_mesh(&context, data->meshes + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->materials_count > 0)
	{
		cgltf_write_line(&context, "\"materials\": [");
		for (cgltf_size i = 0; i < data->materials_count; ++i)
		{
			cgltf_write_material(&context, data->materials + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->nodes_count > 0)
	{
		cgltf_write_line(&context, "\"nodes\": [");
		for (cgltf_size i = 0; i < data->nodes_count; ++i)
		{
			cgltf_write_node(&context, data->nodes + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->samplers_count > 0)
	{
		cgltf_write_line(&context, "\"samplers\": [");
		for (cgltf_size i = 0; i < data->samplers_count; ++i)
		{
			cgltf_write_sampler(&context, data->samplers + i);
		}
		cgltf_write_line(&context, "]");
	}

	cgltf_write_intprop(&context, "scene", data->scene - data->scenes, -1);

	if (data->scenes_count > 0)
	{
		cgltf_write_line(&context, "\"scenes\": [");
		for (cgltf_size i = 0; i < data->scenes_count; ++i)
		{
			cgltf_write_scene(&context, data->scenes + i);
		}
		cgltf_write_line(&context, "]");
	}

	if (data->textures_count > 0)
	{
		cgltf_write_line(&context, "\"textures\": [");
		for (cgltf_size i = 0; i < data->textures_count; ++i)
		{
			cgltf_write_texture(&context, data->textures + i);
		}
		cgltf_write_line(&context, "]");
	}

	// TODO: skins, animations, cameras, extensions

	fputs("\n}\n", context.file);
	fclose(context.file);

	return cgltf_result_success;
}

#endif /* #ifdef CGLTF_WRITE_IMPLEMENTATION */

/* cgltf is distributed under MIT license:
 *
 * Copyright (c) 2018 Johannes Kuhlmann

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
