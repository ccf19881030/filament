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

#ifndef GLTFIO_ASSETLOADER_H
#define GLTFIO_ASSETLOADER_H

namespace gltfio {

class AssetFlattener {
public:
    /**
     * Takes a pointer to the contents of a JSON-based glTF 2.0 file and returns a bundle
     * of Filament objects. Returns null on failure.
     */
    static AssetFlattener* createFromJson(const uint8_t* bytes, uint32_t nbytes);

    /**
     * Takes a pointer to the contents of a GLB glTF 2.0 file and returns a bundle
     * of Filament objects. Returns null on failure.
     */
    static AssetFlattener* createFromBinary(const uint8_t* bytes, uint32_t nbytes);

};



} // namespace gltfio

#endif // GLTFIO_ASSETLOADER_H
