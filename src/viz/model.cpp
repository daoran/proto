#include "play/model.hpp"

glmodel_t::glmodel_t(const std::string &path,
                     const std::string &vs,
                     const std::string &fs,
                     const bool gamma)
    : program{vs, fs}, gamma_correction(gamma) {
  glmodel_load(*this, path);
}

void glmodel_draw(glmodel_t &model, const glcamera_t &camera) {
  // Set projection and view
  model.program.use();
  model.program.setMat4("projection", glcamera_projection(camera));
  model.program.setMat4("view", glcamera_view_matrix(camera));
  model.program.setMat4("model", model.T_SM);

  for (unsigned int i = 0; i < model.meshes.size(); i++) {
    glmesh_draw(model.meshes[i], model.program);
  }
}

void glmodel_load(glmodel_t &model, const std::string &path) {
  // Read
  Assimp::Importer importer;
  const auto options = aiProcess_Triangulate | aiProcess_FlipUVs | aiProcess_CalcTangentSpace;
  const aiScene *scene = importer.ReadFile(path, options);

  // Check for errors
  if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE ||
      !scene->mRootNode) {
    std::cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << std::endl;
    return;
  }

  // Retrieve the directory path of the filepath
  model.directory = path.substr(0, path.find_last_of('/'));

  // Process ASSIMP's root node recursively
  glmodel_process_node(model, scene->mRootNode, scene);
}

void glmodel_process_node(glmodel_t &model, aiNode *node, const aiScene *scene) {
  // Process each mesh located at the current node
  for (unsigned int i = 0; i < node->mNumMeshes; i++) {
    // The node object only contains indices to index the actual objects in the
    // scene.  the scene contains all the data, node is just to keep stuff
    // organized (like relations between nodes).
    aiMesh *mesh = scene->mMeshes[node->mMeshes[i]];
    model.meshes.push_back(glmodel_process_mesh(model, mesh, scene));
  }

  // After we've processed all of the meshes (if any) we then recursively
  // process each of the children nodes
  for (unsigned int i = 0; i < node->mNumChildren; i++) {
    glmodel_process_node(model, node->mChildren[i], scene);
  }
}

glmesh_t glmodel_process_mesh(glmodel_t &model, aiMesh *mesh, const aiScene *scene) {
  // Data to fill
  std::vector<glvertex_t> vertices;
  std::vector<unsigned int> indices;
  std::vector<gltexture_t> textures;

  // Walk through each of the mesh's vertices
  for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
    glvertex_t vertex;
    glm::vec3 vector;
    // Placeholder vector since assimp uses its own vector class that doesn't
    // directly convert to glm's vec3 class so we transfer the data to this
    // placeholder glm::vec3 first.

    // Positions
    vector.x = mesh->mVertices[i].x;
    vector.y = mesh->mVertices[i].y;
    vector.z = mesh->mVertices[i].z;
    vertex.position = vector;

    // Normals
    vector.x = mesh->mNormals[i].x;
    vector.y = mesh->mNormals[i].y;
    vector.z = mesh->mNormals[i].z;
    vertex.normal = vector;

    // Texture coordinates
    if (mesh->mTextureCoords[0]) {
      // Does the mesh contain texture coordinates? A vertex can contain up to
      // 8 different texture coordinates. We thus make the assumption that we
      // won't use models where a vertex can have multiple texture coordinates
      // so we always take the first set (0).
      glm::vec2 vec;
      vec.x = mesh->mTextureCoords[0][i].x;
      vec.y = mesh->mTextureCoords[0][i].y;
      vertex.texcoords = vec;
    } else {
      vertex.texcoords = glm::vec2(0.0f, 0.0f);
    }

    // Tangent
    vector.x = mesh->mTangents[i].x;
    vector.y = mesh->mTangents[i].y;
    vector.z = mesh->mTangents[i].z;
    vertex.tangent = vector;

    // Bi-tangent
    vector.x = mesh->mBitangents[i].x;
    vector.y = mesh->mBitangents[i].y;
    vector.z = mesh->mBitangents[i].z;
    vertex.bitangent = vector;
    vertices.push_back(vertex);
  }

  // Now walk through each of the mesh's faces (a face is a mesh its triangle)
  // and retrieve the corresponding vertex indices.
  for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
    aiFace face = mesh->mFaces[i];

    // Retrieve all indices of the face and store them in the indices vector
    for (unsigned int j = 0; j < face.mNumIndices; j++)
      indices.push_back(face.mIndices[j]);
  }

  // Process materials
  aiMaterial *material = scene->mMaterials[mesh->mMaterialIndex];
  // we assume a convention for sampler names in the shaders. Each diffuse
  // texture should be named as 'texture_diffuseN' where N is a sequential
  // number ranging from 1 to MAX_SAMPLER_NUMBER.  Same applies to other
  // texture as the following list summarizes:
  // - diffuse: texture_diffuseN
  // - specular: texture_specularN
  // - normal: texture_normalN

  // clang-format off
  // 1. Diffuse maps
  const auto diffuse_maps = glmodel_load_textures(
    model,
    material,
    aiTextureType_DIFFUSE,
    "texture_diffuse"
  );
  textures.insert(textures.end(), diffuse_maps.begin(), diffuse_maps.end());

  // 2. Specular maps
  const auto specularMaps = glmodel_load_textures(
    model,
    material,
    aiTextureType_SPECULAR,
    "texture_specular"
  );
  textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());

  // 3. Normal maps
  const auto normalMaps = glmodel_load_textures(
    model,
    material,
    aiTextureType_HEIGHT,
    "texture_normal"
  );
  textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());

  // 4. Height maps
  const auto heightMaps = glmodel_load_textures(
    model,
    material,
    aiTextureType_AMBIENT,
    "texture_height"
  );
  textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
  // clang-format on

  // Return a mesh object created from the extracted mesh data
  return glmesh_t{vertices, indices, textures};
}

// checks all material textures of a given type and loads the textures if
// they're not loaded yet. the required info is returned as a Texture struct.
std::vector<gltexture_t> glmodel_load_textures(glmodel_t &model,
                                               aiMaterial *mat,
                                               aiTextureType type,
                                               std::string typeName) {
  std::vector<gltexture_t> textures;

  for (unsigned int i = 0; i < mat->GetTextureCount(type); i++) {
    aiString str;
    mat->GetTexture(type, i, &str);

    // check if texture was loaded before and if so, continue to next
    // iteration: skip loading a new texture
    bool skip = false;
    for (unsigned int j = 0; j < model.textures_loaded.size(); j++) {
      if (std::strcmp(model.textures_loaded[j].path.data(), str.C_Str()) == 0) {
        textures.push_back(model.textures_loaded[j]);
        skip = true;
        // a texture with the same filepath has already been
        // loaded, continue to next one. (optimization)
        break;
      }
    }

    if (!skip) { // if texture hasn't been loaded already, load it
      gltexture_t texture;
      // const std::string texture_path = model.directory + '/' + str.C_Str();
      texture.id = texture_from_file(model.directory, str.C_Str());
      // texture.id = load_texture(texture_path);
      texture.type = typeName;
      texture.path = str.C_Str();
      textures.push_back(texture);
      model.textures_loaded.push_back(texture);
      // store it as texture loaded for entire model, to ensure we won't
      // unnecesery load duplicate textures.
    }
  }

  return textures;
}

unsigned int texture_from_file(const std::string &dir,
                               const char *fp,
                               bool gamma) {
  const std::string filename = dir + '/' + std::string(fp);
  unsigned int texture_id;
  glGenTextures(1, &texture_id);

  int width, height, nb_components;
  unsigned char *data =
      stbi_load(filename.c_str(), &width, &height, &nb_components, 0);

  if (data) {
    GLenum format;
    switch (nb_components) {
      case 1: format = GL_RED; break;
      case 3: format = GL_RGB; break;
      case 4: format = GL_RGBA; break;
    }

    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 format,
                 width,
                 height,
                 0,
                 format,
                 GL_UNSIGNED_BYTE,
                 data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D,
                    GL_TEXTURE_MIN_FILTER,
                    GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  } else {
    std::cout << "Texture failed to load at path: " << fp << std::endl;
  }

  stbi_image_free(data);
  return texture_id;
}
